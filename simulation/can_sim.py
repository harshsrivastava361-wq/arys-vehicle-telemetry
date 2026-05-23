"""
============================================================
 Arys Garage — Q2: Real-Time Vehicle Telemetry System
 File   : can_sim.py
 About  : Software CAN bus simulation.
          Encodes sensor data into CAN frames and logs them.

 CAN Message IDs:
   0x100  TELEMETRY_GPS    lat, lon, speed, heading
   0x101  TELEMETRY_IMU    ax, ay, az (packed int16)
   0x102  TELEMETRY_WHEEL  4× wheel RPM (uint16 each × 2 fit)
   0x110  BMS_STATUS       SOC, voltage, temperature (stub)
   0x120  VCU_COMMAND      throttle → torque request (stub)
   0x1FF  FAULT_FLAGS      bitmask of active faults

 CAN Frame format (CAN 2.0A, 11-bit ID):
   [ ID 11b ][ RTR ][ DLC 4b ][ DATA 0–8 B ][ CRC 15b ]

 How to run:
   python3 can_sim.py          # standalone CAN log demo

 AI Usage: Frame packing logic and CAN ID table generated
           with Claude (Anthropic). Byte packing adapted from
           CAN 2.0 specification Part A.
============================================================
"""

import struct
import time
import threading
import queue

import sensor_sim as sim

# ── CAN Message IDs ──────────────────────────────────────────
CAN_ID_GPS    = 0x100
CAN_ID_IMU    = 0x101
CAN_ID_WHEEL  = 0x102
CAN_ID_BMS    = 0x110
CAN_ID_VCU    = 0x120
CAN_ID_FAULT  = 0x1FF

# ── Software CAN bus (shared queue, mirrors CAN FIFO) ────────
can_bus_fifo = queue.Queue(maxsize=128)


class CANFrame:
    """
    Represents one CAN 2.0A frame.
    On real hardware this is esp_can_message_t (ESP32 ESP-IDF).
    """
    __slots__ = ('can_id', 'data', 'dlc', 'timestamp')

    def __init__(self, can_id: int, data: bytes, ts: float = None):
        self.can_id    = can_id
        self.data      = data[:8]
        self.dlc       = len(self.data)
        self.timestamp = ts or time.time()

    def __str__(self):
        raw = ' '.join(f'{b:02X}' for b in self.data)
        return (f"[{self.timestamp:9.3f}s]  "
                f"ID=0x{self.can_id:03X}  DLC={self.dlc}  DATA= {raw}")

    def to_csv(self) -> str:
        raw = ''.join(f'{b:02X}' for b in self.data)
        return f"{self.timestamp:.4f},CAN,0x{self.can_id:03X},{self.dlc},{raw}"


# ── Frame encoders ───────────────────────────────────────────

def encode_gps(lat: float, lon: float, speed: float, hdg: float) -> CANFrame:
    """
    Packs GPS data into 8 bytes.
    lat/lon scaled ×10000 → int16   (range ±3.2768° → OK for ±delta from centre)
    speed ×10  → uint16             (0..6553.5 km/h)
    heading ×10 → uint16            (0..3600 = 0..360.0°)
    """
    dlat = int((lat  - 12.9716) * 10000) & 0xFFFF
    dlon = int((lon  - 77.5946) * 10000) & 0xFFFF
    spd  = int(speed * 10)               & 0xFFFF
    hdg  = int(hdg   * 10)               & 0xFFFF
    return CANFrame(CAN_ID_GPS, struct.pack('>HHHH', dlat, dlon, spd, hdg))


def encode_imu(ax: float, ay: float, az: float) -> CANFrame:
    """
    Packs 3-axis accelerometer into 6 bytes.
    Each axis: int16  ×100   (range ±327.67 m/s²)
    """
    def clamp16(v): return max(-32768, min(32767, int(v * 100)))
    return CANFrame(CAN_ID_IMU, struct.pack('>hhh', clamp16(ax), clamp16(ay), clamp16(az)))


def encode_wheel(fl, fr, rl, rr) -> CANFrame:
    """
    Packs 4 wheel RPM values into 8 bytes.
    Each RPM: uint16  ×1  (max 65535 RPM — more than enough)
    """
    def clamp_u16(v): return max(0, min(65535, int(v)))
    return CANFrame(CAN_ID_WHEEL,
                    struct.pack('>HHHH', clamp_u16(fl), clamp_u16(fr),
                                         clamp_u16(rl), clamp_u16(rr)))


def encode_fault(faults) -> CANFrame:
    """
    Packs all fault flags into 1 byte bitmask.
    Bit 0 = GPS loss   Bit 1 = IMU disconnect
    Bit 2 = CAN timeout  Bit 3 = SD write fail
    """
    b = 0
    if faults.gps_loss:       b |= (1 << 0)
    if faults.imu_disconnect: b |= (1 << 1)
    if faults.can_timeout:    b |= (1 << 2)
    if faults.sd_write_fail:  b |= (1 << 3)
    return CANFrame(CAN_ID_FAULT, bytes([b]))


# ── CAN TX Task ──────────────────────────────────────────────
def can_tx_task(log_path: str = 'data/can_log.txt'):
    """
    Reads sensor state and broadcasts CAN frames.

    TX rates:
      GPS frame   : 1  Hz
      IMU frame   : 10 Hz  (reduced from 100 Hz for CAN bandwidth)
      Wheel frame : 20 Hz
      Fault frame : 5  Hz  (safety critical — frequent)

    On real hardware: HAL_CAN_AddTxMessage() / twai_transmit()
    """
    print('[CAN_TX] Started — transmitting on virtual CAN bus')
    import os; os.makedirs('data', exist_ok=True)

    t_gps = t_imu = t_whl = t_flt = 0.0

    with open(log_path, 'w') as f:
        f.write('# Arys Garage — CAN Bus Log\n')
        f.write('# timestamp_s,type,id,dlc,hex_data\n')

        while sim._state['running']:
            now = time.time()

            with sim._lock:
                st = dict(sim._state)
            faults = st['faults']

            def tx(frame: CANFrame):
                try:
                    can_bus_fifo.put_nowait(frame)
                    f.write(frame.to_csv() + '\n')
                    f.flush()
                except queue.Full:
                    pass   # Drop if bus saturated

            if now - t_gps >= 1.0:
                tx(encode_gps(st['latitude'], st['longitude'],
                              st['speed_kmh'], st['heading']))
                t_gps = now

            if now - t_imu >= 0.1:
                tx(encode_imu(st['gforce_x'] * 9.81,
                              st['gforce_y'] * 9.81,
                              st['gforce_z'] * 9.81))
                t_imu = now

            if now - t_whl >= 0.05:
                try:
                    w = sim.wheel_queue.get_nowait()
                    tx(encode_wheel(w.rpm_fl, w.rpm_fr, w.rpm_rl, w.rpm_rr))
                except Exception:
                    pass
                t_whl = now

            if now - t_flt >= 0.2:
                frame = encode_fault(faults)
                tx(frame)
                t_flt = now

            time.sleep(0.01)


# ── CAN RX Task ──────────────────────────────────────────────
def can_rx_task():
    """
    Receives and decodes CAN frames.
    Implements CAN bus timeout watchdog (500 ms window).

    On real hardware:
      - ISR: HAL_CAN_RxFifo0MsgPendingCallback()
      - Watchdog: xTimerReset(can_wd_timer, 0) on every received frame
    """
    print('[CAN_RX] Started — monitoring CAN bus')
    last_rx = time.time()

    while sim._state['running']:
        try:
            frame = can_bus_fifo.get(timeout=0.5)
            last_rx = time.time()

            if frame.can_id == CAN_ID_FAULT:
                flags = frame.data[0]
                if flags:
                    active = []
                    if flags & 1: active.append('GPS_LOSS')
                    if flags & 2: active.append('IMU_DISCONNECT')
                    if flags & 4: active.append('CAN_TIMEOUT')
                    if flags & 8: active.append('SD_WRITE_FAIL')
                    print(f'  [CAN_RX] FAULT frame: {" | ".join(active)}')

        except queue.Empty:
            # 500 ms with no frame → CAN bus timeout
            elapsed = time.time() - last_rx
            if elapsed > 0.5:
                print(f'  [CAN_RX] WARNING: CAN bus silent for {elapsed:.1f}s')
                last_rx = time.time()   # reset so we don't spam


# ── Standalone demo ──────────────────────────────────────────
if __name__ == '__main__':
    print('=' * 55)
    print('  Arys Garage — CAN Bus Simulator')
    print('=' * 55)

    sim.start_all_tasks()

    tx_t = threading.Thread(target=can_tx_task, daemon=True)
    rx_t = threading.Thread(target=can_rx_task, daemon=True)
    tx_t.start(); rx_t.start()

    time.sleep(6)
    print('\nInjecting CAN timeout (simulating bus-off)...')
    sim.inject_fault('can_timeout', 3.0)
    time.sleep(6)

    sim._state['running'] = False
    print('\nDone. Check data/can_log.txt')
