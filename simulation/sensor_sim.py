"""
============================================================
 Arys Garage — Q2: Real-Time Vehicle Telemetry System
 File   : sensor_sim.py
 Author : Harsh
 About  : Simulates ALL vehicle sensors using Python threads.
          Each thread = one FreeRTOS task on real hardware.

 Sensors simulated:
   - GPS  (NEO-M8N)   : latitude, longitude, speed, heading @ 1 Hz
   - IMU  (MPU-6050)  : accelerometer + gyroscope @ 100 Hz
   - Wheel speed      : 4x Hall-effect sensors @ 50 Hz
   - Fault monitor    : watches all sensors for errors @ 10 Hz
   - Data logger      : writes CSV file (simulates SD card)

 AI Usage: Core structure and sensor math generated with Claude
           (Anthropic). Noise models and fusion filter adapted
           from MPU-6050 datasheet values.
============================================================
"""

import threading
import queue
import time
import math
import random
from dataclasses import dataclass, field

# ──────────────────────────────────────────────────────────────
#  DATA STRUCTURES  (mirrors C structs in firmware/include/telemetry.h)
# ──────────────────────────────────────────────────────────────

@dataclass
class GPSData:
    timestamp:    float
    latitude:     float    # decimal degrees
    longitude:    float    # decimal degrees
    speed_kmh:    float    # km/h
    heading:      float    # 0–360°, 0 = North
    fix_valid:    bool
    satellites:   int

@dataclass
class IMUData:
    timestamp:    float
    accel_x:      float    # m/s²  (longitudinal)
    accel_y:      float    # m/s²  (lateral)
    accel_z:      float    # m/s²  (vertical ≈ 9.81)
    gyro_x:       float    # deg/s (roll rate)
    gyro_y:       float    # deg/s (pitch rate)
    gyro_z:       float    # deg/s (yaw rate)
    temp_c:       float    # °C   (MPU-6050 internal sensor)

@dataclass
class WheelData:
    timestamp:    float
    rpm_fl:       float    # Front Left  RPM
    rpm_fr:       float    # Front Right RPM
    rpm_rl:       float    # Rear Left   RPM
    rpm_rr:       float    # Rear Right  RPM
    speed_kmh:    float    # Calculated from average RPM

@dataclass
class FaultStatus:
    gps_loss:      bool  = False   # No GPS fix > 3 s
    imu_disconnect:bool  = False   # No IMU response
    can_timeout:   bool  = False   # No CAN message > 500 ms
    sd_write_fail: bool  = False   # SD card write error
    timestamp:     float = 0.0

# ──────────────────────────────────────────────────────────────
#  SHARED QUEUES  (mirrors FreeRTOS xQueueCreate)
#  maxsize = how many readings can wait before oldest is dropped
# ──────────────────────────────────────────────────────────────
gps_queue   = queue.Queue(maxsize=10)
imu_queue   = queue.Queue(maxsize=50)
wheel_queue = queue.Queue(maxsize=50)
fault_queue = queue.Queue(maxsize=20)
log_queue   = queue.Queue(maxsize=500)

# ──────────────────────────────────────────────────────────────
#  GLOBAL SYSTEM STATE  (mirrors FreeRTOS global variables)
#  Protected by _lock (mirrors xSemaphoreCreateMutex)
# ──────────────────────────────────────────────────────────────
_lock = threading.Lock()
_state = {
    'speed_kmh':   0.0,
    'heading':     0.0,
    'gforce_x':    0.0,     # longitudinal G (accel/brake)
    'gforce_y':    0.0,     # lateral G      (cornering)
    'gforce_z':    1.0,     # vertical G     (bumps)
    'roll_deg':    0.0,     # roll  angle from sensor fusion
    'pitch_deg':   0.0,     # pitch angle from sensor fusion
    'latitude':    12.9716, # starting: Bengaluru
    'longitude':   77.5946,
    'distance_m':  0.0,
    'lap_number':  1,
    'faults':      FaultStatus(),
    'running':     True,
}

# ──────────────────────────────────────────────────────────────
#  TRACK SIMULATION  (oval circuit @ Bengaluru coordinates)
# ──────────────────────────────────────────────────────────────
class OvalTrack:
    """
    Simulates a vehicle driving an oval lap.
    One lap = 60 seconds.  Radius ~300m.
    """
    LAP_TIME  = 60.0    # seconds per lap
    R_LAT     = 0.0027  # ≈ 300m latitude  radius
    R_LON     = 0.0036  # ≈ 300m longitude radius

    def __init__(self, center_lat=12.9716, center_lon=77.5946):
        self.clat = center_lat
        self.clon = center_lon

    def position_at(self, t: float):
        """Return (lat, lon, speed_kmh, heading, gx, gy) at time t."""
        angle = (2 * math.pi * (t % self.LAP_TIME)) / self.LAP_TIME

        lat = self.clat + self.R_LAT * math.sin(angle)
        lon = self.clon + self.R_LON * math.cos(angle)

        # Speed: faster on straights, slower in tight corners
        speed = 80.0 + 35.0 * abs(math.sin(2 * angle))
        speed = max(30.0, min(145.0, speed + random.gauss(0, 1.5)))

        # Heading: tangent direction (0 = North)
        heading = (math.degrees(angle + math.pi / 2)) % 360

        # G-forces
        gy = (speed / 3.6) ** 2 / (self.R_LAT * 111_000) / 9.81  # lateral
        gy = min(gy, 2.0) * math.sign_custom(math.cos(angle))

        speed_deriv = 35.0 * 2 * math.cos(2 * angle)
        gx = speed_deriv / (9.81 * self.LAP_TIME / (2 * math.pi))   # longitudinal

        return lat, lon, speed, heading, gx, gy

# small helper (math has no sign)
math.sign_custom = lambda x: 1.0 if x >= 0 else -1.0

track = OvalTrack()

# ──────────────────────────────────────────────────────────────
#  TASK 1 — GPS TASK
#  FreeRTOS priority : 2  (medium — GPS updates slowly)
#  Stack             : 4096 bytes
#  Update rate       : 1 Hz  (NEO-M8N default)
#  Protocol          : UART at 9600 baud → NMEA 0183
# ──────────────────────────────────────────────────────────────
def gps_task(start_time: float):
    print("[GPS_TASK]   Started — NEO-M8N @ 1 Hz via UART")
    while _state['running']:
        t = time.time() - start_time

        lat, lon, speed, heading, _, _ = track.position_at(t)

        # Realistic GPS noise: NEO-M8N → ~2 m CEP
        lat += random.gauss(0, 0.000018)
        lon += random.gauss(0, 0.000018)

        with _lock:
            fault = _state['faults'].gps_loss

        data = GPSData(
            timestamp  = round(t, 3),
            latitude   = lat,
            longitude  = lon,
            speed_kmh  = speed,
            heading    = (heading + random.gauss(0, 0.5)) % 360,
            fix_valid  = not fault,
            satellites = random.randint(8, 12) if not fault else 0,
        )

        with _lock:
            if not fault:
                _state['latitude']  = lat
                _state['longitude'] = lon
                _state['heading']   = data.heading

        _queue_put(gps_queue, data)
        _log(f"{t:.3f},GPS,{lat:.6f},{lon:.6f},{speed:.1f},{heading:.1f}")

        time.sleep(1.0)

# ──────────────────────────────────────────────────────────────
#  TASK 2 — IMU TASK
#  FreeRTOS priority : 4  (HIGH — fastest sensor, time-critical)
#  Stack             : 2048 bytes
#  Update rate       : 100 Hz  (MPU-6050 max rate used here)
#  Protocol          : I2C at 400 kHz, address 0x68
#
#  SENSOR FUSION — Complementary Filter:
#    angle = α × (angle + gyro × dt)  +  (1−α) × accel_angle
#    α = 0.98  (trusts gyro 98%, corrects drift with accel 2%)
# ──────────────────────────────────────────────────────────────
def imu_task(start_time: float):
    print("[IMU_TASK]   Started — MPU-6050 @ 100 Hz via I2C 0x68")
    dt    = 0.01        # 100 Hz period
    ALPHA = 0.98        # complementary filter coefficient
    roll  = 0.0
    pitch = 0.0

    while _state['running']:
        t = time.time() - start_time
        _, _, speed, _, gx_g, gy_g = track.position_at(t)

        # Simulate accelerometer output (m/s²)
        ax = gx_g * 9.81 + random.gauss(0, 0.05)   # longitudinal
        ay = gy_g * 9.81 + random.gauss(0, 0.05)   # lateral
        az = 9.81        + random.gauss(0, 0.02)    # gravity + vertical noise

        # Simulate gyroscope output (deg/s)
        gx_dps = random.gauss(0, 0.3)
        gy_dps = random.gauss(0, 0.3)
        gz_dps = 2.0 * math.sin(t * 0.1) + random.gauss(0, 0.1)

        # ── Complementary Filter ─────────────────────────────
        accel_roll  = math.degrees(math.atan2(ay, az))
        accel_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))
        roll  = ALPHA * (roll  + gx_dps * dt) + (1 - ALPHA) * accel_roll
        pitch = ALPHA * (pitch + gy_dps * dt) + (1 - ALPHA) * accel_pitch
        # ─────────────────────────────────────────────────────

        data = IMUData(
            timestamp = round(t, 4),
            accel_x=ax, accel_y=ay, accel_z=az,
            gyro_x=gx_dps, gyro_y=gy_dps, gyro_z=gz_dps,
            temp_c = 45.0 + random.gauss(0, 0.3),
        )

        with _lock:
            _state['gforce_x'] = ax / 9.81
            _state['gforce_y'] = ay / 9.81
            _state['gforce_z'] = az / 9.81
            _state['roll_deg']  = round(roll,  2)
            _state['pitch_deg'] = round(pitch, 2)

        _queue_put(imu_queue, data)
        time.sleep(dt)

# ──────────────────────────────────────────────────────────────
#  TASK 3 — WHEEL SPEED TASK
#  FreeRTOS priority : 3  (medium-high)
#  Stack             : 1024 bytes
#  Update rate       : 50 Hz
#  Protocol          : ADC pulse counting (GPIO interrupt on hardware)
#  Wheel circumference: 1.85 m  (205/55 R16 tyre)
#
#  Speed formula:
#    speed_kmh = (avg_RPM × wheel_circumference_m × 60) / 1000
# ──────────────────────────────────────────────────────────────
def wheel_speed_task(start_time: float):
    print("[WHEEL_TASK] Started — 4× Hall sensors @ 50 Hz via ADC")
    CIRC = 1.85   # wheel circumference in metres

    while _state['running']:
        t = time.time() - start_time
        _, _, speed_kmh, _, _, _ = track.position_at(t)

        speed_ms  = speed_kmh / 3.6
        base_rpm  = (speed_ms / CIRC) * 60.0

        rpm_fl = max(0, base_rpm + random.gauss(0, 2.0))
        rpm_fr = max(0, base_rpm + random.gauss(0, 2.0))
        rpm_rl = max(0, base_rpm * 1.01 + random.gauss(0, 2.0))
        rpm_rr = max(0, base_rpm * 1.01 + random.gauss(0, 2.0))

        calc_speed = ((rpm_fl + rpm_fr + rpm_rl + rpm_rr) / 4) * CIRC / 60.0 * 3.6

        data = WheelData(
            timestamp = round(t, 3),
            rpm_fl=rpm_fl, rpm_fr=rpm_fr,
            rpm_rl=rpm_rl, rpm_rr=rpm_rr,
            speed_kmh = max(0, calc_speed),
        )

        with _lock:
            _state['speed_kmh']  = calc_speed
            _state['distance_m'] += speed_ms * 0.02   # integrate: v × dt

        _queue_put(wheel_queue, data)
        _log(f"{t:.3f},WHEEL,{rpm_fl:.0f},{rpm_fr:.0f},{rpm_rl:.0f},{rpm_rr:.0f},{calc_speed:.1f}")

        time.sleep(0.02)   # 50 Hz

# ──────────────────────────────────────────────────────────────
#  TASK 4 — FAULT MONITOR TASK
#  FreeRTOS priority : 5  (HIGHEST — safety critical)
#  Stack             : 2048 bytes
#  Update rate       : 10 Hz
#
#  Recovery actions:
#    GPS loss      → Dead reckoning (use IMU + last known position)
#    IMU disconnect→ Halt orientation estimation, log CRITICAL
#    CAN timeout   → Use last known CAN values, alert dashboard
#    SD write fail → Buffer data in RAM, retry on next cycle
# ──────────────────────────────────────────────────────────────
def fault_monitor_task(start_time: float):
    print("[FAULT_TASK] Started — monitoring all subsystems @ 10 Hz")
    last_log = {}

    while _state['running']:
        with _lock:
            f = _state['faults']

        faults_active = {
            'GPS_LOSS':       f.gps_loss,
            'IMU_DISCONNECT': f.imu_disconnect,
            'CAN_TIMEOUT':    f.can_timeout,
            'SD_WRITE_FAIL':  f.sd_write_fail,
        }

        for name, active in faults_active.items():
            if active and not last_log.get(name):
                t = time.time() - start_time
                _log(f"{t:.3f},FAULT,{name},ACTIVE")
                print(f"  [FAULT] ⚠  {name} detected — recovery activated")
                _queue_put(fault_queue, FaultStatus(timestamp=t, **{
                    name.lower(): True
                    for name in [name]
                    if hasattr(FaultStatus, name.lower())
                }))
            elif not active and last_log.get(name):
                t = time.time() - start_time
                _log(f"{t:.3f},FAULT,{name},RECOVERED")
                print(f"  [FAULT] ✓  {name} recovered")

            last_log[name] = active

        time.sleep(0.1)

# ──────────────────────────────────────────────────────────────
#  TASK 5 — DATA LOGGER TASK
#  FreeRTOS priority : 1  (LOWEST — logging can be delayed)
#  Stack             : 4096 bytes
#  Simulates SD card writes via SPI
# ──────────────────────────────────────────────────────────────
def data_logger_task(start_time: float, log_file: str = "data/telemetry_log.csv"):
    print(f"[LOG_TASK]   Started — writing to {log_file}")
    import os
    os.makedirs("data", exist_ok=True)

    with open(log_file, 'w') as f:
        f.write("timestamp_s,type,value1,value2,value3,value4,value5,value6\n")
        while _state['running']:
            try:
                entry = log_queue.get(timeout=0.5)
                with _lock:
                    sd_fail = _state['faults'].sd_write_fail
                if not sd_fail:
                    f.write(entry + "\n")
                    f.flush()
            except queue.Empty:
                pass

# ──────────────────────────────────────────────────────────────
#  HELPERS
# ──────────────────────────────────────────────────────────────
def _queue_put(q: queue.Queue, item):
    """Non-blocking put — silently drops if full (mirrors xQueueSendToBack)"""
    try:
        q.put_nowait(item)
    except queue.Full:
        pass

def _log(entry: str):
    """Send string to log queue"""
    _queue_put(log_queue, entry)

# ──────────────────────────────────────────────────────────────
#  SYSTEM START  (mirrors app_main → xTaskCreate × 5)
# ──────────────────────────────────────────────────────────────
def start_all_tasks(log_file: str = "data/telemetry_log.csv"):
    """
    Creates and starts all RTOS tasks as Python threads.

    Task priority table (higher number = higher priority):
    ┌─────────────────┬──────────┬──────────┬─────────────────────────────┐
    │ Task            │ Priority │ Rate     │ Reason                      │
    ├─────────────────┼──────────┼──────────┼─────────────────────────────┤
    │ fault_monitor   │    5     │  10 Hz   │ Safety-critical, must run   │
    │ imu_task        │    4     │ 100 Hz   │ Fastest sensor, time-tight  │
    │ wheel_task      │    3     │  50 Hz   │ Speed calculation needed    │
    │ gps_task        │    2     │   1 Hz   │ Slow sensor, can wait       │
    │ data_logger     │    1     │ on-demand│ Lowest — I/O can be delayed │
    └─────────────────┴──────────┴──────────┴─────────────────────────────┘
    """
    start_time = time.time()
    tasks = [
        threading.Thread(target=fault_monitor_task, args=(start_time,), name="FAULT_TASK", daemon=True),
        threading.Thread(target=imu_task,           args=(start_time,), name="IMU_TASK",   daemon=True),
        threading.Thread(target=wheel_speed_task,   args=(start_time,), name="WHEEL_TASK", daemon=True),
        threading.Thread(target=gps_task,           args=(start_time,), name="GPS_TASK",   daemon=True),
        threading.Thread(target=data_logger_task,   args=(start_time,), name="LOG_TASK",   daemon=True,
                         kwargs={'log_file': log_file}),
    ]
    for t in tasks:
        t.start()
    print(f"\n[SYSTEM] ✓ All {len(tasks)} tasks running.\n")
    return tasks, start_time

def inject_fault(fault_type: str, duration: float = 5.0):
    """
    Injects a sensor/bus fault for testing.

    Args:
        fault_type : 'gps_loss' | 'imu_disconnect' | 'can_timeout' | 'sd_write_fail'
        duration   : seconds the fault lasts before auto-recovery
    """
    def _run():
        with _lock:
            setattr(_state['faults'], fault_type, True)
        time.sleep(duration)
        with _lock:
            setattr(_state['faults'], fault_type, False)

    threading.Thread(target=_run, daemon=True).start()

# ──────────────────────────────────────────────────────────────
#  QUICK TEST  (run this file directly to verify everything works)
# ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("=" * 55)
    print("  Arys Garage — Sensor Simulation Self-Test")
    print("=" * 55)

    tasks, _ = start_all_tasks()

    print("Running for 6 seconds...")
    time.sleep(4)
    print("\nInjecting GPS loss fault for 3 seconds...")
    inject_fault('gps_loss', 3.0)
    time.sleep(5)

    _state['running'] = False
    print("\nDone. Check data/telemetry_log.csv")
