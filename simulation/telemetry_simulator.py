"""
Arys Garage — Vehicle Telemetry Simulator
Simulates all 6 RTOS tasks in Python and displays a live dashboard
Shows: Speed, G-Force, GPS Track, Roll/Pitch, RPM, Fault Events
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import csv
import time
import random
import os
from datetime import datetime
from collections import deque

# ── Simulation Parameters ──────────────────────────────────────
SIM_DURATION_S   = 120   # 2 minute lap
SAMPLE_RATE_HZ   = 100   # matches FreeRTOS sensor task
DT               = 1.0 / SAMPLE_RATE_HZ
WHEEL_DIAMETER_M = 0.56
MAX_RPM          = 8000
ALPHA            = 0.98  # complementary filter coefficient

# ── Fault Injection Schedule ───────────────────────────────────
FAULT_SCHEDULE = {
    20: "GPS_LOSS",
    25: "GPS_RESTORE",
    50: "CAN_TIMEOUT",
    55: "CAN_RESTORE",
    80: "SD_FAIL",
    85: "SD_RESTORE",
    100: "IMU_DISCONNECT",
    105: "IMU_RESTORE",
}

# ── Data Buffers (deque = fast append/pop) ─────────────────────
BUF = 500
t_buf     = deque([0]*BUF, maxlen=BUF)
spd_buf   = deque([0]*BUF, maxlen=BUF)
rpm_buf   = deque([0]*BUF, maxlen=BUF)
gx_buf    = deque([0]*BUF, maxlen=BUF)
gy_buf    = deque([0]*BUF, maxlen=BUF)
gz_buf    = deque([0]*BUF, maxlen=BUF)
roll_buf  = deque([0]*BUF, maxlen=BUF)
pitch_buf = deque([0]*BUF, maxlen=BUF)
gf_buf    = deque([1]*BUF, maxlen=BUF)
lat_buf   = deque(maxlen=2000)
lon_buf   = deque(maxlen=2000)
fault_log = []

# ── Simulation State ───────────────────────────────────────────
state = {
    "t": 0.0,
    "speed": 0.0,
    "rpm": 0.0,
    "roll": 0.0,
    "pitch": 0.0,
    "lat": 12.9716,
    "lon": 77.5946,
    "heading": 0.0,
    "faults": 0,
    "gps_ok": True,
    "imu_ok": True,
    "can_ok": True,
    "sd_ok":  True,
    "lap": 0,
    "lap_start": 0.0,
    "distance": 0.0,
}

# ── CSV Log Setup ──────────────────────────────────────────────
os.makedirs("data", exist_ok=True)
log_filename = f"data/telemetry_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
csv_file = open(log_filename, "w", newline="")
csv_writer = csv.writer(csv_file)
csv_writer.writerow([
    "timestamp_ms","speed_kmh","rpm","accel_x","accel_y","accel_z",
    "roll","pitch","g_force","latitude","longitude","heading",
    "lap","distance_m","faults"
])

def apply_faults(t):
    """Inject and clear faults per schedule"""
    t_int = int(t)
    if t_int in FAULT_SCHEDULE:
        event = FAULT_SCHEDULE[t_int]
        if   event == "GPS_LOSS":       state["gps_ok"] = False; fault_log.append((t,"GPS LOSS"))
        elif event == "GPS_RESTORE":    state["gps_ok"] = True;  fault_log.append((t,"GPS OK"))
        elif event == "CAN_TIMEOUT":    state["can_ok"] = False; fault_log.append((t,"CAN TIMEOUT"))
        elif event == "CAN_RESTORE":    state["can_ok"] = True;  fault_log.append((t,"CAN OK"))
        elif event == "SD_FAIL":        state["sd_ok"]  = False; fault_log.append((t,"SD FAIL"))
        elif event == "SD_RESTORE":     state["sd_ok"]  = True;  fault_log.append((t,"SD OK"))
        elif event == "IMU_DISCONNECT": state["imu_ok"] = False; fault_log.append((t,"IMU DISC"))
        elif event == "IMU_RESTORE":    state["imu_ok"] = True;  fault_log.append((t,"IMU OK"))

def simulate_step():
    """One simulation tick — mirrors FreeRTOS sensor + GPS tasks"""
    t = state["t"]
    apply_faults(t)

    # ── Vehicle speed profile (realistic lap) ──
    phase = (t % 60) / 60.0
    target_speed = 80 + 40*np.sin(2*np.pi*phase) + random.gauss(0, 2)
    state["speed"] += (target_speed - state["speed"]) * 0.05
    state["speed"] = max(0, min(160, state["speed"]))
    state["rpm"]   = (state["speed"] / (WHEEL_DIAMETER_M * np.pi * 60 / 1000)) * 60

    # ── IMU simulation (complementary filter) ──
    if state["imu_ok"]:
        ax = random.gauss(0.0, 0.5) + np.sin(t * 0.3) * 2
        ay = random.gauss(0.0, 0.3) + np.cos(t * 0.5) * 1.5
        az = 9.81 + random.gauss(0, 0.1)
        gx = random.gauss(0, 1.0)
        gy = random.gauss(0, 0.8)

        accel_roll  = np.degrees(np.arctan2(ay, az))
        accel_pitch = np.degrees(np.arctan2(-ax, az))
        state["roll"]  = ALPHA*(state["roll"]  + gx*DT) + (1-ALPHA)*accel_roll
        state["pitch"] = ALPHA*(state["pitch"] + gy*DT) + (1-ALPHA)*accel_pitch
        g_force = np.sqrt(ax**2 + ay**2 + az**2) / 9.81
    else:
        ax = ay = gx = gy = 0; az = 9.81; g_force = 1.0

    # ── GPS simulation ──
    if state["gps_ok"]:
        state["heading"] += random.gauss(0, 2)
        rad = np.radians(state["heading"])
        state["lat"] += np.cos(rad) * state["speed"] * DT / 111320
        state["lon"] += np.sin(rad) * state["speed"] * DT / (111320 * np.cos(np.radians(state["lat"])))
        state["distance"] += state["speed"] * DT / 3.6

    # ── Lap detection ──
    if state["distance"] > 1000 * (state["lap"] + 1):
        state["lap"] += 1
        state["lap_start"] = t

    # ── Fault bitmask ──
    faults = (0 if state["gps_ok"] else 1) | \
             (0 if state["imu_ok"] else 2) | \
             (0 if state["can_ok"] else 4) | \
             (0 if state["sd_ok"]  else 8)

    # ── Append to buffers ──
    t_buf.append(t)
    spd_buf.append(state["speed"])
    rpm_buf.append(state["rpm"])
    gx_buf.append(ax); gy_buf.append(ay); gz_buf.append(az)
    roll_buf.append(state["roll"])
    pitch_buf.append(state["pitch"])
    gf_buf.append(g_force)
    lat_buf.append(state["lat"])
    lon_buf.append(state["lon"])

    # ── Log to CSV (mirrors logger_task.c) ──
    csv_writer.writerow([
        int(t*1000), round(state["speed"],2), round(state["rpm"],1),
        round(ax,4), round(ay,4), round(az,4),
        round(state["roll"],2), round(state["pitch"],2), round(g_force,3),
        round(state["lat"],6), round(state["lon"],6), round(state["heading"],1),
        state["lap"], round(state["distance"],1), hex(faults)
    ])

    state["t"] += DT
    return faults

# ── Dashboard Setup ────────────────────────────────────────────
fig = plt.figure(figsize=(16, 9), facecolor="#0d1117")
fig.suptitle("🚗 Arys Garage — Real-Time Vehicle Telemetry Dashboard",
             color="white", fontsize=14, fontweight="bold")

def make_ax(pos, title, ylabel="", xlabel=""):
    ax = fig.add_subplot(pos, facecolor="#161b22")
    ax.set_title(title, color="#58a6ff", fontsize=9, pad=4)
    ax.set_ylabel(ylabel, color="#8b949e", fontsize=8)
    ax.set_xlabel(xlabel, color="#8b949e", fontsize=8)
    ax.tick_params(colors="#8b949e", labelsize=7)
    for spine in ax.spines.values(): spine.set_color("#30363d")
    return ax

ax_spd   = make_ax(331, "Speed (km/h)",        "km/h")
ax_rpm   = make_ax(332, "Engine RPM",           "RPM")
ax_gf    = make_ax(333, "G-Force",              "G")
ax_roll  = make_ax(334, "Roll / Pitch (°)",     "degrees")
ax_accel = make_ax(335, "Accelerometer (m/s²)", "m/s²")
ax_gps   = make_ax(336, "GPS Track",            "Latitude", "Longitude")
ax_fault = make_ax(313, "Fault Timeline",       "Fault Code")

plt.tight_layout(rect=[0,0.03,1,0.95])

status_text = fig.text(0.01, 0.01, "", color="white", fontsize=8,
                       fontfamily="monospace")

frame_count = [0]

def update(frame):
    # Run 10 sim steps per frame (smooth animation)
    faults = 0
    for _ in range(10):
        faults = simulate_step()

    t_arr   = np.array(t_buf)
    spd_arr = np.array(spd_buf)
    rpm_arr = np.array(rpm_buf)
    gf_arr  = np.array(gf_buf)
    roll_arr  = np.array(roll_buf)
    pitch_arr = np.array(pitch_buf)
    ax_arr  = np.array(gx_buf)
    ay_arr  = np.array(gy_buf)
    az_arr  = np.array(gz_buf)

    def redraw(ax, *lines_data, colors):
        ax.cla()
        ax.set_facecolor("#161b22")
        for spine in ax.spines.values(): spine.set_color("#30363d")
        ax.tick_params(colors="#8b949e", labelsize=7)
        for (y, label), color in zip(lines_data, colors):
            ax.plot(t_arr, y, color=color, linewidth=1, label=label)
        if len(lines_data) > 1: ax.legend(fontsize=7, loc="upper left")

    redraw(ax_spd,  (spd_arr,  "Speed"),  colors=["#2ea043"])
    ax_spd.set_title("Speed (km/h)", color="#58a6ff", fontsize=9)
    ax_spd.set_ylabel("km/h", color="#8b949e", fontsize=8)
    ax_spd.set_ylim(0, 180)

    redraw(ax_rpm,  (rpm_arr, "RPM"),   colors=["#f78166"])
    ax_rpm.set_title("Engine RPM", color="#58a6ff", fontsize=9)
    ax_rpm.set_ylim(0, MAX_RPM)

    redraw(ax_gf,   (gf_arr,  "G-Force"), colors=["#d2a8ff"])
    ax_gf.set_title("G-Force", color="#58a6ff", fontsize=9)
    ax_gf.axhline(y=3, color="#f78166", linestyle="--", linewidth=0.8, alpha=0.7)

    redraw(ax_roll, (roll_arr, "Roll"), (pitch_arr, "Pitch"),
           colors=["#ffa657", "#79c0ff"])
    ax_roll.set_title("Roll / Pitch (°)", color="#58a6ff", fontsize=9)

    redraw(ax_accel, (ax_arr, "Ax"), (ay_arr, "Ay"), (az_arr, "Az"),
           colors=["#ff7b72", "#79c0ff", "#2ea043"])
    ax_accel.set_title("Accelerometer (m/s²)", color="#58a6ff", fontsize=9)

    # GPS track
    ax_gps.cla()
    ax_gps.set_facecolor("#161b22")
    for spine in ax_gps.spines.values(): spine.set_color("#30363d")
    ax_gps.tick_params(colors="#8b949e", labelsize=7)
    if len(lat_buf) > 1:
        ax_gps.plot(list(lon_buf), list(lat_buf), color="#58a6ff",
                    linewidth=1, alpha=0.8)
        ax_gps.plot(state["lon"], state["lat"], "o",
                    color="#2ea043", markersize=6)
    ax_gps.set_title(f"GPS Track {'⚠ LOST' if not state['gps_ok'] else '✓'}",
                     color="#f78166" if not state["gps_ok"] else "#58a6ff", fontsize=9)

    # Fault timeline
    ax_fault.cla()
    ax_fault.set_facecolor("#161b22")
    for spine in ax_fault.spines.values(): spine.set_color("#30363d")
    ax_fault.tick_params(colors="#8b949e", labelsize=7)
    ax_fault.set_title("Fault Timeline", color="#58a6ff", fontsize=9)
    ax_fault.set_xlim(0, SIM_DURATION_S)
    ax_fault.set_ylim(-0.5, 3.5)
    ax_fault.set_yticks([0,1,2,3])
    ax_fault.set_yticklabels(["GPS","IMU","CAN","SD"], color="#8b949e", fontsize=8)
    fault_colors = {
        "GPS LOSS":"#f78166","GPS OK":"#2ea043",
        "CAN TIMEOUT":"#ffa657","CAN OK":"#2ea043",
        "SD FAIL":"#d2a8ff","SD OK":"#2ea043",
        "IMU DISC":"#79c0ff","IMU OK":"#2ea043",
    }
    fault_y = {"GPS":0,"IMU":1,"CAN":2,"SD":3}
    for ft, label in fault_log:
        key = label.split()[0]
        y = fault_y.get(key, 0)
        color = fault_colors.get(label, "white")
        ax_fault.axvline(x=ft, ymin=y/4, ymax=(y+1)/4,
                         color=color, linewidth=2)
        ax_fault.text(ft+0.5, y, label, color=color, fontsize=6)

    # Status bar
    fault_str = f"{'⚠GPS ' if not state['gps_ok'] else ''}{'⚠IMU ' if not state['imu_ok'] else ''}{'⚠CAN ' if not state['can_ok'] else ''}{'⚠SD' if not state['sd_ok'] else ''}"
    status_text.set_text(
        f"  T={state['t']:.1f}s  |  Speed={state['speed']:.1f}km/h  |  "
        f"RPM={state['rpm']:.0f}  |  Lap={state['lap']}  |  "
        f"Dist={state['distance']:.0f}m  |  "
        f"Roll={state['roll']:.1f}°  Pitch={state['pitch']:.1f}°  |  "
        f"Faults: {fault_str if fault_str else 'None'}  |  "
        f"Log: {log_filename}"
    )

    frame_count[0] += 1
    if state["t"] >= SIM_DURATION_S:
        csv_file.close()
        print(f"\n✅ Simulation complete! Log saved to: {log_filename}")
        ani.event_source.stop()

ani = animation.FuncAnimation(fig, update, interval=100, cache_frame_data=False)
plt.show()
