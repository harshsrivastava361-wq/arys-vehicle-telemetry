"""
============================================================
 Arys Garage — Q2: Real-Time Vehicle Telemetry System
 File   : dashboard.py
 About  : Live telemetry dashboard — 6 panels updating every 150ms.

 How to run:
     python3 dashboard.py

 What you'll see:
   Panel 1 (top-left)  : Speed history line chart
   Panel 2 (mid-left)  : Lateral & Longitudinal G-force bars
   Panel 3 (right tall): GPS track (live lap trace)
   Panel 4 (mid-centre): Roll & Pitch angles (sensor fusion output)
   Panel 5 (bottom wide): Wheel RPM for all 4 wheels
   Panel 6 (bottom-right): System health — fault indicators

 Faults are auto-injected at t=15s, t=30s, t=42s for demo.

 AI Usage: Dashboard layout generated with Claude (Anthropic).
           Animation loop and colour scheme adapted manually.
============================================================
"""

import matplotlib
matplotlib.use('TkAgg')          # required on macOS
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import numpy as np
import time
import threading
from collections import deque

import sensor_sim as sim

# ──────────────────────────────────────────────────────────────
#  HISTORY BUFFERS  (rolling window for line charts)
# ──────────────────────────────────────────────────────────────
N = 250   # number of samples to show on charts

speed_buf   = deque([0.0] * N, maxlen=N)
gx_buf      = deque([0.0] * N, maxlen=N)
gy_buf      = deque([0.0] * N, maxlen=N)
roll_buf    = deque([0.0] * N, maxlen=N)
pitch_buf   = deque([0.0] * N, maxlen=N)
lat_buf     = deque(maxlen=800)
lon_buf     = deque(maxlen=800)

_start = time.time()

# ──────────────────────────────────────────────────────────────
#  FIGURE SETUP
# ──────────────────────────────────────────────────────────────
plt.style.use('dark_background')
fig = plt.figure(figsize=(15, 8))
fig.patch.set_facecolor('#0d1117')
fig.suptitle(
    'Arys Garage  —  Real-Time Vehicle Telemetry Dashboard',
    fontsize=14, color='#e6edf3', fontweight='bold', y=0.99
)

gs = gridspec.GridSpec(3, 3, figure=fig, hspace=0.55, wspace=0.38)

ax_spd    = fig.add_subplot(gs[0, :2])   # Speed       — wide top
ax_gf     = fig.add_subplot(gs[1, 0])   # G-force
ax_gps    = fig.add_subplot(gs[0:2, 2]) # GPS track   — tall right
ax_ang    = fig.add_subplot(gs[1, 1])   # Roll & Pitch
ax_rpm    = fig.add_subplot(gs[2, :2])  # Wheel RPM   — wide bottom
ax_flt    = fig.add_subplot(gs[2, 2])   # Fault status

BG = '#161b22'
BORDER = '#30363d'

for ax in [ax_spd, ax_gf, ax_gps, ax_ang, ax_rpm, ax_flt]:
    ax.set_facecolor(BG)
    for sp in ax.spines.values():
        sp.set_edgecolor(BORDER)
    ax.tick_params(colors='#8b949e', labelsize=8)

xs = list(range(N))

# ──────────────────────────────────────────────────────────────
#  ANIMATION CALLBACK  (called every 150 ms by FuncAnimation)
# ──────────────────────────────────────────────────────────────
def animate(_frame):
    elapsed = time.time() - _start

    # Snapshot shared state (thread-safe)
    with sim._lock:
        s = dict(sim._state)

    faults = s['faults']

    # Append to rolling buffers
    speed_buf.append(s['speed_kmh'])
    gx_buf.append(s['gforce_x'])
    gy_buf.append(s['gforce_y'])
    roll_buf.append(s['roll_deg'])
    pitch_buf.append(s['pitch_deg'])
    lat_buf.append(s['latitude'])
    lon_buf.append(s['longitude'])

    spds = list(speed_buf)

    # ── Panel 1 : Speed ──────────────────────────────────────
    ax_spd.cla(); ax_spd.set_facecolor(BG)
    for sp in ax_spd.spines.values(): sp.set_edgecolor(BORDER)
    ax_spd.plot(xs, spds, color='#58a6ff', linewidth=1.5, zorder=2)
    ax_spd.fill_between(xs, spds, alpha=0.12, color='#58a6ff', zorder=1)
    ax_spd.axhline(100, color='#f85149', linestyle='--', alpha=0.5, linewidth=0.8)
    ax_spd.text(5, 103, 'speed limit', color='#f85149', fontsize=7)
    ax_spd.set_ylim(0, 160)
    ax_spd.set_title(f'Speed  {s["speed_kmh"]:.1f} km/h', color='#e6edf3', fontsize=11, pad=5)
    ax_spd.set_ylabel('km/h', color='#8b949e', fontsize=9)
    ax_spd.tick_params(colors='#8b949e', labelsize=8)

    # ── Panel 2 : G-Force ────────────────────────────────────
    ax_gf.cla(); ax_gf.set_facecolor(BG)
    for sp in ax_gf.spines.values(): sp.set_edgecolor(BORDER)
    gx, gy = s['gforce_x'], s['gforce_y']
    clrs = ['#f85149' if abs(gx) > 1.0 else '#3fb950',
            '#f85149' if abs(gy) > 1.0 else '#d29922']
    ax_gf.bar(['Long (X)', 'Lat (Y)'], [gx, gy], color=clrs, width=0.45, zorder=2)
    ax_gf.axhline(0, color='#8b949e', linewidth=0.5)
    ax_gf.set_ylim(-2.5, 2.5)
    ax_gf.set_title(f'G-Force  X:{gx:+.2f}g  Y:{gy:+.2f}g', color='#e6edf3', fontsize=10, pad=5)
    ax_gf.set_ylabel('g', color='#8b949e', fontsize=9)
    ax_gf.tick_params(colors='#8b949e', labelsize=8)

    # ── Panel 3 : GPS Track ───────────────────────────────────
    ax_gps.cla(); ax_gps.set_facecolor(BG)
    for sp in ax_gps.spines.values(): sp.set_edgecolor(BORDER)
    if len(lat_buf) > 2:
        lats = list(lat_buf); lons = list(lon_buf)
        n = len(lats)
        colors_gps = plt.cm.plasma(np.linspace(0, 1, n))
        ax_gps.scatter(lons, lats, c=colors_gps, s=1.5, zorder=1)
        ax_gps.plot(lons[-1], lats[-1], 'o', color='#58a6ff', markersize=7, zorder=3)
    fix_label = 'FIX OK' if not faults.gps_loss else 'NO FIX'
    fix_color = '#3fb950' if not faults.gps_loss else '#f85149'
    ax_gps.set_title(f'GPS Track  {fix_label}', color=fix_color, fontsize=10, pad=5)
    ax_gps.set_xlabel('Longitude', color='#8b949e', fontsize=8)
    ax_gps.set_ylabel('Latitude',  color='#8b949e', fontsize=8)
    ax_gps.tick_params(colors='#8b949e', labelsize=7)

    # ── Panel 4 : Roll & Pitch ───────────────────────────────
    ax_ang.cla(); ax_ang.set_facecolor(BG)
    for sp in ax_ang.spines.values(): sp.set_edgecolor(BORDER)
    roll, pitch = s['roll_deg'], s['pitch_deg']
    ax_ang.bar(['Roll', 'Pitch'], [roll, pitch],
               color=['#a371f7', '#79c0ff'], width=0.4, zorder=2)
    ax_ang.axhline(0, color='#8b949e', linewidth=0.5)
    ax_ang.set_ylim(-25, 25)
    ax_ang.set_title(f'Sensor Fusion  R:{roll:.1f}°  P:{pitch:.1f}°',
                     color='#e6edf3', fontsize=10, pad=5)
    ax_ang.set_ylabel('degrees', color='#8b949e', fontsize=9)
    ax_ang.tick_params(colors='#8b949e', labelsize=8)

    # ── Panel 5 : Wheel RPM ──────────────────────────────────
    ax_rpm.cla(); ax_rpm.set_facecolor(BG)
    for sp in ax_rpm.spines.values(): sp.set_edgecolor(BORDER)
    try:
        w = sim.wheel_queue.get_nowait()
        rpms = [w.rpm_fl, w.rpm_fr, w.rpm_rl, w.rpm_rr]
    except Exception:
        base = s['speed_kmh'] / 3.6 / 1.85 * 60
        rpms = [base + i for i in [0, 0.5, 1, 1.5]]
    labels  = ['FL', 'FR', 'RL', 'RR']
    rcolors = ['#3fb950', '#3fb950', '#58a6ff', '#58a6ff']
    bars = ax_rpm.bar(labels, rpms, color=rcolors, width=0.5, zorder=2)
    avg_rpm = sum(rpms) / 4
    ax_rpm.set_title(f'Wheel RPM  (avg {avg_rpm:.0f} RPM  ·  {s["speed_kmh"]:.1f} km/h)',
                     color='#e6edf3', fontsize=10, pad=5)
    ax_rpm.set_ylabel('RPM', color='#8b949e', fontsize=9)
    ax_rpm.tick_params(colors='#8b949e', labelsize=9)
    for bar, v in zip(bars, rpms):
        ax_rpm.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
                    f'{v:.0f}', ha='center', va='bottom', color='#8b949e', fontsize=8)

    # ── Panel 6 : Fault Status ───────────────────────────────
    ax_flt.cla(); ax_flt.set_facecolor(BG)
    for sp in ax_flt.spines.values(): sp.set_edgecolor(BORDER)
    ax_flt.set_xlim(0, 1); ax_flt.set_ylim(0, 1); ax_flt.axis('off')
    ax_flt.set_title('System Health', color='#e6edf3', fontsize=10, pad=5)

    items = [
        ('GPS Module',  not faults.gps_loss),
        ('IMU (MPU6050)', not faults.imu_disconnect),
        ('CAN Bus',     not faults.can_timeout),
        ('SD Logger',   not faults.sd_write_fail),
    ]

    for i, (label, ok) in enumerate(items):
        y = 0.80 - i * 0.19
        col = '#3fb950' if ok else '#f85149'
        txt = ' OK ' if ok else 'FAULT'
        circ = plt.Circle((0.10, y), 0.05, color=col, transform=ax_flt.transAxes, zorder=3)
        ax_flt.add_patch(circ)
        ax_flt.text(0.22, y, label, color='#e6edf3', fontsize=9,
                    va='center', transform=ax_flt.transAxes)
        ax_flt.text(0.82, y, txt, color=col, fontsize=8, fontweight='bold',
                    ha='center', va='center', transform=ax_flt.transAxes)

    ax_flt.text(0.5, 0.04,
                f"Dist: {s['distance_m']:.0f} m   T: {elapsed:.1f} s",
                color='#8b949e', fontsize=8, ha='center', transform=ax_flt.transAxes)

# ──────────────────────────────────────────────────────────────
#  MAIN
# ──────────────────────────────────────────────────────────────
if __name__ == '__main__':
    print('=' * 55)
    print('  Arys Garage — Live Telemetry Dashboard')
    print('=' * 55)

    sim.start_all_tasks(log_file='data/telemetry_log.csv')
    time.sleep(0.3)

    # Auto-inject faults at fixed times for demo recording
    def _schedule():
        time.sleep(15); sim.inject_fault('gps_loss',       5.0)
        time.sleep(15); sim.inject_fault('can_timeout',    4.0)
        time.sleep(12); sim.inject_fault('sd_write_fail',  3.0)
        time.sleep(8);  sim.inject_fault('imu_disconnect', 2.0)

    threading.Thread(target=_schedule, daemon=True).start()
    print('\nFault schedule: GPS@15s · CAN@30s · SD@42s · IMU@50s')
    print('Close window to stop.\n')

    ani = animation.FuncAnimation(
        fig, animate, interval=150, cache_frame_data=False
    )
    plt.show()

    sim._state['running'] = False
    print('Dashboard closed. Log saved to data/telemetry_log.csv')
