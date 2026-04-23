"""Quick scan: find accel-magnitude peaks in a foot session."""
import json, glob, sys
from pathlib import Path

if len(sys.argv) < 2:
    print("usage: analyze_peaks.py <session_dir>"); sys.exit(1)

files = sorted(glob.glob(str(Path(sys.argv[1]) / "**/pkt_*.json"), recursive=True),
               key=lambda p: int(Path(p).stem.split("_")[1]))
print(f"packets: {len(files)}")

all_mags = []
peaks = []   # (ts_ms_since_boot, mag, pkt_idx)
for fp in files:
    j = json.loads(Path(fp).read_text())
    imu = j.get("IMU", [])
    if not imu: continue
    n = len(imu)
    dt_ms = 1000 / n
    ts0 = j["ts"]
    for i, s in enumerate(imu):
        a = s["a"]
        m = (a["x"]**2 + a["y"]**2 + a["z"]**2) ** 0.5
        ts = ts0 + i * dt_ms
        all_mags.append((ts, m))

# Stats
vals = [m for _,m in all_mags]
vals.sort()
def q(p): return vals[int(p*(len(vals)-1))]
print(f"|a| quantiles: min={vals[0]:.2f}  p50={q(0.5):.2f}  p90={q(0.9):.2f}  p99={q(0.99):.2f}  max={vals[-1]:.2f}")

# Local maxima with cooldown
COOLDOWN = 400
THR = 1.6
peaks = []
last_peak_ts = -1e9
rising = False; last_m = 0; last_ts = 0
for ts, m in all_mags:
    if m > last_m:
        rising = True
    elif rising and last_m >= THR and (last_ts - last_peak_ts) > COOLDOWN:
        peaks.append((last_ts, last_m))
        last_peak_ts = last_ts
        rising = False
    elif m < last_m:
        rising = False
    last_m = m; last_ts = ts

print(f"\nfound {len(peaks)} peaks >={THR}g with {COOLDOWN}ms cooldown")
peaks.sort(key=lambda x: -x[1])
print("top 12 peaks (ts ms since boot, |a|):")
for ts, m in peaks[:12]:
    print(f"  ts={ts/1000:.2f}s  |a|={m:.2f}g")
