"""Analyze accel/gyro clipping in a session to choose the right IMU range."""
import json, glob, os, sys
from pathlib import Path

if len(sys.argv) < 2:
    print("usage: analyze_clipping.py <session_dir>")
    sys.exit(1)

session = Path(sys.argv[1])
files = sorted(glob.glob(str(session / "**/pkt_*.json"), recursive=True),
               key=lambda p: int(Path(p).stem.split("_")[1]))
print(f"session: {session}  packets: {len(files)}")

# Thresholds: treat |a| >= 1.95 g as "at the rail" for ±2g range.
# Same logic for gyro defaults (±250 dps).
ACC_LIMIT = 1.95   # g, ±2 g range
GYR_LIMIT = 245.0  # dps, ±250 dps default

total_samples = 0
acc_clipped   = 0          # samples where any axis saturated
max_abs_a     = 0.0
acc_hist      = {"<2g":0, "2-4g":0, "4-8g":0, "8-16g":0, ">=16g":0}
# note: since the sensor was at ±2g, anything ≥1.95 is just "clipped" — we can't
# tell *how much*. We can only see the true magnitude if unclipped.

gyr_over = 0
max_abs_g = 0.0

packet_clip_frac = []      # fraction of samples per packet that clip

for fp in files:
    try:
        j = json.loads(Path(fp).read_text())
    except Exception as e:
        continue
    imu = j.get("IMU", [])
    if not imu: continue
    pkt_clipped = 0
    for s in imu:
        total_samples += 1
        a = s.get("a", {})
        g = s.get("g", {})
        ax, ay, az = a.get("x",0), a.get("y",0), a.get("z",0)
        gx, gy, gz = g.get("x",0), g.get("y",0), g.get("z",0)

        mA = max(abs(ax), abs(ay), abs(az))
        mG = max(abs(gx), abs(gy), abs(gz))
        if mA > max_abs_a: max_abs_a = mA
        if mG > max_abs_g: max_abs_g = mG

        if mA >= ACC_LIMIT:
            acc_clipped += 1
            pkt_clipped += 1
        if mG >= GYR_LIMIT:
            gyr_over += 1

        # Bucket by unclipped magnitude (gives sense of distribution below rail)
        if mA < 2:   acc_hist["<2g"] += 1
        elif mA < 4: acc_hist["2-4g"] += 1
        elif mA < 8: acc_hist["4-8g"] += 1
        elif mA < 16:acc_hist["8-16g"] += 1
        else:        acc_hist[">=16g"] += 1
    if imu: packet_clip_frac.append(pkt_clipped / len(imu))

if total_samples == 0:
    print("no IMU samples found")
    sys.exit(0)

pct_clip = 100 * acc_clipped / total_samples
pct_gyr  = 100 * gyr_over / total_samples

print(f"\nsamples total: {total_samples}  ({len(files)} packets)")
print(f"accel clipped (≥{ACC_LIMIT}g on any axis): {acc_clipped}  ({pct_clip:.1f} %)")
print(f"max |a| seen: {max_abs_a:.3f} g  (capped at 2 g by hardware)")
print(f"gyro near limit (≥{GYR_LIMIT} dps): {gyr_over}  ({pct_gyr:.1f} %)")
print(f"max |g| seen: {max_abs_g:.1f} dps")

print("\nper-packet clipping distribution:")
if packet_clip_frac:
    # quantiles
    pcf = sorted(packet_clip_frac)
    def q(p): return pcf[int(p*(len(pcf)-1))]
    print(f"  median clip-frac: {q(0.5)*100:.1f}%   p90: {q(0.9)*100:.1f}%   max: {max(pcf)*100:.1f}%")
    heavy = sum(1 for x in pcf if x > 0.2)
    print(f"  packets with >20% samples clipped: {heavy} / {len(pcf)} ({100*heavy/len(pcf):.1f}%)")

print("\naccel magnitude histogram (max axis, capped at 2g by hardware):")
for k,v in acc_hist.items():
    print(f"  {k:>8}: {v:>6}  ({100*v/total_samples:5.1f} %)")
