import json, os, glob, wave
base = r'E:/data12'
pkts = sorted(glob.glob(base + '/**/pkt_*.json', recursive=True)
              + glob.glob(base + '/pkt_*.json'),
              key=lambda p: int(os.path.basename(p).split('_')[1].split('.')[0]))
print("idx   ts_ms    dts  imuN  tofN  wav_ms  wav_samples")
prev_ts = None
for p in pkts:
    if os.path.getsize(p) == 0:
        print(f"  {os.path.basename(p)}: EMPTY (truncated)")
        continue
    d = json.load(open(p))
    ts = d.get('ts', d.get('ts_ms', 0))
    imu = len(d.get('IMU', []))
    tof = len(d.get('tof', []))
    dt = ts - prev_ts if prev_ts is not None else 0
    idx = int(os.path.basename(p).split('_')[1].split('.')[0])
    pkt_dir = os.path.dirname(p)
    wav_path = f"{pkt_dir}/rec_{idx}_{ts}.wav"
    wav_ms = '-'
    wav_n = '-'
    if os.path.exists(wav_path):
        with wave.open(wav_path, 'rb') as w:
            wav_n = w.getnframes()
            wav_ms = round(wav_n / w.getframerate() * 1000)
    print(f"{idx:3d}  {ts:6d}  {dt:5d}  {imu:4d}  {tof:4d}  {str(wav_ms):>6}  {wav_n}")
    prev_ts = ts

# Gap between consecutive WAVs: file N ends at ts[N]+1100, file N+1 starts at ts[N+1]
# If ts[N+1] - ts[N] < 1100, there's overlap (good). If > 1100, there's a gap (bad).
print("\nInter-packet coverage (ts delta vs 1100 ms wav length):")
for i in range(1, len(pkts)):
    if os.path.getsize(pkts[i-1]) == 0 or os.path.getsize(pkts[i]) == 0:
        continue
    d0 = json.load(open(pkts[i-1]))
    d1 = json.load(open(pkts[i]))
    t0 = d0.get('ts', d0.get('ts_ms'))
    t1 = d1.get('ts', d1.get('ts_ms'))
    dt = t1 - t0
    overlap = 1100 - dt  # positive = overlap, negative = gap
    status = "OVERLAP" if overlap > 0 else ("GAP" if overlap < 0 else "exact")
    print(f"  pkt {i-1}->{i}: dts={dt}ms, overlap={overlap:+d}ms [{status}]")
