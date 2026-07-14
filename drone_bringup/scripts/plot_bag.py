#!/usr/bin/env python3
"""plot_bag.py — 解析 rosbag .db3，出 CSV + 数值摘要。纯 Python stdlib。"""
import sqlite3, struct, sys, os, math, argparse

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('bag_dir')
    p.add_argument('--summary', action='store_true')
    return p.parse_args()

def find_db3(d):
    for f in os.listdir(d):
        if f.endswith('.db3'):
            return os.path.join(d, f)
    sys.exit(f"no .db3 in {d}")

def read_bag(bag_dir):
    db = find_db3(bag_dir)
    con = sqlite3.connect(db)
    cur = con.cursor()
    cur.execute("SELECT id, name FROM topics")
    topics = {r[1]: r[0] for r in cur.fetchall()}
    print(f"Topics: {list(topics.keys())}")
    out = {}

    for tname, want in [('/drone/odom', 'odom'), ('/drone/motor_rpm_cmd', 'rpm'), ('/drone/goal', 'goals')]:
        if tname not in topics:
            continue
        tid = topics[tname]
        cur.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp", (tid,))
        rows = cur.fetchall()
        data = []
        for ts, blob in rows:
            try:
                if want == 'odom' or want == 'goals':
                    doubles = struct.unpack(f'<{len(blob)//8}d', blob[:len(blob)-(len(blob)%8)])
                    for i in range(len(doubles)-2):
                        x,y,z = doubles[i], doubles[i+1], doubles[i+2]
                        if all(abs(v) < 1e6 for v in (x,y,z)):
                            data.append((ts*1e-9, x, y, z))
                            break
                elif want == 'rpm':
                    floats = struct.unpack(f'<{len(blob)//4}f', blob[:len(blob)-(len(blob)%4)])
                    if len(floats) >= 4:
                        data.append((ts*1e-9, floats[:4]))
            except:
                continue
        out[want] = data
        print(f"  {tname}: {len(data)} samples")
    con.close()
    return out

def summarize(data):
    if 'odom' not in data or not data['odom']:
        print("No odom data"); return
    odom = data['odom']; t0 = odom[0][0]
    # find goal
    gx, gy, gz = 0, 0, 1.5
    if 'goals' in data and data['goals']:
        g = data['goals'][-1]; gx,gy,gz = g[1],g[2],g[3]
    print(f"\n{'='*60}")
    print(f"ANALYSIS: {os.path.basename(args.bag_dir)}")
    print(f"{'='*60}")
    print(f"Goal: ({gx:.2f}, {gy:.2f}, {gz:.2f})")
    # steady-state error (last 1s)
    te = odom[-1][0]
    ss = [d for d in odom if d[0] > te - 1.0]
    if ss:
        ex = sum(abs(d[1]-gx) for d in ss)/len(ss)
        ey = sum(abs(d[2]-gy) for d in ss)/len(ss)
        ez = sum(abs(d[3]-gz) for d in ss)/len(ss)
        et = math.sqrt(ex*ex+ey*ey+ez*ez)
        print(f"Steady-state err (last 1s): x={ex:.4f} y={ey:.4f} z={ez:.4f} ||e||={et:.4f}")
        print(f"Hover <0.3m: {'PASS' if et<0.3 else 'FAIL'}")
    # max overshoot
    if gz > 0.1:
        zmax = max(d[3] for d in odom)
        overshoot = max(0, zmax - gz)
        print(f"Max z overshoot: {overshoot:.3f}m (peak {zmax:.3f} vs target {gz:.2f})")
    # settling time
    for i, d in enumerate(odom[1:], 1):
        e = math.sqrt((d[1]-gx)**2+(d[2]-gy)**2+(d[3]-gz)**2)
        if e < 0.3:
            w = [dd for dd in odom[i:] if dd[0] <= d[0]+0.5]
            if w and all(math.sqrt((dd[1]-gx)**2+(dd[2]-gy)**2+(dd[3]-gz)**2)<0.3 for dd in w):
                print(f"Settling time: {d[0]-t0:.2f}s"); break
    # RPM
    if 'rpm' in data:
        r = data['rpm'][-100:] if len(data['rpm'])>100 else data['rpm']
        for m in range(4):
            vals = [ri[1][m] for ri in r]
            print(f"RPM m{m+1}: mean={sum(vals)/len(vals):.1f} min={min(vals):.1f} max={max(vals):.1f}")
    # trajectory
    dist = sum(math.sqrt((odom[i][1]-odom[i-1][1])**2+(odom[i][2]-odom[i-1][2])**2+(odom[i][3]-odom[i-1][3])**2) for i in range(1,len(odom)))
    dur = odom[-1][0] - odom[0][0]
    print(f"Path: {dist:.2f}m in {dur:.2f}s (avg {dist/dur:.2f}m/s)")

def export_csv(data, bag_dir):
    name = os.path.basename(bag_dir.rstrip('/'))
    for key, fn, cols in [('odom',f'{name}_odom.csv',"t,x,y,z"), ('rpm',f'{name}_rpm.csv',"t,rpm1,rpm2,rpm3,rpm4")]:
        if key not in data: continue
        with open(fn,'w') as f:
            f.write(cols+"\n")
            for d in data[key]:
                if key == 'odom':
                    f.write(f"{d[0]:.6f},{d[1]:.4f},{d[2]:.4f},{d[3]:.4f}\n")
                elif key == 'rpm':
                    f.write(f"{d[0]:.6f},{d[1][0]:.2f},{d[1][1]:.2f},{d[1][2]:.2f},{d[1][3]:.2f}\n")
        print(f"  Exported {fn} ({len(data[key])} rows)")

if __name__ == '__main__':
    args = parse_args()
    data = read_bag(args.bag_dir)
    summarize(data)
    if not args.summary:
        export_csv(data, args.bag_dir)
