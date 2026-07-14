#!/usr/bin/env python3
"""LADRC auto-tuner: 3-stage adaptive refinement (no pip deps needed)"""
import subprocess, sys, time, math, random, threading

def sh(cmd, timeout=10):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)

def pset(k, v): sh(f'ros2 param set /drone_controller {k} {v}', timeout=5)
def goal(x,y,z): sh(f'ros2 topic pub --once /drone/goal geometry_msgs/msg/PoseStamped "{{header: {{frame_id: \\"map\\"}}, pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}}}"', timeout=5)

# ---- 后台 odom 流 ----
odom_lines = []; odom_lock = threading.Lock(); odom_proc = None
def start_odom():
    global odom_proc
    odom_proc = subprocess.Popen(['ros2','topic','echo','/drone/odom','--field','pose.pose.position'],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
def stop_odom(): 
    global odom_proc
    if odom_proc: odom_proc.kill(); odom_proc.wait()
def odom_reader():
    while odom_proc and odom_proc.poll() is None:
        l = odom_proc.stdout.readline()
        if not l: break
        with odom_lock: odom_lines.append(l.strip())
def latest_z():
    with odom_lock:
        for l in reversed(odom_lines): 
            if l.startswith('z:'): return float(l.split(':')[1])
    return None
def clear_odom():
    with odom_lock: odom_lines.clear()

def step_response(tz, dur):
    goal(0,0,tz); clear_odom()
    traj, t0 = [], time.time()
    while time.time()-t0 < dur:
        z=latest_z()
        if z is not None: traj.append(z)
        time.sleep(0.02)
    return traj

def cost(traj, target):
    """代价：瞬态 RMSE（前1/3）+ 稳态误差（后2/3）+ 超调惩罚"""
    if len(traj) < 30: return 999.0
    n = len(traj)
    n1 = n // 3  # 瞬态部分
    # 瞬态 RMSE：收敛阶段的跟踪误差
    err2_trans = sum((z-target)**2 for z in traj[:n1])
    rmse_trans = math.sqrt(err2_trans / n1)
    # 稳态平均绝对误差
    steady_err = sum(abs(z-target) for z in traj[2*n//3:]) / max(n - 2*n//3, 1)
    # 超调：最大值超过 target 的量
    max_z = max(traj)
    min_z = min(traj)
    overshoot_up = max(0, max_z - target)
    overshoot_dn = max(0, target - min_z)
    return rmse_trans + steady_err * 3.0 + overshoot_up * 0.5 + overshoot_dn * 0.5

# ---- 自适应粗搜→细搜 ----
STAGES = [
    # (trials, b0_range, wc_range, wo_range)
    (20, (0.5, 3.0), (1.0, 10.0), (3.0, 50.0)),   # 阶段1: 大范围粗搜
    (15, None, None, None),                          # 阶段2: 40% 范围细搜
    (10, None, None, None),                          # 阶段3: 20% 范围精搜
]

pset('control_mode', 'ladrc')
time.sleep(0.5)
start_odom()
reader = threading.Thread(target=odom_reader, daemon=True); reader.start()
time.sleep(1.0)

try:
    best_b0, best_wc, best_wo = 1.0, 3.0, 15.0
    best_cost = 1e9
    b0_r, wc_r, wo_r = (0.5, 3.0), (1.0, 10.0), (3.0, 50.0)

    for stage_idx, (ntrials, br, wr, wor) in enumerate(STAGES):
        if br is not None:
            b0_r, wc_r, wo_r = br, wr, wor
        else:
            # 细搜：在上轮最佳附近缩窄40%/20%
            scale = 0.4 if stage_idx == 1 else 0.2
            b0_r = (max(0.3, best_b0 - best_b0*scale), best_b0 + best_b0*scale)
            wc_r = (max(0.5, best_wc - best_wc*scale), best_wc + best_wc*scale)
            wo_r = (max(2.0, best_wo - best_wo*scale), best_wo + best_wo*scale)

        print(f"\n--- Stage {stage_idx+1}: {ntrials} trials, "
              f"b0=[{b0_r[0]:.2f},{b0_r[1]:.2f}] "
              f"wc=[{wc_r[0]:.1f},{wc_r[1]:.1f}] "
              f"wo=[{wo_r[0]:.1f},{wo_r[1]:.1f}] ---")

        for i in range(ntrials):
            b0 = b0_r[0] + random.random() * (b0_r[1] - b0_r[0])
            wc = wc_r[0] + random.random() * (wc_r[1] - wc_r[0])
            wo = wo_r[0] + random.random() * (wo_r[1] - wo_r[0])

            pset('ladrc_b0', f'[1.0, 1.0, {b0:.4f}]')
            pset('ladrc_wc', f'[2.0, 2.0, {wc:.4f}]')
            pset('ladrc_wo', f'[10.0, 10.0, {wo:.4f}]')
            time.sleep(0.3)

            traj_up = step_response(1.5, 3.0)
            c1 = cost(traj_up, 1.5)
            traj_dn = step_response(0.0, 3.0)
            c2 = cost(traj_dn, 0.0)
            c = 0.5 * (c1 + c2)

            marker = ""
            if c < best_cost and c < 900:
                best_cost = c
                best_b0, best_wc, best_wo = b0, wc, wo
                marker = " <-- NEW BEST"

            print(f"  [{stage_idx+1}:{i:2d}] b0={b0:.3f} wc={wc:.1f} wo={wo:.1f} "
                  f"cost={c:.4f}  (best={best_cost:.4f} @ b0={best_b0:.3f}){marker}")

    print(f"\n{'='*60}")
    print(f"CONVERGED: b0_z={best_b0:.4f} wc_z={best_wc:.1f} wo_z={best_wo:.1f}")
    print(f"           cost={best_cost:.4f}")
    print(f"  ros2 param set /drone_controller ladrc_b0 '[1.0, 1.0, {best_b0:.4f}]'")
    print(f"  ros2 param set /drone_controller ladrc_wc '[2.0, 2.0, {best_wc:.1f}]'")
    print(f"  ros2 param set /drone_controller ladrc_wo '[10.0, 10.0, {best_wo:.1f}]'")

    # 设最佳值
    pset('ladrc_b0', f'[1.0, 1.0, {best_b0:.4f}]')
    pset('ladrc_wc', f'[2.0, 2.0, {best_wc:.1f}]')
    pset('ladrc_wo', f'[10.0, 10.0, {best_wo:.1f}]')

finally:
    stop_odom()
    print("\nDone. ros2 param set /drone_controller control_mode pd  # to switch back")
