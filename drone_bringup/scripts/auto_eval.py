#!/usr/bin/env python3
"""auto_eval.py — 6 场景自动验收脚本
用法:
  ros2 run drone_bringup auto_eval.py
输出: eval_report.json (pass/fail + 数值结果)
"""
import subprocess, sys, time, json, math, os, signal, threading

SCENARIOS = {
    "hover": {
        "desc": "悬停 10s，稳态 z 误差 < 0.3m",
        "launch": "no_planner.launch.py",
        "duration": 12,
        "goal": None,
        "check": lambda d: abs(d["z_mean"] - 1.5) < 0.3
    },
    "goto_goal": {
        "desc": "飞向 (2,1,1.5)，终点误差 < 0.3m",
        "launch": "no_planner.launch.py",
        "duration": 15,
        "goal": (2.0, 1.0, 1.5),
        "check": lambda d: d["final_err"] < 0.3
    },
    "square": {
        "desc": "正方形航线 4 点 >= 3 点到达误差 < 0.3m",
        "launch": "no_planner.launch.py",
        "duration": 25,
        "goal": None,
        "check": lambda d: d.get("waypoints_reached", 0) >= 3
    },
    "obstacle_avoidance": {
        "desc": "有障碍物时 min_dist > 0.4m",
        "launch": "full.launch.py",
        "duration": 15,
        "goal": (2.0, 1.0, 1.5),
        "check": lambda d: d.get("min_obs_dist", 0) > 0.4
    },
    "narrow_passage": {
        "desc": "窄通道绕行成功到达目标",
        "launch": "full.launch.py",
        "duration": 20,
        "goal": (3.0, 2.5, 1.5),
        "check": lambda d: d["final_err"] < 1.0
    },
    "stability": {
        "desc": "悬停 15s，位置误差标准差 < 0.1m",
        "launch": "no_planner.launch.py",
        "duration": 17,
        "goal": None,
        "check": lambda d: d.get("std_err", 99) < 0.1
    },
}


def _ros_env():
    """Get environment with ROS2 sourced"""
    import subprocess, os
    ros_env = os.environ.copy()
    for line in subprocess.run(
        ['bash','-c','source /opt/ros/humble/setup.bash && source /home/astesia/drone_sim_ws/install/setup.bash && env'],
        capture_output=True, text=True).stdout.split('\n'):
        if '=' in line:
            k,v = line.split('=',1)
            ros_env[k] = v
    return ros_env

def collect_odom(duration):
    """用 ros2 topic echo 收集 /drone/odom 数据"""
    p = subprocess.Popen(
        ['ros2','topic','echo','/drone/odom','--field','pose.pose.position'],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1, env=_ros_env())
    data = {"x": [], "y": [], "z": [], "t": []}
    t0 = time.time()
    while time.time() - t0 < duration:
        try:
            x = y = z = None; state = 0
            for _ in range(20):
                line = p.stdout.readline()
                if not line: break
                line = line.strip()
                if line.startswith('x:'):
                    try: x = float(line.split(':')[1])
                    except: pass
                elif line.startswith('y:'):
                    try: y = float(line.split(':')[1])
                    except: pass
                elif line.startswith('z:'):
                    try: z = float(line.split(':')[1])
                    except: pass
                if x is not None and y is not None and z is not None:
                    data["x"].append(x); data["y"].append(y); data["z"].append(z)
                    data["t"].append(time.time() - t0)
                    break
        except Exception:
            pass
    p.kill()
    return data

def send_goal(x, y, z):
    subprocess.run(['ros2','topic','pub','--once','/drone/goal','geometry_msgs/msg/PoseStamped',
        f'{{header: {{frame_id: "map"}}, pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}}}'],
        capture_output=True, timeout=5)

def run_square():
    waypoints = [(2,0,1.5),(2,2,1.5),(0,2,1.5),(0,0,1.5)]
    reached = 0
    for wp in waypoints:
        send_goal(*wp)
        time.sleep(5)
        # quick check — last 2s of odom
        data = collect_odom(2)
        if data["x"]:
            ex = abs(data["x"][-1] - wp[0])
            ey = abs(data["y"][-1] - wp[1])
            if math.sqrt(ex*ex + ey*ey) < 0.3:
                reached += 1
    return {"waypoints_reached": reached}

def run_scenario(name, cfg):
    print(f"\n{'='*50}")
    print(f"SCENARIO: {name} — {cfg['desc']}")
    print(f"{'='*50}")
    
    # 启动 sim
    launch_proc = subprocess.Popen(
        ['ros2','launch','drone_bringup', cfg['launch']],
        env=_ros_env(),
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(10)
    
    result = {"scenario": name, "pass": False, "data": {}}
    
    try:
        if name == "square":
            result["data"] = run_square()
        else:
            if cfg["goal"]:
                send_goal(*cfg["goal"])
                time.sleep(3)
            data = collect_odom(cfg["duration"])
            if data["x"]:
                n = len(data["x"])
                # final error
                goal = cfg["goal"] or (0, 0, 1.5)
                final_err = math.sqrt(
                    (data["x"][-1]-goal[0])**2 + 
                    (data["y"][-1]-goal[1])**2 + 
                    (data["z"][-1]-1.5)**2)
                # steady z
                n20 = max(1, n//5)
                z_vals = data["z"][-n20:]
                z_mean = sum(z_vals) / len(z_vals)
                # std err
                errors = [math.sqrt((data["x"][i]-goal[0])**2 + (data["z"][i]-1.5)**2) 
                         for i in range(n)]
                std_err = (sum((e - sum(errors)/len(errors))**2 for e in errors) / len(errors))**0.5
                result["data"] = {
                    "samples": n, "final_err": final_err,
                    "z_mean": z_mean, "std_err": std_err,
                }
        
        result["pass"] = cfg["check"](result["data"])
        status = "✅ PASS" if result["pass"] else "❌ FAIL"
        print(f"  {status}")
        for k, v in result["data"].items():
            if isinstance(v, float):
                print(f"    {k}: {v:.4f}")
            else:
                print(f"    {k}: {v}")
    finally:
        launch_proc.terminate()
        time.sleep(1)
        launch_proc.kill()
        # cleanup zombies
        for p in ['dynamics_node','controller_node','map_node','planner_node','rviz2']:
            subprocess.run(['pkill','-9','-f',p], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=_ros_env())
    
    return result

def main():
    # 确保环境正确
    if 'CONDA_PREFIX' in os.environ:
        print("WARNING: conda environment detected — unset CONDA_PREFIX or use /usr/bin/python3")
    
    results = []
    total_pass = 0
    for name, cfg in SCENARIOS.items():
        try:
            r = run_scenario(name, cfg)
            results.append(r)
            if r["pass"]: total_pass += 1
        except Exception as e:
            print(f"  ❌ ERROR: {e}")
            results.append({"scenario": name, "pass": False, "error": str(e)})
    
    # 输出报告
    report = {
        "summary": f"{total_pass}/{len(SCENARIOS)} passed",
        "scenarios": results,
    }
    with open("eval_report.json", "w") as f:
        json.dump(report, f, indent=2)
    
    print(f"\n{'='*50}")
    print(f"FINAL: {total_pass}/{len(SCENARIOS)} passed")
    print(f"Report: eval_report.json")
    return 0 if total_pass == len(SCENARIOS) else 1

if __name__ == '__main__':
    sys.exit(main())
