#!/usr/bin/env python3
"""LADRC auto-tuner via Optuna Bayesian optimization.  pip install optuna numpy"""
import subprocess, sys, time, math, random, argparse

parser = argparse.ArgumentParser()
parser.add_argument('--trials', type=int, default=50)
parser.add_argument('--mode', default='ladrc_z', choices=['ladrc_z','ladrc_xy','ladrc_full'])
args = parser.parse_args()

def sh(cmd, timeout=10):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)

def pset(k, v): sh(f'ros2 param set /drone_controller {k} {v}', timeout=5)
def goal(x,y,z): sh(f'ros2 topic pub --once /drone/goal geometry_msgs/msg/PoseStamped "{{header: {{frame_id: \\"map\\"}}, pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}}}"', timeout=5)
def odom_z():
    r = sh('timeout 2 ros2 topic echo /drone/odom --once --field pose.pose.position', timeout=5)
    for l in r.stdout.split('\n'):
        l=l.strip()
        if l.startswith('z:'): return float(l.split(':')[1])
    return None

def step_response(tz, dur):
    goal(0,0,tz)
    traj, t0 = [], time.time()
    while time.time()-t0 < dur:
        z=odom_z()
        if z is not None: traj.append(z)
        time.sleep(0.05)
    return traj

def cost(traj, target):
    if len(traj)<10: return 999
    e2sum, os_sum = 0.0, 0.0
    for i,z in enumerate(traj):
        e=z-target; e2sum+=e*e
        if i>0 and abs(traj[i]-target)>abs(traj[i-1]-target): os_sum+=abs(e)
    return math.sqrt(e2sum/len(traj)) + 0.5*os_sum/len(traj)

pset('control_mode', 'ladrc')

try:
    import optuna; USE_OPTUNA = True
except ImportError:
    USE_OPTUNA = False; print("[warn] optuna not found, using random search")

# -- Z axis --
if args.mode in ('ladrc_z','ladrc_full'):
    print(f"\n=== LADRC Z tuning ({args.trials} trials) ===")
    if USE_OPTUNA:
        def obj_z(trial):
            b0=trial.suggest_float('b0_z',0.5,3.0); wc=trial.suggest_float('wc_z',1.0,10.0); wo=trial.suggest_float('wo_z',3.0,50.0)
            pset('ladrc_b0',f'[1.0,1.0,{b0:.4f}]'); pset('ladrc_wc',f'[2.0,2.0,{wc:.4f}]'); pset('ladrc_wo',f'[10.0,10.0,{wo:.4f}]')
            time.sleep(0.3)
            c=(cost(step_response(1.5,3.0),1.5)+cost(step_response(0.0,3.0),0.0))*0.5
            print(f"  trial={trial.number:3d} b0={b0:.3f} wc={wc:.1f} wo={wo:.1f} cost={c:.4f}")
            return c
        study=optuna.create_study(direction='minimize',sampler=optuna.samplers.TPESampler(seed=42))
        study.optimize(obj_z,n_trials=args.trials)
        b=study.best_params
        print(f"\n=== BEST Z: b0={b['b0_z']:.4f} wc={b['wc_z']:.1f} wo={b['wo_z']:.2f} ===")
    else:
        best_c,best=1e9,(1.0,3.0,15.0)
        for i in range(args.trials):
            b0=0.5+random.random()*2.5; wc=1.0+random.random()*9.0; wo=3.0+random.random()*47.0
            pset('ladrc_b0',f'[1.0,1.0,{b0:.4f}]'); pset('ladrc_wc',f'[2.0,2.0,{wc:.4f}]'); pset('ladrc_wo',f'[10.0,10.0,{wo:.4f}]')
            time.sleep(0.3); c=cost(step_response(1.5,3.0),1.5)
            print(f"  trial={i:3d} b0={b0:.3f} wc={wc:.1f} wo={wo:.1f} cost={c:.4f}")
            if c<best_c: best_c,best=c,(b0,wc,wo)
        print(f"\n=== BEST Z: b0={best[0]:.4f} wc={best[1]:.1f} wo={best[2]:.2f} ===")

print("\nDone. Switch back to PD: ros2 param set /drone_controller control_mode pd")
