#!/bin/bash
# record_and_analyze.sh — 同时录制 rosbag + odom CSV 用于后续分析
# 用法: bash record_and_analyze.sh bag_hover 5
#   bag_hover: 输出目录名（将生成 bag_hover/ 和 bag_hover_odom.txt）
#   5: 录制时长 秒
BAGNAME=${1:-bag_experiment}
DURATION=${2:-5}

echo "Recording ${BAGNAME} for ${DURATION}s..."

# 启动 bag 录制
ros2 bag record -o "${BAGNAME}" \
  /drone/odom /drone/imu /drone/motor_rpm_cmd /drone/goal \
  /drone/safe_goal /drone/path /drone/planned_path \
  /map/obstacles /drone/planner_status /tf &>/dev/null &
BAG_PID=$!

# 同步 pipe odom 到文本
timeout ${DURATION} ros2 topic echo /drone/odom --field pose.pose.position \
  > "${BAGNAME}_odom.txt" 2>/dev/null &
ODOM_PID=$!

# 同步 pipe rpm 到文本
timeout ${DURATION} ros2 topic echo /drone/motor_rpm_cmd --field data \
  > "${BAGNAME}_rpm.txt" 2>/dev/null &
RPM_PID=$!

wait $ODOM_PID $RPM_PID 2>/dev/null
kill -INT $BAG_PID 2>/dev/null
sleep 1

echo "Done. Files:"
echo "  ${BAGNAME}/          (rosbag)"
echo "  ${BAGNAME}_odom.txt  (position data)"
echo "  ${BAGNAME}_rpm.txt   (RPM data)"
echo ""
echo "=== Quick summary ==="
python3 -c "
import sys
def load_odom(fn):
    data=[]
    with open(fn) as f:
        for line in f:
            line=line.strip()
            if line.startswith('x:'):
                x=float(line.split(':')[1])
                data.append([x,0,0])
            elif line.startswith('y:'):
                data[-1][1]=float(line.split(':')[1])
            elif line.startswith('z:'):
                data[-1][2]=float(line.split(':')[1])
    return data

data = load_odom('${BAGNAME}_odom.txt')
if len(data) < 10:
    print('Insufficient data')
    sys.exit(1)

# steady state (last 20%)
n20 = max(1, len(data)//5)
last = data[-n20:]
ez = sum(abs(d[2]-1.5) for d in last)/len(last)
print(f'Samples: {len(data)}')
print(f'Steady-state z error (last 20%): {ez:.4f}m')
print(f'Last z: {data[-1][2]:.4f}')
# max overshoot
zmax = max(d[2] for d in data)
overshoot = max(0, zmax - 1.5)
print(f'Max z: {zmax:.4f} (overshoot: {overshoot:.4f}m)')
"
