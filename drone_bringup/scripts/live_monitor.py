#!/usr/bin/python3
"""live_monitor — rclpy 原生订阅版，http://localhost:8765 | Ctrl+C 退出并保存 CSV"""
import sys, os, time, json, math, signal, threading, socket, subprocess
from http.server import HTTPServer, BaseHTTPRequestHandler

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray


# ============================================================================
# 共享状态
# ============================================================================
st = {
    't': [], 'px': [], 'py': [], 'pz': [],
    'vx': [], 'vy': [], 'vz': [],
    'rpm': [[], [], [], []], 'rpm_t': [],
    'goal_x': 0.0, 'goal_y': 0.0, 'goal_z': 1.5,
    'obs': [], 'min_d': [], 'min_d_t': [], 'all_d': [],
    'start': time.time(), 'run': True,
}
lock = threading.Lock()

# ============================================================================
# ROS2 订阅节点
# ============================================================================
class MonitorNode(Node):
    def __init__(self):
        super().__init__('ground_station')

        # /drone/odom — 100Hz
        self.odom_sub = self.create_subscription(
            Odometry, '/drone/odom',
            self._on_odom, rclpy.qos.qos_profile_sensor_data)

        # /drone/motor_rpm_cmd — 200Hz
        self.rpm_sub = self.create_subscription(
            Float32MultiArray, '/drone/motor_rpm_cmd',
            self._on_rpm, rclpy.qos.qos_profile_sensor_data)

        # /drone/goal — low rate (topic pub --once or rviz click)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/drone/goal',
            self._on_goal, 10)

        # /map/obstacles — 1Hz
        self.obs_sub = self.create_subscription(
            MarkerArray, '/map/obstacles',
            self._on_obs, 10)

        # 最小障碍物距离计算定时器 — 10Hz
        self.min_d_timer = self.create_timer(0.1, self._calc_min_d)

        # 发布者 — 地面站下发目标点
        self.goal_pub = self.create_publisher(PoseStamped, '/drone/goal', 10)

        self.get_logger().info('ground station ready — http://localhost:8765')

    def _on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        now = time.time() - st['start']
        with lock:
            st['px'].append(x)
            st['py'].append(y)
            st['pz'].append(z)
            st['t'].append(now)
            st['vx'].append(vx)
            st['vy'].append(vy)
            st['vz'].append(vz)

    def _on_rpm(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            return
        now = time.time() - st['start']
        with lock:
            st['rpm_t'].append(now)
            for i in range(4):
                st['rpm'][i].append(float(msg.data[i]))

    def _on_goal(self, msg: PoseStamped):
        z = msg.pose.position.z
        if z < 0.5:
            z = 1.5
        with lock:
            st['goal_x'] = msg.pose.position.x
            st['goal_y'] = msg.pose.position.y
            st['goal_z'] = z

    def _on_obs(self, msg: MarkerArray):
        markers = []
        for m in msg.markers:
            if m.action == m.DELETE or m.ns != 'obstacles':
                continue
            # XY 平面外接圆半径：仅取 scale.x/scale.y 最大值*0.5
            # 不用 scale.z（对圆柱体是高度，远大于半径，会导致 XY 图暴大）
            r = max(abs(m.scale.x), abs(m.scale.y)) * 0.5
            markers.append((m.pose.position.x, m.pose.position.y,
                            m.pose.position.z, r))
        with lock:
            st['obs'] = markers

    def _calc_min_d(self):
        with lock:
            if not st['px']:
                return
            obs = list(st['obs'])
            px, py, pz = st['px'][-1], st['py'][-1], st['pz'][-1]

        if not obs:
            return

        all_d = []
        for (ox, oy, oz, or_) in obs:
            d = math.sqrt((px - ox)**2 + (py - oy)**2 + (pz - oz)**2) - or_
            all_d.append(max(0.0, d))

        md = min(all_d)
        now = time.time() - st['start']
        with lock:
            st['all_d'] = all_d
            st['min_d'].append(md)
            st['min_d_t'].append(now)

    # -------- 目标点下发（供 HTTP handler 调用）--------
    def publish_goal(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        self.get_logger().info(f'goal sent: ({x:.1f}, {y:.1f}, {z:.1f})')


# ============================================================================
# HTTP 服务器 + 内嵌地面站 HTML
# ============================================================================
HTML = r'''<!DOCTYPE html><html lang="zh"><head><meta charset="utf-8"><title>Drone Ground Station</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font:13px "Microsoft YaHei","PingFang SC",sans-serif;background:#0d1117;color:#c9d1d9;padding:10px}
h1{font-size:1.2em;color:#58a6ff;margin-bottom:4px}
.grid{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin-bottom:8px}
.card{background:#161b22;border:1px solid #30363d;border-radius:6px;padding:10px}
.card .lb{font-size:0.72em;color:#8b949e;margin-bottom:2px}
.card .vl{font-size:1.1em;font-weight:bold}
.g{color:#3fb950}.y{color:#d29922}.r{color:#f85149}.c{color:#58a6ff}
h2{font-size:0.9em;color:#8b949e;margin:8px 0 2px}
canvas{border:1px solid #30363d;border-radius:6px;background:#161b22;margin-bottom:4px;display:block}
.main{display:flex;gap:8px}.left{flex:1;min-width:0}.right{width:280px;flex-shrink:0}
.panel{background:#161b22;border:1px solid #30363d;border-radius:6px;padding:10px;margin-bottom:6px}
.panel h3{font-size:0.82em;color:#c9d1d9;margin-bottom:6px;padding-bottom:3px;border-bottom:1px solid #30363d}
input,select,button{font-size:12px;background:#0d1117;color:#c9d1d9;border:1px solid #30363d;border-radius:4px;padding:5px 7px}
input[type=number]{width:65px}input[type=range]{width:100%;margin:3px 0;accent-color:#58a6ff}
button{cursor:pointer}button:hover{background:#21262d}
.btn-go{background:#238636;border-color:#238636;color:#fff;font-weight:bold;width:100%;padding:8px}
.btn-go:hover{background:#2ea043}
.btn-stop{background:#da3633;border-color:#da3633;color:#fff;font-weight:bold;width:100%;padding:8px}
.btn-stop:hover{background:#f85149}
.btn-preset{width:48%;margin:1px 0;font-size:11px}
.row{display:flex;align-items:center;justify-content:space-between;margin:2px 0}
.row label{font-size:0.72em;color:#8b949e;width:55px;flex-shrink:0}
.row .val{font-size:0.72em;color:#c9d1d9;width:32px;text-align:right}
.divider{border-top:1px solid #30363d;margin:6px 0}
.fb{font-size:0.7em;color:#d29922;margin-top:3px;min-height:1em}
</style></head><body>
<h1>&#x1f681; Drone Ground Station</h1>
<div class="main">
<div class="left">
<div class="grid">
<div class="card"><div class="lb">Pos (xyz m)</div><div class="vl c" id="p">-</div></div>
<div class="card"><div class="lb">3D Err</div><div class="vl y" id="pe">-</div></div>
<div class="card"><div class="lb">Steady z Err</div><div class="vl" id="se">-</div></div>
<div class="card"><div class="lb">Vel (xyz)</div><div class="vl c" id="ve">-</div></div>
<div class="card"><div class="lb">RPM</div><div class="vl c" id="rp">-</div></div>
<div class="card"><div class="lb">MinObsDist</div><div class="vl" id="od">-</div></div>
<div class="card"><div class="lb">Overshoot Z</div><div class="vl y" id="os">-</div></div>
<div class="card"><div class="lb">Path/Time</div><div class="vl c" id="pt">-</div></div>
<div class="card"><div class="lb">RPM Sat</div><div class="vl" id="rs">-</div></div>
<div class="card"><div class="lb">Att Div</div><div class="vl" id="ad">-</div></div>
<div class="card"><div class="lb">Goal</div><div class="vl c" id="go">-</div></div></div>
<h2>1. Position Error (ref=0.3m)</h2><canvas id="cpe"></canvas>
<h2>2. Motor RPM (autoscaled)</h2><canvas id="crp"></canvas>
<h2>3. XY Trajectory</h2><canvas id="ctj"></canvas>
<h2>4. Obstacle Distances (ref=0.4m)</h2><canvas id="cmd"></canvas>
</div>
<div class="right">
<div class="panel"><h3>&#x1f3af; Send Goal</h3>
<div class="row"><label>x (m)</label><input type="number" id="gx" value="2.0" step="0.1"></div>
<div class="row"><label>y (m)</label><input type="number" id="gy" value="1.0" step="0.1"></div>
<div class="row"><label>z (m)</label><input type="number" id="gz" value="1.5" step="0.1"></div>
<button class="btn-go" onclick="sendGoal()">&#x1f680; Send Goal</button>
<div class="fb" id="goal-fb"></div>
<div class="divider"></div>
<button class="btn-preset" onclick="preset('home')">&#x1f3e0; Home</button>
<button class="btn-preset" onclick="preset('targetA')">&#x1f3af; (2,1,1.5)</button>
<button class="btn-preset" onclick="preset('square')">&#x25a1; (2,2,1.5)</button>
<button class="btn-preset" onclick="preset('up')">&#x2b06; (0,0,3)</button></div>
<div class="panel"><h3>&#x2699; Controller Tuning</h3>
<div id="sliders"></div>
<button onclick="applyParams()" style="width:100%;margin-top:4px;background:#1f6feb;color:#fff;font-weight:bold">Apply All</button>
<div class="fb" id="param-fb"></div></div>
<div class="panel"><h3>&#x1f32c; Wind Disturbance</h3>
<div class="row"><label>Enabled</label><input type="checkbox" id="wind_enabled" onchange="setWind()"></div>
<div class="row"><label>Force X (N)</label><input type="number" id="wind_fx" value="2.0" step="0.5" onchange="setWind()"></div>
<div class="row"><label>Force Y (N)</label><input type="number" id="wind_fy" value="0.0" step="0.5" onchange="setWind()"></div>
<div class="row"><label>Gust (N)</label><input type="number" id="wind_gust" value="0.5" step="0.1" onchange="setWind()"></div>
<div class="divider"></div>
<button class="btn-preset" onclick="windPreset('none')">&#x1f6ab; No Wind</button>
<button class="btn-preset" onclick="windPreset('light')">&#x1f4a8; Light</button>
<button class="btn-preset" onclick="windPreset('medium')">&#x1f32c; Medium</button>
<button class="btn-preset" onclick="windPreset('strong')">&#x1f32a; Strong</button>
<div class="fb" id="wind-fb"></div></div>
<div class="panel"><h3>&#x1f4e1; Sensor Noise</h3>
<div class="row"><label>IMU Noise</label><input type="checkbox" id="imu_noise" onchange="setNoise()"></div>
<div class="row"><label>Accel &sigma;</label><input type="number" id="accel_nd" value="0.05" step="0.01" onchange="setNoise()"></div>
<div class="row"><label>Gyro &sigma;</label><input type="number" id="gyro_nd" value="0.01" step="0.005" onchange="setNoise()"></div>
<div class="row"><label>Accel Bias</label><input type="number" id="accel_bias" value="0.1" step="0.05" onchange="setNoise()"></div>
<div class="row"><label>Odom Pos &sigma;</label><input type="number" id="odom_pos" value="0.02" step="0.01" onchange="setNoise()"></div>
<div class="row"><label>Odom Vel &sigma;</label><input type="number" id="odom_vel" value="0.01" step="0.01" onchange="setNoise()"></div>
<div class="divider"></div>
<button class="btn-preset" onclick="noisePreset('off')">&#x1f6ab; Off</button>
<button class="btn-preset" onclick="noisePreset('light')">&#x1f4e1; Light</button>
<button class="btn-preset" onclick="noisePreset('realistic')">&#x1f4e1; Realistic</button>
<div class="fb" id="noise-fb"></div></div>
<div class="panel"><h3>&#x1f6d1; Emergency</h3>
<button class="btn-stop" onclick="emergency('stop')">&#x1f6d1; STOP MOTORS</button>
<button onclick="emergency('hover')" style="width:100%;margin-top:4px;background:#d29922;color:#000;font-weight:bold">&#x1f3e0; Return to Origin</button>
<div class="fb" id="emerg-fb"></div></div>
</div></div>
<script>
var CL=['#3fb950','#58a6ff','#d29922','#c9d1d9'];
var HOV_Z=1.5;

function chart(id,series,ylab,ymin,ymax,ref){
  var c=document.getElementById(id); if(!c)return;
  c.width=Math.max(500,c.parentElement.clientWidth-24);
  c.height=220;
  var w=c.width,h=c.height,ctx=c.getContext('2d');
  ctx.clearRect(0,0,w,h);
  // grid
  ctx.strokeStyle='#21262d';ctx.lineWidth=1;
  for(var i=0;i<=4;i++){var y=h*i/4;ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(w,y);ctx.stroke()}
  for(var i=0;i<=10;i++){var x=w*i/10;ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,h);ctx.stroke()}
  // y-axis labels
  ctx.fillStyle='#8b949e';ctx.font='10px sans-serif';
  for(var i=0;i<=4;i++){ctx.fillText((ymax-(ymax-ymin)*i/4).toFixed(1),2,h*i/4+10)}
  ctx.fillStyle='#58a6ff';ctx.fillText(ylab,4,12);
  // reference line
  if(ref!==undefined&&ref!==null){
    ctx.save();ctx.setLineDash([4,6]);ctx.strokeStyle='#f85149';ctx.lineWidth=1.5;
    var ry=h-(ref-ymin)/(ymax-ymin)*h;ctx.beginPath();ctx.moveTo(0,ry);ctx.lineTo(w,ry);ctx.stroke();
    ctx.restore();ctx.fillStyle='#f85149';ctx.font='10px sans-serif';ctx.fillText(ref.toFixed(2),4,ry-4);
  }
  // data lines
  if(!series||!series[0]||!series[0].length){
    ctx.fillStyle='#8b949e';ctx.font='13px sans-serif';ctx.fillText('waiting...',w/2-30,h/2);
  } else {
    for(var si=0;si<series.length;si++){
      ctx.strokeStyle=CL[si%4];ctx.lineWidth=1.6;ctx.beginPath();
      var s=series[si];
      for(var i=0;i<s.length;i++){
        var x=i/s.length*w,y=h-(s[i]-ymin)/(ymax-ymin)*h;
        if(i==0)ctx.moveTo(x,y);else ctx.lineTo(x,y);
      }
      ctx.stroke();
    }
  }
}

var PRESETS={home:{x:0,y:0,z:1.5},targetA:{x:2,y:1,z:1.5},square:{x:2,y:2,z:1.5},up:{x:0,y:0,z:3}};
function preset(n){var p=PRESETS[n];if(!p)return;document.getElementById('gx').value=p.x;document.getElementById('gy').value=p.y;document.getElementById('gz').value=p.z;sendGoal()}
function sendGoal(){var x=parseFloat(document.getElementById('gx').value)||0;var y=parseFloat(document.getElementById('gy').value)||0;var z=parseFloat(document.getElementById('gz').value)||1.5;var fb=document.getElementById('goal-fb');fb.textContent='sending...';fetch('/goal',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({x:x,y:y,z:z})}).then(function(r){return r.json()}).then(function(j){fb.textContent=j.ok?'sent ('+x.toFixed(1)+','+y.toFixed(1)+','+z.toFixed(1)+')':'FAIL: '+j.error;setTimeout(function(){fb.textContent=''},3000)}).catch(function(e){fb.textContent='ERROR: '+e})}
function emergency(a){var fb=document.getElementById('emerg-fb');fb.textContent='sending...';fetch('/emergency',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:a})}).then(function(r){return r.json()}).then(function(j){fb.textContent=j.ok?'done: '+a:'FAIL: '+j.error;setTimeout(function(){fb.textContent=''},3000)}).catch(function(e){fb.textContent='ERROR: '+e})}
var PARAMS={'Kp_pos.x':{min:0.1,max:10,step:0.1,def:2.0},'Kp_pos.z':{min:0.5,max:15,step:0.1,def:3.0},'Kd_pos.x':{min:0,max:6,step:0.1,def:2.0},'Kd_pos.z':{min:0,max:8,step:0.1,def:2.4},'Kp_att.x':{min:1,max:30,step:0.5,def:8},'Kd_rate.x':{min:0.1,max:5,step:0.1,def:0.8},'a_xy_max':{min:1,max:12,step:0.5,def:4.0},'a_z_max':{min:1,max:15,step:0.5,def:6.0}};
(function(){var h='';for(var k in PARAMS){var p=PARAMS[k];var id='sl_'+k.replace(/\./g,'_');h+='<div class=row><label>'+k+'</label><span class=val id=sv_'+k.replace(/\./g,'_')+'>'+p.def.toFixed(1)+'</span></div>';h+='<input type=range id='+id+' min='+p.min+' max='+p.max+' step='+p.step+' value='+p.def+' oninput="var e=document.getElementById(\'sv_'+k.replace(/\./g,'_')+'\');var s=document.getElementById(\''+id+'\');if(e&&s)e.textContent=parseFloat(s.value).toFixed(1)">'}document.getElementById('sliders').innerHTML=h})();
function applyParams(){var fb=document.getElementById('param-fb');var jobs=[];for(var k in PARAMS){var el=document.getElementById('sl_'+k.replace(/\./g,'_'));if(el)jobs.push({node:'/drone_controller',param:k,value:parseFloat(el.value)})}if(!jobs.length)return;fb.textContent='applying '+jobs.length+' params...';var i=0;function nxt(){if(i>=jobs.length){fb.textContent='done!';setTimeout(function(){fb.textContent=''},3000);return}var p=jobs[i];fetch('/param',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(p)}).then(function(r){return r.json()}).then(function(j){if(!j.ok)fb.textContent='FAIL: '+p.param;i++;nxt()}).catch(function(e){fb.textContent='ERROR: '+e})}nxt()}

function windPreset(p){
  var el=document.getElementById('wind_enabled'), fx=document.getElementById('wind_fx'), fy=document.getElementById('wind_fy'), g=document.getElementById('wind_gust');
  if(p=='none'){el.checked=false;fx.value='0.0';fy.value='0.0';g.value='0.0'}
  else if(p=='light'){el.checked=true;fx.value='0.5';fy.value='0.0';g.value='0.2'}
  else if(p=='medium'){el.checked=true;fx.value='2.0';fy.value='0.0';g.value='1.0'}
  else if(p=='strong'){el.checked=true;fx.value='4.0';fy.value='0.0';g.value='2.0'}
  setWind();
}
function setWind(){
  var en=document.getElementById('wind_enabled').checked;
  var fx=parseFloat(document.getElementById('wind_fx').value)||0;
  var fy=parseFloat(document.getElementById('wind_fy').value)||0;
  var gust=parseFloat(document.getElementById('wind_gust').value)||0;
  var fb=document.getElementById('wind-fb');
  applyParam('/drone_dynamics','wind_enabled',en);
  applyParam('/drone_dynamics','wind_force','['+fx.toFixed(1)+','+fy.toFixed(1)+',0.0]');
  applyParam('/drone_dynamics','wind_gust_amplitude',gust);
  fb.textContent=en?('wind '+fx.toFixed(1)+','+fy.toFixed(1)+'N'):'off';
  setTimeout(function(){fb.textContent=''},3000);
}
function noisePreset(p){
  var el=document.getElementById('imu_noise'), an=document.getElementById('accel_nd'), gn=document.getElementById('gyro_nd'), ab=document.getElementById('accel_bias'), op=document.getElementById('odom_pos'), ov=document.getElementById('odom_vel');
  if(p=='off'){el.checked=false;an.value='0.00';gn.value='0.000';ab.value='0.00';op.value='0.00';ov.value='0.00'}
  else if(p=='light'){el.checked=true;an.value='0.02';gn.value='0.005';ab.value='0.05';op.value='0.01';ov.value='0.005'}
  else if(p=='realistic'){el.checked=true;an.value='0.08';gn.value='0.015';ab.value='0.15';op.value='0.03';ov.value='0.02'}
  setNoise();
}
function setNoise(){
  var en=document.getElementById('imu_noise').checked;
  var fb=document.getElementById('noise-fb');
  applyParam('/drone_dynamics','imu_noise_enabled',en);
  if(en){
    applyParam('/drone_dynamics','accel_noise_density',parseFloat(document.getElementById('accel_nd').value));
    applyParam('/drone_dynamics','gyro_noise_density',parseFloat(document.getElementById('gyro_nd').value));
    applyParam('/drone_dynamics','accel_bias_init',parseFloat(document.getElementById('accel_bias').value));
    applyParam('/drone_dynamics','odom_pos_noise',parseFloat(document.getElementById('odom_pos').value));
    applyParam('/drone_dynamics','odom_vel_noise',parseFloat(document.getElementById('odom_vel').value));
  }
  fb.textContent=en?'noise ON':'noise OFF';
  setTimeout(function(){fb.textContent=''},3000);
}
var _paramQueue=[],_paramTimer=null;
function applyParam(node,param,value){
  _paramQueue.push({node:node,param:param,value:value});
  if(!_paramTimer)_paramTimer=setTimeout(_flushParams,100);
}
function _flushParams(){
  _paramTimer=null;if(!_paramQueue.length)return;
  var batch=_paramQueue.splice(0,_paramQueue.length);
  function next(){if(!batch.length)return;var p=batch.shift();
  fetch('/param',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(p)}).then(function(){next()}).catch(function(){next()});}
  next();
}

function update(){
  fetch('/data').then(function(r){return r.json()}).then(function(d){
    if(!d.t||!d.t.length)return;
    var n=d.t.length; HOV_Z=d.goal_z||1.5;
    var px=d.px[n-1],py=d.py[n-1],pz=d.pz[n-1];

    // status cards
    document.getElementById('p').textContent=px.toFixed(2)+' '+py.toFixed(2)+' '+pz.toFixed(2);
    var e3=Math.sqrt(Math.pow(px-d.goal_x,2)+Math.pow(py-d.goal_y,2)+Math.pow(pz-HOV_Z,2));
    document.getElementById('pe').textContent=e3.toFixed(4)+' m';
    document.getElementById('pe').className='vl '+(e3<0.3?'g':'y');

    var n20=Math.max(1,Math.floor(n/5)),se=0;
    for(var i=n-n20;i<n;i++) se+=Math.abs(d.pz[i]-HOV_Z);
    document.getElementById('se').textContent=(se/n20).toFixed(4)+' m';
    document.getElementById('se').className='vl '+((se/n20)<0.3?'g':'r');

    var vi=Math.min(d.vx.length,d.vy.length,d.vz.length)-1;
    if(vi>=0) document.getElementById('ve').textContent=d.vx[vi].toFixed(2)+' '+d.vy[vi].toFixed(2)+' '+d.vz[vi].toFixed(2);

    var ri=d.rpm[0].length-1;
    if(ri>=0) document.getElementById('rp').textContent=d.rpm[0][ri].toFixed(0)+' '+d.rpm[1][ri].toFixed(0)+' '+d.rpm[2][ri].toFixed(0)+' '+d.rpm[3][ri].toFixed(0);

    var mdv=d.min_d.length?d.min_d[d.min_d.length-1]:0;
    document.getElementById('od').textContent=mdv.toFixed(3)+' m';
    document.getElementById('od').className='vl '+(mdv>0.4?'g':(mdv>0.15?'y':'r'));

    var zmax=-Infinity;for(var i=0;i<n;i++) if(d.pz[i]>zmax) zmax=d.pz[i];
    document.getElementById('os').textContent=Math.max(0,zmax-HOV_Z).toFixed(4)+' m';

    var path=0;for(var i=1;i<n;i++) path+=Math.hypot(d.px[i]-d.px[i-1],d.py[i]-d.py[i-1],d.pz[i]-d.pz[i-1]);
    document.getElementById('pt').textContent=path.toFixed(1)+' m / '+(d.t[n-1]).toFixed(1)+' s';

    document.getElementById('go').textContent=d.goal_x.toFixed(1)+' '+d.goal_y.toFixed(1)+' '+HOV_Z.toFixed(1);

    // RPM saturation: only flag if RPM is truly saturated (>= 9990 or motor stopped at 0)
    var sat=false;
    for(var i=0;i<4;i++){
      var rr=d.rpm[i];
      if(rr.length<2) continue;
      for(var j=Math.max(0,rr.length-100);j<rr.length;j++)
        if(rr[j]>=9990){sat=true;break;}
    }
    document.getElementById('rs').textContent=sat?'SATURATED':'OK';
    document.getElementById('rs').className='vl '+(sat?'r':'g');

    var div=false;
    for(var i=0;i<n;i++) if(d.pz[i]<-100||(d.vx[i]&&Math.hypot(d.vx[i]||0,d.vy[i]||0,d.vz[i]||0)>100)) div=true;
    document.getElementById('ad').textContent=div?'DIVERGED':'OK';  document.getElementById('ad').className='vl '+(div?'r':'g');

    // chart 1: position error
    var ex=[],ez=[];
    for(var i=0;i<n;i++){ex.push(Math.abs(d.px[i]-d.goal_x));ez.push(Math.abs(d.pz[i]-HOV_Z))}
    chart('cpe',[ex,ez],'x err=green  z err=blue  ref=0.3m',0,3,0.3);

    // chart 2: RPM — auto-scale y-axis to actual data range
    (function(){
      var allRPM=[];
      for(var i=0;i<4;i++) allRPM=allRPM.concat(d.rpm[i]);
      var rpmMin=allRPM.length?Math.min.apply(null,allRPM):0;
      var rpmMax=allRPM.length?Math.max.apply(null,allRPM):50;
      // Ensure at least 2 units range so flat lines are visible centered
      if(rpmMax-rpmMin<2){rpmMin-=1;rpmMax+=1;}
      rpmMin=Math.max(0,rpmMin);
      chart('crp',[d.rpm[0],d.rpm[1],d.rpm[2],d.rpm[3]],'FL/FR/BL/BR RPM',rpmMin,rpmMax,null);
    })();

    // chart 3: XY trajectory
    (function(){
      var c=document.getElementById('ctj'); if(!c)return;
      c.width=Math.max(500,c.parentElement.clientWidth-24); c.height=220;
      var w=c.width,h=c.height,ctx=c.getContext('2d');
      ctx.clearRect(0,0,w,h);
      ctx.fillStyle='#161b22';ctx.fillRect(0,0,w,h);
      // grid
      ctx.strokeStyle='#21262d';ctx.lineWidth=1;
      for(var i=0;i<=4;i++){ctx.beginPath();ctx.moveTo(0,h*i/4);ctx.lineTo(w,h*i/4);ctx.stroke()}
      for(var i=0;i<=4;i++){ctx.beginPath();ctx.moveTo(w*i/4,0);ctx.lineTo(w*i/4,h);ctx.stroke()}
      // bbox: bounds x/y [-1,4], y axis inverted
      function tx(v){return (v-(-1))/5*w}
      function ty(v){return h-(v-(-1))/5*h}
      // goal
      ctx.fillStyle='#58a6ff';ctx.beginPath();ctx.arc(tx(d.goal_x),ty(d.goal_y),5,0,2*Math.PI);ctx.fill();
      ctx.fillStyle='#58a6ff';ctx.font='10px sans-serif';ctx.fillText('goal',tx(d.goal_x)+8,ty(d.goal_y)-4);
      // obstacles as filled circles
      if(d.all_d&&d.obs){
        var OBS_COL=['#ff6b6b','#ffd93d','#6bcb77','#4d96ff','#ff922b','#845ef7',
                     '#20c997','#f06595','#339af0','#fcc419','#94d82d','#5c7cfa'];
        for(var k=0;k<d.obs.length;k++){
          var o=d.obs[k];
          ctx.fillStyle=OBS_COL[k%12]+'44';
          ctx.beginPath();
          var rx=o[3]/5*w, ry=o[3]/5*h;
          ctx.ellipse(tx(o[0]),ty(o[1]),Math.max(rx,3),Math.max(ry,3),0,0,2*Math.PI);
          ctx.fill();
          ctx.fillStyle=OBS_COL[k%12];ctx.font='9px sans-serif';
          ctx.fillText(k,tx(o[0])+4,ty(o[1])-4);
        }
      }
      // drone path
      ctx.strokeStyle='#3fb950';ctx.lineWidth=2;ctx.beginPath();
      for(var i=0;i<n;i++){var x=tx(d.px[i]),y=ty(d.py[i]); if(i==0)ctx.moveTo(x,y);else ctx.lineTo(x,y);}
      ctx.stroke();
      // start marker
      if(n>0){ctx.fillStyle='#3fb950';ctx.beginPath();ctx.arc(tx(d.px[0]),ty(d.py[0]),4,0,2*Math.PI);ctx.fill();}
      // current position
      if(n>0){ctx.fillStyle='#fff';ctx.beginPath();ctx.arc(tx(px),ty(py),5,0,2*Math.PI);ctx.fill();ctx.fillStyle='#fff';ctx.font='10px sans-serif';ctx.fillText('now',tx(px)+8,ty(py)-4);}
      ctx.fillStyle='#58a6ff';ctx.fillText('XY trajectory  bbox=[-1,4]',4,12);
    })();

    // chart 4: obstacle distances
    (function(){
      var c=document.getElementById('cmd'); if(!c)return;
      c.width=Math.max(500,c.parentElement.clientWidth-24); c.height=220;
      var w=c.width,h=c.height,ctx=c.getContext('2d');
      ctx.clearRect(0,0,w,h);
      ctx.strokeStyle='#21262d';ctx.lineWidth=1;
      for(var i=0;i<=4;i++){ctx.beginPath();ctx.moveTo(0,h*i/4);ctx.lineTo(w,h*i/4);ctx.stroke()}
      // y-axis
      ctx.fillStyle='#8b949e';ctx.font='10px sans-serif';
      for(var i=0;i<=4;i++){ctx.fillText((2-0.5*i).toFixed(1),2,h*i/4+10)}
      // safety ref
      ctx.save();ctx.setLineDash([4,6]);ctx.strokeStyle='#f85149';ctx.lineWidth=1.5;
      var ry=h-0.4/2*h;ctx.beginPath();ctx.moveTo(0,ry);ctx.lineTo(w,ry);ctx.stroke();
      ctx.restore();ctx.fillStyle='#f85149';ctx.font='10px sans-serif';ctx.fillText('0.40m',4,ry-4);
      // per-obstacle lines
      var ads=d.all_d||[];
      if(ads.length){
        var OBS_COL=['#ff6b6b','#ffd93d','#6bcb77','#4d96ff','#ff922b','#845ef7',
                     '#20c997','#f06595','#339af0','#fcc419','#94d82d','#5c7cfa'];
        for(var k=0;k<ads.length;k++){
          ctx.strokeStyle=OBS_COL[k%12]+'88';ctx.lineWidth=1.2;ctx.beginPath();
          var y4=h-ads[k]/2*h;
          ctx.moveTo(0,y4);ctx.lineTo(w,y4);ctx.stroke();
          ctx.fillStyle=OBS_COL[k%12];ctx.font='9px sans-serif';ctx.fillText(''+k,2,y4-2);
        }
      }
      // min distance curve
      if(d.min_d.length){
        ctx.strokeStyle='#3fb950';ctx.lineWidth=2.8;ctx.beginPath();
        for(var i=0;i<d.min_d.length;i++){
          var x4=i/d.min_d.length*w,y4=h-d.min_d[i]/2*h;
          i==0?ctx.moveTo(x4,y4):ctx.lineTo(x4,y4);
        }
        ctx.stroke();ctx.fillStyle='#3fb950';ctx.font='10px sans-serif';ctx.fillText('min',w-24,12);
      }
      ctx.fillStyle='#58a6ff';ctx.fillText('obs dist (m)  green thick=min  ref=0.4m',4,12);
    })();
  }).catch(function(e){console.error(e)});
}
setInterval(update,500);
update();
</script></body></html>'''


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html;charset=utf-8')
            self.end_headers()
            self.wfile.write(HTML.encode())
        elif self.path == '/data':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            with lock:
                d = {k: st[k] for k in [
                    't', 'px', 'py', 'pz', 'vx', 'vy', 'vz',
                    'rpm', 'rpm_t', 'goal_x', 'goal_y', 'goal_z',
                    'min_d', 'min_d_t', 'all_d', 'obs',
                ]}
                if not d.get('goal_z'):
                    d['goal_z'] = 1.5
            self.wfile.write(json.dumps(d).encode())
        else:
            self.send_response(404)
            self.end_headers()

    # -------- POST --------
    def do_POST(self):
        length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(length).decode() if length else '{}'
        try:
            data = json.loads(body)
        except json.JSONDecodeError:
            self._json_resp(400, {'ok': False, 'error': 'invalid JSON'})
            return
        if self.path == '/goal':
            self._handle_goal(data)
        elif self.path == '/emergency':
            self._handle_emergency(data)
        elif self.path == '/param':
            self._handle_param(data)
        else:
            self._json_resp(404, {'ok': False, 'error': 'unknown endpoint'})

    def _json_resp(self, code, obj):
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(obj).encode())

    def _handle_goal(self, data):
        x = float(data.get('x', 0)); y = float(data.get('y', 0))
        z = float(data.get('z', 1.5))
        if z < 0.5: z = 1.5
        try:
            if Handler.node:
                Handler.node.publish_goal(x, y, z)
            self._json_resp(200, {'ok': True})
        except Exception as e:
            self._json_resp(500, {'ok': False, 'error': str(e)})

    def _handle_emergency(self, data):
        action = data.get('action', '')
        try:
            if action == 'stop':
                subprocess.run(
                    ['ros2','topic','pub','--once','/drone/motor_rpm_cmd',
                     'std_msgs/msg/Float32MultiArray','{data: [0,0,0,0]}'],
                    capture_output=True, timeout=3)
            elif action == 'hover':
                if Handler.node:
                    Handler.node.publish_goal(0, 0, 1.5)
            else:
                self._json_resp(400, {'ok': False, 'error': f'unknown action: {action}'})
                return
            self._json_resp(200, {'ok': True})
        except Exception as e:
            self._json_resp(500, {'ok': False, 'error': str(e)})

    def _handle_param(self, data):
        node_name = data.get('node', '/drone_controller')
        param = data.get('param', '')
        value = data.get('value')
        if not param or value is None:
            self._json_resp(400, {'ok': False, 'error': 'missing param or value'})
            return
        try:
            val_str = 'true' if isinstance(value, bool) and value else ('false' if isinstance(value, bool) else str(value))
            r = subprocess.run(
                ['ros2','param','set',node_name, param, val_str],
                capture_output=True, text=True, timeout=5)
            ok = r.returncode == 0
            self._json_resp(200 if ok else 500, {
                'ok': ok, 'param': param, 'value': value,
                'stdout': r.stdout.strip(),
                'stderr': r.stderr.strip() if not ok else '',
            })
        except Exception as e:
            self._json_resp(500, {'ok': False, 'error': str(e)})

    def log_message(self, *args):
        pass


# ============================================================================
# main
# ============================================================================
def save_csv():
    """退出时保存 CSV"""
    csv_path = 'dashboard_log.csv'
    with lock:
        n = max(len(st['t']), len(st['rpm_t']), len(st['min_d_t']))
        lines = ['t,px,py,pz,vx,vy,vz,rpm0,rpm1,rpm2,rpm3,min_obs_dist']
        for i in range(n):
            row = []
            for k in ['t', 'px', 'py', 'pz']:
                row.append(str(st[k][i]) if i < len(st[k]) else '')
            row.append(str(st['vx'][i]) if i < len(st['vx']) else '')
            row.append(str(st['vy'][i]) if i < len(st['vy']) else '')
            row.append(str(st['vz'][i]) if i < len(st['vz']) else '')
            for j in range(4):
                row.append(str(st['rpm'][j][i]) if i < len(st['rpm'][j]) else '')
            row.append(str(st['min_d'][i]) if i < len(st['min_d']) else '')
            lines.append(','.join(row))
    with open(csv_path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print(f'\nSaved {csv_path} ({n} rows)')


def _kill_port(port):
    try:
        subprocess.run(['fuser', '-k', f'{port}/tcp'],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass


def main():
    for _ in range(3):
        _kill_port(8765)
        time.sleep(0.2)
    time.sleep(0.3)

    rclpy.init(args=sys.argv)
    node = MonitorNode()
    Handler.node = node
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    print('Ground Station: http://localhost:8765  |  Ctrl+C to exit & save CSV')

    # HTTP 服务器
    try:
        for retry in range(10):
            try:
                HTTPServer.allow_reuse_address = True
                srv = HTTPServer(('0.0.0.0', 8765), Handler)
                srv.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                srv.serve_forever()
                break
            except OSError:
                print(f'Port 8765 busy (retry {retry+1}/10)...')
                _kill_port(8765)
                time.sleep(0.5)
        else:
            print('ERROR: cannot bind port 8765 after 10 retries')
    except KeyboardInterrupt:
        pass
    finally:
        executor.cancel()
        save_csv()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
