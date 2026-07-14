# 重写 live_monitor.py 的数据采集部分——基于实际 ros2 topic echo 输出格式
import subprocess, threading, time, json, math, os, sys, socket, re, ast
from http.server import HTTPServer, BaseHTTPRequestHandler

# ====== 全局状态 ======
state = {
    't': [], 'px': [], 'py': [], 'pz': [],
    'vx': [], 'vy': [], 'vz': [],
    'rpm': [[],[],[],[]], 'rpm_t': [],
    'goal_x': 0.0, 'goal_y': 0.0, 'goal_z': 1.5,
    'obstacles': [], 'min_obs_dist': [], 'min_obs_t': [],
    'start_time': time.time(), 'running': True,
}
lock = threading.Lock()
SAVE_FILE = "dashboard_log.csv"

def sh(cmd, timeout=5):
    return subprocess.run(['/bin/bash','-c',cmd], capture_output=True, text=True, timeout=timeout)

# ====== 数据采集 ======
def read_odom():
    """采集 odom position + linear velocity"""
    proc = subprocess.Popen(
        ['ros2','topic','echo','/drone/odom'],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
    px=py=pz=None
    in_position=False  # 严格追踪是否在 position: 块内（避免 orientation.x 干扰）
    for line in proc.stdout:
        line=line.strip()
        if line.startswith('position:'): in_position=True; continue
        if line.startswith('orientation:'): in_position=False; continue
        if line.startswith('twist:'): in_position=False; continue
        if line.startswith('covariance:'): in_position=False; continue
        if line == '---': in_position=False; continue
        if in_position:
            if line.startswith('x:'):
                try: px=float(line.split(':')[1].strip())
                except: pass
            elif line.startswith('y:'):
                try: py=float(line.split(':')[1].strip())
                except: pass
            elif line.startswith('z:'):
                try: pz=float(line.split(':')[1].strip())
                except: pass
                if px is not None and py is not None:
                    with lock:
                        state['px'].append(px); state['py'].append(py); state['pz'].append(pz)
                        state['t'].append(time.time()-state['start_time'])
                px=py=pz=None

def read_odom_velocity():
    """单独采集 velocity"""
    proc = subprocess.Popen(
        ['ros2','topic','echo','/drone/odom','--field','twist.twist.linear'],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
    vx=vy=vz=None
    for line in proc.stdout:
        line=line.strip()
        if line.startswith('x:'):
            vx=float(line.split(':')[1].strip())
        elif line.startswith('y:'):
            vy=float(line.split(':')[1].strip())
        elif line.startswith('z:'):
            vz=float(line.split(':')[1].strip())
            if vx is not None and vy is not None:
                with lock:
                    # 确保和 position 数组对齐
                    state['vx'].append(vx); state['vy'].append(vy); state['vz'].append(vz)
                    # 截断到与 position 同等长度
                    if len(state['vx'])>len(state['px']): state['vx'].pop(); state['vy'].pop(); state['vz'].pop()
            vx=vy=vz=None

def read_rpm():
    """RPM: 输出格式为 array('f', [r0,r1,r2,r3])"""
    proc = subprocess.Popen(
        ['ros2','topic','echo','/drone/motor_rpm_cmd','--field','data'],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
    for line in proc.stdout:
        line=line.strip()
        if not line.startswith('array'): continue
        try:
            # 解析 array('f', [r0, r1, r2, r3])
            match=re.search(r'\[(.*?)\]',line)
            if match:
                vals=[float(x.strip()) for x in match.group(1).split(',')]
                if len(vals)==4:
                    with lock:
                        for i in range(4): state['rpm'][i].append(vals[i])
                        state['rpm_t'].append(time.time()-state['start_time'])
        except: pass

def read_goal():
    proc = subprocess.Popen(
        ['ros2','topic','echo','/drone/goal','--field','pose.position'],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
    gx=gy=gz=None
    for line in proc.stdout:
        line=line.strip()
        if line.startswith('x:'): gx=float(line.split(':')[1].strip())
        elif line.startswith('y:'): gy=float(line.split(':')[1].strip())
        elif line.startswith('z:'):
            gz=float(line.split(':')[1].strip())
            if gx is not None and gy is not None:
                with lock: state['goal_x']=gx; state['goal_y']=gy; state['goal_z']=gz
            gx=gy=gz=None

def read_obstacles():
    """障碍物 MarkerArray — 取每个 marker 的 pose.position + scale"""
    proc = subprocess.Popen(
        ['ros2','topic','echo','/map/obstacles'],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
    markers=[]
    cur_cx=cur_cy=cur_cz=cur_r=None
    in_pose=False
    pose_done=0  # 0=wait pos, 1=pos done, 2=wait scale
    for line in proc.stdout:
        line=line.strip()
        if 'position:' in line: in_pose=True; pose_done=0; continue
        if 'scale:' in line: in_pose=False; pose_done=0; continue
        if 'ns:' in line: in_pose=False
        if '---' in line:
            if cur_cx is not None and cur_r is not None:
                markers.append((cur_cx,cur_cy,cur_cz,cur_r))
            if markers:
                with lock: state['obstacles']=markers
            markers=[]; cur_cx=cur_cy=cur_cz=cur_r=None; continue
        if in_pose:
            if line.startswith('x:'): cur_cx=float(line.split(':')[1].strip())
            elif line.startswith('y:'): cur_cy=float(line.split(':')[1].strip())
            elif line.startswith('z:'): cur_cz=float(line.split(':')[1].strip()); in_pose=False
        else:
            if 'x:' in line and cur_r is None:
                try: cur_r=float(line.split(':')[1].strip())
                except: pass

def compute_min_obs_dist():
    while state['running']:
        time.sleep(0.2)
        with lock:
            if not state['px']: continue
            px=state['px'][-1]; py=state['py'][-1]; pz=state['pz'][-1]
            obs=state['obstacles']
        if not obs: continue
        min_d=1e9
        for o in obs:
            dx=px-o[0]; dy=py-o[1]; dz=pz-o[2]; d=math.sqrt(dx*dx+dy*dy+dz*dz)-o[3]
            if d<min_d: min_d=d
        with lock:
            state['min_obs_dist'].append(max(0,min_d))
            state['min_obs_t'].append(time.time()-state['start_time'])

# ====== HTTP ======
HTML = r"""<!DOCTYPE html><html><head><meta charset="utf-8">
<title>Drone Live Monitor</title><style>
*{margin:0;padding:0;box-sizing:border-box}
body{font:13px monospace;background:#0a0a0f;color:#0f0;padding:10px}
h1{font-size:1.1em;margin-bottom:6px;color:#0ff}
.grid{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin-bottom:8px}
.card{background:#111;border:1px solid #333;border-radius:5px;padding:8px}
.card .label{font-size:0.7em;color:#888}
.card .val{font-size:1.1em;font-weight:bold;margin-top:2px}
canvas{width:100%;height:180px;background:#111;border:1px solid #333;border-radius:5px;margin-bottom:6px}
.good{color:#0f0}.warn{color:#ff0}.bad{color:#f44}
h2{font-size:0.95em;color:#888;margin:10px 0 4px}
</style></head><body>
<h1>🚁 Drone Live Monitor <small id="timer">00:00</small></h1>
<div class="grid">
<div class="card"><div class="label">Pos (x,y,z m)</div><div class="val" id="pos">—</div></div>
<div class="card"><div class="label">Pos Err (3D m)</div><div class="val" id="perr">—</div></div>
<div class="card"><div class="label">Steady Err z</div><div class="val" id="serr">—</div></div>
<div class="card"><div class="label">Vel (x,y,z m/s)</div><div class="val" id="vel">—</div></div>
<div class="card"><div class="label">RPM (FL,FR,BL,BR)</div><div class="val" id="rpm">—</div></div>
<div class="card"><div class="label">Min Obs Dist (m)</div><div class="val" id="minobs">—</div></div>
<div class="card"><div class="label">Overshoot Z</div><div class="val" id="overshoot">—</div></div>
<div class="card"><div class="label">Path Len / Time</div><div class="val" id="pathlen">—</div></div>
<div class="card"><div class="label">RPM Sat?</div><div class="val" id="rpmsat">—</div></div>
<div class="card"><div class="label">Att Diverge?</div><div class="val" id="attdiv">—</div></div>
<div class="card"><div class="label">Goal</div><div class="val" id="goal">—</div></div>
<div class="card"><div class="label">Z overshoot (m)</div><div class="val" id="zover">—</div></div>
</div>
<h2>Position (x=green y=magenta z=cyan)</h2><canvas id="cpos"></canvas>
<h2>RPM (4 motors)</h2><canvas id="crpm"></canvas>
<h2>XY Trajectory</h2><canvas id="ctraj"></canvas>
<h2>Min Obstacle Distance</h2><canvas id="cobs"></canvas>
<script>
const COLS=['#0f0','#f0f','#0ff','#ff0'];
function drawLine(cid,series,ylab){
  const c=document.getElementById(cid),w=c.width=Math.max(400,c.parentElement.clientWidth-24),h=c.height=c.width*0.4;
  const ctx=c.getContext('2d');ctx.clearRect(0,0,w,h);ctx.strokeStyle='#333';ctx.lineWidth=1;
  for(let i=0;i<=4;i++){let y=h*i/4;ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(w,y);ctx.stroke()}
  if(!series||!series[0]||!series[0].length)return;
  let ymin=Infinity,ymax=-Infinity;const n=series[0].length;
  for(const s of series)for(const v of s){if(v<ymin)ymin=v;if(v>ymax)ymax=v}
  if(ymax-ymin<0.001){ymin-=1;ymax+=1}
  for(let si=0;si<series.length;si++){
    ctx.strokeStyle=COLS[si%4];ctx.lineWidth=1.5;ctx.beginPath();
    const wx=v=>h-((v-ymin)/(ymax-ymin))*h;
    for(let i=0;i<n;i++){const x=i/n*w,y=wx(series[si][i]);i==0?ctx.moveTo(x,y):ctx.lineTo(x,y)}
    ctx.stroke();ctx.fillStyle=COLS[si%4];ctx.font='9px monospace';ctx.fillText((si+1)+'',4+si*20,10)
  }
  ctx.fillStyle='#888';ctx.fillText(ylab,4,20);
}
function update(){
  fetch('/data').then(r=>r.json()).then(d=>{
    if(!d.t||!d.t.length)return;
    const n=d.t.length,et=d.t[n-1];
    document.getElementById('timer').textContent=new Date(et*1000).toISOString().substr(14,5);
    const px=d.px[n-1],py=d.py[n-1],pz=d.pz[n-1];
    document.getElementById('pos').textContent=px.toFixed(2)+' '+py.toFixed(2)+' '+pz.toFixed(2);
    const ex=Math.abs(px-d.goal_x),ey=Math.abs(py-d.goal_y),ez=Math.abs(pz-d.goal_z);
    document.getElementById('perr').textContent=Math.sqrt(ex*ex+ey*ey+ez*ez).toFixed(4)+' m';
    const n20=Math.max(1,Math.floor(n/5));let se=0;for(let i=n-n20;i<n;i++)se+=Math.abs(d.pz[i]-d.goal_z);
    document.getElementById('serr').textContent=(se/n20).toFixed(4)+'m '+(se/n20<0.3?'✅':'⚠');
    const vn=Math.min(d.vx.length,d.vy.length,d.vz.length)-1;
    if(vn>=0)document.getElementById('vel').textContent=d.vx[vn].toFixed(2)+' '+d.vy[vn].toFixed(2)+' '+d.vz[vn].toFixed(2);
    const rn=d.rpm[0].length-1;
    if(rn>=0)document.getElementById('rpm').textContent=d.rpm[0][rn].toFixed(0)+' '+d.rpm[1][rn].toFixed(0)+' '+d.rpm[2][rn].toFixed(0)+' '+d.rpm[3][rn].toFixed(0);
    if(d.min_obs_dist.length){
      const md=d.min_obs_dist[d.min_obs_dist.length-1];
      document.getElementById('minobs').textContent=md.toFixed(3)+' m';
    }
    let zmax=-Infinity;for(let z of d.pz)if(z>zmax)zmax=z;
    document.getElementById('zover').textContent=Math.max(0,zmax-d.goal_z).toFixed(4)+' m';
    document.getElementById('goal').textContent=d.goal_x.toFixed(1)+' '+d.goal_y.toFixed(1)+' '+d.goal_z.toFixed(1);
    let path=0;for(let i=1;i<n;i++){let dx=d.px[i]-d.px[i-1],dy=d.py[i]-d.py[i-1],dz=d.pz[i]-d.pz[i-1];path+=Math.sqrt(dx*dx+dy*dy+dz*dz)}
    document.getElementById('pathlen').textContent=path.toFixed(1)+'m / '+et.toFixed(1)+'s';
    // RPM saturation
    let sat=false;for(let i=0;i<4;i++){const rr=d.rpm[i];for(let j=Math.max(0,rr.length-50);j<rr.length;j++)if(rr[j]>=9999||rr[j]<=1){sat=true;break}}
    document.getElementById('rpmsat').textContent=sat?'⚠ YES':'No';
    document.getElementById('rpmsat').className=sat?'bad':'good';
    // charts
    drawLine('cpos',[d.px,d.py,d.pz],'Pos');
    if(d.rpm[0].length)drawLine('crpm',[d.rpm[0],d.rpm[1],d.rpm[2],d.rpm[3]],'RPM');
    if(d.min_obs_dist.length)drawLine('cobs',[d.min_obs_dist],'ObsDist');
    // trajectory
    {const c=document.getElementById('ctraj'),w=c.width=Math.max(400,c.parentElement.clientWidth-24),h=c.height=c.width*0.4;
    const ctx=c.getContext('2d');ctx.clearRect(0,0,w,h);ctx.strokeStyle='#333';ctx.lineWidth=1;
    ctx.beginPath();ctx.moveTo(w/2,0);ctx.lineTo(w/2,h);ctx.moveTo(0,h/2);ctx.lineTo(w,h/2);ctx.stroke();
    if(n<2)return;
    let xmin=Infinity,xmax=-Infinity,ymin=Infinity,ymax=-Infinity;
    for(let i=0;i<n;i++){if(d.px[i]<xmin)xmin=d.px[i];if(d.px[i]>xmax)xmax=d.px[i];if(d.py[i]<ymin)ymin=d.py[i];if(d.py[i]>ymax)ymax=d.py[i]}
    const pad=0.2;xmin-=pad;xmax+=pad;ymin-=pad;ymax+=pad;
    if(xmax-xmin<0.1){xmin-=1;xmax+=1}if(ymax-ymin<0.1){ymin-=1;ymax+=1}
    ctx.strokeStyle='#0f0';ctx.lineWidth=1.5;ctx.beginPath();
    for(let i=0;i<n;i++){const x=((d.px[i]-xmin)/(xmax-xmin))*w,y=h-((d.py[i]-ymin)/(ymax-ymin))*h;i==0?ctx.moveTo(x,y):ctx.lineTo(x,y)}
    ctx.stroke();ctx.fillStyle='#0f0';ctx.font='9px monospace';ctx.fillText('XY Traj',4,10)}
  })
}
setInterval(update,500);
</script></body></html>"""

class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path=='/':
            self.send_response(200);self.send_header('Content-Type','text/html');self.end_headers()
            self.wfile.write(HTML.encode())
        elif self.path=='/data':
            self.send_response(200);self.send_header('Content-Type','application/json');self.end_headers()
            with lock:
                d={k:state[k] for k in ['t','px','py','pz','vx','vy','vz','rpm','rpm_t','goal_x','goal_y','goal_z','min_obs_dist','min_obs_t']}
            self.wfile.write(json.dumps(d).encode())
        else: self.send_response(404);self.end_headers()
    def log_message(self,*a): pass

if __name__=='__main__':
    print("Starting live_monitor...")
    print("  Open http://localhost:8765 in your browser")
    print("  Ctrl+C to stop & save CSV")

    for t in [threading.Thread(target=read_odom,daemon=True),
              threading.Thread(target=read_odom_velocity,daemon=True),
              threading.Thread(target=read_rpm,daemon=True),
              threading.Thread(target=read_goal,daemon=True),
              threading.Thread(target=read_obstacles,daemon=True),
              threading.Thread(target=compute_min_obs_dist,daemon=True)]:
        t.start()
    time.sleep(2)

    class ReuseServer(HTTPServer):
        allow_reuse_address = True
        def server_bind(self):
            self.socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            HTTPServer.server_bind(self)
    server=ReuseServer(('0.0.0.0',8765),Handler)
    try: server.serve_forever()
    except KeyboardInterrupt:
        print("\nSaving CSV...")
        with lock:
            with open(SAVE_FILE,'w') as f:
                f.write("t,px,py,pz,vx,vy,vz,rpm0,rpm1,rpm2,rpm3,min_obs_dist\n")
                n=max(len(state['t']),len(state['rpm_t']),len(state['min_obs_t']))
                for i in range(n):
                    vals=[]
                    for k in ['t','px','py','pz','vx','vy','vz']:
                        arr=state[k]; vals.append(str(arr[i]) if i<len(arr) else '')
                    for j in range(4):
                        vals.append(str(state['rpm'][j][i]) if i<len(state['rpm'][j]) else '')
                    vals.append(str(state['min_obs_dist'][i]) if i<len(state['min_obs_dist']) else '')
                    f.write(','.join(vals)+'\n')
        print(f"Saved {SAVE_FILE} ({n} rows)")
        state['running']=False
