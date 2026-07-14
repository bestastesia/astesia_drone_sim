#!/usr/bin/env python3
"""
live_monitor.py — 实时监控面板（浏览器） + 自动存 CSV
=====================================================
启动:  python3 live_monitor.py
浏览器打开: http://localhost:8765
Ctrl+C 停止 → 自动保存 dashboard_log.csv

不需要 matplotlib, rclpy。全靠 ros2 topic echo subprocess + http.server。
"""
import subprocess, threading, time, json, math, os, sys, socket
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse

# ====== 全局状态 ======
state = {
    't': [], 'px': [], 'py': [], 'pz': [],
    'vx': [], 'vy': [], 'vz': [],
    'rpm': [[],[],[],[]],
    'rpm_t': [],
    'goal_x': 0.0, 'goal_y': 0.0, 'goal_z': 1.5,
    'obstacles': [],  # [(cx,cy,cz,r_or_half,shape)]
    'min_obs_dist': [],
    'min_obs_t': [],
    'start_time': time.time(),
    'running': True,
}
lock = threading.Lock()
SAVE_FILE = "dashboard_log.csv"
odom_proc, rpm_proc, goal_proc, obs_proc = None, None, None, None

def sh(cmd, timeout=5):
    return subprocess.run(['/bin/bash','-c',cmd], capture_output=True, text=True, timeout=timeout)

# ====== ROS 数据采集线程 ======
def read_odom():
    global odom_proc
    odom_proc = subprocess.Popen(
        ['ros2','topic','echo','/drone/odom','--field','pose.pose.position','--field','twist.twist.linear'],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
    px=py=pz=vx=vy=vz=None
    for line in odom_proc.stdout:
        line=line.strip()
        if not line.startswith(('x:','y:','z:')): continue
        try:
            if line.startswith('x:'): px=float(line.split(':')[1]); vx_flag=True
            elif line.startswith('y:'): py=float(line.split(':')[1])
            elif line.startswith('z:'):
                if px is not None and py is not None:
                    pz=float(line.split(':')[1])
                    with lock:
                        state['px'].append(px); state['py'].append(py); state['pz'].append(pz)
                        state['t'].append(time.time()-state['start_time'])
                # reset for next tuple
                px=py=pz=None
        except: pass

def read_rpm():
    global rpm_proc
    rpm_proc = subprocess.Popen(
        ['ros2','topic','echo','/drone/motor_rpm_cmd','--field','data'],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
    vals=[]
    for line in rpm_proc.stdout:
        line=line.strip()
        if line.startswith('-'):
            if len(vals)==4:
                with lock:
                    for i in range(4): state['rpm'][i].append(vals[i])
                    state['rpm_t'].append(time.time()-state['start_time'])
            vals=[]
            continue
        try: vals.append(float(line.split(':')[1]))
        except: pass

def read_goal():
    global goal_proc
    goal_proc = subprocess.Popen(
        ['ros2','topic','echo','/drone/goal','--field','pose.position'],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
    gx=gy=gz=None
    for line in goal_proc.stdout:
        line=line.strip()
        try:
            if line.startswith('x:'): gx=float(line.split(':')[1])
            elif line.startswith('y:'): gy=float(line.split(':')[1])
            elif line.startswith('z:'):
                gz=float(line.split(':')[1])
                if gx is not None and gy is not None:
                    with lock: state['goal_x']=gx; state['goal_y']=gy; state['goal_z']=gz
                gx=gy=gz=None
        except: pass

def read_obstacles():
    global obs_proc
    # 解析 Marker 消息的 position 和 scale
    obs_proc = subprocess.Popen(
        ['ros2','topic','echo','/map/obstacles','--field','pose.position','--field','scale'],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
    cx=cy=cz=sx=sy=sz=None
    obstacles=[]
    for line in obs_proc.stdout:
        line=line.strip()
        try:
            if line.startswith('x:'): cx=float(line.split(':')[1])
            elif line.startswith('y:'): cy=float(line.split(':')[1])
            elif line.startswith('z:'): cz=float(line.split(':')[1])
            elif line.startswith('x:') and cx is not None: sx=float(line.split(':')[1])
            elif line.startswith('y:') and sx is not None: sy=float(line.split(':')[1])
            elif line.startswith('z:') and sy is not None:
                sz=float(line.split(':')[1])
                obstacles.append((cx,cy,cz,max(sx,sy,sz)*0.5))
                cx=cy=cz=sx=sy=sz=None
            elif line == '---':
                if obstacles:
                    with lock: state['obstacles']=obstacles
                obstacles=[]
        except: pass

# ====== 最小障碍物距离计算 ======
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
            dx=px-o[0]; dy=py-o[1]; dz=pz-o[2]
            d=math.sqrt(dx*dx+dy*dy+dz*dz)-o[3]
            if d<min_d: min_d=d
        with lock:
            state['min_obs_dist'].append(max(0,min_d))
            state['min_obs_t'].append(time.time()-state['start_time'])

# ====== HTTP 服务器 (仪表盘) ======
HTML = r"""<!DOCTYPE html><html><head><meta charset="utf-8">
<title>Drone Monitor</title><style>
*{margin:0;padding:0;box-sizing:border-box}
body{font:14px monospace;background:#0a0a0f;color:#0f0;padding:12px}
h1{font-size:1.2em;margin-bottom:8px;color:#0ff}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-bottom:12px}
.card{background:#111;border:1px solid #333;border-radius:6px;padding:10px}
.card .label{font-size:0.75em;color:#888}
.card .val{font-size:1.4em;font-weight:bold;margin-top:4px}
canvas{width:100%;height:200px;background:#111;border:1px solid #333;border-radius:6px;margin-bottom:8px}
.status-fail{color:#f44}.status-ok{color:#0f0}
</style></head><body>
<h1>🚁 Drone Live Monitor &nbsp; <small id="timer">00:00</small></h1>
<div class="grid">
<div class="card"><div class="label">Position (xyz)</div><div class="val" id="pos">—</div></div>
<div class="card"><div class="label">Position Error (m)</div><div class="val" id="perr">—</div></div>
<div class="card"><div class="label">Steady-State Err (z, last 1s)</div><div class="val" id="serr">—</div></div>
<div class="card"><div class="label">Velocity (m/s)</div><div class="val" id="vel">—</div></div>
<div class="card"><div class="label">RPM (FL,FR,BL,BR)</div><div class="val" id="rpm">—</div></div>
<div class="card"><div class="label">Min Obstacle Dist (m)</div><div class="val" id="minobs">—</div></div>
<div class="card"><div class="label">Overshoot Z (m)</div><div class="val" id="overshoot">—</div></div>
<div class="card"><div class="label">Path Length (m)</div><div class="val" id="pathlen">—</div></div>
<div class="card"><div class="label">Flight Time / Settle &lt;0.3m</div><div class="val" id="ftime">—</div></div>
<div class="card"><div class="label">RPM Saturation</div><div class="val" id="rpmsat">No</div></div>
<div class="card"><div class="label">Attitude Divergence</div><div class="val" id="attdiv">No</div></div>
<div class="card"><div class="label">Status</div><div class="val status-ok" id="stat">OK</div></div>
</div>
<canvas id="cpos"></canvas><canvas id="crpm"></canvas><canvas id="ctraj"></canvas><canvas id="cobs"></canvas>
<script>
const COLORS=['#0f0','#f0f','#0ff','#ff0'];
function drawLine(canvasId, series, labels, ylabel){
    const c=document.getElementById(canvasId),w=c.width=Math.max(400,c.parentElement.clientWidth-24),h=c.height=c.width*0.45;
    const ctx=c.getContext('2d');ctx.clearRect(0,0,w,h);ctx.strokeStyle='#333';ctx.lineWidth=1;
    for(let i=0;i<=4;i++){let y=h*i/4;ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(w,y);ctx.stroke()}
    if(!series||!series[0]||!series[0].length)return;
    let ymin=Infinity,ymax=-Infinity;const n=series[0].length;
    for(const s of series)for(const v of s){if(v<ymin)ymin=v;if(v>ymax)ymax=v}
    if(ymax-ymin<0.001){ymin-=1;ymax+=1}
    for(let si=0;si<series.length;si++){
        ctx.strokeStyle=COLORS[si%COLORS.length];ctx.lineWidth=1.5;ctx.beginPath();
        let sx,wx=y=>(h-((y-ymin)/(ymax-ymin))*h);
        for(let i=0;i<n;i++){let x=i/n*w,yi=wx(series[si][i]);i==0?ctx.moveTo(x,yi):ctx.lineTo(x,yi)}
        ctx.stroke();
    }
    ctx.fillStyle='#888';ctx.font='10px monospace';
    ctx.fillText(ylabel,4,10);
}
function update(){
    fetch('/data').then(r=>r.json()).then(d=>{
        if(!d.t||!d.t.length)return;
        const n=d.t.length,elapsed=d.t[n-1];
        document.getElementById('timer').textContent=new Date(elapsed*1000).toISOString().substr(14,5);
        const px=d.px[n-1],py=d.py[n-1],pz=d.pz[n-1];
        document.getElementById('pos').textContent=px.toFixed(3)+' '+py.toFixed(3)+' '+pz.toFixed(3);
        const ex=Math.abs(px-d.goal_x),ey=Math.abs(py-d.goal_y),ez=Math.abs(pz-d.goal_z);
        const err3d=Math.sqrt(ex*ex+ey*ey+ez*ez);
        document.getElementById('perr').textContent=err3d.toFixed(4)+' m';
        const n20=Math.max(1,Math.floor(n/5));let se=0;for(let i=n-n20;i<n;i++)se+=Math.abs(d.pz[i]-d.goal_z);
        document.getElementById('serr').textContent=(se/n20).toFixed(4)+' m';
        const sc='<0.3'+(Math.abs(se/n20)<0.3?' ✅ PASS':' ❌ FAIL');
        document.getElementById('serr').textContent=(se/n20).toFixed(4)+' m '+sc;
        document.getElementById('vel').textContent=d.vx[n-1].toFixed(2)+' '+d.vy[n-1].toFixed(2)+' '+d.vz[n-1].toFixed(2);
        if(d.rpm[0].length){const rn=d.rpm[0].length-1;
            document.getElementById('rpm').textContent=d.rpm[0][rn].toFixed(0)+' '+d.rpm[1][rn].toFixed(0)+' '+d.rpm[2][rn].toFixed(0)+' '+d.rpm[3][rn].toFixed(0)}
        const mod=d.min_obs_dist;document.getElementById('minobs').textContent=mod.length?(mod[mod.length-1].toFixed(3)+' m'):'—';
        let zmax=-Infinity;for(let z of d.pz)if(z>zmax)zmax=z;
        document.getElementById('overshoot').textContent=Math.max(0,zmax-d.goal_z).toFixed(4)+' m';
        let path=0;for(let i=1;i<n;i++){let dx=d.px[i]-d.px[i-1],dy=d.py[i]-d.py[i-1],dz=d.pz[i]-d.pz[i-1];path+=Math.sqrt(dx*dx+dy*dy+dz*dz)}
        document.getElementById('pathlen').textContent=path.toFixed(2)+' m';
        document.getElementById('ftime').textContent=elapsed.toFixed(1)+'s / settle <0.3m';
        // RPM saturation check
        let sat=false;for(let i=0;i<4;i++){const rr=d.rpm[i];for(let j=Math.max(0,rr.length-50);j<rr.length;j++){if(rr[j]>=9999||rr[j]<=1){sat=true;break}}}
        document.getElementById('rpmsat').textContent=sat?'⚠ YES':'No';
        document.getElementById('rpmsat').className=sat?'status-fail':'status-ok';
        // draw charts
        const tArr=d.t;
        drawLine('cpos',[d.px,d.py,d.pz],tArr,'Position');
        drawLine('crpm',[d.rpm[0],d.rpm[1],d.rpm[2],d.rpm[3]],d.rpm_t.length?d.rpm_t:tArr,'RPM');
        if(d.min_obs_dist&&d.min_obs_dist.length)drawLine('cobs',[d.min_obs_dist],d.min_obs_t.length?d.min_obs_t:tArr,'MinObsDist');
        // trajectory (XY only on last canvas)
        {const c=document.getElementById('ctraj'),w=c.width=Math.max(400,c.parentElement.clientWidth-24),h=c.height=c.width*0.45;const ctx=c.getContext('2d');ctx.clearRect(0,0,w,h);ctx.strokeStyle='#333';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(w/2,0);ctx.lineTo(w/2,h);ctx.moveTo(0,h/2);ctx.lineTo(w,h/2);ctx.stroke();let xmin=Infinity,xmax=-Infinity,ymin=Infinity,ymax=-Infinity;for(let i=0;i<n;i++){if(d.px[i]<xmin)xmin=d.px[i];if(d.px[i]>xmax)xmax=d.px[i];if(d.py[i]<ymin)ymin=d.py[i];if(d.py[i]>ymax)ymax=d.py[i]}if(xmax-xmin<0.1){xmin-=1;xmax+=1}if(ymax-ymin<0.1){ymin-=1;ymax+=1}const pad=0.1;xmin-=pad;xmax+=pad;ymin-=pad;ymax+=pad;ctx.strokeStyle='#0f0';ctx.lineWidth=1.5;ctx.beginPath();for(let i=0;i<n;i++){let x=((d.px[i]-xmin)/(xmax-xmin))*w,y=h-((d.py[i]-ymin)/(ymax-ymin))*h;i==0?ctx.moveTo(x,y):ctx.lineTo(x,y)}ctx.stroke();ctx.fillStyle='#888';ctx.font='10px monospace';ctx.fillText('Trajectory (XY)',4,10)}
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
        else:
            self.send_response(404);self.end_headers()
    def log_message(self,*a): pass

# ====== 主入口 ======
if __name__=='__main__':
    print("Starting live_monitor...")
    print("  Open http://localhost:8765 in your browser")
    print("  Ctrl+C to stop & save CSV")

    # 启动采集线程
    threads=[threading.Thread(target=read_odom,daemon=True),
             threading.Thread(target=read_rpm,daemon=True),
             threading.Thread(target=read_goal,daemon=True),
             threading.Thread(target=read_obstacles,daemon=True),
             threading.Thread(target=compute_min_obs_dist,daemon=True)]
    for t in threads: t.start()
    time.sleep(2)

    # HTTP server
    import socketserver
    class ReuseServer(HTTPServer):
        allow_reuse_address = True
        def server_bind(self):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
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
                    t0=state['t'][i] if i<len(state['t']) else ''
                    px=state['px'][i] if i<len(state['px']) else ''
                    py=state['py'][i] if i<len(state['py']) else ''
                    pz=state['pz'][i] if i<len(state['pz']) else ''
                    vx=state['vx'][i] if i<len(state['vx']) else ''
                    vy=state['vy'][i] if i<len(state['vy']) else ''
                    vz=state['vz'][i] if i<len(state['vz']) else ''
                    r0=state['rpm'][0][i] if i<len(state['rpm'][0]) else ''; r1=state['rpm'][1][i] if i<len(state['rpm'][1]) else ''
                    r2=state['rpm'][2][i] if i<len(state['rpm'][2]) else ''; r3=state['rpm'][3][i] if i<len(state['rpm'][3]) else ''
                    md=state['min_obs_dist'][i] if i<len(state['min_obs_dist']) else ''
                    f.write(f"{t0},{px},{py},{pz},{vx},{vy},{vz},{r0},{r1},{r2},{r3},{md}\n")
        print(f"Saved {SAVE_FILE} ({n} rows)")
        state['running']=False
        for p in [odom_proc,rpm_proc,goal_proc,obs_proc]:
            if p: p.kill()
