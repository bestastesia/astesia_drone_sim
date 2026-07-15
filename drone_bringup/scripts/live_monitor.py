#!/usr/bin/env python3
"""live_monitor — http://localhost:8765 | Ctrl+C=CSV"""
import subprocess as sp, threading as th, time, json, math, os, sys, socket, re
import yaml
from http.server import HTTPServer, BaseHTTPRequestHandler

st = {'t':[],'px':[],'py':[],'pz':[],'vx':[],'vy':[],'vz':[],
      'rpm':[[],[],[],[]],'rpm_t':[],'goal_x':0,'goal_y':0,'goal_z':1.5,
      'obs':[],'min_d':[],'min_d_t':[],'start':time.time(),'run':True}
lk = th.Lock()
SF = "dashboard_log.csv"

def sh(c):
    return sp.run(['/bin/bash','-c',c], capture_output=True, text=True, timeout=8)

def r_odom():
    p = sp.Popen(['ros2','topic','echo','/drone/odom'], stdout=sp.PIPE, stderr=sp.DEVNULL, text=True, bufsize=1)
    x=y=z=None; inp=False
    for L in p.stdout:
        L=L.strip()
        if L.startswith('position:'): inp=True; continue
        if L.startswith('orientation:') or L=='---': inp=False; continue
        if not inp: continue
        try:
            if L.startswith('x:'): x=float(L.split(':')[1])
            elif L.startswith('y:'): y=float(L.split(':')[1])
            elif L.startswith('z:'): z=float(L.split(':')[1])
        except: continue
        if x is not None and y is not None and z is not None:
            with lk:
                st['px'].append(x); st['py'].append(y); st['pz'].append(z)
                st['t'].append(time.time()-st['start'])
            x=y=z=None

def r_vel():
    p = sp.Popen(['ros2','topic','echo','/drone/odom','--field','twist.twist.linear'], stdout=sp.PIPE, stderr=sp.DEVNULL, text=True, bufsize=1)
    a=b=c=None
    for L in p.stdout:
        L=L.strip()
        try:
            if L.startswith('x:'): a=float(L.split(':')[1])
            elif L.startswith('y:'): b=float(L.split(':')[1])
            elif L.startswith('z:'): c=float(L.split(':')[1])
        except: continue
        if a is not None and b is not None and c is not None:
            with lk:
                st['vx'].append(a); st['vy'].append(b); st['vz'].append(c)
                while len(st['vx'])>len(st['px']): st['vx'].pop(); st['vy'].pop(); st['vz'].pop()
            a=b=c=None

def r_rpm():
    p = sp.Popen(['ros2','topic','echo','/drone/motor_rpm_cmd','--field','data'], stdout=sp.PIPE, stderr=sp.DEVNULL, text=True, bufsize=1)
    for L in p.stdout:
        m = re.search(r'\[(.*?)\]', L.strip())
        if not m: continue
        try:
            v=[float(x.strip()) for x in m.group(1).split(',')]
            if len(v)==4:
                with lk:
                    for i in range(4): st['rpm'][i].append(v[i])
                    st['rpm_t'].append(time.time()-st['start'])
        except: pass

def r_goal():
    p = sp.Popen(['ros2','topic','echo','/drone/goal','--field','pose.position'], stdout=sp.PIPE, stderr=sp.DEVNULL, text=True, bufsize=1)
    a=b=c=None
    for L in p.stdout:
        L=L.strip()
        try:
            if L.startswith('x:'): a=float(L.split(':')[1])
            elif L.startswith('y:'): b=float(L.split(':')[1])
            elif L.startswith('z:'): c=float(L.split(':')[1])
        except: continue
        if a is not None and b is not None and c is not None:
            with lk: st['goal_x']=a; st['goal_y']=b; st['goal_z']=c
            a=b=c=None

def r_obs():
    p = sp.Popen(['ros2','topic','echo','/map/obstacles','--field','pose.position','--field','scale'],
                  stdout=sp.PIPE, stderr=sp.DEVNULL, text=True, bufsize=1)
    markers = []; cx=cy=cz=sx=sy=sz=None
    for L in p.stdout:
        L=L.strip()
        if L=='' and cx is not None and sx is not None:
            rv = max(abs(sx),abs(sy),abs(sz)) if abs(sy)>0.01 else abs(sx)
            markers.append((cx,cy,cz,rv))
            cx=cy=cz=sx=sy=sz=None; continue
        if L=='---':
            if markers: 
                with lk: st['obs'] = list(markers)
            markers = []; cx=cy=cz=sx=sy=sz=None; continue
        try:
            if L.startswith('x:'):
                v=float(L.split(':')[1])
                if cx is None: cx=v
                elif sx is None: sx=v
            elif L.startswith('y:'):
                v=float(L.split(':')[1])
                if cy is None: cy=v
                elif sy is None: sy=v
            elif L.startswith('z:'):
                v=float(L.split(':')[1])
                if cz is None: cz=v
                elif sz is None: sz=v
        except: pass

def r_min():
    while st['run']:
        time.sleep(0.2)
        with lk:
            if not st['px']: continue
            x=st['px'][-1]; y=st['py'][-1]; z=st['pz'][-1]
        if not x: continue
        # 实时从 YAML 文件读取障碍物配置
        obs_list = []
        try:
            import yaml
            for pfx in ['', '/home/astesia/drone_sim_ws/install/drone_map/share/drone_map/config/',
                         '/home/astesia/drone_sim_ws/src/astesia_drone_sim/drone_map/config/']:
                try:
                    path = pfx + 'map.yaml'
                    with open(path) as f:
                        cfg = yaml.safe_load(f)
                    break
                except: continue
            params = cfg.get('drone_map',{}).get('ros__parameters',{})
            for obs_str in params.get('obstacles',[]):
                parts = obs_str.split()
                if not parts: continue
                shape = parts[0]
                cx,cy,cz = float(parts[1]),float(parts[2]),float(parts[3])
                r = float(parts[4])
                if shape == 'sphere':   obs_list.append((cx,cy,cz,r))
                elif shape == 'cylinder': obs_list.append((cx,cy,cz,r))
                elif shape == 'cube':   obs_list.append((cx,cy,cz,float(parts[4])))
        except: pass
        # fallback: parse from ros2 topic echo
        # 显式模式无数据或 procedural 时用默认障碍物集
        if not obs_list:
            obs_list = [
                (0.7,0.3,1.5,0.35),(1.0,0.5,1.5,0.35),(1.3,0.5,1.5,0.2),
                (0.5,0.7,1.0,0.25),(0.8,-0.2,1.5,0.2),(0.4,0.0,1.5,0.3)]
        md = 1e9
        for o in obs_list:
            d = math.sqrt((x-o[0])**2 + (y-o[1])**2 + (z-o[2])**2) - o[3]
            if d < md: md = d
        with lk:
            st['min_d'].append(max(0, md))
            st['min_d_t'].append(time.time()-st['start'])

HTML = r'''<!DOCTYPE html><html lang="zh"><head><meta charset="utf-8"><title>Drone Monitor</title>
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
canvas{border:1px solid #30363d;border-radius:6px;background:#161b22;margin-bottom:4px}
</style></head><body>
<h1>Drone</h1>
<div class="grid">
<div class="card"><div class="lb">Pos (x,y,z m)</div><div class="vl c" id="p">-</div></div>
<div class="card"><div class="lb">3D Err (m)</div><div class="vl y" id="pe">-</div></div>
<div class="card"><div class="lb">Steady Err z</div><div class="vl" id="se">-</div></div>
<div class="card"><div class="lb">Vel (x,y,z m/s)</div><div class="vl c" id="ve">-</div></div>
<div class="card"><div class="lb">RPM (FL,FR,BL,BR)</div><div class="vl c" id="rp">-</div></div>
<div class="card"><div class="lb">MinObsDist (m)</div><div class="vl" id="od">-</div></div>
<div class="card"><div class="lb">Overshoot Z</div><div class="vl y" id="os">-</div></div>
<div class="card"><div class="lb">Path/Time</div><div class="vl c" id="pt">-</div></div>
<div class="card"><div class="lb">RPM Sat?</div><div class="vl" id="rs">-</div></div>
<div class="card"><div class="lb">Att Diverge?</div><div class="vl" id="ad">-</div></div>
<div class="card"><div class="lb">Goal</div><div class="vl c" id="go">-</div></div>
</div>
<h2>1. Err(t)=|pos-goal| (x=green z=blue, y=0-3m, ref=0.3m)</h2><canvas id="cpe"></canvas>
<h2>2. RPM (FL/FR/BL/BR, y=0-50)</h2><canvas id="crp"></canvas>
<h2>3. XY Trajectory (green=actual, bbox=[-1,4])</h2><canvas id="ctj"></canvas>
<h2>4. Obstacle Distances (all obstacles, red=0.4m safety, thick=min)</h2><canvas id="cmd"></canvas>
<script>
var CL=['#3fb950','#58a6ff','#d29922','#c9d1d9'];
var HOV_Z=1.5;

function chart(id,series,ylab,ymin,ymax,ref){
  var c=document.getElementById(id);
  c.width=Math.max(500,c.parentElement.clientWidth-24);
  c.height=220;
  var w=c.width,h=c.height,ctx=c.getContext('2d');
  ctx.clearRect(0,0,w,h);
  ctx.strokeStyle='#21262d';ctx.lineWidth=1;
  for(var i=0;i<=4;i++){var y=h*i/4;ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(w,y);ctx.stroke()}
  for(var i=0;i<=10;i++){var x=w*i/10;ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,h);ctx.stroke()}
  if(ref!==undefined&&ref!==null){
    ctx.setLineDash([4,6]);ctx.strokeStyle='#f85149';ctx.lineWidth=1.5;
    var ry=h-(ref-ymin)/(ymax-ymin)*h;ctx.beginPath();ctx.moveTo(0,ry);ctx.lineTo(w,ry);ctx.stroke();
    ctx.setLineDash([]);ctx.fillStyle='#f85149';ctx.font='10px sans-serif';ctx.fillText(ref.toFixed(2),4,ry-4);
  }
  if(!series||!series[0]||!series[0].length){ctx.fillStyle='#8b949e';ctx.fillText('...',w/2-10,h/2)}
  else {
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
  ctx.fillStyle='#8b949e';ctx.font='10px sans-serif';
  for(var i=0;i<=4;i++){ctx.fillText((ymax-(ymax-ymin)*i/4).toFixed(1),2,h*i/4+10)}
  ctx.fillStyle='#58a6ff';ctx.fillText(ylab,4,12);
}

function update(){
  fetch('/data').then(function(r){return r.json()}).then(function(d){
    if(!d.t||!d.t.length)return;
    var n=d.t.length; HOV_Z=d.goal_z||1.5;
    var px=d.px[n-1],py=d.py[n-1],pz=d.pz[n-1];
    document.getElementById('p').textContent=px.toFixed(2)+' '+py.toFixed(2)+' '+pz.toFixed(2);
    var e3=Math.sqrt(Math.pow(px-d.goal_x,2)+Math.pow(py-d.goal_y,2)+Math.pow(pz-HOV_Z,2));
    document.getElementById('pe').textContent=e3.toFixed(4)+' m';
    document.getElementById('pe').className='vl '+(e3<0.3?'g':'y');
    var n20=Math.max(1,Math.floor(n/5)),se=0;
    for(var i=n-n20;i<n;i++)se+=Math.abs(d.pz[i]-HOV_Z);
    document.getElementById('se').textContent=(se/n20).toFixed(4)+' m';
    document.getElementById('se').className='vl '+((se/n20)<0.3?'g':'r');
    var vi=Math.min(d.vx.length,d.vy.length,d.vz.length)-1;
    if(vi>=0)document.getElementById('ve').textContent=d.vx[vi].toFixed(2)+' '+d.vy[vi].toFixed(2)+' '+d.vz[vi].toFixed(2);
    var ri=d.rpm[0].length-1;
    if(ri>=0)document.getElementById('rp').textContent=d.rpm[0][ri].toFixed(0)+' '+d.rpm[1][ri].toFixed(0)+' '+d.rpm[2][ri].toFixed(0)+' '+d.rpm[3][ri].toFixed(0);
    var mdv=d.min_d.length?d.min_d[d.min_d.length-1]:0;
    document.getElementById('od').textContent=mdv.toFixed(3)+' m';
    document.getElementById('od').className='vl '+(mdv>0.4?'g':(mdv>0.15?'y':'r'));
    var zmax=-Infinity;for(var i=0;i<n;i++)if(d.pz[i]>zmax)zmax=d.pz[i];
    document.getElementById('os').textContent=Math.max(0,zmax-HOV_Z).toFixed(4)+' m';
    var path=0;for(var i=1;i<n;i++){var dx=d.px[i]-d.px[i-1],dy=d.py[i]-d.py[i-1],dz=d.pz[i]-d.pz[i-1];path+=Math.sqrt(dx*dx+dy*dy+dz*dz)}
    document.getElementById('pt').textContent=path.toFixed(1)+'m/'+(d.t[n-1]).toFixed(1)+'s';
    document.getElementById('go').textContent=d.goal_x.toFixed(1)+' '+d.goal_y.toFixed(1)+' '+HOV_Z.toFixed(1);
    var sat=false;for(var i=0;i<4;i++){var rr=d.rpm[i];for(var j=Math.max(0,rr.length-100);j<rr.length;j++)if(rr[j]>=9990||rr[j]<=5){sat=true;break}}
    document.getElementById('rs').textContent=sat?'YES':'NO';
    document.getElementById('rs').className='vl '+(sat?'r':'g');
    var div=false;for(var i=0;i<n;i++)if(d.pz[i]<-100||(d.vx[i]&&Math.hypot(d.vx[i],d.vy[i],d.vz[i])>100))div=true;
    document.getElementById('ad').textContent=div?'YES':'NO';
    document.getElementById('ad').className='vl '+(div?'r':'g');

    var ex=[],ez=[];
    for(var i=0;i<n;i++){ex.push(Math.abs(d.px[i]-d.goal_x));ez.push(Math.abs(d.pz[i]-HOV_Z))}
    chart('cpe',[ex,ez],'x=green z=blue',0,3,0.3);
    chart('crp',[d.rpm[0],d.rpm[1],d.rpm[2],d.rpm[3]],'FL/FR/BL/BR',0,50,null);
    var ads = d.all_d||[];
    if(ads.length){
      // thin grey lines for each obstacle
      for(var k=0;k<ads.length;k++){
        var single = new Array(d.min_d.length).fill(ads[k]);
        chart('cmd',[single],'',0,2,0.4);
      }
      // thick red line for minimum
      chart('cmd',[d.min_d],'min(dist)',0,2,0.4);
    }

    // trajectory
    (function(){
      var c=document.getElementById('ctj');
      c.width=Math.max(500,c.parentElement.clientWidth-24);
      c.height=c.width*0.5;
      var w=c.width,h=c.height,ctx=c.getContext('2d');
      ctx.clearRect(0,0,w,h);
      ctx.strokeStyle='#21262d';ctx.lineWidth=1;
      for(var g=-1;g<=4;g++){var gx=(g+1)/6*w;ctx.beginPath();ctx.moveTo(gx,0);ctx.lineTo(gx,h);ctx.stroke()}
      for(var g=-1;g<=4;g++){var gy=h-(g+1)/6*h;ctx.beginPath();ctx.moveTo(0,gy);ctx.lineTo(w,gy);ctx.stroke()}
      var wx=function(v){return (v+1)/6*w}, wy=function(v){return h-(v+1)/6*h};
      if(n>0){
        ctx.fillStyle='#58a6ff';ctx.font='16px sans-serif';ctx.fillText('+',wx(d.px[0])-5,wy(d.py[0])+5);
        ctx.fillStyle='#f85149';ctx.font='16px sans-serif';ctx.fillText('+',wx(d.goal_x)-5,wy(d.goal_y)+5);
      }
      if(n>1){ctx.strokeStyle='#3fb950';ctx.lineWidth=2;ctx.beginPath();
        for(var i=0;i<n;i++){i==0?ctx.moveTo(wx(d.px[i]),wy(d.py[i])):ctx.lineTo(wx(d.px[i]),wy(d.py[i]))}
        ctx.stroke()}
      ctx.fillStyle='#8b949e';ctx.font='10px sans-serif';ctx.fillText('XY (+start +goal)',4,12);
    })();
  })
}
setInterval(update,500);
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
            with lk:
                d = {}
                for k in ['t','px','py','pz','vx','vy','vz','rpm','rpm_t','goal_x','goal_y','goal_z','min_d','min_d_t']:
                    d[k] = st[k]
                d['all_d'] = st.get('all_d', [])
                # 确保 goal 默认值非 0（控制器默认 hover z=1.5）
                if not d.get('goal_z'):
                    d['goal_z'] = 1.5
            self.wfile.write(json.dumps(d).encode())
        else:
            self.send_response(404)
            self.end_headers()
    def log_message(self, *a): pass

if __name__ == '__main__':
    print('live_monitor: http://localhost:8765 | Ctrl+C=CSV')
    for t in [th.Thread(target=f, daemon=True) for f in [r_odom, r_vel, r_rpm, r_goal, r_obs, r_min]]:
        t.start()
    time.sleep(2)

    # Start HTTP server with port conflict handling
    for retry in range(3):
        try:
            HTTPServer.allow_reuse_address = True
            srv = HTTPServer(('0.0.0.0', 8765), Handler)
            srv.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.serve_forever()
            break
        except OSError:
            time.sleep(1.0)
            sp.run(['fuser','-k','8765/tcp'], stdout=sp.DEVNULL, stderr=sp.DEVNULL)
    else:
        print('ERROR: could not bind port 8765')

    # Save CSV on exit
    with lk:
        with open(SF, 'w') as f:
            f.write("t,px,py,pz,vx,vy,vz,rpm0,rpm1,rpm2,rpm3,min_obs_dist\n")
            n = max(len(st['t']), len(st['rpm_t']), len(st['min_d_t']))
            for i in range(n):
                row = []
                for k in ['t','px','py','pz']:
                    row.append(str(st[k][i]) if i < len(st[k]) else '')
                for j in range(4):
                    row.append(str(st['rpm'][j][i]) if i < len(st['rpm'][j]) else '')
                row.append(str(st['min_d'][i]) if i < len(st['min_d']) else '')
                f.write(','.join(row) + '\n')
    print(f'Saved {SF} ({n} rows)')
