// ================= Jembatan JS <-> Python (Pyodide) =================
// Fisika & flight controller berjalan di Python (lihat <script id="py-src"> di index.html).
// JavaScript hanya menggambar, membaca keyboard, dan menampilkan HUD.
const rad=d=>d*Math.PI/180;
const $=id=>document.getElementById(id);
const clamp=(v,a,b)=>Math.min(b,Math.max(a,v));
const ARM=0.6, DM=ARM/Math.SQRT2, GZ=0.08;
const MPOS=[[DM,-DM],[DM,DM],[-DM,DM],[-DM,-DM]], MSPIN=[-1,1,-1,1];
let S={pos:[0,0,GZ],vel:[0,0,0],bx:[1,0,0],by:[0,1,0],bz:[0,0,1],w:[0,0,0],mot:[0,0,0,0],ang:[0,0,0,0],thr:0,armed:false,t:0};
let trail=[], mode='alt', windOn=false, chase=true, paused=false, msg='', msgT=0;
let pyFrame=null, pyPress=null, ready=false;
const keys=new Set();
const setMsg=(t,s=3)=>{msg=t;msgT=s};
function euler(){return{roll:Math.atan2(S.by[2],S.bz[2]),th:Math.asin(clamp(-S.bx[2],-1,1)),yaw:Math.atan2(S.bx[1],S.bx[0])}}

// Terima state dari Python: pos(3) vel(3) R(9, baris) w(3) motor(4) armed thr mode wind t pesan
function applyState(a){
  S.pos=[a[0],a[1],a[2]];S.vel=[a[3],a[4],a[5]];
  S.bx=[a[6],a[9],a[12]];S.by=[a[7],a[10],a[13]];S.bz=[a[8],a[11],a[14]];
  S.w=[a[15],a[16],a[17]];S.mot=a.slice(18,22);
  S.armed=a[22]>0.5;S.thr=a[23];mode=a[24]>0.5?'stab':'alt';windOn=a[25]>0.5;S.t=a[26];
  if(a[27])setMsg(a[27],4);
}
async function boot(){
  try{
    setMsg('Memuat Python (Pyodide + NumPy)...',999);
    const pyodide=await loadPyodide();
    await pyodide.loadPackage('numpy');
    const res=await fetch('main.py');                 // kode Python dimuat dari main.py
    if(!res.ok)throw new Error('main.py tidak ditemukan');
    pyodide.runPython(await res.text());
    pyFrame=pyodide.globals.get('frame');pyPress=pyodide.globals.get('press');
    ready=true;setMsg('Python siap. Tekan SPASI untuk arm.',4);
  }catch(err){setMsg(location.protocol==='file:'?"Jalankan 'python main.py' lalu buka halaman lewat server lokal":'Gagal memuat Python: '+err.message,999);console.error(err)}
}

// ================= Kamera & proyeksi =================
const cv=$('cv'),ctx=cv.getContext('2d');
let W=0,H=0,DPR=1;
const cam={x:0,y:0,z:0,az:45,off:0,el:22,zoom:1,cy:0};
function resize(){DPR=devicePixelRatio||1;W=cv.clientWidth;H=cv.clientHeight;cv.width=W*DPR;cv.height=H*DPR}
addEventListener('resize',resize);resize();
function project(p){
  const el=rad(cam.el),az=rad(cam.az),ce=Math.cos(el),se=Math.sin(el),ca=Math.cos(az),sa=Math.sin(az);
  const x=p[0]-cam.x,y=p[1]-cam.y,z=p[2]-cam.z;
  const sx=-sa*x+ca*y, sy=-se*ca*x-se*sa*y+ce*z, dp=ce*ca*x+ce*sa*y+se*z;
  const s=Math.min(W,H*1.3)/7*cam.zoom*30/(30-clamp(dp,-60,25));
  return[W/2+sx*s,H*0.58-sy*s,dp,s];
}
let drag=null;
cv.addEventListener('pointerdown',e=>{drag=[e.clientX,e.clientY];cv.setPointerCapture(e.pointerId);cv.focus();$('hint').style.display='none'});
cv.addEventListener('pointermove',e=>{if(!drag)return;const dx=e.clientX-drag[0],dy=e.clientY-drag[1];
  if(chase)cam.off-=dx*0.4;else cam.az-=dx*0.4;cam.el=clamp(cam.el+dy*0.3,4,85);drag=[e.clientX,e.clientY]});
cv.addEventListener('pointerup',()=>drag=null);
cv.addEventListener('wheel',e=>{e.preventDefault();cam.zoom=clamp(cam.zoom*(e.deltaY<0?1.08:1/1.08),.4,3)},{passive:false});

// ================= Input keyboard =================
const KMAP={arrowup:'up',arrowdown:'down',arrowleft:'left',arrowright:'right'};
const normKey=e=>{const k=e.key==' '?'space':e.key.toLowerCase();return KMAP[k]||k};
addEventListener('keydown',e=>{
  const k=normKey(e);
  if(['up','down','left','right','space'].includes(k))e.preventDefault();
  $('hint').style.display='none';
  if(e.repeat)return;
  keys.add(k);
  if(!ready)return;
  if(['space','r','m','g'].includes(k)){
    const m=pyPress(k);if(m)setMsg(m);
    if(k==='r')trail=[];
  }
  else if(k==='c'){chase=!chase;setMsg(chase?'Kamera mengikuti arah drone':'Kamera bebas')}
  else if(k==='p')paused=!paused;
});
addEventListener('keyup',e=>keys.delete(normKey(e)));
addEventListener('blur',()=>keys.clear());

// ================= Gambar =================
const css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const toWorld=l=>[0,1,2].map(i=>S.pos[i]+l[0]*S.bx[i]+l[1]*S.by[i]+l[2]*S.bz[i]);
function path(pts,close){ctx.beginPath();pts.forEach((p,i)=>{const q=project(p);i?ctx.lineTo(q[0],q[1]):ctx.moveTo(q[0],q[1])});if(close)ctx.closePath()}
function seg(a,b){const p=project(a),q=project(b);ctx.beginPath();ctx.moveTo(p[0],p[1]);ctx.lineTo(q[0],q[1])}

function draw(dtReal){
  ctx.setTransform(DPR,0,0,DPR,0,0);ctx.clearRect(0,0,W,H);
  const C={grid:css('--grid'),ink:css('--ink'),acc:css('--acc'),m:css('--m'),vel:css('--vel'),muted:css('--muted')};
  const e=euler();
  // Kamera
  const k=1-Math.exp(-dtReal*6);
  cam.x+=(S.pos[0]-cam.x)*k;cam.y+=(S.pos[1]-cam.y)*k;cam.z+=(S.pos[2]*0.6-cam.z)*k;
  if(chase){cam.cy+=Math.atan2(Math.sin(e.yaw-cam.cy),Math.cos(e.yaw-cam.cy))*(1-Math.exp(-dtReal*2.5));cam.az=cam.cy*180/Math.PI+180+cam.off}

  // Grid lantai (bergerak mengikuti kamera sebagai isyarat gerak)
  const R0=9,gx=Math.round(cam.x),gy=Math.round(cam.y);
  ctx.lineWidth=1;ctx.strokeStyle=C.grid;
  for(let i=-R0;i<=R0;i++){
    for(let j=-R0;j<R0;j++){
      const a=gx+i,b=gy+j;
      const f1=1-Math.hypot(a-cam.x,b+.5-cam.y)/R0, f2=1-Math.hypot(a+.5-cam.x,b-cam.y)/R0;
      if(f1>0){ctx.globalAlpha=f1*.55;seg([a,b,0],[a,b+1,0]);ctx.stroke()}
      if(f2>0){ctx.globalAlpha=f2*.55;seg([b,a,0],[b+1,a,0]);ctx.stroke()}
    }
  }
  ctx.globalAlpha=1;
  // Helipad
  ctx.lineWidth=2;ctx.strokeStyle=C.acc;
  const ring=[];for(let a=0;a<=24;a++)ring.push([1.1*Math.cos(a/24*6.2832),1.1*Math.sin(a/24*6.2832),0]);
  path(ring,true);ctx.stroke();
  seg([-.5,-.4,0],[-.5,.4,0]);ctx.stroke();seg([.5,-.4,0],[.5,.4,0]);ctx.stroke();seg([-.5,0,0],[.5,0,0]);ctx.stroke();

  // Tiang penanda tiap 6 m
  const qd=project(S.pos)[2],posts=[];
  for(let a=Math.floor((cam.x-12)/6)*6;a<=cam.x+12;a+=6)for(let b=Math.floor((cam.y-12)/6)*6;b<=cam.y+12;b+=6){
    if(a===0&&b===0)continue;const p=project([a,b,0]);posts.push({a,b,d:p[2]});
  }
  posts.sort((u,v)=>u.d-v.d);
  const drawPost=pt=>{ctx.strokeStyle=C.muted;ctx.lineWidth=3;seg([pt.a,pt.b,0],[pt.a,pt.b,3]);ctx.stroke();
    const t=project([pt.a,pt.b,3]);ctx.fillStyle=C.acc;ctx.beginPath();ctx.arc(t[0],t[1],4,0,7);ctx.fill()};
  posts.filter(p=>p.d<qd).forEach(drawPost);

  // Bayangan & garis ketinggian
  const alt=S.pos[2]-GZ;
  ctx.fillStyle=C.ink;ctx.globalAlpha=clamp(.28-alt*.03,.04,.28);
  MPOS.forEach(m=>{const c=toWorld([m[0],m[1],0]),pts=[];
    for(let a=0;a<14;a++)pts.push([c[0]+.28*Math.cos(a/14*6.2832),c[1]+.28*Math.sin(a/14*6.2832),0]);
    path(pts,true);ctx.fill()});
  ctx.globalAlpha=.5;ctx.strokeStyle=C.muted;ctx.lineWidth=1;ctx.setLineDash([4,4]);seg(S.pos,[S.pos[0],S.pos[1],0]);ctx.stroke();ctx.setLineDash([]);ctx.globalAlpha=1;

  // Jejak
  if(trail.length>1){ctx.strokeStyle=C.acc;ctx.globalAlpha=.5;ctx.lineWidth=1.5;path(trail,false);ctx.stroke();ctx.globalAlpha=1}

  // Quadcopter: lengan
  const sc=project(S.pos)[3];
  MPOS.forEach((m,i)=>{ctx.strokeStyle=i<2?C.acc:C.ink;ctx.lineWidth=Math.max(3,.05*sc);ctx.lineCap='round';seg(S.pos,toWorld([m[0],m[1],0]));ctx.stroke()});
  // Body (kotak, urut kedalaman)
  const h=.17,hz=.045,V=[];
  for(const z of[-hz,hz])for(const[x,y]of[[-h,-h],[h,-h],[h,h],[-h,h]])V.push(toWorld([x,y,z]));
  const faces=[[0,1,2,3,'b'],[4,5,6,7,'t'],[1,2,6,5,'f'],[0,3,7,4,'k'],[2,3,7,6,'l'],[0,1,5,4,'r']]
    .map(f=>({f,d:f.slice(0,4).reduce((s,i)=>s+project(V[i])[2],0)})).sort((a,b)=>a.d-b.d);
  faces.forEach(({f})=>{path(f.slice(0,4).map(i=>V[i]),true);
    ctx.fillStyle=f[4]==='t'?C.m:f[4]==='f'?C.acc:C.ink;ctx.globalAlpha=f[4]==='t'||f[4]==='f'?.95:.7;ctx.fill();ctx.globalAlpha=1;
    ctx.strokeStyle=C.ink;ctx.lineWidth=1;ctx.stroke()});
  // Propeller (disk transparan, makin pekat saat putaran naik) urut kedalaman
  const props=MPOS.map((m,i)=>({i,c:toWorld([m[0],m[1],.06])})).sort((a,b)=>project(a.c)[2]-project(b.c)[2]);
  props.forEach(({i,c})=>{
    S.ang[i]+=-MSPIN[i]*S.mot[i]*45*dtReal;
    const pts=[];for(let a=0;a<20;a++){const t=a/20*6.2832;pts.push(toWorld([MPOS[i][0]+.28*Math.cos(t),MPOS[i][1]+.28*Math.sin(t),.06]))}
    path(pts,true);ctx.fillStyle=i<2?C.acc:C.m;ctx.globalAlpha=.1+.3*S.mot[i];ctx.fill();ctx.globalAlpha=.8;ctx.strokeStyle=ctx.fillStyle;ctx.lineWidth=1.5;ctx.stroke();
    const an=S.ang[i],o=[.28*Math.cos(an),.28*Math.sin(an)];
    ctx.globalAlpha=.9;ctx.strokeStyle=C.ink;ctx.lineWidth=2;
    seg(toWorld([MPOS[i][0]+o[0],MPOS[i][1]+o[1],.06]),toWorld([MPOS[i][0]-o[0],MPOS[i][1]-o[1],.06]));ctx.stroke();ctx.globalAlpha=1;
  });
  // Vektor kecepatan
  if(Math.hypot(S.vel[0],S.vel[1],S.vel[2])>0.4){
    const t=[S.pos[0]+S.vel[0]*.4,S.pos[1]+S.vel[1]*.4,S.pos[2]+S.vel[2]*.4];
    ctx.strokeStyle=C.vel;ctx.lineWidth=2.5;seg(S.pos,t);ctx.stroke();const q=project(t);ctx.fillStyle=C.vel;ctx.beginPath();ctx.arc(q[0],q[1],4,0,7);ctx.fill();
  }
  posts.filter(p=>p.d>=qd).forEach(drawPost);
}

// ================= HUD =================
function hud(){
  const e=euler(),f=(v,u,d=2)=>v.toFixed(d)+u,deg=v=>v*180/Math.PI;
  const row=(a,b)=>`<div class="r"><span>${a}</span><span>${b}</span></div>`;
  const status=msgT>0?msg:(S.armed?'ARMED':'DISARMED');
  const col=S.armed?'var(--ok)':'var(--bad)';
  const thr=mode==='stab'?S.thr*100:S.mot.reduce((s,m)=>s+m*m,0)/4*100;
  $('hud').innerHTML=`<div id="st" style="color:${col}">${status}</div>`+
   row('Mode',mode==='alt'?'ALT HOLD':'STABILIZE')+row('Angin',windOn?'Aktif':'Mati')+
   '<h3>Posisi</h3>'+row('X',f(S.pos[0],' m'))+row('Y',f(S.pos[1],' m'))+row('Tinggi',f(S.pos[2]-GZ,' m'))+
   '<h3>Sikap</h3>'+row('Roll',f(deg(e.roll),'°',1))+row('Pitch',f(-deg(e.th),'°',1))+row('Yaw',f((deg(e.yaw)+360)%360,'°',1))+
   '<h3>Kecepatan</h3>'+row('Vx / Vy',f(S.vel[0],'',1)+' / '+f(S.vel[1],'',1))+row('Vz',f(S.vel[2],' m/s',1))+row('Jelajah',f(Math.hypot(S.vel[0],S.vel[1]),' m/s',1))+
   '<h3>Gas</h3>'+row(mode==='stab'?'Throttle':'Daya rata-rata',f(thr,' %',0));
  $('mot').innerHTML=S.mot.map((m,i)=>`<div class="mb"><div class="bar"><i style="height:${(m*m*100).toFixed(0)}%"></i></div>M${i+1}</div>`).join('');
}

// ================= Loop =================
let last=performance.now(),hudT=0;
function loop(now){
  const dt=Math.min(.05,(now-last)/1000);last=now;
  if(ready&&!paused){
    const out=pyFrame(Array.from(keys),dt);   // satu langkah simulasi di Python
    applyState(out.toJs());out.destroy();
    if(S.t%.05<dt)trail.push(S.pos.slice());
    if(trail.length>300)trail.shift();
  }
  if(msgT>0&&msgT<900)msgT-=dt;
  draw(paused?0:dt);
  hudT+=dt;if(hudT>.08){hud();hudT=0}
  requestAnimationFrame(loop);
}
cv.focus();
boot();
requestAnimationFrame(loop);