// ── Seeded RNG ────────────────────────────────────────────────────
function mulberry32(seed){let s=seed;return()=>{s|=0;s=s+0x6D2B79F5|0;let t=Math.imul(s^s>>>15,1|s);t=t+Math.imul(t^t>>>7,61|t)^t;return((t^t>>>14)>>>0)/4294967296};}
// ── Helpers ───────────────────────────────────────────────────────
function clamp(v,lo,hi){return Math.max(lo,Math.min(hi,v));}
function heatColor(t){t=clamp(t,0,1);const r=t<.5?Math.round(29+210*t*2):Math.round(239-13*(t-.5)*2);const g=Math.round(158+1*t);const b=t<.5?Math.round(117-78*t*2):Math.round(39+35*(t-.5)*2);return`rgb(${r},${g},${b})`;}
const COLORS=['#1D9E75','#7F77DD','#3B8BD4','#00A896','#EF9F27','#E24B4A'];

// ── Generate fixed scene ──────────────────────────────────────────
const TRUE_CONES=[[5,2.5],[5,-2.5],[9,2.5],[9,-2.5],[13,0]];
function genScene(){
  const rng=mulberry32(42); const pts=[];
  for(const [cx,cy] of TRUE_CONES){
    const n=10+Math.floor(rng()*10);
    for(let i=0;i<n;i++) pts.push({x:cx+(rng()-.5)*.22,y:cy+(rng()-.5)*.22,z:.05+rng()*.30,int:2500+rng()*2500,cone:true});
  }
  // wall
  for(let i=0;i<25;i++) pts.push({x:-2+rng()*18,y:5.5+rng()*.4,z:.01,int:40+rng()*80,cone:false});
  // ground
  for(let i=0;i<50;i++) pts.push({x:-2+rng()*20,y:-4+rng()*10,z:-.05+rng()*.04,int:10+rng()*40,cone:false});
  return pts;
}
const SCENE=genScene();

// ── DBSCAN ────────────────────────────────────────────────────────
function dbscan(pts,eps,minPts){
  const n=pts.length; const labels=new Array(n).fill(-1); let C=-1;
  const vis=new Array(n).fill(false);
  const nb=i=>{const r=[];for(let j=0;j<n;j++){const dx=pts[i].x-pts[j].x;const dy=pts[i].y-pts[j].y;if(dx*dx+dy*dy<=eps*eps)r.push(j);}return r;};
  for(let i=0;i<n;i++){
    if(vis[i])continue;vis[i]=true;const q=nb(i);
    if(q.length<minPts){labels[i]=-1;continue;}
    C++;labels[i]=C;const stack=[...q];
    while(stack.length){const j=stack.pop();if(!vis[j]){vis[j]=true;const q2=nb(j);if(q2.length>=minPts)stack.push(...q2);}if(labels[j]===-1)labels[j]=C;}
  }
  return{labels,nClusters:C+1};
}

// ── World to canvas ───────────────────────────────────────────────
// world: x in [-2,18], y in [-5,5.5]  canvas: W x H
function w2c(x,y,W,H){return{px:(x+2)/20*W,py:(1-(y+5)/11)*H};}

// ── Main draw ─────────────────────────────────────────────────────
const canvas=document.getElementById('mainCanvas');
const ctx=canvas.getContext('2d');

function resize(){canvas.width=canvas.offsetWidth;canvas.height=canvas.offsetHeight;}

function drawGrid(){
  const W=canvas.width,H=canvas.height;
  ctx.strokeStyle='#141428';ctx.lineWidth=1;ctx.beginPath();
  const xs=[-2,0,2,4,6,8,10,12,14,16,18];
  const ys=[-4,-2,0,2,4];
  for(const x of xs){const{px}=w2c(x,0,W,H);ctx.moveTo(px,0);ctx.lineTo(px,H);}
  for(const y of ys){const{py}=w2c(0,y,W,H);ctx.moveTo(0,py);ctx.lineTo(W,py);}
  ctx.stroke();
  // axis labels
  ctx.fillStyle='#333355';ctx.font='9px JetBrains Mono';ctx.textAlign='center';
  for(const x of[0,5,10,15]){const{px,py}=w2c(x,-4,W,H);ctx.fillText(x+'m',px,py+12);}
}

function drawLidar(theta,W,H){
  const ox=w2c(0,0,W,H).px, oy=w2c(0,0,W,H).py;
  const R=Math.min(W,H)*0.7;
  // FOV sweep fill
  ctx.fillStyle='rgba(127,119,221,0.04)';
  ctx.beginPath();ctx.moveTo(ox,oy);
  const steps=Math.max(2,Math.round(theta));
  for(let i=0;i<=steps;i++){
    const a=(i/steps)*theta;
    const rad=a*Math.PI/180-Math.PI/2;
    ctx.lineTo(ox+R*Math.cos(rad),oy+R*Math.sin(rad));
  }
  ctx.closePath();ctx.fill();
  // sweep edge beam
  const edgeRad=(theta-5)*Math.PI/180-Math.PI/2;
  ctx.strokeStyle='rgba(239,159,39,0.6)';ctx.lineWidth=1.5;ctx.setLineDash([4,6]);
  ctx.beginPath();ctx.moveTo(ox,oy);ctx.lineTo(ox+R*Math.cos(edgeRad),oy+R*Math.sin(edgeRad));ctx.stroke();ctx.setLineDash([]);
  // origin
  ctx.fillStyle='#7F77DD';ctx.beginPath();ctx.arc(ox,oy,6,0,Math.PI*2);ctx.fill();
  ctx.fillStyle='#7F77DD';ctx.font='bold 10px JetBrains Mono';ctx.textAlign='left';
  ctx.fillText('LiDAR',ox+9,oy-6);
  // theta label
  ctx.fillStyle='#EF9F27';ctx.font='11px JetBrains Mono';
  ctx.fillText(`θ = ${theta}°`,ox+9,oy+8);
}

function ptInSweep(x,y,theta){
  // angle of point from LiDAR origin, measured from -Y axis (forward)
  if(x===0&&y===0)return false;
  let a=Math.atan2(x,y)*180/Math.PI; // angle from Y axis
  if(a<0)a+=360;
  return a<=theta;
}

function draw(){
  resize();
  const W=canvas.width,H=canvas.height;
  ctx.fillStyle='#080812';ctx.fillRect(0,0,W,H);
  drawGrid();

  const theta=parseFloat(document.getElementById('sTheta').value);
  const eps=parseFloat(document.getElementById('sEps').value);
  const speed=parseFloat(document.getElementById('sSpd').value);
  const T=1.0;
  const distPlaceFwd=speed*T*0.5;

  // ── Stage 1: Sweep — filter pts by angle ──────────────────────
  const visiblePts=SCENE.filter(p=>ptInSweep(p.x,p.y,theta));
  const totalPts=SCENE.length;
  document.getElementById('sv1').textContent=`${visiblePts.length} / ${totalPts} pts collected`;
  st('st1',true,'var(--amber)');

  // ── Stage 2: Ground removal ────────────────────────────────────
  const zSorted=[...visiblePts].map(p=>p.z).sort((a,b)=>a-b);
  const gndZ=zSorted[Math.floor(zSorted.length*.10)]??-0.05;
  const gndPts=visiblePts.filter(p=>p.z<=gndZ+0.1);
  const fgPts=visiblePts.filter(p=>p.z>gndZ+0.1);
  document.getElementById('sv2').textContent=`${fgPts.length} foreground, ${gndPts.length} ground removed`;
  st('st2',fgPts.length>0,'var(--blue)');

  // draw ground pts (muted)
  for(const p of gndPts){
    const{px,py}=w2c(p.x,p.y,W,H);
    ctx.fillStyle='#333355';ctx.beginPath();ctx.arc(px,py,2,0,Math.PI*2);ctx.fill();
  }

  // ── Stage 3: DBSCAN ───────────────────────────────────────────
  let labels=[],nClusters=0;
  if(fgPts.length>0){const r=dbscan(fgPts,eps,2);labels=r.labels;nClusters=r.nClusters;}
  document.getElementById('sv3').textContent=`${nClusters} cluster${nClusters!==1?'s':''} (ε = ${eps.toFixed(2)} m)`;
  st('st3',nClusters>0,'var(--purple)');

  // draw raw fg pts with cluster colour
  for(let i=0;i<fgPts.length;i++){
    const p=fgPts[i];
    const{px,py}=w2c(p.x,p.y,W,H);
    const col=labels[i]>=0?COLORS[labels[i]%COLORS.length]:'#555566';
    ctx.fillStyle=col;ctx.beginPath();ctx.arc(px,py,3,0,Math.PI*2);ctx.fill();
  }

  // ── Stage 4: Validate clusters ────────────────────────────────
  let validCount=0;
  const validCones=[];
  for(let ci=0;ci<nClusters;ci++){
    const cl=fgPts.filter((_,i)=>labels[i]===ci);
    if(!cl.length)continue;
    const xs=cl.map(p=>p.x),ys=cl.map(p=>p.y),zs=cl.map(p=>p.z);
    const w=Math.max(...xs)-Math.min(...xs);
    const ht=Math.max(...zs)-Math.min(...zs);
    const mx=xs.reduce((s,v)=>s+v,0)/cl.length;
    const my=ys.reduce((s,v)=>s+v,0)/cl.length;
    const d=Math.sqrt(mx*mx+my*my);
    const thr=Math.max(20,20000/(d*d||1));
    const avgI=cl.reduce((s,p)=>s+p.int,0)/cl.length;
    const valid=w>=0.03&&w<=0.80&&ht<=0.60&&avgI>=thr*0.8&&cl.length>=2;
    if(valid){validCount++;validCones.push({mx,my,col:COLORS[ci%COLORS.length]});}
  }
  document.getElementById('sv4').textContent=`${validCount} valid cone${validCount!==1?'s':''}`;
  st('st4',validCount>0,'var(--green)');

  // draw true cone markers (triangles) at validated positions
  for(const vc of validCones){
    const{px,py}=w2c(vc.mx,vc.my,W,H);
    // dashed circle
    ctx.strokeStyle=vc.col;ctx.lineWidth=1.5;ctx.setLineDash([3,3]);
    ctx.beginPath();ctx.arc(px,py,13,0,Math.PI*2);ctx.stroke();ctx.setLineDash([]);
    // triangle marker
    ctx.fillStyle=vc.col;
    ctx.beginPath();ctx.moveTo(px,py-11);ctx.lineTo(px-7,py+5);ctx.lineTo(px+7,py+5);ctx.fill();
  }

  // ── Stage 5: Scan distortion ──────────────────────────────────
  const distErr=distPlaceFwd;
  document.getElementById('sv5').textContent=distErr>0?`${distErr.toFixed(3)} m forward shift`:'No distortion at rest';
  st('st5',speed>0,'var(--red)');

  // draw distorted (observed) positions — shifted backward in X
  if(distPlaceFwd>0.01){
    for(const vc of validCones){
      const ox2=vc.mx-distPlaceFwd;
      const{px,py}=w2c(vc.mx,vc.my,W,H);
      const{px:opx,py:opy}=w2c(ox2,vc.my,W,H);
      // error line
      ctx.strokeStyle='rgba(226,75,74,0.5)';ctx.lineWidth=1;ctx.setLineDash([3,4]);
      ctx.beginPath();ctx.moveTo(px,py);ctx.lineTo(opx,opy);ctx.stroke();ctx.setLineDash([]);
      // observed dot
      ctx.fillStyle='#E24B4A';ctx.beginPath();ctx.arc(opx,opy,5,0,Math.PI*2);ctx.fill();
      // error label
      ctx.fillStyle='#E24B4A';ctx.font='9px JetBrains Mono';ctx.textAlign='center';
      ctx.fillText(`${distPlaceFwd.toFixed(2)}m`,opx,opy-10);
    }
  }

  // draw sweep
  drawLidar(theta,W,H);

  // ── Result pill ───────────────────────────────────────────────
  const rb=document.getElementById('resultBox');
  document.getElementById('rVal').textContent=distErr.toFixed(3)+' m';
  if(distErr===0){rb.className='result pass';document.getElementById('rDesc').textContent='No scan distortion at rest. Move speed slider to see distortion.';}
  else if(distErr<=0.2){rb.className='result pass';document.getElementById('rDesc').textContent='Within 0.20 m objective. Speed is acceptable.';}
  else if(distErr<=1.0){rb.className='result warn';document.getElementById('rDesc').textContent='Exceeds FSAI objective (≤0.20 m). Cone positions are offset.';}
  else{rb.className='result fail';document.getElementById('rDesc').textContent=`Severe distortion — detected cones displaced ${distErr.toFixed(2)} m from truth.`;}

  // update DOM labels
  document.getElementById('vTheta').textContent=theta+'°';
  document.getElementById('vEps').textContent=eps.toFixed(2)+' m';
  document.getElementById('vSpd').textContent=speed.toFixed(1)+' m/s';
}

function st(id,on,col){
  const el=document.getElementById(id);
  el.classList.toggle('active',on);
  el.style.borderColor=on?col:'#333355';
}

['sTheta','sEps','sSpd'].forEach(id=>document.getElementById(id).addEventListener('input',draw));
window.addEventListener('resize',draw);
setTimeout(draw,60);
