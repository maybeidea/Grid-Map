#pragma once

#include <ostream>
#include <string>

namespace plan_env {

// Self-contained viewer: no browser network access or Python plotting runtime.
// Palette and camera controls follow test/visualize_round_coverage.py.
inline void writeStandaloneVisualization(std::ostream &output,
                                         const std::string &data_json) {
  output << R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>plan_env / map snapshot</title>
<style>
:root{color-scheme:dark;--bg:#0b1220;--panel:#121a2b;--line:#243149;--text:#eaf0ff;--muted:#9aa8c7;--accent:#00e6c0}
*{box-sizing:border-box}html,body{height:100%;margin:0;background:var(--bg);color:var(--text);font-family:Inter,system-ui,sans-serif}
body{display:grid;grid-template-rows:auto minmax(0,1fr);overflow:hidden}
header{display:flex;align-items:center;gap:14px;flex-wrap:wrap;padding:11px 16px;border-bottom:1px solid var(--line);background:#101827}
h1{margin:0;font-size:15px}.toolbar{display:flex;gap:6px;flex-wrap:wrap;margin-left:auto}
button{min-height:32px;border:1px solid #34435e;border-radius:5px;background:#172238;color:var(--text);padding:0 10px;cursor:pointer;font:inherit;font-size:12px}
button:hover,button.active{border-color:var(--accent);background:#1c3042}button:disabled{opacity:.4;cursor:default}
main{display:grid;grid-template-columns:220px minmax(0,1fr) 220px;min-height:0}
aside{overflow:auto;padding:16px;background:var(--panel)}.metrics{border-right:1px solid var(--line)}.legend{border-left:1px solid var(--line)}
h2{margin:4px 0 12px;font-size:11px;color:var(--muted);text-transform:uppercase;letter-spacing:.08em}
.card{background:#1a2740;border:1px solid #243149;border-left:3px solid #3d5afe;border-radius:6px;padding:12px;margin-bottom:10px}
.card small{display:block;color:var(--muted);font-size:11px;margin-bottom:5px}.card strong{font-size:20px;font-weight:600}
.stats{display:grid;grid-template-columns:1fr auto;gap:9px;font:11px ui-monospace,monospace}.stats span:nth-child(odd){color:var(--muted)}.stats span{overflow-wrap:anywhere}
.stage{position:relative;min-width:0;min-height:0;background:#0d1626}canvas{display:block;width:100%;height:100%;touch-action:none;cursor:grab}canvas.dragging{cursor:grabbing}
.hud{position:absolute;left:12px;bottom:10px;color:var(--muted);font:11px ui-monospace,monospace;pointer-events:none}
.layers{display:grid;gap:12px;margin-bottom:22px}.layers label,.status{display:flex;align-items:center;gap:8px;font-size:12px}.count{margin-left:auto;color:var(--muted);font:11px ui-monospace,monospace}
.swatch{display:inline-block;width:11px;height:11px;border-radius:2px;flex-shrink:0}.range{display:grid;gap:10px;margin-bottom:20px;font-size:12px;color:var(--muted)}input{accent-color:var(--accent)}input[type=range]{width:100%}
.gradient{height:9px;background:linear-gradient(90deg,hsl(250 88% 58%),hsl(150 88% 62%),hsl(45 88% 66%));border-radius:3px}.endpoints{display:flex;justify-content:space-between;font:11px ui-monospace,monospace;color:var(--muted);margin-top:7px}
.help{color:var(--muted);font-size:12px;line-height:1.7;margin-top:22px}.status{margin-bottom:12px}[hidden]{display:none!important}
@media(max-width:1000px){main{grid-template-columns:180px minmax(0,1fr) 180px}aside{padding:12px}}
@media(max-width:760px){body{overflow:auto;display:block}header{padding:10px}.toolbar{margin:0}main{display:flex;flex-direction:column}.stage{height:60vh;min-height:300px;order:0}.metrics{order:1}.legend{order:2}aside{overflow:visible;border-top:1px solid var(--line)}.card{display:inline-block;min-width:140px;margin-right:8px}.stats{grid-template-columns:1fr 1fr}}
</style></head><body>
<header><h1 id="title"></h1><div class="toolbar">
<button id="mode3d" class="active">ESDF 3D</button><button id="mode2d">HeightMap</button>
<button id="reset">Reset</button><button data-view="top">Top</button><button data-view="front">Front</button><button data-view="side">Side</button><button id="spin">Auto rotate</button><button id="shot">Save PNG</button>
</div></header>
<main><aside class="metrics"><h2>Snapshot metrics</h2><div id="cards"></div><div id="stats" class="stats"></div></aside>
<div class="stage"><canvas id="scene" aria-label="Interactive map snapshot"></canvas><div id="hud" class="hud"></div></div>
<aside class="legend"><h2>Layers</h2><div class="layers">
<label><input id="roi" type="checkbox" checked><span class="swatch" style="background:#00e6c0"></span>Task ROI</label>
<label><input id="grid" type="checkbox" checked><span class="swatch" style="background:#9aa8c7"></span>Axes / grid</label>
</div><div id="esdf-legend"><h2>ESDF occupied · height</h2><div class="gradient"></div><div class="endpoints"><span id="zmin"></span><span id="zmax"></span></div>
<div class="range"><label for="size">Point size</label><input id="size" type="range" min="1" max="6" step="0.5" value="2"></div>
<p class="help">Known free and unknown voxels are omitted from the 3D view.</p></div>
<div id="height-legend" hidden><h2>HeightMap status</h2><div id="statuses"></div></div>
<p class="help">Drag to orbit · Shift/right drag to pan<br>Scroll to zoom · Top / Front / Side<br>HeightMap: drag to pan<br>Save PNG exports the current map view.</p>
</aside></main>
<script id="scene-data" type="application/json">)HTML";
  // A frame name may contain HTML-significant text. Keep it inside the JSON
  // script element, including when it contains a literal closing script tag.
  for (const char c : data_json) {
    if (c == '<') {
      output << "\\u003c";
    } else {
      output.put(c);
    }
  }
  output << R"HTML(</script><script>
"use strict";
const d=JSON.parse(document.getElementById('scene-data').textContent);
const canvas=document.getElementById('scene'),ctx=canvas.getContext('2d'),stage=canvas.parentElement;
const [nx,ny,nz]=d.dimensions,[ox,oy,oz]=d.origin,res=d.resolution;
const bounds=[nx*res,ny*res,nz*res],center=[ox+bounds[0]/2,oy+bounds[1]/2,oz+bounds[2]/2];
const radius=Math.max(Math.hypot(...bounds)/2,res),roi=d.roi||[];
const names=['ALL_OPEN','BASE_OPEN','TOOL_OPEN','TOO_LOW','NOT_FLOOR','UNKNOWN'];
const colors=['#2f9e44','#74c0fc','#ffd43b','#f03e3e','#a61e4d','#9775fa'];
const heightCounts=Array(6).fill(0),voxelCounts=[0,0,0];
const points=[];
let occupied=0,zmin=Infinity,zmax=-Infinity;
for(let i=0;i<d.occupancy.length;i++){
  const state=d.occupancy_state[i];
  voxelCounts[state==='0'?0:state==='1'?1:2]++;
  if(d.occupancy[i]===1)occupied++;
}
// Bound interactive rendering work while retaining full data and exact counts.
const stride=Math.max(1,Math.ceil(occupied/30000));
let seen=0;
for(let i=0;i<d.occupancy.length;i++){
  if(d.occupancy[i]!==1)continue;
  const x=Math.floor(i/(ny*nz)),y=Math.floor(i/nz)%ny,z=i%nz;
  const p=[ox+(x+.5)*res,oy+(y+.5)*res,oz+(z+.5)*res];
  zmin=Math.min(zmin,p[2]);zmax=Math.max(zmax,p[2]);
  if(seen++%stride===0)points.push(p);
}
if(!occupied){zmin=oz;zmax=oz+bounds[2]}
const raster=document.createElement('canvas');raster.width=nx;raster.height=ny;
const rasterCtx=raster.getContext('2d'),pixels=rasterCtx.createImageData(nx,ny);
for(let y=0;y<ny;y++)for(let x=0;x<nx;x++){
  const v=d.heightmap[y*nx+x],k=[0,25,50,75,100].indexOf(v),status=k<0?5:k;
  heightCounts[status]++;
  const rgb=parseInt(colors[status].slice(1),16),i=((ny-1-y)*nx+x)*4;
  pixels.data.set([rgb>>16,(rgb>>8)&255,rgb&255,255],i);
}
rasterCtx.putImageData(pixels,0,0);
const percent=(n,total)=>total?(100*n/total).toFixed(1)+'%':'0.0%';
const number=n=>n.toLocaleString();
document.getElementById('title').textContent=`plan_env / map v${d.version} / ${d.frame_id}`;
document.title=`plan_env / map v${d.version}`;
for(const [label,value] of [
  ['ESDF occupied voxels',number(occupied)],
  ['Known occupancy',percent(voxelCounts[0]+voxelCounts[1],d.occupancy.length)],
  ['HeightMap observed',percent(nx*ny-heightCounts[5],nx*ny)]
]){
  const card=document.createElement('div');card.className='card';
  const small=document.createElement('small'),strong=document.createElement('strong');
  small.textContent=label;strong.textContent=value;card.append(small,strong);document.getElementById('cards').append(card);
}
for(const [label,value] of [
  ['Version',d.version],['Frame',d.frame_id],['Resolution',res.toFixed(3)+' m'],
  ['Dimensions',`${nx} × ${ny} × ${nz}`],['ESDF',d.esdf_current?'current':'reused'],
  ['Displayed points',number(points.length)],['Occupancy free',number(voxelCounts[0])],
  ['Occupancy occupied',number(voxelCounts[1])],['Occupancy unknown',number(voxelCounts[2])]
]){
  const key=document.createElement('span'),val=document.createElement('span');
  key.textContent=label;val.textContent=value;document.getElementById('stats').append(key,val);
}
names.forEach((name,k)=>{
  const row=document.createElement('div');row.className='status';
  const swatch=document.createElement('span'),label=document.createElement('span'),count=document.createElement('span');
  swatch.className='swatch';swatch.style.background=colors[k];label.textContent=name;
  count.className='count';count.textContent=number(heightCounts[k]);row.append(swatch,label,count);document.getElementById('statuses').append(row);
});
document.getElementById('zmin').textContent=zmin.toFixed(3)+' m';
document.getElementById('zmax').textContent=zmax.toFixed(3)+' m';
const state={mode:'3d',yaw:-.78,pitch:.58,zoom:1,panX:0,panY:0,pointSize:2,spin:false,drag:null};
let width=1,height=1,scale=1;
function resize(){
  const rect=stage.getBoundingClientRect(),dpr=Math.min(window.devicePixelRatio||1,2);
  width=Math.max(1,rect.width);height=Math.max(1,rect.height);
  canvas.width=Math.round(width*dpr);canvas.height=Math.round(height*dpr);ctx.setTransform(dpr,0,0,dpr,0,0);draw();
}
function project(p){
  const x=p[0]-center[0],y=p[1]-center[1],z=p[2]-center[2];
  const cy=Math.cos(state.yaw),sy=Math.sin(state.yaw),cp=Math.cos(state.pitch),sp=Math.sin(state.pitch);
  const rx=cy*x-sy*y,ry=sy*x+cy*y;
  return [width/2+state.panX+rx*scale,height/2+state.panY-(cp*ry+sp*z)*scale,-sp*ry+cp*z];
}
function line(points,color,lineWidth=1,dash=[]){
  if(points.length<2)return;ctx.beginPath();
  points.forEach((p,i)=>{const q=project(p);if(i)ctx.lineTo(q[0],q[1]);else ctx.moveTo(q[0],q[1])});
  ctx.strokeStyle=color;ctx.lineWidth=lineWidth;ctx.setLineDash(dash);ctx.stroke();ctx.setLineDash([]);
}
function drawGrid(){
  for(let i=0;i<=5;i++){
    const x=ox+bounds[0]*i/5,y=oy+bounds[1]*i/5;
    line([[x,oy,oz],[x,oy+bounds[1],oz]],'#243149');
    line([[ox,y,oz],[ox+bounds[0],y,oz]],'#243149');
    const labels=[[[x,oy,oz],x.toFixed(2),3,15]];
    if(i>0)labels.push([[ox,y,oz],y.toFixed(2),-38,3]);
    for(const [p,label,dx,dy] of labels){
      const q=project(p);ctx.fillStyle='#9aa8c7';ctx.font='10px ui-monospace,monospace';ctx.fillText(label,q[0]+dx,q[1]+dy);
    }
  }
  const axes=[[[ox+bounds[0],oy,oz],'#ff6b6b','x (m)'],[[ox,oy+bounds[1],oz],'#51cf66','y (m)']];
  if(state.mode==='3d')axes.push([[ox,oy,oz+bounds[2]],'#74c0fc','z (m)']);
  for(const [p,color,label] of axes){line([[ox,oy,oz],p],color,1.4);const q=project(p);ctx.fillStyle=color;ctx.fillText(label,q[0]+6,q[1]-6)}
}
function drawRoi(){
  if(roi.length<3)return;
  const closed=[...roi,roi[0]],base=closed.map(p=>[p[0],p[1],oz]);
  line(base,'#00e6c0',2);
  if(state.mode==='3d'){
    const top=closed.map(p=>[p[0],p[1],oz+bounds[2]]);
    line(top,'rgba(0,230,192,.65)',1,[5,4]);
    for(let i=0;i<roi.length;i++)line([base[i],top[i]],'rgba(0,230,192,.28)');
  }
}
function draw(){
  scale=(state.mode==='2d'?Math.min(width/(bounds[0]+res),height/(bounds[1]+res))*.78:Math.min(width,height)/(2.35*radius))*state.zoom;
  ctx.fillStyle='#0d1626';ctx.fillRect(0,0,width,height);
  if(state.mode==='2d'){
    const corner=project([ox,oy+bounds[1],oz]);ctx.imageSmoothingEnabled=false;
    ctx.drawImage(raster,corner[0],corner[1],bounds[0]*scale,bounds[1]*scale);
  }
  if(document.getElementById('grid').checked)drawGrid();
  if(state.mode==='3d'){
    const projected=points.map(p=>[...project(p),p[2]]).sort((a,b)=>a[2]-b[2]);
    ctx.globalAlpha=.72;
    for(const p of projected){
      const t=Math.max(0,Math.min(1,(p[3]-zmin)/Math.max(1e-9,zmax-zmin)));
      ctx.fillStyle=`hsl(${250-205*t} 88% ${58+8*t}%)`;
      ctx.fillRect(p[0]-state.pointSize/2,p[1]-state.pointSize/2,state.pointSize,state.pointSize);
    }ctx.globalAlpha=1;
    if(!occupied){ctx.fillStyle='#9aa8c7';ctx.font='14px system-ui';ctx.fillText('No occupied ESDF voxels in this snapshot',20,30)}
  }
  if(document.getElementById('roi').checked)drawRoi();
  document.getElementById('hud').textContent=`${state.mode==='2d'?'HeightMap · x / y (m)':'ESDF 3D · height (m)'}   zoom ${state.zoom.toFixed(2)}×`;
}
function stopSpin(){state.spin=false;document.getElementById('spin').classList.remove('active')}
function reset(){stopSpin();Object.assign(state,{yaw:state.mode==='2d'?0:-.78,pitch:state.mode==='2d'?0:.58,zoom:1,panX:0,panY:0});draw()}
function mode(value){
  state.mode=value;document.getElementById('mode3d').classList.toggle('active',value==='3d');document.getElementById('mode2d').classList.toggle('active',value==='2d');
  document.getElementById('esdf-legend').hidden=value==='2d';document.getElementById('height-legend').hidden=value==='3d';
  document.getElementById('spin').disabled=value==='2d';
  for(const button of document.querySelectorAll('[data-view]'))button.disabled=value==='2d';reset();
}
document.getElementById('mode3d').onclick=()=>mode('3d');document.getElementById('mode2d').onclick=()=>mode('2d');
document.getElementById('reset').onclick=reset;
for(const id of ['roi','grid'])document.getElementById(id).onchange=draw;
document.getElementById('size').oninput=e=>{state.pointSize=Number(e.target.value);draw()};
for(const button of document.querySelectorAll('[data-view]'))button.onclick=()=>{
  stopSpin();state.panX=state.panY=0;state.zoom=1;state.yaw=button.dataset.view==='side'?Math.PI/2:0;state.pitch=button.dataset.view==='top'?0:Math.PI/2;draw();
};
canvas.addEventListener('pointerdown',e=>{stopSpin();canvas.setPointerCapture(e.pointerId);state.drag={x:e.clientX,y:e.clientY,pan:state.mode==='2d'||e.button===2||e.shiftKey};canvas.classList.add('dragging')});
canvas.addEventListener('pointermove',e=>{
  if(!state.drag)return;const dx=e.clientX-state.drag.x,dy=e.clientY-state.drag.y;
  state.drag.x=e.clientX;state.drag.y=e.clientY;
  if(state.drag.pan){state.panX+=dx;state.panY+=dy}else{state.yaw+=dx*.009;state.pitch+=dy*.009}draw();
});
for(const event of ['pointerup','pointercancel','lostpointercapture'])canvas.addEventListener(event,()=>{state.drag=null;canvas.classList.remove('dragging')});
canvas.addEventListener('contextmenu',e=>e.preventDefault());
canvas.addEventListener('wheel',e=>{e.preventDefault();state.zoom=Math.max(.15,Math.min(12,state.zoom*Math.exp(-e.deltaY*.001)));draw()},{passive:false});
let then=0;
document.getElementById('spin').onclick=()=>{
  state.spin=!state.spin;document.getElementById('spin').classList.toggle('active',state.spin);
  if(state.spin){then=performance.now();requestAnimationFrame(animate)}
};
function animate(now){if(!state.spin)return;state.yaw+=(now-then)*.00018;then=now;draw();requestAnimationFrame(animate)}
document.getElementById('shot').onclick=()=>{const a=document.createElement('a');a.download=`map_v${d.version}_${state.mode==='2d'?'heightmap':'esdf3d'}_view.png`;a.href=canvas.toDataURL('image/png');a.click()};
new ResizeObserver(resize).observe(stage);resize();
</script></body></html>
)HTML";
}

}  // namespace plan_env
