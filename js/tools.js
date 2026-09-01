// ============================================================================
//  §13 · TOOLS & INPUT
//  The tool palette and all pointer handling: hit-testing, snapping, constraint
//  handle editing, and the big pointer-down dispatch that builds constraints.
//    §13.1  tool table + rail build + setTool
//    §13.2  picking & snapping (pickBody/Constraint, distSeg, snapAnchor, ...)
//    §13.3  constraint handles (conHandles, pickHandle, applyHandle)
//    §13.4  pointer state (multi-touch map, pinch, cancelSingle)
//    §13.5  pointerdown  (per-tool dispatch -- where constraints are created)
//    §13.6  pointermove  (drag/pan/pinch/handle articulation)
//    §13.7  pointerup / cancel / wheel
// ============================================================================
// ---- §13.1 · tool table + rail build + setTool ----
const TOOLS=[
  {id:'select',key:'1',tip:'Select / move (1)',svg:'<path d="M5 3l7 16 2-6 6-2z"/>'},
  {id:'body',key:'2',tip:'Add body (2)',svg:'<circle cx="12" cy="12" r="7"/><path d="M12 8v8M8 12h8"/>'},
  {id:'pin',key:'3',tip:'Pin / hinge (3)',svg:'<circle cx="12" cy="12" r="3"/><circle cx="12" cy="12" r="8"/>'},
  {id:'rod',key:'4',tip:'Rigid rod (4)',svg:'<circle cx="6" cy="18" r="2.4"/><circle cx="18" cy="6" r="2.4"/><path d="M7.5 16.5l9-9"/>'},
  {id:'slot',key:'5',tip:'Slot / prismatic (5)',svg:'<path d="M3 9h18M3 15h18"/><rect x="9" y="9" width="6" height="6" rx="1"/>'},
  {id:'belt',key:'b',tip:'Belt (b)',svg:'<circle cx="7" cy="12" r="4"/><circle cx="17" cy="12" r="4"/><path d="M7 8h10M7 16h10"/>'},
  {id:'knife',key:'k',tip:'Knife-edge wheel (k)',svg:'<path d="M4 16h16"/><path d="M12 16l3-9 3 9"/><circle cx="8" cy="16" r="1.5"/>'},
  {id:'cvt',key:'v',tip:'Variable gear / CVT (v)',svg:'<circle cx="9" cy="12" r="6"/><circle cx="17" cy="12" r="3"/><path d="M9 12h8"/>'},
  {id:'cable',key:'c',tip:'Cable (c)',svg:'<circle cx="16" cy="9" r="4"/><path d="M4 19c6 0 8-4 9-7"/><circle cx="4" cy="19" r="1.5"/>'},
  {id:'gas',key:'6',tip:'Gas piston (6)',svg:'<rect x="3" y="8" width="11" height="8" rx="1"/><path d="M14 10.5v3M14 12h7M18 9.5v5"/>'},
  {id:'delete',key:'7',tip:'Delete (7)',svg:'<path d="M6 7h12M9 7V5h6v2M8 7l1 12h6l1-12"/>'},
];
let tool='select';
const rail=document.getElementById('rail');
TOOLS.forEach((t,i)=>{
  if(i===1||i===2||i===9){ const s=document.createElement('div');s.className='rail-sep';rail.appendChild(s);}
  const el=document.createElement('button');el.className='tool';el.dataset.id=t.id;
  el.innerHTML=`<svg viewBox="0 0 24 24">${t.svg}</svg><span class="kbd">${t.key}</span><span class="tip">${t.tip}</span>`;
  el.onclick=()=>setTool(t.id); rail.appendChild(el);
});
function setTool(id){ tool=id; pending=null; bodyPreview=null; hover=null; hoverHandle=null; hoverSnap=null;
  document.querySelectorAll('.tool').forEach(e=>e.classList.toggle('on',e.dataset.id===id));
  cv.style.cursor = id==='select'?'default': id==='delete'?'not-allowed':'crosshair';
  document.getElementById('modehint').textContent = TOOLS.find(t=>t.id===id).tip;
}

// ---- §13.2 · picking & snapping ----
let mouseScreen=[0,0], mouseWorld=[0,0], drag=null, panning=null;

function pickBody(wx,wy){
  for(let i=bodies.length-1;i>=0;i--){ const b=bodies[i];
    if((wx-b.x)**2+(wy-b.y)**2 <= b.r*b.r) return i; }
  return -1;
}
// topmost body under the cursor that isn't `exceptId` -- lets the 2nd pin
// pick reach a body occluded by the one already selected
function pickBodyExcept(wx,wy,exceptId){
  for(let i=bodies.length-1;i>=0;i--){ const b=bodies[i]; if(b.id===exceptId) continue;
    if((wx-b.x)**2+(wy-b.y)**2 <= b.r*b.r) return i; }
  return -1;
}
// per-constraint hit test, factored out of pickConstraint so the pointerdown
// dispatch can prioritise a *specific* (already-selected) constraint over
// whatever else happens to be topmost at the same point (§13.5).
function constraintHit(con,wx,wy){
  const tol=10/cam.scale;
  if(con.type==='rod'){
    const [ax,ay]=epWorld(con.a), [bx,by]=epWorld(con.b);
    return distSeg(wx,wy,ax,ay,bx,by)<=tol;
  }
  if(con.type==='slot'){
    // The rail is an infinite line (drawn viewport-spanning) -- hit-test
    // perpendicular distance to it, not a bounded segment.
    const [ax,ay]=epWorld(con.a), [bx,by]=epWorld(con.b);
    const railAngle=slotRailAngle(con);
    const nx=-Math.sin(railAngle), ny=Math.cos(railAngle);
    const midx=(ax+bx)/2, midy=(ay+by)/2;
    return Math.abs(nx*(wx-midx)+ny*(wy-midy))<=tol;
  }
  const A=bodies[bodyIndex(con.a.id)]; if(!A) return false;
  const [ax,ay]=con.a.off?worldPt(A,con.a.off):[A.x,A.y];
  if(con.type==='pin'){ return (wx-ax)**2+(wy-ay)**2<=tol*tol; }
  if(con.type==='belt'||con.type==='cvt'){ const B=bodies[bodyIndex(con.b.id)]; if(!B) return false;
    return distSeg(wx,wy,A.x,A.y,B.x,B.y)<=tol; }
  if(con.type==='knife'){ const hh=R(A.th,con.dir[0],con.dir[1]); const hl=Math.hypot(hh[0],hh[1])||1;
    const p1=[ax-hh[0]/hl*0.5,ay-hh[1]/hl*0.5], p2=[ax+hh[0]/hl*0.5,ay+hh[1]/hl*0.5];
    return distSeg(wx,wy,p1[0],p1[1],p2[0],p2[1])<=tol; }
  return false;
}
function pickConstraint(wx,wy){
  for(let i=constraints.length-1;i>=0;i--){ if(constraintHit(constraints[i],wx,wy)) return i; }
  return -1;
}
// per-gas / per-cable hit tests, mirroring pickGas/pickCable (inspector.js
// §14.1) -- used the same way, to test one specific already-selected object.
function gasHit(g,wx,wy){ const tol=12/cam.scale; const f=gasFrame(g);
  return distSeg(wx,wy,f.hx,f.hy,f.pax,f.pay)<=tol; }
function cableHit(cb,wx,wy){ const tol=10/cam.scale; const f=cableFrame(cb); if(!f) return false;
  return distSeg(wx,wy,f.T[0],f.T[1],f.Qx,f.Qy)<=tol; }
// perimeter (rim) hit test on a specific body -- the drag handle for radius
// resizing (§13.5/§13.6). A separate tolerance ring around the rim, distinct
// from the filled-disk `pickBody` test used for moving the body.
function bodyRimHit(b,wx,wy){ const tol=10/cam.scale;
  return Math.abs(Math.hypot(wx-b.x,wy-b.y)-b.r)<=tol; }
function distSeg(px,py,ax,ay,bx,by){ const dx=bx-ax,dy=by-ay; const L2=dx*dx+dy*dy||1e-9;
  let t=((px-ax)*dx+(py-ay)*dy)/L2; t=Math.max(0,Math.min(1,t));
  return Math.hypot(px-(ax+t*dx),py-(ay+t*dy)); }

function localOff(bi,wx,wy){ const b=bodies[bi]; const dx=wx-b.x,dy=wy-b.y;
  return R(-b.th,dx,dy); }
function offOf(b,P){ return R(-b.th, P[0]-b.x, P[1]-b.y); }

// snap a world point to the nearest body centre or edge (optionally limited to some bodies)
function snapAnchor(wx,wy,allow){
  const Rr=12/cam.scale; let best=null, bestD=Rr;
  for(const b of bodies){ if(allow && !allow.includes(b.id)) continue;
    const d=Math.hypot(wx-b.x,wy-b.y);
    if(d<bestD){ best={body:b,wp:[b.x,b.y],kind:'centre'}; bestD=d; }
    if(d>1e-6){ const de=Math.abs(d-b.r);
      if(de<bestD){ const ux=(wx-b.x)/d, uy=(wy-b.y)/d; best={body:b,wp:[b.x+ux*b.r,b.y+uy*b.r],kind:'edge'}; bestD=de; } }
  }
  return best;
}
// resolve a click to {body, wp} -- snap first, else any body under the cursor
function anchorTarget(wx,wy){
  const s=snapAnchor(wx,wy); if(s) return {body:s.body, wp:s.wp, snap:s};
  const bi=pickBody(wx,wy); if(bi>=0) return {body:bodies[bi], wp:[wx,wy], snap:null};
  return null;
}
// ---- §13.3 · constraint handles (edit anchors/directions in place) ----
// draggable handles carried by each constraint
function conHandles(con){
  if(con.type==='rod'||con.type==='slot'){
    const [ax,ay]=epWorld(con.a), [bx,by]=epWorld(con.b);
    return [{which:'A',x:ax,y:ay},{which:'B',x:bx,y:by}];
  }
  const A=bodies[bodyIndex(con.a.id)]; if(!A) return [];
  if(con.type==='pin'){ const [x,y]=worldPt(A,con.a.off); return [{which:'pivot',x,y}]; }
  if(con.type==='knife'){ const [px,py]=worldPt(A,con.a.off);
    const hh=R(A.th,con.dir[0],con.dir[1]); const hl=Math.hypot(hh[0],hh[1])||1;
    return [ {which:'anchor',x:px,y:py}, {which:'dir',x:px+hh[0]/hl*0.7, y:py+hh[1]/hl*0.7} ]; }
  return [];
}
// cable handle: the control point on the spool rim (draggable to wind/unwind)
function cableHandlePos(cb){
  const f=cableFrame(cb); if(!f) return null;
  return {x:f.Ax, y:f.Ay};
}
function pickCableHandle(wx,wy){
  const tol=11/cam.scale;
  for(let i=cables.length-1;i>=0;i--){
    const h=cableHandlePos(cables[i]); if(!h) continue;
    if(Math.hypot(wx-h.x,wy-h.y)<=tol) return {cb:cables[i],cbi:i,which:'ctrl'}; }
  return null;
}
function applyCableHandle(ad,wx,wy){
  const cb=ad.cb; const S=bodies[bodyIndex(cb.spool.id)]; if(!S) return;
  // Raw local angle of the dragged point on the spool rim.
  const rawLocal=Math.atan2(wy-S.y, wx-S.x)-S.th;
  if(cb.localAngle===undefined) cb.localAngle=rawLocal;
  let d=rawLocal-cb.localAngle;
  while(d>Math.PI) d-=Math.PI*2;
  while(d<-Math.PI) d+=Math.PI*2;
  cb.localAngle+=d;
  // Recompute spoolAngle from the new localAngle. Both fields must be set here,
  // not just one: cb._spoolAngle is the live unwrap-continuity reference substep
  // reads each physics step (physics.js §08.2) -- leaving it stale would make the
  // next step's unwrap jump by whatever angle the drag just covered; cb.spoolAngle
  // is the persisted twin saved/loaded with the file, and won't be refreshed by a
  // substep if the sim is paused. Ltot is re-derived so the drag also changes how
  // much cable is "let out" at the new anchor, not just where the anchor sits.
  const f=cableFrame(cb); if(!f) return;
  cb._spoolAngle=f.spoolAngle;
  cb.spoolAngle =f.spoolAngle;
  cb.Ltot=cableCurrentLength(cb,f);
}
function pickHandle(wx,wy){
  const tol=11/cam.scale;
  for(let i=constraints.length-1;i>=0;i--){ const con=constraints[i];
    for(const h of conHandles(con)){ if(Math.hypot(wx-h.x,wy-h.y)<=tol) return {con,which:h.which,ci:i}; } }
  return null;
}
// move an anchor while editing; snaps and (for rods) can re-bind to another body
function applyHandle(ad, wx, wy){
  const con=ad.con;
  if(con.type==='pin'){
    const A=bodies[bodyIndex(con.a.id)], B=bodies[bodyIndex(con.b.id)];
    const s=snapAnchor(wx,wy,[A.id,B.id]); lastSnap=s; const P=s?s.wp:[wx,wy];
    con.a.off=offOf(A,P); con.b.off=offOf(B,P);
  } else if(con.type==='rod'||con.type==='slot'){
    // Snap to a body if one is under/near the cursor; otherwise the end
    // re-binds to the background at the raw world point.
    const s=snapAnchor(wx,wy); lastSnap=s;
    const ep = ad.which==='A'? con.a : con.b;
    if(s){ ep.id=s.body.id; ep.off=offOf(s.body,s.wp); }
    else { const bi=pickBody(wx,wy);
      if(bi>=0){ ep.id=bodies[bi].id; ep.off=offOf(bodies[bi],[wx,wy]); }
      else { ep.id=null; ep.off=[wx,wy]; } }
    if(con.type==='rod'){
      const [wax,way]=epWorld(con.a), [wbx,wby]=epWorld(con.b);
      con.len=Math.hypot(wax-wbx,way-wby);
      // Keep any welded end's rest angle consistent with the edited geometry.
      if(con.weldA) setRodWeld(con,'A',true);
      if(con.weldB) setRodWeld(con,'B',true);
    } else {
      // Keep any locked ("prismatic") end's rest angle consistent too.
      if(con.prismaticA) setSlotLock(con,'A',true);
      if(con.prismaticB) setSlotLock(con,'B',true);
    }
  }
  else if(con.type==='knife'){
    const A=bodies[bodyIndex(con.a.id)];
    if(ad.which==='anchor'){ const s=snapAnchor(wx,wy,[A.id]); lastSnap=s; const P=s?s.wp:[wx,wy]; con.a.off=offOf(A,P); }
    else if(ad.which==='dir'){ const [px,py]=worldPt(A,con.a.off); const dx=wx-px,dy=wy-py; const L=Math.hypot(dx,dy)||1;
      lastSnap=null; con.dir=R(-A.th, dx/L, dy/L); }
  }
}
// scale a stored {id, off} endpoint that rides `bodyId`'s local frame by
// `ratio`, leaving everything else (background-anchored endpoints, other
// bodies' endpoints) untouched -- shared by applyBodyResize below.
function scaleOffOnBody(ep, bodyId, ratio){
  if(ep && ep.id===bodyId && ep.off) ep.off=[ep.off[0]*ratio, ep.off[1]*ratio];
}
// Resize the selected body by dragging its rim (§13.5/§13.6): every control
// point anchored on it -- pin/rod/slot/knife offsets, gas cylinder/head
// offsets, a cable tether offset -- scales by the same ratio as the radius,
// so it stays at the same *proportional* position on the body rather than
// snapping to a fixed absolute offset. (A cable's own spool anchor needs no
// such scaling: it's already a pure angle around the rim, so it tracks the
// new radius for free -- see cableFrame, constraints.js §06.3.) A belt's
// wrap radius is a copy of the body's radius taken at creation time, so it
// is rescaled too, with its restPhase recaptured against the *current*
// angles so the resize itself never reads as a spurious constraint jump
// (mirrors captureRestAngle's role for rod/slot locks, constraints.js §06.1).
// Once the offsets are updated, projectPositions rearticulates every other
// joint on the body to the new geometry -- the same "pose" rearticulation a
// plain body drag performs (§13.6). Factored out of the rim-drag handler so
// the inspector panel's radius field (§14.2) can drive the exact same update
// path from a typed value instead of a pointer distance.
function resizeBody(b, newR){
  newR=Math.max(0.08, newR);
  const ratio=newR/b.r;
  if(!isFinite(ratio) || ratio<=0) return;
  for(const con of constraints){
    scaleOffOnBody(con.a, b.id, ratio);
    if(con.b) scaleOffOnBody(con.b, b.id, ratio);
    if(con.type==='belt'){
      let touched=false;
      if(con.a.id===b.id){ con.rA*=ratio; touched=true; }
      if(con.b.id===b.id){ con.rB*=ratio; touched=true; }
      if(touched){ const A=bodies[bodyIndex(con.a.id)], B=bodies[bodyIndex(con.b.id)];
        con.restPhase=con.rA*A.th - con.sense*con.rB*B.th; }
    }
  }
  for(const g of gases){ scaleOffOnBody(g.a, b.id, ratio); if(g.head) scaleOffOnBody(g.head, b.id, ratio); }
  for(const cb of cables){ scaleOffOnBody(cb.tether, b.id, ratio); }
  b.r=newR; refreshMass(b);
  projectPositions(8);
}
function applyBodyResize(rd, wx, wy){
  resizeBody(rd.b, Math.hypot(wx-rd.b.x,wy-rd.b.y));
  saveState();
}

// ---- §13.4 · pointer state (multi-touch, pinch) ----
// active pointers keyed by id, for one-finger pan and two-finger pinch-zoom
const pointers=new Map();
let pinch=null, pinchCooldown=false, downScreen=null, movedFar=false;
let clickArmed=false;   // non-select tools: the tap-committed click (§13.5/§13.7)
let anchorDrag=null, lastSnap=null, resizeDrag=null;
function cancelSingle(){ drag=null; grab=null; bodyPreview=null; panning=null; anchorDrag=null; lastSnap=null; resizeDrag=null; clickArmed=false; }

// `hover` highlights whatever body/interaction sits under the cursor when it
// isn't already selected; `hoverHandle` highlights a control point of the
// *selected* interaction when the cursor is over it; `hoverSnap` highlights
// the specific anchor location (body centre/edge) a placement tool would
// attach to. All three are recomputed on every pointermove (§13.6) and
// cleared whenever the pointer is busy doing something else (dragging,
// panning, pinching, ...).
let hover=null, hoverHandle=null, hoverSnap=null;
function updateHover(wx,wy){
  hover=null; hoverHandle=null; hoverSnap=null;
  if(sim.running) return;
  if(tool==='select'){
    // a control point of the already-selected interaction takes priority
    const ch=pickCableHandle(wx,wy);
    if(ch && ch.cb.sel){ hoverHandle={kind:'cable',cb:ch.cb,which:ch.which}; return; }
    const h=pickHandle(wx,wy);
    if(h && h.con.sel){ hoverHandle={kind:'con',con:h.con,which:h.which}; return; }
    // ...as does the selected body's rim, the resize handle
    if(selBody && bodyRimHit(selBody,wx,wy)){ hoverHandle={kind:'resize',b:selBody}; return; }
    // otherwise highlight whatever is under the cursor, unless it's the selection
    // -- interactions take priority over bodies (matching the delete order,
    // §13.4/§13.5 below) so a constraint/gas/cable coincident with a body is
    // still reachable instead of always losing to the body underneath it
    const cci=pickConstraint(wx,wy); if(cci>=0){ if(!constraints[cci].sel) hover=constraints[cci]; return; }
    const gsi=pickGas(wx,wy); if(gsi>=0){ if(!gases[gsi].sel) hover=gases[gsi]; return; }
    const cbi=pickCable(wx,wy); if(cbi>=0){ if(!cables[cbi].sel) hover=cables[cbi]; return; }
    const bi=pickBody(wx,wy); if(bi>=0){ if(!bodies[bi].sel) hover=bodies[bi]; return; }
    return;
  }
  if(tool==='delete'){
    // interactions take priority over bodies, matching the delete order (§13.5)
    const cci=pickConstraint(wx,wy); if(cci>=0){ hover=constraints[cci]; return; }
    const gsi=pickGas(wx,wy); if(gsi>=0){ hover=gases[gsi]; return; }
    const cbi=pickCable(wx,wy); if(cbi>=0){ hover=cables[cbi]; return; }
    const bi=pickBody(wx,wy); if(bi>=0){ hover=bodies[bi]; return; }
    return;
  }
  if(tool==='pin'||tool==='rod'||tool==='slot'||tool==='gas'||tool==='cable'){
    // these tools attach to a snapped anchor (body centre/edge) or a bare body
    const t=anchorTarget(wx,wy); if(t){ hover=t.body; hoverSnap=t.snap; }
    return;
  }
  if(tool==='belt'||tool==='cvt'||tool==='knife'){
    const bi=pickBody(wx,wy); if(bi>=0) hover=bodies[bi];
    return;
  }
  // 'body' tool has no existing element to highlight -- its own live preview
  // (bodyPreview, render.js §11.7) already shows where the new body will go
}

function startPinch(){
  const pts=[...pointers.values()];
  const mx=(pts[0].x+pts[1].x)/2, my=(pts[0].y+pts[1].y)/2;
  pinch={ dist:Math.hypot(pts[0].x-pts[1].x,pts[0].y-pts[1].y)||1, midWorld:s2w(mx,my), scale:cam.scale };
}

// ---- §13.5 · pointerdown (per-tool dispatch) ----
// This is where each tool builds its constraint. The branches, in order, handle:
// pinch guard, explicit pan, select (+handles/resize/grab -- the only case
// that can claim a one-finger drag instead of panning), body, delete, gas,
// belt/cvt, knife, cable, pin, rod, slot.
//
// Every non-select tool, and every select-tool click that doesn't land on
// the current selection's own draggable region (a handle, its rim, or its
// body), arms `panning` as a fallback: pinch/drag/scroll pan and zoom
// regardless of the active tool. A non-select tool's own click action
// (placing a pending anchor, deleting whatever is under the cursor, ...) is
// deferred until pointerup (`clickArmed`, consumed by endPointer / runToolClick
// below) and only actually runs if the gesture turns out to be a tap rather
// than a drag -- otherwise the drag would both pan *and* place/delete
// something. Search  tool==='<name>'  to reach one.
cv.addEventListener('pointerdown',e=>{
  cv.setPointerCapture(e.pointerId);
  const rect=cv.getBoundingClientRect();
  const px=e.clientX-rect.left, py=e.clientY-rect.top;
  pointers.set(e.pointerId,{x:px,y:py});
  mouseScreen=[px,py]; mouseWorld=s2w(px,py);

  if(pointers.size===2){ cancelSingle(); startPinch(); return; }   // second finger -> pinch, never a tool action
  if(pointers.size>2) return;

  const [wx,wy]=mouseWorld;
  downScreen=[px,py]; movedFar=false; clickArmed=false;

  if(e.button===1 || (e.button===0 && e.altKey)){ panning={sx:e.clientX,sy:e.clientY,cx:cam.x,cy:cam.y}; return; }

  if(tool==='select'){
    if(!sim.running){
      // cable handle check first: allows winding control in edit mode, but only
      // once the cable is already the selection -- a click on an unselected
      // cable's handle just selects it, matching how constraint handles behave.
      const ch=pickCableHandle(wx,wy);
      if(ch){ if(ch.cb.sel){ anchorDrag=ch; applyCableHandle(ch,wx,wy); return; } selectCable(ch.cbi); return; }
      const h=pickHandle(wx,wy);
      if(h){ if(h.con.sel){ anchorDrag=h; applyHandle(h,wx,wy); return; } selectConstraint(h.ci); return; }
      // the selected body's rim -- drag to resize (§13.6 applyBodyResize)
      if(selBody && bodyRimHit(selBody,wx,wy)){ resizeDrag={b:selBody}; return; }
    }
    // the selected body's interior -- prioritised over whatever else happens
    // to be on top at this point, so an occluded selection stays acted-on
    // rather than silently swapped for the topmost body
    if(selBody && (wx-selBody.x)**2+(wy-selBody.y)**2<=selBody.r*selBody.r){
      const bi=bodies.indexOf(selBody);
      if(sim.running){ grab={bi, off:localOff(bi,wx,wy)}; } else { drag={bi, off:localOff(bi,wx,wy)}; }
      return;
    }
    // the selected constraint/gas/cable, hit anywhere along it (not just a
    // handle) -- keeps it selected instead of losing it to an occluding body
    if(selConstraint && constraintHit(selConstraint,wx,wy)){
      panning={sx:e.clientX,sy:e.clientY,cx:cam.x,cy:cam.y}; return; }
    if(selGas && gasHit(selGas,wx,wy)){ panning={sx:e.clientX,sy:e.clientY,cx:cam.x,cy:cam.y}; return; }
    if(selCable && cableHit(selCable,wx,wy)){ panning={sx:e.clientX,sy:e.clientY,cx:cam.x,cy:cam.y}; return; }
    // nothing of the current selection was hit -- a fresh topmost pick, which
    // changes the selection (a body's pick also arms its drag/grab in the
    // same gesture: it becomes selected here, before any pointermove, so a
    // drag starting on it still satisfies "starts on the selected thing").
    // Interactions take priority over bodies here too (matching updateHover
    // and the delete-tool order, §13.4) so a constraint/gas/cable coincident
    // with a body is what gets selected, not the body occluding it.
    const cci=pickConstraint(wx,wy);
    if(cci>=0){ selectConstraint(cci); panning={sx:e.clientX,sy:e.clientY,cx:cam.x,cy:cam.y}; return; }
    const gsi=pickGas(wx,wy);
    if(gsi>=0){ selectGas(gsi); panning={sx:e.clientX,sy:e.clientY,cx:cam.x,cy:cam.y}; return; }
    const cbi=pickCable(wx,wy);
    if(cbi>=0){ selectCable(cbi); panning={sx:e.clientX,sy:e.clientY,cx:cam.x,cy:cam.y}; return; }
    const bi=pickBody(wx,wy);
    if(bi>=0){ selectBody(bi);
      if(sim.running){ grab={bi, off:localOff(bi,wx,wy)}; }
      else { drag={bi, off:localOff(bi,wx,wy)}; }
      return; }
    panning={sx:e.clientX,sy:e.clientY,cx:cam.x,cy:cam.y,candidate:true};   // one-finger background pan; a tap deselects
    return;
  }

  // every other tool: arm the background pan up front, but *don't* run the
  // tool's own click logic yet -- a drag from here must only pan, never also
  // place/delete something. The click logic is deferred to pointerup
  // (§13.7 endPointer) and only fires if the gesture turns out to be a tap
  // (!movedFar); a real drag is left to have panned and nothing else.
  panning={sx:e.clientX,sy:e.clientY,cx:cam.x,cy:cam.y};
  clickArmed=true;
  return;
});
// the click logic for every non-select tool, run only on a confirmed tap
// (pointerdown that never turned into a drag) -- see the pointerdown handler
// above for why this is deferred instead of firing immediately.
function runToolClick(wx,wy){
  if(tool==='body'){
    // two clicks, like every other creation tool: first click drops the
    // centre and previews the radius live as the pointer hovers afterward
    // (drawPreview, render.js §11.7); second click commits it.
    if(!bodyPreview){ bodyPreview={cx:wx,cy:wy,r:0}; return; }
    const r=bodyPreview.r<0.12?0.4:bodyPreview.r;
    const b=makeBody(bodyPreview.cx,bodyPreview.cy,r,false); bodies.push(b); bodyPreview=null;
    selectBody(bodies.length-1); saveState();
    return;
  }
  if(tool==='delete'){
    // interactions take priority over bodies (updateHover, §13.4, mirrors
    // this order) -- a constraint/gas/cable coincident with a body is what
    // most often needs deleting without also taking the body out with it.
    const cci=pickConstraint(wx,wy); if(cci>=0){ constraints.splice(cci,1); clearSelection(); saveState(); return; }
    const gsi=pickGas(wx,wy); if(gsi>=0){ gases.splice(gsi,1); clearSelection(); saveState(); return; }
    const cbi=pickCable(wx,wy); if(cbi>=0){ cables.splice(cbi,1); clearSelection(); saveState(); return; }
    const bi=pickBody(wx,wy);
    if(bi>=0){ const id=bodies[bi].id;
      constraints=constraints.filter(c=>c.a.id!==id && !(c.b&&c.b.id===id));
      bodies.splice(bi,1); clearSelection(); saveState(); }
    return;
  }
  if(tool==='gas'){
    // FIRST click = closed end (head): on a cylinder body, or empty for a world-fixed head
    if(!pending){ const t=anchorTarget(wx,wy);
      pending = t ? {gas:true, headId:t.body.id, headWp:t.wp, wp:t.wp}
                  : {gas:true, headId:null, headWp:[wx,wy], wp:[wx,wy]};
      return; }
    // SECOND click = piston face on a body; axis runs head -> piston (expansion +)
    const t=anchorTarget(wx,wy); if(!t) return;
    const dx=t.wp[0]-pending.headWp[0], dy=t.wp[1]-pending.headWp[1]; const L=Math.hypot(dx,dy);
    if(L<0.2) return;
    const axis=[dx/L,dy/L]; const A=t.body;
    const head = pending.headId!=null
      ? (()=>{ const B=bodies[bodyIndex(pending.headId)]; return {id:B.id, off:offOf(B,pending.headWp), dir:R(-B.th,axis[0],axis[1])}; })()
      : {id:null, off:[pending.headWp[0],pending.headWp[1]], dir:axis};
    pending=null; saveState();
    return;
  }
  if(tool==='belt' || tool==='cvt'){
    // two bodies: A first, then B (occluded B reachable via except-pick)
    if(!pending){ const bi=pickBody(wx,wy); if(bi<0)return; pending={id:bodies[bi].id, wp:[wx,wy]}; return; }
    const bi2=pickBodyExcept(wx,wy,pending.id); if(bi2<0)return;
    const A=bodies[bodyIndex(pending.id)], B=bodies[bi2];
    if(tool==='belt'){
      constraints.push({type:'belt', a:{id:A.id}, b:{id:B.id}, rA:A.r, rB:B.r, sense:1,
                        restPhase:(A.r*A.th - B.r*B.th), sel:false});
    } else {
      constraints.push({type:'cvt', a:{id:A.id}, b:{id:B.id}, sel:false});
    }
    pending=null; saveState();
    return;
  }
  if(tool==='knife'){
    // contact point on a body, then a heading direction
    if(!pending){ const t=anchorTarget(wx,wy); if(!t) return;
      pending={id:t.body.id, off:offOf(t.body,t.wp), wp:t.wp}; return; }
    const dx=wx-pending.wp[0], dy=wy-pending.wp[1]; const L=Math.hypot(dx,dy); if(L<0.15) return;
    const A=bodies[bodyIndex(pending.id)];
    constraints.push({type:'knife', a:{id:pending.id, off:pending.off}, dir:R(-A.th, dx/L, dy/L), sel:false});
    pending=null; saveState();
    return;
  }
  if(tool==='cable'){
    // FIRST click = tether point (on a body, or empty for a world anchor)
    if(!pending){ const t=anchorTarget(wx,wy);
      pending = t ? {cable:true, tid:t.body.id, toff:offOf(t.body,t.wp), wp:t.wp}
                  : {cable:true, tid:null, toff:[wx,wy], wp:[wx,wy]};
      return; }
    // SECOND click = spool body
    const bi=pickBody(wx,wy); if(bi<0) return;
    const S=bodies[bi]; if(pending.tid!=null && S.id===pending.tid) return;
    const T = pending.tid!=null ? (()=>{ const tb=bodies[bodyIndex(pending.tid)]; const [x,y]=worldPt(tb,pending.toff); return [x,y]; })()
                                : [pending.toff[0],pending.toff[1]];
    const dvx=T[0]-S.x, dvy=T[1]-S.y; const d=Math.hypot(dvx,dvy); if(d<1e-6) return;
    const Lfree = d > S.r ? Math.sqrt(d*d-S.r*S.r) : 0;
    // Initialise anchor at the rim point closest to the tether (spoolAngle = 0).
    const localAngle=Math.atan2(dvy,dvx)-S.th;
    const cb0={type:'cable', tether:{id:pending.tid, off:pending.toff}, spool:{id:S.id},
               localAngle, spoolAngle:0, Ltot:Lfree, sel:false};
    cables.push(cb0);
    pending=null; saveState();
    return;
  }
  if(tool==='pin'){
    // FIRST pick -- snapped anchor on body A; pending.wp is the pivot world point
    if(!pending){ const t=anchorTarget(wx,wy); if(!t) return;
      pending={id:t.body.id, off:offOf(t.body,t.wp), wp:t.wp}; return; }
    const Aid=pending.id, Aoff=pending.off;
    // the second click only *names* body B -- click any part of it, including
    // where A covers it -- and B is anchored at the first pivot
    const bi=pickBodyExcept(wx,wy,Aid);
    const Bbody = bi>=0 ? bodies[bi] : (()=>{ const s=snapAnchor(wx,wy); return (s&&s.body.id!==Aid)?s.body:null; })();
    if(!Bbody || Bbody.id===Aid) return;   // nothing indicated -- keep the pivot and wait
    const Boff = offOf(Bbody, pending.wp); // place B's anchor at the first pivot
    constraints.push({type:'pin', a:{id:Aid,off:Aoff}, b:{id:Bbody.id,off:Boff}, sel:false});
    pending=null; saveState();
    return;
  }
  if(tool==='rod'){
    // Each end snaps to a body if one is under/near the click, else attaches
    // to the background at the raw world point (id:null).
    if(!pending){ const t=anchorTarget(wx,wy);
      pending = t ? {id:t.body.id, off:offOf(t.body,t.wp), wp:t.wp} : {id:null, off:[wx,wy], wp:[wx,wy]};
      return; }
    const t=anchorTarget(wx,wy);
    const Bep = t ? {id:t.body.id, off:offOf(t.body,t.wp)} : {id:null, off:[wx,wy]};
    const Aep={id:pending.id, off:pending.off};
    if(Aep.id==null && Bep.id==null) return;      // a rod needs at least one real body -- keep pending, wait for a better second click
    if(Aep.id!=null && Bep.id===Aep.id) return;    // can't rod a body to itself -- ditto
    pending=null;
    // A rod touching the background defaults to both ends welded -- a rigid
    // strut out of the wall -- since that's the anchoring use case; the user
    // can tap either end afterward to free it into a pin.
    const bg = Aep.id==null || Bep.id==null;
    constraints.push(makeRodCon(Aep, Bep, bg, bg));
    saveState();
    return;
  }
  if(tool==='slot'){
    // Two clicks, exactly like rod: each end snaps to a body if one is
    // under/near the click, else attaches to the background. The rail
    // direction is implied by the two points' positions at creation.
    if(!pending){ const t=anchorTarget(wx,wy);
      pending = t ? {id:t.body.id, off:offOf(t.body,t.wp), wp:t.wp} : {id:null, off:[wx,wy], wp:[wx,wy]};
      return; }
    const t=anchorTarget(wx,wy);
    const Bep = t ? {id:t.body.id, off:offOf(t.body,t.wp)} : {id:null, off:[wx,wy]};
    const Aep={id:pending.id, off:pending.off};
    if(Aep.id==null && Bep.id==null) return;      // needs at least one real body -- keep pending, wait for a better second click
    if(Aep.id!=null && Bep.id===Aep.id) return;    // can't rail a body to itself -- ditto
    pending=null;
    // A slot touching the background defaults to both ends prismatic -- a
    // fixed rail -- since that's the confinement use case; tapping either end
    // afterward frees it back into a plain (visual-only) pin.
    const bg = Aep.id==null || Bep.id==null;
    constraints.push(makeSlotCon(Aep, Bep, bg, bg));
    saveState();
    return;
  }
}

// ---- §13.6 · pointermove (drag / pan / pinch / handle articulation) ----
cv.addEventListener('pointermove',e=>{
  const rect=cv.getBoundingClientRect();
  const px=e.clientX-rect.left, py=e.clientY-rect.top;
  if(pointers.has(e.pointerId)) pointers.set(e.pointerId,{x:px,y:py});
  mouseScreen=[px,py]; mouseWorld=s2w(px,py);

  if(pinch && pointers.size>=2){                          // two-finger pan + zoom, anchored at the pinch midpoint
    const pts=[...pointers.values()];
    const mx=(pts[0].x+pts[1].x)/2, my=(pts[0].y+pts[1].y)/2;
    const dist=Math.hypot(pts[0].x-pts[1].x,pts[0].y-pts[1].y)||1;
    cam.scale=Math.max(12,Math.min(300, pinch.scale*(dist/pinch.dist)));
    cam.x=pinch.midWorld[0]-(mx-W()/2)/cam.scale;
    cam.y=pinch.midWorld[1]+(my-H()/2)/cam.scale;
    return;
  }
  if(pinchCooldown) return;                               // ignore the leftover finger after a pinch
  if(downScreen && Math.hypot(px-downScreen[0],py-downScreen[1])>6) movedFar=true;

  if(anchorDrag||resizeDrag||panning||bodyPreview||(drag&&!sim.running)||grab){ hover=null; hoverHandle=null; hoverSnap=null; }
  else updateHover(mouseWorld[0],mouseWorld[1]);
  if(tool==='select') cv.style.cursor = resizeDrag ? 'grabbing' : (hoverHandle && hoverHandle.kind==='resize') ? 'grab' : 'default';

  if(anchorDrag){ if(anchorDrag.cb) applyCableHandle(anchorDrag,mouseWorld[0],mouseWorld[1]); else applyHandle(anchorDrag, mouseWorld[0], mouseWorld[1]); saveState(); return; }
  if(resizeDrag){ applyBodyResize(resizeDrag, mouseWorld[0], mouseWorld[1]); return; }
  if(panning){ cam.x=panning.cx-(e.clientX-panning.sx)/cam.scale; cam.y=panning.cy+(e.clientY-panning.sy)/cam.scale; return; }
  if(bodyPreview){ bodyPreview.r=Math.hypot(mouseWorld[0]-bodyPreview.cx,mouseWorld[1]-bodyPreview.cy); return; }
  if(drag && !sim.running){
    const G=bodies[drag.bi];
    if(G.static){
      // move the root kinematically; the island follows it
      const [gx,gy]=worldPt(G,drag.off); G.x+=mouseWorld[0]-gx; G.y+=mouseWorld[1]-gy;
      projectPositions(8);
    } else {
      // pull the grabbed point toward the cursor; the island articulates to comply.
      // 'dragpin' is an internal-only row type (§06.5) -- never added to
      // `constraints`, just fed through projectPositions as a transient goal.
      const temp={type:'dragpin', a:{id:G.id, off:drag.off}, world:[mouseWorld[0],mouseWorld[1]]};
      projectPositions(8,[temp]);
    }
    saveState();
  }
});

// ---- §13.7 · pointerup / cancel / wheel ----
function endPointer(e){
  pointers.delete(e.pointerId);
  if(pinch && pointers.size<2){ pinch=null; pinchCooldown = pointers.size>0; }
  if(pointers.size===0) pinchCooldown=false;
  if(pinchCooldown){ cancelSingle(); return; }

  if(anchorDrag){
    // A tap (no drag) on a rod's or slot's own control point toggles that end
    // between a freely-rotating pin and a rotation-locked weld/prismatic
    // state, instead of relocating it. That flips a property the inspector
    // panel shows as a checkbox (weld/prismatic), so refresh it too -- item 2:
    // editing an interaction from its control points must stay in sync with
    // the panel, the same as editing it from the panel's own checkboxes.
    const conType = anchorDrag.con && anchorDrag.con.type;
    if(!movedFar && (conType==='rod'||conType==='slot') && (anchorDrag.which==='A'||anchorDrag.which==='B')){
      if(conType==='rod') toggleRodWeld(anchorDrag.con, anchorDrag.which);
      else toggleSlotLock(anchorDrag.con, anchorDrag.which);
      renderInspector(); saveState();
    }
    anchorDrag=null; lastSnap=null; downScreen=null; return;
  }
  if(resizeDrag){ resizeDrag=null; downScreen=null; return; }
  // commit a non-select tool's click now, but only for a genuine tap -- a
  // pointer that moved far enough to count as a drag already just panned
  // (§13.5/§13.6), and a cancelled pointer (e.g. an interrupted gesture)
  // shouldn't place/delete anything either. Recompute the world point from
  // this event's own coordinates (not the possibly-stale `mouseWorld`) so
  // the action fires exactly where the tap was released.
  if(clickArmed && !movedFar && e.type==='pointerup'){
    const rect=cv.getBoundingClientRect();
    const [ux,uy]=s2w(e.clientX-rect.left, e.clientY-rect.top);
    runToolClick(ux,uy);
  }
  clickArmed=false;
  if(panning){ if(panning.candidate && !movedFar) clearSelection(); panning=null; }
  drag=null; grab=null; downScreen=null;
}
cv.addEventListener('pointerup',endPointer);
cv.addEventListener('pointercancel',endPointer);
cv.addEventListener('pointerleave',()=>{ hover=null; hoverHandle=null; hoverSnap=null; });
cv.addEventListener('wheel',e=>{ e.preventDefault();
  const rect=cv.getBoundingClientRect(); const mx=e.clientX-rect.left,my=e.clientY-rect.top;
  const before=s2w(mx,my); cam.scale*=Math.exp(-e.deltaY*0.0012);
  cam.scale=Math.max(12,Math.min(300,cam.scale)); const after=s2w(mx,my);
  cam.x+=before[0]-after[0]; cam.y+=before[1]-after[1];
},{passive:false});
