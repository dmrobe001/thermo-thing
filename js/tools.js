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
  {id:'rectbody',key:'q',tip:'Add rectangle (q)',svg:'<rect x="4" y="6" width="16" height="12" rx="1"/><path d="M12 8v8M6 12h12"/>'},
  {id:'vessel',key:'g',tip:'Add gas vessel (g)',svg:'<rect x="7" y="3" width="10" height="18" rx="1"/><path d="M7 5.5h10M7 18.5h10" stroke-width="2.6"/>'},
  {id:'pin',key:'3',tip:'Pin / hinge (3)',svg:'<circle cx="12" cy="12" r="3"/><circle cx="12" cy="12" r="8"/>'},
  {id:'rod',key:'4',tip:'Rigid rod (4)',svg:'<circle cx="6" cy="18" r="2.4"/><circle cx="18" cy="6" r="2.4"/><path d="M7.5 16.5l9-9"/>'},
  {id:'slot',key:'5',tip:'Slot / prismatic (5)',svg:'<path d="M3 9h18M3 15h18"/><rect x="9" y="9" width="6" height="6" rx="1"/>'},
  {id:'belt',key:'b',tip:'Belt (b)',svg:'<circle cx="7" cy="12" r="4"/><circle cx="17" cy="12" r="4"/><path d="M7 8h10M7 16h10"/>'},
  {id:'knife',key:'k',tip:'Knife-edge wheel (k)',svg:'<path d="M4 16h16"/><path d="M12 16l3-9 3 9"/><circle cx="8" cy="16" r="1.5"/>'},
  {id:'cvt',key:'v',tip:'Variable gear / CVT (v)',svg:'<circle cx="9" cy="12" r="6"/><circle cx="17" cy="12" r="3"/><path d="M9 12h8"/>'},
  {id:'cable',key:'c',tip:'Cable (c)',svg:'<circle cx="16" cy="9" r="4"/><path d="M4 19c6 0 8-4 9-7"/><circle cx="4" cy="19" r="1.5"/>'},
  {id:'spring',key:'8',tip:'Linear spring (8)',svg:'<circle cx="5" cy="18" r="2.4"/><circle cx="19" cy="6" r="2.4"/><path d="M7 16.5l2-3 3 6 3-6 2 3"/>'},
  {id:'rotspring',key:'9',tip:'Rotational spring (9)',svg:'<path d="M12 3a9 9 0 1 0 9 9"/><path d="M12 7a5 5 0 1 0 5 5"/><path d="M12 11a1 1 0 1 0 1 1"/>'},
  {id:'delete',key:'7',tip:'Delete (7)',svg:'<path d="M6 7h12M9 7V5h6v2M8 7l1 12h6l1-12"/>'},
];
let tool='select';
const rail=document.getElementById('rail');
TOOLS.forEach((t,i)=>{
  if(i===1||i===4||i===11){ const s=document.createElement('div');s.className='rail-sep';rail.appendChild(s);}
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
  for(let i=bodies.length-1;i>=0;i--){ if(bodyContains(bodies[i],wx,wy)) return i; }
  return -1;
}
// topmost body under the cursor that isn't `exceptId` -- lets the 2nd pin
// pick reach a body occluded by the one already selected
function pickBodyExcept(wx,wy,exceptId){
  for(let i=bodies.length-1;i>=0;i--){ const b=bodies[i]; if(b.id===exceptId) continue;
    if(bodyContains(b,wx,wy)) return i; }
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
  if(con.type==='pin'){
    const [ax,ay]=epWorld(con.a);
    return (wx-ax)**2+(wy-ay)**2<=tol*tol;
  }
  const A=bodies[bodyIndex(con.a.id)]; if(!A) return false;
  const [ax,ay]=con.a.off?epWorldPt(A,con.a.off):[A.x,A.y];
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
function cableHit(cb,wx,wy){ const tol=10/cam.scale; const f=cableFrame(cb); if(!f) return false;
  return distSeg(wx,wy,f.T[0],f.T[1],f.Qx,f.Qy)<=tol; }
// spring / rotSpring hit tests, same role as cableHit above -- springs
// live in their own arrays (constraints.js §06.6), not `constraints`, so they
// get their own pick path (pickSpring/pickRotSpring, inspector.js §14.1)
// rather than going through pickConstraint.
function springHit(sp,wx,wy){ const tol=10/cam.scale;
  const [ax,ay]=epWorld(sp.a), [bx,by]=epWorld(sp.b);
  return distSeg(wx,wy,ax,ay,bx,by)<=tol; }
function rotSpringHit(rs,wx,wy){ const tol=10/cam.scale;
  const hasA=rs.a.id!=null, hasB=rs.b.id!=null;
  const A=hasA?bodies[bodyIndex(rs.a.id)]:null, B=hasB?bodies[bodyIndex(rs.b.id)]:null;
  if((hasA&&!A)||(hasB&&!B)) return false;
  if(rotSpringVisualMode(rs)==='belt'){
    for(const [p,q] of beltTangents(A.x,A.y,A.r, B.x,B.y,B.r, 1)){ if(distSeg(wx,wy,p[0],p[1],q[0],q[1])<=tol) return true; }
    return Math.abs(Math.hypot(wx-A.x,wy-A.y)-A.r)<=tol || Math.abs(Math.hypot(wx-B.x,wy-B.y)-B.r)<=tol;
  }
  // spiral mode: cheap radius-band test rather than tracing the true spiral
  // path -- good enough to pick a decorative element, no reaction/instrument
  // value rides on exact spiral-arc hit precision the way a rod's line does.
  const geo=rotSpringSpiralGeom(rs);
  const d=Math.hypot(wx-geo.cx,wy-geo.cy);
  return d>=Math.min(geo.outerR,geo.innerR)-tol && d<=Math.max(geo.outerR,geo.innerR)+tol;
}
// perimeter (rim) hit test on a specific body -- the drag handle for
// resizing (§13.5/§13.6). A separate tolerance ring around the rim/corners,
// distinct from the filled-outline `pickBody` test used for moving the body.
// A circle resizes from anywhere on its rim; a rectangle resizes from one of
// its four corners specifically (bodyCornerHit), matching the two-opposite-
// corners mental model the rectbody placement tool uses.
function bodyRimHit(b,wx,wy){
  if(rectLike(b)) return !!bodyCornerHit(b,wx,wy);
  const tol=10/cam.scale;
  return Math.abs(Math.hypot(wx-b.x,wy-b.y)-b.r)<=tol;
}
// Which corner (as a [+-1,+-1] local sign pair) of a rectangle body is under
// the cursor, or null. Used both by bodyRimHit above and to remember *which*
// corner a resize drag is anchored to (tools.js §13.5/§13.6).
function bodyCornerHit(b,wx,wy){
  if(!rectLike(b)) return null;
  const tol=12/cam.scale;
  for(const s of [[-1,-1],[1,-1],[1,1],[-1,1]]){
    const [cx,cy]=worldPt(b,[s[0]*b.hw, s[1]*b.hh]);
    if(Math.hypot(wx-cx,wy-cy)<=tol) return s;
  }
  return null;
}
function distSeg(px,py,ax,ay,bx,by){ const dx=bx-ax,dy=by-ay; const L2=dx*dx+dy*dy||1e-9;
  let t=((px-ax)*dx+(py-ay)*dy)/L2; t=Math.max(0,Math.min(1,t));
  return Math.hypot(px-(ax+t*dx),py-(ay+t*dy)); }

// Both produce an *endpoint* offset, so both go through epOffOf (§05.2c): a plain
// local point on an ordinary body, a material (lat, f) label on a vessel.
function localOff(bi,wx,wy){ const b=bodies[bi]; return epOffOf(b,wx,wy); }
function offOf(b,P){ return epOffOf(b,P[0],P[1]); }

// snap a world point to the nearest body centre or edge (optionally limited to some bodies)
function snapAnchor(wx,wy,allow){
  const Rr=12/cam.scale; let best=null, bestD=Rr;
  for(const b of bodies){ if(allow && !allow.includes(b.id)) continue;
    const d=Math.hypot(wx-b.x,wy-b.y);
    if(d<bestD){ best={body:b,wp:[b.x,b.y],kind:'centre'}; bestD=d; }
    const de=bodyEdgeDist(b,wx,wy);
    if(de<bestD){ best={body:b,wp:bodyEdgePoint(b,wx,wy),kind:'edge'}; bestD=de; }
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
  if(con.type==='spring'){
    // Endpoints behave like rod's (draggable to re-anchor); the rest-length
    // control point (constraints.js §06.6 springRestHandlePos) only exists
    // while selected, matching drawSpringRestLine's own sel-gated render
    // (render.js §11.4b).
    const [ax,ay]=epWorld(con.a), [bx,by]=epWorld(con.b);
    const handles=[{which:'A',x:ax,y:ay},{which:'B',x:bx,y:by}];
    if(con.sel){ const [rx,ry]=springRestHandlePos(con); handles.push({which:'restLen',x:rx,y:ry}); }
    return handles;
  }
  const A=bodies[bodyIndex(con.a.id)]; if(!A) return [];
  if(con.type==='pin'){ const [x,y]=epWorldPt(A,con.a.off); return [{which:'pivot',x,y}]; }
  if(con.type==='knife'){ const [px,py]=epWorldPt(A,con.a.off);
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
  // the selected cable's own handle takes priority over any other cable's
  // handle occupying the same point -- otherwise a click meant to drag the
  // selection's control point can be hijacked into re-selecting whichever
  // cable happens to sit on top there instead.
  if(selCable){ const h=cableHandlePos(selCable);
    if(h && Math.hypot(wx-h.x,wy-h.y)<=tol) return {cb:selCable,cbi:cables.indexOf(selCable),which:'ctrl'}; }
  for(let i=cables.length-1;i>=0;i--){ if(cables[i]===selCable) continue;
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
  // the selected constraint's (or spring's -- constraints.js §06.6 -- they
  // share conHandles/applyHandle, just live in a separate array) own control
  // points take priority over any other one's handle occupying the same
  // point -- mirrors pickCableHandle above, for the same reason. `arr`
  // records which array `ci` indexes into, so the caller can select the
  // right one back (selectConstraint vs selectSpring).
  const selObj = selConstraint || selSpring;
  const arrOf = o => selConstraint===o ? 'constraints' : selSpring===o ? 'springs' : null;
  if(selObj){
    for(const h of conHandles(selObj)){
      if(Math.hypot(wx-h.x,wy-h.y)<=tol) return {con:selObj,which:h.which,
        ci:(selConstraint===selObj?constraints:springs).indexOf(selObj), arr:arrOf(selObj)}; } }
  for(let i=constraints.length-1;i>=0;i--){ const con=constraints[i]; if(con===selObj) continue;
    for(const h of conHandles(con)){ if(Math.hypot(wx-h.x,wy-h.y)<=tol) return {con,which:h.which,ci:i,arr:'constraints'}; } }
  for(let i=springs.length-1;i>=0;i--){ const sp=springs[i]; if(sp===selObj) continue;
    for(const h of conHandles(sp)){ if(Math.hypot(wx-h.x,wy-h.y)<=tol) return {con:sp,which:h.which,ci:i,arr:'springs'}; } }
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
    else if(ad.which==='dir'){ const [px,py]=epWorldPt(A,con.a.off); const dx=wx-px,dy=wy-py; const L=Math.hypot(dx,dy)||1;
      lastSnap=null; con.dir=R(-A.th, dx/L, dy/L); }
  }
  else if(con.type==='spring'){
    if(ad.which==='restLen'){
      // Project the drag onto the spring's own A->B direction from its
      // centre -- unlike rod's endpoint drag (which redefines the enforced
      // length), restLen is free user data, so only this dedicated handle
      // touches it, never the endpoint drag below.
      const [wax,way]=epWorld(con.a), [wbx,wby]=epWorld(con.b);
      const dx=wbx-wax, dy=wby-way, L=Math.hypot(dx,dy)||1e-9;
      const ux=dx/L, uy=dy/L, cx=(wax+wbx)/2, cy=(way+wby)/2;
      const proj=(wx-cx)*ux+(wy-cy)*uy;
      lastSnap=null;
      con.restLen=Math.max(0.05, Math.abs(proj)*2);
    } else {
      // Endpoint drag: snap/re-bind exactly like rod's, but never touch
      // restLen -- a spring's rest length is a free parameter, not the
      // literal current distance the way a rod's `len` is.
      const s=snapAnchor(wx,wy); lastSnap=s;
      const ep = ad.which==='A'? con.a : con.b;
      if(s){ ep.id=s.body.id; ep.off=offOf(s.body,s.wp); }
      else { const bi=pickBody(wx,wy);
        if(bi>=0){ ep.id=bodies[bi].id; ep.off=offOf(bodies[bi],[wx,wy]); }
        else { ep.id=null; ep.off=[wx,wy]; } }
    }
  }
}
// scale a stored {id, off} endpoint that rides `bodyId`'s local frame by
// `ratio`, leaving everything else (background-anchored endpoints, other
// bodies' endpoints) untouched -- shared by applyBodyResize below.
function scaleOffOnBody(ep, bodyId, ratio){
  if(ep && ep.id===bodyId && ep.off) ep.off=[ep.off[0]*ratio, ep.off[1]*ratio];
}
// Resize the selected body by dragging its rim (§13.5/§13.6): every control
// point anchored on it -- pin/rod/slot/knife offsets, a cable tether offset
// -- scales by the same ratio as the radius, so it stays at the same
// *proportional* position on the body rather than
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
  for(const cb of cables){ scaleOffOnBody(cb.tether, b.id, ratio); }
  // Springs (constraints.js §06.6): scale endpoint offsets like rod does,
  // leaving restLen untouched -- same precedent as rod's `len`, which the
  // loop above also never rescales. rotSprings carry no offsets (whole-body
  // frame angle, not a point on the rim), so there's nothing to scale there.
  for(const sp of springs){ scaleOffOnBody(sp.a, b.id, ratio); scaleOffOnBody(sp.b, b.id, ratio); }
  // mass now edits independently of radius (inspector.js §14.2 setBodyMass), so a
  // resize can no longer just reset it to pi*r^2 -- scale it by area (ratio^2) to
  // preserve whatever density the body currently has, mass-editing or not.
  b.mass*=ratio*ratio; b.r=newR; refreshInertia(b);
  projectPositions(8);
}
// Two-axis counterpart of scaleOffOnBody above, for a rectangle body whose
// width and height can change independently.
function scaleOffOnBodyXY(ep, bodyId, rx, ry){
  if(ep && ep.id===bodyId && ep.off) ep.off=[ep.off[0]*rx, ep.off[1]*ry];
}
// Shared tail of a rectangle resize: rescale every anchor riding this body
// (per-axis, mirroring resizeBody's uniform-ratio pass), rescale mass to
// preserve density (area ratio = ratioX*ratioY, the rect analogue of
// resizeBody's ratio^2), then commit the new half-dimensions and centre.
// Belt wrap radii need no rect-specific handling here the way resizeBody
// handles them for a circle: a belt's endpoints are restricted to circle
// bodies at creation (tools.js §13.5), so a rectangle can never be one.
function applyRectResize(b, newHw, newHh, newX, newY){
  newHw=Math.max(0.08,newHw); newHh=Math.max(0.08,newHh);
  const ratioX=newHw/b.hw, ratioY=newHh/b.hh;
  if(!isFinite(ratioX) || !isFinite(ratioY) || ratioX<=0 || ratioY<=0) return;
  for(const con of constraints){ scaleOffOnBodyXY(con.a,b.id,ratioX,ratioY); if(con.b) scaleOffOnBodyXY(con.b,b.id,ratioX,ratioY); }
  for(const cb of cables){ scaleOffOnBodyXY(cb.tether,b.id,ratioX,ratioY); }
  for(const sp of springs){ scaleOffOnBodyXY(sp.a,b.id,ratioX,ratioY); scaleOffOnBodyXY(sp.b,b.id,ratioX,ratioY); }
  b.mass*=ratioX*ratioY; b.hw=newHw; b.hh=newHh; b.x=newX; b.y=newY;
  refreshInertia(b);
  projectPositions(8);
}
// Resize by dragging one corner (tools.js §13.5/§13.6 resizeDrag), keeping
// the diagonally-opposite corner fixed in world space -- the same "two
// opposite corners" model the rectbody placement tool uses, just with one
// corner now anchored instead of both being fresh clicks. `corner` is the
// [+-1,+-1] local sign pair bodyCornerHit identified when the drag started.
function resizeRectCorner(b, corner, wx, wy){
  const [ox,oy]=worldPt(b, [-corner[0]*b.hw, -corner[1]*b.hh]);   // opposite corner, world, pre-resize
  const [dx,dy]=R(-b.th, wx-ox, wy-oy);
  const [cx,cy]=R(b.th, dx/2, dy/2);
  applyRectResize(b, Math.abs(dx)/2, Math.abs(dy)/2, ox+cx, oy+cy);
}
// Resize by typed width/height (inspector.js §14.2), centre held fixed --
// the rect analogue of resizeBody's radius field.
function resizeRectAxes(b, newHw, newHh){
  applyRectResize(b, newHw, newHh, b.x, b.y);
}
// A vessel resizes from a corner exactly as a rectangle does -- opposite corner
// held fixed in world space -- but the new half-extents land on bore and len rather
// than hw/hh, and the gas stays sealed: it keeps its mass and temperature, so its
// pressure is whatever the new volume implies (geometry.js §05.2d resizeVessel).
// Anchor offsets need only their *lateral* component rescaled: the axial one is a
// material fraction, already invariant under a length change by construction.
function resizeVesselCorner(v, corner, wx, wy){
  const [ox,oy]=worldPt(v, [-corner[0]*v.hw, -corner[1]*v.hh]);   // opposite corner, pre-resize
  const [dx,dy]=R(-v.th, wx-ox, wy-oy);
  const newBore=Math.max(0.02,Math.abs(dx)), newLen=Math.max(VESSEL_MIN_LEN,Math.abs(dy));
  const ratioLat=newBore/v.bore;
  for(const con of constraints){ scaleOffOnBodyXY(con.a,v.id,ratioLat,1); if(con.b) scaleOffOnBodyXY(con.b,v.id,ratioLat,1); }
  for(const cb of cables){ scaleOffOnBodyXY(cb.tether,v.id,ratioLat,1); }
  for(const sp of springs){ scaleOffOnBodyXY(sp.a,v.id,ratioLat,1); scaleOffOnBodyXY(sp.b,v.id,ratioLat,1); }
  resizeVessel(v, newBore, newLen);
  const [cx,cy]=R(v.th, dx/2, dy/2);
  v.x=ox+cx; v.y=oy+cy;
  projectPositions(8);
}
function applyBodyResize(rd, wx, wy){
  if(rd.b.shape==='vessel') resizeVesselCorner(rd.b, rd.corner, wx, wy);
  else if(rd.b.shape==='rect') resizeRectCorner(rd.b, rd.corner, wx, wy);
  else resizeBody(rd.b, Math.hypot(wx-rd.b.x,wy-rd.b.y));
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
    // §13.4/§13.5 below) so a constraint/cable coincident with a body is
    // still reachable instead of always losing to the body underneath it
    const cci=pickConstraint(wx,wy); if(cci>=0){ if(!constraints[cci].sel) hover=constraints[cci]; return; }
    const cbi=pickCable(wx,wy); if(cbi>=0){ if(!cables[cbi].sel) hover=cables[cbi]; return; }
    const spi=pickSpring(wx,wy); if(spi>=0){ if(!springs[spi].sel) hover=springs[spi]; return; }
    const rsi=pickRotSpring(wx,wy); if(rsi>=0){ if(!rotSprings[rsi].sel) hover=rotSprings[rsi]; return; }
    const bi=pickBody(wx,wy); if(bi>=0){ if(!bodies[bi].sel) hover=bodies[bi]; return; }
    return;
  }
  if(tool==='delete'){
    // interactions take priority over bodies, matching the delete order (§13.5)
    const cci=pickConstraint(wx,wy); if(cci>=0){ hover=constraints[cci]; return; }
    const cbi=pickCable(wx,wy); if(cbi>=0){ hover=cables[cbi]; return; }
    const spi=pickSpring(wx,wy); if(spi>=0){ hover=springs[spi]; return; }
    const rsi=pickRotSpring(wx,wy); if(rsi>=0){ hover=rotSprings[rsi]; return; }
    const bi=pickBody(wx,wy); if(bi>=0){ hover=bodies[bi]; return; }
    return;
  }
  if(tool==='slot'||tool==='cable'||tool==='pin'||tool==='rod'||tool==='spring'){
    // these tools attach to a snapped anchor (body centre/edge) or a bare body
    const t=anchorTarget(wx,wy); if(t){ hover=t.body; hoverSnap=t.snap; }
    return;
  }
  if(tool==='belt'||tool==='cvt'){
    // both are rim-based (a wrap radius / rolling contact) and restricted to
    // circle bodies at creation (§13.5) -- don't highlight a rectangle or a vessel
    // as if it were a valid pick for either.
    const bi=pickBody(wx,wy); if(bi>=0 && bodies[bi].shape==='circle') hover=bodies[bi];
    return;
  }
  if(tool==='knife'||tool==='rotspring'){
    const bi=pickBody(wx,wy); if(bi>=0) hover=bodies[bi];
    return;
  }
  // 'body'/'rectbody' tools have no existing element to highlight -- their
  // own live preview (bodyPreview, render.js §11.7) already shows where the
  // new body will go
}

function startPinch(){
  const pts=[...pointers.values()];
  const mx=(pts[0].x+pts[1].x)/2, my=(pts[0].y+pts[1].y)/2;
  pinch={ dist:Math.hypot(pts[0].x-pts[1].x,pts[0].y-pts[1].y)||1, midWorld:s2w(mx,my), scale:cam.scale };
}

// ---- §13.5 · pointerdown (per-tool dispatch) ----
// This is where each tool builds its constraint. The branches, in order, handle:
// pinch guard, explicit pan, select (+handles/resize/grab -- the only case
// that can claim a one-finger drag instead of panning), body, delete,
// belt/cvt, knife, cable, pin, rod, slot, spring, rotspring.
//
// Every non-select tool, and every select-tool click that doesn't land on
// a draggable control point (a handle, the selected body's resize rim) or
// pick a body to grab, arms `panning` as a fallback: pinch/drag/scroll pan and zoom
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
      if(h){ if(h.con.sel){ anchorDrag=h; applyHandle(h,wx,wy); return; }
        if(h.arr==='springs') selectSpring(h.ci); else selectConstraint(h.ci); return; }
      // the selected body's rim (or, for a rectangle, a corner) -- drag to
      // resize (§13.6 applyBodyResize)
      if(selBody && bodyRimHit(selBody,wx,wy)){
        resizeDrag = rectLike(selBody) ? {b:selBody, corner:bodyCornerHit(selBody,wx,wy)} : {b:selBody};
        return;
      }
    }
    // Beyond a control point (handled above), act on whatever is actually
    // topmost at this point -- matching updateHover and the delete-tool
    // order (interactions over bodies, §13.4). A held selection no longer
    // gets special priority here: it used to win outright over anything
    // occluding it, which meant a rod drawn in front of a selected body
    // couldn't be clicked-through to at all. Only a real control point
    // (a handle, or the resize rim above) is allowed to pre-empt this pick;
    // a plain interior/segment hit always goes to what's visually on top,
    // even when that isn't the current selection. Hitting the current
    // selection itself just re-arms its drag/pan (selectX on an already-
    // selected thing is a no-op past the redundant render).
    const cci=pickConstraint(wx,wy);
    if(cci>=0){ selectConstraint(cci); panning={sx:e.clientX,sy:e.clientY,cx:cam.x,cy:cam.y}; return; }
    const cbi=pickCable(wx,wy);
    if(cbi>=0){ selectCable(cbi); panning={sx:e.clientX,sy:e.clientY,cx:cam.x,cy:cam.y}; return; }
    const spi=pickSpring(wx,wy);
    if(spi>=0){ selectSpring(spi); panning={sx:e.clientX,sy:e.clientY,cx:cam.x,cy:cam.y}; return; }
    const rsi=pickRotSpring(wx,wy);
    if(rsi>=0){ selectRotSpring(rsi); panning={sx:e.clientX,sy:e.clientY,cx:cam.x,cy:cam.y}; return; }
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
    if(!bodyPreview){ bodyPreview={shape:'circle',cx:wx,cy:wy,r:0}; return; }
    const r=bodyPreview.r<0.12?0.4:bodyPreview.r;
    const b=makeBody(bodyPreview.cx,bodyPreview.cy,r,false); bodies.push(b); bodyPreview=null;
    selectBody(bodies.length-1); saveState();
    return;
  }
  if(tool==='rectbody'){
    // Two clicks naming opposite corners: first click plants one corner and
    // previews the box live to the cursor (drawPreview, render.js §11.7);
    // second click commits it. Always axis-aligned (th=0) -- like the circle
    // body tool, there's no third click to set an orientation.
    if(!bodyPreview){ bodyPreview={shape:'rect',x0:wx,y0:wy,x1:wx,y1:wy}; return; }
    let hw=Math.abs(bodyPreview.x1-bodyPreview.x0)/2, hh=Math.abs(bodyPreview.y1-bodyPreview.y0)/2;
    if(hw<0.06 && hh<0.06){ hw=0.4; hh=0.3; }               // a tap with no drag: a default-sized box
    hw=Math.max(0.08,hw); hh=Math.max(0.08,hh);
    const cx=(bodyPreview.x0+bodyPreview.x1)/2, cy=(bodyPreview.y0+bodyPreview.y1)/2;
    const b=makeRectBody(cx,cy,hw,hh,false); bodies.push(b); bodyPreview=null;
    selectBody(bodies.length-1); saveState();
    return;
  }
  if(tool==='vessel'){
    // Two clicks naming opposite corners, exactly as the rectangle tool -- but the
    // box is read as bore (width) by length (height), so the caps face up and down
    // at th = 0. The gas starts at ambient pressure and temperature, so a freshly
    // placed vessel is balanced against the atmosphere and sits still until
    // something acts on it.
    if(!bodyPreview){ bodyPreview={shape:'vessel',x0:wx,y0:wy,x1:wx,y1:wy}; return; }
    let bore=Math.abs(bodyPreview.x1-bodyPreview.x0), len=Math.abs(bodyPreview.y1-bodyPreview.y0);
    if(bore<0.06 && len<0.06){ bore=0.5; len=1.0; }          // a tap with no drag
    bore=Math.max(0.04,bore); len=Math.max(0.04,len);
    const cx=(bodyPreview.x0+bodyPreview.x1)/2, cy=(bodyPreview.y0+bodyPreview.y1)/2;
    bodies.push(makeVessel(cx,cy,bore,len,false)); bodyPreview=null;
    selectBody(bodies.length-1); saveState();
    return;
  }
  if(tool==='delete'){
    // interactions take priority over bodies (updateHover, §13.4, mirrors
    // this order) -- a constraint/cable coincident with a body is what most
    // often needs deleting without also taking the body out with it.
    const cci=pickConstraint(wx,wy); if(cci>=0){ constraints.splice(cci,1); clearSelection(); saveState(); return; }
    const cbi=pickCable(wx,wy); if(cbi>=0){ cables.splice(cbi,1); clearSelection(); saveState(); return; }
    const spi=pickSpring(wx,wy); if(spi>=0){ springs.splice(spi,1); clearSelection(); saveState(); return; }
    const rsi=pickRotSpring(wx,wy); if(rsi>=0){ rotSprings.splice(rsi,1); clearSelection(); saveState(); return; }
    const bi=pickBody(wx,wy);
    if(bi>=0){ const id=bodies[bi].id;
      constraints=constraints.filter(c=>c.a.id!==id && !(c.b&&c.b.id===id));
      springs=springs.filter(s=>s.a.id!==id && !(s.b&&s.b.id===id));
      rotSprings=rotSprings.filter(s=>s.a.id!==id && s.b.id!==id);
      bodies.splice(bi,1); clearSelection(); saveState(); }
    return;
  }
  if(tool==='belt' || tool==='cvt'){
    // two bodies: A first, then B (occluded B reachable via except-pick).
    // Both are rim-based (a wrap radius for belt, a rolling contact for
    // cvt) and only make sense between circular bodies -- neither a rectangle nor
    // a vessel is a valid pick for either end.
    if(!pending){ const bi=pickBody(wx,wy); if(bi<0||bodies[bi].shape!=='circle')return; pending={id:bodies[bi].id, wp:[wx,wy]}; return; }
    const bi2=pickBodyExcept(wx,wy,pending.id); if(bi2<0||bodies[bi2].shape!=='circle')return;
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
    // SECOND click = spool body -- a cable winds around a rim, so (like
    // belt/cvt above) the spool must be circular; the tether end above is
    // unrestricted since it's just an attachment point, not a wound rim.
    const bi=pickBody(wx,wy); if(bi<0) return;
    const S=bodies[bi]; if(S.shape!=='circle') return;     // a spool is always a disk
    if(pending.tid!=null && S.id===pending.tid) return;
    const T = pending.tid!=null ? (()=>{ const tb=bodies[bodyIndex(pending.tid)]; const [x,y]=epWorldPt(tb,pending.toff); return [x,y]; })()
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
    // FIRST pick -- snapped anchor on body A, pending.wp is the pivot world point
    if(!pending){ const t=anchorTarget(wx,wy); if(!t) return;
      pending={ep:{id:t.body.id, off:offOf(t.body,t.wp)}, wp:t.wp}; return; }
    const Aep=pending.ep;
    // the second click only *names* body B -- click any part of it, including
    // where A covers it -- and B is anchored at the first pivot.
    let Bep=null;
    const bi = Aep.id!=null ? pickBodyExcept(wx,wy,Aep.id) : pickBody(wx,wy);
    if(bi>=0){ Bep={id:bodies[bi].id, off:offOf(bodies[bi],pending.wp)}; }
    else { const s=snapAnchor(wx,wy); if(s && s.body.id!==Aep.id) Bep={id:s.body.id, off:offOf(s.body,pending.wp)}; }
    if(!Bep) return;   // nothing indicated -- keep the pivot and wait
    if(Aep.id!=null && Bep.id===Aep.id) return;
    constraints.push({type:'pin', a:Aep, b:Bep, sel:false});
    pending=null; saveState();
    return;
  }
  if(tool==='rod'){
    // Each end snaps to a body if one is under/near the click, else attaches
    // to the background at the raw world point (id:null). pending wraps
    // {ep,wp} (not the bare endpoint) so drawPending (render.js §11.7) still
    // has a plain world point to draw the pending anchor at.
    if(!pending){ const t=anchorTarget(wx,wy);
      pending = { ep: t ? {id:t.body.id, off:offOf(t.body,t.wp)} : {id:null, off:[wx,wy]}, wp:[wx,wy] };
      return; }
    const t=anchorTarget(wx,wy);
    const Bep = t ? {id:t.body.id, off:offOf(t.body,t.wp)} : {id:null, off:[wx,wy]};
    const Aep=pending.ep;
    if(Aep.id==null && Bep.id==null) return; // a rod needs at least one real target -- keep pending, wait for a better second click
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
  if(tool==='spring'){
    // Two clicks, exactly like rod: each end snaps to a body if one is
    // under/near the click, else attaches to the background. Rest length
    // defaults to the length at creation (makeSpringCon, constraints.js
    // §06.6) -- no weld concept here, a spring only ever pulls/pushes along
    // its own line.
    if(!pending){ const t=anchorTarget(wx,wy);
      pending = { ep: t ? {id:t.body.id, off:offOf(t.body,t.wp)} : {id:null, off:[wx,wy]}, wp:[wx,wy] };
      return; }
    const t=anchorTarget(wx,wy);
    const Bep = t ? {id:t.body.id, off:offOf(t.body,t.wp)} : {id:null, off:[wx,wy]};
    const Aep=pending.ep;
    if(Aep.id==null && Bep.id==null) return; // a spring needs at least one real target -- keep pending, wait for a better second click
    if(Aep.id!=null && Bep.id===Aep.id) return;    // can't spring a body to itself -- ditto
    pending=null;
    springs.push(makeSpringCon(Aep, Bep));
    saveState();
    return;
  }
  if(tool==='rotspring'){
    // Two bodies, like belt/cvt -- no offset, the whole body frame's theta is
    // the feature -- but unlike belt/cvt an empty-space click is a valid
    // pick too (id:null, the background reads as a fixed theta=0 reference,
    // constraints.js §06.6). The rest angle captures whatever relative angle
    // is live at creation, so a freshly-placed rotational spring starts
    // unstressed.
    if(!pending){ const bi=pickBody(wx,wy);
      pending = bi>=0 ? {id:bodies[bi].id, wp:[bodies[bi].x,bodies[bi].y]} : {id:null, wp:[wx,wy]};
      return; }
    const bi2 = pending.id!=null ? pickBodyExcept(wx,wy,pending.id) : pickBody(wx,wy);
    if(bi2<0) return;   // second pick must land on a real body -- keep pending, wait
    const Bid=bodies[bi2].id;
    if(pending.id!=null && Bid===pending.id) return;   // can't spring a body to itself
    const Aid=pending.id;
    pending=null;
    rotSprings.push(makeRotSpringCon(Aid, Bid));
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
  if(bodyPreview){
    if(bodyPreview.shape==='rect'||bodyPreview.shape==='vessel'){ bodyPreview.x1=mouseWorld[0]; bodyPreview.y1=mouseWorld[1]; }
    else { bodyPreview.r=Math.hypot(mouseWorld[0]-bodyPreview.cx,mouseWorld[1]-bodyPreview.cy); }
    return;
  }
  if(drag && !sim.running){
    const G=bodies[drag.bi];
    if(G.static){
      // move the root kinematically; the island follows it
      const [gx,gy]=epWorldPt(G,drag.off); G.x+=mouseWorld[0]-gx; G.y+=mouseWorld[1]-gy;
      projectPositions(8);
    } else {
      // pull the grabbed point toward the cursor; the island articulates to comply.
      // The goal is capped by screen-space distance (§05.4 saturatingPull), not
      // set to the raw cursor position: a hard pin straight to the cursor is one
      // more rigid row demanding exact coincidence, and when the drag has any
      // component the body's real constraints can't satisfy (dragging a rod
      // welded to the background, or a slider, off its rail), that extra row
      // fights the real ones for the same few DOF and shows up as a violation
      // on them even though the reachable part of the drag is perfectly posable.
      // Capping how far the goal itself can get from the body's current point
      // keeps that tug bounded instead of ever-growing, so the real constraints
      // stay solved and only the unreachable sliver of the drag goes unmet.
      // 'dragpin' is an internal-only row type (§06.5) -- never added to
      // `constraints`, just fed through projectPositions as a transient goal.
      const [gx,gy]=epWorldPt(G,drag.off);
      const [px,py]=saturatingPull(gx,gy,mouseWorld[0],mouseWorld[1],DRAG_CAP_PX);
      const temp={type:'dragpin', a:{id:G.id, off:drag.off}, world:[gx+px,gy+py]};
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
