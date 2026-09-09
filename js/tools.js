// ============================================================================
//  §13 · TOOLS & INPUT
//  The tool palette and all pointer handling: hit-testing, snapping, constraint
//  handle editing, and the big pointer-down dispatch that builds constraints.
//    §13.1  tool table + rail build + setTool
//    §13.2  picking & snapping (pickBody/Constraint, distSeg, snapAnchor, ...)
//    §13.3  constraint handles (conHandles, pickHandle, applyHandle)
//    §13.4  pointer state (multi-touch map, pinch, cancelSingle)
//    §13.5  pointerdown  (per-tool dispatch -- where constraints are created, and
//                        where the lasso's loop and the selection box's transform
//                        gestures start -- select.js §18)
//    §13.6  pointermove  (drag/pan/pinch/handle articulation; poseDragTo, where
//                        a posable rod is released -- constraints.js §06.2d)
//    §13.7  pointerup / cancel / wheel
// ============================================================================
// ---- §13.1 · tool table + rail build + setTool ----
const TOOLS=[
  {id:'select',key:'1',tip:'Select / move (1)',svg:'<path d="M5 3l7 16 2-6 6-2z"/>'},
  {id:'lasso',key:'l',tip:'Lasso many \u2014 loop them, or tap to add/remove (l)',svg:'<path d="M12 4c4.4 0 8 2 8 4.5S16.4 13 12 13 4 11 4 8.5 7.6 4 12 4z"/><path d="M9 12.6c-.8 2.4-2.2 3.6-2.2 5.2a1.6 1.6 0 1 0 3.2 0"/>'},
  {id:'body',key:'2',tip:'Add body (2)',svg:'<circle cx="12" cy="12" r="7"/><path d="M12 8v8M8 12h8"/>'},
  {id:'rectbody',key:'q',tip:'Add rectangle (q)',svg:'<rect x="4" y="6" width="16" height="12" rx="1"/><path d="M12 8v8M6 12h12"/>'},
  {id:'vessel',key:'g',tip:'Add gas vessel (g)',svg:'<rect x="7" y="3" width="10" height="18" rx="1"/><path d="M7 5.5h10M7 18.5h10" stroke-width="2.6"/>'},
  {id:'vertex',key:'3',tip:'Vertex \u2014 a named point; tap it again to join another body (3)',svg:'<circle cx="12" cy="12" r="3"/><circle cx="12" cy="12" r="8"/>'},
  {id:'line',key:'4',tip:'Line \u2014 tap vertices in turn; hold the toggle to extend one (4)',svg:'<circle cx="5" cy="19" r="2"/><circle cx="12" cy="12" r="2"/><circle cx="19" cy="5" r="2"/><path d="M3 21L21 3"/>'},
  {id:'belt',key:'b',tip:'Belt (b)',svg:'<circle cx="7" cy="12" r="4"/><circle cx="17" cy="12" r="4"/><path d="M7 8h10M7 16h10"/>'},
  {id:'knife',key:'k',tip:'Knife-edge wheel (k)',svg:'<path d="M4 16h16"/><path d="M12 16l3-9 3 9"/><circle cx="8" cy="16" r="1.5"/>'},
  {id:'cvt',key:'v',tip:'Variable gear / CVT (v)',svg:'<circle cx="9" cy="12" r="6"/><circle cx="17" cy="12" r="3"/><path d="M9 12h8"/>'},
  {id:'cable',key:'c',tip:'Cable (c)',svg:'<circle cx="16" cy="9" r="4"/><path d="M4 19c6 0 8-4 9-7"/><circle cx="4" cy="19" r="1.5"/>'},
  {id:'rotspring',key:'9',tip:'Rotational spring (9)',svg:'<path d="M12 3a9 9 0 1 0 9 9"/><path d="M12 7a5 5 0 1 0 5 5"/><path d="M12 11a1 1 0 1 0 1 1"/>'},
  {id:'heat',key:'h',tip:'Heat interaction (h)',svg:'<path d="M8 3c-2 3 1 3-1 6-2 3 1 5 2 5 2 0 3-2 2-4 2 1 3 3 1 6-3 2-6 0-6-3 0-2 1-3 2-5-1-1-1-3 0-5z"/>'},
  {id:'flow',key:'f',tip:'Mass-flow interaction (f)',svg:'<path d="M4 12c3-4 6 4 9 0M4 8c3-4 6 4 9 0M4 16c3-4 6 4 9 0"/><path d="M17 8l3 4-3 4"/>'},
  {id:'delete',key:'7',tip:'Delete (7)',svg:'<path d="M6 7h12M9 7V5h6v2M8 7l1 12h6l1-12"/>'},
];
let tool='select';
// The line tool's two modes (VERTEX.md §X.11). NEW starts a line from two vertices;
// EXTEND starts by picking an existing one. It is one flag rather than two tools
// because the gesture after the first tap is identical either way -- tap vertices,
// they join the line.
let lineExtendMode=false;
const rail=document.getElementById('rail');
TOOLS.forEach((t,i)=>{
  // separators between the tool families; the indices move whenever the table
  // above does -- select/lasso, then the body makers, the joints, the force
  // elements, the interactions, and delete on its own
  if(i===2||i===5||i===10||i===12||i===14){ const s=document.createElement('div');s.className='rail-sep';rail.appendChild(s);}
  const el=document.createElement('button');el.className='tool';el.dataset.id=t.id;
  el.innerHTML=`<svg viewBox="0 0 24 24">${t.svg}</svg><span class="kbd">${t.key}</span><span class="tip">${t.tip}</span>`;
  el.onclick=()=>setTool(t.id); rail.appendChild(el);
});
function setLineMode(extend){
  lineExtendMode=!!extend; pending=null;
  const b=document.getElementById('lineMode');
  if(b){ b.textContent = lineExtendMode?'extend':'new'; b.classList.toggle('on', lineExtendMode); }
  const t=TOOLS.find(x=>x.id==='line');
  if(t && tool==='line') document.getElementById('modehint').textContent =
    lineExtendMode ? 'Line \u2014 tap a line, then tap vertices to join them to it (4)'
                   : 'Line \u2014 tap vertices in turn; the first two place the line (4)';
}
function setTool(id){ tool=id; pending=null; bodyPreview=null; hover=null; hoverHandle=null; hoverSnap=null;
  const lm=document.getElementById('lineMode'); if(lm) lm.style.display = id==='line' ? '' : 'none';
  document.querySelectorAll('.tool').forEach(e=>e.classList.toggle('on',e.dataset.id===id));
  cv.style.cursor = id==='select'?'default': id==='delete'?'not-allowed':'crosshair';
  document.getElementById('modehint').textContent = TOOLS.find(t=>t.id===id).tip;
  if(id==='line') setLineMode(lineExtendMode);
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
  if(con.type==='line'){
    // A BAR is hit along its segment; a RAIL is an infinite line, hit by perpendicular
    // distance to it, exactly as the slot's was (constraints.js §06.2f lineIsBar).
    // The same placement the canvas draws (constraints.js §06.2f): what is picked has
    // to be what is seen, including a line whose joints have lost their bodies.
    const f=linePlacement(con); if(!f) return false;
    if(lineIsBar(con)) return distSeg(wx,wy,f.wax,f.way,f.wbx,f.wby)<=tol;
    return Math.abs(f.nx*(wx-f.wax)+f.ny*(wy-f.way))<=tol;
  }
  if(con.type==='vertex'){
    // A dot, not a segment: the vertex IS the point. Picked a little more generously
    // than the tolerance a line gets, because it is the thing most often sitting
    // underneath one.
    const [ax,ay]=vertexWorld(con);
    const r=VERTEX_PICK_PX/cam.scale;
    return (wx-ax)**2+(wy-ay)**2<=r*r;
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
// Heat/mass interaction hit test, shared by both kinds -- the same dashed segment
// render.js §11.4c draws, from the mediating body's centre to the vessel's (or a
// short stub toward the background).
function interactionHit(it,wx,wy){ const tol=12/cam.scale; const ep=interactionEndpoints(it); if(!ep) return false;
  return distSeg(wx,wy,ep.p0[0],ep.p0[1],ep.p1[0],ep.p1[1])<=tol; }
function pickInteraction(wx,wy){
  for(let i=interactions.length-1;i>=0;i--){ if(interactionHit(interactions[i],wx,wy)) return i; }
  return -1; }
// Topmost vessel under the cursor, or -1 -- the second pick of an interaction tool.
// Missing it means "the background", which is a deliberate target, not a miss.
function pickVessel(wx,wy){
  for(let i=bodies.length-1;i>=0;i--){ const b=bodies[i];
    if(b.shape==='vessel' && bodyContains(b,wx,wy)) return i; }
  return -1; }
// Every interaction that names body `id` on EITHER side -- as the mediating body or
// as the vessel -- dies with it. Called from every body-deletion path (this file's
// delete tool, inspector.js §14.2/§14.2b, transport.js §16.4), the same way each of
// those already drops the constraints/cables that named it.
function dropInteractionsOn(id){
  interactions=interactions.filter(it=>it.body.id!==id && it.vessel.id!==id);
}
// rotSpring hit test, same role as cableHit above -- rotational springs
// live in their own arrays (constraints.js §06.6), not `constraints`, so they
// get their own pick path (pickRotSpring, inspector.js §14.1)
// rather than going through pickConstraint.
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
// One handle per control point, carrying its index so
// pickHandle/applyHandle can name the point back -- `which` alone cannot, since every
// one of them is a 'pt'.
function conHandles(con){
  // A vertex is one point wearing several bodies' offsets, so it has exactly one
  // handle and dragging it moves the whole coincident set (§06.2e setVertexWorld) --
  // which is what the pin's pivot handle did, on an object that now says so.
  if(con.type==='vertex'){ const [x,y]=vertexWorld(con); return [{which:'pivot',x,y}]; }
  // Only the knife carries handles from here on, and it is the one kind left whose
  // `a` is a body-frame anchor. Everything else -- a line, a belt, a CVT -- is either
  // edited through its vertices or has nothing to drag.
  if(con.type!=='knife') return [];
  const A=bodies[bodyIndex(con.a.id)]; if(!A) return [];
  { const [px,py]=epWorldPt(A,con.a.off);
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
  const selObj = selConstraint;
  const arrOf = o => selConstraint===o ? 'constraints' : null;
  if(selObj){
    for(const h of conHandles(selObj)){
      if(Math.hypot(wx-h.x,wy-h.y)<=tol) return {con:selObj,which:h.which,k:h.k,
        ci:constraints.indexOf(selObj), arr:arrOf(selObj)}; } }
  for(let i=constraints.length-1;i>=0;i--){ const con=constraints[i]; if(con===selObj) continue;
    for(const h of conHandles(con)){ if(Math.hypot(wx-h.x,wy-h.y)<=tol) return {con,which:h.which,k:h.k,ci:i,arr:'constraints'}; } }
  return null;
}
// move an anchor while editing; snaps and (for rods) can re-bind to another body
function applyHandle(ad, wx, wy){
  const con=ad.con;
  // Every control point shares one handle behaviour,
  // whatever constraint carries it, so it is handled ahead of the per-type branches.
  if(con.type==='vertex'){
    // Snap only to the bodies this vertex already touches (snapAnchor's third
    // argument is a whitelist), because those are the ones whose rims and centres the
    // point is being placed against -- the pin's pivot drag did the same. Re-binding
    // a vertex to a DIFFERENT body is joining it, which is the tool's job (§13.5),
    // not a drag's.
    const s=snapAnchor(wx,wy, vertexOns(con).map(e=>e.id).filter(id=>id!=null));
    lastSnap=s; const P=s?s.wp:[wx,wy];
    setVertexWorld(con, P[0], P[1]);
    // ...and every station on every line it is joined to follows the hand. A station
    // is where a joint sits along the bar, so dragging the joint is what SETS it --
    // the rod's endpoint drag, which redefined the rod's length, said in the line's
    // vocabulary. The drag is where that belongs and a typed coordinate is not: a
    // number committed in the panel is a solve ATTEMPT the mechanism may refuse
    // (VERTEX.md §X.11), while a hand on the bar is the geometry the gesture left,
    // which is the rule every pose drag and every scaled box already follows
    // (constraints.js §06.2b recaptureConPose).
    //
    // Every station, not just this joint's, because a station is a distance from the
    // ORIGIN: drag the origin joint -- the one at station 0 -- and it stays at 0 while
    // every other station on the bar moves, the origin having gone somewhere else.
    // recaptureLineStations re-reads them all off the live geometry, which is the one
    // answer to both cases, and it leaves any vertex the line CARRIES where it is.
    for(const e of vertexOns(con)){
      if(!isLineOn(e) || !e.join) continue;
      const line=lineById(e.id); if(line) recaptureLineStations(line);
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
    // conEndpoints, not a.off/b.off by name: every extra control point (§06.2c) is
    // an anchor on a body too, and has to ride the resize like the base pair's.
    for(const ep of conEndpoints(con)) scaleOffOnBody(ep, b.id, ratio);
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
  for(const con of constraints) for(const ep of conEndpoints(con)) scaleOffOnBodyXY(ep,b.id,ratioX,ratioY);
  for(const cb of cables){ scaleOffOnBodyXY(cb.tether,b.id,ratioX,ratioY); }
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
  for(const con of constraints) for(const ep of conEndpoints(con)) scaleOffOnBodyXY(ep,v.id,ratioLat,1);
  for(const cb of cables){ scaleOffOnBodyXY(cb.tether,v.id,ratioLat,1); }
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
function cancelSingle(){ drag=null; endPosing(); grab=null; bodyPreview=null; panning=null; anchorDrag=null; lastSnap=null; resizeDrag=null; groupDrag=null; lasso=null; clickArmed=false; }

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
    if(selGroup){
      // The selection box's own handles come first, for the same reason a control
      // point does: they are what the cursor is actually over (select.js §18.2).
      // Inside the box nothing else highlights either -- the whole region is one
      // target, the group. Outside it the ordinary pick chain still runs, so the
      // click that changes the selection still shows what it would land on.
      const gh=pickGroupHandle(wx,wy);
      if(gh){ hoverHandle={kind:'group', which:gh.kind, k:gh.k}; return; }
      if(groupContains(selGroup,wx,wy)) return;
    } else {
      // a control point of the already-selected interaction takes priority
      const ch=pickCableHandle(wx,wy);
      if(ch && ch.cb.sel){ hoverHandle={kind:'cable',cb:ch.cb,which:ch.which}; return; }
      const h=pickHandle(wx,wy);
      if(h && h.con.sel){ hoverHandle={kind:'con',con:h.con,which:h.which,k:h.k}; return; }
      // ...as does the selected body's rim, the resize handle
      if(selBody && bodyRimHit(selBody,wx,wy)){ hoverHandle={kind:'resize',b:selBody}; return; }
    }
    // otherwise highlight whatever is under the cursor, unless it's the selection
    // -- interactions take priority over bodies (matching the delete order,
    // §13.4/§13.5 below) so a constraint/cable coincident with a body is
    // still reachable instead of always losing to the body underneath it
    const cci=pickConstraint(wx,wy); if(cci>=0){ if(!constraints[cci].sel) hover=constraints[cci]; return; }
    const cbi=pickCable(wx,wy); if(cbi>=0){ if(!cables[cbi].sel) hover=cables[cbi]; return; }
    const rsi=pickRotSpring(wx,wy); if(rsi>=0){ if(!rotSprings[rsi].sel) hover=rotSprings[rsi]; return; }
    const ii=pickInteraction(wx,wy); if(ii>=0){ if(!interactions[ii].sel) hover=interactions[ii]; return; }
    const bi=pickBody(wx,wy); if(bi>=0){ if(!bodies[bi].sel) hover=bodies[bi]; return; }
    return;
  }
  if(tool==='lasso'){
    // a tap adds or removes the body under the cursor (select.js §18.3), so show
    // which one that would be; the loop itself needs no hover
    const bi=pickBody(wx,wy); if(bi>=0) hover=bodies[bi];
    return;
  }
  if(tool==='delete'){
    // interactions take priority over bodies, matching the delete order (§13.5)
    const cci=pickConstraint(wx,wy); if(cci>=0){ hover=constraints[cci]; return; }
    const cbi=pickCable(wx,wy); if(cbi>=0){ hover=cables[cbi]; return; }
    const rsi=pickRotSpring(wx,wy); if(rsi>=0){ hover=rotSprings[rsi]; return; }
    const ii=pickInteraction(wx,wy); if(ii>=0){ hover=interactions[ii]; return; }
    const bi=pickBody(wx,wy); if(bi>=0){ hover=bodies[bi]; return; }
    return;
  }
  if(tool==='cable'||tool==='vertex'||tool==='line'){
    // A VERTEX under the cursor takes the click as another body joined to IT, so it
    // is what the highlight should name -- the same rule as the joint below, on the
    // object whose own pick it is (§13.5 pickVertexAt).
    if(tool==='vertex' || tool==='line'){
      const vi=pickVertexAt(wx,wy); if(vi>=0){ hover=constraints[vi]; return; } }
    // In EXTEND mode the first tap picks the LINE to extend, so that is what the
    // highlight names rather than whatever body is behind it.
    if(tool==='line' && !pending && lineExtendMode){
      const ci=pickConstraintOfType('line',wx,wy); if(ci>=0){ hover=constraints[ci]; return; } }
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
  if(tool==='heat'||tool==='flow'){
    // First pick is the mediating body (any body), second is the vessel it couples
    // through that body -- or empty space, which reads as the background. Highlight
    // whichever of the two the current click would take.
    if(!pending){ const bi=pickBody(wx,wy); if(bi>=0) hover=bodies[bi]; return; }
    const vi=pickVessel(wx,wy); if(vi>=0) hover=bodies[vi];
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
// belt/cvt, knife, cable, vertex, line, rotspring.
//
// The VERTEX and LINE tools check first whether a vertex is already under the cursor
// and take it rather than planting a second one on top of it, which is what makes a
// hinge two taps in one place and a bar two taps on two vertices (§13.5).
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

  // The lasso draws its loop directly, rather than deferring to a tap like every
  // other non-select tool below: the gesture IS the drag, so there is no click to
  // arm and nothing to pan. `clickArmed` still goes up, because a lasso that never
  // moves is a tap, and a tap adds or removes one body (§13.7 -> §18.3).
  if(tool==='lasso'){ lasso={pts:[[wx,wy]]}; clickArmed=true; return; }

  if(tool==='select'){
    // The selection box owns its own handles and its own interior (select.js
    // §18.2): a corner scales, the stem turns, anything inside moves the whole
    // group. It comes first because the box is drawn over everything, and a handle
    // sitting on top of a body has to be the thing the click reaches. It is also
    // the ONLY control-point editing on offer while a group is up: a member joint's
    // own handles would move an end out from under the box's capture, so they are
    // neither drawn (render.js §11.7) nor picked (below) until the group is gone.
    // A click outside the box falls through to the ordinary pick chain, which is
    // how the selection is changed.
    if(!sim.running && selGroup){
      const gh=pickGroupHandle(wx,wy);
      if(gh){ groupDrag=beginGroupDrag(gh,wx,wy); return; }
      if(groupContains(selGroup,wx,wy)){ groupDrag=beginGroupDrag({kind:'move'},wx,wy); return; }
    }
    if(!sim.running && !selGroup){
      // cable handle check first: allows winding control in edit mode, but only
      // once the cable is already the selection -- a click on an unselected
      // cable's handle just selects it, matching how constraint handles behave.
      // Arm the drag but don't apply it yet -- applying here, at the raw
      // pointerdown point, would resnap/reposition the control point even for a
      // tap that never moves (movedFar stays false), which is exactly the
      // gesture a rod/slot end uses to toggle its weld/prismatic lock in place
      // (§13.7 endPointer) rather than relocate. Deferring to pointermove (§13.6)
      // means a pure tap never calls apply*Handle at all -- the point only ever
      // moves once the drag actually does.
      const ch=pickCableHandle(wx,wy);
      if(ch){ if(ch.cb.sel){ anchorDrag=ch; return; } selectCable(ch.cbi); return; }
      const h=pickHandle(wx,wy);
      if(h){ if(h.con.sel){ anchorDrag=h; return; }
        selectConstraint(h.ci); return; }
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
    const rsi=pickRotSpring(wx,wy);
    if(rsi>=0){ selectRotSpring(rsi); panning={sx:e.clientX,sy:e.clientY,cx:cam.x,cy:cam.y}; return; }
    const ii=pickInteraction(wx,wy);
    if(ii>=0){ selectInteraction(ii); panning={sx:e.clientX,sy:e.clientY,cx:cam.x,cy:cam.y}; return; }
    const bi=pickBody(wx,wy);
    if(bi>=0){ selectBody(bi);
      if(sim.running){ grab={bi, off:localOff(bi,wx,wy)}; }
      // beginPosing names the body a posable rod is released around for the whole
      // gesture (constraints.js §06.2d) -- what the canvas draws as a rail. It goes
      // wherever `drag` goes, here and at both of its clears below.
      else { drag={bi, off:localOff(bi,wx,wy)}; beginPosing(bodies[bi].id); }
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
// A pinion is claimed by a click landing near a disk's PERIMETER -- where a rack
// line crossing a wheel actually meshes with it. Anything further in is a click
// through the body, which reads as jointing it to the rack instead. Screen-space, so
// the distinction is the same gesture at every zoom.
// Topmost LINE under the cursor, or -1 -- the pick the line tool's EXTEND mode uses
// to say which line the taps after it are joining vertices to.
function pickConstraintOfType(type,wx,wy){
  for(let i=constraints.length-1;i>=0;i--){
    const c=constraints[i]; if(c.type===type && constraintHit(c,wx,wy)) return i; }
  return -1;
}
// Join a vertex to a line, sliding, and bring it onto the line if it is not already
// there. This is the quick solve VERTEX.md §X.4 asks for, and it is the ordinary
// position projection (§09.1) with nothing added: the new on-line row is a row like
// any other, so asking the assembly to satisfy it IS making the points colinear.
// Nothing moves for the first two joints, whose own placement defines the line.
function joinVertexToLine(line, v){
  if(vertexOns(v).some(e=>e.id===line.id)) return false;
  const [wx,wy]=vertexWorld(v);
  makeVertexOn(v, {id:line.id}, {join:true, slide:true});
  // A vertex nothing grounds is now CARRIED by this line, which needs the station to
  // carry it by (constraints.js §06.2e). One that something grounds is unaffected.
  settleVertex(v, wx, wy);
  projectPositions(12);
  return true;
}
// because the vertex tool has to find one even where a rod is drawn on top of it.
const VERTEX_PICK_PX = 11;
function pickVertexAt(wx,wy){
  const r=VERTEX_PICK_PX/cam.scale;
  for(let i=constraints.length-1;i>=0;i--){ const c=constraints[i];
    if(!isVertex(c)) continue;
    const [x,y]=vertexWorld(c);
    if((wx-x)**2+(wy-y)**2<=r*r) return i; }
  return -1;
}
// Add the topmost body under the cursor to an existing vertex, at the vertex's own
// point. Bodies it already touches are looked THROUGH (see the vertex tool above),
// which is what lets the same tap twice in the same place pin two bodies together.
function joinBodyToVertex(v, wx, wy){
  const [px,py]=vertexWorld(v);
  const taken=vertexOns(v).map(e=>e.id);
  for(let i=bodies.length-1;i>=0;i--){ const b=bodies[i];
    if(!bodyContains(b,wx,wy)) continue;
    if(taken.includes(b.id)) continue;
    makeVertexOn(v, {id:b.id, off:offOf(b,[px,py])}, {join:true});
    return true; }
  return false;
}
// the click logic for every non-select tool, run only on a confirmed tap
// (pointerdown that never turned into a drag) -- see the pointerdown handler
// above for why this is deferred instead of firing immediately.
function runToolClick(wx,wy){
  if(tool==='lasso'){ lassoToggle(wx,wy); return; }
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
  if(tool==='heat' || tool==='flow'){
    // Two clicks: the mediating BODY first, then the VESSEL it is to couple through
    // that body -- or empty space, which names the background (sim.bg). One
    // interaction on its own moves nothing; it takes a second one on the same body,
    // naming the far side, to make a pair (physics.js §08.0b). That is deliberate:
    // the body is a wall, and a wall with only one side against something is not a
    // path. Placing the pair is two runs of this tool.
    if(!pending){ const bi=pickBody(wx,wy); if(bi<0) return;
      pending={interaction:tool, bodyId:bodies[bi].id, wp:[bodies[bi].x,bodies[bi].y]}; return; }
    const bodyId=pending.bodyId; pending=null;
    const vi=pickVessel(wx,wy);
    const vesselId = vi>=0 ? bodies[vi].id : null;
    if(vesselId===bodyId) return;                 // a body cannot be its own far side
    interactions.push(makeInteraction(tool, bodyId, vesselId));
    selectInteraction(interactions.length-1); saveState();
    return;
  }
  if(tool==='delete'){
    // interactions take priority over bodies (updateHover, §13.4, mirrors
    // this order) -- a constraint/cable coincident with a body is what most
    // often needs deleting without also taking the body out with it.
    // deleteConstraint, not a splice: a LINE is named by its joints' incidences, and
    // taking it out without them leaves `on=` tokens pointing at nothing -- a bench
    // that cannot reload the scene it just wrote (constraints.js §06.2b).
    const cci=pickConstraint(wx,wy); if(cci>=0){ deleteConstraint(constraints[cci]); clearSelection(); saveState(); return; }
    const cbi=pickCable(wx,wy); if(cbi>=0){ cables.splice(cbi,1); clearSelection(); saveState(); return; }
    const rsi=pickRotSpring(wx,wy); if(rsi>=0){ rotSprings.splice(rsi,1); clearSelection(); saveState(); return; }
    const ii=pickInteraction(wx,wy); if(ii>=0){ interactions.splice(ii,1); clearSelection(); saveState(); return; }
    const bi=pickBody(wx,wy);
    if(bi>=0){ const id=bodies[bi].id;
      dropBodyFromConstraints(id);
      rotSprings=rotSprings.filter(s=>s.a.id!==id && s.b.id!==id);
      dropInteractionsOn(id);
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
    constraints.push(tool==='belt' ? makeBeltCon(A.id,B.id,1) : makeCvtCon(A.id,B.id));
    pending=null; saveState();
    return;
  }
  if(tool==='knife'){
    // contact point on a body, then a heading direction
    if(!pending){ const t=anchorTarget(wx,wy); if(!t) return;
      pending={id:t.body.id, off:offOf(t.body,t.wp), wp:t.wp}; return; }
    const dx=wx-pending.wp[0], dy=wy-pending.wp[1]; const L=Math.hypot(dx,dy); if(L<0.15) return;
    const A=bodies[bodyIndex(pending.id)];
    constraints.push(makeKnifeCon({id:pending.id, off:pending.off}, R(-A.th, dx/L, dy/L)));
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
    const tether={id:pending.tid, off:pending.toff};
    const [tx,ty]=epWorld(tether);
    if(Math.hypot(tx-S.x,ty-S.y)<1e-6) return;   // degenerate: tether sits on the spool centre
    cables.push(makeCableCon(tether, S.id));
    pending=null; saveState();
    return;
  }
  if(tool==='vertex'){
    // ONE tap plants a point, wherever it lands and whatever it lands on (below).
    //
    // On an EXISTING vertex it JOINS a body to it instead: the topmost body under the
    // cursor the vertex does not already touch, at the vertex's own point. So pinning
    // two bodies together is the same gesture three times in the same place -- one tap
    // for the point, then one for each body, reaching down through them in draw order.
    // A vessel is no exception here, unlike on a rod: a rod's two ends may ride two
    // material planes of one vessel, but a VERTEX is at one material place on each
    // thing it touches, so it takes one incidence per body.
    const vi=pickVertexAt(wx,wy);
    if(vi>=0){ joinBodyToVertex(constraints[vi], wx, wy); selectConstraint(vi); saveState(); return; }
    // A fresh vertex is a POINT and nothing else: one background incidence, unjoined,
    // holding the world coordinates of the place tapped. It is not attached to the
    // body it happens to have landed on -- which body that would be is a question
    // about draw order, and a point should not silently take its frame from whatever
    // is on top. Attaching is a tick in the panel, where every body the point is
    // inside is already listed (constraints.js §06.2e, VERTEX.md §X.11). The snap is
    // still honoured, so a tap near a rim or a centre lands exactly on it.
    const t=anchorTarget(wx,wy);
    const wp = t ? t.wp : [wx,wy];
    const v=makeVertex(null);
    makeVertexOn(v, {id:null, off:[wp[0],wp[1]]}, {join:false});
    constraints.push(v); selectConstraint(constraints.length-1); saveState();
    return;
  }
  if(tool==='line'){
    // TAP VERTICES IN TURN. Each tap takes the vertex under the cursor, or plants one
    // where you tapped (on a body, or on the background) if there is none there.
    //
    //   NEW      the first two taps place a line through those two vertices; every
    //            tap after that joins another vertex to the SAME line, so a rail with
    //            three riders is five taps and never a mode change.
    //   EXTEND   the first tap picks an existing line instead, and every tap after
    //            joins a vertex to it. Which is the same gesture, started differently.
    //
    // Every joint is created SLIDING (constraints.js §06.2f), so the first two ask the
    // solver for nothing and the line is a drawn guide until you say otherwise. From
    // the third tap on there IS something to solve -- a vertex tapped where the line
    // does not go -- and that is what the quick projection below is for: drop the
    // bodies roughly where you want them, then say they are in a line.
    const vi=pickVertexAt(wx,wy);
    let v = vi>=0 ? constraints[vi] : null;
    if(!v){
      const t=anchorTarget(wx,wy);
      v=makeVertex(null);
      makeVertexOn(v, t ? {id:t.body.id, off:offOf(t.body,t.wp)} : {id:null, off:[wx,wy]}, {join:true});
      constraints.push(v);
    }
    if(!pending){
      // EXTEND starts on a line; NEW starts on a vertex and waits for its partner.
      if(lineExtendMode){
        const ci=pickConstraintOfType('line',wx,wy);
        if(ci<0) return;                       // nothing to extend -- wait for a line
        pending={line:constraints[ci].id, wp:[wx,wy]};
        joinVertexToLine(constraints[ci], v);
        selectConstraint(ci); saveState(); return;
      }
      pending={vertex:v, wp:vertexWorld(v)};
      return;
    }
    if(pending.line!=null){
      const L=lineById(pending.line);
      if(L) joinVertexToLine(L, v);
      saveState(); return;
    }
    // NEW, second tap: the line does not exist until there are two vertices to put it
    // through -- one point has no direction, and a line with one joint is not a line.
    if(v===pending.vertex) return;
    const L=makeLine(); constraints.push(L);
    joinVertexToLine(L, pending.vertex);
    joinVertexToLine(L, v);
    pending={line:L.id, wp:[wx,wy]};
    selectConstraint(constraints.indexOf(L)); saveState();
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
// The pose drag itself is poseDragTo, below the listener.
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

  if(anchorDrag||resizeDrag||panning||bodyPreview||groupDrag||lasso||(drag&&!sim.running)||grab){ hover=null; hoverHandle=null; hoverSnap=null; }
  else updateHover(mouseWorld[0],mouseWorld[1]);
  if(tool==='select') cv.style.cursor =
    (resizeDrag||groupDrag) ? 'grabbing'
    : (hoverHandle && (hoverHandle.kind==='resize' || hoverHandle.kind==='group')) ? 'grab'
    : (selGroup && !sim.running && groupContains(selGroup,mouseWorld[0],mouseWorld[1])) ? 'move'
    : 'default';

  if(lasso){
    // One point per pointermove, thinned to a few screen pixels: the loop only has
    // to be accurate enough to test centres against, and an unthinned trail is
    // thousands of points for one gesture.
    const last=lasso.pts[lasso.pts.length-1];
    const [lx,ly]=w2s(last[0],last[1]);
    if(Math.hypot(px-lx,py-ly)>3) lasso.pts.push([mouseWorld[0],mouseWorld[1]]);
    return;
  }
  if(groupDrag){
    // As with a control point (§13.5's deferred apply), a tap that never crosses
    // the threshold must leave the selection exactly where it was.
    if(movedFar){ groupDragTo(mouseWorld[0],mouseWorld[1]); saveState(); }
    return;
  }
  if(anchorDrag){
    // Only once the gesture has actually moved (§13.5's deferred apply) -- a tap
    // that never crosses movedFar's threshold must leave the control point
    // exactly where it was, so endPointer's toggle (§13.7) is the only thing
    // that happens.
    if(movedFar){ if(anchorDrag.cb) applyCableHandle(anchorDrag,mouseWorld[0],mouseWorld[1]); else applyHandle(anchorDrag, mouseWorld[0], mouseWorld[1]); saveState(); }
    return;
  }
  if(resizeDrag){ applyBodyResize(resizeDrag, mouseWorld[0], mouseWorld[1]); return; }
  if(panning){ cam.x=panning.cx-(e.clientX-panning.sx)/cam.scale; cam.y=panning.cy+(e.clientY-panning.sy)/cam.scale; return; }
  if(bodyPreview){
    if(bodyPreview.shape==='rect'||bodyPreview.shape==='vessel'){ bodyPreview.x1=mouseWorld[0]; bodyPreview.y1=mouseWorld[1]; }
    else { bodyPreview.r=Math.hypot(mouseWorld[0]-bodyPreview.cx,mouseWorld[1]-bodyPreview.cy); }
    return;
  }
  if(drag && !sim.running){ poseDragTo(mouseWorld[0], mouseWorld[1]); saveState(); }
});

// One step of a pose drag: articulate the machine so the grabbed point follows the
// cursor to (wx, wy). Factored out of the pointermove handler above so the posing
// mode has one visible scope -- and so the verification scripts can drive a drag
// without synthesizing pointer events (tools/posable-check.js), the same seam
// runToolClick gives the click paths (§13.5).
function poseDragTo(wx,wy){
  // Posing is the scope a `posable` rod jointed to the dragged body is released in
  // (constraints.js §06.2d): for the length of this call such a rod is a bare rail,
  // holding neither its length nor its welds, and grounding nothing. Which body the
  // gesture is on was named by beginPosing at pointerdown. Everything that reads the
  // rows has to sit inside the scope, refreshFrozen included -- G.static is read
  // below, and a body whose only anchor is a released rod is free to be posed.
  withPosing(()=>{
    refreshFrozen();
    const G=bodies[drag.bi];
    if(G.static){
      // PINNED, so the hand moves it nowhere. A body is frozen only when a line
      // grounds it (constraints.js §06.2b lineGrounds), and that takes two joints
      // both held AND welded between it and fixed ground -- which leaves the body no
      // freedom at all. Moving it anyway would mean re-authoring something nobody
      // asked to change: the bar's own held distance, since its ground end cannot
      // follow. Dragging a body is not how a bar's length is set -- dragging the
      // VERTEX is (§13.3) -- so the drag stops here rather than quietly restretching
      // the mechanism it is holding.
      //
      // The way to pose one is to say so, and there are three: tick the line
      // `posable`, which releases it for exactly this gesture (§06.2d -- which is why
      // `G.static` is read INSIDE the posing scope, so a released line has already
      // stopped grounding by the time we get here), untick a weld, or let a joint
      // slide. Each of them is a statement about the mechanism, which is what the
      // freedom to move it is.
      return;
    }
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
    const [px,py]=saturatingPull(gx,gy,wx,wy,DRAG_CAP_PX);
    const temp={type:'dragpin', a:{id:G.id, off:drag.off}, world:[gx+px,gy+py]};
    projectPositions(8,[temp]);
  });
  // Outside the scope again, so the rod is rigid from here on -- and it holds the
  // pose the drag just reached rather than the one it was authored at. The second
  // refreshFrozen is what closes the scope: whatever a released rod was freezing, it
  // freezes again, at the geometry the drag left. Leaving that to the caller's
  // saveState would work but would make every reader of `static` between here and
  // there -- render, the HUD, the inspector -- see the released world.
  recapturePosable();
  refreshFrozen();
}

// ---- §13.7 · pointerup / cancel / wheel ----
function endPointer(e){
  pointers.delete(e.pointerId);
  if(pinch && pointers.size<2){ pinch=null; pinchCooldown = pointers.size>0; }
  if(pointers.size===0) pinchCooldown=false;
  if(pinchCooldown){ cancelSingle(); return; }

  // A closed loop selects; a lasso that never moved falls through to the tap path
  // below (runToolClick), which toggles one body in or out.
  if(lasso){
    const pts=lasso.pts; lasso=null;
    if(movedFar && e.type==='pointerup'){
      // A loop that caught something hands the bench straight to the select tool:
      // the box it just made is transformed there (§13.5), and having to press 1
      // between drawing a selection and moving it is a step with no meaning. A
      // loop that caught nothing leaves the lasso up, since the next thing the
      // player wants is another loop.
      if(lassoSelect(pts)) setTool('select');
      clickArmed=false; downScreen=null; return;
    }
  }
  if(groupDrag){ groupDrag=null; downScreen=null; return; }

  if(anchorDrag){
    // A tap (no drag) on a rod's or slot's own control point toggles that end
    // between a freely-rotating pin and a rotation-locked weld/prismatic
    // state, instead of relocating it. That flips a property the inspector
    // panel shows as a checkbox (weld/prismatic), so refresh it too -- item 2:
    // editing an interaction from its control points must stay in sync with
    // the panel, the same as editing it from the panel's own checkboxes.
    // A tap on a VERTEX's own handle toggles nothing: what a vertex holds is a list
    // of ticks in its panel, not two states to flip between (§14.2).
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
  drag=null; endPosing(); grab=null; downScreen=null;
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
