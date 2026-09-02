// ============================================================================
//  §05 · GEOMETRY HELPERS
//  Pure helpers with no side effects on the sim: rotation, the body factory,
//  and the world<->screen coordinate maps used by every draw and pick routine.
//    §05.1  rotation & rigid-body point math
//    §05.2  body factory (mass/inertia from radius)
//    §05.2c convex polygon overlap (body<->gas contact area for §08.5)
//    §05.3  world<->screen transforms
// ============================================================================

// ---- §05.1 · rotation & rigid-body point math ----
const R = (th,x,y)=>[x*Math.cos(th)-y*Math.sin(th), x*Math.sin(th)+y*Math.cos(th)];
function worldPt(b,off){ const [rx,ry]=R(b.th,off[0],off[1]); return [b.x+rx, b.y+ry, rx, ry]; }
function invMdiag(b){ return b.static?[0,0,0]:[b.invM,b.invM,b.invI]; }

// ---- §05.2 · body factory (mass/inertia from radius) ----
function makeBody(x,y,r,isStatic){
  const mass = Math.PI*r*r;                 // density = 1
  const I = 0.5*mass*r*r;
  return { id:uid++, x, y, th:0, vx:0, vy:0, w:0,
           mass, I, invM:isStatic?0:1/mass, invI:isStatic?0:1/I,
           shape:'circle', r, static:!!isStatic, sel:false, kind:'body' };
}
// Rectangle counterpart of makeBody: hw/hh are the half-width/half-height in
// the body's local (unrotated) frame. Mass is the plate's area (density = 1,
// same convention as the disk); I is the standard uniform-rectangle inertia
// about its own centroid, m*(w^2+h^2)/12 with w=2hw, h=2hh.
function makeRectBody(x,y,hw,hh,isStatic){
  const mass = 4*hw*hh;
  const I = mass*(hw*hw+hh*hh)/3;
  return { id:uid++, x, y, th:0, vx:0, vy:0, w:0,
           mass, I, invM:isStatic?0:1/mass, invI:isStatic?0:1/I,
           shape:'rect', hw, hh, static:!!isStatic, sel:false, kind:'body' };
}
// Recompute I/invM/invI from the body's current mass and shape (uniform
// disk: I = 0.5*mass*r^2; uniform rectangle: I = mass*(hw^2+hh^2)/3). mass
// and the shape dimensions are independent fields -- resizing (§13.6
// resizeBody/resizeRectCorner/resizeRectAxes) scales mass with them to
// preserve density; editing mass directly (inspector.js §14.2 setBodyMass)
// leaves the dimensions untouched. Either path ends here so I/invM/invI
// never drift out of sync with whichever changed.
function refreshInertia(b){
  b.I = b.shape==='rect' ? b.mass*(b.hw*b.hw+b.hh*b.hh)/3 : 0.5*b.mass*b.r*b.r;
  b.invM=b.static?0:1/b.mass; b.invI=b.static?0:1/b.I;
}
function setBodyMass(b,m){ b.mass=m; refreshInertia(b); }
const EFF_MASS_FLOOR = 1e-6;
// A gas vessel with a real moving piston (DEVELOPMENT.md §6.1's redesign)
// is represented, dynamically, by exactly ONE real rigid body -- `v.com`,
// the gas's own center of mass, mass forced to the gas's full `v.mass` --
// plus one genuine extra scalar coordinate, `v.sepRate` (rate of change of
// `v.sep`, the axial separation). This is the *exact* decomposition of a
// uniformly-distributed gas column's kinetic energy: writing v1/v2 for the
// head's/cap's own velocity, `mass*(v1^2+v1*v2+v2^2)/6` is a genuinely
// *coupled* quadratic form (a cross term between the two ends) that no
// per-body-independent mass split can represent (verified by hand: no
// fixed split matches both the head-rigidly-fixed and both-ends-free
// cases at once, since the cap's effective inertia genuinely depends on
// whether the head is free or constrained -- an earlier version of this
// file tried `mass/2` each or `mass/3` on the cap alone and got provably
// wrong accelerations). Substituting center-of-mass/relative coordinates
// `vc=(v1+v2)/2`, `vr=v2-v1` (`vr` IS sepRate) removes the cross term
// completely: `KE = (1/2)*mass*vc^2 + (1/2)*(mass/12)*vr^2` -- the COM
// translates as a rigid lump of the gas's own total mass; the stretching
// motion has its own separate, smaller, fully decoupled inertia
// (`mass/12`). `v.piston` is kept (same `{id,off}` shape as ever) but now
// names a *static*, kinematically-slaved marker body -- never touched by
// the solver, just repositioned each substep (syncVesselMarkers below) --
// so every consumer that only ever needed "some body with the right
// polygon at the right place" (heat/flow attachment, contact-area math,
// picking) keeps working unmodified.
//
// §05.2d below extends the constraint solver's own coordinate space with
// this scalar (indices N..N+V-1, one per vessel-with-a-piston, appended
// after the N real bodies) so ordinary constraint rows (the vessel's own
// "mount" to its head frame, ordinary rods/pins/springs attaching to an
// interior point) can reference it exactly like a body's own DOF.
function syncVesselComMass(v){
  if(!v.piston) return;
  const com = bodies[bodyIndex(v.com.id)]; if(!com) return;
  setBodyMass(com, Math.max(v.mass, EFF_MASS_FLOOR));
}
// Keep the static, kinematically-slaved piston marker at the true cap
// point (gasFrame's pax,pay, itself built from v.sep) and sharing the
// vessel's own orientation (com.th) -- called once per substep alongside
// syncVesselComMass, and after any edit that moves com or changes sep.
function syncVesselMarkers(v){
  if(!v.piston) return;
  const marker = bodies[bodyIndex(v.piston.id)]; if(!marker) return;
  const com = bodies[bodyIndex(v.com.id)]; if(!com) return;
  const f = gasFrame(v);
  marker.x = f.pax; marker.y = f.pay; marker.th = com.th;
}
// Reposition a vessel's length. For a movable piston this means moving
// `com` directly to the new midpoint (head's own frame, hx/hy/dW, doesn't
// depend on sep, so this lands exactly on the new length with zero
// residual constraint error -- an authoritative edit, not something a
// least-squares projectPositions pass could partially undo by shifting
// `sep` back); for a fixed vessel (no piston body) it's a bare constant.
// Used by the inspector's radio-lock recompute (inspector.js) whenever
// `length` is the field being derived or edited.
function setVesselLength(v, newLen){
  newLen = Math.max(newLen, GAS_MIN_X);
  if(v.piston){
    const com = bodies[bodyIndex(v.com.id)]; if(!com) return;
    const f = gasFrame(v);
    com.x = f.hx + f.dW[0]*newLen*0.5;
    com.y = f.hy + f.dW[1]*newLen*0.5;
    v.sep = newLen;
    syncVesselMarkers(v);
    projectPositions(8);
  } else {
    v.len = newLen;
  }
}

// ---- §05.2d · extended coordinate space (bodies ++ per-vessel sepRate) ----
// Every constraint row is {cols:[[idx,jx,jy,jw],...], C}. Indices 0..N-1
// are real bodies as always; a vessel-with-a-piston's sepRate coordinate
// lives at one extra index N..N+V-1, appended after them -- its column is
// always [idx, coeff, 0, 0] (a plain scalar: no lateral or rotational
// component of its own). vesselCoordList() builds this mapping fresh
// (cheap: one pass over gases) wherever a caller needs to translate a gas
// to its flat index; the four coord* helpers dispatch once on idx<N so
// EVERY existing constraint between plain bodies (rod, pin, slot, belt,
// knife, cvt, cable, spring) goes through the *exact* code that was there
// before this redesign -- only rows that actually reference a vessel
// coordinate exercise the new branch. Used by physics.js §08.3's Schur
// assembly/impulse-apply and projection.js's projectPositions in place of
// touching bodies[c[0]] directly.
function vesselCoordList(){
  const N=bodies.length; const list=[]; const idxOf=new Map();
  for(const g of gases) if(g.piston){ idxOf.set(g.id, N+list.length); list.push(g); }
  return {N, list, idxOf};
}
function coordInvM(idx,N,list){
  if(idx<N) return invMdiag(bodies[idx]);
  return [12/Math.max(list[idx-N].mass,EFF_MASS_FLOOR), 0, 0];
}
function coordGetV(idx,N,list){
  if(idx<N){ const b=bodies[idx]; return [b.vx,b.vy,b.w]; }
  return [list[idx-N].sepRate,0,0];
}
function coordApplyImpulse(idx,N,list,c1,c2,c3,lambda){
  if(idx<N){ const b=bodies[idx]; if(b.static) return;
    b.vx+=b.invM*c1*lambda; b.vy+=b.invM*c2*lambda; b.w+=b.invI*c3*lambda; return; }
  const g=list[idx-N]; g.sepRate += (12/Math.max(g.mass,EFF_MASS_FLOOR))*c1*lambda;
}
function coordApplyPos(idx,N,list,c1,c2,c3,amount){
  if(idx<N){ const b=bodies[idx]; if(b.static) return;
    b.x+=b.invM*c1*amount; b.y+=b.invM*c2*amount; b.th+=b.invI*c3*amount; return; }
  const g=list[idx-N]; g.sep += (12/Math.max(g.mass,EFF_MASS_FLOOR))*c1*amount;
}

// ---- §05.2b · shape-generic body geometry ----
// A handful of pure functions any picking/snapping/rendering code can call
// without knowing whether `b` is a circle or a rectangle. Everything else in
// the file (rows, forces, inertia) only ever needs mass/I/invM/invI, which
// are already shape-agnostic scalars -- these four are the ones that reach
// down into the actual outline.
//
// Roughly how big the body reads on screen -- used only for the painter sort
// (render.js §11.1) and the rotational-spring spiral's decorative radius
// (constraints.js §06.6), never for physics. A circle's own radius; a
// rectangle's half-diagonal (its farthest point from centre).
function bodyExtentR(b){ return b.shape==='rect' ? Math.hypot(b.hw,b.hh) : b.r; }
// The local-frame point used to draw a body's "theta=0" rim mark (rotational
// spring control points, constraints.js §06.6) -- a circle's rim along its
// own +x, or a rectangle's right-edge midpoint along the same axis.
function bodyRimLocal(b){ return b.shape==='rect' ? [b.hw,0] : [b.r,0]; }
// Is world point (wx,wy) inside b's outline -- the shape-generic core of
// pickBody/pickBodyExcept (tools.js §13.2).
function bodyContains(b,wx,wy){
  if(b.shape==='rect'){
    const [lx,ly]=R(-b.th, wx-b.x, wy-b.y);
    return Math.abs(lx)<=b.hw && Math.abs(ly)<=b.hh;
  }
  const dx=wx-b.x, dy=wy-b.y; return dx*dx+dy*dy<=b.r*b.r;
}
// Nearest point on b's boundary to a world point, and the distance to it --
// the shape-generic core of snapAnchor's edge-snap and (for a circle) the
// resize-rim hit test (tools.js §13.2). For a rectangle: transform to the
// local frame; a point already inside clamps to whichever of the four edges
// is nearest, a point outside clamps to the box (which for a point outside a
// convex box already lands exactly on its boundary).
function bodyEdgePoint(b,wx,wy){
  if(b.shape==='rect'){
    const [lx,ly]=R(-b.th, wx-b.x, wy-b.y);
    let ex,ey;
    if(Math.abs(lx)<=b.hw && Math.abs(ly)<=b.hh){
      const dx=b.hw-Math.abs(lx), dy=b.hh-Math.abs(ly);
      if(dx<dy){ ex=Math.sign(lx||1)*b.hw; ey=ly; } else { ex=lx; ey=Math.sign(ly||1)*b.hh; }
    } else {
      ex=Math.max(-b.hw,Math.min(b.hw,lx)); ey=Math.max(-b.hh,Math.min(b.hh,ly));
    }
    const [wx2,wy2]=R(b.th, ex, ey);
    return [b.x+wx2, b.y+wy2];
  }
  const dx=wx-b.x, dy=wy-b.y, d=Math.hypot(dx,dy)||1e-9;
  return [b.x+dx/d*b.r, b.y+dy/d*b.r];
}
function bodyEdgeDist(b,wx,wy){
  if(b.shape==='rect'){ const [ex,ey]=bodyEdgePoint(b,wx,wy); return Math.hypot(wx-ex,wy-ey); }
  return Math.abs(Math.hypot(wx-b.x,wy-b.y)-b.r);
}

// ---- §05.2c · convex polygon overlap (body<->gas contact area) ----
// A body's outline as a world-space convex polygon: a rectangle's four
// corners, or a circle approximated by a 20-gon (accurate to <1% area error
// and cheap to clip) -- used only for heat/flow interaction contact area
// (physics.js §08.5), never for physics.js's own collision (there is none by
// design, spec §10) or for the gas force/volume (constraints.js §06.2 keeps
// its exact 1-D axial projection). Both this and gasPolygon below return
// points wound consistently CCW in world (x right, y up) coordinates.
function bodyPolygon(b){
  if(b.shape==='rect'){
    return [[-b.hw,-b.hh],[b.hw,-b.hh],[b.hw,b.hh],[-b.hw,b.hh]].map(([lx,ly])=>{
      const [wx,wy]=worldPt(b,[lx,ly]); return [wx,wy]; });
  }
  const N=20, pts=[];
  for(let i=0;i<N;i++){ const a=i/N*Math.PI*2; pts.push([b.x+b.r*Math.cos(a), b.y+b.r*Math.sin(a)]); }
  return pts;
}
// A gas's rectangular volume as a world-space polygon -- the same four
// corners drawGas already computes (render.js §11.4), reused here for area
// math instead of just paint.
function gasPolygon(g){
  const f=gasFrame(g); const nrm=[-f.dW[1],f.dW[0]]; const hw=g.bore*0.5;
  const H1=[f.hx+nrm[0]*hw, f.hy+nrm[1]*hw], H2=[f.hx-nrm[0]*hw, f.hy-nrm[1]*hw];
  const P1=[f.pax+nrm[0]*hw, f.pay+nrm[1]*hw], P2=[f.pax-nrm[0]*hw, f.pay-nrm[1]*hw];
  return [H1,P1,P2,H2];
}
// Signed shoelace area (positive iff `poly` is wound CCW); |.| is the plain area.
function polySignedArea(poly){ let s=0; for(let i=0;i<poly.length;i++){
  const [x1,y1]=poly[i], [x2,y2]=poly[(i+1)%poly.length]; s+=x1*y2-x2*y1; } return s*0.5; }
function polyArea(poly){ return Math.abs(polySignedArea(poly)); }
// Point where segment p1->p2 crosses line a->b (used only inside clipPoly,
// where the crossing is already known to exist).
function segIntersect(p1,p2,a,b){
  const d1x=p2[0]-p1[0], d1y=p2[1]-p1[1], d2x=b[0]-a[0], d2y=b[1]-a[1];
  const denom=d1x*d2y-d1y*d2x;
  const t = Math.abs(denom)<1e-12 ? 0 : ((a[0]-p1[0])*d2y-(a[1]-p1[1])*d2x)/denom;
  return [p1[0]+t*d1x, p1[1]+t*d1y];
}
// Sutherland-Hodgman: clip `subject` (any winding, possibly empty) against
// convex polygon `clip` (any winding -- normalized to CCW here so the
// inside-test sign is fixed regardless of how the caller wound it).
function clipPoly(subject, clipIn){
  if(subject.length===0) return [];
  const clip = polySignedArea(clipIn)>=0 ? clipIn : [...clipIn].reverse();
  let output=subject;
  for(let i=0;i<clip.length && output.length;i++){
    const A=clip[i], B=clip[(i+1)%clip.length];
    const ex=B[0]-A[0], ey=B[1]-A[1];
    const input=output; output=[];
    for(let j=0;j<input.length;j++){
      const cur=input[j], prev=input[(j-1+input.length)%input.length];
      const curIn = ex*(cur[1]-A[1])-ey*(cur[0]-A[0]) >= 0;
      const prevIn = ex*(prev[1]-A[1])-ey*(prev[0]-A[0]) >= 0;
      if(curIn){ if(!prevIn) output.push(segIntersect(prev,cur,A,B)); output.push(cur); }
      else if(prevIn){ output.push(segIntersect(prev,cur,A,B)); }
    }
  }
  return output;
}
// The contact area between a solid body and a gas's rectangular volume --
// the rate-law input for every heat/flow interaction (physics.js §08.5).
function bodyGasOverlapArea(b,g){ return polyArea(clipPoly(bodyPolygon(b), gasPolygon(g))); }

// ---- §05.3 · world<->screen transforms ----
function W(){ return cv.clientWidth; }
function H(){ return cv.clientHeight; }
function w2s(wx,wy){ return [ W()/2 + (wx-cam.x)*cam.scale, H()/2 - (wy-cam.y)*cam.scale ]; }
function s2w(sx,sy){ return [ cam.x + (sx-W()/2)/cam.scale, cam.y - (sy-H()/2)/cam.scale ]; }

// ---- §05.4 · saturating drag pull ----
// Screen-space-capped offset from a world point toward a world target, used
// everywhere a dragged body is pulled toward the cursor (tools.js §13.6 pose
// drag, physics.js §08.1 play-mode grab spring). Grows ~1:1 with the on-screen
// distance for a small, precise tug -- so an unconstrained body still tracks
// the cursor exactly -- but asymptotes to capPx screen pixels' worth of world
// distance however far the cursor is dragged, instead of growing without
// bound. That keeps a constrained body (a rod welded to the background, a
// slider on a fixed rail) from being yanked toward an ever more distant,
// unreachable goal when the drag has a component the joint can't satisfy --
// the joint's own rows stay near-exactly solved instead of being outvoted by
// a runaway pull.
const DRAG_CAP_PX = 80;
function saturatingPull(fromWx, fromWy, toWx, toWy, capPx){
  const dx=toWx-fromWx, dy=toWy-fromWy, d=Math.hypot(dx,dy);
  if(d<1e-9) return [0,0];
  const capW = capPx/cam.scale;
  const mag = capW*(1-Math.exp(-d/capW));
  const s = mag/d;
  return [dx*s, dy*s];
}
