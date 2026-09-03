// ============================================================================
//  §05 · GEOMETRY HELPERS
//  Pure helpers with no side effects on the sim: rotation, the body factory,
//  and the world<->screen coordinate maps used by every draw and pick routine.
//    §05.1  rotation & rigid-body point math
//    §05.2  body factory (mass/inertia from radius)
//    §05.2d gas vessels (the fourth coordinate, its inertia, and the gas state)
//    §05.2e convex polygon overlap (body<->vessel contact area, §08.0b's rate law)
//    §05.3  world<->screen transforms
// ============================================================================

// ---- §05.1 · rotation & rigid-body point math ----
const R = (th,x,y)=>[x*Math.cos(th)-y*Math.sin(th), x*Math.sin(th)+y*Math.cos(th)];
function worldPt(b,off){ const [rx,ry]=R(b.th,off[0],off[1]); return [b.x+rx, b.y+ry, rx, ry]; }
// The per-body inverse mass diagonal the Schur assembly (physics.js §08.3) and the
// position projection (§09.1) contract each row's columns against. Four entries, not
// three: the fourth is the inverse generalized mass of a vessel's length coordinate
// (§05.2d), and is 0 for every ordinary body -- which is exactly what makes a stray
// len column on a non-vessel row a no-op rather than a corruption.
// `static` and `lenLock` are already baked into these four by refreshInertia /
// refreshVessel, and they are baked in SEPARATELY: a vessel pinned at its mid-plane
// has a frozen pose and a live length, which is the whole reason this is a diagonal
// of four and not a single flag. (It used to short-circuit all four on `static`,
// which silently froze the length of any fixed vessel -- constraints.js §06.2b.)
function invMdiag(b){ return [b.invM,b.invM,b.invI,b.invMu||0]; }
// Is `b` drawn and hit-tested as an axis-aligned box in its own frame? True for a
// plain rectangle body and for a vessel, which mirrors bore/len into hw/hh
// (refreshVessel) precisely so the box-shaped geometry helpers below serve both.
function rectLike(b){ return b.shape==='rect' || b.shape==='vessel'; }

// ---- §05.2 · body factory (mass/inertia from radius) ----
// `static` starts false on every body and is never set by a caller: it is derived
// from the constraints present, every substep (constraints.js §06.2b refreshFrozen).
function makeBody(x,y,r){
  const mass = Math.PI*r*r;                 // density = 1
  const I = 0.5*mass*r*r;
  return { id:uid++, x, y, th:0, vx:0, vy:0, w:0,
           mass, I, invM:1/mass, invI:1/I,
           shape:'circle', r, static:false, sel:false, kind:'body' };
}
// Rectangle counterpart of makeBody: hw/hh are the half-width/half-height in
// the body's local (unrotated) frame. Mass is the plate's area (density = 1,
// same convention as the disk); I is the standard uniform-rectangle inertia
// about its own centroid, m*(w^2+h^2)/12 with w=2hw, h=2hh.
function makeRectBody(x,y,hw,hh){
  const mass = 4*hw*hh;
  const I = mass*(hw*hw+hh*hh)/3;
  return { id:uid++, x, y, th:0, vx:0, vy:0, w:0,
           mass, I, invM:1/mass, invI:1/I,
           shape:'rect', hw, hh, static:false, sel:false, kind:'body' };
}
// Recompute I/invM/invI from the body's current mass and shape (uniform
// disk: I = 0.5*mass*r^2; uniform rectangle: I = mass*(hw^2+hh^2)/3). mass
// and the shape dimensions are independent fields -- resizing (§13.6
// resizeBody/resizeRectCorner/resizeRectAxes) scales mass with them to
// preserve density; editing mass directly (inspector.js §14.2 setBodyMass)
// leaves the dimensions untouched. Either path ends here so I/invM/invI
// never drift out of sync with whichever changed.
function refreshInertia(b){
  if(b.shape==='vessel'){ refreshVessel(b); return; }
  b.I = b.shape==='rect' ? b.mass*(b.hw*b.hw+b.hh*b.hh)/3 : 0.5*b.mass*b.r*b.r;
  b.invM=b.static?0:1/b.mass; b.invI=b.static?0:1/b.I;
}
// A vessel's `mass` is derived (shell + gas), so a mass edit lands on the shell --
// the part the player actually owns. The gas's own mass is a separate field with
// its own thermodynamic meaning (§05.2d setVesselGasMT).
function setBodyMass(b,m){
  if(b.shape==='vessel'){ b.mShell=Math.max(m-b.gas.mass,1e-9); refreshVessel(b); return; }
  b.mass=m; refreshInertia(b);
}
// ---- §05.2d · gas vessels ----
// A vessel is a body of fixed bore and variable length holding a gas. It carries a
// FOURTH configuration coordinate, `len` (rate `vlen`), on top of the usual
// (x, y, th) -- see VESSEL.md for the derivation this code implements. The essentials:
//
//   * Material coordinates. A point of the vessel is labelled (lat, f) with f in
//     [-1/2, +1/2] the fraction of the way along the axis; its LOCAL position is
//     (lat, f*len), so the label is glued to the material and the mass distribution
//     in (lat, f) never changes with len. Caps are the material planes f = -+1/2;
//     walls are lat = -+bore/2. epLocal/epOffOf below are the only places that
//     conversion lives.
//   * The axis is the body's local +y, so a freshly placed vessel has its caps
//     facing up and down and its bore across -- and bore/len map onto a rectangle's
//     width/height exactly (hw/hh mirrors, refreshVessel), which is what lets the
//     rect-shaped picking, snapping and resize code serve vessels unchanged.
//   * Because the mass distribution is symmetric about the centre, the (x,y), th and
//     len coordinates decouple: the generalized mass of len is the axial second
//     moment mu = integral f^2 dm, which is ALSO the coefficient of len^2 in the
//     moment of inertia. One number governs both. For the uniform-slab distribution
//     used here, mu = mass/12 and I = mass*(bore^2 + len^2)/12 -- the very formula
//     makeRectBody already uses, with hh = len/2.
//   * mu > 0 as long as *any* material has axial extent, and the gas is material --
//     so a pressure difference on weightless caps still gives a finite acceleration.
//     Nothing needs the gas's inertia tracked separately; it is already in mu.
//
// The out-of-plane depth is 1 m, so a planar area reads directly as a volume and a
// bore reads directly as a cap area. This is what makes SI pressures (Pa) and
// energies (J) come out right in a two-dimensional world.
const VESSEL_DEPTH = 1;
// Below this the vessel is treated as bottomed out. It is NOT what arrests a closing
// vessel -- the gas's own potential does that, exactly and unconditionally
// (§08.1b vesselGasStep) -- it is only the lower bracket of that step's root find,
// so a *vacuum* vessel (no gas, hence no divergent pressure to stop it) lands softly
// instead of integrating through zero length into negative geometry.
const VESSEL_MIN_LEN = 1e-6;
// Air, in SI: specific gas constant J/(kg*K) and the diatomic ratio of specific
// heats. Working with the *specific* constant rather than the universal one keeps
// the gas's mass -- which the vessel needs anyway, for mu -- as the primary field,
// with no separate mole count or molar mass to carry.
const GAS_AIR = { Rs:287.05, gamma:1.4 };
// Default shell density. Not a physical claim about any real material: it is the
// density at which a default-sized vessel oscillates against 1 atm at a few hertz
// rather than at the substep rate. A lighter shell is perfectly stable (the gas DOF
// is unconditionally so, §08.1b) -- it just rings faster than the eye resolves.
const VESSEL_DENSITY = 2000;

function vesselVol(v){ return v.bore*v.len*VESSEL_DEPTH; }
// Area of one cap -- the bore times the implicit depth. The force a cap carries is
// this times the pressure difference across it.
function vesselCapArea(v){ return v.bore*VESSEL_DEPTH; }
// The gas is stored as (mass, gamma, Rs, kap) where kap = P*V^gamma is the adiabat
// invariant. Mechanics never changes kap or mass, so P, T and U below are pure
// functions of the current geometry -- which is what makes the gas an ordinary
// potential-energy force element (§08.1b) instead of an incremental dU = -P dV
// bookkeeping loop. Only an explicit heat or mass exchange may touch kap or mass.
function gasP(v){ return v.gas.kap*Math.pow(vesselVol(v), -v.gas.gamma); }
function gasT(v){ const V=vesselVol(v); return v.gas.mass>1e-15 ? gasP(v)*V/(v.gas.mass*v.gas.Rs) : 0; }
function gasU(v){ return gasP(v)*vesselVol(v)/(v.gas.gamma-1); }
// Heat capacity at constant volume of the sealed gas, J/K. U = C*T *exactly*
// (U = P*V/(gamma-1) = mass*Rs*T/(gamma-1)), which is what lets the heat pass of
// §08.0b relax temperatures and read the energy it moved straight off them.
function gasC(v){ return v.gas.mass*v.gas.Rs/(v.gas.gamma-1); }
// Specific enthalpy, J/kg -- cp*T. What a kilogram of this gas carries with it when
// it crosses a port into another vessel: its internal energy plus the flow work the
// gas behind it does pushing it out (VESSEL.md §V.10).
function gasEnthalpy(v){ return v.gas.gamma*v.gas.Rs*gasT(v)/(v.gas.gamma-1); }
// Inverse of gasU: set the adiabat invariant so the gas holds internal energy U at
// the current volume. The exchange pass works in energy, not temperature, on the
// receiving side of a mass transfer -- what arrives is a quantity of energy, and the
// temperature it implies depends on the mass that arrived with it.
function setVesselGasU(v,U){
  const V=vesselVol(v);
  v.gas.kap = Math.max(U,0)*(v.gas.gamma-1)*Math.pow(V, v.gas.gamma-1);
}
// Atmospheric potential: the work the background does at the caps' outer faces,
// held as a potential (P_bg * V) rather than accumulated as flow work. Being a state
// function is the whole point -- an accumulated term has to be told which substeps
// may credit it, and a potential does not.
function vesselAtmPE(v){ return sim.bg.P*vesselVol(v); }
// The vessel's total length-coordinate potential, evaluated at an ARBITRARY len --
// the discrete-gradient step (§08.1b) needs it at a trial length, not just the
// current one. Q_len = -dUpot/dlen = (P - P_bg)*capArea, the spec's force law,
// recovered as a gradient rather than asserted.
function vesselUpotAt(v, len){
  const V=v.bore*Math.max(len,VESSEL_MIN_LEN)*VESSEL_DEPTH;
  return v.gas.kap*Math.pow(V,1-v.gas.gamma)/(v.gas.gamma-1) + sim.bg.P*V;
}
function vesselUpot(v){ return vesselUpotAt(v, v.len); }

// Refresh every derived quantity from (mShell, gas.mass, bore, len). Called once per
// substep before islands and the energy baseline are snapshotted (physics.js §08.0),
// and immediately after any edit -- so I, mu and the inverse masses can never drift
// out of step with the geometry or the gas.
function refreshVessel(v){
  v.mass  = v.mShell + v.gas.mass;
  v.mu    = v.mass/12;                       // integral f^2 dm, uniform slab
  v.Alat  = v.mass*v.bore*v.bore/12;         // integral lat^2 dm
  v.I     = v.Alat + v.mu*v.len*v.len;       // = mass*(bore^2 + len^2)/12
  v.invM  = v.static?0:1/v.mass;
  v.invI  = v.static?0:1/v.I;
  // NOT `static || lenLock`. A fixed pose says nothing about the length: a vessel
  // welded to the world at its mid-plane has three coordinates pinned and a fourth
  // entirely free, which is exactly the heat pair's working vessel. Only a length
  // lock -- a strut inside the vessel (constraints.js §06.2b) -- freezes this one.
  v.invMu = v.lenLock?0:1/v.mu;
  v.hw    = v.bore/2; v.hh = v.len/2;        // mirrors for the rect-shaped helpers
}
function refreshVessels(){ for(const b of bodies) if(b.shape==='vessel') refreshVessel(b); }

// Set the gas from a pressure and temperature at the current volume, deriving the
// mass that implies. Used at placement (both at ambient, so a fresh vessel starts
// balanced) and by the inspector's pressure field.
function setVesselGasPT(v,P,T){
  const V=vesselVol(v);
  v.gas.mass = (T>1e-9)? P*V/(v.gas.Rs*T) : 0;
  v.gas.kap  = P*Math.pow(V, v.gas.gamma);
  refreshVessel(v);
}
// Set the gas from a mass and temperature at the current volume, deriving the
// pressure that implies -- the path a geometry edit takes (a sealed vessel keeps its
// gas and its temperature; its pressure is whatever the new volume makes it) and the
// path the inspector's mass and temperature fields take.
function setVesselGasMT(v,m,T){
  const V=vesselVol(v);
  v.gas.mass = Math.max(m,0);
  v.gas.kap  = (v.gas.mass*v.gas.Rs*T/V)*Math.pow(V, v.gas.gamma);
  refreshVessel(v);
}
// Two opposite corners, exactly as makeRectBody -- the placement tool's own model.
// bore is the width, len the height, so the caps face up and down at th = 0.
function makeVessel(x,y,bore,len){
  const v = { id:uid++, kind:'body', shape:'vessel',
              x, y, th:0, len,
              vx:0, vy:0, w:0, vlen:0,
              bore, mShell:VESSEL_DENSITY*bore*len*VESSEL_DEPTH,
              gas:{ mass:0, gamma:GAS_AIR.gamma, Rs:GAS_AIR.Rs, kap:0 },
              lenLock:false, static:false, sel:false };
  setVesselGasPT(v, sim.bg.P, sim.bg.T);     // starts balanced against the atmosphere
  return v;
}
// Change a vessel's geometry, holding the gas sealed: mass and temperature carry
// over, so the pressure is whatever the new volume implies. mShell scales with the
// footprint to preserve density, mirroring how resizeBody/applyRectResize treat an
// ordinary body's mass.
function resizeVessel(v, newBore, newLen){
  newBore=Math.max(0.02,newBore); newLen=Math.max(VESSEL_MIN_LEN,newLen);
  const T=gasT(v) || sim.bg.T;
  const areaRatio=(newBore*newLen)/(v.bore*v.len);
  v.mShell*=areaRatio;
  v.bore=newBore; v.len=newLen;
  setVesselGasMT(v, v.gas.mass, T);
}

// ---- §05.2c · endpoint offsets (material, on a vessel) ----
// Every constraint, spring and cable endpoint is an {id, off} pair. For an ordinary
// body `off` is a plain local-frame point. For a vessel it is a MATERIAL label
// (lat, f) -- so an anchor stays at the same fraction along the axis as the vessel
// breathes, rather than sliding relative to the material. These three functions are
// the only place that distinction lives; everything downstream goes through them.
//
// This is what makes cap/wall/interior attachments one uniform case instead of three:
// the len column of the endpoint's Jacobian is proportional to f (constraints.js
// §06.1 epFrame), so pinning a cap (f = -+1/2) restrains length fully, pinning the
// mid-wall (f = 0) not at all, and anything between in exact proportion.
function epLocal(b, off){ return b.shape==='vessel' ? [off[0], off[1]*b.len] : off; }
function epWorldPt(b, off){ return worldPt(b, epLocal(b,off)); }
// Inverse of epLocal: a world point -> the endpoint offset that names it.
function epOffOf(b, wx, wy){
  const [lx,ly]=R(-b.th, wx-b.x, wy-b.y);
  return b.shape==='vessel' ? [lx, ly/(b.len||1e-9)] : [lx,ly];
}
// The vessel's axis direction in world space (its local +y), and the world-frame
// offset of a material point from the vessel's centre. Both feed the endpoint
// Jacobian: d(world point)/d(len) = f * axis.
function vesselAxis(v){ return [-Math.sin(v.th), Math.cos(v.th)]; }

// ---- §05.2e · convex polygon overlap (body<->vessel contact area) ----
// The contact area between a solid body's outline and a vessel's rectangle is the
// rate-law input for every heat/mass interaction (physics.js §08.0b) -- and the ONLY
// place in the engine where two bodies' outlines are compared at all. It is not
// collision: nothing here produces a force, and the sandbox's "nothing interacts
// unless the player says it does" invariant is untouched, because the test only ever
// runs on the explicitly-paired objects an interaction names.
//
// A body's outline as a world-space convex polygon: a rectangle's (or a vessel's --
// rectLike covers both) four corners, or a circle approximated by a 20-gon, which is
// under 2% low on area and cheap to clip.
function bodyPolygon(b){
  if(rectLike(b)){
    return [[-b.hw,-b.hh],[b.hw,-b.hh],[b.hw,b.hh],[-b.hw,b.hh]]
      .map(o=>{ const [wx,wy]=worldPt(b,o); return [wx,wy]; });
  }
  const N=20, pts=[];
  for(let i=0;i<N;i++){ const a=i/N*Math.PI*2; pts.push([b.x+b.r*Math.cos(a), b.y+b.r*Math.sin(a)]); }
  return pts;
}
// Signed shoelace area (positive iff `poly` is wound CCW); |.| is the plain area.
function polySignedArea(poly){ let s=0; for(let i=0;i<poly.length;i++){
  const [x1,y1]=poly[i], [x2,y2]=poly[(i+1)%poly.length]; s+=x1*y2-x2*y1; } return s*0.5; }
function polyArea(poly){ return Math.abs(polySignedArea(poly)); }
// Point where segment p1->p2 crosses line a->b (used only inside clipPoly, where the
// crossing is already known to exist).
function segIntersect(p1,p2,a,b){
  const d1x=p2[0]-p1[0], d1y=p2[1]-p1[1], d2x=b[0]-a[0], d2y=b[1]-a[1];
  const denom=d1x*d2y-d1y*d2x;
  const t = Math.abs(denom)<1e-12 ? 0 : ((a[0]-p1[0])*d2y-(a[1]-p1[1])*d2x)/denom;
  return [p1[0]+t*d1x, p1[1]+t*d1y];
}
// Sutherland-Hodgman: clip `subject` (any winding, possibly empty) against convex
// polygon `clip` (any winding -- normalized to CCW here so the inside-test sign is
// fixed regardless of how the caller wound it).
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
      const curIn  = ex*(cur[1]-A[1])-ey*(cur[0]-A[0]) >= 0;
      const prevIn = ex*(prev[1]-A[1])-ey*(prev[0]-A[0]) >= 0;
      if(curIn){ if(!prevIn) output.push(segIntersect(prev,cur,A,B)); output.push(cur); }
      else if(prevIn){ output.push(segIntersect(prev,cur,A,B)); }
    }
  }
  return output;
}
// The overlap area, in m^2 (times the implicit 1 m depth, so it reads directly as
// the wall area the exchange crosses).
function contactArea(b,v){ return polyArea(clipPoly(bodyPolygon(b), bodyPolygon(v))); }

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
function bodyExtentR(b){ return rectLike(b) ? Math.hypot(b.hw,b.hh) : b.r; }
// The local-frame point used to draw a body's "theta=0" rim mark (rotational
// spring control points, constraints.js §06.6) -- a circle's rim along its
// own +x, or a rectangle's right-edge midpoint along the same axis.
function bodyRimLocal(b){ return rectLike(b) ? [b.hw,0] : [b.r,0]; }
// Is world point (wx,wy) inside b's outline -- the shape-generic core of
// pickBody/pickBodyExcept (tools.js §13.2).
function bodyContains(b,wx,wy){
  if(rectLike(b)){
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
  if(rectLike(b)){
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
  if(rectLike(b)){ const [ex,ey]=bodyEdgePoint(b,wx,wy); return Math.hypot(wx-ex,wy-ey); }
  return Math.abs(Math.hypot(wx-b.x,wy-b.y)-b.r);
}

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
