// ============================================================================
//  §06 · CONSTRAINT ROWS
//  The heart of the engine. Each constraint is turned into one or more rows of
//  the velocity-linear (Pfaffian) form  J·v = -bias  (spec §3.3). A row is
//    { cols:[[bodyIdx, jx, jy, jw, jlen?], ...], C, nh? }
//  jlen is the column of a vessel's fourth (length) coordinate -- absent, meaning
//  zero, on every ordinary body. Rows never build it by hand: it comes out of
//  epFrame's velCols closure (§06.1), which is why every branch of rowsFor below
//  works on a vessel endpoint unchanged.
//  where C is the raw position error (the value to drive to zero) and nh flags
//  a nonholonomic row (velocity-only, no position invariant -- excluded from the
//  §09 position projection). The §08 solver scales C by beta/h (Baumgarte);
//  the §09 projection uses C directly. Same rows serve both.
//    §06.1  bodyIndex, epWorld, twoPointFrame, endpointAngleLockRow, and the
//           rod/slot constructors and endpoint-lock toggles built on them
//    §06.2  the remaining constraint makers (pin, belt, cvt, knife, cable)
//    §06.2b derived freezing (rodGrounds, rodLocksLength, refreshFrozen)
//    §06.2c extra control points (conPoints, makeConPoint, conEndpoints) -- the
//           third and further ends a pin/rod/slot/rack may carry
//    §06.2d posable rods (withPosing, rodReleased, recapturePosable) -- the
//           pose-time release that turns a rod into a rail while it is dragged
//    §06.3  cableFrame (tetherball tangent geometry for the unilateral cable)
//    §06.4  (retired -- see §06.1)
//    §06.5  rowsFor    (the dispatch: one branch per constraint type)
//    §06.6  spring / rotSpring frames (Hookean force elements, §08.1) -- these
//           are force elements, not constraint rows: they never appear in
//           rowsFor or the solve of §07/§08.3, only in the applied-force pass
//           of §08.1.
// ============================================================================

// ---- §06.1 · bodyIndex ----
function bodyIndex(id){ return bodies.findIndex(b=>b.id===id); }

// Resolve an endpoint {id, off} to its world point. id===null means the
// endpoint is fixed to the background (world-anchored) rather than riding a
// body -- off then holds the world coordinate directly, mirroring the
// null-id convention already used by cable tethers. Shared by rod and slot,
// whose endpoints are both {id, off} pairs.
function epWorld(ep){
  if(ep.id==null) return [ep.off[0], ep.off[1], 0, 0];
  const b=bodies[bodyIndex(ep.id)];
  return worldPt(b, epLocal(b, ep.off));       // material offset on a vessel, §05.2c
}
// The two-endpoint geometry rod and slot both build their rows from: each
// endpoint resolved (body or background), the segment A->B, its length, and
// its perpendicular. phi is that segment's live world angle -- the reference
// a weld/prismatic lock's rest angle is measured against.
//
// phi is unwrapped against con._phiRef (the previous call's phi, persisted on
// the constraint -- same trick as cableFrame's spoolAngleRef) rather than used
// as raw atan2(dy,dx). Raw atan2 has a branch cut at phi=±pi: a rod/slot that
// swings slowly through pointing along -x has dy cross zero with dx<0, and
// the *physical* angle change that step is tiny, but the raw atan2 value
// itself jumps by a full 2pi. A welded/prismatic end's angle-lock row
// (endpointAngleLockRow) measures C = thHere-phi-restAng, and thHere (a
// body's th) is never wrapped -- so that 2pi jump in phi shows up whole in C,
// and the solver's Baumgarte term (kb*C, physics.js §08.3) turns it into a
// spurious multi-turn correction in a single step. Unwrapping phi here keeps
// it continuous through that crossing, so C stays near zero throughout.
// epA/epB are the endpoints resolved through epFrame (§06.1): every row built from
// this frame projects through their velCols/angCols closures rather than assembling
// columns by hand, so a vessel endpoint's length column comes along automatically
// and the rod weld / slot prismatic locks need no vessel-specific algebra of their
// own. The raw wax/rax/... fields remain for rendering and reactionOf (§09.3).
function twoPointFrame(con){
  const hasA=con.a.id!=null, hasB=con.b.id!=null;
  const A = hasA ? bodies[bodyIndex(con.a.id)] : null;
  const B = hasB ? bodies[bodyIndex(con.b.id)] : null;
  const epA=epFrame(con.a), epB=epFrame(con.b);
  const [wax,way,rax,ray] = hasA ? worldPt(A,epLocal(A,con.a.off)) : [con.a.off[0],con.a.off[1],0,0];
  const [wbx,wby,rbx,rby] = hasB ? worldPt(B,epLocal(B,con.b.off)) : [con.b.off[0],con.b.off[1],0,0];
  const ia = hasA?bodyIndex(con.a.id):-1, ib = hasB?bodyIndex(con.b.id):-1;
  const dx=wax-wbx, dy=way-wby, L=Math.hypot(dx,dy)||1e-9;
  const ux=dx/L, uy=dy/L, nx=-uy, ny=ux;
  const phiRaw=Math.atan2(dy,dx);
  let phi=phiRaw;
  if(con._phiRef!=null){
    let da=phiRaw-con._phiRef;
    while(da> Math.PI) da-=Math.PI*2;
    while(da<-Math.PI) da+=Math.PI*2;
    phi=con._phiRef+da;
  }
  con._phiRef=phi;
  return {hasA,hasB,A,B,ia,ib,epA,epB,wax,way,rax,ray,wbx,wby,rbx,rby,ux,uy,nx,ny,L,phi};
}
// One row locking `which` end's frame angle -- a body's theta, or 0 for a
// background end -- to the live direction phi of the segment from B to A.
// Shared by rod's weld and slot's prismatic lock: both are the same
// operation (pin an endpoint's rotation to the line joining the two
// endpoints), just attached to different base constraints.
// The row measures  d/dt(theta_here - phi), with phi the A->B segment's world angle.
// Since dphi/dt = n.(vA - vB)/L, that is the endpoint's own angular-velocity column
// minus (1/L) times the two endpoints' velocity columns along the segment normal --
// which is exactly what the closures below build. This is the same row the previous
// hand-assembled version produced for two plain bodies, and the correct one for a
// vessel endpoint, whose velCols carries the extra length column.
// `here` is the endpoint whose rotation is being locked -- epA or epB for the base
// pair, or an extra control point's own frame (§06.2c), which is why this takes an
// endpoint rather than the 'A'/'B' selector its two callers used to pass.
function pointAngleLockRow(f, here, restAng){
  const {epA,epB,nx,ny,L,phi} = f;
  const k = -1/L;
  const cols = mergeCols([
    epA.velCols(k*nx, k*ny),
    epB.velCols(-k*nx, -k*ny),
    here.angCols()
  ]);
  return { cols, C: here.th-phi-restAng };
}
function endpointAngleLockRow(which, f, restAng){
  return pointAngleLockRow(f, which==='A' ? f.epA : f.epB, restAng);
}
// Capture (or recapture) endpoint `which`'s rest angle against the live A->B
// direction. Called whenever a per-endpoint lock (rod weld, slot prismatic)
// turns on, so toggling never snaps geometry.
//
// phi is the RAW atan2, not twoPointFrame's unwrapped one, and that is deliberate:
// a rest angle persists (it is in the scene file) while `_phiRef`, the unwrapping
// anchor, is transient scratch that restoreState clears (§16.1). A capture taken
// against an unwrapped phi would agree with the rows now and disagree by a whole
// turn after the next Reset re-seeded phi from raw. Captured raw, it agrees with
// both -- as long as the winding the rows have accumulated is cleared alongside it,
// which is what recaptureRodPose does (§06.2b).
function captureRestAngle(con, which){
  const [wax,way]=epWorld(con.a), [wbx,wby]=epWorld(con.b);
  const phi=Math.atan2(way-wby,wax-wbx);
  const ep = which==='A'?con.a:con.b;
  const th = ep.id!=null ? bodies[bodyIndex(ep.id)].th : 0;
  if(which==='A') con.restAngA=th-phi; else con.restAngB=th-phi;
}

// Build a rod constraint between two endpoints, deriving its rest length and
// (for any welded end) its rest angle. `posable` is the one field here that says
// nothing about the running physics: it marks the rod as one the player may pose
// THROUGH, released to a bare rail for the duration of a drag (§06.2d).
function makeRodCon(a,b,weldA,weldB,posable){
  const [wax,way]=epWorld(a), [wbx,wby]=epWorld(b);
  const con={type:'rod', a, b, len:Math.hypot(wax-wbx,way-wby), weldA:!!weldA, weldB:!!weldB,
             posable:!!posable, pts:[], sel:false};
  if(con.weldA) captureRestAngle(con,'A');
  if(con.weldB) captureRestAngle(con,'B');
  return con;
}
// Set (or clear) one end's weld flag, recapturing that end's rest angle
// against the rod's *current* direction so toggling never snaps geometry.
function setRodWeld(con,which,val){
  const key = which==='A'?'weldA':'weldB'; con[key]=!!val;
  if(con[key]) captureRestAngle(con,which);
}
function toggleRodWeld(con,which){ setRodWeld(con,which, !con[which==='A'?'weldA':'weldB']); }

// Build a slot/rail constraint between two endpoints. Unlike a rod, a slot
// with both ends "pin" is physically inert (§06.5) -- prismaticA/prismaticB
// are what give it any rows at all, so their rest angles are always needed
// once either is set.
function makeSlotCon(a,b,prismaticA,prismaticB){
  const con={type:'slot', a, b, prismaticA:!!prismaticA, prismaticB:!!prismaticB,
             pts:[], sel:false};
  if(con.prismaticA) captureRestAngle(con,'A');
  if(con.prismaticB) captureRestAngle(con,'B');
  return con;
}
// Set (or clear) one end's prismatic flag. If this toggle completes the
// both-locked (rigid) state, also refresh the *other* end's rest angle --
// it may have gone stale while only one side was locked -- so the lateral
// position lock (added only once both are true, §06.5) starts exactly on
// the rail with no snap.
function setSlotLock(con,which,val){
  const key = which==='A'?'prismaticA':'prismaticB'; con[key]=!!val;
  if(con[key]) captureRestAngle(con,which);
  if(con.prismaticA && con.prismaticB){ captureRestAngle(con,'A'); captureRestAngle(con,'B'); }
}
function toggleSlotLock(con,which){ setSlotLock(con,which, !con[which==='A'?'prismaticA':'prismaticB']); }
// The slot's current rail angle, for rendering and for the lateral lock row:
// tracked via whichever end is locked (they agree once both are), or -- with
// neither locked, the cosmetic-only case -- just the live segment direction.
function slotRailAngle(con){
  if(con.prismaticB){ const B=con.b.id!=null?bodies[bodyIndex(con.b.id)]:null;
    return (B?B.th:0)-con.restAngB; }
  if(con.prismaticA){ const A=con.a.id!=null?bodies[bodyIndex(con.a.id)]:null;
    return (A?A.th:0)-con.restAngA; }
  const [wax,way]=epWorld(con.a), [wbx,wby]=epWorld(con.b);
  return Math.atan2(way-wby,wax-wbx);
}

// ---- §06.2 · the remaining constraint makers ----
// Pin, belt, CVT, knife and cable were built as object literals at each of their
// call sites until the scene file (§17) needed a third one. Every constraint kind
// now has exactly ONE constructor, called from exactly two places -- the tool
// dispatch (§13.5) and the scene reader (§17.4) -- so "what fields does a belt
// have" has a single answer, and a scene file cannot describe a constraint the
// tools cannot build. See SCENE.md §S.2.

// A pin coincides two body-local points. Both ends are real bodies: pinning a body
// to the background is a rod with a welded background end (§15's rodBG), not this.
function makePinCon(a,b){ return {type:'pin', a, b, pts:[], sel:false}; }

// A belt couples two disks' rim speeds. The wrap radii default to the bodies' own
// radii and the phase is captured from their live angles, so a freshly built belt
// is unstressed -- the same capture the wrap-radius and crossed-belt edits redo
// (inspector.js §14.2).
function makeBeltCon(aId,bId,sense){
  const A=bodies[bodyIndex(aId)], B=bodies[bodyIndex(bId)];
  const sn = sense===-1 ? -1 : 1;
  return {type:'belt', a:{id:aId}, b:{id:bId}, rA:A.r, rB:B.r, sense:sn,
          restPhase:(A.r*A.th - sn*B.r*B.th), sel:false};
}
// The variable-ratio rolling contact carries no captured state at all -- its ratio
// is read from the live geometry every step (§06.5).
function makeCvtCon(aId,bId){ return {type:'cvt', a:{id:aId}, b:{id:bId}, sel:false}; }

// A rack and pinion. The RACK is an infinite, massless toothed line named by TWO
// endpoints, `a` and `b` -- ordinary {id, off} anchors like a rod's or a slot's,
// either of which may ride a body or the fixed background. Between them they say
// where the rack is and which way it points, and nothing else: there is no rest
// length, so the pair is two pins, not a strut.
//
// The two ends are NOT symmetric, and the asymmetry is the physics rather than an
// implementation detail. The rack is rigid, so `a` is the one material point of it
// that is pinned -- the rack's own origin -- while `b` only AIMS it: the line passes
// through b, but b may slide along the rack. Pinning both ends materially would make
// the rack a rod between two bodies, which is a different object.
//
// Either end may additionally be WELDED (weldA/weldB), which locks that end's body
// angle to the rack's own heading exactly as a rod's weld or a slot's prismatic lock
// does -- the same endpointAngleLockRow, against the same phi. Both ends default to
// unwelded (free pins). Put both ends on the SAME body and the rack rides that body's
// frame completely, translating and turning with it, which is the arrangement the
// rack had when its direction was a mandatory weld plus a body-frame angle.
//
// What meshes with the rack lives in `pts` (§06.2c): a 'pinion' point is a circular
// body meshing with perfect traction wherever it sits, a plain point is a body
// jointed to the rack at a fixed station along it. A rack carries as many of each as
// it likes; a freshly built one is given its first pinion by the caller.
function makeRackCon(a,b,weldA,weldB){
  const con={type:'rack', a, b, weldA:!!weldA, weldB:!!weldB, pts:[], sel:false};
  if(con.weldA) captureRestAngle(con,'A');
  if(con.weldB) captureRestAngle(con,'B');
  return con;
}
// Set (or clear) one rack end's weld flag, recapturing that end's rest angle against
// the rack's current heading -- the rod/slot toggles' exact counterpart.
function setRackWeld(con,which,val){
  const key = which==='A'?'weldA':'weldB'; con[key]=!!val;
  if(con[key]) captureRestAngle(con,which);
}
function toggleRackWeld(con,which){ setRackWeld(con,which, !con[which==='A'?'weldA':'weldB']); }
// Rack geometry: the two-endpoint frame every line constraint shares (§06.1
// twoPointFrame), so the rack's heading is read live off its two pins and its
// welded ends measure against the same phi a rod's do. `px,py` is end a's world
// point -- the rack's material origin -- kept under those names because the render
// and hit-test paths draw the line through it.
function rackFrame(con){
  const f = twoPointFrame(con);
  return Object.assign(f, {px:f.wax, py:f.way, ang:f.phi});
}
// One pinion's live pitch geometry: the disk, and rho, its SIGNED perpendicular
// distance from the rack line, positive on the +n side. Signed, not clamped: the row
// stays correct however the pinion crosses to the rack's far side. rho is a
// coordinate rather than a constant, which is what makes the mesh row nonholonomic --
// the same "ratio is a coordinate" move the CVT's contact makes.
// `f` is the constraint's twoPointFrame -- the plain one rowsFor builds as well as
// the rackFrame alias, so this reads wax/way (end a's world point) rather than the
// px/py names only the render path uses.
function rackPitch(f, pt){
  if(!pt) return null;
  const ib = pt.ep.id!=null ? bodyIndex(pt.ep.id) : -1;
  const B = ib>=0 ? bodies[ib] : null;
  if(!B) return null;
  const rho = (B.x-f.wax)*f.nx + (B.y-f.way)*f.ny;
  return {B, ib, rho};
}
// The rack's first pinion, or null -- what the inspector's pitch-radius readout and
// the reaction arrow (§09.3) report on when a rack carries several.
function rackFirstPinion(con){
  for(const pt of conPoints(con)) if(pt.kind==='pinion') return pt;
  return null;
}

// A knife edge forbids sideways motion of one body-local point. `dir` is the
// heading in the body's OWN frame -- callers holding a world direction rotate it in
// by R(-b.th, ...) first, as the tool does.
function makeKnifeCon(a,dir){ return {type:'knife', a, dir, sel:false}; }

// A cable runs from a tether point (a body-local anchor, or the background) to a
// spool disk it winds on. Ltot and localAngle are captured from the live geometry:
// the free span at creation, and the rim point nearest the tether (spoolAngle = 0).
// See CABLE.md §C.3.
function makeCableCon(tether,spoolId){
  const S=bodies[bodyIndex(spoolId)];
  const [tx,ty]=epWorld(tether);
  const dvx=tx-S.x, dvy=ty-S.y, d=Math.hypot(dvx,dvy);
  return {type:'cable', tether, spool:{id:spoolId},
          localAngle:Math.atan2(dvy,dvx)-S.th, spoolAngle:0,
          Ltot: d>S.r ? Math.sqrt(d*d-S.r*S.r) : 0, sel:false};
}

// `mergeCols` sums duplicate body-index entries -- required, not cosmetic:
// physics.js's Schur assembly builds each row's per-body map with
// `mp.set(idx,...)`, which *overwrites* rather than accumulates, so two
// separate column entries for the same body would silently drop one and
// corrupt the row if a caller ever produced two entries on the same body.
// Scale every column of a row by k -- the companion to mergeCols, used wherever a
// row is a linear combination of frames rather than a plain difference of two (the
// bar-rotation term every extra control point's rows carry, §06.2c).
function scaleCols(cols,k){ return cols.map(c=>[c[0], c[1]*k, c[2]*k, c[3]*k, (c[4]||0)*k]); }
function mergeCols(colArrays){
  const m=new Map();
  for(const cols of colArrays) for(const [idx,cx,cy,cw,cl] of cols){
    const l=cl||0;
    const e=m.get(idx); if(e){ e[0]+=cx; e[1]+=cy; e[2]+=cw; e[3]+=l; } else m.set(idx,[cx,cy,cw,l]);
  }
  return [...m.entries()].map(([idx,[cx,cy,cw,cl]])=>[idx,cx,cy,cw,cl]);
}
// Resolve a rod/pin/spring endpoint -- a plain body {id,off} or a background
// point {id:null,off} -- to its live world position plus a `velCols` closure
// giving the velocity-Jacobian columns for an arbitrary probe direction (a
// plain body reduces to the ordinary single-column rotate-form, background
// to no columns at all). Not used by rod's weld / slot's prismatic locks --
// those keep using twoPointFrame directly.
// `angCols` is the companion to velCols for the one row shape that measures an
// endpoint's own *rotation* rather than a point's translation (the rod weld and slot
// prismatic locks, endpointAngleLockRow below): the endpoint body's angular-velocity
// column, or none at all for a background endpoint, whose frame angle is the fixed
// world zero. `th` reads the same way -- a body's own theta, or 0 for background.
//
// On a vessel, `off` is the material label (lat, f) of §05.2c, and the point's world
// position picks up a length dependence: d(world)/d(len) = f * axis. That is the
// whole of the len column, and it is why an endpoint's *material fraction* is what
// decides how much it restrains the vessel's breathing.
function epFrame(ep){
  if(ep.id==null) return { wx:ep.off[0], wy:ep.off[1], idx:-1, th:0,
                           velCols:()=>[], angCols:()=>[] };
  const idx=bodyIndex(ep.id); const b=bodies[idx];
  const [wx,wy,rx,ry]=worldPt(b, epLocal(b,ep.off));
  if(b.shape==='vessel'){
    const f=ep.off[1], ax=vesselAxis(b);
    return { wx, wy, idx, th:b.th,
      velCols:(dirx,diry)=>[[idx, dirx, diry, dirx*(-ry)+diry*rx, f*(dirx*ax[0]+diry*ax[1])]],
      angCols:()=>[[idx,0,0,1,0]] };
  }
  return { wx, wy, idx, th:b.th,
    velCols:(dirx,diry)=>[[idx, dirx, diry, dirx*(-ry)+diry*rx]],
    angCols:()=>[[idx,0,0,1]] };
}

// ---- §06.2b · derived freezing (which coordinates a scene has pinned) ----
// `static` and `lenLock` are not properties a player sets. They are DERIVED, every
// substep, from the constraints actually present -- so a coordinate is frozen only
// when something in the scene says it is, and the thing that says so is a
// first-class object the player can see, select, and delete.
//
// Freezing is an optimization, not the physics: the solver already holds a
// constrained body exactly. What it buys is a coordinate removed from the system
// (and its now-redundant constraint compiled away, `_compiled` below) plus the
// island split that makes a fixed body a wall between what it touches.
//
// TWO patterns are recognized, both structural -- they depend on what is attached,
// never on the current configuration, so nothing freezes or thaws as a mechanism
// swings through a pose. Other arrangements do pin a body (three pin-ended rods to
// the ground, say); they are simply not optimized, and the solver handles them
// exactly as it always has. Recognizing those in general means a rank computation on
// the Jacobian every step, which would be both expensive and configuration-dependent
// -- the very thing this avoids. See SCENE.md §S.8.

// A double-welded rod pins its far end's frame completely: distance, direction and
// orientation are all held. So it grounds a body whose other end is the background,
// or a body already grounded -- applied to a fixed point, that is what "static"
// means, and it is the only thing that makes it so.
//
// The exception is a vessel anchored anywhere but its MID-PLANE. A vessel's fourth
// coordinate moves its own material: a point at material fraction f sits f*len from
// the centre (§05.2c), so pinning a cap fixes the cap, not the centre -- the centre
// still rides the length. Only f = 0, whose world position has no length dependence,
// pins the body's pose. That is the difference between the gas spring (welded at its
// cap, f = -1/2, and genuinely free to move as it breathes) and the heat pair's
// working vessel (welded at its mid-wall, f = 0, pose fixed and length free).
function rodGrounds(con){
  if(con.type!=='rod' || !con.weldA || !con.weldB) return null;
  if(rodReleased(con)) return null;      // released for the pose drag: grounds nothing (§06.2d)
  // A rod carrying extra control points (§06.2c) still has rows of its own to solve
  // once its base pair is compiled away, so it is not a candidate for compiling.
  if(conPoints(con).length) return null;
  const held = ep => { if(ep.id==null) return true;
                       const b=bodies[bodyIndex(ep.id)]; return !!(b && b.static); };
  const far = held(con.a) ? con.b : held(con.b) ? con.a : null;
  if(!far || far.id==null) return null;
  const b = bodies[bodyIndex(far.id)]; if(!b) return null;
  if(b.shape==='vessel' && far.off[1]!==0) return null;      // not the mid-plane
  return b;
}
// A rod with BOTH ends on the same vessel, at different material fractions, holds
// the distance between two points that move only with the length -- so it holds the
// length, and nothing else. Its pose columns cancel exactly (mergeCols sums them),
// which is also why the same rod on a rigid body is degenerate and the tool refuses
// it (§13.5). This is what a reservoir is: a vessel with a strut inside it.
function rodLocksLength(con){
  if(con.type!=='rod' || con.a.id==null || con.a.id!==con.b.id) return null;
  if(rodReleased(con)) return null;      // as above -- a released strut holds no length either
  if(conPoints(con).length) return null;                     // see rodGrounds above
  const v=bodies[bodyIndex(con.a.id)];
  if(!v || v.shape!=='vessel' || con.a.off[1]===con.b.off[1]) return null;
  return v;
}
// Recompute every body's frozen flags and every constraint's `_compiled` mark.
// Iterated to a fixed point because grounding is transitive: a body double-welded to
// a body that is itself grounded is grounded too.
function refreshFrozen(){
  for(const b of bodies){ b.static=false; b.lenLock=false; }
  for(let pass=0; pass<=bodies.length; pass++){
    let changed=false;
    for(const con of constraints){
      const g=rodGrounds(con);    if(g && !g.static){ g.static=true; changed=true; }
      const v=rodLocksLength(con); if(v && !v.lenLock){ v.lenLock=true; changed=true; }
    }
    if(!changed) break;
  }
  // A constraint that does the freezing has nothing left to solve: every column it
  // would write lands on a coordinate that no longer moves. Left in, it would be a
  // row of zeros that only the Tikhonov term keeps solvable, reporting a reaction
  // read off the regularizer rather than off the mechanism. Compile it away instead.
  for(const con of constraints) con._compiled = !!(rodGrounds(con) || rodLocksLength(con));
  for(const b of bodies) refreshInertia(b);   // the inverse masses follow the flags
}
// A body whose EVERY coordinate is frozen is a wall: nothing passes through it, so
// islands may split there (§08.0). A vessel pinned at its mid-plane is not one --
// its length is still a live channel between whatever is attached to it.
const frozenSolid = b => b.static && (b.shape!=='vessel' || b.lenLock);

// After a frozen body is moved by hand -- dragged, or its pose typed into the
// inspector -- nothing in the solver will pull its anchors back into agreement,
// because the rows that would have done so are compiled away. Recapture them from
// the new pose instead, exactly as creating the rod would have.
function recaptureGrounding(b){
  for(const con of constraints){
    if(rodGrounds(con)!==b && rodLocksLength(con)!==b) continue;
    recaptureRodPose(con);
  }
}
// Re-read everything a rod holds off the live geometry -- its length, either weld's
// rest angle, and every extra point's station and rest angle (§06.2c). This is
// exactly what makeRodCon and makeConPoint capture at creation, so a rod recaptured
// at a pose it already satisfies is unchanged, and one recaptured after a hand move
// holds the new pose instead. Called from recaptureGrounding above and, once per
// pose-drag step, from recapturePosable (§06.2d).
function recaptureRodPose(con){
  // Drop the unwrapping anchor first. Every capture below reads the RAW segment
  // angle (captureRestAngle), so the rows have to read it raw too, or a rod that
  // was posed past the branch cut -- swung round its anchor through the -x
  // direction -- comes out of the recapture holding a rest angle a full turn from
  // the phi its own weld row measures against. Cleared, the next twoPointFrame
  // re-seeds from the same raw atan2 the capture used, which is exactly the state a
  // freshly loaded (or freshly Reset) scene is in. Safe to do here and nowhere else:
  // a rod being recaptured is one nothing is currently holding -- compiled away, or
  // released for the drag -- so there is no live phi continuity to break.
  con._phiRef=undefined;
  const [wax,way]=epWorld(con.a), [wbx,wby]=epWorld(con.b);
  con.len=Math.hypot(wax-wbx,way-wby);
  if(con.weldA) captureRestAngle(con,'A');
  if(con.weldB) captureRestAngle(con,'B');
  for(const pt of conPoints(con)){
    if(pt.s!==undefined) pt.s=capturePointStation(con, pt.ep);
    if(pt.lock) pt.restAng=capturePointRestAngle(con, pt.ep);
  }
}

// ---- §06.2c · extra control points (a constraint with more than two ends) ----
// A pin, rod, slot or rack is named by two endpoints, and those two are what the
// constraint IS: a rod's pair fixes its length, a slot's pair is its rail, a rack's
// pair is its line. Anything else attached to the same joint is an EXTRA CONTROL
// POINT, kept in `con.pts` -- an ordinary {id, off} endpoint (a body or the fixed
// background, exactly like a and b) plus what that attachment means:
//
//   pin    the point coincides with the pivot          2 rows
//   rod    the point is fixed to the bar at station s  2 rows  (+1 welded)
//   slot   the point rides the rail, free to slide     1 row   (+1 prismatic)
//   rack   'point'  jointed to the rack at station s   2 rows  (+1 welded)
//          'pinion' a disk meshing with the rack       1 row   (nonholonomic)
//
// `s` is the point's STATION: its signed distance from end a along the line, in the
// direction a - b. It is CAPTURED at creation (SCENE.md §S.3) for the two kinds that
// hold a fixed position along the line, and simply absent for the two that do not --
// a slot's riders slide, and a pin's coincide.
//
// The lateral offset is not captured, because it is not a degree of freedom the
// editor can produce: a point is always placed on the line itself (the placement
// click is projected onto it, tools.js §13.5), so "off the line" is a state a scene
// cannot describe rather than one it stores as zero.
//
// `lock` is the extra point's own rotation lock, the same one a and b carry under
// their per-kind names (weldA/weldB on a rod or rack, prismaticA/prismaticB on a
// slot): set, the point's body angle is held to the line's own heading through the
// same pointAngleLockRow, against a rest angle captured when the lock goes on.
const conPoints = con => con.pts || (con.pts=[]);
// Which kinds take extra points at all, and whether their points hold a station.
const CON_MULTI = ['pin','rod','slot','rack'];
const conTakesPoints = con => CON_MULTI.includes(con.type);
const conPointHasStation = con => con.type==='rod' || con.type==='rack';
// A rack's pinions have no rotation lock and no station: they mesh wherever they sit.
const conPointLockable = (con,pt) => conTakesPoints(con) && con.type!=='pin' && pt.kind!=='pinion';

// The station an extra point currently sits at, read off the live geometry -- the
// capture makeConPoint does when the scene file does not name one.
function capturePointStation(con, ep){
  const f=twoPointFrame(con);
  const [wx,wy]=epWorld(ep);
  return (wx-f.wax)*f.ux + (wy-f.way)*f.uy;
}
// ...and its rest angle, against the line's own direction -- captureRestAngle's
// counterpart for a point that is not one of the two named ends.
function capturePointRestAngle(con, ep){
  const [wax,way]=epWorld(con.a), [wbx,wby]=epWorld(con.b);
  const phi=Math.atan2(way-wby,wax-wbx);            // raw -- see captureRestAngle
  const th = ep.id!=null ? bodies[bodyIndex(ep.id)].th : 0;
  return th-phi;
}
// THE constructor for an extra control point (SCENE.md §S.2): called by the tool
// dispatch (§13.5) and the scene reader (§17.4) and nowhere else. `opts.s` and
// `opts.restAng` are the file's captured values; omitted, both are read off the live
// geometry, so a freshly placed point starts exactly where it was clicked.
function makeConPoint(con, ep, opts){
  const o = opts || {};
  // Normalized to the full {id, off} endpoint shape every anchor in the engine has
  // (geometry.js §05.2c), and copied rather than aliased: a pinion is named by a bare
  // body id in the scene file and in the tool alike, but the handle and render paths
  // resolve every point through epWorld, which needs the offset to exist.
  const e = { id: ep.id, off: ep.off ? ep.off.slice() : [0,0] };
  const pt = { ep:e, kind: o.kind==='pinion' ? 'pinion' : 'point', lock:false };
  if(conPointLockable(con,pt)){
    pt.lock = !!o.lock;
    if(conPointHasStation(con)) pt.s = o.s!==undefined ? o.s : capturePointStation(con, e);
    if(pt.lock) pt.restAng = o.restAng!==undefined ? o.restAng : capturePointRestAngle(con, e);
  }
  conPoints(con).push(pt);
  return pt;
}
// Set (or clear) an extra point's rotation lock, recapturing its rest angle against
// the line's current heading so toggling never snaps geometry (setRodWeld's twin).
function setConPointLock(con, pt, val){
  if(!conPointLockable(con,pt)) return;
  pt.lock=!!val;
  if(pt.lock) pt.restAng=capturePointRestAngle(con, pt.ep);
}
function toggleConPointLock(con, pt){ setConPointLock(con, pt, !pt.lock); }
// The lock a NEWLY added point should get. If every point already on the constraint
// agrees -- all free to rotate, or all locked -- the new one joins them; a constraint
// that already mixes the two gets a locked point, the conservative reading, since a
// lock can be tapped off but a missing one is invisible until the mechanism moves.
function conNewPointLock(con){
  const flags=[];
  if(con.type==='rod'||con.type==='rack') flags.push(!!con.weldA, !!con.weldB);
  else if(con.type==='slot') flags.push(!!con.prismaticA, !!con.prismaticB);
  else return false;                       // a pin has no rotation lock to inherit
  for(const pt of conPoints(con)) if(pt.kind!=='pinion') flags.push(!!pt.lock);
  return flags.every(v=>v===flags[0]) ? flags[0] : true;
}
// Every endpoint a constraint names, base pair and extra points alike. The one
// answer to "which bodies does this couple", read by the island pass (§08.0), the
// delete paths (§13.5, §14.2) and the body-resize rescale (§13.3).
function conEndpoints(con){
  const eps=[];
  if(con.a) eps.push(con.a);
  if(con.b) eps.push(con.b);
  for(const pt of conPoints(con)) eps.push(pt.ep);
  return eps;
}
// The point on a line constraint's own line nearest a world point. Where a placement
// click lands (tools.js §13.5) and where a dragged control point is held: a point is
// always ON the line, never beside it, which is what lets §06.2c store no lateral
// offset. Uses the live a-b segment, the same line the rows measure against -- for a
// slot that is not quite the rail as DRAWN (which tracks a locked end's railAngle),
// and the rows are the thing to agree with.
function conLineProject(con, wx, wy){
  const [wax,way]=epWorld(con.a), [wbx,wby]=epWorld(con.b);
  const dx=wax-wbx, dy=way-wby, L2=dx*dx+dy*dy;
  if(!(L2>0)) return [wax,way];
  const t=((wx-wbx)*dx+(wy-wby)*dy)/L2;
  return [wbx+t*dx, wby+t*dy];
}
// Deleting a body takes with it every constraint whose BASE pair names it -- the
// constraint cannot exist without both its ends -- but only the individual extra
// points that do, since the rest of the joint is still a joint without them.
function dropBodyFromConstraints(id){
  constraints = constraints.filter(c => c.a.id!==id && !(c.b && c.b.id===id));
  for(const c of constraints)
    if(c.pts && c.pts.length) c.pts = c.pts.filter(pt => pt.ep.id!==id);
}

// ---- §06.2d · posable rods (the pose-time release) ----
// A rod may be marked `posable`. It changes nothing about the running physics --
// a posable rod is an ordinary rigid rod at every substep -- and everything about
// what happens while the player POSES the machine: dragging a body around with the
// sim paused (tools.js §13.6). A posable rod DIRECTLY JOINTED TO THE DRAGGED BODY
// is RELEASED for the length of that drag, and holds only its own line:
//
//   * its distance row is gone, so the two ends may slide toward and away from each
//     other -- the rod's length is what the drag is free to change;
//   * its welds are gone, so every body it joins turns freely. That is the whole of
//     "a rail with all joined bodies PINNED, not welded";
//   * its extra control points (§06.2c) stop being rigid attachments at a station
//     and become RIDERS, held on the line and free to slide along it -- the same one
//     row a slot's riders get, which is the sense in which the bar becomes a rail.
//
// "Directly jointed" is conEndpoints (§06.2c) and nothing cleverer: the rod names
// the dragged body as one of its ends or as one of its extra points. A posable rod
// one joint further away stays a rigid rod, so the release reaches exactly as far as
// the hand does -- grab a body and the members it hangs off go slack, and the rest of
// the machine articulates around them as it always would. The alternative, releasing
// every posable rod in the scene for the duration of any drag, is both a bigger edit
// than the gesture asks for and one the player cannot see the extent of.
//
// A rod grounding a body, or locking a vessel's length, releases those too (see
// rodGrounds/rodLocksLength above): a rod that holds nothing cannot be the thing
// that froze a coordinate, and a posable ground strut would be useless if it still
// pinned the body the player is trying to slide along it. That is the one place the
// derived freezing of §06.2b depends on something beyond what is attached -- and what
// it depends on is the gesture in progress, never the configuration, so nothing still
// freezes or thaws as a mechanism swings through a pose (SCENE.md §S.8).
//
// Two pieces of state, because the release answers two questions on two different
// timescales, and conflating them is what a canvas frame drawn BETWEEN pointermoves
// exposes:
//
//   posingRoot  the body a pose GESTURE is dragging, held for the whole gesture --
//               pointerdown to pointerup. This is what the rod IS to the player, and
//               so it is what the canvas draws (render.js §11.5).
//   posing      whether we are inside the row-building scope right now. This is what
//               the rod HOLDS. Between two pointermoves it is zero and the rod is a
//               rigid rod again, which is what makes the recapture meaningful and
//               keeps conMaxC, the violation highlight and saveState honest.
//
// Both live here rather than in the tool layer so that render.js -- which loads
// BEFORE tools.js -- never has to reach forward for them. It did once, and a forward
// reference that fails to resolve throws inside render(), which kills the rAF chain
// in §10 outright: the page stops redrawing and never starts again.
//
// `posing` is a depth counter rather than a flag so that a nested projection (or a
// caller that wraps another) cannot clear it early. `posingRoot` is set and cleared
// by the tool layer wherever it sets and clears its own `drag` (§13.5/§13.7); a stale
// one would only draw a rail nobody is riding, since the rows still need `posing`.
let posing = 0, posingRoot = null;
function beginPosing(rootId){ posingRoot = rootId==null ? null : rootId; }
function endPosing(){ posingRoot = null; }
function withPosing(fn){ posing++; try { return fn(); } finally { posing--; } }
// Whether a drag on body `rootId` would release this rod -- the predicate on its own,
// with no reference to any of the state above, so a check can ask it about a gesture
// that is not happening (tools/posable-check.js does).
const rodPosableFor = (con, rootId) => con.type==='rod' && !!con.posable && rootId!=null
  && conEndpoints(con).some(ep => ep.id===rootId);
// Released by the gesture in progress (what the canvas draws) ...
const rodPosing = con => rodPosableFor(con, posingRoot);
// ... and released right now, in the rows being built (what the solver sees).
const rodReleased = con => posing>0 && rodPosing(con);

// Once the drag step has settled, a released rod re-reads what it holds from the
// pose the player just produced (§06.2b recaptureRodPose): the length, the welds'
// rest angles, the points' stations. Done every pointermove rather than at
// pointerup, for the same reason recaptureGrounding is: between moves the rod is a
// rigid rod again, and one whose captured length disagreed with its live geometry
// would read as a violated constraint -- red on the canvas, and refused as the reset
// baseline (transport.js §16.1) -- for as long as the drag lasted. It walks the same
// rods the release did, so a posable rod the drag never touched keeps the length it
// was authored at. A released rod that was under LOAD when the drag began does lose
// that load, since a released rod is not holding anything for the projection to
// fight: an adjustable member has no preload to speak of, and that is what marking
// one posable declares it to be.
function recapturePosable(){
  for(const con of constraints)
    if(rodPosing(con)) recaptureRodPose(con);
}

// ---- §06.3 · cableFrame ----
// Cable geometry based on a consistently-defined spool angle.
//
// Key points (A, B, C, D per the spec):
//   A = spool anchor -- material point on spool rim, stored as cb.localAngle in
//       the spool body frame.  Initialised so spoolAngle = 0 (anchor at closest
//       rim point to tether).
//   B = spool centre (S.x, S.y)
//   C = tether point T
//   D = tangent point -- rim point where tangent from T touches the spool on the
//       winding side determined by sign(spoolAngle).
//
//   spoolAngle = ABC angle at B (from ray BA to ray BC, CCW positive, unbounded):
//     = tetherAngle - anchorAngle.
//     Positive -> anchor is CW of tether direction -> cable winds CW.
//     Negative -> anchor is CCW of tether direction -> cable winds CCW.
//
//   Tangent point D (world angle from B):
//     d > rs: tangentAngle = tetherAngle - sign(spoolAngle) · arccos(rs/d)
//     d <= rs: tangentAngle = tetherAngle (rim point closest to T; or anchorAngle if d~=0)
//
//   |DBC| = arccos(rs/d) for d>rs; 0 for d<=rs.
//   Q = D (tangent wins) if |DBC| < |spoolAngle|;  else Q = A.
//
//   windAngle (same sign as spoolAngle):
//     tangent wins: windAngle = spoolAngle - sign·arccos(rs/d)   [= 0 at transition;
//       arccos(rs/d) reads as 0 once d<=rs, so this is one continuous formula]
//     anchor wins:  windAngle = 0
//   woundLength = |windAngle| · rs
//   paidLength  = Lfree = sqrt(max(0, d²-rs²))  when tangent wins (0 once d<=rs);
//                 |T - Q|  when anchor wins (Q=A, an ordinary rod to T)
//   totalUsed   = woundLength + paidLength
//
//   Jacobian constrains d/dt(totalUsed) = 0, selected by tangentWins alone --
//   NOT also by d<=rs. A many-turn wind can overshoot to d<=rs for a step near
//   the ell->0 singularity while still genuinely in the tangent regime (large
//   |spoolAngle|); Lfree is already 0 there, so the tangent-mode row stays
//   the right (and continuous) one. Only the anchor-wins case is a real rod.
//   Tangent mode: Jx=(Dx·Lfree - rs·sign·Dy)/d², Jy=(Dy·Lfree + rs·sign·Dx)/d²;
//     spool [-Jx, -Jy, -rs·sign]; tether [Jx, Jy, moment arm].
//   Direct/rod mode (Q=A): spool [-ux,-uy, ux·ry_Q-uy·rx_Q]; tether [ux,uy,arm].
//   Returns null only when the spool body is missing.
function cableFrame(cb, spoolAngleRef){
  const S=bodies[bodyIndex(cb.spool.id)]; if(!S) return null;
  const rs=S.r;
  const is=bodyIndex(cb.spool.id);
  let T, tb=null, trx=0, tryy=0, ti=-1;
  if(cb.tether.id!=null){ ti=bodyIndex(cb.tether.id); tb=bodies[ti];
    const [tx,ty,rx,ry]=worldPt(tb,epLocal(tb,cb.tether.off)); T=[tx,ty]; trx=rx; tryy=ry; }
  else { T=[cb.tether.off[0],cb.tether.off[1]]; }

  // Spool anchor A: material point on rim.
  const localAngle   = cb.localAngle !== undefined ? cb.localAngle : 0;
  const anchorAngle  = S.th + localAngle;
  const rx_anchor    = rs*Math.cos(anchorAngle), ry_anchor = rs*Math.sin(anchorAngle);
  const Ax = S.x + rx_anchor, Ay = S.y + ry_anchor;

  // B->C vector and tether world angle.
  const Dx = T[0]-S.x, Dy = T[1]-S.y;
  const d  = Math.hypot(Dx, Dy);
  const tetherAngle = Math.atan2(Dy, Dx);

  // spoolAngle: ABC, unwrapped around previous reference for continuity.
  const spoolAngleRaw = tetherAngle - anchorAngle;
  let spoolAngle;
  if(spoolAngleRef != null){
    let da = spoolAngleRaw - spoolAngleRef;
    while(da >  Math.PI) da -= Math.PI*2;
    while(da < -Math.PI) da += Math.PI*2;
    spoolAngle = spoolAngleRef + da;
  } else {
    const seed = cb.spoolAngle !== undefined ? cb.spoolAngle : 0;
    let da = spoolAngleRaw - seed;
    while(da >  Math.PI) da -= Math.PI*2;
    while(da < -Math.PI) da += Math.PI*2;
    spoolAngle = seed + da;
  }
  const sign = spoolAngle >= 0 ? 1 : -1;

  // Tangent point D and DBC angle. Lfree is defined for any d (0 once d<=rs)
  // so it stays the source of truth for the free length even through a
  // step that momentarily overshoots the rim -- see the tangentWins branch
  // below.
  const tetherInside = d <= rs;
  const beta  = tetherInside ? 0 : Math.acos(Math.max(-1, Math.min(1, rs/d)));
  const Lfree = Math.sqrt(Math.max(0, d*d - rs*rs));
  const tangentAngle = tetherInside ? (d > 1e-9 ? tetherAngle : anchorAngle) : (tetherAngle - sign*beta);
  const dbc = beta;                              // |DBC| = arccos(rs/d), 0 when inside
  const rx_tan = rs*Math.cos(tangentAngle), ry_tan = rs*Math.sin(tangentAngle);
  const Qtan_x = S.x + rx_tan, Qtan_y = S.y + ry_tan;

  // Separation point Q. tangentWins governs both Q's choice and (below) which
  // Jacobian form applies -- it must NOT also fork on tetherInside: a body
  // deep in a many-turn wind can overshoot to d<=rs for a step near the
  // ell->0 singularity while still genuinely in the tangent regime (large
  // |spoolAngle|), and Lfree already degrades continuously to 0 there. Only
  // gating on tetherInside forced a jump to the anchor-rod formula against
  // the wrong point (Qtan, not A) for that step -- a large, energy-adding
  // direction discontinuity, not the harmless near-Delta=0 interior tether the
  // rod formula is actually for (design note §C.6).
  const tangentWins = dbc < Math.abs(spoolAngle) - 1e-10;
  let Qx, Qy, rx_Q, ry_Q, windAngle, paidLength;
  if(tangentWins){
    Qx = Qtan_x; Qy = Qtan_y; rx_Q = rx_tan; ry_Q = ry_tan;
    windAngle  = spoolAngle - sign*beta;
    paidLength = Lfree;
  } else {
    Qx = Ax; Qy = Ay; rx_Q = rx_anchor; ry_Q = ry_anchor;
    windAngle  = 0;
    paidLength = Math.hypot(T[0]-Qx, T[1]-Qy);
  }
  const woundLength = Math.abs(windAngle) * rs;
  const totalUsed   = woundLength + paidLength;
  const Lallow      = cb.Ltot - woundLength;

  const ux = paidLength > 1e-9 ? (T[0]-Qx)/paidLength : 0;
  const uy = paidLength > 1e-9 ? (T[1]-Qy)/paidLength : 1;

  // Jacobian for d/dt(totalUsed) = 0. The -rs·sign angular term on the spool row
  // was re-derived directly from d(totalUsed)/dt (chain rule through spoolAngle,
  // beta, Lfree) and matches: it is the same physical row as the pre-rebuild
  // +rs·side, given that convention's side = -sign(spoolAngle) (see the wrap/side
  // reconstruction in physics.js's cable migration).
  // The tether end's columns go through epFrame (§06.1) rather than being written
  // out here, so a tether anchored on a vessel picks up its length column the same
  // way every other endpoint does. The spool row stays hand-built: it is a rim
  // tangent term, not a point-velocity projection, and a spool is always a disk.
  const epT = tb ? epFrame(cb.tether) : null;
  let cols;
  if(tangentWins){
    // d2 floors away from 0 for a step that overshoots deep past the rim
    // (large angular rate near ell->0, see cableFrame's header) -- Lfree is
    // already 0 there, so the row is still the correct tangential direction,
    // just guarded against an actual division by a near-zero d.
    const d2 = Math.max(d*d, 1e-9);
    const Jx = (Dx*Lfree - rs*sign*Dy) / d2;
    const Jy = (Dy*Lfree + rs*sign*Dx) / d2;
    cols = mergeCols([ S.static?[]:[[is, -Jx, -Jy, -rs*sign]],
                       (tb&&!tb.static)?epT.velCols(Jx,Jy):[] ]);
  } else {
    cols = mergeCols([ S.static?[]:[[is, -ux, -uy, ux*ry_Q - uy*rx_Q]],
                       (tb&&!tb.static)?epT.velCols(ux,uy):[] ]);
  }

  return {
    S, rs, T, Qx, Qy, ux, uy, cols,
    spoolAngle, windAngle, woundLength, paidLength, totalUsed, Lallow,
    Lfree, Ax, Ay, rx_anchor, ry_anchor, anchorAngle,
    tangentAngle, rx_tan, ry_tan, Qtan_x, Qtan_y, tangentWins,
    rx_Q, ry_Q, localAngle, tb, ti, trx, tryy, is, tetherInside
  };
}

// Current geometric cable path length for this configuration.
function cableCurrentLength(cb, f){
  const cf = f || cableFrame(cb); if(!cf) return 0;
  return cf.totalUsed;
}

// ---- §06.4 · (slotFrame retired -- slot is now a two-endpoint constraint,
// built from the shared twoPointFrame/endpointAngleLockRow in §06.1, exactly
// like rod. See rowsFor's 'slot' branch below.) ----

// The rows one extra control point contributes to a LINE constraint (rod, slot, or
// a rack's jointed point). `f` is the constraint's twoPointFrame; `station` says
// whether the point also holds its place along the line (a rod's and a rack's do; a
// slot's riders slide).
//
// The point is held at  P_k = P_a + s*u,  with u the live a-b heading, so the rows
// are the two components of that in the line's own frame:
//
//   lateral      C = n . (P_k - P_a)          -- on the line
//   longitudinal C = u . (P_k - P_a) - s      -- at its station
//
// Differentiating picks up the line's own rotation, since u and n turn with it:
// du/dt = w*n and dn/dt = -w*u, where w = n.(v_a - v_b)/L is the bar's angular rate
// (the same dphi/dt endpointAngleLockRow uses). That is the third term in each row,
// and it is why the columns are a scaled combination of the endpoint closures rather
// than a plain difference -- one place scaleCols exists for.
// `unlocked` overrides the point's own rotation lock -- the pose-time release
// (§06.2d) makes every rider a pin, whatever the lock the rod holds it by when it
// is rigid again.
function linePointRows(con, f, pt, station, unlocked){
  const K=epFrame(pt.ep);
  const Dx=K.wx-f.wax, Dy=K.wy-f.way;
  const du=f.ux*Dx+f.uy*Dy, dn=f.nx*Dx+f.ny*Dy;
  // The bar's angular-rate columns, w = n.(v_a - v_b)/L.
  const wCols=mergeCols([ f.epA.velCols(f.nx/f.L, f.ny/f.L), f.epB.velCols(-f.nx/f.L, -f.ny/f.L) ]);
  const rows=[{
    cols: mergeCols([ K.velCols(f.nx,f.ny), f.epA.velCols(-f.nx,-f.ny), scaleCols(wCols,-du) ]),
    C: dn }];
  if(station) rows.push({
    cols: mergeCols([ K.velCols(f.ux,f.uy), f.epA.velCols(-f.ux,-f.uy), scaleCols(wCols, dn) ]),
    C: du-(pt.s||0) });
  if(pt.lock && !unlocked) rows.push(pointAngleLockRow(f, K, pt.restAng||0));
  return rows;
}

// ---- §06.5 · rowsFor (constraint -> rows dispatch) ----
// One branch per con.type; to reach a specific joint's row math, search its tag,
// e.g.  type==='rod'. Catalog (rows) -- cross-references spec §4:
//   pin            2   shared point coincident
//   rod            1   distance held along the connecting line; +1 per welded
//                      end (locks that end's body -- or the fixed world frame,
//                      for a background end -- to the rod's own direction).
//                      A `posable` rod RELEASED for a pose drag (§06.2d) drops
//                      all of that and keeps only its line: 0 rows for the pair,
//                      and one rider row per extra point, as a slot's.
//   slot           0   two pins = purely visual; +1 per "prismatic" end
//                      (locks that end to the segment direction, as rod's
//                      weld does); +1 more once BOTH ends are prismatic
//                      (kills lateral drift off the rail -- the classic
//                      point-on-line lock, giving a rigid prismatic joint)
//   belt           1   fixed phase ratio of two rim angles (holonomic)
//   knife          1   no-side-slip contact (NONHOLONOMIC, nh:true)
//   cvt            1   tangential match at a variable-radius contact (NONHOLONOMIC)
//   rack           0   a rack line named by two pins; +1 per welded pin (as rod's
//                      weld); +1 per meshing pinion (tangential match at the
//                      pinion's live pitch radius, NONHOLONOMIC)
// Every one of pin, rod, slot and rack may carry EXTRA CONTROL POINTS on top of the
// above (§06.2c): +2 per point on a pin or a rod, +1 on a slot, +2 on a rack's
// jointed point, and +1 more wherever that point is rotation-locked. Their rows are
// always appended after the base pair's, which is what lets §09.3 keep reading the
// pair's multipliers off fixed indices.
// (Cable rows are built inline in §08.2, not here, because they are unilateral.)
function rowsFor(con){
  // Each row carries the raw position error C (the value to drive to zero). The
  // velocity solver scales it by beta/h (Baumgarte); the position projection uses
  // C directly. Same Jacobian rows serve both.
  if(con.type==='dragpin'){
    // Internal-only: pins a point on A to a fixed world point. Never a user
    // constraint -- §13.6 feeds one through projectPositions as a transient
    // goal while the player drags a body around in edit mode. Marked `soft`
    // so projectPositions gives it extra compliance (§09.1): it's a UI pull,
    // not a real joint, and in any DOF a real constraint also has a say over
    // (dragging a rod's free end off the circle its weld allows, a slider off
    // its rail), the real joint should win outright rather than splitting the
    // difference and showing up as a violation on a mechanism that's actually
    // fully satisfied within its own reachable directions.
    const A = bodies[bodyIndex(con.a.id)];
    const ep = epFrame(con.a);
    return [
      { cols:ep.velCols(1,0), C: ep.wx-con.world[0], soft:true },
      { cols:ep.velCols(0,1), C: ep.wy-con.world[1], soft:true }
    ];
  }
  if(con.type==='pin'){
    const A=epFrame(con.a), B=epFrame(con.b);
    const Cx = A.wx-B.wx, Cy = A.wy-B.wy;
    const rows=[
      { cols: mergeCols([A.velCols(1,0), B.velCols(-1,0)]), C:Cx },
      { cols: mergeCols([A.velCols(0,1), B.velCols(0,-1)]), C:Cy }
    ];
    // Every extra point (§06.2c) is one more body brought to the same pivot: the
    // identical pair of rows, measured against end a. A three-armed hinge is three
    // endpoints on one pin, not two pins stacked at the same place.
    for(const pt of conPoints(con)){
      const K=epFrame(pt.ep);
      rows.push({ cols: mergeCols([K.velCols(1,0), A.velCols(-1,0)]), C:K.wx-A.wx });
      rows.push({ cols: mergeCols([K.velCols(0,1), A.velCols(0,-1)]), C:K.wy-A.wy });
    }
    return rows;
  }
  if(con.type==='rod'){
    // Either end may be background-anchored (id===null, off holds the world
    // point directly -- §06.1 epWorld).
    const f=twoPointFrame(con);
    if(rodReleased(con)){
      // Pose-time release (§06.2d): no distance row and no weld rows -- the pair is
      // free to slide apart and to turn -- and every extra point rides the line as a
      // slot's rider does, station and lock dropped. The pair itself needs no
      // point-on-line row: the line IS the segment between them, so such a row would
      // be the same tautology the slot's base pair avoids.
      const rows=[];
      for(const pt of conPoints(con)) rows.push(...linePointRows(con, f, pt, false, true));
      return rows;
    }
    const {ux,uy,L}=f;
    // d/dt|A-B| = u.(vA - vB): the two endpoints' velocity columns along the segment.
    const distCols=mergeCols([f.epA.velCols(ux,uy), f.epB.velCols(-ux,-uy)]);
    const rows=[{ cols:distCols, C:L-con.len }];
    // A welded end locks its body's angle (or, for a background end, the
    // fixed world frame) to the rod's own direction phi -- see
    // endpointAngleLockRow. phi is recomputed fresh each step (not
    // unwrap-tracked like the cable's spoolAngle), so a welded end that
    // spins through more than ~half a turn between steps can see its
    // Baumgarte bias jump -- fine for the intended use (fixed/rigid
    // attachments), not for a fast-spinning weld.
    if(con.weldA) rows.push(endpointAngleLockRow('A', f, con.restAngA));
    if(con.weldB) rows.push(endpointAngleLockRow('B', f, con.restAngB));
    // Extra control points (§06.2c) ride the bar as rigid attachments: on the line,
    // and at their own captured station along it. Two rows each -- see linePointRows
    // -- plus the same angle lock a welded end gets, and appended AFTER the base
    // rows so §09.3's row-order walk over the pair is untouched by them.
    for(const pt of conPoints(con)) rows.push(...linePointRows(con, f, pt, true));
    return rows;
  }
  if(con.type==='slot'){
    // A rail between two endpoints (either may be background-anchored, as
    // for rod). Unlike rod there is no base row: two pins is purely a
    // visual guide (0 rows). A locked ("prismatic") end adds the same
    // angle-lock row as rod's weld, pinning that end's frame to the segment
    // direction. Only once BOTH ends are locked do the two angle-locks pin
    // down a shared rail direction worth adding a third row for -- the
    // classic point-stays-on-rail lock (killing lateral drift), giving the
    // rigid prismatic joint. A single locked end therefore constrains
    // rotation only... except when that end is the background: a fixed
    // point whose angle to the other end is held constant *is* a fixed
    // positional rail (a ray from that point), with zero rotation lock on
    // the other end -- this is how a slider gets confined to a line
    // while still spinning freely (see makeSlotCon call sites, e.g. the
    // crank/integrator examples). That single-ended case has one caveat:
    // it locks phi = atan2(...) directly, which is singular if the live
    // endpoint ever passes through the fixed one -- keep the fixed
    // anchor well outside the slider's range of travel.
    const f=twoPointFrame(con);
    const rows=[];
    if(con.prismaticA) rows.push(endpointAngleLockRow('A', f, con.restAngA));
    if(con.prismaticB) rows.push(endpointAngleLockRow('B', f, con.restAngB));
    if(con.prismaticA && con.prismaticB){
      // Lateral lock: kill point A's drift off the rail, whose direction is
      // tracked live via B's frame (theta_B - restAngB) rather than
      // the raw A->B segment -- that segment is *always* perpendicular
      // to its own normal, so using it here would make this row a
      // tautology. Mirrors the old body-hosted slotFrame exactly (dDot is
      // the rail normal's own rotation rate, theta_B's contribution to
      // d/dt[n·D]).
      const {hasB,ib,wax,way,wbx,wby,epA,epB}=f;
      const railAngle=(hasB?f.B.th:0)-con.restAngB;
      const rdx=Math.cos(railAngle), rdy=Math.sin(railAngle);
      const rnx=-rdy, rny=rdx;
      const Dx=wax-wbx, Dy=way-wby;
      // d/dt(n_rail . D) = n_rail.(vA - vB) + (dn_rail/dt).D, and dn/dt = -w_B*d_rail,
      // so B's own angular column picks up -(d_rail . D). Same row as before, now
      // routed through the endpoint closures so a vessel end carries its len column.
      const dDot=rdx*Dx+rdy*Dy;
      const cols=mergeCols([
        epA.velCols(rnx,rny), epB.velCols(-rnx,-rny),
        hasB?[[ib,0,0,-dDot,0]]:[]
      ]);
      rows.push({ cols, C: rnx*Dx+rny*Dy });
    }
    // Extra control points (§06.2c) are RIDERS on the rail, not definitions of it:
    // each gets the point-on-line row unconditionally (that is what riding means)
    // and slides freely along, so unlike a rod's points it holds no station. The
    // lateral direction here is the live a->b segment's normal rather than a locked
    // end's railAngle -- the tautology that forced the base pair's row to use
    // railAngle does not arise, because a rider is a third point, not one of the two
    // the segment is drawn between.
    for(const pt of conPoints(con)) rows.push(...linePointRows(con, f, pt, false));
    return rows;
  }
  if(con.type==='belt'){
    // inextensible belt: rim tangential speeds equal -> fixed phase ratio (holonomic).
    // sense +1 open belt (same sense), -1 crossed.
    const A=bodies[bodyIndex(con.a.id)], B=bodies[bodyIndex(con.b.id)];
    const ia=bodyIndex(con.a.id), ib=bodyIndex(con.b.id), s=con.sense;
    const C=(con.rA*A.th - s*con.rB*B.th) - con.restPhase;
    return [{ cols:[[ia,0,0,con.rA],[ib,0,0,-s*con.rB]], C }];
  }
  if(con.type==='knife'){
    // no-side-slip (Chaplygin knife edge): the contact point's velocity across the
    // heading is zero. Velocity-only -- no position invariant (nonholonomic).
    const A=bodies[bodyIndex(con.a.id)];
    const hh=R(A.th,con.dir[0],con.dir[1]); const hl=Math.hypot(hh[0],hh[1])||1;
    const nx=-hh[1]/hl, ny=hh[0]/hl;                 // lateral normal to heading
    return [{ cols:epFrame(con.a).velCols(nx,ny), C:0, nh:true }];
  }
  if(con.type==='cvt'){
    // rolling contact at P = the point on A's rim nearest B. Match the two bodies'
    // tangential material-point velocities there. r_A is A's radius; B's arm is the
    // distance from B's centre to P, namely (d - r_A) -- a coordinate -> nonholonomic.
    const A=bodies[bodyIndex(con.a.id)], B=bodies[bodyIndex(con.b.id)];
    const ia=bodyIndex(con.a.id), ib=bodyIndex(con.b.id);
    let rvx=B.x-A.x, rvy=B.y-A.y; const d=Math.hypot(rvx,rvy)||1e-6;
    const ux=rvx/d, uy=rvy/d; const tx=-uy, ty=ux;   // tangent at contact
    const rA=A.r, armB=d-rA;
    return [{ cols:[[ia, tx, ty, rA],[ib, -tx, -ty, armB]], C:0, nh:true }];
  }
  if(con.type==='rack'){
    // The rack line, its two pins, and everything meshed with or jointed to it.
    const f=twoPointFrame(con);
    const rows=[];
    // A welded pin locks its body's angle to the rack's heading -- the same row a
    // rod's weld and a slot's prismatic lock build, against the same phi. With both
    // pins on one body the row is identically zero (the body already fixes phi), so
    // the arrangement costs nothing beyond a multiplier the regularizer zeroes.
    if(con.weldA) rows.push(endpointAngleLockRow('A', f, con.restAngA));
    if(con.weldB) rows.push(endpointAngleLockRow('B', f, con.restAngB));
    for(const pt of conPoints(con)){
      if(pt.kind!=='pinion'){
        // A jointed point is fixed to the rack exactly as a rod's extra point is
        // fixed to its bar: on the line, at its captured station (§06.2c).
        rows.push(...linePointRows(con, f, pt, true));
        continue;
      }
      // Rolling contact between the rack and this pinion at Q, the foot of the
      // perpendicular from the pinion's centre to the rack line: the two materials
      // in contact there must have the same velocity ALONG the rack.
      //
      // Rack side. The rack is rigid and pinned at end a, so its material velocity
      // along u is the SAME at every point of the line -- two points of a rigid body
      // a distance d apart along u differ by w x (d*u), which is perpendicular to u.
      // So end a's own velCols along u is the rack's tangential speed at the contact,
      // whatever the contact's station, and end b contributes nothing: it aims the
      // rack without locating it. A's angular column comes out of the same closure
      // every other endpoint row uses (it works out to -(r.n): a body's spin tells on
      // the rack only when the pin sits off the rack's own line through its centre).
      // Pinion side. Its material velocity at Q along u is vB.u + wB*rho, rho the
      // signed pitch radius -- the same derivation as the CVT's contact row, just
      // against a straight rack instead of a second rim.
      // rho is a live coordinate (it changes as either body moves, and as the rack
      // swings), so this row is NONHOLONOMIC exactly as the CVT's is.
      const g=rackPitch(f, pt); if(!g) continue;
      const cols=mergeCols([ f.epA.velCols(f.ux,f.uy), [[g.ib, -f.ux, -f.uy, -g.rho]] ]);
      rows.push({ cols, C:0, nh:true });
    }
    return rows;
  }
  return [];
}

// ---- §06.6 · spring / rotSpring frames ----
// Build a linear spring between two endpoints (same {id,off} shape as rod --
// either end may be background-anchored). Unlike a rod there is no weld: a
// spring only ever pulls/pushes along its own line, so twoPointFrame's phi
// (used by endpointAngleLockRow) is simply unused here. Rest length defaults
// to the current length, so a freshly-placed spring starts at equilibrium.
const SPRING_DEFAULT_K = 30;
function makeSpringCon(a,b){
  const [wax,way]=epWorld(a), [wbx,wby]=epWorld(b);
  return { type:'spring', a, b, restLen:Math.hypot(wax-wbx,way-wby), k:SPRING_DEFAULT_K, sel:false };
}
// World position of the draggable rest-length control point (constraints.js
// §06.6 / render.js §11.5): the midpoint of the live spring, offset
// perpendicular by a small screen-space gap (so the rest-length line reads as
// a separate parallel indicator, not an overlay on the spring itself), then
// out along the spring's own direction by half the rest length -- i.e. one
// end of the rest-length line, whose other end mirrors it through the centre.
const SPRING_LINE_OFFSET_PX = 14;
function springRestHandlePos(con){
  const [wax,way]=epWorld(con.a), [wbx,wby]=epWorld(con.b);
  const dx=wbx-wax, dy=wby-way, L=Math.hypot(dx,dy)||1e-9;
  const ux=dx/L, uy=dy/L, nx=-uy, ny=ux;
  const off=SPRING_LINE_OFFSET_PX/cam.scale;
  const cx=(wax+wbx)/2+nx*off, cy=(way+wby)/2+ny*off;
  return [cx+ux*con.restLen/2, cy+uy*con.restLen/2];
}

// Build a rotational (torsional) spring between two bodies -- 'a' and 'b' are
// bare {id} refs (no offset: like belt/cvt, the whole body's frame angle is
// the feature, not a point on it). Either may be background-anchored
// (id===null), which reads as a fixed theta=0 reference -- mirroring the
// null-id convention rod/slot use for a world-anchored end. The rest
// angle captures whatever the live relative angle is at creation, so a
// freshly-placed rotational spring starts unstressed (mirrors rod weld's
// captureRestAngle, §06.1).
const ROTSPRING_DEFAULT_K = 8;
function rotSpringRelAngle(rs){
  const thA = rs.a.id!=null ? bodies[bodyIndex(rs.a.id)].th : 0;
  const thB = rs.b.id!=null ? bodies[bodyIndex(rs.b.id)].th : 0;
  return thA-thB;
}
function makeRotSpringCon(aId,bId){
  const rs = { type:'rotspring', a:{id:aId}, b:{id:bId}, restAngle:0, k:ROTSPRING_DEFAULT_K, sel:false };
  rs.restAngle = rotSpringRelAngle(rs);
  return rs;
}
// The two theta=0 reference marks drawn on-canvas (render.js §11.5): with two
// real bodies, each mark rides its own body's local theta=0 point on its rim.
// With one end on the background, both marks sit on the *same* (real) body's
// rim -- one riding the body's own theta=0 (spins with it), the other held at
// the fixed world +x direction from that body's centre (the "ground's
// theta=0") -- so the pair visibly splays apart as the body twists away from
// its rest angle relative to the fixed frame.
function rotSpringControlPoints(con){
  const hasA=con.a.id!=null, hasB=con.b.id!=null;
  if(hasA && hasB){
    const A=bodies[bodyIndex(con.a.id)], B=bodies[bodyIndex(con.b.id)];
    const [ax,ay]=worldPt(A,bodyRimLocal(A)), [bx,by]=worldPt(B,bodyRimLocal(B));
    return {pA:[ax,ay], pB:[bx,by]};
  }
  const body = hasA ? bodies[bodyIndex(con.a.id)] : bodies[bodyIndex(con.b.id)];
  const [ox,oy]=worldPt(body,bodyRimLocal(body));
  const groundPt=[body.x+bodyExtentR(body), body.y];
  return hasA ? {pA:[ox,oy], pB:groundPt} : {pA:groundPt, pB:[ox,oy]};
}
// Belt vs. spiral: a belt reads as a connection between two separate rims, so
// it only makes sense while the two bodies' disks are not one fully inside
// the other (the standard "circle A contains circle B" test, distance between
// centres plus the smaller radius at most the larger radius). Full overlap,
// or either end on the background (no second rim to run a belt to at all),
// falls back to the spiral instead.
function rotSpringVisualMode(con){
  if(con.a.id==null || con.b.id==null) return 'spiral';
  const A=bodies[bodyIndex(con.a.id)], B=bodies[bodyIndex(con.b.id)];
  // The belt rendering is two tangent lines between two round rims -- only
  // meaningful when both ends are actually circles; a rectangle (or either
  // body missing) falls back to the spiral, same as the background-anchored
  // case above.
  if(A.shape==='rect' || B.shape==='rect') return 'spiral';
  const d=Math.hypot(A.x-B.x,A.y-B.y);
  const rMax=Math.max(A.r,B.r), rMin=Math.min(A.r,B.r);
  return (d+rMin<=rMax+1e-9) ? 'spiral' : 'belt';
}
// Geometry for the spiral render: centred on the larger body (or the sole
// real body, for a background-attached spring), sweeping from its own
// perimeter down to either the smaller body's perimeter or (background case)
// the centre. The sweep angle is a fixed decorative two turns plus the live
// deviation from rest, so a spring visibly winds up or loosens as the bodies
// twist relative to each other -- capped well short of the many-turn range so
// a fast spin doesn't wind the drawing into an unreadable knot.
function rotSpringSpiralGeom(con){
  const hasA=con.a.id!=null, hasB=con.b.id!=null;
  let outer, outerR, innerR;
  if(hasA && hasB){
    const A=bodies[bodyIndex(con.a.id)], B=bodies[bodyIndex(con.b.id)];
    const eA=bodyExtentR(A), eB=bodyExtentR(B);
    const outerIsA = eA>=eB;
    outer = outerIsA?A:B; outerR=outerIsA?eA:eB; innerR=outerIsA?eB:eA;
  } else {
    outer = hasA ? bodies[bodyIndex(con.a.id)] : bodies[bodyIndex(con.b.id)];
    outerR = bodyExtentR(outer); innerR = 0;
  }
  const dev = rotSpringRelAngle(con) - con.restAngle;
  const base = Math.PI*4;
  let sweep = base+dev;
  const mag = Math.max(Math.PI*1, Math.min(Math.PI*12, Math.abs(sweep)));
  sweep = mag*(sweep<0?-1:1);
  return {cx:outer.x, cy:outer.y, outerR, innerR, angle0:outer.th, sweep};
}
