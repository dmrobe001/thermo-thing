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
//    §06.2b derived freezing (rodGrounds, rodLocksLength, refreshFrozen) and the
//           recaptures that follow a hand move (recaptureConAngles/recaptureConPose)
//    §06.2c extra control points (conPoints, makeConPoint, conEndpoints) -- the
//           third and further ends a pin/rod/slot/rack may carry
//    §06.2d posable joints (withPosing, conReleased, recapturePosable) -- the
//           pose-time release that turns a rod into a rail while it is dragged,
//           and takes a belt out of the way entirely
//    §06.2e the belt (beltFrame, beltSegments, beltRows, beltTension) -- a closed
//           loop of belting through an ordered list of wheels and eyelets
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
// which is what recaptureConPose does (§06.2b).
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

// A belt is a closed loop through an ordered list of nodes and has no two-ended
// base pair at all, so its constructor and everything it holds live together in
// §06.2e below -- search makeBeltCon there.
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
    recaptureConPose(con);
  }
}
// Re-read every ANGLE a line joint holds off the live geometry -- each locked end's
// rest angle (a rod's or a rack's weld, a slot's prismatic lock) and every extra
// point's (§06.2c) -- plus the unwrapping anchor those angles are measured against.
// This is exactly what captureRestAngle does when a lock is switched on, so a joint
// recaptured at a pose it already holds is unchanged.
//
// The unwrapping anchor goes first, and it is the subtle half. Every capture below
// reads the RAW segment angle (captureRestAngle), so the rows have to read it raw
// too, or a rod that was posed past the branch cut -- swung round its anchor through
// the -x direction -- comes out of the recapture holding a rest angle a full turn
// from the phi its own weld row measures against. Cleared, the next twoPointFrame
// re-seeds from the same raw atan2 the capture used, which is exactly the state a
// freshly loaded (or freshly Reset) scene is in. Safe wherever the joint is not
// being held across the edit -- compiled away, released for a drag (§06.2d), or
// carried bodily by a selection box (select.js §18.2) -- which is every caller here.
function recaptureConAngles(con){
  con._phiRef=undefined;
  // A belt holds no rest angle against a two-point line: what it holds is one
  // material constant per segment plus a locked eyelet's own angle, and beltRefresh
  // (§06.2e) re-reads all of it off the live path -- clearing the heading anchors as
  // it goes, for the same reason the line joints clear _phiRef.
  if(con.type==='belt'){ beltRefresh(con); return; }
  if(con.type==='rod' || con.type==='rack'){
    if(con.weldA) captureRestAngle(con,'A');
    if(con.weldB) captureRestAngle(con,'B');
  }
  if(con.type==='slot'){
    if(con.prismaticA) captureRestAngle(con,'A');
    if(con.prismaticB) captureRestAngle(con,'B');
  }
  for(const pt of conPoints(con)) if(pt.lock) pt.restAng=capturePointRestAngle(con, pt.ep);
}
// ...and everything else a line joint holds: the angles above plus the two things
// that are LENGTHS along it -- a rod's rest length and every extra point's station.
// Together that is exactly what makeRodCon and makeConPoint capture at creation, so
// a joint recaptured at a pose it already satisfies is unchanged, and one recaptured
// after a hand move holds the new pose instead. Called from recaptureGrounding
// above, once per pose-drag step from recapturePosable (§06.2d), and from a scaled
// selection box (select.js §18.2). Safe on any constraint kind: a belt, a CVT or a
// knife holds none of these and comes out untouched but for the phi anchor.
function recaptureConPose(con){
  recaptureConAngles(con);
  if(con.type==='rod'){
    const [wax,way]=epWorld(con.a), [wbx,wby]=epWorld(con.b);
    con.len=Math.hypot(wax-wbx,way-wby);
  }
  for(const pt of conPoints(con)) if(pt.s!==undefined) pt.s=capturePointStation(con, pt.ep);
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
// A belt is on this list too, and is the one kind where the points are not EXTRA:
// every node of the loop is one, and the belt has no base pair besides (§06.2e).
const CON_MULTI = ['pin','rod','slot','rack','belt'];
const conTakesPoints = con => CON_MULTI.includes(con.type);
const conPointHasStation = con => con.type==='rod' || con.type==='rack';
// A rack's pinions have no rotation lock and no station: they mesh wherever they sit,
// and neither does a belt's wheel, whose no-slip contact already ties it to the belt.
const conPointLockable = (con,pt) =>
  con.type==='belt' ? !beltIsWheel(pt)
  : conTakesPoints(con) && con.type!=='pin' && pt.kind!=='pinion';

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
  if(con.type==='belt'){
    // A belt's node (§06.2e). A WHEEL carries its wrap radius -- the body's own,
    // unless the file names another -- and which way round the belt passes it; an
    // EYELET carries whether it grips the belt and whether it turns its body with it.
    // The captured constants (restSeg, restAng) are the file's where the file gives
    // them and read off the geometry by beltRefresh where it does not, which is why
    // nothing is captured here: the loop is not complete until every node is on it.
    const wheel = o.kind==='wheel';
    const nd = wheel
      ? { ep:e, kind:'wheel', r: o.r!==undefined ? o.r : ((bodies[bodyIndex(e.id)]||{}).r || 0),
          wrap: o.wrap<0 ? -1 : 1 }
      : { ep:e, kind:'eyelet', tied: !!o.tied, lock: !!o.lock };
    if(o.mu!==undefined && beltGrips(nd)) nd.mu=o.mu;
    if(o.restAng!==undefined && nd.lock) nd.restAng=o.restAng;
    conPoints(con).push(nd);
    return nd;
  }
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
  if(con.type==='belt'){ setBeltLock(con, pt, val); return; }
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
  if(con.type==='belt') return false;      // a new eyelet routes the belt; it does not grip it
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
  constraints = constraints.filter(c => !c.a || (c.a.id!==id && !(c.b && c.b.id===id)));
  const shrank=new Set();
  for(const c of constraints){
    if(!c.pts || !c.pts.length) continue;
    const n=c.pts.length;
    c.pts = c.pts.filter(pt => pt.ep.id!==id);
    if(c.pts.length!==n) shrank.add(c);
  }
  // A belt is nothing but its nodes (§06.2e), so losing one is losing part of the
  // loop itself, not an attachment to it: the belt that is left takes a different
  // route, and what it holds is re-read against that route. A loop down to a single
  // node is no loop at all. Only the belts that actually lost a node are touched --
  // a recapture on one that did not would throw away an authored stress for nothing.
  constraints = constraints.filter(c => c.type!=='belt' || beltNodes(c).length>=2);
  for(const c of constraints) if(c.type==='belt' && shrank.has(c)) beltRefresh(c);
}

// ---- §06.2d · posable joints (the pose-time release) ----
// A rod or a belt may be marked `posable`. It changes nothing about the running
// physics -- a posable member is an ordinary rigid one at every substep -- and
// everything about what happens while the player POSES the machine: dragging a body
// around with the sim paused (tools.js §13.6). A posable joint is RELEASED for the
// length of that drag, and re-reads what it holds from the pose the drag leaves.
//
// A released BELT is simply not there: it contributes no rows and no tension, so
// every wheel it turns may be moved or spun freely and the belt re-reads its segment
// material -- and so its rest length -- from wherever the pose left them. That is the
// whole of it: a belt has no line to fall back to the way a rod does, and half a belt
// would be a worse answer than none.
//
// A released ROD holds only its own line:
//
//   * its distance row is gone, so the two ends may slide toward and away from each
//     other -- the rod's length is what the drag is free to change;
//   * its welds are gone, so every body it joins turns freely. That is the whole of
//     "a rail with all joined bodies PINNED, not welded";
//   * its extra control points (§06.2c) stop being rigid attachments at a station
//     and become RIDERS, held on the line and free to slide along it -- the same one
//     row a slot's riders get, which is the sense in which the bar becomes a rail.
//
// HOW FAR THE RELEASE REACHES is the one place the two differ, and the difference
// follows from what each of them becomes.
//
// A posable ROD is released only where it is DIRECTLY JOINTED to the dragged body --
// conEndpoints (§06.2c) and nothing cleverer: the rod names that body as one of its
// ends or as one of its extra points. A posable rod one joint further away stays a
// rigid rod, so the release reaches exactly as far as the hand does -- grab a body
// and the members it hangs off go slack, and the rest of the machine articulates
// around them as it always would. Releasing every posable rod in the scene for the
// duration of any drag would be both a bigger edit than the gesture asks for and one
// the player cannot see the extent of: a released rod is still a RAIL, and everything
// riding it rides it differently.
//
// A posable BELT is released for ANY pose drag, wherever the hand is. Nothing about
// the reach argument above survives the fact that a released belt is not a rail but
// an absence: releasing one whose wheels the drag cannot reach changes nothing at
// all, because its rows were satisfied and stay satisfied and the recapture re-reads
// geometry that did not move. What the narrow rule DID cost was the case a belt is
// usually in -- its wheels are not what you grab. Mount an idler on a swinging bar
// and drag the BAR, and under the rod's rule the belt is not released (it does not
// name the bar), so it holds its length rigidly and the bar does not move: a posable
// belt that refuses to be posed through. Released on any drag, the bar swings, the
// wheels follow, and the belt re-fits to the configuration the hand produced -- which
// is what marking it posable asked for.
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
const conPosableFor = (con, rootId) => !!con.posable && rootId!=null
  && (con.type==='belt' || conEndpoints(con).some(ep => ep.id===rootId));
// Released by the gesture in progress (what the canvas draws) ...
const conPosing = con => conPosableFor(con, posingRoot);
// ... and released right now, in the rows being built (what the solver sees).
const conReleased = con => posing>0 && conPosing(con);
// The same three, narrowed to the rod -- what the rod's own render, its freezing
// rules and tools/posable-check.js ask about.
const rodPosableFor = (con, rootId) => con.type==='rod' && conPosableFor(con, rootId);
const rodPosing = con => con.type==='rod' && conPosing(con);
const rodReleased = con => con.type==='rod' && conReleased(con);

// Once the drag step has settled, a released rod re-reads what it holds from the
// pose the player just produced (§06.2b recaptureConPose): the length, the welds'
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
    if(conPosing(con)) recaptureConPose(con);
}

// ---- §06.2e · the belt (a closed loop through an ordered list of nodes) ----
// A belt is a CLOSED LOOP of belting that runs through an ordered, cyclic list of
// NODES, and the nodes are the whole of it -- there is no base `a`/`b` pair, because
// no node of a loop is more the belt than any other. Every node lives in `con.pts`,
// the same extra-control-point list a pin, rod, slot or rack carries (§06.2c), and
// is built by the same one constructor (makeConPoint). Two kinds:
//
//   wheel   a disk the belt WRAPS, entering and leaving on tangent. It carries a
//           wrap radius `r` (the body's own, unless authored otherwise) and `wrap`,
//           which way round the disk the belt passes. In this collisionless flatland
//           the belt may cross back over itself freely, so `wrap` is a free choice
//           per wheel rather than one `crossed` flag for the whole belt -- an open
//           belt is every wheel the same way, a crossed one is a wheel flipped.
//   eyelet  a point the belt PASSES THROUGH, anchored to a body or to the fixed
//           background by an ordinary {id, off} endpoint. Untied, the belt slides
//           through it and it only bends the path -- which is how a belt is routed
//           somewhere its wheels would not take it, and how a tied body is made to
//           travel a chosen route. TIED, it grips the belt at one material point of
//           it, so the belt can no longer slide there and the eyelet's body is
//           carried along by the belt instead. LOCKED (welded), the body's own angle
//           follows the belt's local direction through the eyelet.
//
// A wheel and a tied eyelet are the two GRIPPING nodes: the belt's material is held
// at them, and they are what this constraint is about.
//
// BELT DISTANCE is the one idea the rest of this section is built on. Run a tape
// measure round the unstretched belting from an arbitrary mark on it and every point
// of the belt has a coordinate, `mu`, running 0 .. restLen and wrapping round. Each
// gripping node stores ONE number: the belt distance of the belting it holds.
//
//   tied eyelet   mu is exactly that -- how far round the belt this body is clamped.
//   wheel         mu is the same thing carried in the wheel's own frame, so that the
//                 belting at the contact is  mu + sigma*r*(psi - theta)  with psi the
//                 belt's heading there. Turn the wheel and the belting at the contact
//                 changes, which is what no-slip means; the constant does not.
//
// Both are the one expression beltMu below, since sigma*r is zero on an eyelet.
//
// The constants are ABSOLUTE, not gaps between neighbours, and that is the whole
// point of them. What the belt HOLDS is that consecutive grips are the right belting
// apart -- but WHICH grips are consecutive is not a fact about the belt, it is a fact
// about where everything currently is: a wheel the belting peels off stops being a
// grip, a tied eyelet carried round a wheel ends up between a different pair. Stored
// as gaps, every one of those events means re-deriving constants that did not
// physically change, and the belt loses whatever it was holding. Stored as belt
// distances, none of them touch the stored data at all: the pairing is derived, the
// constants are not. `restLen` is then the loop's own period -- how much belting
// there is -- and is a field of the belt rather than a sum over its nodes.
const beltNodes = con => conPoints(con);
const beltIsWheel = nd => nd.kind==='wheel';
// Whether this node holds the belt's material rather than letting it slide past.
const beltGrips = nd => nd.kind==='wheel' || !!nd.tied;
// The node's SIGNED wrap radius, sigma*r: the one number the rows and the geometry
// both read, and zero on an eyelet, which is what makes an eyelet a wheel of no
// radius everywhere below rather than a case of its own.
const beltSignedR = nd => nd.kind==='wheel' ? (nd.wrap<0?-1:1)*(nd.r||0) : 0;
const beltGripCount = con => beltNodes(con).filter(beltGrips).length;
// The belt distance of the belting AT this node, given the heading the belt has
// there -- a wheel's arrival and departure differ by its wrap arc, an eyelet's do
// not. `N` is a frame node (below), which carries the live sg and theta.
const beltMu = (N, psi) => (N.nd.mu||0) + N.sg*(psi - N.th);
const beltRestLen = con => con.restLen||0;

// The belt's live geometry, and the one place its path is worked out. Everything
// else -- the rows, the tension, the canvas, the hit test, the inspector -- reads
// this and nothing of its own.
//
// For each SPAN (one path node to the next, cyclically) the belt leaves the first
// node's rim on tangent and arrives on the second's on tangent, on each node's own
// wrap side. In terms of the span's heading psi, the outward radial at a tangency is
// sigma*rot(u,-90) = sigma*(sin psi, -cos psi), so with d the centre-to-centre vector
// and k the difference of the two signed radii, tangency is d.rot(u,-90) + k = 0 --
// one equation for psi, solved as psi = atan2(d) + asin(-k/|d|) with the branch that
// makes the span's length positive. The span's length is then sqrt(|d|^2 - k^2), the
// familiar external/internal tangent length, and an eyelet (no rim, k contribution
// zero) falls out of the same formula as a plain straight run to the point.
//
// psi is UNWRAPPED against the previous call's value (con._psi, transient scratch
// that restoreState clears -- §16.1) for the same reason twoPointFrame unwraps its
// phi: the rows measure sigma*r*psi against a captured constant, and a raw atan2
// jumping a full turn as a span swings through pointing along -x would land in the
// Baumgarte bias whole. Unwrapped, it also lets a belt genuinely wind more than once
// around a wheel and keeps the wrap arc counting the turns. The reference is keyed by
// the node the span LEAVES rather than by position along the path, so that a wheel
// dropping out of the path (below) leaves its neighbours' continuity untouched.
//
// The unwrapped headings are a LADDER of m+1 rungs over m path nodes, not a ring of
// m, and the extra rung is the whole of what makes a loop a loop. Traverse a closed
// belt once and the heading turns by a full 2*pi (more, if the belt winds), so no
// single branch of psi can serve every wrap: the span you started on is met again a
// turn further on. So the seed walks the ladder -- each rung taken to the branch that
// makes the wrap it completes turn the way that wheel's sigma says, which puts every
// wrap angle in [0, 2pi) -- and then takes the FIRST span a second time, as the top
// rung, for the wrap that closes the loop. A node's arrival heading is the rung below
// it and its departure heading the rung above, and the first node reads both off the
// top of the ladder. Nothing downstream needs the two copies of that span to agree:
// every place psi is used it appears once as some node's arrival and once as some
// node's departure, and what is between them is that node's own wrap.
function beltPath(con, nds, live){
  const idx = [];
  for(let i=0;i<nds.length;i++) if(live[i]) idx.push(i);
  const m = idx.length;
  if(m < 2) return null;
  const nodes = idx.map((i,j) => {
    const nd = nds[i], epf = epFrame(nd.ep);
    return { i, j, nd, epf, wx:epf.wx, wy:epf.wy, th:epf.th, sg:beltSignedR(nd) };
  });
  const ref = con._psi || (con._psi = {});
  const raw = new Array(m), spans = new Array(m);
  for(let j=0;j<m;j++){
    const a=nodes[j], b=nodes[(j+1)%m];
    const dx=b.wx-a.wx, dy=b.wy-a.wy;
    const D=Math.hypot(dx,dy), k=b.sg-a.sg;
    const sn = D>1e-9 ? Math.max(-1,Math.min(1,-k/D)) : 0;
    raw[j]=Math.atan2(dy,dx)+Math.asin(sn);
    spans[j]={ ia:j, ib:(j+1)%m, L:Math.sqrt(Math.max(0, D*D-k*k)) };
  }
  // The ladder: rung j is the heading of the span node j leaves on, and the top rung
  // is the first span again, as the first node leaves on it a full traversal later.
  const lad=new Array(m+1);
  for(let j=0;j<=m;j++){
    const r=raw[j%m], key = j<m ? nodes[j].i : 'close';
    lad[j] = ref[key]!==undefined ? unwrapNear(r, ref[key])
           : j===0 ? r
           : beltSeedPsi(r, lad[j-1], nodes[j%m].sg, nodes[j%m].nd);
    ref[key]=lad[j];
  }
  for(let j=0;j<m;j++){
    const p=lad[j], ux=Math.cos(p), uy=Math.sin(p);
    const rx=uy, ry=-ux;                                  // rot(u,-90): the +sigma radial
    const s=spans[j], a=nodes[s.ia], b=nodes[s.ib];
    s.psi=p; s.ux=ux; s.uy=uy; s.nx=-uy; s.ny=ux;
    s.Dx=a.wx+a.sg*rx; s.Dy=a.wy+a.sg*ry;                 // leaves this node here
    s.Ax=b.wx+b.sg*rx; s.Ay=b.wy+b.sg*ry;                 // arrives at the next one here
  }
  let straight=0, arcs=0;
  for(const s of spans) straight+=s.L;
  for(let j=0;j<m;j++){
    nodes[j].inPsi  = j===0 ? lad[m-1] : lad[j-1];
    nodes[j].outPsi = j===0 ? lad[m]   : lad[j];
    nodes[j].alpha = beltIsWheel(nodes[j].nd)
      ? (nodes[j].sg<0?-1:1)*(nodes[j].outPsi-nodes[j].inPsi) : 0;
    arcs += Math.abs(nodes[j].sg)*nodes[j].alpha;
  }
  return { con, nodes, spans, psi:lad, straight, arcs, length:straight+arcs };
}
// ...and which nodes the belting is actually ON. A wheel is only in the path while
// the belt WRAPS it, and belting can only bear on a wheel from outside: the wrap
// angle is a length of contact, and a negative one is the belt cutting through the
// disk rather than lying on it. So a wheel whose wrap would come out negative has
// been PEELED OFF -- the belting runs straight past it and it turns free.
//
// This is not an approximation bolted on. It is exactly continuous: at the moment a
// wrap reaches zero the wheel's arrival and departure tangent points coincide AND
// its two spans are the same tangent line, so the three points are collinear and
// dropping the wheel changes neither the path, its length, nor the belting held
// between the grips on either side. Peeling off and re-seating are both silent.
//
// Resolved by a fixed point rather than one pass, because dropping one wheel moves
// its neighbours' tangents and can peel another; only DROPS are taken in a pass, so
// it terminates, and re-seating is re-decided from scratch on the next call.
//
// A peeled wheel KEEPS its heading anchor, and that is not housekeeping -- it is what
// makes the test mean anything. The wrap is only negative relative to the branch the
// anchor picks; clear the anchor and the next call re-seeds the wrap into [0, 2pi)
// (beltSeedPsi), reads a hair under a full turn instead of a hair below zero, and
// seats the wheel straight back on. So the anchor is kept, and while the wheel is off
// the path it is carried along by the belting that passes it, so that whenever the
// belt does come back the two agree on which turn they are on.
function beltFrame(con){
  const nds = beltNodes(con), n = nds.length;
  if(n < 2) return null;
  const live = nds.map(()=>true);
  let f=null;
  for(let pass=0; pass<=n; pass++){
    f = beltPath(con, nds, live);
    if(!f) return null;
    let dropped=false, m=f.nodes.length;
    for(const N of f.nodes){
      if(m<=2) break;                         // a loop needs two points; never below
      if(beltIsWheel(N.nd) && N.alpha < 0){ live[N.i]=false; m--; dropped=true; }
    }
    if(!dropped) break;
  }
  f.live = live;
  f.at = new Array(n); for(const N of f.nodes) f.at[N.i]=N;
  // Carry each peeled wheel's anchor on the belting that now runs past it: the span
  // leaving the nearest node still on the path before it.
  for(let i=0;i<n;i++){
    if(live[i]) continue;
    // f.psi[N.j], not N.outPsi: the first path node's DEPARTURE is the ladder's top
    // rung, a whole traversal above the span's own heading, and anchoring to that
    // would leave the wheel a full turn out when it tried to seat again.
    for(let k=1;k<=n;k++){ const N=f.at[(i-k+n)%n];
      if(N){ con._psi[i]=f.psi[N.j]; break; } }
  }
  return f;
}
// Unwrap `a` to the branch nearest `ref` -- twoPointFrame's own move (§06.1), pulled
// out because the belt does it once per span.
function unwrapNear(a, ref){
  let d=a-ref;
  while(d> Math.PI) d-=Math.PI*2;
  while(d<-Math.PI) d+=Math.PI*2;
  return ref+d;
}
// The seed branch for one span's heading, given the previous span's: on a wheel, the
// one that makes the wrap this node completes turn the way its sigma says, so the
// wrap angle seeds into [0, 2pi); on an eyelet (nothing wrapped) simply the nearest,
// since there is no arc for the choice to be about.
function beltSeedPsi(p, prev, sg, nd){
  if(!beltIsWheel(nd)) return unwrapNear(p, prev);
  const s = sg<0 ? -1 : 1;
  let a = s*(p-prev);
  a = a % (Math.PI*2); if(a<0) a += Math.PI*2;
  return prev + s*a;
}
// The belt's total path length -- straight runs plus wrapped arcs.
function beltLength(con, f){ const g=f||beltFrame(con); return g?g.length:0; }

// ---- what the belt HOLDS -----------------------------------------------------
// Consecutive GRIPS along the path -- wheels the belting is on, and tied eyelets --
// with the belting that runs between them. One PAIR each, and one row each:
//
//     (belt distance where the belting meets the far grip)
//   - (belt distance where it left the near one)
//   = (the straight belting between them)
//
// plus one whole restLen on the pair that closes the loop, since belt distance is
// only defined round the loop. Reading the belt's material coordinate along the
// loop, a wheel's no-slip contact pins its rate at the rim to sigma*r*(dpsi/dt -
// omega) and an untied eyelet pins nothing (the belt slides through), so writing the
// row out and differencing across the pair, every dpsi/dt term cancels against the
// tangent points' own drift and what is left is entirely local and velocity-linear:
//
//     sigma_p r_p omega_p - sigma_q r_q omega_q  =  sum over the pair's spans of
//                                                     u . (v_next - v_this)
//
// with v the nodes' own anchor velocities (a wheel's centre, an eyelet's point). It
// is a material balance: what the near wheel feeds in, less what the far wheel takes
// out, is what the belting between them lengthens by. Summed over the loop the belt
// distances telescope into the wrapped arcs, so the whole set says nothing more nor
// less than L = restLen -- which is why the rest length is a field and not a row.
function beltPairs(con, f){
  const g=[]; for(const N of f.nodes) if(beltGrips(N.nd)) g.push(N.j);
  const m=f.nodes.length, out=[];
  if(!g.length) return out;                    // no grip: the loop closes on itself
  for(let k=0;k<g.length;k++){
    const pj=g[k], qj=g[(k+1)%g.length], spans=[];
    for(let j=pj; ; j=(j+1)%m){ spans.push(j); if((j+1)%m===qj) break; }
    // The pair that closes the loop is the one whose spans wrap past the start; with
    // one grip only, its single pair goes all the way round and closes.
    out.push({ p:pj, q:qj, spans, closes: k===g.length-1 });
  }
  return out;
}
// One pair's row: the material balance above, as columns and as C.
function beltPairRow(con, f, pr){
  const parts=[];
  let L=0;
  for(const sj of pr.spans){ const s=f.spans[sj];
    parts.push(f.nodes[s.ia].epf.velCols(s.ux, s.uy));
    parts.push(f.nodes[s.ib].epf.velCols(-s.ux, -s.uy));
    L+=s.L;
  }
  const P=f.nodes[pr.p], Q=f.nodes[pr.q];
  // p and q are the SAME node on a loop with one grip; mergeCols sums the two
  // angular columns to nothing there, which is the right answer -- a belt with one
  // wheel in it has no second grip to hold a ratio against.
  parts.push(scaleCols(P.epf.angCols(),  P.sg));
  parts.push(scaleCols(Q.epf.angCols(), -Q.sg));
  const C = beltMu(Q, Q.inPsi) + (pr.closes ? beltRestLen(con) : 0)
          - beltMu(P, P.outPsi) - L;
  return { cols: mergeCols(parts), C };
}
// The one row a belt with NO gripping node at all has: nothing holds its material, so
// all it can say is that the loop is as long as it is.
function beltLoopRow(con, f){
  const parts=[], m=f.nodes.length;
  for(let j=0;j<m;j++){
    const sIn=f.spans[(j-1+m)%m], sOut=f.spans[j];
    parts.push(f.nodes[j].epf.velCols(sIn.ux-sOut.ux, sIn.uy-sOut.uy));
  }
  return { cols: mergeCols(parts), C: f.length - beltRestLen(con) };
}
// One locked eyelet's row: its body's angle held to the belt's LOCAL DIRECTION there,
// which is the bisector of the headings the belt arrives and leaves on -- the two
// agree wherever the belt runs straight through, and the bisector is the only choice
// that treats the two sides alike where it does not. d(psi)/dt of a span is
// n.(v_next - v_this)/L (the tangent points' own drift along n is zero), so the row
// is the endpoint's angular column less half of each neighbouring span's turn rate.
// This is pointAngleLockRow's counterpart for a joint whose "line" is a path.
function beltLockRow(con, f, j){
  const m=f.nodes.length, K=f.nodes[j];
  const parts=[K.epf.angCols()];
  for(const sj of [(j-1+m)%m, j]){
    const s=f.spans[sj], L=Math.max(s.L, 1e-9);
    parts.push(scaleCols(mergeCols([ f.nodes[s.ib].epf.velCols(s.nx, s.ny),
                                     f.nodes[s.ia].epf.velCols(-s.nx, -s.ny) ]), -0.5/L));
  }
  return { cols: mergeCols(parts), C: K.th - beltNodeAngle(f, j) - (K.nd.restAng||0) };
}
// The belt's local direction at a path node: the bisector of the two span headings.
// The difference is taken WRAPPED, which makes the answer independent of which copy
// of a span's heading the ladder handed over -- the two differ by whole turns, and a
// whole turn is exactly what the wrap discards. The cost is a branch cut where the
// belt doubles back on itself through an eyelet, folded flat: a locked eyelet wants
// belting that runs on through it, which is what an eyelet is for.
function beltNodeAngle(f, j){
  const m=f.nodes.length, a=f.psi[(j-1+m)%m], b=f.psi[j];
  let d=b-a;
  while(d> Math.PI) d-=Math.PI*2;
  while(d<-Math.PI) d+=Math.PI*2;
  return a + d/2;
}
// The rows the belt contributes (§06.5 dispatches here).
//
//   softness 0   every pair row, holonomic. Their sum is the whole loop's
//                L = restLen, so an inextensible belt needs nothing added.
//   softness > 0 the DIFFERENCES of adjacent pair rows -- one fewer row, saying
//                that whatever the belt has stretched, it has stretched by the same
//                amount everywhere. The length itself is then not held at all: it is
//                carried by the tension force element of §08.1, which is what makes
//                softness a modulus rather than a solver fudge. The no-slip ratios
//                the differences hold are exact either way.
//
// Plus one row per locked eyelet, and none of it at all while the belt is RELEASED
// for a pose drag (§06.2d).
function beltRows(con){
  if(conReleased(con)) return [];
  const f=beltFrame(con); if(!f) return [];
  const rows=[];
  const prs=beltPairs(con,f);
  const raw = prs.length ? prs.map(pr=>beltPairRow(con, f, pr)) : [beltLoopRow(con, f)];
  if(!(con.soft>0)) rows.push(...raw);
  else for(let j=0;j+1<raw.length;j++)
    rows.push({ cols: mergeCols([raw[j].cols, scaleCols(raw[j+1].cols,-1)]),
                C: raw[j].C - raw[j+1].C });
  for(const N of f.nodes) if(!beltIsWheel(N.nd) && N.nd.lock) rows.push(beltLockRow(con, f, N.j));
  return rows;
}
// The tension each stretch of belting is carrying, read off the solve -- one number
// per pair, in beltPairs order, positive when that belting is taut. A pair row's own
// multiplier IS its tension (its columns are the pull the tension applies, so
// lambda/h is the force), which is what makes a belt's instrumentation free in the
// same way every other joint's is (§09.3).
//
// A SOFT belt's rows are the DIFFERENCES of adjacent pairs (beltRows), so their
// multipliers are tension differences: undo the differencing by a running sum, and
// add the elastic tension every stretch carries alike.
function beltTensions(con){
  const f=beltFrame(con);
  const n = f ? Math.max(1, beltPairs(con,f).length) : 1;
  const l=con._lam||[], h=sim.h;
  const out=new Array(n).fill(0);
  if(!(con.soft>0)){ for(let j=0;j<n;j++) out[j]=(l[j]||0)/h; return out; }
  const T=beltTension(con, f);
  let prev=0;
  for(let j=0;j<n;j++){ const mu = j<n-1 ? (l[j]||0) : 0; out[j]=T+(mu-prev)/h; prev=mu; }
  return out;
}
// The belt's tension, as a force element (§08.1) -- the whole of what `soft` buys.
// soft is a COMPLIANCE, the reciprocal of the belt's elastic modulus: T = stretch /
// soft, so soft = 0 is the inextensible belt (held by the rows above instead) and a
// bigger soft is a slacker belt. Tension only: a belt shorter than its rest length is
// slack, and slack belting pushes nothing.
function beltTension(con, f){
  if(!(con.soft>0) || conReleased(con)) return 0;
  const g=f||beltFrame(con); if(!g) return 0;
  return Math.max(0, g.length - beltRestLen(con)) / con.soft;
}
// ...and the potential that tension is the gradient of, for the energy ledger
// (§12.1) and the per-island conservation target (§08.6). Same shape as a spring's.
function beltEnergy(con){
  if(!(con.soft>0)) return 0;
  const e=Math.max(0, beltLength(con) - beltRestLen(con));
  return 0.5*e*e/con.soft;
}
// Each node's share of a unit pull around the loop: d(L)/dt = sum over nodes of
// v_node . (u_in - u_out), so a tension T pulls each node's anchor by T*(u_out-u_in).
// The wheels' spins do not appear -- turning a wheel does not lengthen the path --
// which is exactly why a soft belt transmits its torque through the no-slip rows
// above and not through this force.
function beltPullCols(f, T){
  const parts=[], m=f.nodes.length;
  for(let j=0;j<m;j++){
    const sIn=f.spans[(j-1+m)%m], sOut=f.spans[j];
    parts.push(f.nodes[j].epf.velCols(T*(sOut.ux-sIn.ux), T*(sOut.uy-sIn.uy)));
  }
  return mergeCols(parts);
}

// ---- building and recapturing a belt ----------------------------------------
// THE constructor (SCENE.md §S.2). `nodes` is the ordered loop: each entry an
// endpoint plus what that node is, in makeConPoint's own vocabulary. A freshly built
// belt is unstressed -- every belt distance, and so the rest length, is read off the
// geometry it was built at.
function makeBeltCon(nodes, opts){
  const o=opts||{};
  const con={type:'belt', pts:[], soft:o.soft||0, posable:!!o.posable, restLen:0, sel:false};
  for(const nd of nodes) makeConPoint(con, nd, nd);
  beltRefresh(con, true);
  return con;
}
// Re-read what the belt holds off its live path: every grip's belt distance, the rest
// length that is the loop's own period, and each locked eyelet's rest angle. With
// `fill` set the file's own captured values stand and only what is MISSING is read
// off the geometry (§17.4) -- and because belt distances are one measurement round
// one loop, they are missing together or not at all: a line that names some and not
// others gets the whole loop fitted to the machine as drawn, which is also what makes
// a terse hand-written belt (nodes and nothing else) legal and mean the obvious thing.
//
// The heading anchors go first, and the captures follow them raw, for exactly the
// reason recaptureConAngles gives (§06.2b): a constant captured against an unwrapped
// heading would disagree by a whole turn with the rows after the next Reset re-seeded
// that heading from a fresh atan2.
function beltRefresh(con, fill){
  if(!fill) con._psi=undefined;
  const f=beltFrame(con); if(!f) return;
  const grips=f.nodes.filter(N=>beltGrips(N.nd));
  const haveAll = grips.length && grips.every(N=>N.nd.mu!==undefined) && con.restLen>0;
  if(!(fill && haveAll)){
    // One walk round the loop with the tape measure: the first grip's arrival is the
    // origin, every span adds its straight run, every wheel its arc, and coming back
    // to the origin gives the loop's period.
    let mu=0;
    if(grips.length){
      const m=f.nodes.length;
      let j=grips[0].j;
      grips[0].nd.mu = -grips[0].sg*(grips[0].inPsi - grips[0].th);
      for(let step=0; step<m; step++){
        const N=f.nodes[j];
        mu += Math.abs(N.sg)*N.alpha;                 // across this node's own arc
        mu += f.spans[j].L;                           // and along the span it leaves on
        j=(j+1)%m;
        const K=f.nodes[j];
        if(K.j===grips[0].j) break;
        if(beltGrips(K.nd)) K.nd.mu = mu - K.sg*(K.inPsi - K.th);
      }
      con.restLen = mu;
    } else {
      con.restLen = f.length;
    }
  }
  for(const N of f.nodes){
    if(beltIsWheel(N.nd) || !N.nd.lock) continue;
    if(fill && N.nd.restAng!==undefined) continue;
    N.nd.restAng = N.th - beltNodeAngle(f, N.j);
  }
}
// Set the belt's rest length: there is more (or less) belting, and it is the WHOLE
// belt that is longer, so every grip's belt distance scales with the loop's period.
// Lengthen a belt and the slack appears evenly all round rather than parked in one
// span, and the wheels' phases relative to one another are undisturbed.
function beltSetRestLen(con, v){
  const now=beltRestLen(con);
  if(!(v>0) || !(now>0)){ con.restLen=Math.max(1e-9, v); return; }
  const k=v/now, f=beltFrame(con), nds=beltNodes(con);
  for(let i=0;i<nds.length;i++){
    const nd=nds[i];
    if(!beltGrips(nd) || nd.mu===undefined) continue;
    const N = f && f.at[i];
    if(N) nd.mu = k*beltMu(N, N.inPsi) - N.sg*(N.inPsi - N.th);
    else   nd.mu = k*nd.mu;                          // peeled off: no contact to read
  }
  con.restLen=v;
}
// A wheel the belting has peeled off is not gripping anything, so there is no grip
// point for it to hold: its belt distance follows whatever belting is passing nearest
// to it, which is what makes it seat again silently wherever the belt comes back.
// Called once per substep (physics.js §08.1), the belt's counterpart of the cable's
// slack-time bookkeeping (CABLE.md §C.5) -- and, like it, the reason this constant is
// something a RUN can change and so is in the Reset snapshot (§17.6).
//
// The wheel re-seats in phase but not necessarily at speed: it has been turning free
// while the belt was off it, and the rows put it back on the belt's own rim speed the
// step it returns. That is a clutch, and a real one dissipates; this engine has no
// dissipation channel, so §08.6 holds the ledger flat across it instead. Brief
// peel-offs (the common case, a belt shaken off and back) barely show it.
function beltSettle(con){
  const f=beltFrame(con); if(!f) return;
  const nds=beltNodes(con), m=f.nodes.length;
  const prs=beltPairs(con,f);
  if(!prs.length) return;
  for(let i=0;i<nds.length;i++){
    const nd=nds[i];
    if(!beltIsWheel(nd) || f.at[i]) continue;         // on the path: it holds its own
    // Walk the pairs for the span that passes nearest this wheel's centre, carrying
    // the belt distance along with us, and read the distance off at the closest point.
    const [cx,cy]=epWorld(nd.ep);
    let best=null;
    for(const pr of prs){
      let mu=beltMu(f.nodes[pr.p], f.nodes[pr.p].outPsi);
      for(const sj of pr.spans){
        const s=f.spans[sj];
        const dx=s.Ax-s.Dx, dy=s.Ay-s.Dy, L2=dx*dx+dy*dy;
        const t = L2>0 ? Math.max(0, Math.min(1, ((cx-s.Dx)*dx+(cy-s.Dy)*dy)/L2)) : 0;
        const d = Math.hypot(cx-(s.Dx+t*dx), cy-(s.Dy+t*dy));
        if(!best || d<best.d) best={d, mu: mu + t*s.L, psi:s.psi};
        mu += s.L;
      }
    }
    if(best) nd.mu = best.mu - beltSignedR(nd)*(best.psi - epFrame(nd.ep).th);
  }
}
// Set (or clear) a node's own flags, recapturing what that changes. Toggling `tied`
// gives a node a belt distance it did not have (or takes one away), so it is read off
// the live geometry; toggling `wrap` re-routes the belt around that wheel, which is a
// different loop, so the whole tape measure is run again. Each is the belt's
// counterpart of setRodWeld.
function setBeltWrap(con, nd, v){ if(!beltIsWheel(nd)) return; nd.wrap = v<0?-1:1; beltRefresh(con); }
function toggleBeltWrap(con, nd){ setBeltWrap(con, nd, beltSignedR(nd)<0 ? 1 : -1); }
function setBeltTied(con, nd, v){
  if(beltIsWheel(nd)) return;
  nd.tied=!!v;
  if(!nd.tied){ delete nd.mu; return; }
  // A grip added to a loop that already has some reads its own belt distance off the
  // belting it is on, leaving every other grip's alone -- which is the whole payoff
  // of storing distances rather than gaps.
  const f=beltFrame(con); if(!f) return;
  const N=f.at[beltNodes(con).indexOf(nd)];
  nd.mu = N ? beltDistanceAt(con, f, N) : 0;
}
function setBeltLock(con, nd, v){
  if(beltIsWheel(nd)) return;
  nd.lock=!!v;
  if(!nd.lock){ delete nd.restAng; return; }
  const f=beltFrame(con); if(!f) return;
  const N=f.at[beltNodes(con).indexOf(nd)];
  if(N) nd.restAng = N.th - beltNodeAngle(f, N.j);
}
// The belt distance of the belting where it meets path node N, worked out by walking
// back to the nearest grip before it and adding the belting in between. The reading a
// node takes when it starts gripping -- and every other grip's own reading is left
// exactly as it was, which is the whole payoff of storing distances rather than gaps.
function beltDistanceAt(con, f, N){
  const m=f.nodes.length;
  let j=N.j, back=0;
  for(let step=0; step<m; step++){
    const prev=(j-1+m)%m;
    back += f.spans[prev].L;
    if(beltGrips(f.nodes[prev].nd)) return beltMu(f.nodes[prev], f.nodes[prev].outPsi) + back;
    j=prev;                       // an untied eyelet bends the belting, adds none
  }
  return 0;
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
//   belt           n   one material-balance row per SEGMENT of the loop -- what the
//                      wheel at one end feeds in, less what the wheel at the other
//                      takes out, is what that stretch of belting lengthens by
//                      (holonomic; §06.2e). Their sum is the whole loop's
//                      inextensibility, so an n-segment belt holds n-1 speed ratios
//                      and its own length. A SOFT belt drops to the n-1 differences
//                      and carries its length as tension instead (§08.1). +1 per
//                      welded eyelet (its body turns with the belt).
//   knife          1   no-side-slip contact (NONHOLONOMIC, nh:true)
//   cvt            1   tangential match at a variable-radius contact (NONHOLONOMIC)
//   rack           0   a rack line named by two pins; +1 per welded pin (as rod's
//                      weld); +1 per meshing pinion (tangential match at the
//                      pinion's live pitch radius, NONHOLONOMIC)
// Every one of pin, rod, slot and rack may carry EXTRA CONTROL POINTS on top of the
// above (§06.2c): +2 per point on a pin or a rod, +1 on a slot, +2 on a rack's
// jointed point, and +1 more wherever that point is rotation-locked. Their rows are
// always appended after the base pair's, which is what lets §09.3 keep reading the
// pair's multipliers off fixed indices. A belt is the same list with no base pair
// under it: its rows are its points' rows and nothing else (§06.2e).
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
  if(con.type==='belt') return beltRows(con);
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
