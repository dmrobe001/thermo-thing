// ============================================================================
//  §06 · CONSTRAINT ROWS
//  The heart of the engine. Each constraint is turned into one or more rows of
//  the velocity-linear (Pfaffian) form  J·v = -bias  (spec §3.3). A row is
//    { cols:[[bodyIdx, jx, jy, jw], ...], C, nh? }
//  where C is the raw position error (the value to drive to zero) and nh flags
//  a nonholonomic row (velocity-only, no position invariant -- excluded from the
//  §09 position projection). The §08 solver scales C by beta/h (Baumgarte);
//  the §09 projection uses C directly. Same rows serve both.
//    §06.1  bodyIndex, epWorld, twoPointFrame, endpointAngleLockRow, and the
//           rod/slot constructors and endpoint-lock toggles built on them
//    §06.2  gasFrame   (cylinder geometry for the gas force element, §08.1/§08.5)
//    §06.2b gasStopRow (Jacobian/C for the gas's minimum-volume floor, consumed
//           as a standalone elastic impulse by physics.js §08.3, not a row here)
//    §06.3  cableFrame (tetherball tangent geometry for the unilateral cable)
//    §06.4  (retired -- see §06.1)
//    §06.5  rowsFor    (the dispatch: one branch per constraint type)
//    §06.6  spring / rotSpring frames (Hookean force elements, §08.1) -- these
//           are force elements, not constraint rows: they never appear in
//           rowsFor or the solve of §07/§08.3, only in the applied-force pass
//           of §08.1, mirroring gasFrame's role for the gas piston.
// ============================================================================

// ---- §06.1 · bodyIndex ----
function bodyIndex(id){ return bodies.findIndex(b=>b.id===id); }

// Resolve an endpoint {id, off} to its world point. id===null means the
// endpoint is fixed to the background (world-anchored) rather than riding a
// body -- off then holds the world coordinate directly, mirroring the
// null-id convention already used by gas heads and cable tethers. Shared by
// rod and slot, whose endpoints are both {id, off} pairs.
function epWorld(ep){
  if(ep.id==null) return [ep.off[0], ep.off[1], 0, 0];
  return worldPt(bodies[bodyIndex(ep.id)], ep.off);
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
function twoPointFrame(con){
  const hasA=con.a.id!=null, hasB=con.b.id!=null;
  const A = hasA ? bodies[bodyIndex(con.a.id)] : null;
  const B = hasB ? bodies[bodyIndex(con.b.id)] : null;
  const [wax,way,rax,ray] = hasA ? worldPt(A,con.a.off) : [con.a.off[0],con.a.off[1],0,0];
  const [wbx,wby,rbx,rby] = hasB ? worldPt(B,con.b.off) : [con.b.off[0],con.b.off[1],0,0];
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
  return {hasA,hasB,A,B,ia,ib,wax,way,rax,ray,wbx,wby,rbx,rby,ux,uy,nx,ny,L,phi};
}
// One row locking `which` end's frame angle -- a body's theta, or 0 for a
// background end -- to the live direction phi of the segment from B to A.
// Shared by rod's weld and slot's prismatic lock: both are the same
// operation (pin an endpoint's rotation to the line joining the two
// endpoints), just attached to different base constraints.
function endpointAngleLockRow(which, f, restAng){
  const {hasA,hasB,ia,ib,rax,ray,rbx,rby,nx,ny,L,phi} = f;
  const cols=[];
  if(which==='A'){
    if(hasA) cols.push([ia, -nx/L, -ny/L, 1+(nx*ray-ny*rax)/L]);
    if(hasB) cols.push([ib,  nx/L,  ny/L, -(nx*rby-ny*rbx)/L]);
  } else {
    if(hasA) cols.push([ia, -nx/L, -ny/L, -(ny*rax-nx*ray)/L]);
    if(hasB) cols.push([ib,  nx/L,  ny/L, 1+(ny*rbx-nx*rby)/L]);
  }
  const thHere = which==='A' ? (hasA?f.A.th:0) : (hasB?f.B.th:0);
  return { cols, C: thHere-phi-restAng };
}
// Capture (or recapture) endpoint `which`'s rest angle against the live A->B
// direction. Called whenever a per-endpoint lock (rod weld, slot prismatic)
// turns on, so toggling never snaps geometry.
function captureRestAngle(con, which){
  const [wax,way]=epWorld(con.a), [wbx,wby]=epWorld(con.b);
  const phi=Math.atan2(way-wby,wax-wbx);
  const ep = which==='A'?con.a:con.b;
  const th = ep.id!=null ? bodies[bodyIndex(ep.id)].th : 0;
  if(which==='A') con.restAngA=th-phi; else con.restAngB=th-phi;
}

// Build a rod constraint between two endpoints, deriving its rest length and
// (for any welded end) its rest angle.
function makeRodCon(a,b,weldA,weldB){
  const [wax,way]=epWorld(a), [wbx,wby]=epWorld(b);
  const con={type:'rod', a, b, len:Math.hypot(wax-wbx,way-wby), weldA:!!weldA, weldB:!!weldB, sel:false};
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
  const con={type:'slot', a, b, prismaticA:!!prismaticA, prismaticB:!!prismaticB, sel:false};
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

// ---- §06.2 · gasFrame ----
// gas vessel frame: a rectangle of width `bore` running along an axis fixed
// in the head body's frame (or the world, if head.id is null). The far
// (piston) face is either a real, movable body -- g.piston:{id,off}, exactly
// the {id,off} shape every other endpoint uses -- or, when g.piston is null,
// a fixed length g.len: a static vessel with no movable wall at all (the
// "two-corner rectangle" placement with neither corner free to move). x is
// the signed axial length; xc is x softened with a floor (GAS_MIN_X) purely
// so P=nT/V and V=bore*x never divide by/report zero -- it is a formula
// safety net, not what keeps the piston from actually reaching it. That job
// belongs to gasStopRow below: without a real geometric stop, a fast piston
// can cross GAS_MIN_X within a single substep, after which xc plateaus (P
// stops rising the way a real gas's would) while x keeps shrinking --
// decoupling the mechanism's real position/KE from the gas's own V/U
// bookkeeping and showing up as energy loss (physics.js §08.6's comment on
// the stop row has the full account). Everything else (drawGas, gasPolygon,
// the piston force) is unchanged by which case this is.
const GAS_MIN_X = 0.03;
function gasFrame(g){
  let hx,hy,dW,B=null,ib=-1,hrx=0,hry=0;
  if(g.head.id!=null){ ib=bodyIndex(g.head.id); B=bodies[ib];
    const [x,y,rx,ry]=worldPt(B,g.head.off); hx=x;hy=y;hrx=rx;hry=ry;
    dW=R(B.th,g.head.dir[0],g.head.dir[1]); }
  else { hx=g.head.off[0]; hy=g.head.off[1]; dW=[g.head.dir[0],g.head.dir[1]]; }
  const dl=Math.hypot(dW[0],dW[1])||1; dW=[dW[0]/dl,dW[1]/dl];
  let A=null, ia=-1, pax,pay,prx=0,pry=0,x;
  if(g.piston){ ia=bodyIndex(g.piston.id); A=bodies[ia];
    const [px,py,rx,ry]=worldPt(A,g.piston.off); pax=px;pay=py;prx=rx;pry=ry;
    x=(pax-hx)*dW[0]+(pay-hy)*dW[1];
  } else {
    x=g.len; pax=hx+dW[0]*x; pay=hy+dW[1]*x;
  }
  return {A,ia,pax,pay,prx,pry,B,ib,hrx,hry,hx,hy,dW,x,xc:Math.max(x,GAS_MIN_X)};
}
// Midpoint of a gas's rectangle -- a cheap stand-in for its centroid, used
// only for drawing/hit-testing heat/flow interactions (render.js §11.4c,
// tools.js §13.2), never for physics.
function gasCentroid(g){ const f=gasFrame(g); return [(f.hx+f.pax)/2,(f.hy+f.pay)/2]; }

// ---- §06.2b · gasStopRow ----
// Jacobian (cols) and position error (C=x-GAS_MIN_X) for the geometric floor
// that keeps a gas's *true* axial separation x from compressing past
// GAS_MIN_X, so gasFrame's own xc clamp above never actually has to bind in
// practice -- the geometric stop, not the softened pressure formula, is what
// arrests the piston. Same Jacobian shape as a rod's axial row (rowsFor's
// 'rod' branch below), built from gasFrame's own dW/prx,pry/hrx,hry instead
// of a freshly computed unit vector: the gas's hidden prismatic joint
// (DEVELOPMENT.md §6.1) already confines relative motion to that one axis,
// so a single row suffices here exactly as it does for the pressure force
// itself (physics.js §08.1). Only meaningful for a real movable piston -- a
// fixed-length vessel (g.piston===null) has no motion to stop, and callers
// gate on that before calling this.
//
// This is *not* pushed through the ordinary Schur-complement rows the way
// the cable's own end-stop (§06.3) is -- physics.js §08.3's comment where
// this is consumed has the reasoning: those rows all share one Baumgarte
// target that's wrong for an energy-conserving stop, so this row's cols/C
// instead feed a standalone elastic-reflection impulse there.
function gasStopRow(g){
  const f=gasFrame(g);
  const cols=[];
  if(f.A) cols.push([f.ia, f.dW[0], f.dW[1], -f.dW[0]*f.pry+f.dW[1]*f.prx]);
  if(f.B) cols.push([f.ib, -f.dW[0], -f.dW[1], f.dW[0]*f.hry-f.dW[1]*f.hrx]);
  return { cols, C: f.x-GAS_MIN_X };
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
    const [tx,ty,rx,ry]=worldPt(tb,cb.tether.off); T=[tx,ty]; trx=rx; tryy=ry; }
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
  let cols;
  if(tangentWins){
    // d2 floors away from 0 for a step that overshoots deep past the rim
    // (large angular rate near ell->0, see cableFrame's header) -- Lfree is
    // already 0 there, so the row is still the correct tangential direction,
    // just guarded against an actual division by a near-zero d.
    const d2 = Math.max(d*d, 1e-9);
    const Jx = (Dx*Lfree - rs*sign*Dy) / d2;
    const Jy = (Dy*Lfree + rs*sign*Dx) / d2;
    cols=[];
    if(!S.static) cols.push([is, -Jx, -Jy, -rs*sign]);
    if(tb&&!tb.static) cols.push([ti, Jx, Jy, Jx*(-tryy)+Jy*trx]);
  } else {
    cols=[];
    if(!S.static) cols.push([is, -ux, -uy, ux*ry_Q - uy*rx_Q]);
    if(tb&&!tb.static) cols.push([ti, ux, uy, -ux*tryy + uy*trx]);
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

// ---- §06.5 · rowsFor (constraint -> rows dispatch) ----
// One branch per con.type; to reach a specific joint's row math, search its tag,
// e.g.  type==='rod'. Catalog (rows) -- cross-references spec §4:
//   pin            2   shared point coincident
//   rod            1   distance held along the connecting line; +1 per welded
//                      end (locks that end's body -- or the fixed world frame,
//                      for a background end -- to the rod's own direction)
//   slot           0   two pins = purely visual; +1 per "prismatic" end
//                      (locks that end to the segment direction, as rod's
//                      weld does); +1 more once BOTH ends are prismatic
//                      (kills lateral drift off the rail -- the classic
//                      point-on-line lock, giving a rigid prismatic joint)
//   belt           1   fixed phase ratio of two rim angles (holonomic)
//   knife          1   no-side-slip contact (NONHOLONOMIC, nh:true)
//   cvt            1   tangential match at a variable-radius contact (NONHOLONOMIC)
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
    const [wax,way,rax,ray] = worldPt(A,con.a.off);
    const ia = bodyIndex(con.a.id);
    return [
      { cols:[[ia,1,0,-ray]], C: wax-con.world[0], soft:true },
      { cols:[[ia,0,1, rax]], C: way-con.world[1], soft:true }
    ];
  }
  if(con.type==='pin'){
    const A = bodies[bodyIndex(con.a.id)], B = bodies[bodyIndex(con.b.id)];
    const [wax,way,rax,ray] = worldPt(A,con.a.off);
    const [wbx,wby,rbx,rby] = worldPt(B,con.b.off);
    const Cx = wax-wbx, Cy = way-wby;
    const ia = bodyIndex(con.a.id), ib = bodyIndex(con.b.id);
    return [
      { cols:[[ia,1,0,-ray],[ib,-1,0, rby]], C:Cx },
      { cols:[[ia,0,1, rax],[ib,0,-1,-rbx]], C:Cy }
    ];
  }
  if(con.type==='rod'){
    // Either end may be background-anchored (id===null, off holds the world
    // point directly -- §06.1 epWorld) instead of riding a body.
    const f=twoPointFrame(con);
    const {hasA,hasB,ia,ib,rax,ray,rbx,rby,ux,uy,L}=f;
    const distCols=[];
    if(hasA) distCols.push([ia, ux, uy, -ux*ray+uy*rax]);
    if(hasB) distCols.push([ib,-ux,-uy,  ux*rby-uy*rbx]);
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
      const {hasA,hasB,ia,ib,rax,ray,rbx,rby,wax,way,wbx,wby}=f;
      const railAngle=(hasB?f.B.th:0)-con.restAngB;
      const rdx=Math.cos(railAngle), rdy=Math.sin(railAngle);
      const rnx=-rdy, rny=rdx;
      const Dx=wax-wbx, Dy=way-wby;
      const cols=[];
      if(hasA) cols.push([ia, rnx, rny, -rnx*ray+rny*rax]);
      if(hasB){ const dDot=rdx*Dx+rdy*Dy;
        cols.push([ib, -rnx, -rny, rnx*rby-rny*rbx-dDot]); }
      rows.push({ cols, C: rnx*Dx+rny*Dy });
    }
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
    const [px,py,rx,ry]=worldPt(A,con.a.off);
    const hh=R(A.th,con.dir[0],con.dir[1]); const hl=Math.hypot(hh[0],hh[1])||1;
    const nx=-hh[1]/hl, ny=hh[0]/hl;                 // lateral normal to heading
    const ia=bodyIndex(con.a.id);
    return [{ cols:[[ia, nx, ny, -nx*ry + ny*rx]], C:0, nh:true }];
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
// null-id convention rod/slot/gas use for a world-anchored end. The rest
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
