// ============================================================================
//  §06 · CONSTRAINT ROWS
//  The heart of the engine. Each constraint is turned into one or more rows of
//  the velocity-linear (Pfaffian) form  J·v = -bias  (spec §3.3). A row is
//    { cols:[[bodyIdx, jx, jy, jw], ...], C, nh? }
//  where C is the raw position error (the value to drive to zero) and nh flags
//  a nonholonomic row (velocity-only, no position invariant — excluded from the
//  §09 position projection). The §08 solver scales C by beta/h (Baumgarte);
//  the §09 projection uses C directly. Same rows serve both.
//    §06.1  bodyIndex (id -> array slot)
//    §06.2  gasFrame   (cylinder geometry for the gas force element, §08.1/§08.5)
//    §06.3  cableFrame (tetherball tangent geometry for the unilateral cable)
//    §06.4  slotFrame  (point-on-line rail geometry)
//    §06.5  rowsFor    (the dispatch: one branch per constraint type)
// ============================================================================

// ---- §06.1 · bodyIndex ----
function bodyIndex(id){ return bodies.findIndex(b=>b.id===id); }

// Resolve a rod endpoint {id, off} to its world point. id===null means the
// endpoint is fixed to the background (world-anchored) rather than riding a
// body — off then holds the world coordinate directly, mirroring the
// null-id convention already used by gas heads, cable tethers and slot lines.
function epWorld(ep){
  if(ep.id==null) return [ep.off[0], ep.off[1], 0, 0];
  return worldPt(bodies[bodyIndex(ep.id)], ep.off);
}

// Build a rod constraint between two endpoints, deriving its rest length and
// (for any welded end) the rest angle between that end's body — or the fixed
// world frame, for a background end — and the rod's own direction.
function makeRodCon(a,b,weldA,weldB){
  const [wax,way]=epWorld(a), [wbx,wby]=epWorld(b);
  const len=Math.hypot(wax-wbx,way-wby);
  const con={type:'rod', a, b, len, weldA:!!weldA, weldB:!!weldB, sel:false};
  const phi=Math.atan2(way-wby,wax-wbx);
  if(con.weldA) con.restAngA=(a.id!=null?bodies[bodyIndex(a.id)].th:0)-phi;
  if(con.weldB) con.restAngB=(b.id!=null?bodies[bodyIndex(b.id)].th:0)-phi;
  return con;
}
// Set (or clear) one end's weld flag, recapturing that end's rest angle
// against the rod's *current* direction so toggling never snaps geometry.
function setRodWeld(con,which,val){
  const key = which==='A'?'weldA':'weldB'; con[key]=!!val;
  if(con[key]){
    const [wax,way]=epWorld(con.a), [wbx,wby]=epWorld(con.b);
    const phi=Math.atan2(way-wby,wax-wbx);
    const ep = which==='A'?con.a:con.b;
    const th = ep.id!=null ? bodies[bodyIndex(ep.id)].th : 0;
    if(which==='A') con.restAngA=th-phi; else con.restAngB=th-phi;
  }
}
function toggleRodWeld(con,which){ setRodWeld(con,which, !con[which==='A'?'weldA':'weldB']); }

// ---- §06.2 · gasFrame ----
// gas cylinder frame: piston point on A, closed-end (head) + axis fixed in B or world.
// x is the signed gas-column length along the axis (clamped so V never goes ≤ 0).
function gasFrame(g){
  const A=bodies[bodyIndex(g.a.id)];
  const [pax,pay,prx,pry]=worldPt(A,g.a.off);
  let hx,hy,dW,B=null,ib=-1,hrx=0,hry=0;
  if(g.head.id!=null){ ib=bodyIndex(g.head.id); B=bodies[ib];
    const [x,y,rx,ry]=worldPt(B,g.head.off); hx=x;hy=y;hrx=rx;hry=ry;
    dW=R(B.th,g.head.dir[0],g.head.dir[1]); }
  else { hx=g.head.off[0]; hy=g.head.off[1]; dW=[g.head.dir[0],g.head.dir[1]]; }
  const dl=Math.hypot(dW[0],dW[1])||1; dW=[dW[0]/dl,dW[1]/dl];
  const x=(pax-hx)*dW[0]+(pay-hy)*dW[1];
  return {A,pax,pay,prx,pry,B,ib,hrx,hry,hx,hy,dW,x,xc:Math.max(x,0.03)};
}

// ---- §06.3 · cableFrame ----
// Cable geometry based on a consistently-defined spool angle.
//
// Key points (A, B, C, D per the spec):
//   A = spool anchor — material point on spool rim, stored as cb.localAngle in
//       the spool body frame.  Initialised so spoolAngle = 0 (anchor at closest
//       rim point to tether).
//   B = spool centre (S.x, S.y)
//   C = tether point T
//   D = tangent point — rim point where tangent from T touches the spool on the
//       winding side determined by sign(spoolAngle).
//
//   spoolAngle = ABC angle at B (from ray BA to ray BC, CCW positive, unbounded):
//     = tetherAngle − anchorAngle.
//     Positive → anchor is CW of tether direction → cable winds CW.
//     Negative → anchor is CCW of tether direction → cable winds CCW.
//
//   Tangent point D (world angle from B):
//     d > rs: tangentAngle = tetherAngle − sign(spoolAngle) · arccos(rs/d)
//     d ≤ rs: tangentAngle = tetherAngle (rim point closest to T; or anchorAngle if d≈0)
//
//   |DBC| = arccos(rs/d) for d>rs; 0 for d≤rs.
//   Q = D (tangent wins) if |DBC| < |spoolAngle|;  else Q = A.
//
//   windAngle (same sign as spoolAngle):
//     tangent wins: windAngle = spoolAngle − sign·arccos(rs/d)   [= 0 at transition;
//       arccos(rs/d) reads as 0 once d≤rs, so this is one continuous formula]
//     anchor wins:  windAngle = 0
//   woundLength = |windAngle| · rs
//   paidLength  = Lfree = sqrt(max(0, d²−rs²))  when tangent wins (0 once d≤rs);
//                 |T − Q|  when anchor wins (Q=A, an ordinary rod to T)
//   totalUsed   = woundLength + paidLength
//
//   Jacobian constrains d/dt(totalUsed) = 0, selected by tangentWins alone —
//   NOT also by d≤rs. A many-turn wind can overshoot to d≤rs for a step near
//   the ℓ→0 singularity while still genuinely in the tangent regime (large
//   |spoolAngle|); Lfree is already 0 there, so the tangent-mode row stays
//   the right (and continuous) one. Only the anchor-wins case is a real rod.
//   Tangent mode: Jx=(Dx·Lfree − rs·sign·Dy)/d², Jy=(Dy·Lfree + rs·sign·Dx)/d²;
//     spool [−Jx, −Jy, −rs·sign]; tether [Jx, Jy, moment arm].
//   Direct/rod mode (Q=A): spool [−ux,−uy, ux·ry_Q−uy·rx_Q]; tether [ux,uy,arm].
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

  // B→C vector and tether world angle.
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
  // step that momentarily overshoots the rim — see the tangentWins branch
  // below.
  const tetherInside = d <= rs;
  const beta  = tetherInside ? 0 : Math.acos(Math.max(-1, Math.min(1, rs/d)));
  const Lfree = Math.sqrt(Math.max(0, d*d - rs*rs));
  const tangentAngle = tetherInside ? (d > 1e-9 ? tetherAngle : anchorAngle) : (tetherAngle - sign*beta);
  const dbc = beta;                              // |DBC| = arccos(rs/d), 0 when inside
  const rx_tan = rs*Math.cos(tangentAngle), ry_tan = rs*Math.sin(tangentAngle);
  const Qtan_x = S.x + rx_tan, Qtan_y = S.y + ry_tan;

  // Separation point Q. tangentWins governs both Q's choice and (below) which
  // Jacobian form applies — it must NOT also fork on tetherInside: a body
  // deep in a many-turn wind can overshoot to d<=rs for a step near the
  // ℓ->0 singularity while still genuinely in the tangent regime (large
  // |spoolAngle|), and Lfree already degrades continuously to 0 there. Only
  // gating on tetherInside forced a jump to the anchor-rod formula against
  // the wrong point (Qtan, not A) for that step — a large, energy-adding
  // direction discontinuity, not the harmless near-Δ=0 interior tether the
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
    // (large angular rate near ℓ->0, see cableFrame's header) — Lfree is
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

// ---- §06.4 · slotFrame ----
// resolve a slot's world line frame: slider point on A, plus a line (anchor + unit
// direction) fixed in body B — or in the world when line.id is null
function slotFrame(con){
  const A=bodies[bodyIndex(con.a.id)];
  const [wax,way,rax,ray]=worldPt(A,con.a.off);
  let anchor,dW,B=null,ib=-1,rbx=0,rby=0;
  if(con.line.id!=null){ ib=bodyIndex(con.line.id); B=bodies[ib];
    const [ax,ay,rx,ry]=worldPt(B,con.line.off); anchor=[ax,ay]; rbx=rx; rby=ry;
    dW=R(B.th, con.line.dir[0], con.line.dir[1]); }
  else { anchor=[con.line.off[0],con.line.off[1]]; dW=[con.line.dir[0],con.line.dir[1]]; }
  const dl=Math.hypot(dW[0],dW[1])||1; dW=[dW[0]/dl,dW[1]/dl];
  const n=[-dW[1],dW[0]];                       // unit normal to the rail
  return {A,wax,way,rax,ray,B,ib,rbx,rby,anchor,dW,n};
}

// ---- §06.5 · rowsFor (constraint -> rows dispatch) ----
// One branch per con.type; to reach a specific joint's row math, search its tag,
// e.g.  type==='rod'. Catalog (rows) — cross-references spec §4:
//   pin            2   shared point coincident
//   rod            1   distance held along the connecting line; +1 per welded
//                      end (locks that end's body — or the fixed world frame,
//                      for a background end — to the rod's own direction)
//   slot           1   point-on-line (across-rail zero); +1 more if lockRot (prismatic)
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
    // constraint — §13.6 feeds one through projectPositions as a transient
    // goal while the player drags a body around in edit mode.
    const A = bodies[bodyIndex(con.a.id)];
    const [wax,way,rax,ray] = worldPt(A,con.a.off);
    const ia = bodyIndex(con.a.id);
    return [
      { cols:[[ia,1,0,-ray]], C: wax-con.world[0] },
      { cols:[[ia,0,1, rax]], C: way-con.world[1] }
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
    // point directly — §06.1 epWorld) instead of riding a body.
    const hasA=con.a.id!=null, hasB=con.b.id!=null;
    const A = hasA ? bodies[bodyIndex(con.a.id)] : null;
    const B = hasB ? bodies[bodyIndex(con.b.id)] : null;
    const [wax,way,rax,ray] = hasA ? worldPt(A,con.a.off) : [con.a.off[0],con.a.off[1],0,0];
    const [wbx,wby,rbx,rby] = hasB ? worldPt(B,con.b.off) : [con.b.off[0],con.b.off[1],0,0];
    const ia = hasA?bodyIndex(con.a.id):-1, ib = hasB?bodyIndex(con.b.id):-1;
    let dx=wax-wbx, dy=way-wby, L=Math.hypot(dx,dy)||1e-9;
    const ux=dx/L, uy=dy/L, C=L-con.len;
    const distCols=[];
    if(hasA) distCols.push([ia, ux, uy, -ux*ray+uy*rax]);
    if(hasB) distCols.push([ib,-ux,-uy,  ux*rby-uy*rbx]);
    const rows=[{ cols:distCols, C }];
    // A welded end locks its body's angle (or, for a background end, the fixed
    // world frame — a zero contribution) to the rod's own direction φ. The
    // Jacobian is the rod's angular rate ω = n·(vA−vB)/L (n = perp to the rod),
    // so a weld-A row reads (θȦ − ω) and a weld-B row (θḂ − ω); both share the
    // same ω terms and differ only in which side's θ̇ is added.
    // φ = atan2(dy,dx) is recomputed fresh each step (not unwrap-tracked like
    // the cable's spoolAngle), so a welded end that spins through more than
    // ~half a turn between steps can see its Baumgarte bias jump — fine for
    // the intended use (fixed/rigid attachments), not for a fast-spinning weld.
    if(con.weldA || con.weldB){
      const nx=-uy, ny=ux, phi=Math.atan2(dy,dx);
      if(con.weldA){
        const cols=[];
        if(hasA) cols.push([ia, -nx/L, -ny/L, 1+(nx*ray-ny*rax)/L]);
        if(hasB) cols.push([ib,  nx/L,  ny/L, -(nx*rby-ny*rbx)/L]);
        rows.push({ cols, C: (hasA?A.th:0)-phi-con.restAngA });
      }
      if(con.weldB){
        const cols=[];
        if(hasA) cols.push([ia, -nx/L, -ny/L, -(ny*rax-nx*ray)/L]);
        if(hasB) cols.push([ib,  nx/L,  ny/L, 1+(ny*rbx-nx*rby)/L]);
        rows.push({ cols, C: (hasB?B.th:0)-phi-con.restAngB });
      }
    }
    return rows;
  }
  if(con.type==='slot'){
    // point-on-line: kill the slider point's velocity across the rail normal n.
    // lockRot adds relative angular lock → prismatic.
    const f=slotFrame(con);
    const Dx=f.wax-f.anchor[0], Dy=f.way-f.anchor[1];
    const C=f.n[0]*Dx+f.n[1]*Dy;
    const ia=bodyIndex(con.a.id);
    const row={ cols:[[ia, f.n[0], f.n[1], -f.n[0]*f.ray + f.n[1]*f.rax]], C };
    if(f.B){ const dDot=f.dW[0]*Dx+f.dW[1]*Dy;
      row.cols.push([f.ib, -f.n[0], -f.n[1], f.n[0]*f.rby - f.n[1]*f.rbx - dDot]); }
    const rows=[row];
    if(con.lockRot){
      if(f.B) rows.push({cols:[[ia,0,0,1],[f.ib,0,0,-1]], C:((f.A.th-f.B.th)-con.restAng)});
      else    rows.push({cols:[[ia,0,0,1]], C:(f.A.th-con.restAng)});
    }
    return rows;
  }
  if(con.type==='belt'){
    // inextensible belt: rim tangential speeds equal → fixed phase ratio (holonomic).
    // sense +1 open belt (same sense), −1 crossed.
    const A=bodies[bodyIndex(con.a.id)], B=bodies[bodyIndex(con.b.id)];
    const ia=bodyIndex(con.a.id), ib=bodyIndex(con.b.id), s=con.sense;
    const C=(con.rA*A.th - s*con.rB*B.th) - con.restPhase;
    return [{ cols:[[ia,0,0,con.rA],[ib,0,0,-s*con.rB]], C }];
  }
  if(con.type==='knife'){
    // no-side-slip (Chaplygin knife edge): the contact point's velocity across the
    // heading is zero. Velocity-only — no position invariant (nonholonomic).
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
    // distance from B's centre to P, namely (d − r_A) — a coordinate → nonholonomic.
    const A=bodies[bodyIndex(con.a.id)], B=bodies[bodyIndex(con.b.id)];
    const ia=bodyIndex(con.a.id), ib=bodyIndex(con.b.id);
    let rvx=B.x-A.x, rvy=B.y-A.y; const d=Math.hypot(rvx,rvy)||1e-6;
    const ux=rvx/d, uy=rvy/d; const tx=-uy, ty=ux;   // tangent at contact
    const rA=A.r, armB=d-rA;
    return [{ cols:[[ia, tx, ty, rA],[ib, -tx, -ty, armB]], C:0, nh:true }];
  }
  return [];
}
