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
//   pin / ground   2   shared point coincident (ground pins to a fixed world pt)
//   rod            1   distance held along the connecting line
//   weld           3   coincident point + relative angle locked
//   slot           1   point-on-line (across-rail zero); +1 more if lockRot (prismatic)
//   belt           1   fixed phase ratio of two rim angles (holonomic)
//   knife          1   no-side-slip contact (NONHOLONOMIC, nh:true)
//   cvt            1   tangential match at a variable-radius contact (NONHOLONOMIC)
// (Cable rows are built inline in §08.2, not here, because they are unilateral.)
function rowsFor(con){
  // Each row carries the raw position error C (the value to drive to zero). The
  // velocity solver scales it by beta/h (Baumgarte); the position projection uses
  // C directly. Same Jacobian rows serve both.
  if(con.type==='pin' || con.type==='ground'){
    const A = bodies[bodyIndex(con.a.id)];
    const [wax,way,rax,ray] = worldPt(A,con.a.off);
    let wbx,wby, colsB=null;
    if(con.type==='pin'){
      const B = bodies[bodyIndex(con.b.id)];
      const [x,y,rbx,rby] = worldPt(B,con.b.off); wbx=x; wby=y;
      colsB = { ib:bodyIndex(con.b.id), rbx, rby };
    } else { wbx=con.world[0]; wby=con.world[1]; }
    const Cx = wax-wbx, Cy = way-wby;
    const ia = bodyIndex(con.a.id);
    const rowX = { cols:[[ia,1,0,-ray]], C:Cx };
    const rowY = { cols:[[ia,0,1, rax]], C:Cy };
    if(colsB){ rowX.cols.push([colsB.ib,-1,0, colsB.rby]); rowY.cols.push([colsB.ib,0,-1,-colsB.rbx]); }
    return [rowX,rowY];
  }
  if(con.type==='rod'){
    const A=bodies[bodyIndex(con.a.id)], B=bodies[bodyIndex(con.b.id)];
    const [wax,way,rax,ray]=worldPt(A,con.a.off);
    const [wbx,wby,rbx,rby]=worldPt(B,con.b.off);
    let dx=wax-wbx, dy=way-wby, L=Math.hypot(dx,dy)||1e-9;
    const ux=dx/L, uy=dy/L, C=L-con.len;
    const ia=bodyIndex(con.a.id), ib=bodyIndex(con.b.id);
    return [{ cols:[ [ia, ux, uy, -ux*ray+uy*rax],
                     [ib,-ux,-uy,  ux*rby-uy*rbx] ], C }];
  }
  if(con.type==='weld'){
    const A=bodies[bodyIndex(con.a.id)], B=bodies[bodyIndex(con.b.id)];
    const [wax,way,rax,ray]=worldPt(A,con.a.off);
    const [wbx,wby,rbx,rby]=worldPt(B,con.b.off);
    const ia=bodyIndex(con.a.id), ib=bodyIndex(con.b.id);
    const Cx=wax-wbx, Cy=way-wby, Ca=(A.th-B.th)-con.restAng;
    return [
      { cols:[[ia,1,0,-ray],[ib,-1,0, rby]], C:Cx },
      { cols:[[ia,0,1, rax],[ib,0,-1,-rbx]], C:Cy },
      { cols:[[ia,0,0,1],[ib,0,0,-1]], C:Ca }
    ];
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
