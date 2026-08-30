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
// cable geometry: the spool stores a control-point angle in its local frame
// (cb.localAngle).  The natural tangent from tether T to the spool gives angle
// qang; the wound amount wb = (qang − qctrl)·side.
//   wb ≥ 0  →  "tangent mode": cable runs T→Q (natural tangent point); the
//              wound arc Q→Q_ctrl holds the rest of the total length.
//   wb < 0  →  "direct mode": control point has swung past the tangent toward
//              the tether; cable runs straight T→Q_ctrl.
// Returns null when the tether sits inside the spool.
// The Jacobian correctly couples spool rotation to cable length (torque term
// −rs·side on the spool ω column, derived from d(Lfree+rs·wb)/dt).
function cableFrame(cb){
  const S=bodies[bodyIndex(cb.spool.id)]; if(!S) return null;
  const rs=S.r;
  let T, tb=null, trx=0, tryy=0, ti=-1;
  if(cb.tether.id!=null){ ti=bodyIndex(cb.tether.id); tb=bodies[ti];
    const [tx,ty,rx,ry]=worldPt(tb,cb.tether.off); T=[tx,ty]; trx=rx; tryy=ry; }
  else { T=[cb.tether.off[0],cb.tether.off[1]]; }
  const Dx=T[0]-S.x, Dy=T[1]-S.y; const d=Math.hypot(Dx,Dy);
  if(d<=rs*1.0001) return null;
  const phi=Math.atan2(Dy,Dx); const beta=Math.acos(Math.max(-1,Math.min(1,rs/d)));
  const qang=phi + cb.side*beta;
  const Lfree=Math.sqrt(Math.max(0,d*d-rs*rs));
  const d2=d*d;
  // Control point: localAngle is the angle in the spool's own frame; it stays
  // fixed relative to the spool material and rotates with it.  When undefined
  // (old saves), default to the current natural tangent position (wb = 0).
  const localAngle=cb.localAngle!==undefined ? cb.localAngle : qang-S.th;
  const qctrl=S.th+localAngle;
  const rx_ctrl=rs*Math.cos(qctrl), ry_ctrl=rs*Math.sin(qctrl);
  const Qctrl_x=S.x+rx_ctrl, Qctrl_y=S.y+ry_ctrl;
  const wb=(qang-qctrl)*cb.side;          // signed wound angle (radians)
  const is=bodyIndex(cb.spool.id);
  let cols, Qx, Qy, ux, uy, mode;
  if(wb>=0){
    // Tangent mode — correct Jacobian for d(Lfree + rs·wb)/dt:
    //   ∂/∂vS = −(Dx·Lfree − rs·side·Dy)/d², −(Dy·Lfree + rs·side·Dx)/d², −rs·side
    //   ∂/∂vT =  (Dx·Lfree − rs·side·Dy)/d²,  (Dy·Lfree + rs·side·Dx)/d², moment arm
    mode='tangent';
    Qx=S.x+rs*Math.cos(qang); Qy=S.y+rs*Math.sin(qang);
    ux=(T[0]-Qx)/(Lfree||1); uy=(T[1]-Qy)/(Lfree||1);
    const Jx=(Dx*Lfree - rs*cb.side*Dy)/d2;
    const Jy=(Dy*Lfree + rs*cb.side*Dx)/d2;
    cols=[];
    if(!S.static) cols.push([is, -Jx, -Jy, -rs*cb.side]);
    if(tb&&!tb.static) cols.push([ti, Jx, Jy, Jx*(-tryy)+Jy*trx]);
  } else {
    // Direct mode — cable goes T→Q_ctrl (a material point on the spool rim)
    mode='direct';
    Qx=Qctrl_x; Qy=Qctrl_y;
    const dcx=T[0]-Qctrl_x, dcy=T[1]-Qctrl_y; const dc=Math.hypot(dcx,dcy)||1e-9;
    ux=dcx/dc; uy=dcy/dc;
    cols=[];
    if(!S.static) cols.push([is, -ux, -uy, ux*ry_ctrl-uy*rx_ctrl]);
    if(tb&&!tb.static) cols.push([ti, ux, uy, -ux*tryy+uy*trx]);
  }
  return {S,rs,T,Qx,Qy,Lfree,ux,uy,cols,qang,qctrl,wb,Qctrl_x,Qctrl_y,rx_ctrl,ry_ctrl,mode,localAngle};
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
