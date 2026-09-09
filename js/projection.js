// ============================================================================
//  §09 · POSITION PROJECTION & REACTIONS
//  Two post-solve services that read the same rows §06 produces.
//    §09.1  projectPositions  (Gauss-Newton drift correction, holonomic rows only)
//    §09.2  conMaxC           (worst position error on a constraint -- HUD/violations)
//    §09.3  reactionOf        (multiplier lambda/h -> the reaction force/torque a joint carries)
//  §09.3 is the instrumentation payoff (spec §7): the constraint that enforces a
//  joint also hands back the force it carries, for free, from the same solve.
// ============================================================================
// ---- §09.1 · projectPositions ----
// Snaps the assembly to the nearest consistent configuration, mass-weighted so
// heavy/static bodies move least. `extra` holds transient constraints (e.g. a
// drag goal). Used at Play and while articulating a dragged body.
//
// NONHOLONOMIC ROWS (nh:true) have no position invariant to project onto -- there
// is no C(q) whose zero set they are, which is what "nonholonomic" means. They are
// still enforced here, on the only thing that IS defined for them: the position
// DELTA of this edit. A rolling row says J(q).dq = 0 for an increment dq, so the
// residual driven to zero is J(q).(q - q0), the slip accumulated since the pose
// this call started from -- rebuilt each Newton pass against the live J. That is
// what makes a paused drag articulate a rolling pair: pull the rack body along and
// the pinion turns to keep the slip at zero, the same as it would while running.
//
// It is a first-order account of a path-dependent quantity -- true rolling slip is
// the integral of J.dq along the path taken, and this evaluates J at the ends of
// the increment rather than through it -- so a drag rolls accurately when followed
// in small steps (pointermove's) and only approximately if the pose is teleported
// in one jump. That is inherent to rolling, not to this implementation: where the
// pinion ends up genuinely depends on the route the rack took to get there.
//
// `baseline` lets a caller that MOVED something itself before projecting (the
// kinematic drag of a frozen body, tools.js §13.6) hand in the pose from before
// its own move, so that motion counts toward the slip too. Omitted, the baseline
// is simply the pose on entry.
// Extra compliance folded into a `soft` row's own diagonal term (relative to
// its own weight, so it scales with whatever body/mass it happens to touch --
// see the dragpin row in constraints.js). Sized so a soft row alone (nothing
// else contending for the same DOF) still closes to within a fraction of a
// percent of its target over `iters` Newton passes, while a hard row sharing
// that DOF drowns it out almost entirely -- the hard row's own diagonal carries
// no such penalty, so the least-squares split leans overwhelmingly its way.
const DRAG_SOFT_ALPHA = 1;
// The pose a projection measures its nonholonomic slip against: every coordinate
// the rows can write, in body order (the same order their columns index).
function poseSnapshot(){ return bodies.map(b=>[b.x, b.y, b.th, b.len||0]); }
// A nonholonomic row's residual in delta form: J(q) . (q - q0), the slip this row
// has accumulated since `q0`. Bodies added or removed since the snapshot are
// skipped rather than read across a shifted index.
function nhSlip(row, q0){
  let s=0;
  for(const c of row.cols){
    const b=bodies[c[0]], p=q0[c[0]]; if(!b || !p) continue;
    s += c[1]*(b.x-p[0]) + c[2]*(b.y-p[1]) + c[3]*(b.th-p[2]) + (c[4]||0)*((b.len||0)-p[3]);
  }
  return s;
}
function projectPositions(iters, extra, baseline){
  // The frozen flags gate which coordinates may move here as much as they do in the
  // substep, and projection runs from edit paths that never reach one -- so derive
  // them first rather than depending on a substep having happened (§06.2b).
  refreshFrozen();
  const cons = (extra && extra.length) ? constraints.concat(extra) : constraints;
  const q0 = (baseline && baseline.length===bodies.length) ? baseline : poseSnapshot();
  for(let it=0; it<iters; it++){
    const rows=[]; for(const con of cons){ if(con._compiled) continue;
                     for(const r of rowsFor(con)) rows.push(r.nh ? {...r, C:nhSlip(r,q0)} : r); }
    const m=rows.length; if(!m) return;
    let maxC=0; for(const r of rows){ const a=Math.abs(r.C); if(a>maxC)maxC=a; }
    if(maxC<1e-7) return;
    // Four components per column -- see the same assembly in physics.js §08.3.
    const maps=rows.map(r=>{ const mp=new Map(); for(const c of r.cols) mp.set(c[0],[c[1],c[2],c[3],c[4]||0]); return mp; });
    const Kt=[]; for(let i=0;i<m;i++) Kt.push(new Array(m).fill(0));
    for(let i=0;i<m;i++){
      for(let j=i;j<m;j++){ let s=0;
        for(const [bi,ji] of maps[i]){ const jj=maps[j].get(bi); if(!jj)continue; const im=invMdiag(bodies[bi]);
          s+=ji[0]*jj[0]*im[0]+ji[1]*jj[1]*im[1]+ji[2]*jj[2]*im[2]+ji[3]*jj[3]*im[3]; }
        Kt[i][j]=s; Kt[j][i]=s; }
      if(rows[i].soft) Kt[i][i]+=Kt[i][i]*DRAG_SOFT_ALPHA;
      Kt[i][i]+=sim.reg;
    }
    const rhs=rows.map(r=>-r.C);
    const lam=solveLinear(Kt,rhs,m);
    for(let i=0;i<m;i++){ const li=lam[i]; if(!li)continue;
      for(const c of rows[i].cols){ const b=bodies[c[0]];
        b.x+=b.invM*c[1]*li; b.y+=b.invM*c[2]*li; b.th+=b.invI*c[3]*li;
        // A vessel's length is a configuration coordinate like any other, so the
        // projection settles it too -- pinning both caps really does snap the
        // vessel to the length its constraints imply.
        if(c[4] && b.invMu){ b.len=Math.max(VESSEL_MIN_LEN, b.len+b.invMu*c[4]*li); refreshVessel(b); } } }
  }
}
// ---- §09.2 · conMaxC ----
function conMaxC(con){ if(con._compiled) return 0; let m=0; for(const r of rowsFor(con)){ if(r.nh) continue; const a=Math.abs(r.C); if(a>m)m=a; } return m; }
// Same drift tolerance the HUD/canvas use (render.js §11.5) to flag a constraint as
// visibly unsatisfied -- shared here so the reset baseline can apply the identical
// standard (§16.1 saveState) rather than drifting out of step with what the red
// highlight already tells the player is wrong.
const CON_DRIFT_TOL = 2e-3;
// Whether every constraint currently holds within that tolerance -- the gate
// saveState() uses to decide whether the live pose is fit to become the reset
// baseline (transport.js §16.1).
function constraintsSatisfied(){
  for(const con of constraints){ if(conMaxC(con) > CON_DRIFT_TOL) return false; }
  return true;
}
// ---- §09.3 · reactionOf ----
// Each multiplier is the impulse its row carried this substep; divided by h it is
// the force (or torque) the joint is holding. Which multiplier is which used to be
// worked out by counting: every branch below re-derived rowsFor's row ORDER from the
// joint's own flags -- weldA? then weldB? then two per control point, three if it is
// locked -- which is the same arithmetic written four times, in a second file, that
// nothing checks against the rows it is describing. Adding a row to a kind silently
// moved every readout after it.
//
// Rows now carry their own role (constraints.js §06.5), captured beside the
// multipliers in physics.js §08.3, so a readout ASKS for the row it wants:
//
//   online   a joint held on a line       station  a joint held at its place along it
//   weld     an angle lock                 mesh     a disk's rolling contact
//   pin      a coincidence row at a vertex
//   belt / cvt / knife                    the one row each of those carries
//
// `at` distinguishes several rows of one role on the same joint -- a vertex's label
// on a line's rows, the disk's id on a mesh row. Omitted, the first row of that role
// is what a readout reports, which is the rule every branch below follows.
function lamAll(con, role, at){
  const roles=con._roles, l=con._lam, out=[];
  if(!roles || !l) return out;
  for(let i=0;i<roles.length;i++){
    if(roles[i][0]!==role) continue;
    if(at!==undefined && roles[i][1]!==at) continue;
    out.push(l[i]);
  }
  return out;
}
const lamAt = (con, role, at) => lamAll(con, role, at)[0];
function reactionOf(con){
  const l=con._lam; const h=sim.h; if(!l||!l.length) return null;
  const lam = (role,at)=>{ const v=lamAt(con,role,at); return v===undefined?undefined:v/h; };
  // The torque reported alongside a force is the first angle lock the joint holds.
  const tau = lam('weld');
  if(con.type==='belt'){ return {belt:true, val:lam('belt')||0}; }
  if(con.type==='cvt'){ const A=bodies[bodyIndex(con.a.id)], B=bodies[bodyIndex(con.b.id)];
    let rvx=B.x-A.x, rvy=B.y-A.y, d=Math.hypot(rvx,rvy)||1e-6; const ux=rvx/d,uy=rvy/d; const tx=-uy, ty=ux;
    const t=lam('cvt')||0;
    return {x:A.x+ux*A.r, y:A.y+uy*A.r, fx:tx*t, fy:ty*t}; }
  if(con.type==='line'){
    // A line reports the AXIAL force in its first held stretch, at the origin those
    // stations are measured from and along the bar -- which on a two-joint bar is
    // exactly the tension the rod used to report. A line that holds no stretch but
    // drives a disk reports the rolling force instead, at the foot of the
    // perpendicular from that disk's centre, exactly as the rack's did.
    const f=lineFrame(con); if(!f) return null;
    const t=lam('station');
    if(t!==undefined) return {x:f.wax, y:f.way, fx:f.ux*t, fy:f.uy*t, tau};
    for(const id of (con.mesh||[])){
      const m=lam('mesh',id); if(m===undefined) continue;
      const B=bodies[bodyIndex(id)]; if(!B) continue;
      const rho=(B.x-f.wax)*f.nx + (B.y-f.way)*f.ny;
      return {x:B.x-rho*f.nx, y:B.y-rho*f.ny, fx:f.ux*m, fy:f.uy*m, tau};
    }
    return {x:f.wax, y:f.way, fx:0, fy:0, tau};
  }
  if(con.type==='vertex'){
    // Every joined incidence past the primary carries its own pair of rows, and the
    // readout reports the FIRST -- the same rule the rest of §09.3 follows, and on a
    // two-body vertex (what a pin was) it is the only one there is.
    // The rows come x, y per incidence, so the first pair is the first held body's.
    const [wx,wy]=vertexWorld(con);
    const f=lamAll(con,'pin');
    return {x:wx, y:wy, fx:(f[0]||0)/h, fy:(f[1]||0)/h};
  }
  const A=bodies[bodyIndex(con.a.id)]; if(!A) return null;
  const [wax,way]=epWorldPt(A,con.a.off);
  if(con.type==='knife'){ const hh=R(A.th,con.dir[0],con.dir[1]); const hl=Math.hypot(hh[0],hh[1])||1;
    const nx=-hh[1]/hl, ny=hh[0]/hl; const t=lam('knife')||0;
    return {x:wax,y:way, fx:nx*t, fy:ny*t}; }
  return null;
}
