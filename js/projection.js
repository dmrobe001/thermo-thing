// ============================================================================
//  §09 · POSITION PROJECTION & REACTIONS
//  Two post-solve services that read the same rows §06 produces.
//    §09.1  projectPositions  (Gauss-Newton drift correction, holonomic rows only)
//    §09.2  conMaxC           (worst position error on a constraint — HUD/violations)
//    §09.3  reactionOf        (multiplier λ/h -> the reaction force/torque a joint carries)
//  §09.3 is the instrumentation payoff (spec §7): the constraint that enforces a
//  joint also hands back the force it carries, for free, from the same solve.
// ============================================================================
// ---- §09.1 · projectPositions ----
// Snaps the assembly to the nearest consistent configuration, mass-weighted so
// heavy/static bodies move least. `extra` holds transient constraints (e.g. a
// drag goal). Used at Play and while articulating a dragged body. Nonholonomic
// rows (nh:true) are skipped — they have no position invariant to project onto.
function projectPositions(iters, extra){
  const cons = (extra && extra.length) ? constraints.concat(extra) : constraints;
  for(let it=0; it<iters; it++){
    const rows=[]; for(const con of cons){ for(const r of rowsFor(con)) if(!r.nh) rows.push(r); }
    const m=rows.length; if(!m) return;
    let maxC=0; for(const r of rows){ const a=Math.abs(r.C); if(a>maxC)maxC=a; }
    if(maxC<1e-7) return;
    const maps=rows.map(r=>{ const mp=new Map(); for(const c of r.cols) mp.set(c[0],[c[1],c[2],c[3]]); return mp; });
    const Kt=[]; for(let i=0;i<m;i++) Kt.push(new Array(m).fill(0));
    for(let i=0;i<m;i++){
      for(let j=i;j<m;j++){ let s=0;
        for(const [bi,ji] of maps[i]){ const jj=maps[j].get(bi); if(!jj)continue; const im=invMdiag(bodies[bi]);
          s+=ji[0]*jj[0]*im[0]+ji[1]*jj[1]*im[1]+ji[2]*jj[2]*im[2]; }
        Kt[i][j]=s; Kt[j][i]=s; }
      Kt[i][i]+=sim.reg;
    }
    const rhs=rows.map(r=>-r.C);
    const lam=solveLinear(Kt,rhs,m);
    for(let i=0;i<m;i++){ const li=lam[i]; if(!li)continue;
      for(const c of rows[i].cols){ const b=bodies[c[0]]; if(b.static)continue;
        b.x+=b.invM*c[1]*li; b.y+=b.invM*c[2]*li; b.th+=b.invI*c[3]*li; } }
  }
}
// ---- §09.2 · conMaxC ----
function conMaxC(con){ let m=0; for(const r of rowsFor(con)){ if(r.nh) continue; const a=Math.abs(r.C); if(a>m)m=a; } return m; }
// ---- §09.3 · reactionOf ----
function reactionOf(con){
  const l=con._lam; const h=sim.h; if(!l||!l.length) return null;
  if(con.type==='belt'){ return {belt:true, val:l[0]/h}; }
  if(con.type==='cvt'){ const A=bodies[bodyIndex(con.a.id)], B=bodies[bodyIndex(con.b.id)];
    let rvx=B.x-A.x, rvy=B.y-A.y, d=Math.hypot(rvx,rvy)||1e-6; const ux=rvx/d,uy=rvy/d; const tx=-uy, ty=ux;
    return {x:A.x+ux*A.r, y:A.y+uy*A.r, fx:tx*(l[0]/h), fy:ty*(l[0]/h)}; }
  const A=bodies[bodyIndex(con.a.id)]; if(!A) return null;
  const [wax,way]=worldPt(A,con.a.off);
  if(con.type==='knife'){ const hh=R(A.th,con.dir[0],con.dir[1]); const hl=Math.hypot(hh[0],hh[1])||1;
    const nx=-hh[1]/hl, ny=hh[0]/hl; return {x:wax,y:way, fx:nx*(l[0]/h), fy:ny*(l[0]/h)}; }
  if(con.type==='pin'||con.type==='ground'){ return {x:wax,y:way, fx:l[0]/h, fy:l[1]/h}; }
  if(con.type==='rod'){
    const B=bodies[bodyIndex(con.b.id)]; const [wbx,wby]=worldPt(B,con.b.off);
    let dx=wax-wbx,dy=way-wby,L=Math.hypot(dx,dy)||1e-9;
    return {x:wax,y:way, fx:(dx/L)*(l[0]/h), fy:(dy/L)*(l[0]/h)};
  }
  if(con.type==='weld'){ return {x:wax,y:way, fx:l[0]/h, fy:l[1]/h, tau:l[2]/h}; }
  if(con.type==='slot'){ const f=slotFrame(con);
    return {x:f.wax,y:f.way, fx:f.n[0]*(l[0]/h), fy:f.n[1]*(l[0]/h),
            tau:(con.lockRot&&l[1]!==undefined)? l[1]/h : undefined}; }
  return null;
}
