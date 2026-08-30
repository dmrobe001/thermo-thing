// ============================================================================
//  §08 · PHYSICS SUBSTEP
//  One fixed-step advance of the entire world, in five inline stages. This is a
//  velocity-impulse form: forces integrate to candidate velocities, then a
//  single constraint solve projects those velocities onto the manifold, then
//  positions integrate, then the gas state advances by the first law.
//    §08.1  applied forces -> candidate velocities (gravity, drag spring, gas)
//    §08.2  cable pre-pass (tetherball taut/slack + winding bookkeeping)
//    §08.3  constraint assembly -> Schur solve (§07) -> impulse apply
//    §08.4  position integration
//    §08.5  gas thermodynamics (first law: dU = δQ − P dV)
//  The stage numbers below (1..5) are the original inline markers; the §08.x
//  tokens above are the greppable handles.
// ============================================================================
// ---- §08.1 · applied forces -> candidate velocities ----
let grab=null; // {bi, off} while mouse-dragging a body during play
function substep(h){
  const N=bodies.length;
  // 1) accumulate applied forces, then integrate into candidate velocities v*
  const FX=new Array(N).fill(0), FY=new Array(N).fill(0), TAU=new Array(N).fill(0);
  for(let i=0;i<N;i++){ const b=bodies[i]; if(b.static) continue;
    if(sim.gravity) FY[i] += b.mass*(-sim.g);
    if(grab && bodies[grab.bi]===b){
      const [wx,wy,rx,ry]=worldPt(b,grab.off);
      const vpx=b.vx - b.w*ry, vpy=b.vy + b.w*rx;
      const K=40*b.mass, Cd=9*b.mass;
      const Fx=K*(mouseWorld[0]-wx)-Cd*vpx, Fy=K*(mouseWorld[1]-wy)-Cd*vpy;
      FX[i]+=Fx; FY[i]+=Fy; TAU[i]+=rx*Fy-ry*Fx;
    }
  }
  // gas-piston force elements: F = P·bore along the axis, pushing piston off the head
  for(const g of gases){
    const f=gasFrame(g);
    const P=g.n*g.T/(g.bore*f.xc); g._P=P; g._V=g.bore*f.xc; g._x=f.x; g._dW=f.dW;
    const Fx=P*g.bore*f.dW[0], Fy=P*g.bore*f.dW[1];
    const ia=bodyIndex(g.a.id), A=bodies[ia];
    if(!A.static){ FX[ia]+=Fx; FY[ia]+=Fy; TAU[ia]+= f.prx*Fy - f.pry*Fx; }
    if(f.B && !f.B.static){ FX[f.ib]-=Fx; FY[f.ib]-=Fy; TAU[f.ib]+= f.hrx*(-Fy) - f.hry*(-Fx); }
  }
  for(let i=0;i<N;i++){ const b=bodies[i]; if(b.static){ b.vx=0;b.vy=0;b.w=0; continue; }
    b.vx += h*b.invM*FX[i]; b.vy += h*b.invM*FY[i]; b.w += h*b.invI*TAU[i]; }

  // ---- §08.2 · cable pre-pass ----
  // 2) cables: straight tangent + wound remainder.  The spool stores the control
  //    point in its local frame (cb.localAngle); the wound amount wb is derived
  //    from the current tangent angle and cb.localAngle each step — no explicit
  //    wrap accumulation needed.  Unilateral: active only when taut and pulling.
  for(const cb of cables){ cb._rows=[];
    // Migrate old saves that stored wrap but not localAngle: derive localAngle
    // from the then-current tangent angle and the saved wrap.
    if(cb.localAngle===undefined){
      const f0=cableFrame(cb);   // called with localAngle undefined → wb=0 fallback
      if(!f0){ cb._active=false; cb._C=0; cb._cols=null; continue; }
      cb.localAngle=f0.qang-f0.S.th-(cb.wrap||0)*cb.side;
    }
    const f=cableFrame(cb); if(!f){ cb._active=false; cb._C=0; cb._cols=null; continue; }
    const wb=f.wb;
    cb._wrap=wb;                             // signed wound angle for rendering
    cb.wrap=Math.max(0,wb);                  // non-negative display field (inspector)
    let C;
    if(f.mode==='tangent'){
      const Lused=f.Lfree+f.rs*wb;
      C=Lused-cb.Ltot;
      cb._Lallow=cb.Ltot-f.rs*wb;
    } else {
      const dc=Math.hypot(f.T[0]-f.Qctrl_x, f.T[1]-f.Qctrl_y);
      C=dc-cb.Ltot;
      cb._Lallow=cb.Ltot;
    }
    cb._C=C; cb._cols=f.cols;
    let Jv=0; for(const c of f.cols){ const b=bodies[c[0]]; Jv+=c[1]*b.vx+c[2]*b.vy+c[3]*b.w; }
    cb._active = C>-1e-4 && (C>1e-4 || Jv>0);
  }

  // ---- §08.3 · constraint assembly -> Schur solve -> impulse apply ----
  // 3) assemble all constraint rows (+ active cable rows)
  const rows=[];
  for(let ci=0;ci<constraints.length;ci++){
    const rs=rowsFor(constraints[ci]);
    constraints[ci]._rows=[];
    for(const r of rs){ constraints[ci]._rows.push(rows.length); rows.push(r); }
  }
  for(const cb of cables){ if(cb._active){ cb._rows.push(rows.length); rows.push({cols:cb._cols, C:cb._C}); } }
  const m=rows.length;
  if(m>0){
    // per-row body->j map for K assembly
    const maps=rows.map(r=>{ const mp=new Map(); for(const c of r.cols) mp.set(c[0],[c[1],c[2],c[3]]); return mp; });
    const Jv=new Array(m);
    for(let i=0;i<m;i++){ let s=0; for(const c of rows[i].cols){ const b=bodies[c[0]]; s+=c[1]*b.vx+c[2]*b.vy+c[3]*b.w; } Jv[i]=s; }
    // K = J M^-1 J^T  (+ reg)
    const Kt=[]; for(let i=0;i<m;i++) Kt.push(new Array(m).fill(0));
    for(let i=0;i<m;i++){
      for(let j=i;j<m;j++){
        let s=0;
        for(const [bi,ji] of maps[i]){ const jj=maps[j].get(bi); if(!jj)continue;
          const im=invMdiag(bodies[bi]); s+=ji[0]*jj[0]*im[0]+ji[1]*jj[1]*im[1]+ji[2]*jj[2]*im[2]; }
        Kt[i][j]=s; Kt[j][i]=s;
      }
      Kt[i][i]+=sim.reg;
    }
    const kb=sim.beta/sim.h;
    const rhs=new Array(m); for(let i=0;i<m;i++) rhs[i]=-(Jv[i]+kb*rows[i].C);
    const lam=solveLinear(Kt,rhs,m);
    // 4) apply impulses  v += M^-1 J^T lambda
    for(let i=0;i<m;i++){ const li=lam[i]; if(!li)continue;
      for(const c of rows[i].cols){ const b=bodies[c[0]]; if(b.static)continue;
        b.vx+=b.invM*c[1]*li; b.vy+=b.invM*c[2]*li; b.w+=b.invI*c[3]*li; } }
    for(let ci=0;ci<constraints.length;ci++) constraints[ci]._lam=constraints[ci]._rows.map(ri=>lam[ri]);
    for(const cb of cables) cb._lam=cb._rows.map(ri=>lam[ri]);
  } else {
    for(const c of constraints) c._lam=[]; for(const cb of cables) cb._lam=[];
  }

  // ---- §08.4 · position integration ----
  // 4) integrate positions
  for(const b of bodies){ if(b.static)continue; b.x+=h*b.vx; b.y+=h*b.vy; b.th+=h*b.w; }

  // ---- §08.5 · gas thermodynamics (first law) ----
  // 5) thermodynamics: dU = δQ − P dV, with δQ = κ(T_res − T) dt when connected.
  //    (R = 1 in abstract units; c_v = 1/(γ−1).) Work leaves the gas as the
  //    same P·ΔV the piston force just did on the mechanism → energy is consistent.
  for(const g of gases){
    const f2=gasFrame(g); const Vnew=g.bore*f2.xc; const dV=Vnew-g._V;
    const Q=(g.connected? g.kappa*(g.Tres-g.T):0)*h;
    const dU=Q - g._P*dV;
    g.T += dU/(g.n*(1/(g.gamma-1)));
    if(g.T<1e-4) g.T=1e-4;
    g._Q=Q/h;
  }
}
