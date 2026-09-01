// ============================================================================
//  §08 · PHYSICS SUBSTEP
//  One fixed-step advance of the entire world, in six inline stages. This is a
//  velocity-impulse form: forces integrate to candidate velocities, then a
//  single constraint solve projects those velocities onto the manifold, then
//  positions integrate, then the gas state advances by the first law, then an
//  energy-conservation rescale closes the gap the earlier stages only
//  approximate.
//    §08.1  applied forces -> candidate velocities (gravity, drag spring, gas)
//    §08.2  cable pre-pass (tetherball taut/slack + winding bookkeeping)
//    §08.3  constraint assembly -> Schur solve (§07) -> impulse apply
//    §08.4  position integration
//    §08.5  gas thermodynamics (first law: dU = deltaQ - P dV)
//    §08.6  energy-conservation rescale (exact, chain-length-independent)
//  The stage numbers below (1..5) are the original inline markers; the §08.x
//  tokens above are the greppable handles.
// ============================================================================
// ---- §08.1 · applied forces -> candidate velocities ----
let grab=null; // {bi, off} while mouse-dragging a body during play
function substep(h){
  const N=bodies.length;
  // Energy-conservation target: the mechanical+gas total this substep must
  // return to, absorbing whatever the Baumgarte-stabilized solve below gets
  // only approximately right (see §08.6). Skipped while the user is actively
  // dragging a body -- that spring is meant to inject/remove energy visibly.
  const preE = energy().tot;
  const grabbing = !!(grab && bodies[grab.bi] && !bodies[grab.bi].static);
  // 1) accumulate applied forces, then integrate into candidate velocities v*
  const FX=new Array(N).fill(0), FY=new Array(N).fill(0), TAU=new Array(N).fill(0);
  for(let i=0;i<N;i++){ const b=bodies[i]; if(b.static) continue;
    if(sim.gravity) FY[i] += b.mass*(-sim.g);
    if(grab && bodies[grab.bi]===b){
      const [wx,wy,rx,ry]=worldPt(b,grab.off);
      const vpx=b.vx - b.w*ry, vpy=b.vy + b.w*rx;
      const K=40*b.mass, Cd=9*b.mass;
      // Same screen-space-capped pull as the pose-mode drag (§05.4, tools.js
      // §13.6): the spring's *reach* saturates with on-screen distance, so a
      // body a constraint won't let follow the cursor gets pulled by a bounded
      // force instead of one that keeps growing with however far the mouse has
      // strayed.
      const [px,py]=saturatingPull(wx,wy,mouseWorld[0],mouseWorld[1],DRAG_CAP_PX);
      const Fx=K*px-Cd*vpx, Fy=K*py-Cd*vpy;
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
  // 2) cables: wound arc + straight paid-out segment.  The spool stores the
  //    anchor in its local frame (cb.localAngle).  The spoolAngle (angle ABC,
  //    B=centre, A=anchor, C=tether) is tracked as an unbounded signed value
  //    in cb._spoolAngle, unwrapped each step so it accumulates continuously.
  //    The separation point (Q) and wound arc are derived from that each step.
  //    Unilateral: active only when taut and pulling. cb._spoolAngle only
  //    commits a new value on an active step -- while slack the wound-length
  //    bookkeeping freezes at its last taut value, since a limp cable's wrap
  //    is otherwise indeterminate (design note §C.4).
  const rowJv=(colsIn)=>{ let s=0; for(const c of colsIn){ const b=bodies[c[0]]; s+=c[1]*b.vx+c[2]*b.vy+c[3]*b.w; } return s; };
  for(const cb of cables){ cb._rows=[];
    // Migrate old saves: if no localAngle, reconstruct spoolAngle from old wrap/side
    // FIRST, then place the anchor (localAngle) consistent with that spoolAngle --
    // anchorAngle = tetherAngle - spoolAngle. Deriving localAngle independently as
    // "closest point to tether" (spoolAngle~=0) while separately keeping a nonzero
    // reconstructed spoolAngle would put the anchor and the wound-arc bookkeeping
    // at odds: cableFrame recomputes spoolAngle from the actual anchor position, so
    // an anchor placed at the closest point would immediately re-derive spoolAngle
    // as (near) a multiple of a full turn away from that reconstruction rather than
    // matching it, corrupting Lallow/woundLength. Matches the pre-spoolAngle
    // migration precedent (physics.js, commit 64ab23b): localAngle = tangentAngle -
    // S.th + wrap·side.
    if(cb.localAngle===undefined){
      const S0=bodies[bodyIndex(cb.spool.id)];
      if(!S0){ cb._active=false; cb._C=0; cb._cols=null; cb._Lallow=null; cb._spoolAngle=undefined; continue; }
      let T0;
      if(cb.tether.id!=null){ const tb0=bodies[bodyIndex(cb.tether.id)]; if(!tb0){ cb._active=false; cb._C=0; cb._cols=null; cb._Lallow=null; cb._spoolAngle=undefined; continue; }
        const [tx,ty]=worldPt(tb0,cb.tether.off); T0=[tx,ty]; }
      else T0=[cb.tether.off[0],cb.tether.off[1]];
      const tetherAngle0=Math.atan2(T0[1]-S0.y, T0[0]-S0.x);
      // Reconstruct spoolAngle from old wrap/side (old side=+1 means CCW -> new spoolAngle < 0).
      if(cb.side !== undefined){
        const d0=Math.hypot(T0[0]-S0.x, T0[1]-S0.y);
        const beta0 = d0 > S0.r ? Math.acos(Math.max(-1,Math.min(1,S0.r/d0))) : 0;
        cb.spoolAngle = -cb.side * ((cb.wrap || 0) + beta0);
      } else {
        cb.spoolAngle = 0;
      }
      cb.localAngle = tetherAngle0 - cb.spoolAngle - S0.th;
      cb._spoolAngle = cb.spoolAngle;
    }
    // Migrate saves that have localAngle but predate spoolAngle tracking. The
    // anchor (localAngle) here is already authoritative, so derive spoolAngle from
    // it directly (spoolAngle = tetherAngle - anchorAngle) rather than trusting the
    // old wrap/side estimate outright -- that estimate only recovers which multiple
    // of a full turn the true value sits at (via unwrap), it doesn't override the
    // stored anchor's principal angle.
    if(cb._spoolAngle === undefined){
      if(cb.spoolAngle !== undefined){
        cb._spoolAngle = cb.spoolAngle;
      } else {
        const S0=bodies[bodyIndex(cb.spool.id)]; if(!S0){ cb._active=false; cb._C=0; cb._cols=null; cb._Lallow=null; continue; }
        let T0; if(cb.tether.id!=null){ const tb0=bodies[bodyIndex(cb.tether.id)]; if(!tb0){ cb._active=false; cb._C=0; cb._cols=null; cb._Lallow=null; continue; }
          const [tx,ty]=worldPt(tb0,cb.tether.off); T0=[tx,ty]; } else T0=[cb.tether.off[0],cb.tether.off[1]];
        const tetherAngle0=Math.atan2(T0[1]-S0.y, T0[0]-S0.x);
        const anchorAngle0=S0.th+cb.localAngle;
        let raw=tetherAngle0-anchorAngle0;                 // principal value from the actual stored anchor
        if(cb.side !== undefined){
          // Old side=+1 CCW -> new spoolAngle < 0; use the old wrap/side estimate only
          // to pick which full-turn multiple of `raw` is the true continuous value.
          const d0=Math.hypot(T0[0]-S0.x, T0[1]-S0.y);
          const beta0 = d0 > S0.r ? Math.acos(Math.max(-1,Math.min(1,S0.r/d0))) : 0;
          const est = -cb.side * ((cb.wrap || 0) + beta0);
          let da=raw-est;
          while(da >  Math.PI) da -= Math.PI*2;
          while(da < -Math.PI) da += Math.PI*2;
          raw = est + da;
        }
        cb.spoolAngle = raw;
        cb._spoolAngle = raw;
      }
    }

    // Candidate frame: recomputed live every step from the frozen reference
    // cb._spoolAngle (see persistence below), never chained from a moving
    // reference -- so a slack cable's raw departure geometry never drifts.
    const f=cableFrame(cb, cb._spoolAngle);
    if(!f){ cb._active=false; cb._C=0; cb._cols=null; cb._Lallow=null; continue; }

    // At full wind (Lallow<=0, no free length left) this used to switch to a
    // rigid 2-DOF hinge pinning the tether straight to the material anchor
    // A. That pin point is wrong whenever the accumulated wrap isn't a whole
    // number of turns from the anchor (the general case) -- Q, the true
    // current departure point, sits elsewhere on the rim -- so the hinge
    // yanked the body across the spool toward A in one step, an
    // energy-adding discontinuity a player would see as an unphysical jump.
    // There's no need for a special case at all: the tangent-branch row's
    // Jacobian (§06.3) is finite and well-defined in the ell->0 limit -- it's
    // the tangent-line direction at Q, which continuously becomes the
    // tangent direction *at the tether itself* as Q->T -- so the same one-row
    // taut constraint pins the tether to the rim and only blocks the
    // wind-further direction, exactly as the design note's end-stop
    // (CABLE.md §C.4 step 6) intends, with no discontinuity to cross.
    const C=f.totalUsed - cb.Ltot;
    cb._Lallow=f.Lallow;
    const active = C>-1e-4 && (C>1e-4 || rowJv(f.cols)>0);
    cb._C=C; cb._cols=f.cols; cb._active=active;
    // Freeze the wound-length bookkeeping while slack (design note §C.4): a limp
    // cable's wrap is indeterminate, so only a taut/end-stop-active step commits
    // the new spoolAngle. A slack tether still moves freely and is still drawn
    // from live geometry (render.js/inspector.js reread cb.spoolAngle fresh each
    // frame) -- only the persisted reference used for next step's continuity holds.
    if(active){ cb._spoolAngle = f.spoolAngle; cb.spoolAngle = f.spoolAngle; }
  }

  // ---- §08.3 · constraint assembly -> Schur solve -> impulse apply ----
  // 3) assemble all constraint rows (+ active cable rows)
  const rows=[];
  for(let ci=0;ci<constraints.length;ci++){
    const rs=rowsFor(constraints[ci]);
    constraints[ci]._rows=[];
    for(const r of rs){ constraints[ci]._rows.push(rows.length); rows.push(r); }
  }
  for(const cb of cables){
    if(cb._active){
      cb._rows.push(rows.length); rows.push({cols:cb._cols, C:cb._C});
    }
  }
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
  // 5) thermodynamics: dU = deltaQ - P dV, with deltaQ = kappa(T_res - T) dt when connected.
  //    (R = 1 in abstract units; c_v = 1/(gamma-1).) Work leaves the gas as the
  //    same P·DeltaV the piston force just did on the mechanism -> energy is consistent.
  let totalQ=0;
  for(const g of gases){
    const f2=gasFrame(g); const Vnew=g.bore*f2.xc; const dV=Vnew-g._V;
    const Q=(g.connected? g.kappa*(g.Tres-g.T):0)*h;
    const dU=Q - g._P*dV;
    g.T += dU/(g.n*(1/(g.gamma-1)));
    if(g.T<1e-4) g.T=1e-4;
    g._Q=Q/h;
    totalQ+=Q;
  }

  // ---- §08.6 · energy-conservation rescale ----
  // The Baumgarte-stabilized solve (§08.3) is only an approximate velocity
  // projection -- its leak grows with per-substep drift and with how many
  // rows are coupled, which is why a beta tuned to look flat on a double
  // pendulum still leaks visibly on longer chains (spec drift note, §04.3).
  // Rather than chase a bigger gain (a shorter correction lag traded for a
  // bigger per-step kick, and a lower instability ceiling on longer chains),
  // close the gap exactly: absent a live drag (which legitimately injects/
  // removes energy) the mechanical+gas total entering this substep (preE),
  // plus whatever heat a connected reservoir legitimately added (totalQ), is
  // treated as an invariant. Any discrepancy after the constraint solve,
  // integration, and thermodynamics above is folded back by a single uniform
  // rescale of every body's velocity -- the same idea as a velocity-
  // rescaling thermostat in molecular dynamics. This is exact regardless of
  // chain length, unlike Baumgarte, and needs no new per-mechanism tuning.
  if(!grabbing){
    const post=energy();
    // totalQ is the raw reservoir heat Q (not net dU): the P·dV term inside
    // dU=Q-P·dV is internal work already transferred into the mechanism's
    // KE/PE by §08.1/§08.3/§08.4 and reflected in post.pe/post.ke, so adding
    // it again here would double-count it. Only the externally-sourced Q
    // belongs in the invariant alongside preE (which already includes the
    // gas's own pre-substep internal energy).
    const keTarget=preE+totalQ-post.pe-post.U;
    if(keTarget>0 && post.ke>1e-12){
      const s=Math.sqrt(keTarget/post.ke);
      for(const b of bodies){ if(b.static)continue; b.vx*=s; b.vy*=s; b.w*=s; }
    }
    // keTarget<=0 (all mechanical energy would have to come from an
    // impossible negative KE) or the system is momentarily at rest: leave
    // velocities as-is rather than divide by ~0 or inject energy from
    // nothing -- rare, and self-corrects next substep once ke>0 again.
  }
}
