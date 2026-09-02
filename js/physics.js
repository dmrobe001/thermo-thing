// ============================================================================
//  §08 · PHYSICS SUBSTEP
//  One fixed-step advance of the entire world, in six inline stages. This is a
//  velocity-impulse form: forces integrate to candidate velocities, then a
//  single constraint solve projects those velocities onto the manifold, then
//  positions integrate, then the gas state advances by the first law, then an
//  energy-conservation rescale closes the gap the earlier stages only
//  approximate.
//    §08.0b heat & flow interactions (exponential relaxation toward
//           equilibrium T/P; the gas half of the reservoir/piston overhaul)
//    §08.1  applied forces -> candidate velocities (gravity, drag spring, gas,
//           user-placed linear/rotational springs)
//    §08.2  cable pre-pass (tetherball taut/slack + winding bookkeeping)
//    §08.3  constraint assembly -> Schur solve (§07) -> impulse apply
//    §08.4  position integration
//    §08.5  gas thermodynamics (mechanical work: dU = -P dV)
//    §08.6  energy-conservation rescale (exact, chain-length-independent,
//           momentum-conserving on any island not anchored to the world)
//  The stage numbers below (1..5) are the original inline markers; the §08.x
//  tokens above are the greppable handles. §08.0 and §08.0b are setup passes
//  with no original inline number of their own.
// ============================================================================
// ---- §08.0 · momentum-conserving islands ----
// Partition the world into islands: maximal groups of bodies transitively
// coupled by a constraint, cable, spring, rotational spring or gas. §08.6
// scopes its energy-conservation rescale to one island at a time (instead of
// one scalar over the whole world) so an unrelated mechanism elsewhere in
// the scene can't leak energy into this one, and -- for an island with
// nothing anchoring it to the world -- conserves its linear and angular
// momentum exactly rather than folding them into the same scalar that also
// has to fix energy (see the long comment at §08.6 for why that matters).
//
// Plain array-based union-find (path-halving, no union-by-rank -- islands
// are small) over the N body indices plus one extra virtual node, WORLD,
// standing for "the background, or anything static". Every coupling element
// with a background-fixed endpoint (an {id:null} endpoint -- epWorld's
// world-anchor convention, constraints.js §06.1) or that touches a static
// body gets unioned to WORLD; an island whose root lands on WORLD is
// anchored and has no momentum invariant (the background can source/sink
// it), same as today's global rescale assumed of everything.
function ufFind(p,i){ while(p[i]!==i){ p[i]=p[p[i]]; i=p[i]; } return i; }
function ufUnion(p,a,b){ const ra=ufFind(p,a), rb=ufFind(p,b); if(ra!==rb) p[ra]=rb; }
// Every two-endpoint coupling type shares the {id,...} endpoint shape
// (constraints' a/b, cables' spool/tether, springs'/rotSprings' a/b, gases'
// a/head) -- this table drives the union pass generically instead of
// special-casing each element type. A missing endpoint field (e.g. the
// single-ended 'knife' constraint) reads the same as a background-fixed one.
// Wrapped in getters, not a plain array of the globals themselves: state.js
// §-level code reassigns bodies/constraints/springs/rotSprings/gases/cables
// wholesale (e.g. `constraints=constraints.filter(...)`, `bodies=[]` on load)
// rather than mutating them in place, so a plain array captured once at
// script-load time would keep pointing at whatever was live at that instant.
const COUPLING_TABLES = [
  ()=>[constraints,'a','b'], ()=>[cables,'spool','tether'],
  ()=>[springs,'a','b'], ()=>[rotSprings,'a','b'], ()=>[gases,'com','head'],
];
function computeIslands(){
  const N=bodies.length, WORLD=N;
  const p=new Array(N+1); for(let i=0;i<=N;i++) p[i]=i;
  for(let i=0;i<N;i++) if(bodies[i].static) ufUnion(p,i,WORLD);
  for(const get of COUPLING_TABLES){ const [arr,fa,fb]=get();
    for(const el of arr){
      const ea=el[fa], eb=el[fb];
      const idxs=[]; let anyBg=false;
      if(ea && ea.id!=null) idxs.push(bodyIndex(ea.id)); else anyBg=true;
      if(eb && eb.id!=null) idxs.push(bodyIndex(eb.id)); else anyBg=true;
      for(let i=1;i<idxs.length;i++) ufUnion(p,idxs[0],idxs[i]);
      if(anyBg) for(const idx of idxs) ufUnion(p,idx,WORLD);
    }
  }
  const worldRoot=ufFind(p,WORLD);
  const byRoot=new Map();
  const islandOf=i=>{ const r=ufFind(p,i); let isl=byRoot.get(r);
    if(!isl){ isl={bodyIdx:[],anchored:r===worldRoot,springs:[],rotSprings:[],gases:[]}; byRoot.set(r,isl); }
    return isl; };
  for(let i=0;i<N;i++) if(!bodies[i].static) islandOf(i).bodyIdx.push(i);
  // Bucket each force element's PE/heat bookkeeping into its (now-final)
  // island for §08.6's per-island energy target, keyed off whichever
  // endpoint is a real body (an element always has at least one).
  for(const sp of springs){ islandOf(bodyIndex((sp.a.id!=null?sp.a:sp.b).id)).springs.push(sp); }
  for(const rs of rotSprings){ islandOf(bodyIndex((rs.a.id!=null?rs.a:rs.b).id)).rotSprings.push(rs); }
  // Bucket a gas into whichever of its two boundary bodies is real -- its
  // own COM body if it has a piston, else its head (both may be missing/
  // world, e.g. a free-floating fixed-volume vessel used only via heat/flow
  // interactions elsewhere; such a gas belongs to no mechanical island and
  // never joins isl.gases, exactly like a gas with a real body has nothing
  // to lose from not being scoped to a rescale that can't touch it anyway).
  for(const g of gases){ const realId = g.piston ? g.com.id : g.head.id;
    if(realId!=null) islandOf(bodyIndex(realId)).gases.push(g); }
  return [...byRoot.values()];
}
// §08.6's rescale is a *multiplicative* correction (v *= sqrt(target/actual)),
// which is well-defined only while there's actual KE to scale -- right at an
// island's own turning point (a pendulum at the top of its swing, a piston at
// the top of its bounce) KE passes through zero for real, not as an
// artifact, so neither branch below can apply its correction there and both
// fall back to leaving velocities untouched for that one substep. That
// residual keTarget-vs-actual gap doesn't vanish on its own: the next
// substep's preE is read straight off whatever state was left standing, so
// an uncorrected gap becomes the new baseline permanently rather than
// something later substeps make up for -- confirmed by instrumented play on
// the plain (gasless) pendulum example: 100% of its slow energy loss over a
// long run landed exactly on the substeps where this fallback fired, with
// every other substep's own delta at floating-point zero. ENERGY_BANK defers
// that gap instead of dropping it: keyed by the island's own body-id
// signature (stable frame to frame barring a mid-play topology edit), it
// carries the unresolved amount forward and folds it into the very next
// substep where this same island's KE is large enough to actually rescale,
// so the running total stays exactly conserved across a turning point
// instead of only between them.
const ENERGY_BANK = new Map();
function islandBankKey(isl){ return isl.bodyIdx.map(i=>bodies[i].id).sort((a,b)=>a-b).join(','); }
// Mass, centre of mass, linear momentum and angular momentum (about that
// COM) of a set of body indices, read live from their current x/y/vx/vy/w --
// used both to snapshot an island's pre-substep momentum (§08.1 below) and
// to read its post-solve momentum back at §08.6. Ieff (parallel-axis total
// moment of inertia about the COM) is what §08.6 divides L by to get the
// island's effective single spin rate.
function islandKinematics(idxs){
  let M=0,cx=0,cy=0;
  for(const i of idxs){ const b=bodies[i]; M+=b.mass; cx+=b.mass*b.x; cy+=b.mass*b.y; }
  cx/=M; cy/=M;
  let px=0,py=0,L=0,Ieff=0;
  for(const i of idxs){ const b=bodies[i];
    px+=b.mass*b.vx; py+=b.mass*b.vy;
    const rx=b.x-cx, ry=b.y-cy;
    L += b.I*b.w + b.mass*(rx*b.vy-ry*b.vx);
    Ieff += b.I + b.mass*(rx*rx+ry*ry);
  }
  return {M,cx,cy,px,py,L,Ieff};
}
// ---- §08.0b · heat & flow interactions ----
// Every heatInteractions/flowInteractions entry is {bodyId, gasId, k} --
// gasId===null reads as the background (state.js §04.3 sim.bg), mirroring
// the null-id convention used everywhere else. Two interactions sharing the
// same bodyId are a *pair*: the body mediates a coupling between whatever
// its two interactions each name, at a rate set by the smaller of their two
// body<->gas contact areas (geometry.js §05.2c bodyGasOverlapArea) and the
// pair's conductivities combined in series (like two resistors/conductors
// back to back: 1/k_eff = 1/k1 + 1/k2). A lone interaction with no partner
// on its body moves nothing -- there is no source/sink on the far side.
function groupInteractionsByBody(list){
  const m=new Map();
  for(const it of list){ let a=m.get(it.bodyId); if(!a){ a=[]; m.set(it.bodyId,a); } a.push(it); }
  return m;
}
function gasById(id){ return id==null?null:gases.find(g=>g.id===id); }
// Exact two-body heat relaxation over one substep. Both the equilibrium
// temperature and the exponential approach to it fall straight out of
// solving the pair's linear ODE analytically (spec: "approached
// exponentially ... to avoid overshooting equilibrium") --
//   C_A dT_A/dt = cond(T_B-T_A),  C_B dT_B/dt = -cond(T_B-T_A)
// so D=T_A-T_B decays as D0*exp(-lambda t), lambda=cond(1/C_A+1/C_B), and
// C_A T_A + C_B T_B is exactly conserved -- giving both T's in closed form,
// unconditionally stable for any h. A background side has infinite
// capacity (C->infinity): it never itself moves, and lambda reduces to
// cond/C on the real side alone (Newton's law of cooling toward a fixed
// bath). Mutates the real gas(es)' T and accumulates their _Qstep (the
// non-mechanical energy this substep, physics.js §08.6's rescale target).
function applyHeatPair(iA,iB,h){
  const body=bodies[bodyIndex(iA.bodyId)]; if(!body) return;
  const gA=gasById(iA.gasId), gB=gasById(iB.gasId);
  if(gA===gB) return; // both background, or the same gas on both sides: nothing to move
  const areaA = gA ? bodyGasOverlapArea(body,gA) : Infinity;
  const areaB = gB ? bodyGasOverlapArea(body,gB) : Infinity;
  const areaMin=Math.min(areaA,areaB);
  if(!(areaMin>1e-9)) return;
  const cond = areaMin/(1/Math.max(iA.k,1e-6)+1/Math.max(iB.k,1e-6));
  const TA = gA?gA.T:sim.bg.T, TB = gB?gB.T:sim.bg.T;
  const D0=TA-TB; if(D0===0) return;
  if(gA && gB){
    const CA=gA.mass/(gA.gamma-1), CB=gB.mass/(gB.gamma-1);
    const D1=D0*Math.exp(-cond*(1/CA+1/CB)*h);
    const W=CA*TA+CB*TB;
    const TA1=(W+CB*D1)/(CA+CB), TB1=(W-CA*D1)/(CA+CB);
    gA._Qstep+=CA*(TA1-TA); gA.T=TA1;
    gB._Qstep+=CB*(TB1-TB); gB.T=TB1;
  } else if(gA){
    const CA=gA.mass/(gA.gamma-1);
    const TA1=TB+D0*Math.exp(-cond/CA*h);
    gA._Qstep+=CA*(TA1-TA); gA.T=TA1;
  } else {
    const CB=gB.mass/(gB.gamma-1);
    const TB1=TA-D0*Math.exp(-cond/CB*h);
    gB._Qstep+=CB*(TB1-TB); gB.T=TB1;
  }
}
// Signed recoil-force contribution from a molar flow rate `mdotIn` (moles/
// time flowing INTO the gas this belongs to) -- a thrust-like |P·A| effect
// applied along the gas's own axis (dW), scaled by a thermal-speed-like
// characteristic velocity sqrt(T) of whichever side the moles are actually
// leaving (the source's T, not the destination's). Summed per-gas into
// g._flowForce (reset each substep) and applied in §08.1 alongside the
// pressure force, split across the same piston/head attachment points.
function flowForceMag(mdotIn, Tsrc, Town){
  const vChar=Math.sqrt(Math.max(mdotIn>0?Tsrc:Town, 1e-6));
  return mdotIn*vChar;
}
// Exact two-body flow relaxation, the mass-transfer analogue of
// applyHeatPair: with T and V held fixed over the substep (same
// instantaneous-equilibrium stance as the gas's own P=nT/V, spec §6.1), each
// side's "pressure per mole" s=T/V is constant, P=n·s, and
//   dn_A/dt = flowCond(P_B-P_A),  dn_B/dt = -flowCond(P_B-P_A)
// reduces to the identical linear form as the heat ODE with s playing the
// role of 1/C -- D=P_A-P_B decays as D0*exp(-lambda t), lambda=flowCond
// (s_A+s_B), and n_A+n_B is exactly conserved, giving n_A(t) (and hence
// n_B(t)) in closed form. A background side holds a fixed P (sim.bg.P): the
// real side alone relaxes toward n_eq=P_bg/s. Moles that cross carry their
// *source* gas's own internal energy (dn·cv_src·T_src) into the
// destination, which is absorbed at the destination's own cv (mirrors a
// real gas mixing at constant composition); the gas left behind keeps its
// own T unchanged, since removing gas at a given T doesn't change the T of
// what remains.
function applyFlowPair(iA,iB,h){
  const body=bodies[bodyIndex(iA.bodyId)]; if(!body) return;
  const gA=gasById(iA.gasId), gB=gasById(iB.gasId);
  if(gA===gB) return;
  const areaA = gA ? bodyGasOverlapArea(body,gA) : Infinity;
  const areaB = gB ? bodyGasOverlapArea(body,gB) : Infinity;
  const areaMin=Math.min(areaA,areaB);
  if(!(areaMin>1e-9)) return;
  const flowCond = areaMin/(1/Math.max(iA.k,1e-6)+1/Math.max(iB.k,1e-6));
  const VA = gA?Math.max(gA.bore*gasFrame(gA).xc,1e-6):null;
  const VB = gB?Math.max(gB.bore*gasFrame(gB).xc,1e-6):null;
  const sA = gA?gA.T/VA:null, sB = gB?gB.T/VB:null;
  const PA = gA?gA.mass*sA:sim.bg.P, PB = gB?gB.mass*sB:sim.bg.P;
  let dnBtoA; // net moles moved from B's side to A's side this substep (signed)
  if(gA && gB){
    // dn_A/dt = flowCond(P_B-P_A) = -flowCond·D (D=P_A-P_B), so n_A moves
    // *down* when A is the higher-pressure side (D0>0) -- the minus sign
    // here is that direction, not a typo of the heat pair's plus.
    const D0=PA-PB, D1=D0*Math.exp(-flowCond*(sA+sB)*h);
    dnBtoA=-(D0-D1)/(sA+sB);
  } else if(gA){
    const nEq=sim.bg.P/sA;
    dnBtoA=(nEq-gA.mass)*(1-Math.exp(-flowCond*sA*h));
  } else {
    const nEq=sim.bg.P/sB;
    dnBtoA=-(nEq-gB.mass)*(1-Math.exp(-flowCond*sB*h));
  }
  if(!isFinite(dnBtoA) || dnBtoA===0) return;
  const mdot=dnBtoA/h; // molar rate into A (out of B)
  if(dnBtoA>0){
    const Tsrc = gB?gB.T:sim.bg.T, cvSrc = gB?1/(gB.gamma-1):1/(sim.bg.gamma-1);
    if(gB) gB.mass=Math.max(gB.mass-dnBtoA,1e-6);
    if(gA){ const cvA=1/(gA.gamma-1); const Uold=gA.mass*cvA*gA.T; const dU=dnBtoA*cvSrc*Tsrc;
      gA.mass+=dnBtoA; gA.T=(Uold+dU)/(gA.mass*cvA); gA._Qstep+=dU; }
  } else {
    const dn=-dnBtoA;
    const Tsrc = gA?gA.T:sim.bg.T, cvSrc = gA?1/(gA.gamma-1):1/(sim.bg.gamma-1);
    if(gA) gA.mass=Math.max(gA.mass-dn,1e-6);
    if(gB){ const cvB=1/(gB.gamma-1); const Uold=gB.mass*cvB*gB.T; const dU=dn*cvSrc*Tsrc;
      gB.mass+=dn; gB.T=(Uold+dU)/(gB.mass*cvB); gB._Qstep+=dU; }
  }
  if(gA) gA._flowForce += flowForceMag(mdot, gB?gB.T:sim.bg.T, gA.T);
  if(gB) gB._flowForce += flowForceMag(-mdot, gA?gA.T:sim.bg.T, gB.T);
}
// ---- §08.1 · applied forces -> candidate velocities ----
let grab=null; // {bi, off} while mouse-dragging a body during play
function substep(h){
  // Keep every vessel's COM body mass equal to its gas's own mass, and its
  // static piston marker at the true cap point (geometry.js
  // syncVesselComMass/syncVesselMarkers) *before* islands/preE are
  // captured below -- so a mass change from the previous substep's flow
  // transfer is already baked into this substep's energy baseline instead
  // of being read as drift by §08.6's rescale.
  for(const g of gases){ syncVesselComMass(g); syncVesselMarkers(g); }
  const N=bodies.length;
  // Snapshot each island's pre-substep energy budget (§08.6's target) and
  // momentum (§08.6's momentum-conservation target for a free island)
  // before any force this substep has been applied.
  const islands = computeIslands();
  for(const isl of islands){
    isl.preE = energy(isl).tot;
    const k0 = islandKinematics(isl.bodyIdx);
    isl.P0=[k0.px,k0.py]; isl.L0=k0.L; isl.M=k0.M; isl.com0=[k0.cx,k0.cy];
  }
  const grabbing = !!(grab && bodies[grab.bi] && !bodies[grab.bi].static);

  // §08.0b heat & flow interactions: resolved once per substep, strictly
  // after preE above so its energy delta (_Qstep) reads as the legitimate
  // external input the §08.6 rescale expects, never as pre-existing state.
  for(const g of gases){ g._Qstep=0; g._flowForce=0; g._reflected=false; }
  for(const [,list] of groupInteractionsByBody(heatInteractions))
    for(let i=0;i<list.length;i++) for(let j=i+1;j<list.length;j++) applyHeatPair(list[i],list[j],h);
  for(const [,list] of groupInteractionsByBody(flowInteractions))
    for(let i=0;i<list.length;i++) for(let j=i+1;j<list.length;j++) applyFlowPair(list[i],list[j],h);
  for(const g of gases){ if(g.T<1e-4) g.T=1e-4; }

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
  // gas force elements: net F = (P_gas - P_bg)·bore along the axis, pushing
  // the piston off the head -- the movable wall feels internal *and*
  // atmospheric pressure (spec: piston update), the rest of the vessel feels
  // the equal and opposite reaction. A fixed-volume gas (no piston body,
  // f.A null) has no movable wall to push, so contributes no force at all.
  // A separate contribution, g._flowForce (§08.0b), rides the same axis and
  // the same head/piston split -- the recoil from gas mass crossing this
  // gas's own boundary via a flow interaction.
  for(const g of gases){
    const f=gasFrame(g);
    const P=g.mass*g.T/(g.bore*f.xc); g._P=P; g._V=g.bore*f.xc; g._x=f.x; g._dW=f.dW;
    if(!g.piston) continue;
    // The movable wall's net axial force -- internal pressure against
    // atmosphere, plus the recoil from any mass crossing this gas's own
    // boundary (§08.0b) -- is now a *generalized* force on the vessel's own
    // sepRate coordinate directly (the "genuine new internal coordinate"
    // architecture, DEVELOPMENT.md §6.1): no body force/torque bookkeeping,
    // no head/piston split. The reaction the host frame feels comes out of
    // the gasmount constraint's own multiplier in §08.3, same as any other
    // joint reaction.
    const Fmag=(P-sim.bg.P)*g.bore + g._flowForce;
    // The flow-force's own mechanical work is drawn from the same
    // transferred internal energy already booked into _Qstep at §08.0b --
    // credit it here (pre-solve sepRate, first-order in h) so §08.6's
    // rescale treats this recoil KE as legitimate external input instead of
    // erasing it as drift. The ordinary pressure-force work needs no such
    // credit: it already trades against the gas's own P·dV term (§08.5).
    if(Math.abs(g._flowForce)>1e-12) g._Qstep += g._flowForce*g.sepRate*h;
    const effMassR=Math.max(g.mass/12, EFF_MASS_FLOOR);
    g.sepRate += h*Fmag/effMassR;
  }
  // linear spring force elements: Hookean, F = k*(restLen-L) along the line
  // joining the two endpoints -- same two-endpoint frame as rod (twoPointFrame,
  // §06.1), reused as-is since a spring needs only L and the unit direction,
  // never phi (there is no weld/angle-lock row for a force element).
  for(const sp of springs){
    // Either endpoint may be a vessel-interior point (constraints.js
    // §06.2d) -- epFrame handles both uniformly; a force (Fx,Fy) applied at
    // a point is exactly that point's velCols evaluated at (Fx,Fy) rather
    // than a unit direction (virtual-work identity, see §06.2d's comment).
    const A=epFrame(sp.a), B=epFrame(sp.b);
    const dx=A.wx-B.wx, dy=A.wy-B.wy, L=Math.hypot(dx,dy)||1e-9;
    const ux=dx/L, uy=dy/L;
    const Fmag=sp.k*(sp.restLen-L);
    const Fx=Fmag*ux, Fy=Fmag*uy;
    for(const [idx,cx,cy,cw] of mergeCols([A.velCols(Fx,Fy), B.velCols(-Fx,-Fy)])){
      if(bodies[idx].static) continue;
      FX[idx]+=cx; FY[idx]+=cy; TAU[idx]+=cw;
    }
  }
  // rotational spring force elements: torsional, tau = k*(restAngle-thRel)
  // between the two bodies' frame angles (background reads as a fixed
  // theta=0, mirroring rotSpringRelAngle, §06.6) -- a pure couple, no point
  // of application, so no torque-arm term the way rod/gas need one.
  for(const rs of rotSprings){
    const hasA=rs.a.id!=null, hasB=rs.b.id!=null;
    const ia=hasA?bodyIndex(rs.a.id):-1, ib=hasB?bodyIndex(rs.b.id):-1;
    const tau=rs.k*(rs.restAngle-rotSpringRelAngle(rs));
    if(hasA && !bodies[ia].static) TAU[ia]+=tau;
    if(hasB && !bodies[ib].static) TAU[ib]-=tau;
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
  // The coordinate space now extends past the real bodies: indices 0..N-1
  // are bodies as always, N..N+V-1 are one flat scalar per vessel-with-a-
  // piston (its sepRate). coordGetV/coordInvM/coordApplyImpulse (geometry.js
  // §05.2d) branch on idx<N to delegate to *exactly* the same bodies[]/
  // invMdiag path used before this rearchitecture, so every row built only
  // from ordinary bodies is provably unaffected -- only rows that actually
  // carry a vessel-coordinate column (the gasmount constraint, a vessel-
  // interior endpoint, or the floor-stop row below) exercise the new branch.
  const vlist = vesselCoordList().list;
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
  // Gas minimum-volume stop: keeps a piston's true axial separation g.sep
  // from ever compressing past GAS_MIN_X. Without this, that job fell to
  // gasFrame's xc=max(sep,GAS_MIN_X) clamp inside the P=nT/V formula alone
  // -- softening the *pressure*, not the geometry. A piston fast enough to
  // cross GAS_MIN_X within one substep then kept compressing past it
  // unopposed by any further pressure rise (xc, and so P and V=bore*xc, all
  // plateau once sep<GAS_MIN_X), so §08.5's dU=-P.dV saw a frozen V and
  // stopped crediting/debiting the gas's own U for motion that was still
  // really happening -- decoupling the mechanism's actual KE change from the
  // gas's energy ledger, which §08.6's rescale then "corrected" by erasing
  // real KE every time a stroke dipped below the floor.
  //
  // Every other row here shares one Baumgarte target (drive post-solve Jv
  // toward -kb*C), which is exactly right for a bilateral joint but wrong
  // for this stop -- pinning sepRate at ~0 for as long as it sits at the
  // floor would make the stop a fully inelastic "glue" that destroys its
  // entire closing KE on contact, and because that KE lands at exactly zero
  // rather than merely shrinking, §08.6's multiplicative rescale can't hand
  // it back either. A stop meant to be energy-conserving needs restitution,
  // not Baumgarte, so this row (when active) carries its own `restitution:1`
  // target instead of a Baumgarte one (see the rhs loop below) -- reflect
  // the closing rate, don't drive it to zero.
  //
  // This row IS pushed through the *same* combined Schur solve as everything
  // else, not a standalone impulse: sepRate is only a free coordinate from
  // the gasmount constraint's own perspective in appearance -- flipping it
  // alone, with `com`'s own velocity left untouched, violates the mount's
  // position-lock rows (a vessel-interior point's velocity is COM's rigid
  // motion *plus* sepRate's own contribution, constraints.js §06.2d), which
  // a separate/standalone reflection pass has no way to see or account for.
  // Sharing the K matrix with the mount's rows is what gives this stop the
  // *correct* coupled effective inertia (the same coupling that makes an
  // anchored piston's effective inertia mass/3, not the bare mass/12) rather
  // than bouncing off an artificially soft (or hard) floor and immediately
  // being partially undone by the mount's own correction on the very next
  // pass -- confirmed by instrumented play: a standalone flip-sepRate-alone
  // reflection let the vessel tunnel straight through GAS_MIN_X into
  // negative separation within a couple of substeps once anything (even
  // just the mount) coupled sepRate to another coordinate.
  //
  // Activation is anticipatory: C+h*Jv (this step's own linear prediction of
  // where C=sep-GAS_MIN_X lands after §08.4's position integration, at the
  // current pre-solve closing rate) rather than just the current C.
  // GAS_MIN_X is deliberately tiny (a numerical floor, not a real object's
  // thickness), so a piston needs only a modest closing speed for h*Jv to
  // already exceed the whole gap once C gets close; waiting for C itself to
  // cross zero the way the cable's own end-stop does would let §08.4
  // integrate position past the floor first, reopening the same gap for the
  // one substep it takes to react.
  for(const g of gases){
    if(!g.piston) continue; // fixed-length vessel: x is a constant, nothing to stop
    const C=g.sep-GAS_MIN_X, Jv=g.sepRate;
    if(!(C<-1e-4 || (C+h*Jv<1e-4 && Jv<0))) continue; // not about to violate
    if(Jv<0){
      const sepIdx=vesselCoordList().idxOf.get(g.id);
      rows.push({cols:[[sepIdx,1,0,0]], C:0, restitution:1});
      g._reflected=true;
    }
    if(C<0){ g.sep=GAS_MIN_X; } // already past the floor -- snap back out
  }
  const m=rows.length;
  if(m>0){
    // per-row body->j map for K assembly
    const maps=rows.map(r=>{ const mp=new Map(); for(const c of r.cols) mp.set(c[0],[c[1],c[2],c[3]]); return mp; });
    const Jv=new Array(m);
    for(let i=0;i<m;i++){ let s=0; for(const c of rows[i].cols){ const [vx,vy,vw]=coordGetV(c[0],N,vlist); s+=c[1]*vx+c[2]*vy+c[3]*vw; } Jv[i]=s; }
    // K = J M^-1 J^T  (+ reg)
    const Kt=[]; for(let i=0;i<m;i++) Kt.push(new Array(m).fill(0));
    for(let i=0;i<m;i++){
      for(let j=i;j<m;j++){
        let s=0;
        for(const [bi,ji] of maps[i]){ const jj=maps[j].get(bi); if(!jj)continue;
          const im=coordInvM(bi,N,vlist); s+=ji[0]*jj[0]*im[0]+ji[1]*jj[1]*im[1]+ji[2]*jj[2]*im[2]; }
        Kt[i][j]=s; Kt[j][i]=s;
      }
      Kt[i][i]+=sim.reg;
    }
    const kb=sim.beta/sim.h;
    // Every row wants post-solve Jv to land on its own target: a bilateral
    // (or cable) row's target is the Baumgarte drift correction -kb*C; the
    // gas floor-stop row's target is a restitution reflection, -e*Jv itself
    // (post-solve Jv = -e*Jv-before) -- same K matrix, same solve, only the
    // right-hand side differs. rhs[i] solves for that: Jv[i]+rhs[i] == target.
    const rhs=new Array(m);
    for(let i=0;i<m;i++){
      const r=rows[i];
      rhs[i] = r.restitution!=null ? -(1+r.restitution)*Jv[i] : -(Jv[i]+kb*r.C);
    }
    const lam=solveLinear(Kt,rhs,m);
    // 4) apply impulses  v += M^-1 J^T lambda
    for(let i=0;i<m;i++){ const li=lam[i]; if(!li)continue;
      for(const c of rows[i].cols) coordApplyImpulse(c[0],N,vlist,c[1],c[2],c[3],li); }
    for(let ci=0;ci<constraints.length;ci++) constraints[ci]._lam=constraints[ci]._rows.map(ri=>lam[ri]);
    for(const cb of cables) cb._lam=cb._rows.map(ri=>lam[ri]);
  } else {
    for(const c of constraints) c._lam=[]; for(const cb of cables) cb._lam=[];
  }

  // ---- §08.4 · position integration ----
  // 4) integrate positions
  for(const b of bodies){ if(b.static)continue; b.x+=h*b.vx; b.y+=h*b.vy; b.th+=h*b.w; }
  for(const g of gases){ if(g.piston) g.sep += h*g.sepRate; }

  // ---- §08.5 · gas thermodynamics (mechanical work) ----
  // 5) mechanical work: dU = -P dV over this substep's actual volume change
  //    (R = 1 in abstract units; c_v = 1/(gamma-1)). Work leaves the gas as the
  //    same P·DeltaV the piston force just did on the mechanism -> energy is
  //    consistent. Heat/flow's non-mechanical energy input was already folded
  //    into T (and n) at §08.0b, above the force pass -- g._Qstep already
  //    holds that amount for the §08.6 rescale below, so it isn't added again
  //    here; a gas with no interactions (kappa/flow = 0, the adiabatic "gas
  //    spring" case) is untouched by §08.0b and traverses its adiabat purely
  //    through this term, exactly as before.
  //
  //    The gas's own dU only accounts for work crossing *its* boundary, but
  //    the force actually delivered to the bodies (§08.1) is the *net*
  //    (P_gas - P_bg)·bore, not P_gas·bore alone -- the background makes up
  //    the difference by doing P_bg·dV of flow work on (or absorbing it from)
  //    the piston at its open face, same as atmospheric pressure doing work
  //    on a piston expanding into open air. That term never touches the
  //    gas's own T, but it does leave (or enter) the mechanism's own
  //    KE/PE/U ledger, so it has to be booked here -- g._Watm, alongside
  //    g._Qstep -- or §08.6's invariant is missing an energy channel and
  //    "corrects" for it by silently overwriting the correct post-solve KE,
  //    which is exactly what masks a real (P_gas - P_bg) restoring force.
  for(const g of gases){
    const f2=gasFrame(g); const Vnew=g.bore*f2.xc; const dV=Vnew-g._V;
    // A substep in which the minimum-volume stop (§08.3) elastically
    // reflected this gas's piston doesn't get an ordinary dV credited to
    // its *own* U here. That reflection redirects KE alone (no U change, by
    // design), but position still integrates over this *entire* step
    // (§08.4) at the post-reflection (outward) velocity, so Vnew-g._V reads
    // as a real expansion that never happened as gas work -- it's just the
    // reflection's own kinematics, whose KE it already conserved on its
    // own. Crediting that phantom dV to P·dV here double-books it: on top
    // of the KE the reflection already conserved, this would *also* debit
    // the gas's U for having (apparently) produced that KE -- a real,
    // uncompensated loss confirmed by instrumented play (every reflecting
    // substep's own energy dropped; every other substep's didn't, and the
    // two didn't cancel over a cycle). So dV reads as zero for *this* gas's
    // own thermodynamic purposes on a reflected step; g._V/g._P simply
    // resynchronize to the new true geometry starting next substep.
    //
    // g._Watm, below, does NOT get the same treatment, even though it's
    // built from the same dV: it isn't the gas's own energy, it's real
    // P_bg·dV work crossing the piston's *outer*, atmosphere-facing face,
    // and the background does that work regardless of *why* the piston
    // moved this step -- reflection or ordinary compression -- because the
    // real geometry (and so the real V) changed either way. Zeroing it here
    // too (an earlier version of this fix did exactly that) quietly broke
    // the *other* half of the invariant instead: §08.6's keTarget subtracts
    // totalWatm from the mechanism's own energy precisely so a piston
    // expanding into the atmosphere is credited for the work leaving
    // through that open face: forge Watm to 0 while x still genuinely grew
    // this step and the mechanism's tracked energy keeps everything it
    // "should" have paid the atmosphere for that expansion, injecting a
    // real surplus once per reflection -- confirmed by instrumented play:
    // KE+PE+U+P_bg·V (the true, atmosphere-inclusive total) grew by exactly
    // P_bg·dV on every reflecting step once dU alone stopped leaking.
    const dU = g._reflected ? 0 : -g._P*dV;
    const cv=1/(g.gamma-1);
    g.T += dU/(g.mass*cv);
    let clampedAway=0;
    if(g.T<1e-4){
      // T can't actually go negative, but the substep that would have taken
      // it there already delivered the *mechanical* side of that dU to the
      // bodies in full back at §08.1 (computed from g._P, itself derived
      // from T *before* this clamp) -- clamping T alone, with nothing else
      // adjusted, leaves post.U reading higher than the naive dU implied
      // while KE already reflects the full amount, so §08.6 below is left
      // to "discover" that surplus and correct for it via whatever spare
      // capacity that substep's rescale happens to have (the same
      // degenerate-KE limitation as a turning point, banked the same way --
      // but this floor binds far more often than a real turning point does,
      // so the unresolved residue adds up over long runs instead of staying
      // negligible). Crediting the clamped-away amount to g._Watm keeps it
      // out of the invariant the same way real atmospheric flow work is --
      // it already left the tracked KE+PE+U system as real KE, so it
      // shouldn't also sit in keTarget waiting to be clawed back.
      clampedAway=(1e-4-g.T)*(g.mass*cv);
      g.T=1e-4;
    }
    g._Q=g._Qstep/h;
    g._Watm=sim.bg.P*dV+clampedAway;
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
  // plus whatever heat/flow interactions (§08.0b) legitimately added or
  // removed this substep (totalQ), is treated as an invariant -- per island
  // (§08.0), not once over the whole
  // world, so an unrelated mechanism elsewhere in the scene can't leak
  // energy into (or out of) this one through a shared scalar.
  //
  // An *anchored* island (touches a static body or a background-fixed
  // endpoint, §08.0) has the background as an unlimited momentum sink/
  // source, so folding its whole discrepancy into one uniform speed rescale
  // is fine -- same idea as a velocity-rescaling thermostat in molecular
  // dynamics, and exact regardless of chain length.
  //
  // A *free* island (nothing anchoring it) has a second exact invariant a
  // bare rescale can't see: its internal forces are all equal-and-opposite
  // (Newton's third law), so its total linear and angular momentum are
  // conserved by the real physics. Gravity is the only external force that
  // can touch them -- it changes momentum by the known amount M*g*h and
  // contributes zero net torque about the island's own centre of mass
  // (uniform gravity can't spin a system about its own COM) -- so both are
  // exactly computable targets, not just "whatever they happened to be".
  // A single global scalar can't tell "legitimate spring/rotSpring
  // oscillation" apart from "residual momentum leak" (mainly invM*mass
  // round-off): it amplifies both together, and since translation/rotation
  // have no restoring force to check them, any leak random-walks into
  // visible drift over many substeps -- the "flying ice cube" failure mode
  // of global velocity-rescaling thermostats in MD. So a free island's
  // correction is two exact-then-approximate steps instead of one scalar:
  // an additive fix that pins its rigid motion (COM translation + rotation
  // about the COM) to the momentum/angular-momentum target, then the same
  // multiplicative energy rescale as the anchored case, but applied only to
  // what's left over -- the internal/relative ("shape") velocity field,
  // which by construction carries zero net momentum and angular momentum,
  // so scaling it can never reintroduce the drift the first step removed.
  if(!grabbing){
    for(const isl of islands){
      let totalQ=0, totalWatm=0; for(const g of isl.gases){ totalQ+=g._Qstep; totalWatm+=g._Watm; }
      // Free island: apply the exact-trapezoidal COM position correction
      // *before* post.pe is read below -- see the "Free island" comment
      // further down for the full derivation of why this exists at all.
      // Doing it before `post=energy(isl)` is what keeps this and the
      // internal-motion multiplicative rescale from double-correcting the
      // same gravity-vs-position mismatch: post.pe has to already reflect
      // the corrected geometry, or keTarget still carries the old (stale)
      // shortfall and the internal rescale "discovers" and re-fixes it a
      // second time on top of this one, injecting real spurious energy --
      // confirmed by instrumented play (this ordering bug alone inflated a
      // gas piston's total energy over 20x across 2000 simulated seconds
      // once there was internal motion for the rescale to (over)act on).
      if(!isl.anchored){
        const k0now=islandKinematics(isl.bodyIdx);
        const Ptx0=isl.P0[0], Pty0=isl.P0[1]+(sim.gravity? -sim.g*isl.M*h : 0);
        const V00=[isl.P0[0]/isl.M, isl.P0[1]/isl.M], Vt0=[Ptx0/isl.M, Pty0/isl.M];
        const comTargetX=isl.com0[0]+0.5*(V00[0]+Vt0[0])*h, comTargetY=isl.com0[1]+0.5*(V00[1]+Vt0[1])*h;
        const dComX=comTargetX-k0now.cx, dComY=comTargetY-k0now.cy;
        for(const i of isl.bodyIdx){ bodies[i].x+=dComX; bodies[i].y+=dComY; }
      }
      const post=energy(isl);
      // totalQ is the raw reservoir heat Q (not net dU): the P·dV term inside
      // dU=Q-P·dV is internal work already transferred into the mechanism's
      // KE/PE by §08.1/§08.3/§08.4 and reflected in post.pe/post.ke, so adding
      // it again here would double-count it. Only the externally-sourced Q
      // belongs in the invariant alongside preE (which already includes the
      // gas's own pre-substep internal energy). totalWatm (§08.5) is the
      // flow work each gas traded with the background at its open face
      // (P_bg·dV) -- energy that leaves the mechanism as it expands (or
      // enters it as it's compressed) without ever being heat or being
      // credited to the gas's own U, so it has to leave the invariant the
      // same way totalQ enters it. post.SPE (spring potential energy,
      // hud.js §12.1) is subtracted the same way post.pe is: it's a
      // legitimate KE<->PE channel the spring force itself already moved
      // energy through in §08.1/§08.4, not a discrepancy to fold back.
      const bankKey=islandBankKey(isl);
      const banked=ENERGY_BANK.get(bankKey)||0;
      const keTarget=isl.preE+totalQ-totalWatm-post.pe-post.U-post.SPE+banked;
      if(isl.anchored){
        // keTarget<=0 (all mechanical energy would have to come from an
        // impossible negative KE) or the island is momentarily at rest: leave
        // velocities as-is rather than divide by ~0 or inject energy from
        // nothing -- genuinely rare in isolation, but a real turning point
        // hits it on every single swing, so the gap this substep couldn't
        // resolve is banked (see ENERGY_BANK above) for the next substep
        // that can, rather than silently dropped.
        if(keTarget>0 && post.ke>1e-12){
          const s=Math.sqrt(keTarget/post.ke);
          for(const i of isl.bodyIdx){ const b=bodies[i]; b.vx*=s; b.vy*=s; b.w*=s; }
          for(const g of isl.gases) if(g.piston) g.sepRate*=s;
          ENERGY_BANK.delete(bankKey);
        } else {
          ENERGY_BANK.set(bankKey,keTarget);
        }
        continue;
      }
      // Free island: additive rigid-motion fix to the exact momentum target,
      // then a multiplicative rescale of only the leftover internal field.
      // (Position was already corrected above, before `post` -- `now` here
      // reads the corrected geometry.)
      const now=islandKinematics(isl.bodyIdx);
      const Ieff=now.Ieff;
      const Ptx=isl.P0[0], Pty=isl.P0[1]+(sim.gravity? -sim.g*isl.M*h : 0);
      const Ltarget=isl.L0;
      const Vt=[Ptx/isl.M, Pty/isl.M], wt=Ieff>1e-9?Ltarget/Ieff:0;
      const Vnow=[now.px/isl.M, now.py/isl.M], wnow=Ieff>1e-9?now.L/Ieff:0;
      const keRigidTarget=0.5*isl.M*(Vt[0]*Vt[0]+Vt[1]*Vt[1])+0.5*Ieff*wt*wt;
      // The rigid velocity fix above lands on Vt/wt *exactly* -- momentum's
      // impulse-momentum theorem is exact for gravity regardless of step
      // size, so there's no error left to correct there. But `keRigidTarget`
      // built from that exact Vt is therefore *also* exact, for the COM
      // motion a body with that exact momentum actually has right now -- and
      // that could come out short of what §08.4's plain `x+=h*vx` position
      // update alone needs to keep total energy flat, with no freedom left
      // in velocity to make up the difference without breaking momentum
      // (fully determined once P/L are fixed, nothing left to tune). Left
      // uncorrected, that's exactly the classic secular energy drift of
      // symplectic Euler applied to *constant* (unbounded, non-oscillating)
      // acceleration: confirmed on a single free body with gravity and
      // nothing else touching it -- no gas, no internal motion for the
      // multiplicative rescale below to ever absorb the deficit into, so
      // ENERGY_BANK just grew forever instead of resolving. The COM position
      // correction applied above (before `post`) is what actually fixes
      // this -- the island's true trajectory under the exact momentum
      // history isl.P0 -> Pt is the trapezoidal (midpoint-velocity)
      // displacement 0.5*(V0+Vt)*h, which -- unlike the plain forward-Euler
      // step §08.4 already took -- is *exact* for constant acceleration (the
      // textbook reason leapfrog/Verlet integrators don't drift on a falling
      // body); shifting every body in the island by the same delta is the
      // position analogue of this velocity fix, purely rigid so it can't
      // touch the internal/shape field's own zero-net momentum property.
      // Angular motion doesn't need the same treatment: gravity contributes
      // zero net torque about an island's own COM (see Ltarget above), so
      // there's no analogous *torque*-driven position drift for rotation to
      // correct -- only linear momentum has an external driver here at all.
      let keInt=0; const internal=[];
      for(const i of isl.bodyIdx){ const b=bodies[i];
        const rx=b.x-now.cx, ry=b.y-now.cy;
        // Internal ("shape") velocity: current velocity minus the rigid
        // field implied by the island's *actual* current momentum (Vnow/
        // wnow) -- zero-net by construction, so it's pure relative motion,
        // never the momentum error itself, whatever s ends up being below.
        const ivx=b.vx-(Vnow[0]-wnow*ry), ivy=b.vy-(Vnow[1]+wnow*rx), iw=b.w-wnow;
        internal.push([ivx,ivy,iw,rx,ry]);
        keInt+=0.5*b.mass*(ivx*ivx+ivy*ivy)+0.5*b.I*iw*iw;
      }
      // A vessel's sepRate carries zero net island momentum by construction
      // (the internal axial force pair nets to zero COM force -- see the
      // plan's two-benchmark derivation), so it's pure internal/shape motion
      // just like a body's own (ivx,ivy,iw) above -- same keInt pool, same
      // multiplicative scale at apply time, no additive rigid-field term to
      // add back (unlike a body's velocity, sepRate has no rigid component).
      for(const g of isl.gases){ if(!g.piston) continue;
        const effMassR=Math.max(g.mass/12, EFF_MASS_FLOOR);
        keInt += 0.5*effMassR*g.sepRate*g.sepRate;
      }
      // s=1 (desiredInt<=0 or no internal motion to scale) reproduces the
      // exact momentum-target rigid field plus the untouched internal field
      // -- the free-island analogue of the anchored branch's "leave as-is",
      // and banks the same way: the rigid part above is an additive fix that
      // always lands exactly on target, so it's only this internal/shape
      // component that can go unresolved at its own island's turning point.
      const desiredInt=keTarget-keRigidTarget;
      const resolved=desiredInt>0 && keInt>1e-12;
      const s=resolved?Math.sqrt(desiredInt/keInt):1;
      if(resolved) ENERGY_BANK.delete(bankKey); else ENERGY_BANK.set(bankKey,desiredInt-keInt);
      isl.bodyIdx.forEach((i,k)=>{ const b=bodies[i]; const [ivx,ivy,iw,rx,ry]=internal[k];
        b.vx=Vt[0]-wt*ry+s*ivx; b.vy=Vt[1]+wt*rx+s*ivy; b.w=wt+s*iw;
      });
      for(const g of isl.gases) if(g.piston) g.sepRate*=s;
    }
    // Every island present this substep either cleared or refreshed its own
    // ENERGY_BANK entry above; anything left over belongs to an island that
    // no longer exists (bodies/constraints edited or deleted mid-play), so
    // it's stale and would otherwise sit in the map forever.
    const liveBankKeys=new Set(islands.map(islandBankKey));
    for(const k of ENERGY_BANK.keys()) if(!liveBankKeys.has(k)) ENERGY_BANK.delete(k);
  }
}
