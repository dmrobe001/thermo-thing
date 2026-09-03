// ============================================================================
//  §08 · PHYSICS SUBSTEP
//  One fixed-step advance of the entire world, in four inline stages. This is a
//  velocity-impulse form: forces integrate to candidate velocities, then a
//  single constraint solve projects those velocities onto the manifold, then
//  positions integrate, then an energy-conservation rescale closes the gap the
//  earlier stages only approximate.
//    §08.0b heat & mass exchange -- the closed-form relaxation an interaction
//           pair runs at frozen geometry, ahead of everything mechanical
//    §08.1b vesselGasStep -- the gas force, as a discrete gradient of its own
//           potential, on a vessel's length coordinate (defined ahead of §08.1,
//           which calls it)
//    §08.1  applied forces -> candidate velocities (gravity, drag spring,
//           user-placed linear/rotational springs, vessel centrifugal term)
//    §08.2  cable pre-pass (tetherball taut/slack + winding bookkeeping)
//    §08.3  constraint assembly -> Schur solve (§07) -> impulse apply
//    §08.4  position integration
//    §08.6  energy-conservation rescale (exact, chain-length-independent,
//           momentum-conserving on any island not anchored to the world)
//  The stage numbers below (1..4) are the original inline markers; the §08.x
//  tokens above are the greppable handles. §08.0 and §08.0b are setup passes
//  with no original inline number of their own.
// ============================================================================
// ---- §08.0 · momentum-conserving islands ----
// Partition the world into islands: maximal groups of bodies transitively
// coupled by a constraint, cable, spring, or rotational spring. §08.6
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
// (constraints' a/b, cables' spool/tether, springs'/rotSprings' a/b) -- this
// table drives the union pass generically instead of special-casing each
// element type. A missing endpoint field (e.g. the single-ended 'knife'
// constraint) reads the same as a background-fixed one. Wrapped in getters,
// not a plain array of the globals themselves: state.js §-level code
// reassigns bodies/constraints/springs/rotSprings/cables wholesale (e.g.
// `constraints=constraints.filter(...)`, `bodies=[]` on load) rather than
// mutating them in place, so a plain array captured once at script-load time
// would keep pointing at whatever was live at that instant.
const COUPLING_TABLES = [
  ()=>[constraints,'a','b'], ()=>[cables,'spool','tether'],
  ()=>[springs,'a','b'], ()=>[rotSprings,'a','b'],
];
function computeIslands(){
  const N=bodies.length, WORLD=N;
  const p=new Array(N+1); for(let i=0;i<=N;i++) p[i]=i;
  for(let i=0;i<N;i++) if(frozenSolid(bodies[i])) ufUnion(p,i,WORLD);
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
    if(!isl){ isl={bodyIdx:[],anchored:r===worldRoot,springs:[],rotSprings:[]}; byRoot.set(r,isl); }
    return isl; };
  // frozenSolid, not `static`: a vessel pinned at its mid-plane still has a live
  // length, so it belongs to an island and its length energy is that island's.
  for(let i=0;i<N;i++) if(!frozenSolid(bodies[i])) islandOf(i).bodyIdx.push(i);
  // Bucket each force element's PE bookkeeping into its (now-final) island
  // for §08.6's per-island energy target, keyed off whichever endpoint is a
  // real body (an element always has at least one).
  for(const sp of springs){ islandOf(bodyIndex((sp.a.id!=null?sp.a:sp.b).id)).springs.push(sp); }
  for(const rs of rotSprings){ islandOf(bodyIndex((rs.a.id!=null?rs.a:rs.b).id)).rotSprings.push(rs); }
  return [...byRoot.values()];
}
// §08.6's rescale is a *multiplicative* correction (v *= sqrt(target/actual)),
// which is well-defined only while there's actual KE to scale -- right at an
// island's own turning point (a pendulum at the top of its swing) KE passes
// through zero for real, not as an artifact, so neither branch below can
// apply its correction there and both fall back to leaving velocities
// untouched for that one substep. That residual keTarget-vs-actual gap
// doesn't vanish on its own: the next substep's preE is read straight off
// whatever state was left standing, so an uncorrected gap becomes the new
// baseline permanently rather than something later substeps make up for --
// confirmed by instrumented play on the plain pendulum example: 100% of its
// slow energy loss over a long run landed exactly on the substeps where this
// fallback fired, with every other substep's own delta at floating-point
// zero. ENERGY_BANK defers
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
// ---- §08.0b · heat & mass exchange between vessels ----
// The thermodynamic pass, and the only code in the engine allowed to write a gas's
// `kap` or `mass` (geometry.js §05.2d). It runs FIRST -- ahead of islands, ahead of
// the energy baseline, ahead of every force -- at FROZEN geometry, which is what
// makes each exchange a closed-form relaxation rather than a stepped ODE, and what
// makes the two sides' dU exactly equal and opposite (VESSEL.md §V.10).
//
// Frozen geometry is doing real work here. Because `U` is a state function of
// (kap, mass, len) and `len` does not move during the pass, "how much energy moved"
// is a difference of two evaluations of that function, not an integral anyone has to
// accumulate, attribute, or decide about. The stripped implementation's `_Qstep` /
// `_Watm` / `_reflected` triple existed only because it integrated dU = -P dV
// incrementally alongside a rescale that also had to be told which increments were
// legitimate; none of it has an analogue here.
//
// Running before §08.0's snapshot rather than after is what leaves §08.6 completely
// untouched. VESSEL.md §V.10 anticipated one new term in the island invariant --
// "the amount moved this substep ... exactly one term, computed once". Ordering it
// ahead of `preE` is that term, spent instead of carried: every island's baseline is
// simply read off the post-exchange state, so the rescale defends a target that
// already includes whatever the exchange moved, per island, with no channel to plumb
// and nothing to miscredit.
//
// An interaction names one solid body and one vessel (`vessel.id === null` reads as
// the background, mirroring the null-id convention used everywhere else). A LONE
// interaction moves nothing -- there is no source or sink on the far side of the
// body. Two interactions of the same kind sharing the same body are a PAIR: the body
// is a wall between the two things they name, and the pair couples them *through* it.
// That is deliberately the same shape as every other coupling in the library -- a
// relation between two named participants, active only where the player put it -- and
// it keeps the outline-overlap test below confined to explicitly-paired objects,
// leaving the sandbox's "nothing interacts unless the player says it does" invariant
// exactly where it was.
function vesselById(id){
  if(id==null) return null;
  const b=bodies.find(x=>x.id===id);
  return (b && b.shape==='vessel') ? b : null;
}
// The two sides of a pair and the conductance between them, or null if the pair
// cannot move anything. The rate depends on geometry the same way for heat and for
// mass: the contact area between the mediating body's outline and each vessel's
// rectangle (geometry.js §05.2e), limited by the SMALLER of the two -- conduction or
// flow through a wall is bottlenecked by whichever side touches less of it -- and the
// two interactions' own coefficients combined in series, like two conductors back to
// back. A background side has no outline and imposes no area limit of its own.
function exchangePair(it,jt){
  const bi=bodyIndex(it.body.id); if(bi<0) return null;
  const body=bodies[bi];
  if(it.vessel.id!=null && !vesselById(it.vessel.id)) return null;   // dangling reference
  if(jt.vessel.id!=null && !vesselById(jt.vessel.id)) return null;
  const A=vesselById(it.vessel.id), B=vesselById(jt.vessel.id);
  if(A===B) return null;                        // same vessel both sides, or both background
  const areaA = A ? contactArea(body,A) : Infinity;
  const areaB = B ? contactArea(body,B) : Infinity;
  const area  = Math.min(areaA,areaB);
  if(!(area>1e-9)) return null;                 // the body does not actually touch both
  const k = 1/(1/Math.max(it.k,1e-30)+1/Math.max(jt.k,1e-30));
  return {A,B,area,cond:k*area};
}
// Exact two-sided heat relaxation over one substep. Both the equilibrium temperature
// and the exponential approach to it fall straight out of solving the pair's linear
// ODE analytically:
//     C_A dT_A/dt = cond*(T_B - T_A),   C_B dT_B/dt = -cond*(T_B - T_A)
// so D = T_A - T_B decays as D0*exp(-lambda*h) with lambda = cond*(1/C_A + 1/C_B),
// while C_A T_A + C_B T_B is conserved -- giving both temperatures in closed form,
// unconditionally stable at any step size and incapable of overshooting equilibrium.
// A background side has infinite capacity: it never itself moves, and lambda reduces
// to cond/C on the real side alone (Newton's law of cooling toward a fixed bath).
//
// Nothing mechanical is touched at all. Volume is frozen, mass does not move, so each
// side's new `kap` is just its new temperature re-expressed at the unchanged volume
// (setVesselGasMT), and U = C*T exactly (geometry.js §05.2d gasC) makes the energy
// moved readable straight off the temperature change.
function applyHeatPair(it,jt,h){
  const p=exchangePair(it,jt); if(!p) return;
  const {A,B,cond}=p;
  const TA = A?gasT(A):sim.bg.T, TB = B?gasT(B):sim.bg.T;
  const D0 = TA-TB; if(D0===0) return;
  const CA = A?gasC(A):Infinity, CB = B?gasC(B):Infinity;
  let TA1=TA, TB1=TB;
  if(A && B){
    if(!(CA>0) || !(CB>0)) return;              // a vessel holding no gas has no capacity
    const D1=D0*Math.exp(-cond*(1/CA+1/CB)*h);
    const W=CA*TA+CB*TB;
    TA1=(W+CB*D1)/(CA+CB); TB1=(W-CA*D1)/(CA+CB);
  } else if(A){
    if(!(CA>0)) return;
    TA1 = TB + D0*Math.exp(-cond/CA*h);
  } else {
    if(!(CB>0)) return;
    TB1 = TA - D0*Math.exp(-cond/CB*h);
  }
  let dUA=0, dUB=0;
  if(A){ dUA=CA*(TA1-TA); setVesselGasMT(A, A.gas.mass, TA1); }
  if(B){ dUB=CB*(TB1-TB); setVesselGasMT(B, B.gas.mass, TB1); }
  // Between two real vessels dUA + dUB is zero by construction and the world total
  // does not move. Against the background it is exactly what the bath supplied, and
  // the ledger's own row (state.js sim.bathQ, hud.js §12.1) carries it so the running
  // total still reads flat.
  if(!A || !B) sim.bathQ += dUA+dUB;
  // Each interaction reports the rate into *its own* side, so a pair always reads as
  // one positive and one negative. A background side's share is whatever the pair
  // moved that the real side did not keep.
  const aId = A?A.id:null;
  it._rate += (it.vessel.id!=null ? (it.vessel.id===aId?dUA:dUB) : -(dUA+dUB))/h;
  jt._rate += (jt.vessel.id!=null ? (jt.vessel.id===aId?dUA:dUB) : -(dUA+dUB))/h;
}
// Move `dm` kilograms of gas from `src` to `dst` (either may be null, meaning the
// background), exactly conserving energy and linear momentum. This is the one place
// the exchange pass touches a mechanical quantity, and it does so because mass cannot
// move without its momentum -- refusing to carry it would be the *less* honest
// choice, not the more conservative one. It replaces the stripped implementation's
// separate thrust-like recoil force, which was a force law invented to stand in for
// exactly this transfer.
//
//   * Leaving. Removing gas from a rigid volume leaves what stays behind on its own
//     isentrope -- T ~ m^(gamma-1) at fixed V, i.e. kap ~ m^gamma -- which is the
//     exact integral of the open-system balance dU = -h dm, not an approximation of
//     it. The source's four rates are unchanged (material leaves at the body's own
//     velocity), so its momenta drop by precisely the departing mass's share, and the
//     kinetic energy that share carried leaves with it.
//   * Arriving. The crossing mass brings its source's translation and the energy it
//     took out of the source. Its spin and breathing do not survive the port, which
//     is a throttle, not a pipe with a shape: what those modes carried arrives as
//     energy rather than as momentum. The destination absorbs the linear momentum by
//     an inelastic merge and dilutes its own spin and length rates (I*w and mu*vlen
//     conserved as I and mu grow), and every joule the merge did not keep as kinetic
//     energy lands in the destination's internal energy -- which is what mixing
//     dissipation physically is.
//   * Climbing. Mass that moves between two different heights changes the scene's
//     gravitational potential, and the ledger counts a vessel's whole mass (hud.js
//     §12.1), so that change is real and has to be paid for: gas that goes up pays
//     m*g*dy out of its own internal energy, gas that comes down is warmed by it.
//     `peOf` reads height by exactly the rule the ledger does -- a static body's
//     potential is not tracked, so it contributes none -- which is what keeps the two
//     books identical rather than merely similar.
//
// Sum it: the source gives up (kinetic carried + energy carried), the destination
// takes on (kinetic absorbed + potential gained + all the rest), and the terms cancel
// identically. Against the background the same quantities cross sim.bathQ instead.
function vesselMassTransfer(src,dst,dm){
  const peOf = v => (v && !v.static && sim.gravity) ? sim.g*v.y : 0;
  if(!(dm>0)) return 0;
  if(src) dm=Math.min(dm, src.gas.mass);
  if(!(dm>1e-18)) return 0;
  let eCarried, keCarried=0, svx=0, svy=0;
  if(src){
    const m0=src.gas.mass, U0=gasU(src);
    src.gas.mass = m0-dm;
    src.gas.kap  = src.gas.kap*Math.pow(src.gas.mass/m0, src.gas.gamma);
    eCarried = U0-gasU(src);                    // == the enthalpy the crossing mass carries
    keCarried = 0.5*dm*(src.vx*src.vx+src.vy*src.vy)
              + 0.5*(dm*(src.bore*src.bore+src.len*src.len)/12)*src.w*src.w
              + 0.5*(dm/12)*src.vlen*src.vlen;
    svx=src.vx; svy=src.vy;
    refreshVessel(src);
  } else {
    // From the background: ambient air, at rest, arriving with the destination's own
    // gas properties. Composition is not tracked -- what crosses a port here is mass
    // and energy, never a second substance (VESSEL.md §V.11).
    const g=dst.gas.gamma;
    eCarried = dm*g*dst.gas.Rs*sim.bg.T/(g-1);
    sim.bathQ += eCarried;
  }
  // Off to the background: its enthalpy, its kinetic energy and the potential its
  // height carried all leave the tracked world together, and the bath took them.
  if(!dst){ sim.bathQ -= eCarried+keCarried+dm*peOf(src); return dm; }
  const m0=dst.mass, I0=dst.I, mu0=dst.mu;
  const ke0=0.5*m0*(dst.vx*dst.vx+dst.vy*dst.vy)+0.5*I0*dst.w*dst.w+0.5*mu0*dst.vlen*dst.vlen;
  const U0=gasU(dst);
  dst.gas.mass+=dm; refreshVessel(dst);
  dst.vx=(m0*dst.vx+dm*svx)/dst.mass; dst.vy=(m0*dst.vy+dm*svy)/dst.mass;
  dst.w   = I0 *dst.w   /dst.I;
  dst.vlen= mu0*dst.vlen/dst.mu;
  const ke1=0.5*dst.mass*(dst.vx*dst.vx+dst.vy*dst.vy)+0.5*dst.I*dst.w*dst.w+0.5*dst.mu*dst.vlen*dst.vlen;
  const ePE=dm*(peOf(dst)-peOf(src));
  setVesselGasU(dst, U0+eCarried+keCarried-(ke1-ke0)-ePE);
  return dm;
}
// Exact two-sided mass relaxation, the transfer analogue of applyHeatPair. Holding T
// and V frozen over the substep -- the same stance the whole pass takes -- each side's
// pressure per kilogram s = Rs*T/V is a constant and P = m*s, so
//     dm_A/dt = cond*(P_B - P_A),   dm_B/dt = -cond*(P_B - P_A)
// is the identical linear form as the heat ODE with s playing the role of 1/C: the
// pressure difference decays as D0*exp(-lambda*h) with lambda = cond*(s_A + s_B),
// total mass is exactly conserved, and the amount that crossed comes out in closed
// form. A background side holds a fixed P and the real side alone relaxes toward the
// mass that gives it ambient pressure.
//
// An evacuated vessel has no temperature of its own (gasT reads 0 at zero mass), so
// its `s` is taken at the other side's temperature -- a stand-in only for the rate
// law's slope. Its pressure is still exactly zero, because m = 0.
function applyFlowPair(it,jt,h){
  const p=exchangePair(it,jt); if(!p) return;
  const {A,B,cond}=p;
  const rawTA = A?gasT(A):sim.bg.T, rawTB = B?gasT(B):sim.bg.T;
  const Tref = Math.max(rawTA, rawTB, 1e-3);
  const TA = rawTA>1e-6?rawTA:Tref, TB = rawTB>1e-6?rawTB:Tref;
  const sA = A? A.gas.Rs*TA/vesselVol(A) : 0;
  const sB = B? B.gas.Rs*TB/vesselVol(B) : 0;
  const PA = A? A.gas.mass*sA : sim.bg.P;
  const PB = B? B.gas.mass*sB : sim.bg.P;
  let dm;                                        // signed: kilograms moved from B into A
  if(A && B){
    if(!(sA+sB>0)) return;
    const D0=PA-PB, D1=D0*Math.exp(-cond*(sA+sB)*h);
    dm = -(D0-D1)/(sA+sB);
  } else if(A){
    if(!(sA>0)) return;
    dm = (sim.bg.P/sA - A.gas.mass)*(1-Math.exp(-cond*sA*h));
  } else {
    if(!(sB>0)) return;
    dm = -(sim.bg.P/sB - B.gas.mass)*(1-Math.exp(-cond*sB*h));
  }
  if(!isFinite(dm) || dm===0) return;
  const moved = dm>0 ? vesselMassTransfer(B,A,dm) : -vesselMassTransfer(A,B,-dm);
  if(moved===0) return;
  const aId = A?A.id:null;                       // `moved` is signed into A's side
  it._rate += (it.vessel.id===aId ? moved : -moved)/h;
  jt._rate += (jt.vessel.id===aId ? moved : -moved)/h;
}
// One pass over every interaction pair. Interactions are grouped by the body they
// name; within a body, every unordered same-kind pair runs. Several simultaneous
// pairs are resolved by sequential operator splitting (each pair's relaxation sees
// the previous pair's already-updated state) rather than one joint solve -- each
// pair's own math stays exactly conservative, which is the property that matters.
function vesselExchangeStep(h){
  if(!interactions.length) return;
  for(const it of interactions) it._rate=0;
  const byBody=new Map();
  for(const it of interactions){ let a=byBody.get(it.body.id); if(!a){ a=[]; byBody.set(it.body.id,a); } a.push(it); }
  for(const [,list] of byBody){
    for(let i=0;i<list.length;i++) for(let j=i+1;j<list.length;j++){
      if(list[i].type!==list[j].type) continue;
      if(list[i].type==='heat') applyHeatPair(list[i],list[j],h);
      else                      applyFlowPair(list[i],list[j],h);
    }
  }
}

// ---- §08.1b · vesselGasStep (the gas force, as a discrete gradient) ----
// Advance one vessel's length rate under the gas, the atmosphere, and every other
// generalized force already accumulated on that coordinate (`Fother`).
//
// The gas is a potential (geometry.js §05.2d vesselUpotAt): internal energy plus
// the atmosphere's own P*V, both pure functions of length once the adiabat
// invariant and the gas mass are held fixed -- which mechanics never changes. So
// the vessel is an ordinary nonlinear spring, and the honest way to integrate a
// spring whose stiffness diverges is a DISCRETE GRADIENT: pick the force over the
// step so the work it does equals the potential drop *exactly*,
//
//     f_gas = -(Upot(Ln) - Upot(L0)) / (Ln - L0)
//
// paired with the trapezoidal length update §08.4 applies. Then
//
//     d(1/2 mu v^2) = 1/2 h f_tot (vn + v0) = f_tot * dLen
//
// identically, for ANY step size -- so kinetic + potential is conserved to
// rounding and the only net energy change is the work Fother genuinely did.
//
// Two properties follow that the earlier (stripped) implementation had to build by
// hand and never got exactly right. First, no minimum-volume floor is needed: as
// the trial length approaches zero, Upot diverges, so the residual below goes to
// -infinity and the root is always strictly positive -- the step *cannot* cross
// zero volume, at any closing speed. Second, there is no incremental dU = -P dV to
// book, hence no atmospheric flow-work channel and no rule about which substeps
// may credit a volume change: a state function has no increments to misattribute.
//
// The residual is monotone enough for bisection and its bracket is guaranteed
// (negative at the floor, positive far out), so no derivative and no failure mode.
const VESSEL_GAS_ITERS = 80;
function vesselGasStep(v, h, Fother){
  const invMu=v.invMu;
  if(!(invMu>0)){ v.vlen=0; return; }          // length-locked, or static
  const L0=v.len, v0=v._vlen0, U0=vesselUpotAt(v,L0);
  // The secant of Upot between L0 and the trial length; its limit as the trial
  // length approaches L0 is the ordinary force law, (P - P_bg) * capArea.
  const force=Ln=>{ const dL=Ln-L0;
    return Math.abs(dL)<1e-13 ? (gasP(v)-sim.bg.P)*vesselCapArea(v)
                              : -(vesselUpotAt(v,Ln)-U0)/dL; };
  const rateAt=Ln=>v0 + h*(Fother+force(Ln))*invMu;
  const resid =Ln=>Ln - L0 - h*(v0+rateAt(Ln))/2;
  let lo=VESSEL_MIN_LEN, hi=Math.max(L0,1)+Math.abs(v0)*h*4+1;
  for(let k=0;k<64 && resid(hi)<0;k++) hi*=2;
  if(resid(lo)>=0){
    // No root above the floor. Only reachable for a vessel with (near) no gas at
    // all -- a vacuum has no divergent pressure to arrest the caps, so it really
    // does close completely. Land on the floor rather than integrating through it,
    // and stop the closing motion instead of letting it run: this is the one path
    // in the file where the length coordinate's energy identity does not hold, and
    // §08.6 absorbs the difference the same way it does any other residue.
    v.vlen=Math.max(rateAt(lo),0); return;
  }
  for(let k=0;k<VESSEL_GAS_ITERS;k++){ const mid=0.5*(lo+hi); if(resid(mid)<0) lo=mid; else hi=mid; }
  v.vlen=rateAt(0.5*(lo+hi));
}

// ---- §08.1 · applied forces -> candidate velocities ----
let grab=null; // {bi, off} while mouse-dragging a body during play
function substep(h){
  const N=bodies.length;
  // Every vessel's derived inertia depends on its length, and its mass on the gas
  // sealed inside it, so both are refreshed before islands or the energy baseline
  // read them (geometry.js §05.2d). _vlen0 is this substep's starting length rate,
  // kept for the trapezoidal length update at §08.4 -- that update, paired with
  // §08.1b's discrete-gradient force, is what makes the gas exactly conservative.
  refreshVessels();
  // Which coordinates this scene has pinned, and which constraints are therefore
  // compiled away, follow from the constraints present -- recomputed here so an
  // edit between substeps takes effect, and before computeIslands reads them
  // (constraints.js §06.2b).
  refreshFrozen();
  // The thermodynamic pass (§08.0b), at frozen geometry and ahead of everything
  // mechanical -- including the energy/momentum snapshot below, which is exactly what
  // spares §08.6 a heat channel of its own. Whatever an interaction pair moved is
  // already standing in the state `preE` is about to read.
  vesselExchangeStep(h);
  for(const b of bodies) if(b.shape==='vessel') b._vlen0=b.vlen;
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

  // 1) accumulate applied forces, then integrate into candidate velocities v*
  // FL is the generalized-force accumulator for the fourth (vessel length)
  // coordinate, alongside FX/FY/TAU for the usual three. Gravity contributes
  // nothing to it: for a mass distribution symmetric about the vessel's centre the
  // gravitational potential is independent of length, so a vessel does not sag
  // under its own weight (VESSEL.md §V.6).
  const FX=new Array(N).fill(0), FY=new Array(N).fill(0), TAU=new Array(N).fill(0),
        FL=new Array(N).fill(0);
  for(let i=0;i<N;i++){ const b=bodies[i]; if(b.static) continue;
    if(sim.gravity) FY[i] += b.mass*(-sim.g);
    // Centrifugal coupling: I(len) = Alat + mu*len^2 depends on the length
    // coordinate, so d/dlen of the rotational kinetic energy is a real generalized
    // force -- 1/2 * dI/dlen * w^2 = mu*len*w^2 -- that stretches a spinning vessel.
    // Keeping it is what makes a spinning vessel conserve energy and angular
    // momentum exactly; dropping it breaks both (VESSEL.md §V.6).
    if(b.shape==='vessel') FL[i] += b.mu*b.len*b.w*b.w;
    if(grab && bodies[grab.bi]===b){
      const [wx,wy,rx,ry]=worldPt(b,epLocal(b,grab.off));
      const vpx=b.vx - b.w*ry, vpy=b.vy + b.w*rx;
      const K=40*b.mass, Cd=9*b.mass;
      // Same screen-space-capped pull as the pose-mode drag (§05.4, tools.js
      // §13.6): the spring's *reach* saturates with on-screen distance, so a
      // body a constraint won't let follow the cursor gets pulled by a bounded
      // force instead of one that keeps growing with however far the mouse has
      // strayed.
      const [px,py]=saturatingPull(wx,wy,mouseWorld[0],mouseWorld[1],DRAG_CAP_PX);
      const Fx=K*px-Cd*vpx, Fy=K*py-Cd*vpy;
      // Through the endpoint's own columns rather than by hand, so grabbing a
      // vessel anywhere but its mid-plane also pulls on its length coordinate --
      // the same virtual-work identity the spring pass below relies on.
      for(const c of epFrame({id:b.id, off:grab.off}).velCols(Fx,Fy)){
        FX[c[0]]+=c[1]; FY[c[0]]+=c[2]; TAU[c[0]]+=c[3]; FL[c[0]]+=c[4]||0;
      }
    }
  }
  // linear spring force elements: Hookean, F = k*(restLen-L) along the line
  // joining the two endpoints -- same two-endpoint frame as rod (twoPointFrame,
  // §06.1), reused as-is since a spring needs only L and the unit direction,
  // never phi (there is no weld/angle-lock row for a force element).
  for(const sp of springs){
    // A force (Fx,Fy) applied at a point is exactly that point's velCols
    // evaluated at (Fx,Fy) rather than a unit direction (virtual-work
    // identity: dir.v_point = sum(jx*vx+jy*vy+jw*w) holds for any dir, so it
    // holds component-wise for dir=(Fx,Fy) too).
    const A=epFrame(sp.a), B=epFrame(sp.b);
    const dx=A.wx-B.wx, dy=A.wy-B.wy, L=Math.hypot(dx,dy)||1e-9;
    const ux=dx/L, uy=dy/L;
    const Fmag=sp.k*(sp.restLen-L);
    const Fx=Fmag*ux, Fy=Fmag*uy;
    // No skip for a frozen body: its inverse masses are already zero, so the pose
    // terms integrate to nothing, while a pinned VESSEL's length column -- which is
    // not frozen with its pose -- still has to get its share.
    for(const [idx,cx,cy,cw,cl] of mergeCols([A.velCols(Fx,Fy), B.velCols(-Fx,-Fy)])){
      FX[idx]+=cx; FY[idx]+=cy; TAU[idx]+=cw; FL[idx]+=cl||0;
    }
  }
  // rotational spring force elements: torsional, tau = k*(restAngle-thRel)
  // between the two bodies' frame angles (background reads as a fixed
  // theta=0, mirroring rotSpringRelAngle, §06.6) -- a pure couple, no point
  // of application, so no torque-arm term the way rod needs one.
  for(const rs of rotSprings){
    const hasA=rs.a.id!=null, hasB=rs.b.id!=null;
    const ia=hasA?bodyIndex(rs.a.id):-1, ib=hasB?bodyIndex(rs.b.id):-1;
    const tau=rs.k*(rs.restAngle-rotSpringRelAngle(rs));
    if(hasA) TAU[ia]+=tau;
    if(hasB) TAU[ib]-=tau;
  }

  for(let i=0;i<N;i++){ const b=bodies[i];
    // Clearing a frozen pose's rates is bookkeeping, not dynamics -- the inverse
    // masses below are zero, so nothing would move them anyway; this just keeps the
    // inspector, the ledger and an exported scene from carrying a stale rate. There
    // is no `continue`: a vessel frozen in pose still has a length to advance.
    if(b.static){ b.vx=0;b.vy=0;b.w=0; }
    b.vx += h*b.invM*FX[i]; b.vy += h*b.invM*FY[i]; b.w += h*b.invI*TAU[i];
    // A vessel's length rate is advanced by §08.1b instead of a plain explicit step:
    // the gas force has to be solved implicitly over the step to be conservative,
    // and FL[i] (every *other* generalized force on this coordinate) rides along
    // inside that same solve so the energy identity covers the pair.
    if(b.shape==='vessel') vesselGasStep(b, h, FL[i]);
  }

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
  const rowJv=(colsIn)=>{ let s=0; for(const c of colsIn){ const b=bodies[c[0]];
    s+=c[1]*b.vx+c[2]*b.vy+c[3]*b.w+(c[4]||0)*(b.vlen||0); } return s; };
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
        const [tx,ty]=worldPt(tb0,epLocal(tb0,cb.tether.off)); T0=[tx,ty]; }
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
          const [tx,ty]=worldPt(tb0,epLocal(tb0,cb.tether.off)); T0=[tx,ty]; } else T0=[cb.tether.off[0],cb.tether.off[1]];
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
    constraints[ci]._rows=[];
    // A compiled-away constraint contributes nothing: it is the thing that froze
    // the coordinates it touches, so every column it would write is zero
    // (constraints.js §06.2b). Skipping it keeps a row of zeros out of the Schur
    // complement, where only the Tikhonov term would have kept it solvable.
    if(constraints[ci]._compiled) continue;
    for(const r of rowsFor(constraints[ci])){ constraints[ci]._rows.push(rows.length); rows.push(r); }
  }
  for(const cb of cables){
    if(cb._active){
      cb._rows.push(rows.length); rows.push({cols:cb._cols, C:cb._C});
    }
  }
  const m=rows.length;
  if(m>0){
    // per-row body->j map for K assembly
    // Four components per column, not three -- the fourth is the vessel length
    // coordinate (geometry.js §05.2d), zero on every ordinary body, so a row that
    // never touches a vessel is unaffected by its presence here.
    const maps=rows.map(r=>{ const mp=new Map(); for(const c of r.cols) mp.set(c[0],[c[1],c[2],c[3],c[4]||0]); return mp; });
    const Jv=new Array(m);
    for(let i=0;i<m;i++){ let s=0; for(const c of rows[i].cols){ const b=bodies[c[0]];
      s+=c[1]*b.vx+c[2]*b.vy+c[3]*b.w+(c[4]||0)*(b.vlen||0); } Jv[i]=s; }
    // K = J M^-1 J^T  (+ reg)
    const Kt=[]; for(let i=0;i<m;i++) Kt.push(new Array(m).fill(0));
    for(let i=0;i<m;i++){
      for(let j=i;j<m;j++){
        let s=0;
        for(const [bi,ji] of maps[i]){ const jj=maps[j].get(bi); if(!jj)continue;
          const im=invMdiag(bodies[bi]);
          s+=ji[0]*jj[0]*im[0]+ji[1]*jj[1]*im[1]+ji[2]*jj[2]*im[2]+ji[3]*jj[3]*im[3]; }
        Kt[i][j]=s; Kt[j][i]=s;
      }
      Kt[i][i]+=sim.reg;
    }
    const kb=sim.beta/sim.h;
    const rhs=new Array(m); for(let i=0;i<m;i++) rhs[i]=-(Jv[i]+kb*rows[i].C);
    const lam=solveLinear(Kt,rhs,m);
    // 4) apply impulses  v += M^-1 J^T lambda
    for(let i=0;i<m;i++){ const li=lam[i]; if(!li)continue;
      for(const c of rows[i].cols){ const b=bodies[c[0]];
        b.vx+=b.invM*c[1]*li; b.vy+=b.invM*c[2]*li; b.w+=b.invI*c[3]*li;
        if(c[4] && b.invMu) b.vlen+=b.invMu*c[4]*li; } }
    for(let ci=0;ci<constraints.length;ci++) constraints[ci]._lam=constraints[ci]._rows.map(ri=>lam[ri]);
    for(const cb of cables) cb._lam=cb._rows.map(ri=>lam[ri]);
  } else {
    for(const c of constraints) c._lam=[]; for(const cb of cables) cb._lam=[];
  }

  // ---- §08.4 · position integration ----
  // 4) integrate positions. The three rigid coordinates take the engine's usual
  //    symplectic-Euler step; a vessel's length takes the TRAPEZOIDAL one instead,
  //    averaging this substep's start and end rates. That is what completes §08.1b's
  //    exact work identity for the gas (and is the same correction §08.6 already
  //    applies to a free island's centre of mass, for the same reason).
  for(const b of bodies){
    b.x+=h*b.vx; b.y+=h*b.vy; b.th+=h*b.w;
    if(b.shape==='vessel') b.len=Math.max(VESSEL_MIN_LEN, b.len+h*(b._vlen0+b.vlen)/2);
  }
  // I(len) and the hw/hh render mirrors must reflect the new geometry before §08.6
  // reads this island's post-solve energy and effective inertia.
  refreshVessels();

  // ---- §08.6 · energy-conservation rescale ----
  // The Baumgarte-stabilized solve (§08.3) is only an approximate velocity
  // projection -- its leak grows with per-substep drift and with how many
  // rows are coupled, which is why a beta tuned to look flat on a double
  // pendulum still leaks visibly on longer chains (spec drift note, §04.3).
  // Rather than chase a bigger gain (a shorter correction lag traded for a
  // bigger per-step kick, and a lower instability ceiling on longer chains),
  // close the gap exactly: absent a live drag (which legitimately injects/
  // removes energy) the mechanical total entering this substep (preE) is
  // treated as an invariant -- per island (§08.0), not once over the whole
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
      // Free island: apply the exact-trapezoidal COM position correction
      // *before* post.pe is read below -- see the "Free island" comment
      // further down for the full derivation of why this exists at all.
      // Doing it before `post=energy(isl)` is what keeps this and the
      // internal-motion multiplicative rescale from double-correcting the
      // same gravity-vs-position mismatch: post.pe has to already reflect
      // the corrected geometry, or keTarget still carries the old (stale)
      // shortfall and the internal rescale "discovers" and re-fixes it a
      // second time on top of this one, injecting real spurious energy.
      if(!isl.anchored){
        const k0now=islandKinematics(isl.bodyIdx);
        const Ptx0=isl.P0[0], Pty0=isl.P0[1]+(sim.gravity? -sim.g*isl.M*h : 0);
        const V00=[isl.P0[0]/isl.M, isl.P0[1]/isl.M], Vt0=[Ptx0/isl.M, Pty0/isl.M];
        const comTargetX=isl.com0[0]+0.5*(V00[0]+Vt0[0])*h, comTargetY=isl.com0[1]+0.5*(V00[1]+Vt0[1])*h;
        const dComX=comTargetX-k0now.cx, dComY=comTargetY-k0now.cy;
        for(const i of isl.bodyIdx){ bodies[i].x+=dComX; bodies[i].y+=dComY; }
      }
      const post=energy(isl);
      // post.SPE (spring potential energy, hud.js §12.1) is subtracted the
      // same way post.pe is: it's a legitimate KE<->PE channel the spring
      // force itself already moved energy through in §08.1/§08.4, not a
      // discrepancy to fold back.
      // post.U (gas internal energy) and post.WA (the atmosphere's own P*V) are
      // subtracted alongside post.pe and post.SPE for exactly the same reason: they
      // are legitimate KE<->PE channels the gas force of §08.1b already moved energy
      // through, not discrepancies to fold back. Because both are state functions of
      // the current geometry (geometry.js §05.2d), there is nothing here to
      // accumulate or to decide about -- unlike an incremental dU = -P dV, they
      // cannot be credited twice or missed.
      const bankKey=islandBankKey(isl);
      const banked=ENERGY_BANK.get(bankKey)||0;
      const keTarget=isl.preE-post.pe-post.SPE-post.U-post.WA+banked;
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
          for(const i of isl.bodyIdx){ const b=bodies[i]; b.vx*=s; b.vy*=s; b.w*=s;
            if(b.shape==='vessel') b.vlen*=s; }
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
      // nothing else touching it -- no internal motion for the
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
      // A vessel's length rate belongs entirely to the internal/shape field: the
      // stretch is symmetric about the vessel's own centre, so it carries zero net
      // linear momentum (integral f dm = 0) and zero net angular momentum
      // (integral f*lat dm = 0). It therefore never enters islandKinematics at all,
      // and scaling it here can no more disturb the rigid fix above than scaling any
      // other relative motion can (VESSEL.md §V.6).
      let keInt=0; const internal=[];
      for(const i of isl.bodyIdx){ const b=bodies[i];
        const rx=b.x-now.cx, ry=b.y-now.cy;
        // Internal ("shape") velocity: current velocity minus the rigid
        // field implied by the island's *actual* current momentum (Vnow/
        // wnow) -- zero-net by construction, so it's pure relative motion,
        // never the momentum error itself, whatever s ends up being below.
        const ivx=b.vx-(Vnow[0]-wnow*ry), ivy=b.vy-(Vnow[1]+wnow*rx), iw=b.w-wnow;
        const ivl=(b.shape==='vessel')?b.vlen:0;
        internal.push([ivx,ivy,iw,rx,ry,ivl]);
        keInt+=0.5*b.mass*(ivx*ivx+ivy*ivy)+0.5*b.I*iw*iw;
        if(b.shape==='vessel') keInt+=0.5*b.mu*ivl*ivl;
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
      isl.bodyIdx.forEach((i,k)=>{ const b=bodies[i]; const [ivx,ivy,iw,rx,ry,ivl]=internal[k];
        b.vx=Vt[0]-wt*ry+s*ivx; b.vy=Vt[1]+wt*rx+s*ivy; b.w=wt+s*iw;
        if(b.shape==='vessel') b.vlen=s*ivl;
      });
    }
    // Every island present this substep either cleared or refreshed its own
    // ENERGY_BANK entry above; anything left over belongs to an island that
    // no longer exists (bodies/constraints edited or deleted mid-play), so
    // it's stale and would otherwise sit in the map forever.
    const liveBankKeys=new Set(islands.map(islandBankKey));
    for(const k of ENERGY_BANK.keys()) if(!liveBankKeys.has(k)) ENERGY_BANK.delete(k);
  }
}
