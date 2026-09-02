// ============================================================================
//  §15 · EXAMPLES
//  Prebuilt machines wired from the panel buttons (§03.4 / §14.2). Each clears
//  the bench and assembles bodies + constraints from library primitives only.
//  loadExample(kind) dispatches on the data-ex key; search e.g.  kind==='crank'.
// ============================================================================
function loadExample(kind){
  bodies=[];constraints=[];gases=[];heatInteractions=[];flowInteractions=[];cables=[];uid=1;clearSelection();eHist.length=0;
  // uid restarts at 1 above, so a fresh scene's bodies can reuse ids a prior
  // scene's islands used as their ENERGY_BANK key (physics.js §08.6) --
  // clear it so a stale banked correction can never leak into an unrelated
  // scene's very first substep.
  ENERGY_BANK.clear();
  sim.gravity=true; const gc=document.getElementById('tgGrav'); if(gc) gc.checked=true;
  const dy=(x,y,r=0.38)=>{ const b=makeBody(x,y,r,false); bodies.push(b); return b; };
  // A plain (unwelded) rod between two bodies -- free to rotate at both ends.
  const rod=(A,B)=>constraints.push(makeRodCon({id:A.id,off:[0,0]},{id:B.id,off:[0,0]},false,false));
  // A rod pinned to a fixed background point -- the free-swinging pivot a
  // pendulum needs (both ends unwelded, unlike the tool's rigid-strut default).
  const rodBG=(wx,wy,B)=>constraints.push(makeRodCon({id:null,off:[wx,wy]},{id:B.id,off:[0,0]},false,false));
  // A vertical piston vessel in a cylinder (head at headOff, axis +y,
  // initial length `len`) -- the redesigned single-internal-coordinate
  // piston (DEVELOPMENT.md §6.1): a real, synthetic COM body (hidden from
  // the player as a separate body, drawn as part of the one vessel
  // rectangle) mounted to the head frame by the same `gasmount` constraint
  // a real placement (tools.js §13.5) would also create, its mass forced to
  // the gas's own full mass (see geometry.js syncVesselComMass), plus a
  // static, kinematically-slaved marker body at the true cap point. The
  // head is anchored to the world by a visible, both-ends-welded rod --
  // DEVELOPMENT.md §4.1's "how a body is anchored to the world" (there is
  // no bare world-anchored frame, no standalone static toggle) -- so every
  // part of this example is something the player could place and connect
  // with the tool palette, not a hidden hard-coded anchor with nothing on
  // screen explaining why the vessel doesn't fall. Shared by the gasspring
  // and heatengine examples below; returns the vessel, with `g.head.id`/
  // `g.piston.id` already naming the two (real, if hidden) boundary bodies.
  const gasPiston=(headOff,T,mass=5,gamma=1.4,bore=1.0,len=1.0)=>{
    const headBody=makeRectBody(headOff[0], headOff[1], bore/2, 0.06, false);
    headBody.synthetic=true; bodies.push(headBody);
    const comBody=makeRectBody(headOff[0], headOff[1]+len*0.5, bore/2, 0.06, false);
    comBody.synthetic=true; bodies.push(comBody);
    const capMarker=makeRectBody(headOff[0], headOff[1]+len, bore/2, 0.06, true);
    capMarker.synthetic=true; bodies.push(capMarker);
    const head={id:headBody.id, off:[0,0], dir:[0,1]}, piston={id:capMarker.id, off:[0,0]}, com={id:comBody.id};
    const g={kind:'gas', id:uid++, head, piston, com, bore, mass, gamma, T, sep:len, sepRate:0, lockedField:'P', sel:false};
    const mount=makeGasMountCon(g);
    constraints.push(mount); gases.push(g);
    syncVesselComMass(g); syncVesselMarkers(g);
    constraints.push(makeRodCon({id:null,off:[headOff[0],headOff[1]-0.4]}, {id:headBody.id,off:[0,0]}, true, true));
    return g;
  };
  if(kind==='pendulum'){ const b=dy(2.6,4.4); rodBG(0,4.4,b); }
  else if(kind==='double'){ const b1=dy(1.8,4.6,0.32); const b2=dy(3.6,4.6,0.32); rodBG(0,4.6,b1); rod(b1,b2); }
  else if(kind==='fourbar'){
    const a=dy(-1.2,2.8,0.3), b=dy(1.3,2.9,0.3); rodBG(-1.6,1.2,a); rod(a,b); rodBG(1.6,1.2,b); }
  else if(kind==='crank'){ const pin=dy(-1.0,2.4,0.24); const P=dy(1.7,2.4,0.34);
    rodBG(-1.6,2.4,pin); rod(pin,P);
    // Position-only rail: P (pin, free to rotate) confined to the horizontal
    // line through a background point that's prismatic. A single locked end
    // pins phi=atan2(...) directly (§06.5), which is singular if P ever passes
    // through the anchor -- keep it well outside P's travel range.
    constraints.push(makeSlotCon({id:P.id,off:[0,0]}, {id:null,off:[P.x-10,P.y]}, false, true)); }
  else if(kind==='gasspring'){
    // No heat/flow interactions at all -- with nothing to exchange with, the
    // gas traverses its adiabat purely through the mechanical P·dV term
    // (physics.js §08.5), exactly the "kappa=0" gas spring the old reservoir
    // model needed a special-cased toggle for.
    gasPiston([0,1.8],1.0); }
  else if(kind==='heatengine'){
    // Same piston/cylinder as the gas spring above, but started hot (T=2,
    // background defaults to T=1) and with a heat-and-mass conduit -- a
    // real, ordinary, player-visible body (not the vessel's own hidden
    // wall) straddling the bore near the fixed head end: half of it
    // overlaps the gas (bodyGasOverlapArea, geometry.js §05.2c), half
    // sticks out into open air. Two heat interactions and two flow
    // interactions share that one body -- one of each pair naming the
    // gas, the other naming the background (gasId:null) -- so the conduit
    // mediates both couplings (spec: two interactions sharing a body
    // couple whatever they each name), exactly like a heat-conducting
    // plug through a real cylinder wall. It's placed near the head (not
    // riding the moving piston) so the overlap stays valid across the
    // piston's whole stroke without needing its own mechanical attachment
    // -- welded to the world by the same visible-anchor pattern gasPiston
    // uses for the head, rather than a hidden coupling with nothing shown
    // connecting the vessel to the background (spec: "should be impossible").
    const g=gasPiston([0,1.8],2.0);
    const conduit=makeRectBody(g.bore/2, 2.05, 0.15, 0.15, false);
    bodies.push(conduit);
    constraints.push(makeRodCon({id:null,off:[g.bore/2+0.5,2.05]}, {id:conduit.id,off:[0,0]}, true, true));
    heatInteractions.push({kind:'heat', id:uid++, bodyId:conduit.id, gasId:g.id, k:1.5, sel:false});
    heatInteractions.push({kind:'heat', id:uid++, bodyId:conduit.id, gasId:null, k:1.5, sel:false});
    flowInteractions.push({kind:'flow', id:uid++, bodyId:conduit.id, gasId:g.id, k:0.4, sel:false});
    flowInteractions.push({kind:'flow', id:uid++, bodyId:conduit.id, gasId:null, k:0.4, sel:false}); }
  else if(kind==='skate'){ sim.gravity=false; if(gc) gc.checked=false;
    const b=dy(0,2.6,0.45);
    constraints.push({type:'knife', a:{id:b.id, off:[0.42,0]}, dir:[1,0], sel:false}); }
  else if(kind==='integrator'){ sim.gravity=false; if(gc) gc.checked=false;
    const A=dy(0,2.6,0.95);
    // A short background-welded rod into A's own centre: the weld on the
    // background end locks the rod's direction, so its free (pinned) far end
    // -- which sits exactly at A's centre -- is itself fixed in space, letting
    // A spin freely about that fixed axis. Replaces the old ground-pin tool.
    constraints.push(makeRodCon({id:null,off:[A.x-0.5,A.y]},{id:A.id,off:[0,0]},true,false));
    const B=dy(1.17,2.6,0.22);
    // Position-only rail for the CVT follower B (pin, free to spin) -- same
    // single-locked-background pattern as the crank example above.
    constraints.push(makeSlotCon({id:B.id,off:[0,0]}, {id:null,off:[A.x-10,A.y]}, false, true));
    constraints.push({type:'cvt', a:{id:A.id}, b:{id:B.id}, sel:false}); }
  else if(kind==='cable'){ const S=makeBody(0,4.6,0.4,true); bodies.push(S); const m=dy(0.9,4.4,0.32);
    const dvx=m.x-S.x, dvy=m.y-S.y; const d=Math.hypot(dvx,dvy);
    const Lfree=d>S.r?Math.sqrt(d*d-S.r*S.r):0;
    const cb0={type:'cable', tether:{id:m.id, off:[0,0]}, spool:{id:S.id},
               localAngle:Math.atan2(dvy,dvx)-S.th, spoolAngle:0, Ltot:Lfree, sel:false};
    cables.push(cb0); }
  saveState();
  cam.x=0;cam.y=2.6;cam.scale=64;
}
