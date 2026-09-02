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
  // A vertical piston P sliding in a world-fixed cylinder, with a gas below it
  // (head at headOff, axis +y) and the auto piston<->cylinder hidden
  // prismatic link a real placement (tools.js §13.5) would also create --
  // shared by the gasspring and heatengine examples below.
  const gasPiston=(P,headOff,T,n=5,gamma=1.4,bore=1.0)=>{
    const link=makeSlotCon({id:P.id,off:[0,0]}, {id:null,off:headOff}, true, true);
    const g={kind:'gas', id:uid++, head:{id:null,off:headOff,dir:[0,1]}, piston:{id:P.id,off:[0,0]}, bore, n, gamma, T, sel:false};
    link.hidden=true; link.gasLink=g.id;
    constraints.push(link); gases.push(g);
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
  else if(kind==='gasspring'){ const P=dy(0,2.8,0.4);
    // No heat/flow interactions at all -- with nothing to exchange with, the
    // gas traverses its adiabat purely through the mechanical P·dV term
    // (physics.js §08.5), exactly the "kappa=0" gas spring the old reservoir
    // model needed a special-cased toggle for.
    gasPiston(P,[P.x,P.y-1],1.0); }
  else if(kind==='heatengine'){ const P=dy(0,2.8,0.4);
    // Same piston/cylinder as the gas spring above, but started hot (T=2,
    // background defaults to T=1) and with a heat *and* a flow interaction
    // from the piston plate to both the gas and the background -- the plate
    // mediates both couplings (spec: two interactions sharing a body couple
    // whatever they each name), so heat and mass both leak from the hot gas
    // out to atmosphere at a finite rate, and the piston settles as they do.
    const g=gasPiston(P,[P.x,P.y-1],2.0);
    heatInteractions.push({kind:'heat', id:uid++, bodyId:P.id, gasId:g.id, k:1.5, sel:false});
    heatInteractions.push({kind:'heat', id:uid++, bodyId:P.id, gasId:null, k:1.5, sel:false});
    flowInteractions.push({kind:'flow', id:uid++, bodyId:P.id, gasId:g.id, k:0.4, sel:false});
    flowInteractions.push({kind:'flow', id:uid++, bodyId:P.id, gasId:null, k:0.4, sel:false}); }
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
