// ============================================================================
//  §15 · EXAMPLES
//  Prebuilt machines wired from the panel buttons (§03.4 / §14.2). Each clears
//  the bench and assembles bodies + constraints from library primitives only.
//  loadExample(kind) dispatches on the data-ex key; search e.g.  kind==='crank'.
// ============================================================================
function loadExample(kind){
  // springs/rotSprings are cleared alongside the rest: leaving them behind would
  // leave force elements pointing at body ids the new scene has reused or dropped,
  // which epFrame resolves to a missing body.
  bodies=[];constraints=[];cables=[];springs=[];rotSprings=[];uid=1;clearSelection();eHist.length=0;
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
  else if(kind==='gasspring'){
    // A vessel standing on the ground: its lower cap -- the material plane
    // f = -1/2 -- is welded to a fixed world point, which pins that cap and locks
    // the vessel's rotation, leaving the length as the only free coordinate.
    //
    // Nothing here is a "gas spring" primitive. The oscillation is the vessel's own
    // gas potential (geometry.js §05.2d) against the weight the constraint transfers
    // onto the length coordinate: with the cap pinned, the centre of mass rises by
    // half of any extension, so gravity acquires a generalized force on `len` that
    // it does not have for a free vessel. The adiabat is not imposed either -- an
    // isolated gas simply never changes its adiabat invariant.
    const v=makeVessel(0,1.1,0.5,1.2,false); bodies.push(v);
    const capY=v.y-v.len/2;
    constraints.push(makeRodCon({id:null,off:[0,capY-0.35]},{id:v.id,off:[0,-0.5]},true,true));
  }
  else if(kind==='spinvessel'){
    // A free vessel with nothing attached, spinning in zero gravity. I(len) grows as
    // it stretches, so the spin slows and the centrifugal generalized force
    // (physics.js §08.1) trades against the gas -- the vessel breathes. Angular
    // momentum and total energy both hold flat while it does, which is the point:
    // the same constant mu governs the length inertia and the len^2 term in I.
    sim.gravity=false; if(gc) gc.checked=false;
    const v=makeVessel(0,2.6,0.4,1.0,false); bodies.push(v);
    v.w=9.0;
  }
  else if(kind==='cable'){ const S=makeBody(0,4.6,0.4,true); bodies.push(S); const m=dy(0.9,4.4,0.32);
    const dvx=m.x-S.x, dvy=m.y-S.y; const d=Math.hypot(dvx,dvy);
    const Lfree=d>S.r?Math.sqrt(d*d-S.r*S.r):0;
    const cb0={type:'cable', tether:{id:m.id, off:[0,0]}, spool:{id:S.id},
               localAngle:Math.atan2(dvy,dvx)-S.th, spoolAngle:0, Ltot:Lfree, sel:false};
    cables.push(cb0); }
  saveState();
  cam.x=0;cam.y=2.6;cam.scale=64;
}
