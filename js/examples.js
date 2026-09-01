// ============================================================================
//  §15 · EXAMPLES
//  Prebuilt machines wired from the panel buttons (§03.4 / §14.2). Each clears
//  the bench and assembles bodies + constraints from library primitives only.
//  loadExample(kind) dispatches on the data-ex key; search e.g.  kind==='crank'.
// ============================================================================
function loadExample(kind){
  bodies=[];constraints=[];gases=[];cables=[];uid=1;clearSelection();eHist.length=0;
  sim.gravity=true; const gc=document.getElementById('tgGrav'); if(gc) gc.checked=true;
  const dy=(x,y,r=0.38)=>{ const b=makeBody(x,y,r,false); bodies.push(b); return b; };
  // A plain (unwelded) rod between two bodies — free to rotate at both ends.
  const rod=(A,B)=>constraints.push(makeRodCon({id:A.id,off:[0,0]},{id:B.id,off:[0,0]},false,false));
  // A rod pinned to a fixed background point — the free-swinging pivot a
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
    // pins φ=atan2(...) directly (§06.5), which is singular if P ever passes
    // through the anchor — keep it well outside P's travel range.
    constraints.push(makeSlotCon({id:P.id,off:[0,0]}, {id:null,off:[P.x-10,P.y]}, false, true)); }
  else if(kind==='gasspring'){ const P=dy(0,2.8,0.4);
    // Rigid prismatic: both ends locked, so the rail also confines position
    // (not just rotation) — P is held on the vertical line through the
    // background point and can't rotate either.
    constraints.push(makeSlotCon({id:P.id,off:[0,0]}, {id:null,off:[P.x,P.y-1]}, true, true));
    gases.push({kind:'gas', a:{id:P.id,off:[0,0]}, head:{id:null, off:[0,0.5], dir:[0,1]},
                bore:1.0, n:5, gamma:1.4, T:1.0, Tinit:1.0, kappa:0, Tres:1.0, connected:false, sel:false}); }
  else if(kind==='skate'){ sim.gravity=false; if(gc) gc.checked=false;
    const b=dy(0,2.6,0.45);
    constraints.push({type:'knife', a:{id:b.id, off:[0.42,0]}, dir:[1,0], sel:false}); }
  else if(kind==='integrator'){ sim.gravity=false; if(gc) gc.checked=false;
    const A=dy(0,2.6,0.95);
    // A short background-welded rod into A's own centre: the weld on the
    // background end locks the rod's direction, so its free (pinned) far end
    // — which sits exactly at A's centre — is itself fixed in space, letting
    // A spin freely about that fixed axis. Replaces the old ground-pin tool.
    constraints.push(makeRodCon({id:null,off:[A.x-0.5,A.y]},{id:A.id,off:[0,0]},true,false));
    const B=dy(1.17,2.6,0.22);
    // Position-only rail for the CVT follower B (pin, free to spin) — same
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
