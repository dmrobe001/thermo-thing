// ============================================================================
//  §15 · EXAMPLES
//  Prebuilt machines wired from the panel buttons (§03.4 / §14.2). Each clears
//  the bench and assembles bodies + constraints from library primitives only.
//  loadExample(kind) dispatches on the data-ex key; search e.g.  kind==='crank'.
// ============================================================================
function loadExample(kind){
  bodies=[];constraints=[];gases=[];cables=[];uid=1;clearSelection();eHist.length=0;
  sim.gravity=true; const gc=document.getElementById('tgGrav'); if(gc) gc.checked=true;
  const st=(x,y,r=0.16)=>{ const b=makeBody(x,y,r,true); bodies.push(b); return b; };
  const dy=(x,y,r=0.38)=>{ const b=makeBody(x,y,r,false); bodies.push(b); return b; };
  const rod=(A,B)=>{ const [ax,ay]=[A.x,A.y],[bx,by]=[B.x,B.y];
    constraints.push({type:'rod',a:{id:A.id,off:[0,0]},b:{id:B.id,off:[0,0]},len:Math.hypot(ax-bx,ay-by),sel:false}); };
  if(kind==='pendulum'){ const s=st(0,4.4); const b=dy(2.6,4.4); rod(s,b); }
  else if(kind==='double'){ const s=st(0,4.6); const b1=dy(1.8,4.6,0.32); const b2=dy(3.6,4.6,0.32); rod(s,b1); rod(b1,b2); }
  else if(kind==='fourbar'){ const pL=st(-1.6,1.2), pR=st(1.6,1.2);
    const a=dy(-1.2,2.8,0.3), b=dy(1.3,2.9,0.3); rod(pL,a); rod(a,b); rod(pR,b); }
  else if(kind==='crank'){ const S=st(-1.6,2.4); const pin=dy(-1.0,2.4,0.24); const P=dy(1.7,2.4,0.34);
    rod(S,pin); rod(pin,P);
    constraints.push({type:'slot', a:{id:P.id,off:[0,0]}, line:{id:null, off:[P.x,P.y], dir:[1,0]},
                      lockRot:false, restAng:P.th, sel:false}); }
  else if(kind==='gasspring'){ const P=dy(0,2.8,0.4);
    constraints.push({type:'slot', a:{id:P.id,off:[0,0]}, line:{id:null, off:[P.x,P.y], dir:[0,1]},
                      lockRot:true, restAng:P.th, sel:false});
    gases.push({kind:'gas', a:{id:P.id,off:[0,0]}, head:{id:null, off:[0,0.5], dir:[0,1]},
                bore:1.0, n:5, gamma:1.4, T:1.0, Tinit:1.0, kappa:0, Tres:1.0, connected:false, sel:false}); }
  else if(kind==='skate'){ sim.gravity=false; if(gc) gc.checked=false;
    const b=dy(0,2.6,0.45);
    constraints.push({type:'knife', a:{id:b.id, off:[0.42,0]}, dir:[1,0], sel:false}); }
  else if(kind==='integrator'){ sim.gravity=false; if(gc) gc.checked=false;
    const A=dy(0,2.6,0.95);
    constraints.push({type:'ground', a:{id:A.id, off:[0,0]}, world:[A.x,A.y], sel:false});
    const B=dy(1.17,2.6,0.22);
    constraints.push({type:'slot', a:{id:B.id,off:[0,0]}, line:{id:null, off:[A.x,A.y], dir:[1,0]}, lockRot:false, restAng:0, sel:false});
    constraints.push({type:'cvt', a:{id:A.id}, b:{id:B.id}, sel:false}); }
  else if(kind==='cable'){ const S=st(0,4.6,0.4); const m=dy(0.9,4.4,0.32);
    cables.push({type:'cable', tether:{id:m.id, off:[0,0]}, spool:{id:S.id}, side:1, Ltot:2.6, wrap:0, sel:false}); }
  saveState();
  cam.x=0;cam.y=2.6;cam.scale=64;
}
