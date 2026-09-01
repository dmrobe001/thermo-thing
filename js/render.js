// ============================================================================
//  §11 · RENDER
//  All canvas drawing. §11.1 is the orchestrator; the rest are element painters
//  it calls in back-to-front order. None of these mutate sim state.
//    §11.1  render            (per-frame scene orchestrator; sizing + draw order)
//    §11.2  background        (drawGrid, drawAxes)
//    §11.3  bodies            (drawBody, jointDot)
//    §11.4  gas & cable       (drawCable, drawGas, drawGasForce)
//    §11.4b springs           (drawSpring, drawRotSpring, drawSpiral -- force elements)
//    §11.4c heat & flow overlays (drawHeatInteraction, drawFlowInteraction)
//    §11.5  constraints       (drawConstraint + drawRim, beltTangents)
//    §11.6  reaction vectors  (drawReaction -- the lambda arrows)
//    §11.7  interaction overlays (drawPending, drawPreview, drawHandles, drawSnap)
// ============================================================================
// ---- §11.1 · render (scene orchestrator) ----
function render(){
  const dpr=window.devicePixelRatio||1;
  if(cv.width!==Math.round(W()*dpr)||cv.height!==Math.round(H()*dpr)){
    cv.width=Math.round(W()*dpr); cv.height=Math.round(H()*dpr);
  }
  ctx.setTransform(dpr,0,0,dpr,0,0);
  ctx.clearRect(0,0,W(),H());
  if(sim.showGrid) drawGrid();
  drawAxes();
  violCount=0;
  // Bodies paint first so every constraint/force-element visualization --
  // joint dots, tether points, rods, rims -- draws in front of them instead
  // of being occluded by a body's opaque fill. Within that, larger bodies
  // paint before smaller ones so a small body nested against/inside a big
  // one is never hidden by it (a sort copy -- `bodies` itself stays in
  // creation order, which pick/hit-testing still relies on).
  for(const b of [...bodies].sort((p,q)=>bodyExtentR(q)-bodyExtentR(p))) drawBody(b);
  for(const g of gases) drawGas(g);
  for(const cb of cables) drawCable(cb);
  for(const sp of springs) drawSpring(sp);
  for(const rs of rotSprings) drawRotSpring(rs);
  // A gas's auto-created piston<->cylinder prismatic (tools.js §13.5, marked
  // `hidden`) is deliberately not visualized (spec: "a mutually prismatic
  // interaction which is not visualized") -- it's bookkeeping for the gas,
  // not a joint the player placed.
  for(const con of constraints) if(!con.hidden) drawConstraint(con);
  for(const hi of heatInteractions) drawHeatInteraction(hi);
  for(const fi of flowInteractions) drawFlowInteraction(fi);
  drawHandles();
  if(sim.showForces && sim.running){ for(const con of constraints) if(!con.hidden) drawReaction(con); for(const g of gases) drawGasForce(g); }
  if(pending) drawPending();
  if(bodyPreview) drawPreview();
  drawSnap();
  document.getElementById('hint').style.display = bodies.length? 'none':'block';
}

// ---- §11.2 · background (grid + axes) ----
function drawGrid(){
  const step=1; // world units
  const [x0,y0]=s2w(0,H()), [x1,y1]=s2w(W(),0);
  ctx.lineWidth=1;
  for(let gx=Math.floor(x0);gx<=Math.ceil(x1);gx++){
    const [sx]=w2s(gx,0);
    ctx.strokeStyle = gx%5===0? 'rgba(255,255,255,.07)':'rgba(255,255,255,.032)';
    ctx.beginPath();ctx.moveTo(sx,0);ctx.lineTo(sx,H());ctx.stroke();
  }
  for(let gy=Math.floor(y0);gy<=Math.ceil(y1);gy++){
    const [,sy]=w2s(0,gy);
    ctx.strokeStyle = gy%5===0? 'rgba(255,255,255,.07)':'rgba(255,255,255,.032)';
    ctx.beginPath();ctx.moveTo(0,sy);ctx.lineTo(W(),sy);ctx.stroke();
  }
}
function drawAxes(){
  const [,sy0]=w2s(0,0);
  ctx.strokeStyle='rgba(255,255,255,.14)';ctx.lineWidth=1;
  ctx.beginPath();ctx.moveTo(0,sy0);ctx.lineTo(W(),sy0);ctx.stroke();
}

// ---- §11.3 · bodies (drawBody, jointDot) ----
function drawBody(b){
  const [sx,sy]=w2s(b.x,b.y);
  if(b.shape==='rect'){
    // Corners in screen space, via worldPt (world-frame rotation, same math
    // every other rim-point in the file already uses) rather than ctx.rotate
    // -- keeps the sign convention identical to worldPt/w2s everywhere else
    // instead of re-deriving how canvas rotation interacts with w2s's y-flip.
    const corners=[[-b.hw,-b.hh],[b.hw,-b.hh],[b.hw,b.hh],[-b.hw,b.hh]]
      .map(o=>{ const [wx,wy]=worldPt(b,o); return w2s(wx,wy); });
    const path=()=>{ ctx.beginPath(); corners.forEach((p,i)=> i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1])); ctx.closePath(); };
    path();
    if(b.static){
      ctx.fillStyle='#2b323f'; ctx.fill();
      ctx.save(); path(); ctx.clip();
      const xs=corners.map(p=>p[0]), ys=corners.map(p=>p[1]);
      const x0=Math.min(...xs), x1=Math.max(...xs), y0=Math.min(...ys), y1=Math.max(...ys), span=(x1-x0)+(y1-y0);
      ctx.strokeStyle='rgba(255,255,255,.10)';ctx.lineWidth=1;
      for(let i=-span;i<span;i+=6){ctx.beginPath();ctx.moveTo(x0+i,y0);ctx.lineTo(x0+i+span,y0+span);ctx.stroke();}
      ctx.restore();
    } else {
      ctx.fillStyle='#cdd5e0'; ctx.fill();
    }
    const hoverOn = hover===b;
    const resizeOn = hoverHandle && hoverHandle.kind==='resize' && hoverHandle.b===b;
    ctx.lineWidth = resizeOn?3.5 : b.sel?2.5: hoverOn?2:1.4;
    ctx.strokeStyle = resizeOn?'#8fd0ff' : b.sel? '#5aa9f0': hoverOn? '#8fc4f7':'#7c8798';
    path(); ctx.stroke();
    const [tx,ty]=w2s(b.x+Math.cos(b.th)*b.hw, b.y+Math.sin(b.th)*b.hw);
    ctx.strokeStyle = b.static?'rgba(255,255,255,.25)':'#5d6878';
    ctx.beginPath();ctx.moveTo(sx,sy);ctx.lineTo(tx,ty);ctx.stroke();
    ctx.fillStyle=b.static?'rgba(255,255,255,.3)':'#5d6878';
    ctx.beginPath();ctx.arc(sx,sy,2,0,Math.PI*2);ctx.fill();
    return;
  }
  const rr=b.r*cam.scale;
  ctx.beginPath();ctx.arc(sx,sy,rr,0,Math.PI*2);
  if(b.static){
    ctx.fillStyle='#2b323f'; ctx.fill();
    ctx.save();ctx.clip();
    ctx.strokeStyle='rgba(255,255,255,.10)';ctx.lineWidth=1;
    for(let i=-rr*2;i<rr*2;i+=6){ctx.beginPath();ctx.moveTo(sx+i,sy-rr);ctx.lineTo(sx+i+rr*2,sy+rr);ctx.stroke();}
    ctx.restore();
  } else {
    ctx.fillStyle='#cdd5e0'; ctx.fill();
  }
  const hoverOn = hover===b;
  const resizeOn = hoverHandle && hoverHandle.kind==='resize' && hoverHandle.b===b;
  ctx.lineWidth = resizeOn?3.5 : b.sel?2.5: hoverOn?2:1.4;
  ctx.strokeStyle = resizeOn?'#8fd0ff' : b.sel? '#5aa9f0': hoverOn? '#8fc4f7':'#7c8798';
  ctx.stroke();
  // orientation tick + centre dot (shows spin)
  const [tx,ty]=w2s(b.x+Math.cos(b.th)*b.r, b.y+Math.sin(b.th)*b.r);
  ctx.strokeStyle = b.static?'rgba(255,255,255,.25)':'#5d6878';
  ctx.beginPath();ctx.moveTo(sx,sy);ctx.lineTo(tx,ty);ctx.stroke();
  ctx.fillStyle=b.static?'rgba(255,255,255,.3)':'#5d6878';
  ctx.beginPath();ctx.arc(sx,sy,2,0,Math.PI*2);ctx.fill();
}

function jointDot(x,y,col){ const [sx,sy]=w2s(x,y);
  ctx.fillStyle='#13161c';ctx.beginPath();ctx.arc(sx,sy,4.5,0,Math.PI*2);ctx.fill();
  ctx.strokeStyle=col;ctx.lineWidth=2;ctx.beginPath();ctx.arc(sx,sy,4.5,0,Math.PI*2);ctx.stroke(); }

// One endpoint of a rod or slot: a ground hatch if it's background-anchored,
// plus a square (locked -- rotation-locked to the other endpoint's line, i.e.
// rod's "weld" or slot's "prismatic") or a round joint dot (pinned -- free to
// rotate).
function drawEndMarker(x,y,locked,isBackground,col){
  const [sx,sy]=w2s(x,y);
  if(isBackground){
    ctx.strokeStyle=col;ctx.lineWidth=1.5;
    ctx.beginPath();ctx.moveTo(sx,sy+4);ctx.lineTo(sx-7,sy+14);ctx.lineTo(sx+7,sy+14);ctx.closePath();ctx.stroke();
    for(let i=-7;i<=7;i+=4){ctx.beginPath();ctx.moveTo(sx+i,sy+14);ctx.lineTo(sx+i-4,sy+19);ctx.stroke();}
  }
  if(locked){
    ctx.fillStyle='#13161c';ctx.strokeStyle=col;ctx.lineWidth=2;
    ctx.beginPath();ctx.rect(sx-5,sy-5,10,10);ctx.fill();ctx.stroke();
  } else {
    jointDot(x,y,col);
  }
}

// ---- §11.4 · gas & cable (drawCable, drawGas, drawGasForce) ----
function drawCable(cb){
  const f=cableFrame(cb); if(!f)return;
  const lamMag = cb._lam&&cb._lam.length ? Math.hypot(...cb._lam) : 0;
  const taut = lamMag>1e-5;
  const hoverOn = hover===cb;
  const col = cb.sel? '#5aa9f0' : hoverOn? '#8fc4f7' : (taut? '#e0c060' : '#8a94a6');
  const [tx,ty]=w2s(f.T[0],f.T[1]);
  ctx.strokeStyle=col; ctx.lineWidth= taut?2.5:1.8;
  // Straight segment: tether T -> separation point Q
  ctx.beginPath(); ctx.moveTo(tx,ty);
  const [qx,qy]=w2s(f.Qx,f.Qy); ctx.lineTo(qx,qy); ctx.stroke();
  // Wound arc: separation point Q -> anchor A (if any winding present)
  const sweep=Math.abs(f.windAngle);
  if(sweep > 1e-4){
    const sign = f.windAngle >= 0 ? 1 : -1;
    const sweepDraw = Math.min(sweep, Math.PI*8);  // cap visual arc at 4 turns
    const nSeg = Math.max(24, Math.round(sweepDraw/Math.PI*12));
    ctx.lineWidth=1.8; ctx.beginPath();
    for(let i=0;i<=nSeg;i++){
      const a = f.anchorAngle + sign*(sweep - sweepDraw*i/nSeg);   // Q->(capped toward A)
      const [sx,sy]=w2s(f.S.x+f.rs*Math.cos(a), f.S.y+f.rs*Math.sin(a));
      i?ctx.lineTo(sx,sy):ctx.moveTo(sx,sy); }
    ctx.stroke();
  }
  jointDot(f.T[0],f.T[1],col);
  // anchor handle -- always visible when selected; brighter + larger when hovered
  if(cb.sel){
    const handleHover = hoverHandle && hoverHandle.kind==='cable' && hoverHandle.cb===cb;
    const rad = handleHover?6.5:5;
    const [ax2,ay2]=w2s(f.Ax,f.Ay);
    ctx.fillStyle='#13161c'; ctx.beginPath(); ctx.arc(ax2,ay2,rad,0,Math.PI*2); ctx.fill();
    ctx.strokeStyle=handleHover?'#8fd0ff':col; ctx.lineWidth=2; ctx.beginPath(); ctx.arc(ax2,ay2,rad,0,Math.PI*2); ctx.stroke();
  }
}
function drawGas(g){
  const f=gasFrame(g); const sel=g.sel; const hoverOn=hover===g;
  const nrm=[-f.dW[1],f.dW[0]]; const hw=g.bore*0.5;
  const H1=[f.hx+nrm[0]*hw, f.hy+nrm[1]*hw], H2=[f.hx-nrm[0]*hw, f.hy-nrm[1]*hw];
  const P1=[f.pax+nrm[0]*hw, f.pay+nrm[1]*hw], P2=[f.pax-nrm[0]*hw, f.pay-nrm[1]*hw];
  const P=g._P || g.n*g.T/(g.bore*f.xc);
  // fill tinted by pressure, hue warmed by temperature
  const a=Math.max(0.05, Math.min(0.55, P*0.03));
  const warm=Math.max(0, Math.min(1, (g.T-0.4)/2));
  ctx.beginPath();
  [H1,P1,P2,H2].forEach((p,i)=>{ const [X,Y]=w2s(p[0],p[1]); i?ctx.lineTo(X,Y):ctx.moveTo(X,Y); });
  ctx.closePath();
  ctx.fillStyle=`rgba(${Math.round(210+warm*40)},${Math.round(150-warm*70)},${Math.round(90-warm*50)},${a})`;
  ctx.fill();
  // cylinder walls + closed end
  const col=sel?'#5aa9f0':hoverOn?'#8fc4f7':'#8a94a6'; ctx.strokeStyle=col; ctx.lineWidth=2;
  const ext=[f.dW[0]*0.25, f.dW[1]*0.25];
  const walls=[[H1,[P1[0]+ext[0],P1[1]+ext[1]]],[H2,[P2[0]+ext[0],P2[1]+ext[1]]]];
  for(const [q1,q2] of walls){ const [a1,a2]=w2s(q1[0],q1[1]), [b1,b2]=w2s(q2[0],q2[1]);
    ctx.beginPath();ctx.moveTo(a1,a2);ctx.lineTo(b1,b2);ctx.stroke(); }
  const [hc1,hc2]=w2s(H1[0],H1[1]), [hd1,hd2]=w2s(H2[0],H2[1]);
  ctx.beginPath();ctx.moveTo(hc1,hc2);ctx.lineTo(hd1,hd2);ctx.stroke();
  // piston face
  const [pc1,pc2]=w2s(P1[0],P1[1]), [pd1,pd2]=w2s(P2[0],P2[1]);
  ctx.lineWidth=3.5;ctx.beginPath();ctx.moveTo(pc1,pc2);ctx.lineTo(pd1,pd2);ctx.stroke();
}
// ---- §11.4c · heat & flow interaction overlays ----
// Each interaction couples one body to one gas (or the background,
// gasId===null) -- drawn as a short dashed line from the body's centre to
// the gas's centroid (constraints.js gasCentroid), or, for the background, a
// fixed-length stub topped with a small hatch mark standing in for "open to
// atmosphere". Purely a rate-law-input visualization: color alone tells heat
// (warm) from flow (cool) apart.
function interactionEndpoints(it){
  const body=bodies[bodyIndex(it.bodyId)]; if(!body) return null;
  const gas = it.gasId!=null ? gases.find(g=>g.id===it.gasId) : null;
  const p0=[body.x,body.y];
  if(gas) return {p0, p1:gasCentroid(gas), bg:false};
  return {p0, p1:[body.x, body.y+0.55], bg:true};
}
function drawInteractionLine(ep,col){
  const [x0,y0]=w2s(ep.p0[0],ep.p0[1]), [x1,y1]=w2s(ep.p1[0],ep.p1[1]);
  ctx.save();
  ctx.strokeStyle=col; ctx.lineWidth=2; ctx.setLineDash([5,4]);
  ctx.beginPath(); ctx.moveTo(x0,y0); ctx.lineTo(x1,y1); ctx.stroke();
  ctx.setLineDash([]);
  if(ep.bg){
    const ang=Math.atan2(y1-y0,x1-x0);
    for(const off of [-6,0,6]){ const px=x1-Math.sin(ang)*off, py=y1+Math.cos(ang)*off;
      const ex=px+Math.cos(ang)*10, ey=py+Math.sin(ang)*10;
      ctx.beginPath(); ctx.moveTo(px,py); ctx.lineTo(ex,ey); ctx.stroke(); }
  }
  ctx.fillStyle=col; ctx.beginPath(); ctx.arc(x0,y0,3.5,0,Math.PI*2); ctx.fill();
  ctx.restore();
}
function drawHeatInteraction(hi){
  const ep=interactionEndpoints(hi); if(!ep) return;
  const sel=hi.sel, hoverOn=!sel&&hover===hi;
  drawInteractionLine(ep, sel?'#5aa9f0':hoverOn?'#8fc4f7':'#e0895a');
}
function drawFlowInteraction(fi){
  const ep=interactionEndpoints(fi); if(!ep) return;
  const sel=fi.sel, hoverOn=!sel&&hover===fi;
  drawInteractionLine(ep, sel?'#5aa9f0':hoverOn?'#8fc4f7':'#5ac2e0');
}
function drawGasForce(g){
  const f=gasFrame(g); const P=g._P||g.n*g.T/(g.bore*f.xc); const mag=P*g.bore; if(mag<1e-6)return;
  sim.forceRef=Math.max(sim.forceRef*0.985, mag);
  const px=58*mag/sim.forceRef; const [sx,sy]=w2s(f.pax,f.pay);
  const ex=sx+f.dW[0]*px, ey=sy-f.dW[1]*px;
  ctx.strokeStyle='#f0a24c';ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(sx,sy);ctx.lineTo(ex,ey);ctx.stroke();
  const ang=Math.atan2(-f.dW[1],f.dW[0]);
  ctx.beginPath();ctx.moveTo(ex,ey);ctx.lineTo(ex-8*Math.cos(ang-0.4),ey-8*Math.sin(ang-0.4));
  ctx.moveTo(ex,ey);ctx.lineTo(ex-8*Math.cos(ang+0.4),ey-8*Math.sin(ang+0.4));ctx.stroke();
}
// ---- §11.4b · springs (drawSpring, drawRotSpring, drawSpiral) ----
// Force elements, not constraints (constraints.js §06.6) -- drawn in their own
// pass alongside gas/cable, before drawConstraint's rigid-joint pass.
// Zigzag/coil line for a linear spring's body, between two world points, with
// short straight leads at each end (the standard spring glyph). Amplitude and
// lead length are fixed in screen pixels (like the tol/off constants
// elsewhere in this file) so the coil reads the same at any zoom; the number
// of zigzags scales with on-screen length so it neither vanishes nor
// over-packs as the spring stretches.
function drawCoil(wax,way,wbx,wby){
  const dx=wbx-wax, dy=wby-way, L=Math.hypot(dx,dy)||1e-9;
  const ux=dx/L, uy=dy/L, nx=-uy, ny=ux;
  const leadW=10/cam.scale, ampW=7/cam.scale;
  const bodyLen=Math.max(0, L-2*leadW);
  const coils=Math.max(3, Math.round(L*cam.scale/26));
  const n=coils*2;
  const p1x=wax+ux*leadW, p1y=way+uy*leadW;
  const p2x=wbx-ux*leadW, p2y=wby-uy*leadW;
  ctx.beginPath();
  const [s0x,s0y]=w2s(wax,way); ctx.moveTo(s0x,s0y);
  const [s1x,s1y]=w2s(p1x,p1y); ctx.lineTo(s1x,s1y);
  for(let i=0;i<=n;i++){
    const t=i/n;
    const amp = (i===0||i===n) ? 0 : (i%2 ? ampW : -ampW);
    const px=p1x+ux*bodyLen*t+nx*amp, py=p1y+uy*bodyLen*t+ny*amp;
    const [sx,sy]=w2s(px,py); ctx.lineTo(sx,sy);
  }
  const [s2x,s2y]=w2s(p2x,p2y); ctx.lineTo(s2x,s2y);
  const [s3x,s3y]=w2s(wbx,wby); ctx.lineTo(s3x,s3y);
  ctx.stroke();
}
// The rest-length indicator, shown only while the spring is selected: a
// dimension-style capped line parallel to the spring, centred on it
// (springRestHandlePos/constraints.js §06.6 places the draggable end of this
// same line -- drawn here, picked/dragged via conHandles/applyHandle,
// tools.js §13.3).
function drawSpringRestLine(con,col){
  const [wax,way]=epWorld(con.a), [wbx,wby]=epWorld(con.b);
  const dx=wbx-wax, dy=wby-way, L=Math.hypot(dx,dy)||1e-9;
  const ux=dx/L, uy=dy/L, nx=-uy, ny=ux;
  const off=SPRING_LINE_OFFSET_PX/cam.scale;
  const cx=(wax+wbx)/2+nx*off, cy=(way+wby)/2+ny*off;
  const hx=ux*con.restLen/2, hy=uy*con.restLen/2;
  const p1=[cx-hx,cy-hy], p2=[cx+hx,cy+hy];
  const cap=6/cam.scale;
  ctx.strokeStyle=col; ctx.lineWidth=1.5; ctx.setLineDash([4,3]);
  const [s1x,s1y]=w2s(p1[0],p1[1]), [s2x,s2y]=w2s(p2[0],p2[1]);
  ctx.beginPath();ctx.moveTo(s1x,s1y);ctx.lineTo(s2x,s2y);ctx.stroke();
  ctx.setLineDash([]);
  for(const p of [p1,p2]){
    const a=[p[0]-nx*cap,p[1]-ny*cap], b=[p[0]+nx*cap,p[1]+ny*cap];
    const [ax,ay]=w2s(a[0],a[1]), [bx,by]=w2s(b[0],b[1]);
    ctx.beginPath();ctx.moveTo(ax,ay);ctx.lineTo(bx,by);ctx.stroke();
  }
}
function drawSpring(sp){
  const sel=sp.sel, hoverOn=!sel && hover===sp;
  const col = sel?'#5aa9f0':hoverOn?'#8fc4f7':'#8a94a6';
  const [wax,way]=epWorld(sp.a), [wbx,wby]=epWorld(sp.b);
  ctx.strokeStyle=col; ctx.lineWidth=sel?2.5:2;
  drawCoil(wax,way,wbx,wby);
  drawEndMarker(wax,way,false,sp.a.id==null,col);
  drawEndMarker(wbx,wby,false,sp.b.id==null,col);
  if(sel) drawSpringRestLine(sp,col);
}
// An Archimedean-style spiral: radius sweeps linearly from outerR to innerR
// over `sweep` (signed) radians starting at angle0 -- the rotational spring's
// "wound coil" rendering (constraints.js §06.6 rotSpringSpiralGeom picks
// when this applies vs. the belt rendering below).
function drawSpiral(geo,col){
  const {cx,cy,outerR,innerR,angle0,sweep}=geo;
  const n=Math.max(24, Math.round(Math.abs(sweep)/Math.PI*16));
  ctx.strokeStyle=col; ctx.lineWidth=2; ctx.beginPath();
  for(let i=0;i<=n;i++){
    const t=i/n; const r=outerR+(innerR-outerR)*t; const a=angle0+sweep*t;
    const [sx,sy]=w2s(cx+r*Math.cos(a), cy+r*Math.sin(a));
    i?ctx.lineTo(sx,sy):ctx.moveTo(sx,sy);
  }
  ctx.stroke();
}
function drawRotSpring(rs){
  const hasA=rs.a.id!=null, hasB=rs.b.id!=null;
  const A=hasA?bodies[bodyIndex(rs.a.id)]:null, B=hasB?bodies[bodyIndex(rs.b.id)]:null;
  if((hasA&&!A)||(hasB&&!B)) return;
  const sel=rs.sel, hoverOn=!sel && hover===rs;
  const col = sel?'#5aa9f0':hoverOn?'#8fc4f7':'#8a94a6';
  if(rotSpringVisualMode(rs)==='belt'){
    drawRim(A.x,A.y,A.r,col,A.th); drawRim(B.x,B.y,B.r,col,B.th);
    ctx.strokeStyle=col; ctx.lineWidth=2.5;
    for(const [p,q] of beltTangents(A.x,A.y,A.r, B.x,B.y,B.r, 1)){
      const [x1,y1]=w2s(p[0],p[1]), [x2,y2]=w2s(q[0],q[1]);
      ctx.beginPath();ctx.moveTo(x1,y1);ctx.lineTo(x2,y2);ctx.stroke();
    }
  } else {
    drawSpiral(rotSpringSpiralGeom(rs), col);
  }
  const {pA,pB}=rotSpringControlPoints(rs);
  jointDot(pA[0],pA[1],col); jointDot(pB[0],pB[1],col);
}
// ---- §11.5 · constraints (drawConstraint + drawRim, beltTangents) ----
// Branches by con.type, mirroring §06.5; search e.g. type==='belt' to reach one.
function drawConstraint(con){
  const viol = !sim.running && conMaxC(con) > 2e-3;
  if(viol) violCount++;
  const sel = con.sel; const hoverOn = !sel && hover===con;
  const col = viol ? '#ec5b52' : (sel? '#5aa9f0': hoverOn? '#8fc4f7':'#8a94a6');
  if(con.type==='rod'){
    const [wax,way]=epWorld(con.a), [wbx,wby]=epWorld(con.b);
    const [ax,ay]=w2s(wax,way), [bx,by]=w2s(wbx,wby);
    ctx.strokeStyle=col;ctx.lineWidth=sel?2.5:2;
    ctx.beginPath();ctx.moveTo(ax,ay);ctx.lineTo(bx,by);ctx.stroke();
    drawEndMarker(wax,way,con.weldA,con.a.id==null,col);
    drawEndMarker(wbx,wby,con.weldB,con.b.id==null,col);
    return;
  }
  if(con.type==='slot'){
    // The rail itself: two parallel lines (a track motif) spanning the
    // viewport, through the midpoint of the two endpoints, in the current
    // rail direction (§06.1 slotRailAngle -- tracked via whichever end is
    // locked, or just the live segment direction if neither is).
    const [wax,way]=epWorld(con.a), [wbx,wby]=epWorld(con.b);
    const railAngle=slotRailAngle(con);
    const rdx=Math.cos(railAngle), rdy=Math.sin(railAngle);
    const rnx=-rdy, rny=rdx;
    const midx=(wax+wbx)/2, midy=(way+wby)/2;
    const [vx0,vy0]=s2w(0,0), [vx1,vy1]=s2w(W(),H());
    const span=Math.hypot(vx1-vx0,vy1-vy0);
    const off=3.5/cam.scale;
    ctx.strokeStyle=col; ctx.lineWidth=sel?2:1.5;
    for(const side of [-1,1]){
      const px=midx+rnx*off*side, py=midy+rny*off*side;
      const [x1,y1]=w2s(px-rdx*span, py-rdy*span);
      const [x2,y2]=w2s(px+rdx*span, py+rdy*span);
      ctx.beginPath();ctx.moveTo(x1,y1);ctx.lineTo(x2,y2);ctx.stroke();
    }
    drawEndMarker(wax,way,con.prismaticA,con.a.id==null,col);
    drawEndMarker(wbx,wby,con.prismaticB,con.b.id==null,col);
    return;
  }
  const A=bodies[bodyIndex(con.a.id)]; if(!A) return;
  const [wax,way]=con.a.off?worldPt(A,con.a.off):[A.x,A.y];
  if(con.type==='pin'){ jointDot(wax,way,col); }
  else if(con.type==='belt'){
    const B=bodies[bodyIndex(con.b.id)]; if(!B)return;
    drawRim(A.x,A.y,con.rA,col); drawRim(B.x,B.y,con.rB,col);
    ctx.strokeStyle=col; ctx.lineWidth=2.5;
    for(const [p,q] of beltTangents(A.x,A.y,con.rA, B.x,B.y,con.rB, con.sense)){
      const [x1,y1]=w2s(p[0],p[1]), [x2,y2]=w2s(q[0],q[1]);
      ctx.beginPath();ctx.moveTo(x1,y1);ctx.lineTo(x2,y2);ctx.stroke(); }
  }
  else if(con.type==='knife'){
    const hh=R(A.th,con.dir[0],con.dir[1]); const hl=Math.hypot(hh[0],hh[1])||1; const hx=hh[0]/hl,hy=hh[1]/hl;
    const p1=[wax-hx*0.5,way-hy*0.5], p2=[wax+hx*0.5,way+hy*0.5];
    const [x1,y1]=w2s(p1[0],p1[1]), [x2,y2]=w2s(p2[0],p2[1]);
    ctx.strokeStyle=col;ctx.lineWidth=3.5;ctx.beginPath();ctx.moveTo(x1,y1);ctx.lineTo(x2,y2);ctx.stroke();
    // small lateral tick marking the blocked direction
    const [cx,cy]=w2s(wax,way);
    ctx.lineWidth=1.5;ctx.beginPath();ctx.moveTo(cx+hy*7,cy+hx*7);ctx.lineTo(cx-hy*7,cy-hx*7);ctx.stroke();
    jointDot(wax,way,col);
  }
  else if(con.type==='cvt'){
    const B=bodies[bodyIndex(con.b.id)]; if(!B)return;
    const d=Math.hypot(B.x-A.x,B.y-A.y)||1e-6;
    const rB=Math.max(d-A.r,0);                        // current contact radius on B, the disk
    drawRim(A.x,A.y,A.r,col,A.th);                      // wheel: fixed perimeter
    drawRim(B.x,B.y,rB,col,B.th);                       // disk: current contact radius
  }
}
// Dotted circle whose dash phase is tied to `ang` (the owning body's rotation),
// so the pattern visibly spins with the body rather than staying screen-fixed.
// Screen angle runs opposite world angle (w2s flips y), so the dash offset --
// measured as arc length along the canvas path -- carries a matching sign flip.
function drawRim(x,y,r,col,ang=0){ const [sx,sy]=w2s(x,y); const rr=r*cam.scale;
  ctx.strokeStyle=col;ctx.lineWidth=3;ctx.setLineDash([8,6]);ctx.lineDashOffset=-ang*rr;
  ctx.beginPath();ctx.arc(sx,sy,rr,0,Math.PI*2);ctx.stroke();
  ctx.setLineDash([]);ctx.lineDashOffset=0; }
function beltTangents(ax,ay,ra, bx,by,rb, sense){
  const dx=bx-ax, dy=by-ay, d=Math.hypot(dx,dy)||1e-6; const base=Math.atan2(dy,dx); const segs=[];
  if(sense>0){ let c=(ra-rb)/d; c=Math.max(-1,Math.min(1,c)); const g=Math.acos(c);
    for(const s of [1,-1]){ const a=base+s*g;
      segs.push([[ax+ra*Math.cos(a),ay+ra*Math.sin(a)],[bx+rb*Math.cos(a),by+rb*Math.sin(a)]]); } }
  else { let c=(ra+rb)/d; c=Math.max(-1,Math.min(1,c)); const g=Math.acos(c);
    for(const s of [1,-1]){ const a=base+s*g;
      segs.push([[ax+ra*Math.cos(a),ay+ra*Math.sin(a)],[bx-rb*Math.cos(a),by-rb*Math.sin(a)]]); } }
  return segs;
}

// ---- §11.6 · reaction vectors (the lambda arrows) ----
function drawReaction(con){
  const r=reactionOf(con); if(!r || r.fx===undefined)return;
  const mag=Math.hypot(r.fx,r.fy); if(mag<1e-6)return;
  sim.forceRef=Math.max(sim.forceRef*0.985, mag);
  const px=58*mag/sim.forceRef;
  const [sx,sy]=w2s(r.x,r.y);
  const ux=r.fx/mag, uy=r.fy/mag;              // world dir; screen y flips
  const ex=sx+ux*px, ey=sy-uy*px;
  ctx.strokeStyle='#ec5b52';ctx.lineWidth=2;
  ctx.beginPath();ctx.moveTo(sx,sy);ctx.lineTo(ex,ey);ctx.stroke();
  const ang=Math.atan2(-(uy),ux);
  ctx.beginPath();ctx.moveTo(ex,ey);
  ctx.lineTo(ex-8*Math.cos(ang-0.4), ey-8*Math.sin(ang-0.4));
  ctx.moveTo(ex,ey);
  ctx.lineTo(ex-8*Math.cos(ang+0.4), ey-8*Math.sin(ang+0.4));
  ctx.stroke();
}

// ---- §11.7 · interaction overlays (pending, preview, handles, snap) ----
// `pending` also holds the first pick of a two-step tool (§13.5); it lives here
// because drawPending visualises it, but §13 is what writes and consumes it.
let pending=null;      // first pick of a two-step constraint tool
function drawPending(){
  const [sx,sy]=w2s(pending.wp[0],pending.wp[1]);
  ctx.strokeStyle='#5aa9f0';ctx.lineWidth=2;ctx.setLineDash([4,4]);
  ctx.beginPath();ctx.arc(sx,sy,7,0,Math.PI*2);ctx.stroke();
  ctx.setLineDash([]);
  ctx.beginPath();ctx.moveTo(sx,sy);ctx.lineTo(mouseScreen[0],mouseScreen[1]);ctx.stroke();
}
// {shape:'circle',cx,cy,r} while dragging out a new disk (body tool), or
// {shape:'rect',x0,y0,x1,y1} while dragging out a new rectangle (rectbody
// tool, or the gas tool's piston-bounding box -- tools.js §13.5) from its
// first-clicked corner to the live cursor position.
let bodyPreview=null;
function drawPreview(){
  ctx.strokeStyle='#5aa9f0';ctx.lineWidth=1.5;ctx.setLineDash([5,4]);
  if(bodyPreview.shape==='rect'){
    const [sx0,sy0]=w2s(bodyPreview.x0,bodyPreview.y0), [sx1,sy1]=w2s(bodyPreview.x1,bodyPreview.y1);
    ctx.strokeRect(Math.min(sx0,sx1),Math.min(sy0,sy1),Math.abs(sx1-sx0),Math.abs(sy1-sy0));
  } else {
    const [sx,sy]=w2s(bodyPreview.cx,bodyPreview.cy);
    ctx.beginPath();ctx.arc(sx,sy,Math.max(bodyPreview.r*cam.scale,3),0,Math.PI*2);ctx.stroke();
  }
  ctx.setLineDash([]);
}
function drawHandles(){
  if(sim.running) return;
  const drawOne=(con,h)=>{
    const isHover = hoverHandle && hoverHandle.kind==='con' && hoverHandle.con===con && hoverHandle.which===h.which;
    const rad = isHover?7.5:6;
    const [sx,sy]=w2s(h.x,h.y);
    ctx.fillStyle='#13161c';ctx.beginPath();ctx.arc(sx,sy,rad,0,Math.PI*2);ctx.fill();
    ctx.strokeStyle=isHover?'#8fd0ff':'#5aa9f0';ctx.lineWidth=2;ctx.beginPath();ctx.arc(sx,sy,rad,0,Math.PI*2);ctx.stroke();
    ctx.fillStyle=isHover?'#8fd0ff':'#5aa9f0';ctx.beginPath();ctx.arc(sx,sy,2,0,Math.PI*2);ctx.fill();
  };
  for(const con of constraints){ if(!con.sel) continue; for(const h of conHandles(con)) drawOne(con,h); }
  // Springs (constraints.js §06.6) carry their own handles (endpoints, plus
  // the rest-length control point once selected) via the same conHandles/
  // pickHandle/applyHandle machinery -- they just live in a separate array.
  for(const sp of springs){ if(!sp.sel) continue; for(const h of conHandles(sp)) drawOne(sp,h); }
}
function drawSnap(){
  // an active handle drag's live snap takes priority; otherwise show a
  // placement tool's hover snap (updateHover, tools.js §13.4) -- the anchor
  // (body centre/edge) a click would attach to right now
  const s = (anchorDrag && lastSnap) ? lastSnap : hoverSnap;
  if(!s) return;
  const [sx,sy]=w2s(s.wp[0],s.wp[1]);
  ctx.strokeStyle='#57c78a';ctx.lineWidth=2;ctx.setLineDash([3,3]);
  ctx.beginPath();ctx.arc(sx,sy,9,0,Math.PI*2);ctx.stroke();ctx.setLineDash([]);
  ctx.fillStyle='#57c78a';ctx.font='10px ui-monospace, monospace';ctx.fillText(s.kind, sx+12, sy-9);
}
