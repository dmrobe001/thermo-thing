// ============================================================================
//  §11 · RENDER
//  All canvas drawing. §11.1 is the orchestrator; the rest are element painters
//  it calls in back-to-front order. None of these mutate sim state.
//    §11.1  render            (per-frame scene orchestrator; sizing + draw order)
//    §11.2  background        (drawGrid, drawAxes)
//    §11.3  bodies            (drawBody, jointDot)
//    §11.4  gas & cable       (drawCable, drawGas, drawGasForce)
//    §11.5  constraints       (drawConstraint + drawRim, beltTangents)
//    §11.6  reaction vectors  (drawReaction — the λ arrows)
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
  for(const g of gases) drawGas(g);
  for(const cb of cables) drawCable(cb);
  for(const con of constraints) drawConstraint(con);
  for(const b of bodies) drawBody(b);
  drawHandles();
  if(sim.showForces && sim.running){ for(const con of constraints) drawReaction(con); for(const g of gases) drawGasForce(g); }
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
  const [sx,sy]=w2s(b.x,b.y); const rr=b.r*cam.scale;
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
  ctx.lineWidth = b.sel?2.5:1.4;
  ctx.strokeStyle = b.sel? 'var(--accent)':'#7c8798';
  ctx.strokeStyle = b.sel? '#5aa9f0':'#7c8798';
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

// ---- §11.4 · gas & cable (drawCable, drawGas, drawGasForce) ----
function drawCable(cb){
  const f=cableFrame(cb); if(!f)return;
  const taut = cb._lam && cb._lam.length && Math.abs(cb._lam[0])>1e-5;
  const col = cb.sel? '#5aa9f0' : (taut? '#e0c060' : '#8a94a6');
  const [tx,ty]=w2s(f.T[0],f.T[1]);
  ctx.strokeStyle=col; ctx.lineWidth= taut?2.5:1.8;
  ctx.beginPath(); ctx.moveTo(tx,ty);
  if(f.mode==='tangent'){
    // cable line T → natural tangent point Q
    const [qx,qy]=w2s(f.Qx,f.Qy); ctx.lineTo(qx,qy); ctx.stroke();
    // wound arc from tangent point toward control point
    const sweep=Math.min(Math.max(0,f.wb), Math.PI*6);
    if(sweep>0){
      ctx.lineWidth=1.8; ctx.beginPath();
      for(let i=0;i<=24;i++){ const a=f.qang - cb.side*(sweep*i/24);
        const [sx,sy]=w2s(f.S.x+f.rs*Math.cos(a), f.S.y+f.rs*Math.sin(a)); i?ctx.lineTo(sx,sy):ctx.moveTo(sx,sy); }
      ctx.stroke();
    }
  } else {
    // direct mode: cable goes straight T → control point
    const [cx,cy]=w2s(f.Qctrl_x,f.Qctrl_y); ctx.lineTo(cx,cy); ctx.stroke();
  }
  jointDot(f.T[0],f.T[1],col);
  // control-point handle — always visible when selected
  if(cb.sel){
    const [cx2,cy2]=w2s(f.Qctrl_x,f.Qctrl_y);
    ctx.fillStyle='#13161c'; ctx.beginPath(); ctx.arc(cx2,cy2,5,0,Math.PI*2); ctx.fill();
    ctx.strokeStyle=col; ctx.lineWidth=2; ctx.beginPath(); ctx.arc(cx2,cy2,5,0,Math.PI*2); ctx.stroke();
  }
}
function drawGas(g){
  const f=gasFrame(g); const sel=g.sel;
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
  const col=sel?'#5aa9f0':'#8a94a6'; ctx.strokeStyle=col; ctx.lineWidth=2;
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
// ---- §11.5 · constraints (drawConstraint + drawRim, beltTangents) ----
// Branches by con.type, mirroring §06.5; search e.g. type==='belt' to reach one.
function drawConstraint(con){
  const viol = !sim.running && conMaxC(con) > 2e-3;
  if(viol) violCount++;
  const sel = con.sel; const col = viol ? '#ec5b52' : (sel? '#5aa9f0':'#8a94a6');
  const A=bodies[bodyIndex(con.a.id)]; if(!A) return;
  const [wax,way]=con.a.off?worldPt(A,con.a.off):[A.x,A.y];
  if(con.type==='pin'){ jointDot(wax,way,col); }
  else if(con.type==='ground'){
    const [sx,sy]=w2s(con.world[0],con.world[1]);
    jointDot(con.world[0],con.world[1],col);
    ctx.strokeStyle=col;ctx.lineWidth=1.5;
    ctx.beginPath();ctx.moveTo(sx,sy+4);ctx.lineTo(sx-7,sy+14);ctx.lineTo(sx+7,sy+14);ctx.closePath();ctx.stroke();
    for(let i=-7;i<=7;i+=4){ctx.beginPath();ctx.moveTo(sx+i,sy+14);ctx.lineTo(sx+i-4,sy+19);ctx.stroke();}
  }
  else if(con.type==='rod'){
    const B=bodies[bodyIndex(con.b.id)]; if(!B)return; const [wbx,wby]=worldPt(B,con.b.off);
    const [ax,ay]=w2s(wax,way), [bx,by]=w2s(wbx,wby);
    ctx.strokeStyle=col;ctx.lineWidth=sel?2.5:2;
    ctx.beginPath();ctx.moveTo(ax,ay);ctx.lineTo(bx,by);ctx.stroke();
    jointDot(wax,way,col); jointDot(wbx,wby,col);
  }
  else if(con.type==='weld'){
    const B=bodies[bodyIndex(con.b.id)]; if(!B)return; const [wbx,wby]=worldPt(B,con.b.off);
    const [ax,ay]=w2s(wax,way);
    ctx.fillStyle='#13161c';ctx.strokeStyle=col;ctx.lineWidth=2;
    ctx.beginPath();ctx.rect(ax-5,ay-5,10,10);ctx.fill();ctx.stroke();
  }
  else if(con.type==='slot'){
    const f=slotFrame(con);
    const s=(f.wax-f.anchor[0])*f.dW[0]+(f.way-f.anchor[1])*f.dW[1];
    const half=Math.max(1.0, Math.abs(s)+0.5);
    const p1=[f.anchor[0]-f.dW[0]*half, f.anchor[1]-f.dW[1]*half];
    const p2=[f.anchor[0]+f.dW[0]*half, f.anchor[1]+f.dW[1]*half];
    const [x1,y1]=w2s(p1[0],p1[1]), [x2,y2]=w2s(p2[0],p2[1]);
    ctx.strokeStyle=col; ctx.lineWidth=sel?2.5:2;
    ctx.beginPath();ctx.moveTo(x1,y1);ctx.lineTo(x2,y2);ctx.stroke();
    for(const p of [p1,p2]){ const q1=[p[0]-f.n[0]*0.14,p[1]-f.n[1]*0.14], q2=[p[0]+f.n[0]*0.14,p[1]+f.n[1]*0.14];
      const [c1x,c1y]=w2s(q1[0],q1[1]), [c2x,c2y]=w2s(q2[0],q2[1]);
      ctx.beginPath();ctx.moveTo(c1x,c1y);ctx.lineTo(c2x,c2y);ctx.stroke(); }
    ctx.fillStyle='#13161c';ctx.strokeStyle=col;ctx.lineWidth=2;
    if(con.lockRot){ const hd=0.17,hn=0.11; const cs=[[hd,hn],[hd,-hn],[-hd,-hn],[-hd,hn]];
      ctx.beginPath(); cs.forEach((c,i)=>{ const X=f.wax+f.dW[0]*c[0]+f.n[0]*c[1], Y=f.way+f.dW[1]*c[0]+f.n[1]*c[1];
        const [sx,sy]=w2s(X,Y); i?ctx.lineTo(sx,sy):ctx.moveTo(sx,sy); }); ctx.closePath();ctx.fill();ctx.stroke(); }
    else { const [bx,by]=w2s(f.wax,f.way); ctx.beginPath();ctx.arc(bx,by,5,0,Math.PI*2);ctx.fill();ctx.stroke(); }
  }
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
    let rvx=B.x-A.x, rvy=B.y-A.y, d=Math.hypot(rvx,rvy)||1e-6; const ux=rvx/d, uy=rvy/d;
    const px=A.x+ux*A.r, py=A.y+uy*A.r;              // contact on A's rim toward B
    drawRim(A.x,A.y,A.r,col);
    const [ax,ay]=w2s(A.x,A.y), [bx,by]=w2s(B.x,B.y);
    ctx.strokeStyle=col;ctx.lineWidth=1.5;ctx.setLineDash([2,3]);
    ctx.beginPath();ctx.moveTo(ax,ay);ctx.lineTo(bx,by);ctx.stroke();ctx.setLineDash([]);
    jointDot(px,py,col);
  }
}
function drawRim(x,y,r,col){ const [sx,sy]=w2s(x,y);
  ctx.strokeStyle=col;ctx.lineWidth=1.5;ctx.setLineDash([4,3]);
  ctx.beginPath();ctx.arc(sx,sy,r*cam.scale,0,Math.PI*2);ctx.stroke();ctx.setLineDash([]); }
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

// ---- §11.6 · reaction vectors (the λ arrows) ----
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
let bodyPreview=null;  // {cx,cy,r} while dragging out a new body
function drawPreview(){
  const [sx,sy]=w2s(bodyPreview.cx,bodyPreview.cy);
  ctx.strokeStyle='#5aa9f0';ctx.lineWidth=1.5;ctx.setLineDash([5,4]);
  ctx.beginPath();ctx.arc(sx,sy,Math.max(bodyPreview.r*cam.scale,3),0,Math.PI*2);ctx.stroke();
  ctx.setLineDash([]);
}
function drawHandles(){
  if(sim.running) return;
  for(const con of constraints){ if(!con.sel) continue;
    for(const h of conHandles(con)){ const [sx,sy]=w2s(h.x,h.y);
      ctx.fillStyle='#13161c';ctx.beginPath();ctx.arc(sx,sy,6,0,Math.PI*2);ctx.fill();
      ctx.strokeStyle='#5aa9f0';ctx.lineWidth=2;ctx.beginPath();ctx.arc(sx,sy,6,0,Math.PI*2);ctx.stroke();
      ctx.fillStyle='#5aa9f0';ctx.beginPath();ctx.arc(sx,sy,2,0,Math.PI*2);ctx.fill(); } }
}
function drawSnap(){
  if(!anchorDrag || !lastSnap) return;
  const [sx,sy]=w2s(lastSnap.wp[0],lastSnap.wp[1]);
  ctx.strokeStyle='#57c78a';ctx.lineWidth=2;ctx.setLineDash([3,3]);
  ctx.beginPath();ctx.arc(sx,sy,9,0,Math.PI*2);ctx.stroke();ctx.setLineDash([]);
  ctx.fillStyle='#57c78a';ctx.font='10px ui-monospace, monospace';ctx.fillText(lastSnap.kind, sx+12, sy-9);
}
