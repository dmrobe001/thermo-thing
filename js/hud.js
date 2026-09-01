// ============================================================================
//  §12 · HUD  (energy ledger + status + spark)
//  Reads world state and paints the DOM ledger, the status line, and the energy
//  sparkline. The running energy total is the honesty check: with no dissipation
//  it should hold flat (spec §7, system-level accounting).
//    §12.1  energy     (KE + PE + gas internal U + spring PE -> totals)
//    §12.2  updateHUD  (write ledger/status DOM, push spark history)
//    §12.3  drawSpark  (total-energy trace)
// ============================================================================
// ---- §12.1 · energy ----
function energy(){
  let ke=0, pe=0;
  for(const b of bodies){ if(b.static)continue;
    ke+=0.5*b.mass*(b.vx*b.vx+b.vy*b.vy)+0.5*b.I*b.w*b.w;
    if(sim.gravity) pe+=b.mass*sim.g*b.y;
  }
  let U=0; for(const g of gases) U += g.n*(1/(g.gamma-1))*g.T;   // internal energy of the gas
  // spring potential energy: 0.5*k*deviation^2 for each linear (length) and
  // rotational (angle) spring, so §08.6's rescale sees them as a legitimate
  // KE<->PE channel rather than a discrepancy to erase (see physics.js §08.6).
  let SPE=0;
  for(const sp of springs){ const [wax,way]=epWorld(sp.a), [wbx,wby]=epWorld(sp.b);
    const L=Math.hypot(wax-wbx,way-wby); SPE += 0.5*sp.k*(L-sp.restLen)*(L-sp.restLen); }
  for(const rs of rotSprings){ const dev=rotSpringRelAngle(rs)-rs.restAngle; SPE += 0.5*rs.k*dev*dev; }
  return {ke,pe,U,SPE,tot:ke+pe+U+SPE};
}
// ---- §12.2 · updateHUD ----
function updateHUD(){
  const e=energy();
  document.getElementById('eKE').textContent=e.ke.toFixed(2);
  document.getElementById('ePE').textContent=e.pe.toFixed(2);
  const eu=document.getElementById('eU'); if(eu) eu.textContent=e.U.toFixed(2);
  const esp=document.getElementById('eSPE'); if(esp) esp.textContent=e.SPE.toFixed(2);
  document.getElementById('eTot').textContent=e.tot.toFixed(2);
  document.getElementById('eTotHead').textContent=e.tot.toFixed(2);
  document.getElementById('status').textContent=
    `${sim.running?'running':'paused'} · ${bodies.length} bodies · ${constraints.length} constraints`
    + (!sim.running && violCount ? ` · [!] ${violCount} unsatisfied` : '');
  if(sim.running){ eHist.push(e.tot); if(eHist.length>200) eHist.shift(); drawSpark(); }
  if(selBody||selConstraint||selGas||selCable||selSpring||selRotSpring) updateInspectorLive();
}
// ---- §12.3 · drawSpark ----
function drawSpark(){
  const dpr=window.devicePixelRatio||1;
  const w=spark.clientWidth,h=spark.clientHeight;
  if(spark.width!==w*dpr){spark.width=w*dpr;spark.height=h*dpr;}
  sctx.setTransform(dpr,0,0,dpr,0,0);sctx.clearRect(0,0,w,h);
  if(eHist.length<2)return;
  let lo=Math.min(...eHist),hi=Math.max(...eHist); const pad=(hi-lo)*0.15||1; lo-=pad;hi+=pad;
  sctx.strokeStyle='#57c78a';sctx.lineWidth=1.5;sctx.beginPath();
  eHist.forEach((v,i)=>{ const x=i/(eHist.length-1)*w; const y=h-(v-lo)/(hi-lo)*h;
    i?sctx.lineTo(x,y):sctx.moveTo(x,y); }); sctx.stroke();
}
