// ============================================================================
//  §12 · HUD  (energy ledger + status + spark)
//  Reads world state and paints the DOM ledger, the status line, and the energy
//  sparkline. The running energy total is the honesty check: with no dissipation
//  it should hold flat (spec §7, system-level accounting).
//    §12.1  energy     (KE + PE + spring PE + gas internal + atmospheric -> totals)
//    §12.1b bathTotal  (the ledger total, net of what the background bath supplied)
//    §12.2  updateHUD  (write ledger/status DOM, push spark history)
//    §12.3  drawSpark  (total-energy trace)
// ============================================================================
// ---- §12.1 · energy ----
// island is an optional {bodyIdx,springs,rotSprings} scope (see physics.js
// §08.0/§08.6) restricting the totals to one momentum-island instead of the
// whole world; omit it for the HUD's whole-scene reading.
function energy(island){
  const bs = island ? island.bodyIdx.map(i=>bodies[i]) : bodies;
  const sps = island ? island.springs : springs;
  const rss = island ? island.rotSprings : rotSprings;
  let ke=0, pe=0, U=0, WA=0;
  for(const b of bs){
    // A vessel's gas internal energy and the atmosphere's own P*V are read even for
    // a static or length-locked vessel -- they are properties of its geometry, not
    // of its motion, and a reservoir's contribution to the ledger is real.
    if(b.shape==='vessel'){ U+=gasU(b); WA+=vesselAtmPE(b); }
    if(b.static)continue;
    // A vessel's length rate is a genuine kinetic channel with its own generalized
    // mass mu (geometry.js §05.2d); b.I is already the length-dependent I(len).
    ke+=0.5*b.mass*(b.vx*b.vx+b.vy*b.vy)+0.5*b.I*b.w*b.w;
    if(b.shape==='vessel') ke+=0.5*b.mu*b.vlen*b.vlen;
    if(sim.gravity) pe+=b.mass*sim.g*b.y;
  }
  // spring potential energy: 0.5*k*deviation^2 for each linear (length) and
  // rotational (angle) spring, so §08.6's rescale sees them as a legitimate
  // KE<->PE channel rather than a discrepancy to erase (see physics.js §08.6).
  let SPE=0;
  for(const sp of sps){ const [wax,way]=epWorld(sp.a), [wbx,wby]=epWorld(sp.b);
    const L=Math.hypot(wax-wbx,way-wby); SPE += 0.5*sp.k*(L-sp.restLen)*(L-sp.restLen); }
  for(const rs of rss){ const dev=rotSpringRelAngle(rs)-rs.restAngle; SPE += 0.5*rs.k*dev*dev; }
  return {ke,pe,SPE,U,WA,tot:ke+pe+SPE+U+WA};
}
// ---- §12.1b · bathTotal ----
// The scene total the ledger and the sparkline actually show. `energy().tot` is the
// world's own mechanical+gas content, which is what §08.6 defends per island; but a
// heat or mass interaction against the BACKGROUND (physics.js §08.0b) legitimately
// moves energy across a boundary the world does not track, because the background is
// an infinite reservoir rather than a body. sim.bathQ is the running total the bath
// has supplied, so subtracting it restores the flat line: the ledger shows it as its
// own row (negated, so the rows still sum to the total displayed) and the honesty
// check keeps meaning what it meant before interactions existed. In any scene with no
// background interaction, bathQ is identically zero and nothing here changes.
function bathTotal(e){ return e.tot - sim.bathQ; }
// ---- §12.2 · updateHUD ----
function updateHUD(){
  const e=energy();
  document.getElementById('eKE').textContent=e.ke.toFixed(2);
  document.getElementById('ePE').textContent=e.pe.toFixed(2);
  const esp=document.getElementById('eSPE'); if(esp) esp.textContent=e.SPE.toFixed(2);
  const eu=document.getElementById('eU'); if(eu) eu.textContent=e.U.toFixed(2);
  const ea=document.getElementById('eWA'); if(ea) ea.textContent=e.WA.toFixed(2);
  const eb=document.getElementById('eBath'); if(eb) eb.textContent=(-sim.bathQ).toFixed(2);
  const tot=bathTotal(e);
  document.getElementById('eTot').textContent=tot.toFixed(2);
  document.getElementById('eTotHead').textContent=tot.toFixed(2);
  document.getElementById('status').textContent=
    `${sim.running?'running':'paused'} · ${bodies.length} bodies · ${constraints.length} constraints`
    + (!sim.running && violCount ? ` · [!] ${violCount} unsatisfied` : '');
  if(sim.running){ eHist.push(tot); if(eHist.length>200) eHist.shift(); drawSpark(); }
  if(selBody||selConstraint||selCable||selSpring||selRotSpring||selInteraction) updateInspectorLive();
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
