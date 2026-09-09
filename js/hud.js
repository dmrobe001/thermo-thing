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
// island is an optional {bodyIdx,rotSprings,lines} scope (see physics.js
// §08.0/§08.6) restricting the totals to one momentum-island instead of the
// whole world; omit it for the HUD's whole-scene reading.
function energy(island){
  const bs = island ? island.bodyIdx.map(i=>bodies[i]) : bodies;
  const rss = island ? island.rotSprings : rotSprings;
  const lns = island ? island.lines : constraints.filter(c=>isLine(c) && c.soft>0);
  let ke=0, pe=0, U=0, WA=0;
  for(const b of bs){
    // A vessel's gas internal energy and the atmosphere's own P*V are read even for
    // a static or length-locked vessel -- they are properties of its geometry, not
    // of its motion, and a reservoir's contribution to the ledger is real.
    if(b.shape==='vessel'){ U+=gasU(b); WA+=vesselAtmPE(b); }
    // A pinned vessel keeps its length as a kinetic channel even though its pose
    // is fixed; a fully pinned body has none. Pinned coordinates carry no potential
    // either -- they cannot move, so their height is a constant offset, and the
    // ledger has always left it out.
    if(b.static){ if(b.shape==='vessel') ke+=0.5*b.mu*b.vlen*b.vlen; continue; }
    // A vessel's length rate is a genuine kinetic channel with its own generalized
    // mass mu (geometry.js §05.2d); b.I is already the length-dependent I(len).
    ke+=0.5*b.mass*(b.vx*b.vx+b.vy*b.vy)+0.5*b.I*b.w*b.w;
    if(b.shape==='vessel') ke+=0.5*b.mu*b.vlen*b.vlen;
    if(sim.gravity) pe+=b.mass*sim.g*b.y;
  }
  // Stored elastic energy, so §08.6's rescale sees it as a legitimate KE<->PE
  // channel rather than a discrepancy to erase: a rotational spring's 0.5*k*dev^2,
  // and a compliant line's, one stretch at a time.
  let SPE=0;
  for(const rs of rss){ const dev=rotSpringRelAngle(rs)-rs.restAngle; SPE += 0.5*rs.k*dev*dev; }
  // A COMPLIANT line (constraints.js §06.2f) stores strain in each stretch between
  // consecutive non-sliding joints, at k = 1/soft. Without this the per-island
  // rescale of §08.6 would read that store appearing and disappearing as a leak and
  // "correct" it, which is the same reason the two spring kinds are here.
  for(const line of lns){
    const f=lineFrame(line); if(!f) continue;
    const held=f.J.filter(K=>!K.e.slide);
    for(let i=1;i<held.length;i++){
      const A=held[i-1], B=held[i];
      const rest=Math.abs((B.e.s||0)-(A.e.s||0));
      const L=Math.hypot(A.ep.wx-B.ep.wx, A.ep.wy-B.ep.wy);
      SPE += 0.5*(L-rest)*(L-rest)/line.soft;
    }
  }
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
  if(selGroup||selBody||selConstraint||selCable||selRotSpring||selInteraction) updateInspectorLive();
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
