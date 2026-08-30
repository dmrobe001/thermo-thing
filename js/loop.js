// ============================================================================
//  §10 · MAIN LOOP  (fixed-step accumulator)
//  Drives real time into an integer number of fixed §08 substeps, then renders
//  (§11) and refreshes the HUD (§12). Spiral-of-death guarded by maxSub.
// ============================================================================
let acc=0, last=performance.now();
const eHist=[];
function frame(now){
  let dt=(now-last)/1000; last=now; if(dt>0.05) dt=0.05;
  if(sim.running){
    acc+=dt; let n=0;
    while(acc>=sim.h && n<sim.maxSub){ substep(sim.h); acc-=sim.h; n++; }
    if(n===sim.maxSub) acc=0;
  }
  render();
  updateHUD();
  requestAnimationFrame(frame);
}
