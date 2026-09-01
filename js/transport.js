// ============================================================================
//  §16 · TRANSPORT & BOOT
//  Run control, the reset snapshot, DOM control wiring, keyboard shortcuts, and
//  the startup sequence that kicks off the §10 loop.
//    §16.1  snapshots        (saveState / restoreState -- the reset baseline)
//    §16.2  transport        (play / step / reset + setRunning)
//    §16.3  toggles          (forces / grid / gravity / gravity value)
//    §16.4  keyboard shortcuts
//    §16.5  boot             (initial tool, ledger wiring, first frame)
// ============================================================================
// ---- §16.1 · snapshots (reset baseline) ----
let saved=null;
function saveState(){ saved={ b:bodies.map(b=>({id:b.id,x:b.x,y:b.y,th:b.th,vx:b.vx,vy:b.vy,w:b.w})), gT:gases.map(g=>g.T), cSA:cables.map(c=>c.spoolAngle) }; }
function restoreState(){ if(!saved)return; for(const s of saved.b){ const b=bodies.find(x=>x.id===s.id);
  if(b){ b.x=s.x;b.y=s.y;b.th=s.th;b.vx=s.vx||0;b.vy=s.vy||0;b.w=s.w||0; } }
  gases.forEach((g,i)=>{ if(saved.gT[i]!=null) g.T=saved.gT[i]; });
  // _phiRef is the rod/slot continuity anchor twoPointFrame unwraps phi
  // against (constraints.js). Bodies just snapped back to the saved rest
  // pose, so a reference accumulated across possibly many turns of prior
  // rotation is stale and, left in place, would unwrap the restored geometry's
  // angle onto the wrong winding -- a huge, permanent Baumgarte bias (same
  // failure this fixes at the horizontal crossing, but constant instead of
  // one-step). Clearing it makes the next twoPointFrame call re-seed from the
  // restored geometry's raw atan2, matching how restAngA/B were themselves
  // captured from a fresh, un-accumulated angle. Mirrors cables' c._spoolAngle
  // reset below.
  constraints.forEach(c=>{ c._lam=[]; c._rows=[]; c._phiRef=undefined; });
  cables.forEach((c,i)=>{
    if(saved.cSA[i]!=null){ c.spoolAngle=saved.cSA[i]; }
    c._lam=[]; c._rows=[]; c._active=false; c._C=0; c._cols=null;
    c._Lallow=null; c._spoolAngle=undefined;
  });
}

// ---- §16.2 · transport (play / step / reset) ----
const btnPlay=document.getElementById('btnPlay');
function setRunning(r){ sim.running=r; btnPlay.textContent=r?'Pause':'Play'; btnPlay.classList.toggle('on',r);
  if(r){ saveState(); projectPositions(20); sim.forceRef=1; last=performance.now(); acc=0; hover=null; hoverHandle=null; hoverSnap=null; } }
btnPlay.onclick=()=>setRunning(!sim.running);
document.getElementById('btnStep').onclick=()=>{ if(sim.running)return; if(!saved)saveState(); projectPositions(20); substep(sim.h); };
document.getElementById('btnReset').onclick=()=>{ setRunning(false); restoreState(); eHist.length=0; };

// ---- §16.3 · toggles (forces / grid / gravity) ----
document.getElementById('tgForces').onchange=e=>sim.showForces=e.target.checked;
document.getElementById('tgGrid').onchange=e=>sim.showGrid=e.target.checked;
document.getElementById('tgGrav').onchange=e=>sim.gravity=e.target.checked;
const gv=document.getElementById('gravVal');
gv.oninput=e=>{ sim.g=parseFloat(e.target.value); document.getElementById('gravRead').textContent=sim.g.toFixed(1); };

// ---- §16.4 · keyboard shortcuts ----
window.addEventListener('keydown',e=>{
  if(e.target.tagName==='INPUT')return;
  if(e.code==='Space'){ e.preventDefault(); setRunning(!sim.running); }
  else if(e.key==='r'||e.key==='R'){ setRunning(false); restoreState(); eHist.length=0; }
  else if(e.key==='s'||e.key==='S'){ if(!sim.running){ if(!saved)saveState(); substep(sim.h);} }
  else if(e.key==='Escape'){ pending=null; bodyPreview=null; setTool('select'); }
  else if(e.key==='Delete'||e.key==='Backspace'){ if(selBody){const id=selBody.id;
      constraints=constraints.filter(c=>c.a.id!==id&&!(c.b&&c.b.id===id));
      springs=springs.filter(s=>s.a.id!==id&&!(s.b&&s.b.id===id));
      rotSprings=rotSprings.filter(s=>s.a.id!==id&&s.b.id!==id);
      bodies=bodies.filter(b=>b!==selBody); clearSelection(); saveState();}
      else if(selConstraint){ constraints=constraints.filter(c=>c!==selConstraint); clearSelection(); saveState(); }
      else if(selSpring){ springs=springs.filter(s=>s!==selSpring); clearSelection(); saveState(); }
      else if(selRotSpring){ rotSprings=rotSprings.filter(s=>s!==selRotSpring); clearSelection(); saveState(); } }
  else { const t=TOOLS.find(t=>t.key===e.key); if(t) setTool(t.id); }
});

// ---- §16.5 · boot (initial tool, ledger wiring, first frame) ----
setTool('select'); renderInspector();
const ledgerEl=document.getElementById('ledger');
document.getElementById('ledgerHead').addEventListener('click',()=>ledgerEl.classList.toggle('collapsed'));
if(window.innerWidth<=820) ledgerEl.classList.add('collapsed');
requestAnimationFrame(frame);
