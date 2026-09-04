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
// WHAT to snapshot is not decided here: it is the `state` list on each row of the
// scene ledger (§17.1), walked by snapshotState/applyState (§17.6). That list used
// to live in this file, as a second definition of "the scene's state" maintained
// alongside the one the scene format uses -- and it fell out of step twice, once
// when a vessel's fourth coordinate and its gas arrived and again when sim.bathQ
// did, each time letting Reset silently change the scene's energy. One list now.
//
// What stays here is what is genuinely transport's own: clearing the solver's
// per-element scratch, which is not state a snapshot carries but staleness a jump
// back in time creates.
// refreshFrozen first: saveState is the one hook every structural edit passes
// through -- every tool that pushes or splices, every inspector commit and delete,
// the keyboard delete -- so it is where the derived freezing (§06.2b) has to catch
// up. Without it a strut just placed in a paused bench would not read as locked
// until the next substep or projection happened to run.
//
// An edit that leaves a constraint visibly unsatisfied (the same red-highlight
// drift the canvas/HUD already flag, projection.js §09.2) must not become the
// reset baseline or the exported scene -- Reset would then "restore" a pose that
// was never actually valid, and an exported file would silently bake the error
// in. So the snapshot here is gated on constraintsSatisfied(): an edit that drags
// a mechanism somewhere it can't fully reach just leaves `saved` at the last pose
// that WAS satisfied, until an edit brings it back into agreement. `force` is the
// one deliberate exception: importScene (scene.js §17.4) treats the file's pose as
// the one thing a fidelity-first format must not second-guess, violated or not.
function saveState(force){ refreshFrozen(); if(force || constraintsSatisfied()) saved=snapshotState(); }
function restoreState(){
  if(!saved) return;
  applyState(saved);
  interactions.forEach(it=>{ it._rate=0; });
  // _phiRef is the rod/slot continuity anchor twoPointFrame unwraps phi
  // against (constraints.js). Bodies just snapped back to the saved rest
  // pose, so a reference accumulated across possibly many turns of prior
  // rotation is stale and, left in place, would unwrap the restored geometry's
  // angle onto the wrong winding -- a huge, permanent Baumgarte bias (same
  // failure this fixes at the horizontal crossing, but constant instead of
  // one-step). Clearing it makes the next twoPointFrame call re-seed from the
  // restored geometry's raw atan2, matching how restAngA/B were themselves
  // captured from a fresh, un-accumulated angle. Mirrors cables' _spoolAngle
  // reset below.
  constraints.forEach(c=>{ c._lam=[]; c._rows=[]; c._phiRef=undefined; });
  cables.forEach(c=>{
    c._lam=[]; c._rows=[]; c._active=false; c._C=0; c._cols=null;
    c._Lallow=null; c._spoolAngle=undefined;
  });
  // A selection box describes a pose that Reset has just replaced, so re-fit it
  // around the same bodies at the restored one (select.js §18.1 regroup) rather
  // than leave a frame whose next drag would snap the scene back to a pose that
  // no longer exists.
  if(selGroup) regroup();
}

// ---- §16.2 · transport (play / step / reset) ----
const btnPlay=document.getElementById('btnPlay');
// Starting a run must NOT resave the baseline: every edit already calls
// saveState() itself (drag, inspector commit, tool dispatch, §16.1's own header),
// so `saved` already reflects whatever pose the bench is in the moment Play is
// pressed. Resaving here too used to mean a second Play (after a Pause with no
// edit in between) silently re-baselined onto the mid-run pose the first run left
// behind, so a load -> play -> pause -> play -> pause -> Reset cycle landed back
// on that paused-mid-run pose instead of the scene as loaded. Only the very first
// Play of a session -- before anything, even boot, has ever saved a baseline --
// still needs to establish one.
function setRunning(r){ sim.running=r; btnPlay.textContent=r?'Pause':'Play'; btnPlay.classList.toggle('on',r);
  // A selection box holds its bodies' poses against a frame captured when the
  // selection was made (select.js §18.2). A run moves those bodies out from under
  // it, so the box would be describing a pose that no longer exists -- drop it at
  // the moment the run starts rather than draw a stale frame.
  if(r && selGroup) clearSelection();
  if(r){ if(!saved) saveState(); projectPositions(20); sim.forceRef=1; last=performance.now(); acc=0; hover=null; hoverHandle=null; hoverSnap=null; } }
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
  // TEXTAREA as well as INPUT: the scene card and the stash card are both text
  // areas, and every single-letter branch below is a tool shortcut that would
  // otherwise fire while someone is typing a scene into one.
  if(e.target.tagName==='INPUT' || e.target.tagName==='TEXTAREA')return;
  // Copy and place a selection (select.js §18.4). Checked ahead of every plain-key
  // branch below, since 'c' and 'v' are themselves tool shortcuts.
  if((e.ctrlKey||e.metaKey) && !e.altKey && (e.key==='c'||e.key==='C')){ e.preventDefault(); stashMsg=copySelection(); renderInspector(); return; }
  if((e.ctrlKey||e.metaKey) && !e.altKey && (e.key==='v'||e.key==='V')){ e.preventDefault(); pasteHere(); return; }
  if(e.ctrlKey||e.metaKey) return;                 // leave every other browser shortcut alone
  if(e.code==='Space'){ e.preventDefault(); setRunning(!sim.running); }
  else if(e.key==='r'||e.key==='R'){ setRunning(false); restoreState(); eHist.length=0; }
  else if(e.key==='s'||e.key==='S'){ if(!sim.running){ if(!saved)saveState(); substep(sim.h);} }
  else if(e.key==='Escape'){ pending=null; bodyPreview=null; lasso=null; setTool('select'); }
  else if(e.key==='Delete'||e.key==='Backspace'){ if(selGroup){ deleteGroup(); }
      else if(selBody){const id=selBody.id;
      dropBodyFromConstraints(id);
      springs=springs.filter(s=>s.a.id!==id&&!(s.b&&s.b.id===id));
      rotSprings=rotSprings.filter(s=>s.a.id!==id&&s.b.id!==id);
      dropInteractionsOn(id);
      bodies=bodies.filter(b=>b!==selBody); clearSelection(); saveState();}
      else if(selConstraint){ constraints=constraints.filter(c=>c!==selConstraint); clearSelection(); saveState(); }
      else if(selSpring){ springs=springs.filter(s=>s!==selSpring); clearSelection(); saveState(); }
      else if(selRotSpring){ rotSprings=rotSprings.filter(s=>s!==selRotSpring); clearSelection(); saveState(); }
      else if(selInteraction){ interactions=interactions.filter(x=>x!==selInteraction); clearSelection(); saveState(); } }
  else { const t=TOOLS.find(t=>t.key===e.key); if(t) setTool(t.id); }
});

// Ctrl/Cmd-V, and the one place that decides where a pasted widget lands: under the
// cursor if it is over the canvas, else the middle of the view. The internal
// clipboard is preferred over the system one because it is synchronous and needs no
// permission; the system clipboard is only consulted when nothing has been copied in
// this bench yet, and then asynchronously (select.js §18.4).
function pasteHere(){
  const at = (mouseWorld && isFinite(mouseWorld[0])) ? mouseWorld : viewCentre();
  if(widgetClip){ stashMsg=pasteWidget(widgetClip, at[0], at[1]); renderInspector(); return; }
  if(typeof navigator!=='undefined' && navigator.clipboard && navigator.clipboard.readText){
    navigator.clipboard.readText().then(t=>{
      if(!t || !t.trim()){ stashMsg={ok:false, text:'Nothing to paste.'}; renderInspector(); return; }
      widgetClip=t; stashMsg=pasteWidget(t, at[0], at[1]); renderInspector();
    }, ()=>{ stashMsg={ok:false, text:'Nothing copied here, and the system clipboard is not readable.'}; renderInspector(); });
    return;
  }
  stashMsg={ok:false, text:'Nothing to paste -- copy a selection first.'};
  renderInspector();
}

// ---- §16.5 · boot (initial tool, ledger wiring, first frame) ----
setTool('select'); renderInspector();
const ledgerEl=document.getElementById('ledger');
document.getElementById('ledgerHead').addEventListener('click',()=>ledgerEl.classList.toggle('collapsed'));
if(window.innerWidth<=820) ledgerEl.classList.add('collapsed');
requestAnimationFrame(frame);
