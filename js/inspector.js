// ============================================================================
//  §14 · SELECTION & INSPECTOR
//  What is selected, and the right-hand panel that reflects and edits it.
//    §14.0  numeric fields (numRow/numVal -- one editable number, as arithmetic)
//    §14.1  selection state (clearSelection, select*, pickCable)
//    §14.2  renderInspector    (build the panel DOM per selection type)
//    §14.2b renderVesselInspector (the vessel's own panel)
//    §14.2c the incidence row (one relation, three panels: a vertex's, a body's,
//           a line's -- all built out of incidenceRow)
//    §14.3  updateInspectorLive (per-frame refresh of the live readouts)
// ============================================================================
// ---- §14.0 · numeric fields ----
// Every editable number in the panel is built and read through this pair. What a
// person may type in one is an EXPRESSION (§19): `2*pi/3`, `bg.P/2`, `b3.x+b3.r`
// -- so that the geometry you declare can be declared exactly, rather than as
// whatever the arithmetic rounded to on the way to the keyboard. `worldExprEnv`
// (scene.js §17.8) says what the names mean, and they are the scene file's own
// names, so a field and a file speak the same vocabulary.
//
// Two consequences of that, one in each direction.
//
// The input is `type="text"`, not `type="number"`. A number input hands back an
// empty string for anything it cannot read as a literal, so `2*pi` would be gone
// before this file ever saw it. What it costs is the spinner, so the arrow keys are
// wired back on below -- they step the VALUE the field currently says, which is why
// stepping a field holding `2*pi` leaves a number behind.
//
// What is committed is the number. Nothing keeps the text, and nothing re-evaluates
// it later: type `b3.x` into a body's x and it lands where body 3 is *now*, and
// stays there when body 3 moves. A value that has to keep following another value
// is a constraint or an interaction -- that is what those are for.
const EXPR_HINT = 'Arithmetic allowed: 2*pi/3, bg.P/2, b3.x+b3.r. Up/Down arrows step the value.';
function numIn(id, value, o={}){
  const step = o.step!==undefined ? o.step : 0.1;
  return `<input class="numin" type="text" spellcheck="false" autocomplete="off" id="${id}"`
       + ` value="${value}" data-step="${step}"${o.min!==undefined ? ` data-min="${o.min}"` : ''}`
       + ` title="${EXPR_HINT}">`;
}
const numRow = (lab, id, value, o={}) =>
  `<div class="field${o.cls?' '+o.cls:''}"><span class="lab">${lab}</span>${numIn(id, value, o)}</div>`;

// Read one field. `ok` is what this particular field accepts (a radius is positive,
// a temperature is above zero); a value that fails it is refused exactly as a
// malformed expression is. Either way the field is MARKED and keeps the text it
// holds -- the panel does not silently revert what you typed -- and NaN comes back,
// which is what every caller already tests for.
function numVal(id, ok){
  const el = document.getElementById(id);
  if(!el) return NaN;
  let v = NaN, msg = null;
  try { v = evalExpr(el.value, worldExprEnv()); }
  catch(e){ msg = String(e.message || e); }
  if(msg===null && ok && !ok(v)) msg = `${fmtLoose(v)} is not a value this field can take`;
  el.classList.toggle('bad', msg!==null);
  el.title = msg===null ? EXPR_HINT : msg;
  return msg===null ? v : NaN;
}
const fmtLoose = v => String(Number(v.toPrecision(12)));

// The arrow keys, put back by hand since the field is no longer a spinner. They
// step what the field EVALUATES to, so they work on an expression as readily as on
// a digit string (and leave a number behind, because that is what a step of a value
// is). Shift steps by ten of them. `change` is dispatched because the panel's edits
// are wired to it, and a spinner's arrows fire it too.
function wireNumIns(){
  for(const el of document.querySelectorAll('#panelBody .numin[data-step]')){
    el.onkeydown = ev => {
      const d = ev.key==='ArrowUp' ? 1 : ev.key==='ArrowDown' ? -1 : 0;
      if(!d) return;
      ev.preventDefault();
      let v; try { v = evalExpr(el.value, worldExprEnv()); } catch(e){ return; }
      const step = Number(el.dataset.step) || 0.1;
      const min  = el.dataset.min!==undefined ? Number(el.dataset.min) : -Infinity;
      el.value = fmtLoose(Math.max(min, v + d*step*(ev.shiftKey?10:1)));
      el.dispatchEvent(new Event('change'));
    };
  }
}
// ---- §14.1 · selection state ----
let selBody=null, selConstraint=null, selCable=null, selRotSpring=null, selInteraction=null;
// `selGroup` (select.js §18.1) is the seventh: a MANY-body selection with a box
// around it. It is cleared here with the rest -- one selection at a time is the
// invariant every select* below keeps, and a group is a selection like any other.
function clearSelection(){ bodies.forEach(b=>b.sel=false); constraints.forEach(c=>c.sel=false); cables.forEach(c=>c.sel=false);
  rotSprings.forEach(s=>s.sel=false); interactions.forEach(i=>i.sel=false);
  selBody=null; selConstraint=null; selCable=null; selRotSpring=null; selInteraction=null;
  selGroup=null; groupDrag=null;
  renderInspector(); }
function selectBody(i){ clearSelection(); bodies[i].sel=true; selBody=bodies[i]; renderInspector(); }
function selectConstraint(i){ clearSelection(); constraints[i].sel=true; selConstraint=constraints[i]; renderInspector(); }
function selectCable(i){ clearSelection(); cables[i].sel=true; selCable=cables[i]; renderInspector(); }
function selectRotSpring(i){ clearSelection(); rotSprings[i].sel=true; selRotSpring=rotSprings[i]; renderInspector(); }
function selectInteraction(i){ clearSelection(); interactions[i].sel=true; selInteraction=interactions[i]; renderInspector(); }
function pickCable(wx,wy){
  for(let i=cables.length-1;i>=0;i--){ if(cableHit(cables[i],wx,wy)) return i; }
  return -1; }
function pickRotSpring(wx,wy){
  for(let i=rotSprings.length-1;i>=0;i--){ if(rotSpringHit(rotSprings[i],wx,wy)) return i; }
  return -1; }

// ---- §14.2b · renderVesselInspector ----
// A vessel's panel. It differs from a body's in exposing a FOURTH coordinate --
// `len`, with its rate `vlen` -- alongside x, y and theta, and in carrying the gas
// state sealed inside it (geometry.js §05.2d).
//
// P, T and the gas mass are three faces of one state at a fixed volume, so editing
// any one has to say what it holds fixed. The rule, shown in the panel itself:
//   temperature -> holds the mass (heating a sealed vessel raises its pressure)
//   pressure    -> holds the temperature (pumping gas in or out)
//   gas mass    -> holds the temperature (the same, stated the other way)
// Each is a deliberate, player-authored change to the gas's energy, exactly as
// typing a velocity into a body's panel is a deliberate change to its kinetic
// energy -- not something the simulation does on its own.
function renderVesselInspector(v){
  const p=document.getElementById('panelBody');
  const P=gasP(v), T=gasT(v), V=vesselVol(v);
  p.innerHTML=`
    <h3>Vessel ${v.id}</h3><p class="sub">gas vessel &middot; fixed bore, variable length</p>
    <div class="card"><div class="cardhead">geometry</div>
      ${numRow('bore', 'v_bore', v.bore.toFixed(3), {step:0.02, min:0.02})}
      ${numRow('length', 'v_len', v.len.toFixed(4), {step:0.02, min:0.001})}
      <div class="field"><span class="lab">volume</span><span class="val" id="v_V">${V.toFixed(4)}</span></div>
      ${numRow('shell mass', 'v_shell', v.mShell.toFixed(3), {step:1, min:0.001})}
      <div class="field"><span class="lab">total mass</span><span class="val" id="v_mass">${v.mass.toFixed(3)}</span></div>
      <div class="field"><span class="lab">inertia</span><span class="val" id="v_I">${v.I.toFixed(4)}</span></div>
      <div class="field"><span class="lab">length inertia</span><span class="val" id="v_mu">${v.mu.toFixed(4)}</span></div>
      <div class="field"><span class="lab">pose</span><span class="val">${v.static?'pinned':'free'}</span></div>
      <div class="field"><span class="lab">length</span><span class="val">${v.lenLock?'locked':'free'}</span></div>
      <p class="muted" style="margin:8px 0 0">Both are read off the scene, not set here. A vessel's pose is pinned by a rod welded at both ends to its mid-wall (f&nbsp;=&nbsp;0) and to the background; its length is locked by a rod between two of its own material planes &mdash; a strut inside it. Delete that rod and the coordinate is free again.</p>
    </div>
    <div class="card"><div class="cardhead">gas</div>
      ${numRow('pressure', 'v_P', P.toFixed(1), {step:1000, min:0})}
      ${numRow('temperature', 'v_T', T.toFixed(2), {step:5, min:0.1})}
      ${numRow('gas mass', 'v_gm', v.gas.mass.toFixed(5), {step:0.01, min:0})}
      ${numRow('gamma', 'v_gam', v.gas.gamma.toFixed(3), {step:0.05, min:1.01})}
      <div class="field"><span class="lab">internal energy</span><span class="val" id="v_U">${gasU(v).toFixed(1)}</span></div>
      <div class="field force"><span class="lab">cap force</span><span class="val" id="v_F">${((P-sim.bg.P)*vesselCapArea(v)).toFixed(1)}</span></div>
      <p class="muted" style="margin:8px 0 0">SI throughout: Pa, K, kg, m, J. Ambient is ${(sim.bg.P/1000).toFixed(1)} kPa at ${sim.bg.T.toFixed(2)} K. Editing temperature holds the gas mass; editing pressure or mass holds the temperature. Resizing keeps the gas sealed, so the pressure follows the new volume.</p>
    </div>
    <div class="card"><div class="cardhead">state</div>
      ${numRow('x', 'v_x', v.x.toFixed(3), {step:0.1})}
      ${numRow('y', 'v_y', v.y.toFixed(3), {step:0.1})}
      ${numRow('theta', 'v_th', v.th.toFixed(3), {step:0.05})}
      ${numRow('vx', 'v_vx', v.vx.toFixed(3), {step:0.1})}
      ${numRow('vy', 'v_vy', v.vy.toFixed(3), {step:0.1})}
      ${numRow('w', 'v_w', v.w.toFixed(3), {step:0.1})}
      ${numRow('len rate', 'v_vlen', v.vlen.toFixed(3), {step:0.1})}
    </div>
    ${bodyVerticesCard(v)}
    <button class="del" id="v_del">Delete vessel</button>`;
  wireIncidenceRows();
  const commit=()=>{ renderInspector(); saveState(); };
  // Geometry edits go through resizeVessel, which keeps the gas sealed (mass and
  // temperature carry over) and scales the shell mass with the footprint to hold
  // its density -- the same convention resizeBody uses for an ordinary body.
  // A field whose text cannot be used stops the edit here and keeps that text, so
  // the re-render below never silently reverts what was typed (§14.0 numVal).
  const commitGeom=()=>{ const bore=numVal('v_bore',x=>x>0), len=numVal('v_len',x=>x>0);
    if(!isFinite(bore) || !isFinite(len)) return;
    resizeVessel(v,bore,len);
    projectPositions(8); commit(); };
  document.getElementById('v_bore').onchange=commitGeom;
  document.getElementById('v_len').onchange=commitGeom;
  document.getElementById('v_shell').onchange=()=>{ const m=numVal('v_shell',x=>x>0);
    if(!isFinite(m)) return;
    v.mShell=m; refreshVessel(v); commit(); };
  document.getElementById('v_P').onchange=()=>{ const Pn=numVal('v_P',x=>x>=0);
    if(!isFinite(Pn)) return;
    setVesselGasPT(v,Pn,gasT(v)||sim.bg.T); commit(); };
  document.getElementById('v_T').onchange=()=>{ const Tn=numVal('v_T',x=>x>0);
    if(!isFinite(Tn)) return;
    setVesselGasMT(v,v.gas.mass,Tn); commit(); };
  document.getElementById('v_gm').onchange=()=>{ const mn=numVal('v_gm',x=>x>=0);
    if(!isFinite(mn)) return;
    setVesselGasMT(v,mn,gasT(v)||sim.bg.T); commit(); };
  // gamma changes c_v, hence the internal energy at the same P and V. Hold P and T
  // (the measurable state) and let U follow, rather than the reverse.
  document.getElementById('v_gam').onchange=()=>{ const g=numVal('v_gam',x=>x>1.001);
    if(!isFinite(g)) return;
    const Pk=gasP(v), Tk=gasT(v)||sim.bg.T; v.gas.gamma=g; setVesselGasPT(v,Pk,Tk); commit(); };
  const commitPose=()=>{ const x=numVal('v_x'), y=numVal('v_y'), th=numVal('v_th');
    if(!isFinite(x) || !isFinite(y) || !isFinite(th)) return;
    // Snapshot first: a typed pose is a rigid move like any other, so a rolling
    // pair attached to this body has to take it up as slip (projection.js §09.1).
    const q0=poseSnapshot();
    v.x=x; v.y=y; v.th=th;
    recaptureGrounding(v); projectPositions(8,null,q0); commit(); };
  ['v_x','v_y','v_th'].forEach(id=>document.getElementById(id).onchange=commitPose);
  const commitVel=()=>{ const vx=numVal('v_vx'), vy=numVal('v_vy'), w=numVal('v_w'), vl=numVal('v_vlen');
    if(!isFinite(vx) || !isFinite(vy) || !isFinite(w) || !isFinite(vl)) return;
    v.vx=vx; v.vy=vy; v.w=w; v.vlen=vl; commit(); };
  ['v_vx','v_vy','v_w','v_vlen'].forEach(id=>document.getElementById(id).onchange=commitVel);
  wireNumIns();
  document.getElementById('v_del').onclick=()=>{ const id=v.id;
    dropBodyFromConstraints(id);
    rotSprings=rotSprings.filter(s=>s.a.id!==id && s.b.id!==id);
    cables=cables.filter(c=>c.spool.id!==id && c.tether.id!==id);
    dropInteractionsOn(id);
    bodies=bodies.filter(x=>x!==v); clearSelection(); saveState(); };
}

// ---- §14.2c · the two lists a vertex and a body show of each other ----
// One relation, read from either side: a vertex lists what it is at, a body lists the
// vertices at it. What each list holds is the EXTENT relation of constraints.js
// §06.2e -- everything the point is inside, not just what has been joined to it --
// so the two panels agree with each other and with the picture, and the tick that
// says `joined` is the only control either of them needs.
// What to call a body in a list: its label, and a line says so, because "line 7" and
// "body 7" are two very different things to be attached to.
function bodyName(id){
  if(id==null) return 'the background';
  const L=lineById(id); if(L) return `line ${L.label}`;
  const b=bodies[bodyIndex(id)];
  return b ? `body ${b.label!==undefined?b.label:b.id}` : `body ${id}`;
}
// ---- the incidence ROW, which all three panels are built out of ----
// A vertex's panel, a body's and a line's show the same row from different sides, so
// there is one builder and they cannot drift apart. It is also the one place that
// says what an edit in such a row MEANS:
//
//   the coordinates  are where the vertex sits in that body's own frame -- its
//                    station, for a line. Committing one moves something and then
//                    asks the assembly to agree (projectPositions): where an
//                    incidence holds the vertex there, it is that ANCHOR that moves
//                    and the bodies that have to follow; where nothing holds it,
//                    there is no anchor to move, so the VERTEX goes to the point
//                    named and its own joins are re-read. Either way it is a solve
//                    attempt and not an assertion -- a distance the mechanism cannot
//                    take leaves the bench where the solver could get to.
//   joined           makes the incidence, or releases it. Releasing leaves the body
//                    on the list holding nothing, which is what a "remove" button
//                    used to be for and is why there is no longer one.
//   welded / slide   are what a joined incidence may additionally hold; both are
//                    disabled without it, because both are things a JOIN does.
//
// `incRows` is the render-scoped registry the fields and ticks index into. It is
// cleared by renderInspector before any panel is built, so a row's number means the
// same thing to the markup that wrote it and the handler that reads it.
let incRows=[];
// Where a row's coordinates come from: the incidence's own stored offset where there
// is one -- that is the authored number, and under a violation it is not what the
// geometry reads -- and the live geometry where there is none to read.
function incidenceOff(v, id){
  const e=vertexOns(v).find(x=>x.id===id);
  if(e && !isLineOn(e)) return e.off;
  const [wx,wy]=vertexWorld(v);
  if(id==null) return [wx,wy];
  const b=bodies[bodyIndex(id)];
  return b ? epOffOf(b,wx,wy) : [0,0];
}
function incidenceStation(v, line){
  const e=vertexOns(v).find(x=>x.id===line.id);
  if(e && e.join && !e.slide) return e.s||0;
  const [wx,wy]=vertexWorld(v);
  return lineStationAt(line, wx, wy);
}
function incidenceRow(v, id, o={}){
  const k=incRows.length;
  incRows.push({v, id});
  const line = id!=null ? lineById(id) : null;
  const e = vertexOns(v).find(x=>x.id===id) || null;
  const join = !!(e && e.join);
  // Whose name the row carries: the BODY's on a vertex's panel, the VERTEX's on a
  // body's or a line's -- and there it is a link, because the other panel is where
  // the rest of what that vertex holds is said.
  const name = o.selVertex ? `<a href="#" data-inc-sel="${k}">${v.label}</a>` : bodyName(id);
  const P = vertexPrimary(v);
  const b = id!=null ? bodies[bodyIndex(id)] : null;
  const what = line
    ? (!e ? 'lies on it' : !join ? 'not held' : e.slide ? 'slides along it' : 'held')
    : (!e ? 'inside it' : !join ? 'marks a spot' : e===P ? 'joined &middot; locates it' : 'joined');
  const coords = line
    ? numRow('station', `inc_s${k}`, incidenceStation(v,line).toFixed(3), {step:0.05})
    : (()=>{ const off=incidenceOff(v,id);
        const unit = b && b.shape==='vessel' ? ' material' : '';
        return numRow('x'+unit, `inc_x${k}`, off[0].toFixed(3), {step:0.05})
             + numRow('y'+unit, `inc_y${k}`, off[1].toFixed(3), {step:0.05}); })();
  const slide = line
    ? `<label class="chk"><input type="checkbox" data-inc-slide="${k}" ${e&&e.slide?'checked':''} ${join?'':'disabled'}> slide</label>` : '';
  return `<div class="inc">
    <div class="field"><span class="lab">${name}</span><span class="val">${what}</span></div>
    ${coords}
    <div class="chkrow">
      <label class="chk"><input type="checkbox" data-inc-join="${k}" ${join?'checked':''}> joined</label>
      ${slide}
      <label class="chk"><input type="checkbox" data-inc-weld="${k}" ${e&&e.weld?'checked':''} ${join?'':'disabled'}> welded</label>
    </div>
  </div>`;
}
// Ticking `joined` on. Where there is no incidence yet it makes one AT THE PLACE THE
// VERTEX ALREADY IS, so the tick never snaps anything -- the discipline every capture
// in the engine follows. A line's joint is made sliding, as the line tool makes them
// (VERTEX.md §X.11), and gets the quick solve that brings the vertex onto the bar; a
// body's is exact where it stands and needs none.
function setIncidenceJoin(v, id, want){
  const e = vertexOns(v).find(x=>x.id===id) || null;
  if(e){ setVertexJoin(v, e, want); return; }
  if(!want) return;
  const line = id!=null ? lineById(id) : null;
  const [wx,wy]=vertexWorld(v);
  const b = id!=null ? bodies[bodyIndex(id)] : null;
  const off = id==null ? [wx,wy] : b ? epOffOf(b,wx,wy) : [0,0];
  if(!makeVertexOn(v, {id, off}, {join:true, slide:true})) return;
  if(line) projectPositions(12);
}
// The two commits a row's coordinates make, lifted out of the handlers so the panel
// wiring is only wiring and each is one testable thing. Both snapshot the pose first:
// whatever moves here is a rigid move like any other, so a rolling pair on it takes
// it up as slip (projection.js §09.1). Both end in a projection, which is an ATTEMPT
// -- a distance the mechanism cannot take leaves the bench wherever the solver got to,
// exactly as typing a body's pose does.
function commitIncidenceOff(v, id, x, y){
  const e=vertexOns(v).find(z=>z.id===id);
  if(e && isLineOn(e)) return;             // a line holds a station, not an offset
  const q0=poseSnapshot();
  if(e) e.off=[x,y];
  else {
    // Nothing stored to move, so the VERTEX goes to the point named: the same edit
    // its own x/y field makes, said in this body's frame instead.
    const b = id!=null ? bodies[bodyIndex(id)] : null;
    const [wx,wy] = id==null ? [x,y] : b ? epWorldPt(b,[x,y]) : [x,y];
    setVertexWorld(v, wx, wy);
  }
  projectPositions(8, null, q0);
}
// A HELD joint owns its station, so the number is stored and the bar rearranges
// around it. A slider owns none -- and neither does a vertex merely lying on the line
// -- so there the number says where to put the point, and the joins re-read.
function commitIncidenceStation(v, line, s){
  const q0=poseSnapshot();
  const e=vertexOns(v).find(z=>z.id===line.id);
  if(e && e.join && !e.slide) e.s=s;
  else { const p=lineStationPoint(line, s); if(!p) return; setVertexWorld(v, p[0], p[1]); }
  projectPositions(8, null, q0);
}
function wireIncidenceRows(){
  const done=()=>{ renderInspector(); saveState(); };
  const el=id=>document.getElementById(id);
  incRows.forEach((R,k)=>{
    const v=R.v, id=R.id, line = id!=null ? lineById(id) : null;
    const fx=el(`inc_x${k}`), fy=el(`inc_y${k}`), fs=el(`inc_s${k}`);
    if(fx && fy){
      const commit=()=>{ const x=numVal(`inc_x${k}`), y=numVal(`inc_y${k}`);
        if(!isFinite(x) || !isFinite(y)) return;
        commitIncidenceOff(v, id, x, y); done(); };
      fx.onchange=commit; fy.onchange=commit;
    }
    if(fs && line) fs.onchange=()=>{ const s=numVal(`inc_s${k}`);
      if(!isFinite(s)) return;
      commitIncidenceStation(v, line, s); done(); };
  });
  for(const t of document.querySelectorAll('[data-inc-join]')){
    const R=incRows[Number(t.dataset.incJoin)]; if(!R) continue;
    t.onchange=ev=>{ setIncidenceJoin(R.v, R.id, ev.target.checked); done(); }; }
  for(const t of document.querySelectorAll('[data-inc-weld]')){
    const R=incRows[Number(t.dataset.incWeld)]; if(!R) continue;
    t.onchange=ev=>{ const e=vertexOns(R.v).find(z=>z.id===R.id);
      if(e) setVertexWeld(R.v, e, ev.target.checked); done(); }; }
  for(const t of document.querySelectorAll('[data-inc-slide]')){
    const R=incRows[Number(t.dataset.incSlide)]; if(!R) continue;
    t.onchange=ev=>{ const e=vertexOns(R.v).find(z=>z.id===R.id);
      if(e) setVertexSlide(R.v, e, ev.target.checked); done(); }; }
  for(const t of document.querySelectorAll('[data-inc-sel]')){
    const R=incRows[Number(t.dataset.incSel)]; if(!R) continue;
    t.onclick=ev=>{ ev.preventDefault(); selectConstraint(constraints.indexOf(R.v)); }; }
}
// A vertex's own panel: its label, where it is, and one row per site -- every body
// whose extent covers the point, the background included, with the joined ones ticked.
function vertexInspectorHTML(v){
  const [wx,wy]=vertexWorld(v);
  const welds=vertexOns(v).filter(e=>e.join&&e.weld).length;
  const rows=vertexSites(v).map(s=>incidenceRow(v, s.id)).join('');
  // A single weld holds nothing, and saying so is better than drawing a tick that
  // does not do anything: welding ties one frame to another, and one frame has
  // nothing to tie to (VERTEX.md §X.3).
  const weldNote = welds===1
    ? '<p class="muted" style="margin:8px 0 0">Only one thing is welded here, so nothing is held: a weld ties two frames together. Weld a second body at this vertex &mdash; or the background &mdash; to make the joint rigid.</p>' : '';
  return `
    <h3>Vertex ${v.label}</h3><p class="sub">a named point, and the bodies it touches</p>
    <div class="card"><div class="cardhead">label</div>
      <div class="field"><span class="lab">name</span><input class="num" id="vt_label" type="text" value="${v.label}"></div>
    </div>
    <div class="card"><div class="cardhead">position</div>
      ${numRow('x', 'vt_x', wx.toFixed(3), {step:0.05})}
      ${numRow('y', 'vt_y', wy.toFixed(3), {step:0.05})}
      <p class="muted" style="margin:8px 0 0">Moving the vertex re-reads every joined body's own anchor, so the bodies stay where they are and the point moves between them.</p>
    </div>
    <div class="card"><div class="cardhead">bodies at this vertex</div>
      ${rows || '<p class="muted">none</p>'}${weldNote}
      <p class="muted" style="margin:8px 0 0">Everything whose extent covers this point, whether it is held here or not &mdash; the background always, since its extent is the whole plane. Tick <b>joined</b> to hold the vertex to one, untick it to let go; the body stays listed either way, because what is listed is where the point <em>is</em>. Editing a coordinate moves the anchor and asks the assembly to follow.</p>
    </div>
    <button class="del" id="vt_del">Delete vertex</button>`;
}
function wireVertexCard(v){
  const el=id=>document.getElementById(id);
  el('vt_label').onchange=ev=>{
    const want=String(ev.target.value).trim();
    const okName=/^[A-Za-z_][A-Za-z0-9_]*$/.test(want);
    const taken=constraints.some(c=>isVertex(c)&&c!==v&&c.label===want);
    if(okName && !taken) v.label=want;
    renderInspector(); saveState(); };
  const commitPos=()=>{ const x=numVal('vt_x'), y=numVal('vt_y');
    if(!isFinite(x) || !isFinite(y)) return;
    const q0=poseSnapshot();
    setVertexWorld(v,x,y); projectPositions(8,null,q0);
    renderInspector(); saveState(); };
  el('vt_x').onchange=commitPos; el('vt_y').onchange=commitPos;
  // A vertex leaves on its own: every line it was a joint on keeps its identity and
  // its other joints, and every body it touched is untouched. Nothing here owns
  // anything else (constraints.js §06.2b).
  el('vt_del').onclick=()=>{ deleteConstraint(v); clearSelection(); saveState(); };
}
// ...and the same relation from the body's side: every vertex INSIDE this body, each
// with its place in the body's frame and the same ticks the vertex's own panel shows.
function bodyVerticesCard(b){
  const vs=verticesInExtent(b.id);
  const rows=vs.map(v=>incidenceRow(v, b.id, {selVertex:true})).join('');
  return `<div class="card"><div class="cardhead">vertices on this body</div>
    ${rows || '<p class="muted">none</p>'}
    <p class="muted" style="margin:8px 0 0">Every vertex this body's extent covers, held here or not. Coordinates are in the body's own frame, and editing one moves the anchor and asks the assembly to follow.</p></div>`;
}
// ---- §14.2 · renderInspector (panel DOM per selection type) ----
// One branch per selection: body, constraint, cable, spring, rotational
// spring, or the empty bench.
function renderInspector(){
  const p=document.getElementById('panelBody');
  // Every incidence row any panel below draws numbers itself out of this, so it is
  // cleared here, once, before any of them is built (§14.2c).
  incRows=[];
  // A group is checked first: it is a selection of many bodies, so none of the
  // single-object branches below can speak for it (select.js §18.5).
  if(selGroup){ p.innerHTML=groupInspectorHTML(selGroup); wireGroupCard(); return; }
  if(selConstraint && isVertex(selConstraint)){
    p.innerHTML=vertexInspectorHTML(selConstraint);
    wireVertexCard(selConstraint); wireIncidenceRows(); wireNumIns(); return; }
  if(selBody && selBody.shape==='vessel'){ renderVesselInspector(selBody); return; }
  if(selBody){
    const b=selBody; const isRect=b.shape==='rect';
    p.innerHTML=`
      <h3>Body ${b.label!==undefined?b.label:b.id}</h3><p class="sub">${isRect?'rigid rectangle':'rigid disk'}</p>
      <div class="card"><div class="cardhead">label</div>
        <div class="field"><span class="lab">name</span><input class="num" id="f_label" type="text" value="${b.label!==undefined?b.label:b.id}"></div>
      </div>
      <div class="card"><div class="cardhead">properties</div>
        ${isRect
          ? `${numRow('width', 'f_rw', (b.hw*2).toFixed(3), {step:0.05, min:0.16})}
             ${numRow('height', 'f_rh', (b.hh*2).toFixed(3), {step:0.05, min:0.16})}`
          : `${numRow('radius', 'f_r', b.r.toFixed(3), {step:0.05, min:0.08})}`}
        ${numRow('mass', 'f_mass', b.mass.toFixed(3), {step:0.05, min:0.001})}
        <div class="field"><span class="lab">inertia</span><span class="val" id="f_I">${b.I.toFixed(3)}</span></div>
        <div class="field"><span class="lab">pose</span><span class="val">${b.static?'pinned':'free'}</span></div>
        ${b.static?'<p class="muted" style="margin:8px 0 0">Pinned by a rod welded at both ends to fixed ground &mdash; not a property set here. Delete or unweld that rod and the body is free.</p>':''}
      </div>
      <div class="card"><div class="cardhead">state</div>
        ${numRow('x', 'f_x', b.x.toFixed(3), {step:0.1})}
        ${numRow('y', 'f_y', b.y.toFixed(3), {step:0.1})}
        ${numRow('theta', 'f_th', b.th.toFixed(3), {step:0.05})}
        ${numRow('vx', 'f_vx', b.vx.toFixed(3), {step:0.1})}
        ${numRow('vy', 'f_vy', b.vy.toFixed(3), {step:0.1})}
        ${numRow('w', 'f_w', b.w.toFixed(3), {step:0.1})}
      </div>
      ${bodyVerticesCard(b)}
      <button class="del" id="f_del">Delete body</button>`;
    wireIncidenceRows();
    document.getElementById('f_label').onchange=ev=>{
      const want=String(ev.target.value).trim();
      const taken=bodies.some(x=>x!==b && String(x.label)===want)
               || constraints.some(x=>x.label===want);
      if(/^[A-Za-z_][A-Za-z0-9_]*$|^[1-9][0-9]*$/.test(want) && !taken) b.label=want;
      renderInspector(); saveState(); };
    if(isRect){
      const commitSize=()=>{ const w=numVal('f_rw',x=>x>0.16), h=numVal('f_rh',x=>x>0.16);
        if(!isFinite(w) || !isFinite(h)) return;
        resizeRectAxes(b,w/2,h/2);
        renderInspector(); saveState(); };
      document.getElementById('f_rw').onchange=commitSize;
      document.getElementById('f_rh').onchange=commitSize;
    } else {
      document.getElementById('f_r').onchange=()=>{ const v=numVal('f_r',x=>x>0.08);
        if(!isFinite(v)) return;
        resizeBody(b,v);
        renderInspector(); saveState(); };
    }
    document.getElementById('f_mass').onchange=()=>{ const v=numVal('f_mass',x=>x>0.001);
      if(!isFinite(v)) return;
      setBodyMass(b,v);
      renderInspector(); saveState(); };
    const commitPose=()=>{ const x=numVal('f_x'), y=numVal('f_y'), th=numVal('f_th');
      if(!isFinite(x) || !isFinite(y) || !isFinite(th)) return;
      // Snapshot first -- see the vessel's own commitPose above.
      const q0=poseSnapshot();
      b.x=x; b.y=y; b.th=th;
      recaptureGrounding(b); projectPositions(8,null,q0);
      renderInspector(); saveState(); };
    document.getElementById('f_x').onchange=commitPose;
    document.getElementById('f_y').onchange=commitPose;
    document.getElementById('f_th').onchange=commitPose;
    const commitVel=()=>{ const vx=numVal('f_vx'), vy=numVal('f_vy'), w=numVal('f_w');
      if(!isFinite(vx) || !isFinite(vy) || !isFinite(w)) return;
      b.vx=vx; b.vy=vy; b.w=w;
      renderInspector(); saveState(); };
    document.getElementById('f_vx').onchange=commitVel;
    document.getElementById('f_vy').onchange=commitVel;
    document.getElementById('f_w').onchange=commitVel;
    document.getElementById('f_del').onclick=()=>{ const id=b.id;
      dropBodyFromConstraints(id);
      rotSprings=rotSprings.filter(s=>s.a.id!==id && s.b.id!==id);
      dropInteractionsOn(id);
      bodies=bodies.filter(x=>x!==b); clearSelection(); saveState(); };
  } else if(selConstraint && isLine(selConstraint)){
    // ---- the LINE panel (constraints.js §06.2f) ----
    // Its joints in station order, the distance between each consecutive pair that is
    // HELD, and the one number it owns: the compliance.
    const c=selConstraint;
    // The PLACEMENT, so the panel describes the line the canvas is drawing -- one
    // whose joints have lost their bodies included (constraints.js §06.2f).
    const f=linePlacement(c);
    const J=f?f.J:[];
    const bar=lineIsBar(c);
    // Every vertex the line's extent covers, in station order -- its joints, and the
    // ones merely lying on it, which are a tick away from being joints (§14.2c). P
    // sits at the highest station and Q at the lowest, which is the order lineFrame
    // already sorts its own joints into.
    const onLine = verticesInExtent(c.id)
      .map(v=>({v, s:incidenceStation(v,c)}))
      .sort((a,b2)=>b2.s-a.s);
    const jointRows = onLine.map(K=>incidenceRow(K.v, c.id, {selVertex:true})).join('');
    // One row per stretch between consecutive held joints -- editable, and editing one
    // shifts the stations after it, which is what changing a bar's length means.
    const segs = lineSegments(c);
    const segRows = segs.map((sg,k)=>
      numRow(`${sg.from.v.label}&ndash;${sg.to.v.label}`, `L_seg${k}`, sg.len.toFixed(3), {step:0.05, min:0.001})).join('');
    p.innerHTML=`
      <h3>Line ${c.label}</h3><p class="sub">${bar?'a bar &mdash; every joint held':'a rail &mdash; something slides on it'}</p>
      <div class="card"><div class="cardhead">label</div>
        <div class="field"><span class="lab">name</span><input class="num" id="L_label" type="text" value="${c.label}"></div>
      </div>
      <div class="card"><div class="cardhead">reaction</div>
        <div class="field force"><span class="lab">|force|</span><span class="val" id="f_rf">--</span></div>
        <div class="field force"><span class="lab">torque</span><span class="val" id="f_rt">--</span></div>
      </div>
      <div class="card"><div class="cardhead">vertices along the line</div>
        ${jointRows || '<p class="muted">none</p>'}
        <p class="muted" style="margin:8px 0 0">Every vertex on this line, held here or not. A joint that <em>slides</em> may travel along it; one that is held keeps its station, and the distances below are what that comes to. Editing a station moves that joint and asks the assembly to follow.</p>
        ${J.filter(K=>!K.e.slide).length===1 ? '<p class="muted" style="margin:8px 0 0">Only one joint here is held, so nothing is: a station is a distance from another held joint, and the first one is what the rest are measured from. Hold a second joint &mdash; one of the ones placing the line, if you want this one pinned in the world &mdash; and the distance between them becomes real.</p>' : ''}
      </div>
      ${segs.length?`<div class="card"><div class="cardhead">held distances</div>${segRows}</div>`:''}
      <div class="card"><div class="cardhead">elasticity</div>
        ${numRow('compliance 1/k', 'L_soft', String(c.soft), {step:0.001, min:0})}
        <label class="chk"><input type="checkbox" id="L_posable" ${c.posable?'checked':''}> posable</label>
        <p class="muted" style="margin:8px 0 0">Zero &mdash; the default &mdash; makes the held distances a <em>constraint</em>, solved exactly. Above zero each held stretch becomes a spring of stiffness 1/soft instead, and the line still holds everything on it in line. <b>Posable</b> changes nothing about the running line: while you drag a body it touches, with the sim paused, it is released to a bare rail and is rigid again at whatever pose the drag leaves.</p>
      </div>
      ${c.mesh&&c.mesh.length?`<div class="card"><div class="cardhead">meshing disks</div>${
        c.mesh.map(id=>`<div class="field"><span class="lab">body ${id}</span><span class="val">rolls on the line</span></div>`).join('')}
        <p class="muted" style="margin:8px 0 0">Rolling contact with perfect traction wherever the disk sits, at a pitch radius that is its own live distance from the line. Nonholonomic.</p></div>`:''}
      <button class="del" id="f_del">Delete line</button>`;
    wireIncidenceRows();
    document.getElementById('L_label').onchange=ev=>{
      const want=String(ev.target.value).trim();
      const taken=constraints.some(x=>x!==c && x.label===want) || bodies.some(b2=>String(b2.label)===want);
      if(/^[A-Za-z_][A-Za-z0-9_]*$|^[1-9][0-9]*$/.test(want) && !taken) c.label=want;
      renderInspector(); saveState(); };
    document.getElementById('L_soft').onchange=()=>{ const v=numVal('L_soft',x=>x>=0);
      if(!isFinite(v)) return; c.soft=v; renderInspector(); saveState(); };
    document.getElementById('L_posable').onchange=ev=>{ c.posable=ev.target.checked; renderInspector(); saveState(); };
    segs.forEach((sg,k)=>{
      document.getElementById(`L_seg${k}`).onchange=()=>{
        const v=numVal(`L_seg${k}`,x=>x>0.001); if(!isFinite(v)) return;
        // Editing one stretch shifts every station past it, so the rest of the bar
        // keeps the distances it had -- the stations are relative, and only this one
        // gap was asked to change.
        const held=lineFrame(c).J.filter(K=>!K.e.slide);
        const at=held.indexOf(sg.to);
        const sign=Math.sign((sg.to.e.s||0)-(sg.from.e.s||0)) || 1;
        const delta=sign*v - ((sg.to.e.s||0)-(sg.from.e.s||0));
        for(let i=at;i<held.length;i++) held[i].e.s=(held[i].e.s||0)+delta;
        projectPositions(8); renderInspector(); saveState(); };
    });
    // Deleting the line takes its joints' INCIDENCES and nothing else: the vertices
    // it ran through stay where they are, each still whatever else it was joined to.
    document.getElementById('f_del').onclick=()=>{
      deleteConstraint(c); clearSelection(); saveState(); };
  } else if(selConstraint){
    const c=selConstraint;
    const title = ({belt:'Belt',knife:'Knife-edge wheel',cvt:'Variable gear (CVT)'})[c.type];
    const isBelt=c.type==='belt', isCvt=c.type==='cvt';
    const forceLabel = isBelt?'tension':'|force|';
    let extra='';
    if(isBelt) extra=`<label class="chk"><input type="checkbox" id="f_cross" ${c.sense<0?'checked':''}> crossed belt</label>
        ${numRow('wrap rA', 'f_rA', c.rA.toFixed(3), {step:0.02, min:0.02})}
        ${numRow('wrap rB', 'f_rB', c.rB.toFixed(3), {step:0.02, min:0.02})}
        <div class="field"><span class="lab">ratio</span><span class="val" id="f_bratio">${(c.rB/c.rA).toFixed(2)}</span></div>`;
    if(isCvt) extra=`<div class="field"><span class="lab">ratio (d-rA) / rA</span><span class="val" id="f_ratio">--</span></div>`;
    const note = c.type==='knife' ? 'Nonholonomic: the contact point cannot move sideways, but slides along its heading and pivots freely.'
               : isCvt ? 'Nonholonomic: contact rides A\u2019s rim; the ratio changes as B moves nearer or farther.'
               : 'Reaction is the Lagrange multiplier lambda / h -- the force this joint carries. Run the sim to read it.';
    p.innerHTML=`
      <h3>${title}</h3><p class="sub">${c.type} constraint</p>
      <div class="card"><div class="cardhead">reaction</div>
        <div class="field force"><span class="lab">${forceLabel}</span><span class="val" id="f_rf">--</span></div>
        ${extra}
        <p class="muted" style="margin:8px 0 0">${note}</p>
      </div>
      <button class="del" id="f_del">Delete constraint</button>`;
    if(isBelt){
      // recapturing restPhase against the *current* body angles after a wrap-radius
      // edit is the same trick the crossed-belt toggle already uses just below --
      // it keeps the edit from reading as a spurious phase jump next step.
      const recapturePhase=()=>{ const A=bodies[bodyIndex(c.a.id)],B=bodies[bodyIndex(c.b.id)];
        c.restPhase=c.rA*A.th - c.sense*c.rB*B.th; };
      document.getElementById('f_cross').onchange=ev=>{ c.sense=ev.target.checked?-1:1; recapturePhase(); renderInspector(); saveState(); };
      const commitWrap=()=>{ const rA=numVal('f_rA',x=>x>0.02), rB=numVal('f_rB',x=>x>0.02);
        if(!isFinite(rA) || !isFinite(rB)) return;
        c.rA=rA; c.rB=rB;
        recapturePhase(); renderInspector(); saveState(); };
      document.getElementById('f_rA').onchange=commitWrap;
      document.getElementById('f_rB').onchange=commitWrap;
    }
    document.getElementById('f_del').onclick=()=>{ deleteConstraint(c); clearSelection(); saveState(); };
  } else if(selInteraction){
    const it=selInteraction, isHeat=it.type==='heat';
    const far = it.vessel.id==null ? 'background' : ('vessel '+it.vessel.id);
    // How many *other* interactions of the same kind sit on this body: a lone one is
    // inert by design, and saying so here is the difference between "nothing is
    // happening" reading as a bug and reading as the rule.
    const partners = interactions.filter(x=>x!==it && x.type===it.type && x.body.id===it.body.id).length;
    p.innerHTML=`
      <h3>${isHeat?'Heat':'Mass-flow'} interaction</h3>
      <p class="sub">body ${it.body.id} &harr; ${far}</p>
      <div class="card"><div class="cardhead">state</div>
        <div class="field"><span class="lab">contact area</span><span class="val" id="i_area">--</span></div>
        <div class="field force"><span class="lab">${isHeat?'heat rate in':'mass rate in'}</span><span class="val" id="i_rate">--</span></div>
        <div class="field"><span class="lab">partners on body</span><span class="val" id="i_pair">${partners}</span></div>
      </div>
      <div class="card"><div class="cardhead">${isHeat?'conductivity':'flow conductance'}</div>
        ${numRow('k', 'i_k', it.k, {step:isHeat?'50':'1e-6', min:0})}
        <p class="muted" style="margin:8px 0 0">${isHeat
          ? 'Heat transfer coefficient, W/(m&sup2;&middot;K). Rate = k_eff &middot; area &middot; (T_far &minus; T_near), solved in closed form over the substep, so it approaches equilibrium exponentially and can never overshoot it at any step size.'
          : 'Flow conductance, kg/(s&middot;m&sup2;&middot;Pa). Rate = k_eff &middot; area &middot; (P_far &minus; P_near) -- the same closed-form relaxation with pressure and mass in place of temperature and capacity. Gas that crosses carries its source&rsquo;s enthalpy, so the emptying side cools along its own isentrope.'}</p>
        <p class="muted" style="margin:8px 0 0">Two interactions of the same kind on the same body are a <em>pair</em>: that body is the wall between what they each name, and the rate uses their k&rsquo;s in series and the smaller of the two contact areas. One on its own moves nothing.</p>
      </div>
      <button class="del" id="i_del">Delete interaction</button>`;
    document.getElementById('i_k').onchange=()=>{ const v=numVal('i_k',x=>x>=0);
      if(!isFinite(v)) return;
      it.k=v; renderInspector(); saveState(); };
    document.getElementById('i_del').onclick=()=>{ interactions=interactions.filter(x=>x!==it); clearSelection(); saveState(); };
  } else if(selCable){
    const cb=selCable;
    p.innerHTML=`
      <h3>Cable</h3><p class="sub">tetherball · tension only</p>
      <div class="card"><div class="cardhead">state</div>
        <div class="field force"><span class="lab">tension</span><span class="val" id="cb_T">--</span></div>
        ${numRow('total length', 'cb_Ltot', cb.Ltot.toFixed(3), {step:0.1, min:0.01})}
        <div class="field"><span class="lab">current length</span><span class="val" id="cb_Lcur">--</span></div>
        <div class="field"><span class="lab">paid out</span><span class="val" id="cb_L">--</span></div>
        <div class="field"><span class="lab">wound turns</span><span class="val" id="cb_W">--</span></div>
        <p class="muted" style="margin:8px 0 0">Fixed total length. Drag the anchor handle to wind/unwind. The spool angle encodes which side the cable winds around and accumulates without bound.</p>
      </div>
      <button class="del" id="cb_del">Delete cable</button>`;
    document.getElementById('cb_Ltot').onchange=()=>{ const v=numVal('cb_Ltot',x=>x>0.01);
      if(!isFinite(v)) return;
      cb.Ltot=v;
      renderInspector(); saveState(); };
    document.getElementById('cb_del').onclick=()=>{ cables=cables.filter(x=>x!==cb); clearSelection(); saveState(); };
  } else if(selRotSpring){
    const rs=selRotSpring;
    p.innerHTML=`
      <h3>Rotational spring</h3><p class="sub">force element · tau = k(restAngle-relAngle)</p>
      <div class="card"><div class="cardhead">state</div>
        <div class="field force"><span class="lab">|torque|</span><span class="val" id="rs_T">--</span></div>
        <div class="field"><span class="lab">relative angle</span><span class="val" id="rs_ang">--</span></div>
        ${numRow('rest angle', 'rs_rest', rs.restAngle.toFixed(3), {step:0.05})}
        ${numRow('spring constant k', 'rs_k', rs.k.toFixed(2), {step:0.5, min:0})}
        <p class="muted" style="margin:8px 0 0">Torsional force element between the two bodies' frame angles (the background reads as a fixed theta=0). Not a rigid constraint.</p>
      </div>
      <button class="del" id="rs_del">Delete rotational spring</button>`;
    document.getElementById('rs_rest').onchange=()=>{ const v=numVal('rs_rest');
      if(!isFinite(v)) return;
      rs.restAngle=v;
      renderInspector(); saveState(); };
    document.getElementById('rs_k').onchange=()=>{ const v=numVal('rs_k',x=>x>=0);
      if(!isFinite(v)) return;
      rs.k=v;
      renderInspector(); saveState(); };
    document.getElementById('rs_del').onclick=()=>{ rotSprings=rotSprings.filter(x=>x!==rs); clearSelection(); saveState(); };
  } else {
    p.innerHTML=`
      <h3>Bench</h3><p class="sub">nothing selected</p>
      <p class="muted">Select a body or constraint to inspect it. Every joint reports the reaction force it carries once the sim is running.</p>
      <div class="card"><div class="cardhead">examples</div>
        <div class="examples">
          <button data-ex="pendulum">Rigid pendulum</button>
          <button data-ex="double">Double pendulum</button>
          <button data-ex="fourbar">Four-bar linkage</button>
          <button data-ex="crank">Slider-crank mechanism</button>
          <button data-ex="skate">Skate (knife-edge)</button>
          <button data-ex="integrator">Wheel integrator (CVT)</button>
          <button data-ex="rack">Rack and pinion</button>
          <button data-ex="cable">Cable ratchet</button>
          <button data-ex="hinge">Ground pin and weld (vertices)</button>
          <button data-ex="gasspring">Gas spring (vessel on ground)</button>
          <button data-ex="spinvessel">Spinning vessel (free)</button>
          <button data-ex="heatpair">Heat exchange (two vessels)</button>
          <button data-ex="flowpair">Gas flow (two vessels)</button>
          <button data-ex="clear">Clear bench</button>
        </div>
      </div>
      <div class="card"><div class="cardhead">controls</div>
        <p class="muted">Wheel to zoom · middle-drag or Alt-drag to pan · Space play/pause · R reset · keys 1-9, b/f/g/h/k/l/v/c/q/t tools · lasso (l) to select many, then drag inside the box to move it, a corner to scale, the stem to turn · Ctrl/Cmd-C copies a selection, Ctrl/Cmd-V places it</p>
      </div>
      <div class="card"><div class="cardhead">typing numbers</div>
        <p class="muted">Any number field &mdash; here or in a scene file &mdash; takes arithmetic:
        <code>2*pi/3</code>, <code>0.4*sqrt(2)</code>, <code>bg.P/2</code>, <code>b3.x+b3.r</code>.
        Names are <code>pi tau e deg</code>, the ambient <code>bg.P</code> and <code>bg.T</code>,
        <code>g</code>, <code>air.Rs</code>, <code>air.gamma</code>, and any body's own fields as
        <code>b&lt;id&gt;.&lt;field&gt;</code> &mdash; the same names the scene file uses.
        Functions: <code>sin cos tan asin acos atan atan2 sqrt cbrt exp ln log log2 abs sign
        floor ceil round pow mod clamp min max hypot</code>. Angles are radians, so
        <code>30*deg</code> is a degree measure.</p>
        <p class="muted">What is stored is the number it works out to, not the expression: it
        fixes the value once, at the moment you type it. Something that must keep tracking
        another value as the scene moves is a constraint or an interaction, not a formula.</p>
      </div>
      ${stashCardHTML()}
      ${sceneCardHTML()}`;
    p.querySelectorAll('[data-ex]').forEach(btn=>btn.onclick=()=>loadExample(btn.dataset.ex));
    wireStashCard();
    wireSceneCard();
  }
  // Every branch above that drew editable numbers ends here (the group and the
  // vessel return early and wire their own), so one call covers the lot.
  wireNumIns();
}
// ---- §14.3 · updateInspectorLive (per-frame readout refresh) ----
// refresh an input's value from live sim state, but never while the user has
// it focused -- clobbering mid-edit would fight their keystrokes
function setLive(id,v){ const el=document.getElementById(id); if(el && document.activeElement!==el) el.value=v; }
function updateInspectorLive(){
  if(selGroup){ updateGroupLive(); return; }
  if(selBody && selBody.shape==='vessel' && document.getElementById('v_len')){
    const v=selBody, P=gasP(v), T=gasT(v);
    setLive('v_len',v.len.toFixed(4)); setLive('v_bore',v.bore.toFixed(3));
    setLive('v_x',v.x.toFixed(3)); setLive('v_y',v.y.toFixed(3)); setLive('v_th',v.th.toFixed(3));
    setLive('v_vx',v.vx.toFixed(3)); setLive('v_vy',v.vy.toFixed(3)); setLive('v_w',v.w.toFixed(3));
    setLive('v_vlen',v.vlen.toFixed(3));
    setLive('v_P',P.toFixed(1)); setLive('v_T',T.toFixed(2));
    setLive('v_gm',v.gas.mass.toFixed(5)); setLive('v_shell',v.mShell.toFixed(3));
    document.getElementById('v_V').textContent=vesselVol(v).toFixed(4);
    document.getElementById('v_mass').textContent=v.mass.toFixed(3);
    document.getElementById('v_I').textContent=v.I.toFixed(4);
    document.getElementById('v_mu').textContent=v.mu.toFixed(4);
    document.getElementById('v_U').textContent=gasU(v).toFixed(1);
    document.getElementById('v_F').textContent=((P-sim.bg.P)*vesselCapArea(v)).toFixed(1);
  }
  if(selBody){ const b=selBody;
    if(document.getElementById('f_x')){
      setLive('f_x',b.x.toFixed(3)); setLive('f_y',b.y.toFixed(3)); setLive('f_th',b.th.toFixed(3));
      setLive('f_vx',b.vx.toFixed(3)); setLive('f_vy',b.vy.toFixed(3)); setLive('f_w',b.w.toFixed(3));
      // radius (or width/height)/mass change live while dragging the rim/a
      // corner to resize (§13.6); mass also scales with it there, but is
      // independently editable (setBodyMass)
      if(b.shape==='rect'){ setLive('f_rw',(b.hw*2).toFixed(3)); setLive('f_rh',(b.hh*2).toFixed(3)); }
      else setLive('f_r',b.r.toFixed(3));
      setLive('f_mass',b.mass.toFixed(3));
      document.getElementById('f_I').textContent=b.I.toFixed(3); } }
  if(selConstraint){ const c=selConstraint; const r=reactionOf(c); const el=document.getElementById('f_rf');
    if(el){ if(c.type==='belt') el.textContent=(r?Math.abs(r.val):0).toFixed(2);
      else if(r&&r.fx!==undefined){ el.textContent=Math.hypot(r.fx,r.fy).toFixed(2);
        const t=document.getElementById('f_rt'); if(t&&r.tau!==undefined) t.textContent=r.tau.toFixed(2); } }
    if(c.type==='cvt'){ const A=bodies[bodyIndex(c.a.id)],B=bodies[bodyIndex(c.b.id)];
      const d=Math.hypot(B.x-A.x,B.y-A.y); const er=document.getElementById('f_ratio');
      if(er) er.textContent=((d-A.r)/A.r).toFixed(2); }
  }
  if(selCable){ const cb=selCable; const f=cableFrame(cb);
    const eT=document.getElementById('cb_T');
    if(eT){ eT.textContent=(cb._lam&&cb._lam.length?Math.hypot(...cb._lam)/sim.h:0).toFixed(2);
      setLive('cb_Ltot',cb.Ltot.toFixed(3));
      document.getElementById('cb_Lcur').textContent=cableCurrentLength(cb,f).toFixed(3);
      document.getElementById('cb_L').textContent=(f?f.paidLength:(cb._Lallow!=null?cb._Lallow:0)).toFixed(3);
      document.getElementById('cb_W').textContent=(f?f.windAngle/(2*Math.PI):0).toFixed(2); } }
  if(selInteraction){ const it=selInteraction;
    const ea=document.getElementById('i_area');
    if(ea){
      const bi=bodyIndex(it.body.id), vi=it.vessel.id!=null?bodyIndex(it.vessel.id):-1;
      // The background has no outline, so its side imposes no area limit -- what the
      // panel shows is this interaction's OWN contact patch, which is the half of the
      // pair's min() the player can actually change by sliding the body.
      const area = (bi>=0 && vi>=0) ? contactArea(bodies[bi],bodies[vi]) : (bi>=0 ? Infinity : 0);
      ea.textContent = isFinite(area) ? area.toFixed(4) : 'unbounded';
      const r=it._rate||0;
      document.getElementById('i_rate').textContent =
        it.type==='heat' ? r.toFixed(2)+' W' : r.toExponential(2)+' kg/s';
      setLive('i_k', it.k);
    } }
  if(selRotSpring){ const rs=selRotSpring;
    const rel=rotSpringRelAngle(rs);
    const eT=document.getElementById('rs_T');
    if(eT){ eT.textContent=Math.abs(rs.k*(rs.restAngle-rel)).toFixed(3);
      document.getElementById('rs_ang').textContent=rel.toFixed(3);
      setLive('rs_rest',rs.restAngle.toFixed(3)); setLive('rs_k',rs.k.toFixed(2)); } }
}
