// ============================================================================
//  §18 · SELECTION GROUPS & THE STASH
//  Selecting MANY bodies at once, moving/turning/scaling them as one, and
//  keeping the result as a reusable part.
//    §18.1  the group      (which objects it holds, its box, capture)
//    §18.2  transform      (translate / rotate / scale, and what each recaptures)
//    §18.3  the lasso      (the gesture, and what it catches)
//    §18.4  widgets        (copy, paste, and the stash)
//    §18.5  panel cards    (the group inspector, and the stash on the bench card)
//
//  A group is a SELECTION, not a scene object: nothing here is serialized, nothing
//  here is solved, and the moment the box is dismissed the bench is an ordinary
//  bench again. What it does own, for as long as it is up, is the pose of every body
//  it holds -- see §18.2.
// ============================================================================

// ---- §18.1 · the group (what it holds) ----
// `selGroup` is the seventh selection kind, sitting alongside selBody/selConstraint/...
// (inspector.js §14.1) and cleared by the same clearSelection. Its shape:
//
//   ids      the body ids the player picked, as a Set
//   m        the MEMBERS: those bodies, plus every coupling that came with them
//   cx,cy    where the box's centre is now; hw,hh its half-extents at scale 1
//   ang,s    the frame's rotation and uniform scale, both relative to capture
//   base     every member's pose in the box's own frame, captured once (§18.2)
//
// Which bodies are in the group is the player's pick. Which COUPLINGS come with them
// is derived, by one rule: a constraint, cable, spring or interaction belongs to the
// selection when EVERY BODY IT NAMES is in it. An end anchored to the fixed
// background does not disqualify anything -- the background is not a body, it is a
// point, and that point travels with the box (§18.2 moves it). That matters more
// than it sounds: a rod welded from the ground to a body is how this engine pins
// anything at all (constraints.js §06.2b), so a rule that dropped background
// anchorage would turn every stashed pendulum into a loose disk. The narrower
// reading -- "at least two of its bodies are in the selection" -- is what this rule
// reduces to for everything that names no background.
//
// Anything with a foot outside is simply not a member: it is neither carried nor
// recaptured, and a transform will leave it visibly violated, which is the honest
// report that the selection cut through a machine rather than around one.
let selGroup = null;

function groupMembers(ids){
  const inSel = ep => ep.id==null || ids.has(ep.id);
  const all = eps => eps.length>0 && eps.every(inSel) && eps.some(ep => ep.id!=null && ids.has(ep.id));
  return {
    bodies:       bodies.filter(b => ids.has(b.id)),
    constraints:  constraints.filter(c => all(conEndpoints(c))),
    cables:       cables.filter(c => all([c.tether, c.spool])),
    springs:      springs.filter(s => all([s.a, s.b])),
    rotSprings:   rotSprings.filter(s => all([s.a, s.b])),
    interactions: interactions.filter(i => all([i.body, i.vessel])),
  };
}
// Every background anchor the members carry -- an endpoint with no body (id null)
// whose `off` is therefore a WORLD point rather than a body-frame one. These are the
// only things besides the bodies themselves that a transform has to move. A
// rotational spring's background end is not one (it names a direction, theta = 0,
// not a place) and neither is an interaction's background vessel (it names the
// ambient); both are handled as angles/nothing in §18.2 instead.
function groupAnchors(m){
  const out=[];
  const take = ep => { if(ep && ep.id==null && ep.off) out.push(ep); };
  for(const c of m.constraints){ take(c.a); take(c.b); for(const pt of conPoints(c)) take(pt.ep); }
  for(const c of m.cables) take(c.tether);
  for(const s of m.springs){ take(s.a); take(s.b); }
  return out;
}
// A body's axis-aligned extent -- its outline, not its centre, so the box wraps what
// is drawn. bodyPolygon (§05.2e) already gives a rectangle's or a vessel's four
// turned corners; a disk is its centre plus its radius.
function bodyAABB(b){
  if(rectLike(b)){
    const poly=bodyPolygon(b), xs=poly.map(p=>p[0]), ys=poly.map(p=>p[1]);
    return [Math.min(...xs), Math.min(...ys), Math.max(...xs), Math.max(...ys)];
  }
  return [b.x-b.r, b.y-b.r, b.x+b.r, b.y+b.r];
}
// The box, at capture: axis-aligned, around every body AND every background anchor
// the members carry. The anchors are in it because they move with it -- a box that
// did not contain the ground point it is about to drag would be lying about its
// own contents.
function groupBounds(m){
  let x0=Infinity, y0=Infinity, x1=-Infinity, y1=-Infinity;
  const eat=(ax,ay,bx,by)=>{ x0=Math.min(x0,ax); y0=Math.min(y0,ay); x1=Math.max(x1,bx); y1=Math.max(y1,by); };
  for(const b of m.bodies){ const [ax,ay,bx,by]=bodyAABB(b); eat(ax,ay,bx,by); }
  for(const ep of groupAnchors(m)) eat(ep.off[0], ep.off[1], ep.off[0], ep.off[1]);
  if(!isFinite(x0)) return null;
  const pad = 8/cam.scale;                    // a hair of daylight, so the box reads as a box
  return { cx:(x0+x1)/2, cy:(y0+y1)/2, hw:(x1-x0)/2+pad, hh:(y1-y0)/2+pad };
}

// Capture a group around `idList`. Everything a transform needs is read ONCE, here:
// each body's pose relative to the box centre, each background anchor's world point
// relative to the same, and the two captured fields that measure against the fixed
// world frame rather than against the selection (§18.2 explains those two). Every
// later transform is then computed from this base rather than from the last frame,
// so a drag is exact rather than accumulated, and dragging back to where you started
// puts the scene back where it started.
function makeGroup(idList){
  const ids = new Set(idList);
  const m = groupMembers(ids);
  const bx = groupBounds(m);
  if(!bx) return null;
  const g = { ids, m, cx:bx.cx, cy:bx.cy, hw:bx.hw, hh:bx.hh, ang:0, s:1, scaled:false, base:null };
  g.base = {
    bodies: m.bodies.map(b => ({ b, u:b.x-bx.cx, v:b.y-bx.cy, th:b.th, vx:b.vx, vy:b.vy })),
    anchors: groupAnchors(m).map(ep => ({ ep, u:ep.off[0]-bx.cx, v:ep.off[1]-bx.cy })),
    // A belt's phase is r_A*th_A - sense*r_B*th_B, which is an angle sum with UNEQUAL
    // weights -- turning the whole widget rigidly changes it, even though nothing
    // slipped. The correction is exact and linear, so it is applied from the capture
    // rather than recaptured (which would silently unstress an authored belt).
    belts: m.constraints.filter(c => c.type==='belt')
      .map(c => ({ c, restPhase:c.restPhase, rate:c.rA - c.sense*c.rB })),
    // Same story for a rotational spring with one end on the background: its
    // reference is the fixed world's theta = 0, so a turn of the box turns it.
    rotSprings: m.rotSprings.filter(rs => rs.a.id==null || rs.b.id==null)
      .map(rs => ({ rs, restAngle:rs.restAngle, sign: rs.a.id==null ? -1 : 1 })),
    springs: m.springs.map(sp => ({ sp, restLen:sp.restLen })),
    cables: m.cables.map(cb => ({ cb, Ltot:cb.Ltot })),
  };
  return g;
}
const groupCount = g => g.m.bodies.length;
const groupCouplings = g => g.m.constraints.length + g.m.cables.length + g.m.springs.length
                          + g.m.rotSprings.length + g.m.interactions.length;

// Select a set of bodies as a group, marking every member so the canvas highlights
// it. Returns the group, or null if nothing was caught.
function selectGroup(idList){
  clearSelection();
  const g = makeGroup(idList);
  if(!g) { renderInspector(); return null; }
  selGroup = g;
  for(const b of g.m.bodies) b.sel=true;
  for(const list of ['constraints','cables','springs','rotSprings','interactions'])
    for(const o of g.m[list]) o.sel=true;
  renderInspector();
  return g;
}
// Re-capture the group around the same bodies -- after a paste has moved it, after an
// edit changed what is attached. The box goes back to ang 0 / scale 1 around the
// current pose, which is what "the selection's bounding box" means at any moment.
function regroup(){ if(selGroup) selectGroup([...selGroup.ids]); }

// ---- §18.2 · transform (the box owns the pose) ----
// While the box is up, every selected body's position and angle are FIXED TO THE
// BOX'S FRAME: the frame carries (cx,cy), a rotation `ang` and a uniform scale `s`,
// and applying it writes every member's pose from the base capture. So each body
// turns by exactly the box's own change in angle, and the arrangement between them
// is a similarity of the one it was captured at.
//
// What the transform does NOT touch is body SIZE. A radius, a rectangle's half-
// extents, a vessel's bore and length are the parts themselves; scaling spreads the
// parts apart without growing them, which is the operation a person laying out a
// machine actually wants. The cost is that a scaled selection is not a similarity of
// the whole scene -- the parts kept their size while the distances between their
// anchor points did not -- and that is exactly why the captured geometry of the
// couplings has to be re-read rather than multiplied through. See groupRecapture.
//
// Velocities ride along: a vector attached to the configuration, so it turns with
// the frame and scales with it. Angular velocity is invariant under a similarity and
// is left alone. Nothing here integrates or projects -- the box is the authority on
// where its bodies are, and a coupling with a foot outside the selection is left
// visibly unsatisfied rather than quietly solved away.
const GROUP_MIN_SCALE = 0.02;

function groupPoint(g,u,v){
  const c=Math.cos(g.ang), sn=Math.sin(g.ang);
  return [ g.cx + g.s*(u*c - v*sn), g.cy + g.s*(u*sn + v*c) ];
}
function groupApply(g){
  const c=Math.cos(g.ang), sn=Math.sin(g.ang), s=g.s;
  for(const r of g.base.bodies){
    const [x,y]=groupPoint(g, r.u, r.v);
    r.b.x=x; r.b.y=y; r.b.th=r.th+g.ang;
    r.b.vx = s*(r.vx*c - r.vy*sn); r.b.vy = s*(r.vx*sn + r.vy*c);
  }
  for(const r of g.base.anchors){
    const [x,y]=groupPoint(g, r.u, r.v);
    r.ep.off[0]=x; r.ep.off[1]=y;
  }
  groupRecapture(g);
}
// What the couplings have to be told about the move. Three cases, and the split is
// the whole of what "constraint positions and angles may change, body sizes may not"
// comes to in practice:
//
//   * A pure TRANSLATION changes nothing captured. Every distance and every relative
//     angle between the points a member names is invariant, because all of those
//     points moved together.
//   * A ROTATION is invariant in the same way, with the two exceptions captured in
//     `base` above -- a belt's phase and a background-referenced rotational spring's
//     rest angle, both of which measure against the fixed world. Those two shift
//     analytically. The rest angles of welds and prismatic locks are re-read rather
//     than left alone, not because their value changes (it does not) but because
//     they are measured against a raw atan2 whose branch the turn may have crossed;
//     recaptureConAngles (§06.2b) re-seeds both sides of that comparison together.
//   * A SCALE re-reads everything geometric. A rod's length, a control point's
//     station and every rotation lock's rest angle are distances and directions
//     between anchor points that the scale moved by different amounts (the centres
//     spread, the body-frame offsets did not), so the only correct new value is the
//     one the live geometry now shows -- which is exactly what building the joint
//     here would have captured. A spring's rest length and a cable's paid-out length
//     are the opposite case: they are lengths the ELEMENT owns rather than distances
//     the pose implies, so they scale, and an authored stretch or slack survives.
function groupRecapture(g){
  for(const r of g.base.belts) r.c.restPhase = r.restPhase + r.rate*g.ang;
  for(const r of g.base.rotSprings) r.rs.restAngle = r.restAngle + r.sign*g.ang;
  for(const r of g.base.springs) r.sp.restLen = r.restLen * g.s;
  for(const r of g.base.cables) r.cb.Ltot = r.Ltot * g.s;
  // Once the box has been scaled at ALL, the joints' lengths and stations are read
  // off the geometry from then on, including on the way back. That is what makes the
  // box a frame rather than a ratchet: dragging a corner out and back in returns the
  // rod lengths it re-read, because it re-reads them again at the pose it returns to.
  // A box that has only ever been moved and turned never disturbs them in the first
  // place, so a pure translation is checked for and skipped outright.
  if(g.s!==1) g.scaled=true;
  if(g.ang===0 && !g.scaled) return;
  for(const c of g.m.constraints){
    if(g.scaled) recaptureConPose(c); else recaptureConAngles(c);
  }
}

// ---- the box's handles ----
// Four corners (drag one to scale about the corner opposite it) and one stem out of
// the top edge (drag it to turn). Everything else inside the box is a move. The stem
// is a screen-space length, so it stands the same distance off the box at any zoom.
const GROUP_ROT_PX = 26;
const GROUP_HANDLE_PX = 11;
const GROUP_CORNERS = [[-1,-1],[1,-1],[1,1],[-1,1]];
function groupCornerPoints(g){ return GROUP_CORNERS.map(([i,j]) => groupPoint(g, i*g.hw, j*g.hh)); }
function groupRotPoint(g){ return groupPoint(g, 0, g.hh + GROUP_ROT_PX/(cam.scale*(g.s||1))); }
function groupHandles(g){
  const out = groupCornerPoints(g).map((p,k) => ({kind:'scale', k, x:p[0], y:p[1]}));
  const r = groupRotPoint(g);
  out.push({kind:'rotate', k:-1, x:r[0], y:r[1]});
  return out;
}
function pickGroupHandle(wx,wy){
  const g=selGroup; if(!g) return null;
  const tol=GROUP_HANDLE_PX/cam.scale;
  let best=null, bestD=tol;
  for(const h of groupHandles(g)){
    const d=Math.hypot(wx-h.x, wy-h.y);
    if(d<=bestD){ best=h; bestD=d; }
  }
  return best;
}
// Is a world point inside the box itself -- the region a drag MOVES the selection
// from. Tested in the box's own frame, so it follows the box round as it turns.
function groupContains(g,wx,wy){
  const c=Math.cos(-g.ang), sn=Math.sin(-g.ang);
  const dx=wx-g.cx, dy=wy-g.cy;
  const u=(dx*c - dy*sn)/(g.s||1), v=(dx*sn + dy*c)/(g.s||1);
  return Math.abs(u)<=g.hw && Math.abs(v)<=g.hh;
}

// A transform gesture in progress. Held from the pointerdown that started it, and
// -- like the box itself -- every step is computed from that start rather than from
// the previous move, so a drag is exact and reversible.
let groupDrag=null;
function beginGroupDrag(h, wx, wy){
  const g=selGroup; if(!g) return null;
  const d={ kind:h.kind, k:h.k, wx0:wx, wy0:wy,
            cx0:g.cx, cy0:g.cy, ang0:g.ang, s0:g.s };
  if(h.kind==='rotate') d.a0 = Math.atan2(wy-g.cy, wx-g.cx) - g.ang;
  if(h.kind==='scale'){
    // The corner opposite the grabbed one is the pivot the scale is anchored at, and
    // it must not move -- so it is resolved to a world point now, once.
    const [i,j]=GROUP_CORNERS[h.k];
    const ox=i*g.hw, oy=j*g.hh;
    const c=Math.cos(g.ang), sn=Math.sin(g.ang);
    const rot=(u,v)=>[u*c - v*sn, u*sn + v*c];
    const [rx,ry]=rot(ox,oy);
    d.anchor=[g.cx - g.s*rx, g.cy - g.s*ry];   // the far corner, held fixed
    d.diag=rot(2*ox, 2*oy);                    // the diagonal at scale 1
    d.corner=rot(ox,oy);
  }
  return d;
}
function groupDragTo(wx,wy){
  const g=selGroup, d=groupDrag; if(!g || !d) return;
  if(d.kind==='move'){ g.cx = d.cx0 + (wx-d.wx0); g.cy = d.cy0 + (wy-d.wy0); }
  else if(d.kind==='rotate'){ g.ang = Math.atan2(wy-g.cy, wx-g.cx) - d.a0; }
  else if(d.kind==='scale'){
    const [ax,ay]=d.anchor, [dx,dy]=d.diag;
    const dd=dx*dx+dy*dy;
    if(dd>0){
      const s=Math.max(GROUP_MIN_SCALE, ((wx-ax)*dx + (wy-ay)*dy)/dd);
      g.s=s; g.cx=ax + s*d.corner[0]; g.cy=ay + s*d.corner[1];
    }
  }
  groupApply(g);
}

// The typed counterparts of the three gestures (§18.5's fields), plus the one the
// paste path uses to put a fresh widget where it was asked for.
function groupSetFrame(cx,cy,ang,s){
  const g=selGroup; if(!g) return;
  g.cx=cx; g.cy=cy; g.ang=ang; g.s=Math.max(GROUP_MIN_SCALE,s);
  groupApply(g); saveState();
}
function groupMoveTo(cx,cy){ const g=selGroup; if(g) groupSetFrame(cx,cy,g.ang,g.s); }

// Delete everything the group holds -- the bodies, and with them every coupling that
// named one, member or not. The same cascade each single-body delete path runs
// (tools.js §13.5, inspector.js §14.2, transport.js §16.4), once per body.
function deleteGroup(){
  const g=selGroup; if(!g) return;
  const ids=g.ids;
  for(const id of ids){ dropBodyFromConstraints(id); dropInteractionsOn(id); }
  springs     = springs.filter(s => !ids.has(s.a.id) && !(s.b && ids.has(s.b.id)));
  rotSprings  = rotSprings.filter(s => !ids.has(s.a.id) && !ids.has(s.b.id));
  cables      = cables.filter(c => !ids.has(c.spool.id) && !ids.has(c.tether.id));
  bodies      = bodies.filter(b => !ids.has(b.id));
  clearSelection(); saveState();
}

// ---- §18.3 · the lasso (the gesture, and what it catches) ----
// Drag a freehand loop with the lasso tool (tools.js §13.1) and every body whose
// CENTRE falls inside it joins the group. Centres, not outlines: a rule about the
// whole outline makes a big body impossible to catch without an enormous loop, and
// one about any overlap sweeps in everything the loop grazes. The centre is the one
// reading a person can aim.
//
// The loop is closed implicitly (last point back to first) and tested by the
// even-odd crossing rule, which handles the self-intersecting scribble a real
// freehand lasso produces without anyone having to think about winding.
function pointInPoly(pts, x, y){
  let inside=false;
  for(let i=0, j=pts.length-1; i<pts.length; j=i++){
    const [xi,yi]=pts[i], [xj,yj]=pts[j];
    if((yi>y)!==(yj>y) && x < (xj-xi)*(y-yi)/(yj-yi) + xi) inside=!inside;
  }
  return inside;
}
function lassoSelect(pts){
  if(!pts || pts.length<3){ clearSelection(); return null; }
  const ids = bodies.filter(b => pointInPoly(pts, b.x, b.y)).map(b => b.id);
  if(!ids.length){ clearSelection(); return null; }
  return selectGroup(ids);
}
// A tap with the lasso tool, on a body: add it to the group, or take it out again.
// The correction gesture -- one body missed or one caught by mistake is otherwise a
// whole loop redrawn.
function lassoToggle(wx,wy){
  const bi=pickBody(wx,wy);
  if(bi<0){ clearSelection(); return; }
  const id=bodies[bi].id;
  const ids = selGroup ? new Set(selGroup.ids) : new Set();
  if(ids.has(id)) ids.delete(id); else ids.add(id);
  if(!ids.size){ clearSelection(); return; }
  selectGroup([...ids]);
}

// ---- §18.4 · widgets (copy, paste, and the stash) ----
// A WIDGET is a piece of a bench held as text -- a scene fragment (scene.js §17.7),
// which is to say a scene file with no `sim` and no `cam` line. That is the whole
// representation, and it is deliberately the SAME text the scene card shows and the
// same reader loads: a widget can be read, edited, mailed, pasted into the scene
// box, or dropped on the canvas as a scene of its own. Nothing here is a second
// serialization format.
//
// The stash is a named list of them, kept in localStorage so it outlives the tab.
// The clipboard is one unnamed one, kept in a variable so a copy and a paste inside
// this bench never depend on the browser's clipboard permissions -- the system
// clipboard is written too, as a courtesy, and read only as a fallback.
const STASH_KEY = 'mechbench.stash.v1';
let stash = [];
let widgetClip = null;                 // the internal clipboard: one fragment's text
let stashMsg = null;                   // {ok, text} shown under the stash card

function stashLoad(){
  try {
    const raw = localStorage.getItem(STASH_KEY);
    const arr = raw ? JSON.parse(raw) : [];
    stash = Array.isArray(arr) ? arr.filter(w => w && typeof w.name==='string' && typeof w.text==='string') : [];
  } catch(e){ stash=[]; }               // private mode, a full quota, a corrupt entry: no stash, no crash
}
function stashPersist(){
  try { localStorage.setItem(STASH_KEY, JSON.stringify(stash)); return true; }
  catch(e){ return false; }
}
if(typeof localStorage!=='undefined') stashLoad();

// The current selection as a fragment -- a group, or (so that copy means something
// with one thing picked) a single selected body and whatever came with it. Returns
// null when nothing is selected; throws only if the bench itself cannot be written.
function selectionFragment(){
  let m=null;
  if(selGroup) m=selGroup.m;
  else if(selBody) m=groupMembers(new Set([selBody.id]));
  if(!m || !m.bodies.length) return null;
  return exportFragment(m);
}
function copySelection(){
  let text;
  try { text = selectionFragment(); }
  catch(e){ return {ok:false, text:String(e.message||e)}; }
  if(!text) return {ok:false, text:'Nothing selected to copy.'};
  widgetClip = text;
  if(typeof navigator!=='undefined' && navigator.clipboard) navigator.clipboard.writeText(text).catch(()=>{});
  const n = (text.match(/\n/g)||[]).length;
  return {ok:true, text:`Copied ${n} line${n===1?'':'s'}.`, fragment:text};
}
// Read a fragment in and put it down centred on (wx,wy), selected as a group so the
// very next gesture can place it properly. The paste is one edit: parse (which
// throws before anything is touched), build, then move the whole thing into position
// through the box -- the same transform a drag uses, so a background anchor lands
// where the widget's own geometry says it should relative to everything else.
function pasteWidget(text, wx, wy){
  let made;
  try { made = pasteFragment(text); }
  catch(e){ return {ok:false, text:String(e.message||e)}; }
  if(!made.bodies.length){ return {ok:false, text:'That fragment contains no bodies.'}; }
  const g = selectGroup(made.bodies.map(b=>b.id));
  if(g && wx!=null && isFinite(wx) && isFinite(wy)) groupMoveTo(wx,wy);
  saveState();
  renderInspector();
  return {ok:true, text:`Pasted ${made.bodies.length} bodies.`};
}
// Where a paste with no pointer behind it lands: the middle of the view.
const viewCentre = () => [cam.x, cam.y];

function stashAdd(name, text){
  // Validated before it is stored, so the stash cannot hold something that will not
  // load. parseScene touches nothing (scene.js §17.4) -- it is a pure check here.
  try { parseScene(text); }
  catch(e){ return {ok:false, text:String(e.message||e)}; }
  const nm = (name||'').trim() || `widget ${stash.length+1}`;
  stash.push({ name:nm, text });
  const kept = stashPersist();
  return {ok:true, text: kept ? `Stashed "${nm}".` : `Stashed "${nm}" (this session only -- storage refused).`};
}
function stashRemove(i){ stash.splice(i,1); stashPersist(); }

// ---- §18.5 · panel cards ----
// Two of them: the group's own inspector, which replaces the body/constraint panel
// while a box is up, and the stash, which sits on the bench card alongside the
// examples and the scene file (inspector.js §14.2).
//
// The box's four numbers are editable for the same reason a body's pose is: a
// gesture is how you place something, a typed number is how you place it exactly.
// They are the frame itself -- where its centre sits, how far it has been turned,
// and by how much it has been scaled since the selection was made -- so setting the
// angle to 0 and the scale to 1 always puts the selection back as it was picked.
function groupInspectorHTML(g){
  const parts=[];
  const line=(n,what)=>{ if(n) parts.push(`<div class="field"><span class="lab">${what}</span><span class="val">${n}</span></div>`); };
  line(g.m.bodies.length,'bodies');
  line(g.m.constraints.length,'constraints');
  line(g.m.cables.length,'cables');
  line(g.m.springs.length,'springs');
  line(g.m.rotSprings.length,'rotational springs');
  line(g.m.interactions.length,'interactions');
  return `
    <h3>Selection</h3><p class="sub">${groupCount(g)} bodies &middot; ${groupCouplings(g)} couplings</p>
    <div class="card"><div class="cardhead">box</div>
      <div class="field"><span class="lab">centre x</span><input class="numin" type="number" step="0.1" id="g_x" value="${g.cx.toFixed(3)}"></div>
      <div class="field"><span class="lab">centre y</span><input class="numin" type="number" step="0.1" id="g_y" value="${g.cy.toFixed(3)}"></div>
      <div class="field"><span class="lab">angle</span><input class="numin" type="number" step="0.05" id="g_ang" value="${g.ang.toFixed(4)}"></div>
      <div class="field"><span class="lab">scale</span><input class="numin" type="number" step="0.05" min="${GROUP_MIN_SCALE}" id="g_s" value="${g.s.toFixed(4)}"></div>
      <p class="muted" style="margin:8px 0 0">Drag inside the box to move it, a corner to scale it about the opposite corner, the stem above it to turn it. Every selected body turns by the box's own change in angle; body sizes never change, so scaling spreads the parts apart rather than growing them. Angle 0 and scale 1 are the pose the selection was picked at.</p>
    </div>
    <div class="card"><div class="cardhead">contents</div>${parts.join('')}
      <p class="muted" style="margin:8px 0 0">A coupling joins the selection when every body it names is in it &mdash; a background anchor comes along and moves with the box. One with a foot outside is left where it is, and will read as violated until you put things back.</p>
    </div>
    <div class="card"><div class="cardhead">widget</div>
      <div class="field"><span class="lab">name</span><input class="numin" style="width:140px;text-align:left" type="text" id="g_name" placeholder="four-bar" value=""></div>
      <div class="scenebtns">
        <button id="g_stash">Stash</button>
        <button id="g_copy">Copy</button>
        <button id="g_regroup">Re-fit box</button>
      </div>
      ${stashMsg ? `<p class="${stashMsg.ok?'muted':'scerr'}" style="margin:8px 0 0">${escHtml(stashMsg.text)}</p>` : ''}
      <p class="muted" style="margin:8px 0 0">Stash keeps this selection as a named widget you can place again later; Copy puts it on the clipboard for Ctrl/Cmd-V. Either way it travels as a scene fragment &mdash; the same text the scene card speaks.</p>
    </div>
    <button class="del" id="g_del">Delete selection</button>
    <button class="del" style="color:var(--mid);border-color:var(--hair)" id="g_clear">Deselect</button>`;
}
function wireGroupCard(){
  const g=selGroup; if(!g) return;
  const num=id=>parseFloat(document.getElementById(id).value);
  const commit=()=>{
    const x=num('g_x'), y=num('g_y'), a=num('g_ang'), s=num('g_s');
    if(isFinite(x)&&isFinite(y)&&isFinite(a)&&isFinite(s)&&s>0) groupSetFrame(x,y,a,s);
    renderInspector();
  };
  ['g_x','g_y','g_ang','g_s'].forEach(id=>{ const el=document.getElementById(id); if(el) el.onchange=commit; });
  document.getElementById('g_stash').onclick=()=>{
    let text; try { text=selectionFragment(); }
    catch(e){ stashMsg={ok:false, text:String(e.message||e)}; renderInspector(); return; }
    stashMsg = text ? stashAdd(document.getElementById('g_name').value, text)
                    : {ok:false, text:'Nothing selected.'};
    renderInspector();
  };
  document.getElementById('g_copy').onclick=()=>{ stashMsg=copySelection(); renderInspector(); };
  document.getElementById('g_regroup').onclick=()=>{ stashMsg=null; regroup(); };
  document.getElementById('g_del').onclick=()=>{ stashMsg=null; deleteGroup(); };
  document.getElementById('g_clear').onclick=()=>{ stashMsg=null; clearSelection(); };
}
// The live half: the box's four numbers move under a drag, exactly as a body's pose
// fields do (inspector.js §14.3), so the panel and the canvas never disagree.
function updateGroupLive(){
  const g=selGroup; if(!g || !document.getElementById('g_x')) return;
  setLive('g_x',g.cx.toFixed(3)); setLive('g_y',g.cy.toFixed(3));
  setLive('g_ang',g.ang.toFixed(4)); setLive('g_s',g.s.toFixed(4));
}

// The stash, on the bench card. One row per widget: place it, copy it, or drop it.
// Placing puts it at the middle of the view and selects it, so the box is already up
// and the next gesture is where it actually goes.
function stashCardHTML(){
  const rows = stash.length
    ? stash.map((w,i)=>`<div class="stashrow"><span class="nm">${escHtml(w.name)}</span>
        <button data-place="${i}">Place</button><button data-wcopy="${i}">Copy</button><button class="x" data-wdel="${i}">&times;</button></div>`).join('')
    : `<p class="muted">Nothing stashed yet. Lasso some bodies (l), then press Stash in the selection panel.</p>`;
  return `
    <div class="card"><div class="cardhead">widget stash</div>
      ${rows}
      <div class="scenebtns"><button id="w_paste">Paste clipboard</button></div>
      <textarea id="w_text" class="scenebox" style="height:96px" spellcheck="false"
        placeholder="Paste a scene string here and press Import as widget to keep it."></textarea>
      <div class="field" style="margin-top:6px"><span class="lab">name</span><input class="numin" style="width:140px;text-align:left" type="text" id="w_name" placeholder="widget"></div>
      <div class="scenebtns"><button id="w_add">Import as widget</button></div>
      ${stashMsg ? `<p class="${stashMsg.ok?'muted':'scerr'}" style="margin:8px 0 0">${escHtml(stashMsg.text)}</p>` : ''}
      <p class="muted" style="margin:8px 0 0">A widget is a scene fragment &mdash; a scene file without the <code>sim</code> and <code>cam</code> lines. A whole scene string is a legal one: importing it here keeps it as a part to place, instead of loading it over your bench. The stash lives in this browser.</p>
    </div>`;
}
function wireStashCard(){
  const ta=document.getElementById('w_text'); if(!ta) return;
  const say=(m)=>{ stashMsg=m; renderInspector(); };
  for(const el of document.querySelectorAll('[data-place]')){
    const i=Number(el.dataset.place);
    el.onclick=()=>{ const w=stash[i]; if(!w) return;
      const [cx,cy]=viewCentre(); say(pasteWidget(w.text, cx, cy)); };
  }
  for(const el of document.querySelectorAll('[data-wcopy]')){
    const i=Number(el.dataset.wcopy);
    el.onclick=()=>{ const w=stash[i]; if(!w) return;
      widgetClip=w.text;
      if(typeof navigator!=='undefined' && navigator.clipboard) navigator.clipboard.writeText(w.text).catch(()=>{});
      say({ok:true, text:`"${w.name}" is on the clipboard -- Ctrl/Cmd-V to place it.`}); };
  }
  for(const el of document.querySelectorAll('[data-wdel]')){
    const i=Number(el.dataset.wdel);
    el.onclick=()=>{ stashRemove(i); say(null); };
  }
  document.getElementById('w_paste').onclick=()=>{
    if(!widgetClip){ say({ok:false, text:'The clipboard is empty -- copy a selection first.'}); return; }
    const [cx,cy]=viewCentre(); say(pasteWidget(widgetClip, cx, cy));
  };
  document.getElementById('w_add').onclick=()=>{
    const text=ta.value.trim();
    if(!text){ say({ok:false, text:'Paste a scene or fragment into the box first.'}); return; }
    say(stashAdd(document.getElementById('w_name').value, text+'\n'));
  };
}
