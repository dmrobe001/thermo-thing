// ============================================================================
//  §06 · CONSTRAINT ROWS
//  The heart of the engine. Each constraint is turned into one or more rows of
//  the velocity-linear (Pfaffian) form  J·v = -bias  (spec §3.3). A row is
//    { cols:[[bodyIdx, jx, jy, jw, jlen?], ...], C, nh? }
//  jlen is the column of a vessel's fourth (length) coordinate -- absent, meaning
//  zero, on every ordinary body. Rows never build it by hand: it comes out of
//  epFrame's velCols closure (§06.1), which is why every branch of rowsFor below
//  works on a vessel endpoint unchanged.
//  where C is the raw position error (the value to drive to zero) and nh flags
//  a nonholonomic row (velocity-only, no position invariant -- excluded from the
//  §09 position projection). The §08 solver scales C by beta/h (Baumgarte);
//  the §09 projection uses C directly. Same rows serve both.
//    §06.1  bodyIndex, epWorld, epFrame -- an endpoint resolved to a frame
//    §06.1b the LINE frame (lineFrameOf, frameAngleRow) -- a straight bar's own
//           frame, derived from its two endpoints and owning no coordinates. The
//           third frame kind beside a body's and the background's (VERTEX.md §X.5)
//    §06.2  the remaining constraint makers (belt, cvt, knife, cable)
//    §06.2b derived freezing (lineGrounds, lineLocksLength, refreshFrozen) and the
//           recaptures that follow a hand move (recaptureConAngles/recaptureConPose)
//    §06.2c (retired with the rod, slot and rack -- a line's joints are vertices)
//    §06.2d the pose-time release (withPosing, recapturePosable) -- what a
//           `posable` LINE does while a body it touches is dragged
//    §06.2e the VERTEX (makeVertex, makeVertexOn, vertexWorld, verticesOn) -- a
//           named point and the bodies it touches, which is what a pin became;
//           plus vertexSites/verticesInExtent, the same relation widened to what
//           the point is INSIDE, which is what the two panels list
//    §06.2f the LINE (makeLine, lineFrame, lineJoints, lineStationAt) -- a straight
//           bar whose frame is derived from its joints, which is what rod/slot/rack
//           became
//    §06.3  cableFrame (tetherball tangent geometry for the unilateral cable)
//    §06.4  (retired -- see §06.1)
//    §06.5  rowsFor    (the dispatch: one branch per constraint type)
//    §06.6  spring / rotSpring frames (Hookean force elements, §08.1) -- these
//           are force elements, not constraint rows: they never appear in
//           rowsFor or the solve of §07/§08.3, only in the applied-force pass
//           of §08.1.
// ============================================================================

// ---- §06.1 · bodyIndex ----
function bodyIndex(id){ return bodies.findIndex(b=>b.id===id); }

// Resolve an endpoint {id, off} to its world point. id===null means the
// endpoint is fixed to the background (world-anchored) rather than riding a
// body -- off then holds the world coordinate directly, mirroring the
// null-id convention already used by cable tethers. Shared by rod and slot,
// whose endpoints are both {id, off} pairs.
function epWorld(ep){
  if(ep.id==null) return [ep.off[0], ep.off[1], 0, 0];
  const b=bodies[bodyIndex(ep.id)];
  return worldPt(b, epLocal(b, ep.off));       // material offset on a vessel, §05.2c
}
// ---- §06.1b · the LINE frame (a frame with no coordinates of its own) ----
// A rod, a slot and a rack are all a straight massless bar, and a bar has a FRAME:
// a heading, and a material point at every station along it. What it does not have
// is coordinates -- its pose is a function of the two endpoints that define it, so
// nothing about it is integrated, stored, snapshotted or given an inverse mass.
//
// `lineFrameOf` is that frame, and it answers the same two questions epFrame (§06.1)
// answers for a body and for the background:
//
//   angCols()             the columns of the frame's own ANGLE
//   minusPointAlong/Across  the columns of the frame's material point at (s, t),
//                         negated, since every caller measures something relative
//                         to it (see below)
//
// Three frame kinds, one interface: a body's angle column is its own theta, the
// background's is nothing at all (a fixed world frame), and a line's is the pair of
// endpoint closures below. That is the whole reason this exists as an object rather
// than as arithmetic inlined into the three row builders that used to each carry
// their own copy of it. See VERTEX.md §X.5.
//
// The angle. dphi/dt = n.(v_a - v_b)/L, so the line's angular column is (1/L) times
// the two endpoints' velocity columns along the segment normal -- which is what the
// closures build, and which is correct unchanged for a vessel endpoint, whose
// velCols carries the extra length column.
//
// The material point at (station s along u, lateral t along n) has world position
// P = P_a + s*u + t*n, and since u and n turn with the bar (du/dt = w*n,
// dn/dt = -w*u), its velocity is v_a + w*(s*n - t*u). Probed along u that is
// v_a.u - w*t; probed along n it is v_a.n + w*s. Those two are the whole of what the
// point-on-line and station rows need, and they are returned NEGATED because both
// rows measure a real point's motion RELATIVE to the bar's material point under it.
// Returned as a LIST of column arrays rather than pre-merged, so the caller merges
// once, flat -- which is what keeps a row's summation order (and so its last bit)
// exactly what it was before this frame was factored out.
function lineFrameOf(f){
  const {epA,epB,ux,uy,nx,ny,L,phi} = f;
  let w=null;
  const angCols = () => (w || (w = mergeCols([ epA.velCols(nx/L, ny/L),
                                               epB.velCols(-nx/L, -ny/L) ])));
  return {
    th: phi,
    angCols,
    // Where a world point sits in the bar's own frame: [station, lateral], both
    // measured from end a. The lateral is zero for any point the rows are holding.
    localOf: (wx,wy) => { const Dx=wx-f.wax, Dy=wy-f.way;
                          return [ux*Dx+uy*Dy, nx*Dx+ny*Dy]; },
    minusPointAlong:  (s,t) => [ epA.velCols(-ux,-uy), scaleCols(angCols(),  t) ],
    minusPointAcross: (s,t) => [ epA.velCols(-nx,-ny), scaleCols(angCols(), -s) ],
  };
}
// One row tying `here`'s frame angle to `there`'s, offset by a captured rest angle.
// `here` is an endpoint (a body's theta, or the fixed world zero for a background
// end) and `there` is the line frame above -- so this is "pin this end's rotation to
// the bar's heading", which is the ONE row that today appears under five names: a
// rod's and a rack's weldA/weldB, a slot's prismaticA/prismaticB, and an extra
// control point's lock. Written against two frames rather than against a
// segment, it is also the row a vertex will build between any two frames meeting at
// it (VERTEX.md §X.5), which is why it takes frames and not a constraint.
function frameAngleRow(here, there, restAng){
  const cols = mergeCols([ scaleCols(there.angCols(), -1), here.angCols() ]);
  return { cols, C: here.th-there.th-restAng };
}
// ---- §06.2 · the remaining constraint makers ----
// Belt, CVT, knife and cable were built as object literals at each of their
// call sites until the scene file (§17) needed a third one. Every constraint kind
// now has exactly ONE constructor, called from exactly two places -- the tool
// dispatch (§13.5) and the scene reader (§17.4) -- so "what fields does a belt
// have" has a single answer, and a scene file cannot describe a constraint the
// tools cannot build. See SCENE.md §S.2.

// A belt couples two disks' rim speeds. The wrap radii default to the bodies' own
// radii and the phase is captured from their live angles, so a freshly built belt
// is unstressed -- the same capture the wrap-radius and crossed-belt edits redo
// (inspector.js §14.2).
function makeBeltCon(aId,bId,sense){
  const A=bodies[bodyIndex(aId)], B=bodies[bodyIndex(bId)];
  const sn = sense===-1 ? -1 : 1;
  return {type:'belt', a:{id:aId}, b:{id:bId}, rA:A.r, rB:B.r, sense:sn,
          restPhase:(A.r*A.th - sn*B.r*B.th), sel:false};
}
// The variable-ratio rolling contact carries no captured state at all -- its ratio
// is read from the live geometry every step (§06.5).
function makeCvtCon(aId,bId){ return {type:'cvt', a:{id:aId}, b:{id:bId}, sel:false}; }

// A knife edge forbids sideways motion of one body-local point. `dir` is the
// heading in the body's OWN frame -- callers holding a world direction rotate it in
// by R(-b.th, ...) first, as the tool does.
function makeKnifeCon(a,dir){ return {type:'knife', a, dir, sel:false}; }

// A cable runs from a tether point (a body-local anchor, or the background) to a
// spool disk it winds on. Ltot and localAngle are captured from the live geometry:
// the free span at creation, and the rim point nearest the tether (spoolAngle = 0).
// See CABLE.md §C.3.
function makeCableCon(tether,spoolId){
  const S=bodies[bodyIndex(spoolId)];
  const [tx,ty]=epWorld(tether);
  const dvx=tx-S.x, dvy=ty-S.y, d=Math.hypot(dvx,dvy);
  return {type:'cable', tether, spool:{id:spoolId},
          localAngle:Math.atan2(dvy,dvx)-S.th, spoolAngle:0,
          Ltot: d>S.r ? Math.sqrt(d*d-S.r*S.r) : 0, sel:false};
}

// `mergeCols` sums duplicate body-index entries -- required, not cosmetic:
// physics.js's Schur assembly builds each row's per-body map with
// `mp.set(idx,...)`, which *overwrites* rather than accumulates, so two
// separate column entries for the same body would silently drop one and
// corrupt the row if a caller ever produced two entries on the same body.
// Scale every column of a row by k -- the companion to mergeCols, used wherever a
// row is a linear combination of frames rather than a plain difference of two (the
// bar-rotation term a line's rows carry, §06.2f).
function scaleCols(cols,k){ return cols.map(c=>[c[0], c[1]*k, c[2]*k, c[3]*k, (c[4]||0)*k]); }
function mergeCols(colArrays){
  const m=new Map();
  for(const cols of colArrays) for(const [idx,cx,cy,cw,cl] of cols){
    const l=cl||0;
    const e=m.get(idx); if(e){ e[0]+=cx; e[1]+=cy; e[2]+=cw; e[3]+=l; } else m.set(idx,[cx,cy,cw,l]);
  }
  return [...m.entries()].map(([idx,[cx,cy,cw,cl]])=>[idx,cx,cy,cw,cl]);
}
// Resolve a rod/pin/spring endpoint -- a plain body {id,off} or a background
// point {id:null,off} -- to its live world position plus a `velCols` closure
// giving the velocity-Jacobian columns for an arbitrary probe direction (a
// plain body reduces to the ordinary single-column rotate-form, background
// to no columns at all). Not used by rod's weld / slot's prismatic locks --
// those keep using twoPointFrame directly.
// `angCols` is the companion to velCols for the one row shape that measures an
// endpoint's own *rotation* rather than a point's translation (the rod weld and slot
// prismatic locks, endpointAngleLockRow below): the endpoint body's angular-velocity
// column, or none at all for a background endpoint, whose frame angle is the fixed
// world zero. `th` reads the same way -- a body's own theta, or 0 for background.
//
// On a vessel, `off` is the material label (lat, f) of §05.2c, and the point's world
// position picks up a length dependence: d(world)/d(len) = f * axis. That is the
// whole of the len column, and it is why an endpoint's *material fraction* is what
// decides how much it restrains the vessel's breathing.
function epFrame(ep){
  if(ep.id==null) return { wx:ep.off[0], wy:ep.off[1], idx:-1, th:0,
                           velCols:()=>[], angCols:()=>[] };
  const idx=bodyIndex(ep.id); const b=bodies[idx];
  const [wx,wy,rx,ry]=worldPt(b, epLocal(b,ep.off));
  if(b.shape==='vessel'){
    const f=ep.off[1], ax=vesselAxis(b);
    return { wx, wy, idx, th:b.th,
      velCols:(dirx,diry)=>[[idx, dirx, diry, dirx*(-ry)+diry*rx, f*(dirx*ax[0]+diry*ax[1])]],
      angCols:()=>[[idx,0,0,1,0]] };
  }
  return { wx, wy, idx, th:b.th,
    velCols:(dirx,diry)=>[[idx, dirx, diry, dirx*(-ry)+diry*rx]],
    angCols:()=>[[idx,0,0,1]] };
}

// ---- §06.2b · derived freezing (which coordinates a scene has pinned) ----
// `static` and `lenLock` are not properties a player sets. They are DERIVED, every
// substep, from the constraints actually present -- so a coordinate is frozen only
// when something in the scene says it is, and the thing that says so is a
// first-class object the player can see, select, and delete.
//
// Freezing is an optimization, not the physics: the solver already holds a
// constrained body exactly. What it buys is a coordinate removed from the system
// (and its now-redundant constraint compiled away, `_compiled` below) plus the
// island split that makes a fixed body a wall between what it touches.
//
// The same two structural patterns, said in the line's vocabulary (VERTEX.md §X.9).
// A LINE with exactly two joints, both held at a station and both welded, is the
// welded rod that grounded its far end: the background (or an already-grounded body)
// welded at one joint fixes the bar's heading, the station row fixes the distance,
// and the weld at the other joint fixes that body's angle -- three coordinates, gone.
// Still purely structural: it reads what is attached and never where anything is.
function lineGrounds(line){
  if(!isLine(line) || lineReleased(line)) return null;
  const J=lineJoints(line);
  if(J.length!==2) return null;
  if(J.some(K => K.e.slide || !K.e.weld)) return null;
  const w0=vertexWeldRef(J[0].v), w1=vertexWeldRef(J[1].v);
  if(!w0 || !w1) return null;                    // a weld with nothing to hold
  const held = e => { if(e.id==null) return true;
                      const b=bodies[bodyIndex(e.id)]; return !!(b && b.static); };
  const far = held(w0) ? w1 : held(w1) ? w0 : null;
  if(!far || far.id==null) return null;
  const b=bodies[bodyIndex(far.id)]; if(!b) return null;
  if(b.shape==='vessel' && far.off[1]!==0) return null;      // not the mid-plane
  return b;
}
// ...and a line whose two held joints ride two material planes of the SAME vessel
// holds the distance between two points that move only with the length, which is the
// length and nothing else. The strut inside a reservoir, unchanged in substance.
function lineLocksLength(line){
  if(!isLine(line) || lineReleased(line)) return null;
  const J=lineJoints(line);
  if(J.length!==2 || J.some(K=>K.e.slide)) return null;
  const a=vertexPrimary(J[0].v), b=vertexPrimary(J[1].v);
  if(!a || !b || a.id==null || a.id!==b.id) return null;
  const v=bodies[bodyIndex(a.id)];
  if(!v || v.shape!=='vessel' || a.off[1]===b.off[1]) return null;
  return v;
}
// Recompute every body's frozen flags and every constraint's `_compiled` mark.
// Iterated to a fixed point because grounding is transitive: a body double-welded to
// a body that is itself grounded is grounded too.
function refreshFrozen(){
  for(const b of bodies){ b.static=false; b.lenLock=false; }
  for(let pass=0; pass<=bodies.length; pass++){
    let changed=false;
    for(const con of constraints){
      const g=lineGrounds(con);
      if(g && !g.static){ g.static=true; changed=true; }
      const v=lineLocksLength(con);
      if(v && !v.lenLock){ v.lenLock=true; changed=true; }
    }
    if(!changed) break;
  }
  // A constraint that does the freezing has nothing left to solve: every column it
  // would write lands on a coordinate that no longer moves. Left in, it would be a
  // row of zeros that only the Tikhonov term keeps solvable, reporting a reaction
  // read off the regularizer rather than off the mechanism. Compile it away instead.
  for(const con of constraints)
    con._compiled = !!(lineGrounds(con) || lineLocksLength(con));
  for(const b of bodies) refreshInertia(b);   // the inverse masses follow the flags
}
// A body whose EVERY coordinate is frozen is a wall: nothing passes through it, so
// islands may split there (§08.0). A vessel pinned at its mid-plane is not one --
// its length is still a live channel between whatever is attached to it.
const frozenSolid = b => b.static && (b.shape!=='vessel' || b.lenLock);

// After a frozen body is moved by hand -- dragged, or its pose typed into the
// inspector -- nothing in the solver will pull its anchors back into agreement,
// because the rows that would have done so are compiled away. Recapture them from
// the new pose instead, exactly as creating the rod would have.
function recaptureGrounding(b){
  for(const con of constraints){
    if(lineGrounds(con)!==b && lineLocksLength(con)!==b) continue;
    recaptureConPose(con);
  }
}
// Re-read every ANGLE a line joint holds off the live geometry -- each locked end's
// rest angle (a rod's or a rack's weld, a slot's prismatic lock) and every extra
// joint's -- plus the unwrapping anchor those angles are measured against.
// This is exactly what captureRestAngle does when a lock is switched on, so a joint
// recaptured at a pose it already holds is unchanged.
//
// The unwrapping anchor goes first, and it is the subtle half. Every capture below
// reads the RAW segment angle (captureRestAngle), so the rows have to read it raw
// too, or a rod that was posed past the branch cut -- swung round its anchor through
// the -x direction -- comes out of the recapture holding a rest angle a full turn
// from the phi its own weld row measures against. Cleared, the next twoPointFrame
// re-seeds from the same raw atan2 the capture used, which is exactly the state a
// freshly loaded (or freshly Reset) scene is in. Safe wherever the joint is not
// being held across the edit -- compiled away, released for a drag (§06.2d), or
// carried bodily by a selection box (select.js §18.2) -- which is every caller here.
function recaptureConAngles(con){
  if(isVertex(con)){ recaptureVertex(con); return; }
  con._phiRef=undefined;
}
// ...and everything else a line joint holds: the angles above plus the two things
// that are LENGTHS along it -- a rod's rest length and every extra point's station.
// Together that is exactly what placing a line and its joints captures, so
// a joint recaptured at a pose it already satisfies is unchanged, and one recaptured
// after a hand move holds the new pose instead. Called from recaptureGrounding
// above, once per pose-drag step from recapturePosable (§06.2d), and from a scaled
// selection box (select.js §18.2). Safe on any constraint kind: a belt, a CVT or a
// knife holds none of these and comes out untouched but for the phi anchor.
function recaptureConPose(con){
  recaptureConAngles(con);
  // A VERTEX's counterpart to a rod's rest length. Its incidences are body-frame
  // offsets that agreed when it was placed; a scaled selection box spreads the
  // bodies without resizing them (select.js §18.2), so those offsets stop naming one
  // point. Re-seat every joined incidence on the PRIMARY's world point -- the same
  // "the geometry the transform left is the geometry to hold" rule the rod's length
  // and the points' stations follow, with the primary as the tiebreak because the
  // disagreeing anchors give no single live answer to re-read.
  if(isVertex(con)){
    const P=vertexPrimary(con);
    if(P){ const [wx,wy]=epWorld(P); setVertexWorld(con, wx, wy); }
    return;
  }
  // A line holds nothing of its own -- its stations and its welds' rest angles live
  // on the joints -- so re-reading it is re-reading them.
  if(isLine(con)){
    con._phiRef=undefined;
    for(const K of lineJoints(con)) recaptureVertex(K.v);
    return;
  }
}

// Every endpoint a coupling names. The one
// answer to "which bodies does this couple", read by the island pass (§08.0), the
// delete paths (§13.5, §14.2) and the body-resize rescale (§13.3).
function conEndpoints(con){
  const eps=[];
  // A vertex names its bodies through its incidences, which ARE {id, off} endpoints
  // (§06.2e) -- so islands, deletion, resizing and the selection's membership rule
  // all read it through this one function like everything else.
  // A line's bodies are the ones its joints are located by, plus the disks meshing
  // with it. Returned as fresh refs carrying only an id: the OFFSETS belong to the
  // vertices, which report them through this same function, and handing the same
  // object back twice would let a body resize scale it twice (tools.js §13.3).
  if(isLine(con)){
    const out=[];
    for(const K of lineJoints(con)) out.push({id:K.ep.idx>=0 ? bodies[K.ep.idx].id : null});
    for(const id of (con.mesh||[])) out.push({id});
    return out;
  }
  if(isVertex(con)) return vertexOns(con).filter(e => !isLineOn(e));
  if(con.a) eps.push(con.a);
  if(con.b) eps.push(con.b);
  return eps;
}
// Take a body (or a line -- one id space) out of everything that names it. What goes
// is the RELATION and nothing else: every vertex keeps its incidence list minus the
// row that named the departing body, every line keeps its joints minus the ones that
// named it, and NO OBJECT IS DELETED FOR HAVING LOST ONE.
//
// That is not tidiness, it is the model. A disk, a vertex and a line are three
// objects of equal standing; a constraint between them is a statement ABOUT them and
// never a claim on their existence (VERTEX.md §X.2, §X.15). This function used to
// cascade -- a vertex went once nothing located it, a line went once it was down to
// fewer than two joints, and the two ran to a fixed point -- so deleting one disk
// silently took the vertex at its centre AND the line between two of them. Each rule
// was true about ROWS and false about OBJECTS, and together they made the disk the
// owner of both. Under-determined is a state an object is in, not a reason to delete
// it.
//
// `settleVertex` runs first, on the pose the player is still looking at, so a vertex
// this body was placing keeps the place it had: it comes to ride a line it is joined
// to, or leaves a background mark where it stood (§06.2e). Nothing jumps to the
// origin and nothing disappears.
function dropBodyFromConstraints(id){
  for(const c of constraints){
    if(isVertex(c)){
      if(vertexOns(c).some(e => e.id===id)){
        const [wx,wy]=vertexWorld(c);
        c.on = vertexOns(c).filter(e => e.id!==id);
        settleVertex(c, wx, wy);
      }
    }
    else if(isLine(c)) c.mesh = (c.mesh||[]).filter(m => m!==id);
  }
  // The departing object itself, and the two-ended couplings that named it -- a belt,
  // a CVT, a knife edge -- which are relations with no identity apart from their ends.
  constraints = constraints.filter(c =>
    isVertex(c) ? true
    : isLine(c)  ? c.id!==id
    : (c.a.id!==id && !(c.b && c.b.id===id)));
  for(const c of constraints)
    if(c.pts && c.pts.length) c.pts = c.pts.filter(pt => pt.ep.id!==id);
}
// Deleting ONE coupling, from the delete tool, the keyboard and the panels alike. A
// line carries an id that vertices name, so its joints' incidences go with it -- and
// only those: the vertices themselves stay, at the places they are already at. Every
// other coupling is named by nothing and simply leaves.
//
// Routed through one function because the four delete paths had drifted apart: two
// spliced a line straight out of the array and left `on=` tokens pointing at an id
// the file no longer defines, which the reader rejects -- a bench that could not
// reload the scene it had just written.
function deleteConstraint(c){
  if(isLine(c)) dropBodyFromConstraints(c.id);
  else constraints = constraints.filter(x => x!==c);
}

// ---- §06.2e · the vertex (a named point, and the bodies it touches) ----
// A VERTEX is a named point. It carries a label, and a list of INCIDENCES -- one per
// body it touches -- and it has no coordinates of its own. That last clause is why
// it costs the engine nothing: a vertex is not a particle, it contributes no column,
// and it is never solved for. It is a NAME for a coincidence, and it compiles to the
// rows the pin already built. See VERTEX.md §X.2, §X.3.
//
// It lives in `constraints` because that is the list of couplings, and a vertex is
// one -- the pin generalized, and the pin lived there. Everything that walks
// couplings (the island pass §08.0, the row assembly §08.3, the projection §09.1,
// transport's scratch clearing §16.1, the selection's membership rule §18.1) works
// on it with no change at all.
//
//   { type:'vertex', label:'A',
//     on:[ {id, off, join, weld, restAng}, ... ] }
//
// `off` is the ordinary endpoint offset every anchor in the engine has (§05.2c),
// material on a vessel. `id === null` is the background -- a body with a fixed frame,
// which is what epFrame's null branch has always made it.
//
//   join   held to the vertex, rather than merely marking a spot on that body
//   weld   held to the vertex's FRAME as well as its point
//
// One incidence per body: a vertex is at one material place on each thing it
// touches, so a second incidence on the same body would be the vertex claiming to be
// in two places at once. The background included -- it is one body.
const vertexOns = v => v.on || (v.on=[]);
const isVertex = con => con && con.type==='vertex';
// What GROUNDS a vertex: the joined incidence whose frame has coordinates of its own.
// The BACKGROUND first, because it is the one that does not move; then a real body.
// A LINE is never one of these, and that is what keeps the whole thing acyclic: a
// line's own frame is read off the vertices on it (§06.2f lineFrame), so those
// vertices must be placed by something that is not the line. This is therefore also
// exactly what the rows resolve a vertex through, and what `lineJoints` demands of a
// joint before it will let it help place the bar.
const incidenceRank = e => e.id==null ? 0 : 1;
const isLineOn = e => e.kind==='line';
function vertexPrimary(v){
  let best=null;
  for(const e of vertexOns(v)){
    if(!e.join || isLineOn(e)) continue;
    if(!best || incidenceRank(e) < incidenceRank(best)) best=e;
  }
  return best;
}
// ...and a line CARRIES a vertex that nothing grounds. It does not place the bar --
// it is not in `lineJoints` and it builds no rows -- it simply rides it, at the
// station it holds, like a mark painted on the bar. That is not a second kind of
// locator sneaking past the rule above: the line's frame is derived without ever
// consulting this vertex, so the definition still bottoms out.
//
// Which is what the physics of it says too. A bare vertex has no mass and no forces
// on it, so nothing can drive it along its slot; whether the joint is ticked to slide
// makes no difference until something with mass is joined at it, and then it slides
// freely. So a vertex joined to a line moves with the line, as a joint that could not
// slide would -- and the moment a body is joined at it, it becomes an ordinary joint
// on the bar and the `slide` tick starts to mean what it says.
//
// The station is the one thing that has to be remembered: a point on a rail with
// nothing saying where along it is not a position, so a line with no station for this
// vertex carries nothing and the fallbacks below take over.
function vertexRide(v){
  if(vertexPrimary(v)) return null;
  for(const e of vertexOns(v)){
    if(!e.join || !isLineOn(e) || e.s===undefined) continue;
    if(lineById(e.id)) return e;
  }
  return null;
}
// ...and where the vertex IS, which is one question with one answer, in this order:
//
//   what GROUNDS it   a joined body, or the background
//   what CARRIES it   a line it is joined to, at its station
//   its own mark      the background incidence, which is simply its world coordinates
//
// A JOINED incidence always wins, which is the whole of the rule the first two lines
// state: joined to a body, the vertex moves with the body; joined only to a line, it
// moves with the line. An incidence on a body the vertex is NOT joined to never
// locates it -- it marks a material spot, saying where the vertex was rather than
// where it now is, so reading a position out of it would make letting go of a body
// silently the same as still being held by it. The trailing `find` is the legacy path:
// a hand-written file whose only incidence is such a mark still has to resolve to
// something, and that mark is all there is.
function vertexWorld(v){
  const ons=vertexOns(v);
  const P = vertexPrimary(v);
  if(P){ const [wx,wy]=epWorld(P); return [wx,wy]; }   // epWorld's trailing arm vector is a rod's business
  const R = vertexRide(v);
  if(R){ const p=lineStationPoint(lineById(R.id), R.s); if(p) return p; }
  const a = vertexAnchorWorld(v);
  return a || [0,0];
}
// Where a vertex is WITHOUT asking any line: what grounds it, else the mark it left.
// Null for a vertex that has neither.
//
// This is the position a LINE may be placed from, and the exclusion of the ride is
// what keeps the definition from closing on itself -- a line cannot be placed by a
// point it is itself carrying. `lineJoints` already makes the same exclusion for the
// ROWS, by demanding a primary; this is the same rule for the geometry.
function vertexAnchorWorld(v){
  const ons=vertexOns(v);
  const e = vertexPrimary(v) || ons.find(x=>x.id==null) || ons.find(x=>!isLineOn(x));
  if(!e) return null;
  const [wx,wy]=epWorld(e);
  return [wx,wy];
}
// Settle which of those three says where the vertex is, after anything that changes
// what it is joined to. Exactly one of them does, so this both picks it and makes sure
// it CAN: a carrying line needs a station, and an ungrounded, uncarried vertex needs
// its background mark. `wx,wy` is where the vertex stood BEFORE the change, so every
// path here holds the pose it found -- the discipline every capture toggle follows.
//
// The one place it does move the point is a vertex the line has just taken over: it
// goes onto the bar, because that is what riding the bar means and it was only ever
// within a glyph's width of it. Nothing else moves, and it carries no rows either way.
function settleVertex(v, wx, wy){
  const ons=vertexOns(v);
  if(vertexPrimary(v)){
    // Grounded: its line joints are ordinary joints again, and a SLIDING one owns no
    // station -- leaving a stale one behind would be a number the file keeps and
    // nothing reads.
    for(const e of ons) if(isLineOn(e) && e.join && e.slide) delete e.s;
    return;
  }
  for(const e of ons){
    if(!e.join || !isLineOn(e)) continue;
    const line=lineById(e.id); if(!line) continue;
    // A carrying line answers only for as long as it can place itself, and it may
    // stop: it takes two placed joints, and deleting a body under one of them leaves
    // one. Where it cannot, it carries nothing -- the station goes rather than being
    // left behind as a number measured against a bar that had no origin to measure
    // from, which would put the point back at station 0 the moment the bar could be
    // placed again, on top of whatever sits there. The mark below is the BOTTOM of
    // the chain, not a fourth answer competing with the other three.
    if(!linePlacement(line)){ delete e.s; break; }
    e.s = lineStationAt(line, wx, wy);
    return;
  }
  const bg = ons.find(e=>e.id==null);
  if(!bg) makeVertexOn(v, {id:null, off:[wx,wy]}, {join:false});
  else if(!bg.join) bg.off=[wx,wy];
}
// The first WELDED incidence: the frame every other welded one is held against. A
// vertex with fewer than two welds holds no angle, and that is not a wart -- welding
// is a relation between two frames, and one frame has nothing to relate to. The
// inspector says so rather than pretending (§14.2).
// Body incidences only: a welded LINE at this vertex is tied to this same reference,
// but the row that does it is built by the line (which has already worked out its own
// frame), not here. Either way the reference is one and the same.
function vertexWeldRef(v){
  for(const e of vertexOns(v)) if(e.join && e.weld && !isLineOn(e)) return e;
  return null;
}
// An incidence's captured rest angle is its body's OWN angle at the moment the weld
// went on -- an absolute world angle, not one measured against another incidence. So
// each capture is independent of every other, the row between two welds is
// (th_i - rest_i) - (th_j - rest_j), and which incidence happens to be the reference
// cannot matter. It also needs no unwrapping anchor: a body's `th` is never wrapped,
// so there is no atan2 branch cut here of the kind twoPointFrame has to track.
const incidenceAngle = e => e.id==null ? 0 : (bodies[bodyIndex(e.id)] || {th:0}).th;

// THE constructor for a vertex (SCENE.md §S.2), and THE constructor for one of its
// incidences -- called by the tool dispatch (§13.5) and the scene reader (§17.4) and
// nowhere else.
function makeVertex(label){
  return { type:'vertex', label: label || nextVertexLabel(), on:[], pts:[], sel:false };
}
// `opts.restAng` is the file's captured value; omitted, it is read off the live
// geometry, so a freshly welded incidence starts exactly where it already was.
function makeVertexOn(v, ep, opts){
  const o = opts || {};
  const id = ep.id==null ? null : ep.id;
  const ons = vertexOns(v);
  if(ons.some(e => e.id===id)) return null;         // one incidence per body
  const line = id!=null ? lineById(id) : null;
  const e = line ? { id, kind:'line', slide: o.slide!==undefined ? !!o.slide : true,
                     join: o.join!==undefined ? !!o.join : true, weld:false }
                 : { id, off: ep.off ? ep.off.slice() : [0,0],
                     join: o.join!==undefined ? !!o.join : true, weld:false };
  ons.push(e);
  // Pushed BEFORE the captures: a station is read off the line's own live frame, and
  // the frame is built from the joints, so this joint has to be one of them first.
  // A SLIDING joint takes a station only from the file, and only a vertex the line
  // carries is written with one -- a slider that something grounds owns none.
  if(line && e.join && (!e.slide || o.s!==undefined)){
    // The file's captured station, or -- for a freshly placed joint -- the one the
    // live geometry shows, taken against the origin this joint may itself have just
    // become (recaptureLineStations settles the rest of them either way).
    if(o.s!==undefined) e.s = o.s;
    else { e.s = captureLineStation(line, v); recaptureLineStations(line); }
  }
  if(e.join && o.weld){ e.weld=true;
    e.restAng = o.restAng!==undefined ? o.restAng
              : (line ? lineAngle(line) : incidenceAngle(e)); }
  return e;
}
// A line's own frame angle, for a weld captured against it -- the RAW segment angle,
// never the unwrapped one, for the reason captureRestAngle gives: a rest angle
// persists into the file while `_phiRef` is transient scratch that Reset clears, so a
// capture taken against an unwrapped phi would disagree by a whole turn afterwards.
function lineAngle(line){
  const f=lineFrame(line);
  return f ? Math.atan2(f.way-f.wby, f.wax-f.wbx) : 0;
}
// Turning `join` ON re-reads the offset from where the vertex actually is, so the
// tick never snaps anything -- the same discipline every capture in the engine
// follows, so a toggle holds the pose it found. Turning it off takes the weld with it: welding is
// something a JOINED body does, and a weld on a body the vertex is not held to would
// tie an angle to a point that is not there.
function setVertexJoin(v, e, val){
  if(!!val === !!e.join) return;
  if(val){
    const [wx,wy]=vertexWorld(v);                // where it stood, before the tick
    e.join = true;
    if(isLineOn(e)){ const line=lineById(e.id);
      if(line){ if(!e.slide) e.s = captureLineStation(line, v); recaptureLineStations(line); } }
    else {
      e.off = e.id==null ? [wx,wy] : epOffOf(bodies[bodyIndex(e.id)], wx, wy);
    }
    settleVertex(v, wx, wy);
  } else {
    // Where it is BEFORE letting go, so the release holds the pose it found -- the
    // same discipline every capture toggle here follows.
    const [wx,wy]=vertexWorld(v);
    e.join=false; e.weld=false; delete e.restAng; delete e.s;
    settleVertex(v, wx, wy);
  }
}
function setVertexWeld(v, e, val){
  if(!e.join){ e.weld=false; delete e.restAng; return; }
  e.weld = !!val;
  if(e.weld) e.restAng = isLineOn(e) ? lineAngle(lineById(e.id)) : incidenceAngle(e);
  else delete e.restAng;
}
// A joint that stops sliding captures the station it is at, so the tick never snaps it
// along the bar; one that starts sliding has no station to hold -- unless the line is
// what CARRIES the vertex, in which case the station is not a constraint at all but
// the record of where on the bar the point rides, and it stays whichever way the tick
// goes. (While the vertex is bare that tick changes nothing anyway: nothing pushes a
// point with no mass along its slot. It starts meaning something the moment a body is
// joined at the vertex, and settleVertex drops the station then.)
function setVertexSlide(v, e, val){
  if(!isLineOn(e)) return;
  const [wx,wy]=vertexWorld(v);
  const line=lineById(e.id);
  e.slide = !!val;
  if(e.slide) delete e.s;
  else e.s = line ? captureLineStation(line, v) : 0;
  if(line) recaptureLineStations(line);
  settleVertex(v, wx, wy);
}
// Re-read every weld's rest angle off the live pose -- recaptureConAngles' vertex
// counterpart, called from the same places (a hand move, a scaled selection box).
function recaptureVertex(v){
  for(const e of vertexOns(v)){
    if(!e.join) continue;
    if(isLineOn(e)){
      const line=lineById(e.id); if(!line) continue;
      line._phiRef=undefined;                    // see recaptureConAngles
      if(e.weld) e.restAng = lineAngle(line);
      if(!e.slide || e===vertexRide(v)) e.s = captureLineStation(line, v);
    } else if(e.weld) e.restAng = incidenceAngle(e);
  }
}
// Move the whole coincident set to a world point: every incidence re-reads its own
// offset, so the bodies stay where they are and the vertex moves between them.
// What the pivot handle drags (§13.3) and what the inspector's position field commits.
//
// EVERY incidence, joined or not, and that is the point of it: an offset is where the
// vertex sits in that body's frame, and after the move it sits somewhere else. An
// unjoined one is a spot the point is AT rather than a hold on it, so nothing about it
// resists the move -- but it is still a statement about where the point is, and left
// unread it would be the one number on the panel that had stopped saying so while the
// row beside it kept up (§14.2c). Moving the BODY is the other direction and is
// untouched: a mark is material, so the body takes it along and leaves the vertex.
function setVertexWorld(v, wx, wy){
  const ons=vertexOns(v);
  // Unless the point has left the body altogether. An unjoined incidence outside its
  // body's extent is a coordinate neither list will show (verticesInExtent below is
  // what both of them read) and no gesture could reach again, so the move that carried
  // the point out is what ends it -- dropped here, at that gesture, rather than left
  // stored where no panel can say it is there. A JOINED incidence is never dropped: it
  // is what holds the vertex, and where it holds it from is its own business.
  for(let i=ons.length-1;i>=0;i--){
    const e=ons[i];
    if(e.join || e.id==null) continue;
    if(!extentCovers(e.id, wx, wy)) ons.splice(i,1);
  }
  // ...and nothing is ever left with nowhere to be: where the mark just dropped was
  // the one saying where the vertex is, the background takes it over at the point,
  // which is the same bottom of the chain settleVertex plants (§06.2e).
  if(!vertexAnchorWorld(v) && !vertexRide(v)) makeVertexOn(v, {id:null, off:[wx,wy]}, {join:false});
  for(const e of ons){
    if(isLineOn(e)) continue;                    // a line holds no offset to re-read
    // The background incidence is re-read like the rest: joined it is a ground pin,
    // unjoined it is simply where the vertex is (vertexWorld above), and either way it
    // has to follow the point.
    if(e.id==null){ e.off=[wx,wy]; continue; }
    const b=bodies[bodyIndex(e.id)]; if(!b) continue;
    e.off = epOffOf(b, wx, wy);
  }
  // A vertex the line CARRIES has no offset anywhere that says where it is -- its
  // station does. Moving it therefore moves it ALONG the bar, to the station nearest
  // the point asked for, which is the only place on the bar there is to put it. A HELD
  // joint's station is not re-read here, because this is also the path a TYPED
  // coordinate commits through, and a typed number is a solve attempt the bar may
  // refuse (VERTEX.md §X.11). A drag is the other thing -- the hand sets the length --
  // and it re-reads the stations itself, off the geometry the gesture left
  // (tools.js §13.3, recaptureLineStations).
  const R=vertexRide(v);
  if(R) R.s = lineStationAt(lineById(R.id), wx, wy);
}
// Every vertex touching a body, and every body a vertex touches -- the one relation,
// read from either side. This is the INCIDENCE relation, which is what the rows, the
// scene walk and the selection's membership rule read; what the two inspector lists
// show is the wider one just below.
const verticesOn = id => constraints.filter(c => isVertex(c) && vertexOns(c).some(e=>e.id===id));

// ---- what the two lists LIST: extent, not incidence (§14.2c) ----
// A vertex sitting inside a body is at a place on that body whether or not anything
// has said so yet, and that is what the panels show: every body whose extent covers
// the point, with the joined ones ticked. The list is therefore not a record of what
// has been attached -- it is what the vertex is *at*, which is a fact about the
// geometry and not about the editing history. Three things follow, and they are the
// point of it:
//
//   * `joined` is the only control such a list needs. Ticking it on makes the
//     incidence; ticking it off releases it and leaves the body listed, holding
//     nothing. There is no "remove" to press, and so nothing a press can lose.
//   * the background is in every list. Its extent is the whole plane -- which is what
//     a body with a fixed frame comes to -- and a ground pin is what ticking it on is.
//   * a body under another body is listed too. Depth decides what a CLICK lands on
//     (§13.2 pickBody); it has nothing to say about what a point is inside.
//
// A body's extent is its own outline, so `bodyContains` (§05.2) answers it exactly. A
// LINE has no width, so "on it" has to be a tolerance -- and the tolerance is a
// question about the DRAWING, not about the metres: the vertex is on the line when
// its dot touches the line's stroke, which is what a person tapping the two together
// is judging. So it is measured in PIXELS and converted through the camera, the way
// every other hit test here already is (§13.2). A fixed world distance would mean
// something different at every zoom -- a millimetre is untouchable zoomed out and
// enormous zoomed in -- and it would make the obvious gesture, tap a spot on the line
// and then tick `joined`, work or not work depending on how far you had scrolled.
//
// The number is the vertex dot's own radius (3.5px, render.js §11.4) plus a bar's
// half-stroke (1.5px, §11.5): the two glyphs touching, and nothing more.
const LINE_EXTENT_PX = 5;
const lineExtentTol = () => LINE_EXTENT_PX / (cam.scale || 1);
function lineCovers(line, wx, wy){
  const f=linePlacement(line); if(!f) return false;
  const tol=lineExtentTol();
  if(Math.abs(f.nx*(wx-f.wax) + f.ny*(wy-f.way)) > tol) return false;
  if(!lineIsBar(line)) return true;                    // a rail runs on past its joints
  const du = f.ux*(wx-f.wax) + f.uy*(wy-f.way);        // P sits at 0, Q at -L (§06.2f)
  return du <= tol && du >= -f.L - tol;
}
function extentCovers(id, wx, wy){
  if(id==null) return true;
  const L=lineById(id); if(L) return lineCovers(L, wx, wy);
  const b=bodies[bodyIndex(id)];
  return !!b && bodyContains(b, wx, wy);
}
// The vertex's side of it: one SITE per thing the vertex is at, each carrying the
// incidence that holds it there or null where nothing does yet. Ordered background,
// bodies, lines -- deterministic, and the order `vertexPrimary` already ranks by. An
// incidence naming something the list did not reach is appended rather than dropped,
// so a stored one is never hidden behind a list read off live geometry.
function vertexSites(v){
  const [wx,wy]=vertexWorld(v);
  const ons=vertexOns(v);
  const at = id => ons.find(e=>e.id===id) || null;
  const out=[{ id:null, e:at(null) }];
  for(const b of bodies){ const e=at(b.id); if(e || bodyContains(b,wx,wy)) out.push({id:b.id, e}); }
  for(const c of constraints){ if(!isLine(c)) continue;
    const e=at(c.id); if(e || lineCovers(c,wx,wy)) out.push({id:c.id, e}); }
  const shown=new Set(out.map(s=>s.id));
  for(const e of ons) if(!shown.has(e.id)) out.push({id:e.id, e});
  return out;
}
// ...and the body's side of the same list. `verticesOn` above stays what it was --
// the incidence relation, which is what the row assembly and the scene walk want;
// this one is what a PANEL wants, and the difference between them is the whole of the
// paragraph above.
function verticesInExtent(id){
  const out=[];
  for(const c of constraints){
    if(!isVertex(c)) continue;
    if(vertexOns(c).some(e=>e.id===id)){ out.push(c); continue; }
    const [wx,wy]=vertexWorld(c);
    if(extentCovers(id, wx, wy)) out.push(c);
  }
  return out;
}

// Labels. A vertex defaults to A, B, ... Z, AA, AB, ..., taking the first name not
// already in use, so deleting one frees its letter again. One namespace with bodies
// is the plan (VERTEX.md §X.8); bodies keep their numeric ids as labels until the
// format version that gives them their own.
function labelFor(n){
  let s='';
  for(n=n+1; n>0; n=Math.floor((n-1)/26)) s = String.fromCharCode(65+(n-1)%26) + s;
  return s;
}
function nextVertexLabel(){
  const taken = new Set(constraints.filter(isVertex).map(c=>c.label));
  for(let n=0;;n++){ const s=labelFor(n); if(!taken.has(s)) return s; }
}

// ---- §06.2d · the pose-time release ----
// While the player drags a body with the sim paused (tools.js §13.6), a LINE marked
// `posable` and touching that body is RELEASED for the length of the gesture: every
// joint slides and nothing is welded, so the bar holds only its own line and the
// mechanism articulates around the hand. It is rigid again, at the geometry the drag
// reached, the moment the drag step is over (§06.2f lineReleased, recapturePosable
// below). A line one joint further away stays rigid, so the release reaches exactly
// as far as the hand does.
//
// Two pieces of state, because the release answers two questions on two timescales,
// and a canvas frame drawn BETWEEN pointermoves is what exposes the difference:
//
//   posingRoot  the body a pose GESTURE is dragging, held pointerdown to pointerup.
//               This is what the line IS to the player, so it is what the canvas
//               draws (render.js §11.5).
//   posing      whether we are inside the row-building scope right now. Between two
//               pointermoves it is zero and the line is rigid again, which is what
//               keeps conMaxC, the violation highlight and saveState honest.
//
// Both live here rather than in the tool layer so that render.js -- which loads
// BEFORE tools.js -- never has to reach forward for them. It did once, and a forward
// reference that fails to resolve throws inside render(), which kills the rAF chain
// in §10 outright: the page stops redrawing and never starts again.
let posing = 0, posingRoot = null;
function beginPosing(rootId){ posingRoot = rootId==null ? null : rootId; }
function endPosing(){ posingRoot = null; }
function withPosing(fn){ posing++; try { return fn(); } finally { posing--; } }
// Whether a drag on body `rootId` would release this rod -- the predicate on its own,
// with no reference to any of the state above, so a check can ask it about a gesture
// that is not happening (tools/posable-check.js does).

// Once the drag step has settled, a released rod re-reads what it holds from the
// pose the player just produced (§06.2b recaptureConPose): the length, the welds'
// rest angles, the points' stations. Done every pointermove rather than at
// pointerup, for the same reason recaptureGrounding is: between moves the rod is a
// rigid rod again, and one whose captured length disagreed with its live geometry
// would read as a violated constraint -- red on the canvas, and refused as the reset
// baseline (transport.js §16.1) -- for as long as the drag lasted. It walks the same
// rods the release did, so a posable rod the drag never touched keeps the length it
// was authored at. A released rod that was under LOAD when the drag began does lose
// that load, since a released rod is not holding anything for the projection to
// fight: an adjustable member has no preload to speak of, and that is what marking
// one posable declares it to be.
function recapturePosable(){
  for(const con of constraints)
    if(linePosing(con)) recaptureConPose(con);
}

// ---- §06.2f · the line (a straight bar with a frame and no coordinates) ----
// A LINE is a body: a straight, massless bar. Its frame is not stored -- it is
// DERIVED from the vertices joined to it (§06.2e), which is what lets it be a body
// without being a coordinate. So it lives in `constraints` beside the vertex, for the
// same reason: that array is the list of things that produce ROWS, and `bodies` is
// the list of things that produce COLUMNS. See VERTEX.md §X.4, §X.5.
//
//   { type:'line', id, label, soft, posable, mesh:[bodyId,...] }
//
// Its joints are the vertices whose incidence list names it. Every joined joint lies
// ON the line -- that is what joining to a line means -- and the incidence's `slide`
// says only whether it additionally holds a STATION, a material position along the
// bar. Two things follow from the joints, and they are not the same thing:
//
//   PLACEMENT   where the line is and which way it points, from the two joints
//               FURTHEST APART (sliding or not, since every joint is on the line).
//               The longest available baseline, which is better conditioned than the
//               rod's was -- a rod's frame was whichever two ends the tool placed
//               first.
//   ORIGIN      what stations are measured from: the first non-sliding joint in
//               station order. It exists only when some joint does not slide, because
//               a station is a distance from a MATERIAL point and an origin that slid
//               would make every station meaningless. (This is the rack's asymmetry
//               -- "a pins the rack, b only aims it" -- promoted to the general rule.)
//
// Every joint slides by DEFAULT, so a fresh two-joint line is placement and nothing
// else: no rows, a drawn guide, exactly what a two-pin slot always was. A third joint
// adds one on-line row -- a rail with a rider. Untick `slide` on TWO and the distance
// between them is held -- a rod. Three objects, one branch-free rule.
//
// "on two" is not a slip. A station is a distance from the origin, so ONE held joint
// holds nothing: it is the origin, and an origin has nothing to be measured against.
// That is the same shape as one weld at a vertex holding nothing (§06.2e), and it has
// the same reason -- both are relations, and a relation needs two parties. To pin a
// rider in the world, hold one of the joints that places the line as well.
const isLine = con => con && con.type==='line';
const lineById = id => { for(const c of constraints) if(isLine(c) && c.id===id) return c; return null; };
// THE constructor for a line (SCENE.md §S.2), called by the tool dispatch (§13.5) and
// the scene reader (§17.4) and nowhere else. It takes an id from the same allocator
// bodies use, because a line IS a body as far as anything naming one is concerned --
// one namespace, so an incidence names a disk and a line the same way.
function makeLine(){
  const id = uid++;
  return { type:'line', id, label:String(id), soft:0, posable:false, mesh:[], sel:false };
}
// The joints on a line, each with the frame of the vertex's own LOCATOR -- the body
// or background incidence that says where the vertex is (§06.2e vertexPrimary). A
// vertex with no locator is not on the line in any sense the rows can use: a line
// holds points, and that vertex has none.
function lineJoints(line){
  const out=[];
  for(const c of constraints){
    if(!isVertex(c)) continue;
    const e = vertexOns(c).find(x => x.kind==='line' && x.id===line.id && x.join);
    if(!e) continue;
    const P = vertexPrimary(c); if(!P) continue;
    out.push({ v:c, e, ep:epFrame(P) });
  }
  return out;
}
// The line's live geometry: the extreme pair that places it, the origin that stations
// are measured from, and the joints in station order. Shaped exactly like
// twoPointFrame's return (§06.1) -- epA/epB, wax/way, u, n, L, phi -- so lineFrameOf
// (§06.1b) and everything built on it work on a line unchanged.
//
// phi is unwrapped against line._phiRef for the reason twoPointFrame documents: a
// weld row measures th - phi against a rest angle, and a raw atan2 jumps by a whole
// turn as the bar swings through pointing along -x.
function lineFrame(line){
  const J = lineJoints(line);
  if(J.length<2) return null;
  // The two furthest apart. They are also the extremes in station order: a joint
  // projecting beyond one of them would be further from the other, which is what
  // "furthest apart" rules out. O(n^2) over a handful of joints.
  let P=null, Q=null, best=-1;
  for(let i=0;i<J.length;i++) for(let j=i+1;j<J.length;j++){
    const d=(J[i].ep.wx-J[j].ep.wx)**2 + (J[i].ep.wy-J[j].ep.wy)**2;
    if(d>best){ best=d; P=J[i]; Q=J[j]; }
  }
  if(!(best>1e-18)) return null;                 // every joint at one point: no direction
  // WHICH of the pair is P fixes the line's heading, and a weld measures against it,
  // so a flip is a half-turn error in every weld on the bar. Two things pin it down.
  // With no memory -- a fresh load, or just after a Reset -- P is whichever of the
  // two comes first in the line's own joint order, which is the order the vertices
  // sit in `constraints`, which is the order the file wrote them: deterministic, and
  // the same before and after a round trip. Within a session `_phiRef` then keeps it
  // continuous, which is what stops a joint added BEYOND the old P (it can only ever
  // enter the pair as Q, being later in the array) from turning the bar end for end.
  if(line._phiRef!=null){
    const raw=Math.atan2(P.ep.wy-Q.ep.wy, P.ep.wx-Q.ep.wx);
    let d=raw-line._phiRef;
    while(d> Math.PI) d-=Math.PI*2;
    while(d<-Math.PI) d+=Math.PI*2;
    if(Math.abs(d) > Math.PI/2){ const t=P; P=Q; Q=t; }
  }
  const dx=P.ep.wx-Q.ep.wx, dy=P.ep.wy-Q.ep.wy, L=Math.hypot(dx,dy)||1e-9;
  const ux=dx/L, uy=dy/L, nx=-uy, ny=ux;
  const phiRaw=Math.atan2(dy,dx);
  let phi=phiRaw;
  if(line._phiRef!=null){
    let da=phiRaw-line._phiRef;
    while(da> Math.PI) da-=Math.PI*2;
    while(da<-Math.PI) da+=Math.PI*2;
    phi=line._phiRef+da;
  }
  line._phiRef=phi;
  // Station order, measured from P along the same u twoPointFrame uses (which points
  // from the second end to the first, so Q sits at -L).
  for(const K of J) K.du = ux*(K.ep.wx-P.ep.wx) + uy*(K.ep.wy-P.ep.wy);
  // The ORIGIN is the first held joint in JOINT order -- the order the vertices sit
  // in `constraints`, which is the order the file wrote them -- and NOT the first in
  // station order. Stations are stored relative to it, so an origin that moved when a
  // joint was added or the bar swung would silently reinterpret every one of them;
  // pinned to joint order it can only change when the joint set itself does, and
  // that is an authoring action the captures are re-read for (recaptureLineStations).
  const O = J.find(K => !K.e.slide) || null;
  J.sort((a,b)=>b.du-a.du);                      // P first, Q last
  return { J, P, Q, O, epA:P.ep, epB:Q.ep,
           wax:P.ep.wx, way:P.ep.wy, wbx:Q.ep.wx, wby:Q.ep.wy,
           ux, uy, nx, ny, L, phi };
}
// A line with any sliding joint is a RAIL and is drawn infinite; with every joint
// held at a station it is a BAR, drawn between its extremes. Derived, not a field:
// what a line is IS what its joints do, and there is nothing else to say.
// ---- where a line is, for the eye rather than for the rows ----
// `lineFrame` above is the KINEMATIC frame: it is built out of the joints whose
// vertices something GROUNDS, because a row needs columns and only a grounded vertex
// has any. That is right for the physics and wrong for everything else, because a
// line does not stop existing when the bodies under its joints do. A vertex always
// knows where it is, so a line always knows where it is too -- it just may have
// nothing to hold anything to.
//
// So: every joined joint, at its vertex's ANCHOR -- what grounds it, else the mark it
// left, never the ride. Excluding the ride is what stops the definition closing on
// itself: a line cannot be placed by a point it is itself carrying (§06.2e).
function lineJointsAll(line){
  const out=[];
  for(const c of constraints){
    if(!isVertex(c)) continue;
    const e = vertexOns(c).find(x => x.kind==='line' && x.id===line.id && x.join);
    if(!e) continue;
    const w = vertexAnchorWorld(c); if(!w) continue;
    out.push({ v:c, e, ep:vertexPrimary(c) ? epFrame(vertexPrimary(c)) : null, wx:w[0], wy:w[1] });
  }
  return out;
}
// The line's PLACEMENT: the kinematic frame wherever there is one -- so the canvas,
// the picker and the panel agree with the solver in every ordinary case -- and a
// geometry-only stand-in built the same way out of the joints' anchors where there is
// not. The stand-in carries no `epA`/`epB` and no columns, and nothing that builds
// rows may take it: `rowsFor`, the projection, the energy ledger and the captures all
// go on reading `lineFrame` and getting null, which is the honest answer that a line
// holding nothing has no rows.
function linePlacement(line){
  const f=lineFrame(line); if(f) return f;
  const J=lineJointsAll(line);
  if(J.length<2) return null;
  let P=null, Q=null, best=-1;
  for(let i=0;i<J.length;i++) for(let j=i+1;j<J.length;j++){
    const d=(J[i].wx-J[j].wx)**2 + (J[i].wy-J[j].wy)**2;
    if(d>best){ best=d; P=J[i]; Q=J[j]; }
  }
  if(!(best>1e-18)) return null;                 // every joint at one point: no direction
  const dx=P.wx-Q.wx, dy=P.wy-Q.wy, L=Math.hypot(dx,dy)||1e-9;
  const ux=dx/L, uy=dy/L;
  for(const K of J) K.du = ux*(K.wx-P.wx) + uy*(K.wy-P.wy);
  const O = J.find(K => !K.e.slide) || null;
  J.sort((a,b)=>b.du-a.du);
  return { J, P, Q, O, epA:null, epB:null,
           wax:P.wx, way:P.wy, wbx:Q.wx, wby:Q.wy,
           ux, uy, nx:-uy, ny:ux, L, phi:Math.atan2(dy,dx), placedOnly:true };
}
// Read off every joint, not just the grounded ones, so a bar does not read as a rail
// the moment a body under one of its joints goes away.
const lineIsBar = line => { const J=lineJointsAll(line); return J.length>=2 && J.every(K=>!K.e.slide); };
// The station a joint currently sits at, measured from the line's origin -- what
// makeVertexOn captures when a joint stops sliding, and what the segment distances
// the inspector lists are differences of.
// A joint's own world point IS the vertex's, so this is `lineStationAt` read off the
// vertex rather than off the joint list -- which is also what makes it work for a
// vertex the line CARRIES, which is not in the joint list at all (§06.2e vertexRide).
function captureLineStation(line, v){
  const [wx,wy]=vertexWorld(v);
  return lineStationAt(line, wx, wy);
}
// The station a world POINT sits at -- the same measure, read off the geometry rather
// than off the joint list, so a vertex that merely LIES on the line has one too and
// the panel can show it (§14.2c). For a vertex that is a joint the two agree exactly:
// a joint's `du` is taken from its locator's world point, which is where the vertex is.
function lineStationAt(line, wx, wy){
  // The PLACEMENT, not the kinematic frame: this is what a panel shows, what a typed
  // station commits against and what a carried vertex is placed by, so it has to
  // agree with the line the canvas draws even where that line holds nothing. The
  // captures that write a JOINT's `e.s` -- which the rows read -- use lineFrame.
  const f=linePlacement(line); if(!f) return 0;
  return f.ux*(wx-f.wax) + f.uy*(wy-f.way) - (f.O ? f.O.du : 0);
}
// ...and back again: the world point a station names. What the panel's station field
// commits for a joint that holds no station of its own -- a slider, or a vertex on
// the line with nothing joining it there.
function lineStationPoint(line, s){
  const f=linePlacement(line); if(!f) return null;
  const du = s + (f.O ? f.O.du : 0);
  return [f.wax + f.ux*du, f.way + f.uy*du];
}
// The distances between consecutive NON-SLIDING joints, in station order: what the
// inspector lists (VERTEX.md §X.4) and what the file writes. A line with fewer than
// two of them has none.
function lineSegments(line){
  const f=lineFrame(line); if(!f) return [];
  const held=f.J.filter(K=>!K.e.slide);
  const out=[];
  for(let i=1;i<held.length;i++)
    out.push({ from:held[i-1], to:held[i], len:Math.abs((held[i].e.s||0)-(held[i-1].e.s||0)) });
  return out;
}
// Re-read every station on a line, against whatever its origin is now. Called when
// the joint set changes or a joint starts or stops sliding -- both of which can move
// the origin, and a station means nothing except relative to the origin it was taken
// from. Like every other capture toggle in the engine (a rod's weld, a point's lock),
// it holds the pose it finds: a bar recaptured at a pose it already satisfies is
// unchanged, and one recaptured after a hand edit holds the new pose instead.
function recaptureLineStations(line){
  // The riders' stations are measured from the same origin, so a move of the origin
  // would silently shift them too -- and unlike a joint's, a rider's station IS its
  // position, so that shift would visibly slide it along the bar. Their world points
  // are read first and their stations put back from those afterwards, which leaves
  // them exactly where they were whatever the origin does (§06.2e vertexRide).
  const riders=[];
  for(const c of constraints){
    if(!isVertex(c)) continue;
    const R=vertexRide(c);
    if(R && R.id===line.id) riders.push([R, vertexWorld(c)]);
  }
  const f=lineFrame(line); if(!f || !f.O) return;
  for(const K of f.J) if(!K.e.slide) K.e.s = K.du - f.O.du;
  for(const [R,[wx,wy]] of riders) R.s = lineStationAt(line, wx, wy);
}
// A posable line releases for a pose drag exactly as a posable rod did (§06.2d):
// while the player drags a body it touches, every joint slides and nothing is welded,
// so the bar is a bare rail for the length of the gesture and rigid again the moment
// it ends, at the geometry the drag reached.
const linePosableFor = (line, rootId) => isLine(line) && !!line.posable && rootId!=null
  && conEndpoints(line).some(ep => ep.id===rootId);
const linePosing = line => linePosableFor(line, posingRoot);
const lineReleased = line => posing>0 && linePosing(line);

// ---- §06.3 · cableFrame ----
// Cable geometry based on a consistently-defined spool angle.
//
// Key points (A, B, C, D per the spec):
//   A = spool anchor -- material point on spool rim, stored as cb.localAngle in
//       the spool body frame.  Initialised so spoolAngle = 0 (anchor at closest
//       rim point to tether).
//   B = spool centre (S.x, S.y)
//   C = tether point T
//   D = tangent point -- rim point where tangent from T touches the spool on the
//       winding side determined by sign(spoolAngle).
//
//   spoolAngle = ABC angle at B (from ray BA to ray BC, CCW positive, unbounded):
//     = tetherAngle - anchorAngle.
//     Positive -> anchor is CW of tether direction -> cable winds CW.
//     Negative -> anchor is CCW of tether direction -> cable winds CCW.
//
//   Tangent point D (world angle from B):
//     d > rs: tangentAngle = tetherAngle - sign(spoolAngle) · arccos(rs/d)
//     d <= rs: tangentAngle = tetherAngle (rim point closest to T; or anchorAngle if d~=0)
//
//   |DBC| = arccos(rs/d) for d>rs; 0 for d<=rs.
//   Q = D (tangent wins) if |DBC| < |spoolAngle|;  else Q = A.
//
//   windAngle (same sign as spoolAngle):
//     tangent wins: windAngle = spoolAngle - sign·arccos(rs/d)   [= 0 at transition;
//       arccos(rs/d) reads as 0 once d<=rs, so this is one continuous formula]
//     anchor wins:  windAngle = 0
//   woundLength = |windAngle| · rs
//   paidLength  = Lfree = sqrt(max(0, d²-rs²))  when tangent wins (0 once d<=rs);
//                 |T - Q|  when anchor wins (Q=A, an ordinary rod to T)
//   totalUsed   = woundLength + paidLength
//
//   Jacobian constrains d/dt(totalUsed) = 0, selected by tangentWins alone --
//   NOT also by d<=rs. A many-turn wind can overshoot to d<=rs for a step near
//   the ell->0 singularity while still genuinely in the tangent regime (large
//   |spoolAngle|); Lfree is already 0 there, so the tangent-mode row stays
//   the right (and continuous) one. Only the anchor-wins case is a real rod.
//   Tangent mode: Jx=(Dx·Lfree - rs·sign·Dy)/d², Jy=(Dy·Lfree + rs·sign·Dx)/d²;
//     spool [-Jx, -Jy, -rs·sign]; tether [Jx, Jy, moment arm].
//   Direct/rod mode (Q=A): spool [-ux,-uy, ux·ry_Q-uy·rx_Q]; tether [ux,uy,arm].
//   Returns null only when the spool body is missing.
function cableFrame(cb, spoolAngleRef){
  const S=bodies[bodyIndex(cb.spool.id)]; if(!S) return null;
  const rs=S.r;
  const is=bodyIndex(cb.spool.id);
  let T, tb=null, trx=0, tryy=0, ti=-1;
  if(cb.tether.id!=null){ ti=bodyIndex(cb.tether.id); tb=bodies[ti];
    const [tx,ty,rx,ry]=worldPt(tb,epLocal(tb,cb.tether.off)); T=[tx,ty]; trx=rx; tryy=ry; }
  else { T=[cb.tether.off[0],cb.tether.off[1]]; }

  // Spool anchor A: material point on rim.
  const localAngle   = cb.localAngle !== undefined ? cb.localAngle : 0;
  const anchorAngle  = S.th + localAngle;
  const rx_anchor    = rs*Math.cos(anchorAngle), ry_anchor = rs*Math.sin(anchorAngle);
  const Ax = S.x + rx_anchor, Ay = S.y + ry_anchor;

  // B->C vector and tether world angle.
  const Dx = T[0]-S.x, Dy = T[1]-S.y;
  const d  = Math.hypot(Dx, Dy);
  const tetherAngle = Math.atan2(Dy, Dx);

  // spoolAngle: ABC, unwrapped around previous reference for continuity.
  const spoolAngleRaw = tetherAngle - anchorAngle;
  let spoolAngle;
  if(spoolAngleRef != null){
    let da = spoolAngleRaw - spoolAngleRef;
    while(da >  Math.PI) da -= Math.PI*2;
    while(da < -Math.PI) da += Math.PI*2;
    spoolAngle = spoolAngleRef + da;
  } else {
    const seed = cb.spoolAngle !== undefined ? cb.spoolAngle : 0;
    let da = spoolAngleRaw - seed;
    while(da >  Math.PI) da -= Math.PI*2;
    while(da < -Math.PI) da += Math.PI*2;
    spoolAngle = seed + da;
  }
  const sign = spoolAngle >= 0 ? 1 : -1;

  // Tangent point D and DBC angle. Lfree is defined for any d (0 once d<=rs)
  // so it stays the source of truth for the free length even through a
  // step that momentarily overshoots the rim -- see the tangentWins branch
  // below.
  const tetherInside = d <= rs;
  const beta  = tetherInside ? 0 : Math.acos(Math.max(-1, Math.min(1, rs/d)));
  const Lfree = Math.sqrt(Math.max(0, d*d - rs*rs));
  const tangentAngle = tetherInside ? (d > 1e-9 ? tetherAngle : anchorAngle) : (tetherAngle - sign*beta);
  const dbc = beta;                              // |DBC| = arccos(rs/d), 0 when inside
  const rx_tan = rs*Math.cos(tangentAngle), ry_tan = rs*Math.sin(tangentAngle);
  const Qtan_x = S.x + rx_tan, Qtan_y = S.y + ry_tan;

  // Separation point Q. tangentWins governs both Q's choice and (below) which
  // Jacobian form applies -- it must NOT also fork on tetherInside: a body
  // deep in a many-turn wind can overshoot to d<=rs for a step near the
  // ell->0 singularity while still genuinely in the tangent regime (large
  // |spoolAngle|), and Lfree already degrades continuously to 0 there. Only
  // gating on tetherInside forced a jump to the anchor-rod formula against
  // the wrong point (Qtan, not A) for that step -- a large, energy-adding
  // direction discontinuity, not the harmless near-Delta=0 interior tether the
  // rod formula is actually for (design note §C.6).
  const tangentWins = dbc < Math.abs(spoolAngle) - 1e-10;
  let Qx, Qy, rx_Q, ry_Q, windAngle, paidLength;
  if(tangentWins){
    Qx = Qtan_x; Qy = Qtan_y; rx_Q = rx_tan; ry_Q = ry_tan;
    windAngle  = spoolAngle - sign*beta;
    paidLength = Lfree;
  } else {
    Qx = Ax; Qy = Ay; rx_Q = rx_anchor; ry_Q = ry_anchor;
    windAngle  = 0;
    paidLength = Math.hypot(T[0]-Qx, T[1]-Qy);
  }
  const woundLength = Math.abs(windAngle) * rs;
  const totalUsed   = woundLength + paidLength;
  const Lallow      = cb.Ltot - woundLength;

  const ux = paidLength > 1e-9 ? (T[0]-Qx)/paidLength : 0;
  const uy = paidLength > 1e-9 ? (T[1]-Qy)/paidLength : 1;

  // Jacobian for d/dt(totalUsed) = 0. The -rs·sign angular term on the spool row
  // was re-derived directly from d(totalUsed)/dt (chain rule through spoolAngle,
  // beta, Lfree) and matches: it is the same physical row as the pre-rebuild
  // +rs·side, given that convention's side = -sign(spoolAngle) (see the wrap/side
  // reconstruction in physics.js's cable migration).
  // The tether end's columns go through epFrame (§06.1) rather than being written
  // out here, so a tether anchored on a vessel picks up its length column the same
  // way every other endpoint does. The spool row stays hand-built: it is a rim
  // tangent term, not a point-velocity projection, and a spool is always a disk.
  const epT = tb ? epFrame(cb.tether) : null;
  let cols;
  if(tangentWins){
    // d2 floors away from 0 for a step that overshoots deep past the rim
    // (large angular rate near ell->0, see cableFrame's header) -- Lfree is
    // already 0 there, so the row is still the correct tangential direction,
    // just guarded against an actual division by a near-zero d.
    const d2 = Math.max(d*d, 1e-9);
    const Jx = (Dx*Lfree - rs*sign*Dy) / d2;
    const Jy = (Dy*Lfree + rs*sign*Dx) / d2;
    cols = mergeCols([ S.static?[]:[[is, -Jx, -Jy, -rs*sign]],
                       (tb&&!tb.static)?epT.velCols(Jx,Jy):[] ]);
  } else {
    cols = mergeCols([ S.static?[]:[[is, -ux, -uy, ux*ry_Q - uy*rx_Q]],
                       (tb&&!tb.static)?epT.velCols(ux,uy):[] ]);
  }

  return {
    S, rs, T, Qx, Qy, ux, uy, cols,
    spoolAngle, windAngle, woundLength, paidLength, totalUsed, Lallow,
    Lfree, Ax, Ay, rx_anchor, ry_anchor, anchorAngle,
    tangentAngle, rx_tan, ry_tan, Qtan_x, Qtan_y, tangentWins,
    rx_Q, ry_Q, localAngle, tb, ti, trx, tryy, is, tetherInside
  };
}

// Current geometric cable path length for this configuration.
function cableCurrentLength(cb, f){
  const cf = f || cableFrame(cb); if(!cf) return 0;
  return cf.totalUsed;
}

// ---- §06.5 · rowsFor (constraint -> rows dispatch) ----
// One branch per con.type; to reach a specific joint's row math, search its tag,
// e.g.  type==='line'. Catalog (rows) -- cross-references spec §4:
//   vertex         2 per joined incidence past the primary (a shared point), plus
//                      1 per welded incidence past the first (a shared frame angle)
//   rod            1   distance held along the connecting line; +1 per welded
//                      end (locks that end's body -- or the fixed world frame,
//                      for a background end -- to the rod's own direction).
//                      A `posable` rod RELEASED for a pose drag (§06.2d) drops
//                      all of that and keeps only its line: 0 rows for the pair,
//                      and one rider row per extra point, as a slot's.
//   slot           0   two pins = purely visual; +1 per "prismatic" end
//                      (locks that end to the segment direction, as rod's
//                      weld does); +1 more once BOTH ends are prismatic
//                      (kills lateral drift off the rail -- the classic
//                      point-on-line lock, giving a rigid prismatic joint)
//   belt           1   fixed phase ratio of two rim angles (holonomic)
//   knife          1   no-side-slip contact (NONHOLONOMIC, nh:true)
//   cvt            1   tangential match at a variable-radius contact (NONHOLONOMIC)
//   rack           0   a rack line named by two pins; +1 per welded pin (as rod's
//                      weld); +1 per meshing pinion (tangential match at the
//                      pinion's live pitch radius, NONHOLONOMIC)
// EVERY ROW CARRIES A ROLE -- dist, weld, online, station, lateral, mesh, pin, belt,
// cvt, knife -- and, where a kind has more than one row of a role, an `at` naming
// which end ('A'/'B') or which control point (its index in con.pts). physics.js
// §08.3 records them beside the multipliers, and the reaction readout (§09.3) looks
// one up by name. It used to count instead: every branch of §09.3 re-derived this
// section's row ORDER from the joint's own flags, which is the same arithmetic
// written out four times, in another file, that nothing checked against the rows it
// described -- so adding a row to a kind silently moved every readout after it.
// A new row needs a role; tools/frame-check.js fails if you skip this.
// (Cable rows are built inline in §08.2, not here, because they are unilateral.)
// One endpoint frame's velocity columns along a direction, negated -- the station row
// measures a joint's motion along the bar RELATIVE to the origin's, and every other
// row of this shape gets its negation from lineFrameOf's minusPoint* closures.
const O_NEG = (ep, dx, dy) => scaleCols(ep.velCols(dx,dy), -1);
function rowsFor(con){
  // Each row carries the raw position error C (the value to drive to zero). The
  // velocity solver scales it by beta/h (Baumgarte); the position projection uses
  // C directly. Same Jacobian rows serve both.
  if(con.type==='dragpin'){
    // Internal-only: pins a point on A to a fixed world point. Never a user
    // constraint -- §13.6 feeds one through projectPositions as a transient
    // goal while the player drags a body around in edit mode. Marked `soft`
    // so projectPositions gives it extra compliance (§09.1): it's a UI pull,
    // not a real joint, and in any DOF a real constraint also has a say over
    // (dragging a rod's free end off the circle its weld allows, a slider off
    // its rail), the real joint should win outright rather than splitting the
    // difference and showing up as a violation on a mechanism that's actually
    // fully satisfied within its own reachable directions.
    const A = bodies[bodyIndex(con.a.id)];
    const ep = epFrame(con.a);
    return [
      { cols:ep.velCols(1,0), C: ep.wx-con.world[0], soft:true, role:'drag' },
      { cols:ep.velCols(0,1), C: ep.wy-con.world[1], soft:true, role:'drag' }
    ];
  }
  if(con.type==='line'){
    // The row table of VERTEX.md §X.4, and every row in it is a statement about one
    // joint relative to the bar:
    //
    //   P, Q -- the two that PLACE the line   none: they are the line
    //   any other joint                       1 on-line row
    //   each non-sliding joint except O       1 station row, its distance from O
    //   each meshing disk                     1 rolling row (nonholonomic)
    //   the line's own weld at a joint        1 angle row, where a body is welded too
    //
    // P's and Q's on-line rows would be tautologies -- the line is drawn THROUGH
    // them -- which is the whole difference between a rod (both ends held, one
    // distance row) and a two-pin slot (0 rows, a drawn guide), with no branch
    // between the two.
    const f=lineFrame(con);
    if(!f) return [];
    const LF=lineFrameOf(f);
    const released=lineReleased(con);
    const rows=[];
    const locOf = K => LF.localOf(K.ep.wx, K.ep.wy);
    for(const K of f.J){
      if(K===f.P || K===f.Q) continue;
      const [du,dn]=locOf(K);
      rows.push({ cols: mergeCols([ K.ep.velCols(f.nx,f.ny), ...LF.minusPointAcross(du,dn) ]),
                  C: dn, role:'online', at:K.v.label });
    }
    // Stations are measured from the ORIGIN, not from the placement pair: an origin
    // has to be material, and P is only whichever joint happens to be furthest out.
    // With a compliance the station rows are gone -- the distance is carried by a
    // force instead (physics.js §08.1) -- and a released posable line holds nothing.
    if(f.O && !released && !(con.soft>0)){
      const [duO,dnO]=locOf(f.O);
      for(const K of f.J){
        if(K===f.O || K.e.slide) continue;
        const [du,dn]=locOf(K);
        rows.push({ cols: mergeCols([ K.ep.velCols(f.ux,f.uy), O_NEG(f.O.ep, f.ux, f.uy),
                                      scaleCols(LF.angCols(), dn-dnO) ]),
                    C: (du-duO) - (K.e.s||0), role:'station', at:K.v.label });
      }
    }
    // A welded joint ties the LINE's frame angle to whatever else is welded at that
    // vertex (§06.2e). The row is built here rather than on the vertex because the
    // line has already worked its frame out; the reference is the same one the
    // vertex's own weld rows use, so a body, another body and the line at one vertex
    // cost two rows between them and not three.
    if(!released) for(const K of f.J){
      if(!K.e.weld) continue;
      const W=vertexWeldRef(K.v); if(!W) continue;
      const B=epFrame(W);
      rows.push({ cols: mergeCols([ scaleCols(LF.angCols(),-1), B.angCols() ]),
                  C: (B.th-(W.restAng||0)) - (f.phi-(K.e.restAng||0)),
                  role:'weld', at:K.v.label });
    }
    // A meshing disk rolls on the line with perfect traction wherever it sits: the
    // two materials in contact at the foot of the perpendicular have the same speed
    // ALONG the bar. The rack's row, unchanged (§06.5), with the line's placement
    // origin P in the rack's end-a role -- correct because the bar is rigid, so its
    // material speed along u is the same at every point of it.
    for(const id of (con.mesh||[])){
      const ib=bodyIndex(id); const B=bodies[ib]; if(!B) continue;
      const rho=(B.x-f.wax)*f.nx + (B.y-f.way)*f.ny;
      rows.push({ cols: mergeCols([ f.epA.velCols(f.ux,f.uy), [[ib, -f.ux, -f.uy, -rho]] ]),
                  C:0, nh:true, role:'mesh', at:id });
    }
    return rows;
  }
  if(con.type==='vertex'){
    // A vertex is a coincidence with a name (§06.2e). Two rules, and they are the
    // whole of what pin, and every weld the line joints will carry, are:
    //
    //   every JOINED incidence but the primary   2 rows -- its point is the primary's
    //   every WELDED incidence but the first     1 row -- its angle is that one's
    //
    // Both are stated against ONE reference rather than pairwise, which is what
    // makes m incidences cost 2(m-1) rows and not m(m-1): coincidence is transitive,
    // so holding each to the first holds all of them to each other.
    const rows=[];
    const ons=vertexOns(con);
    const P=vertexPrimary(con);
    if(!P) return rows;                       // joined to nothing: a bare marker
    const F=epFrame(P);
    ons.forEach((e,at)=>{
      // A LINE incidence is not a coincidence with a point -- it says the vertex lies
      // ON the bar, which is one row or two depending on whether it slides, and the
      // LINE builds it (§06.2f) because only the line knows which of its joints place
      // it and which are held.
      if(!e.join || e===P || isLineOn(e)) return;
      const K=epFrame(e);
      rows.push({ cols: mergeCols([K.velCols(1,0), F.velCols(-1,0)]), C:K.wx-F.wx, role:'pin', at });
      rows.push({ cols: mergeCols([K.velCols(0,1), F.velCols(0,-1)]), C:K.wy-F.wy, role:'pin', at });
    });
    // The angle rows measure (th - restAng) against the reference's own
    // (th - restAng): each rest angle is that body's absolute angle at the moment
    // its weld went on (§06.2e), so neither side is privileged and no capture
    // depends on any other.
    const W=vertexWeldRef(con);
    if(W){
      const R0=epFrame(W), c0=(R0.th-(W.restAng||0));
      ons.forEach((e,at)=>{
        if(!e.join || !e.weld || e===W || isLineOn(e)) return;   // a line's weld: §06.2f
        const K=epFrame(e);
        rows.push({ cols: mergeCols([K.angCols(), scaleCols(R0.angCols(),-1)]),
                    C: (K.th-(e.restAng||0)) - c0, role:'weld', at });
      });
    }
    return rows;
  }
  if(con.type==='belt'){
    // inextensible belt: rim tangential speeds equal -> fixed phase ratio (holonomic).
    // sense +1 open belt (same sense), -1 crossed.
    const A=bodies[bodyIndex(con.a.id)], B=bodies[bodyIndex(con.b.id)];
    const ia=bodyIndex(con.a.id), ib=bodyIndex(con.b.id), s=con.sense;
    const C=(con.rA*A.th - s*con.rB*B.th) - con.restPhase;
    return [{ cols:[[ia,0,0,con.rA],[ib,0,0,-s*con.rB]], C, role:'belt' }];
  }
  if(con.type==='knife'){
    // no-side-slip (Chaplygin knife edge): the contact point's velocity across the
    // heading is zero. Velocity-only -- no position invariant (nonholonomic).
    const A=bodies[bodyIndex(con.a.id)];
    const hh=R(A.th,con.dir[0],con.dir[1]); const hl=Math.hypot(hh[0],hh[1])||1;
    const nx=-hh[1]/hl, ny=hh[0]/hl;                 // lateral normal to heading
    return [{ cols:epFrame(con.a).velCols(nx,ny), C:0, nh:true, role:'knife' }];
  }
  if(con.type==='cvt'){
    // rolling contact at P = the point on A's rim nearest B. Match the two bodies'
    // tangential material-point velocities there. r_A is A's radius; B's arm is the
    // distance from B's centre to P, namely (d - r_A) -- a coordinate -> nonholonomic.
    const A=bodies[bodyIndex(con.a.id)], B=bodies[bodyIndex(con.b.id)];
    const ia=bodyIndex(con.a.id), ib=bodyIndex(con.b.id);
    let rvx=B.x-A.x, rvy=B.y-A.y; const d=Math.hypot(rvx,rvy)||1e-6;
    const ux=rvx/d, uy=rvy/d; const tx=-uy, ty=ux;   // tangent at contact
    const rA=A.r, armB=d-rA;
    return [{ cols:[[ia, tx, ty, rA],[ib, -tx, -ty, armB]], C:0, nh:true, role:'cvt' }];
  }
  return [];
}

// Build a rotational (torsional) spring between two bodies -- 'a' and 'b' are
// bare {id} refs (no offset: like belt/cvt, the whole body's frame angle is
// the feature, not a point on it). Either may be background-anchored
// (id===null), which reads as a fixed theta=0 reference -- mirroring the
// null-id convention rod/slot use for a world-anchored end. The rest
// angle captures whatever the live relative angle is at creation, so a
// freshly-placed rotational spring starts unstressed (mirrors rod weld's
// captureRestAngle, §06.1).
const ROTSPRING_DEFAULT_K = 8;
function rotSpringRelAngle(rs){
  const thA = rs.a.id!=null ? bodies[bodyIndex(rs.a.id)].th : 0;
  const thB = rs.b.id!=null ? bodies[bodyIndex(rs.b.id)].th : 0;
  return thA-thB;
}
function makeRotSpringCon(aId,bId){
  const rs = { type:'rotspring', a:{id:aId}, b:{id:bId}, restAngle:0, k:ROTSPRING_DEFAULT_K, sel:false };
  rs.restAngle = rotSpringRelAngle(rs);
  return rs;
}
// The two theta=0 reference marks drawn on-canvas (render.js §11.5): with two
// real bodies, each mark rides its own body's local theta=0 point on its rim.
// With one end on the background, both marks sit on the *same* (real) body's
// rim -- one riding the body's own theta=0 (spins with it), the other held at
// the fixed world +x direction from that body's centre (the "ground's
// theta=0") -- so the pair visibly splays apart as the body twists away from
// its rest angle relative to the fixed frame.
function rotSpringControlPoints(con){
  const hasA=con.a.id!=null, hasB=con.b.id!=null;
  if(hasA && hasB){
    const A=bodies[bodyIndex(con.a.id)], B=bodies[bodyIndex(con.b.id)];
    const [ax,ay]=worldPt(A,bodyRimLocal(A)), [bx,by]=worldPt(B,bodyRimLocal(B));
    return {pA:[ax,ay], pB:[bx,by]};
  }
  const body = hasA ? bodies[bodyIndex(con.a.id)] : bodies[bodyIndex(con.b.id)];
  const [ox,oy]=worldPt(body,bodyRimLocal(body));
  const groundPt=[body.x+bodyExtentR(body), body.y];
  return hasA ? {pA:[ox,oy], pB:groundPt} : {pA:groundPt, pB:[ox,oy]};
}
// Belt vs. spiral: a belt reads as a connection between two separate rims, so
// it only makes sense while the two bodies' disks are not one fully inside
// the other (the standard "circle A contains circle B" test, distance between
// centres plus the smaller radius at most the larger radius). Full overlap,
// or either end on the background (no second rim to run a belt to at all),
// falls back to the spiral instead.
function rotSpringVisualMode(con){
  if(con.a.id==null || con.b.id==null) return 'spiral';
  const A=bodies[bodyIndex(con.a.id)], B=bodies[bodyIndex(con.b.id)];
  // The belt rendering is two tangent lines between two round rims -- only
  // meaningful when both ends are actually circles; a rectangle (or either
  // body missing) falls back to the spiral, same as the background-anchored
  // case above.
  if(A.shape==='rect' || B.shape==='rect') return 'spiral';
  const d=Math.hypot(A.x-B.x,A.y-B.y);
  const rMax=Math.max(A.r,B.r), rMin=Math.min(A.r,B.r);
  return (d+rMin<=rMax+1e-9) ? 'spiral' : 'belt';
}
// Geometry for the spiral render: centred on the larger body (or the sole
// real body, for a background-attached spring), sweeping from its own
// perimeter down to either the smaller body's perimeter or (background case)
// the centre. The sweep angle is a fixed decorative two turns plus the live
// deviation from rest, so a spring visibly winds up or loosens as the bodies
// twist relative to each other -- capped well short of the many-turn range so
// a fast spin doesn't wind the drawing into an unreadable knot.
function rotSpringSpiralGeom(con){
  const hasA=con.a.id!=null, hasB=con.b.id!=null;
  let outer, outerR, innerR;
  if(hasA && hasB){
    const A=bodies[bodyIndex(con.a.id)], B=bodies[bodyIndex(con.b.id)];
    const eA=bodyExtentR(A), eB=bodyExtentR(B);
    const outerIsA = eA>=eB;
    outer = outerIsA?A:B; outerR=outerIsA?eA:eB; innerR=outerIsA?eB:eA;
  } else {
    outer = hasA ? bodies[bodyIndex(con.a.id)] : bodies[bodyIndex(con.b.id)];
    outerR = bodyExtentR(outer); innerR = 0;
  }
  const dev = rotSpringRelAngle(con) - con.restAngle;
  const base = Math.PI*4;
  let sweep = base+dev;
  const mag = Math.max(Math.PI*1, Math.min(Math.PI*12, Math.abs(sweep)));
  sweep = mag*(sweep<0?-1:1);
  return {cx:outer.x, cy:outer.y, outerR, innerR, angle0:outer.th, sweep};
}
