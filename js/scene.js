// ============================================================================
//  §17 · SCENE FILE
//  A human-readable text form of the whole bench: every body with its state,
//  every constraint, force element and interaction, the ambient and the camera.
//  Reading and writing it are two halves of one table (§17.1), which is also the
//  written-down answer to "what can a scene contain" -- see SCENE.md.
//
//  The reader builds ONLY by calling the constructors the tool dispatch calls
//  (§05.2/§06.1/§06.2) and rejects any key the table does not list, so a scene
//  file cannot describe something the editor cannot build. That is the point of
//  the format, not a side effect of it (SCENE.md §S.2).
//
//    §17.1  the ledger      (SCENE_SCHEMA -- one row per scene-object kind)
//    §17.2  values          (formatting, tokenizing, endpoint syntax)
//    §17.3  exportScene     (world -> text)
//    §17.4  importScene     (text -> world)
//    §17.6  state snapshot  (snapshotState/applyState -- what Reset restores)
//    §17.5  panel UI        (the scene-file card, and file drag-drop)
// ============================================================================

// Version 2 dropped `static` and `lenlock`. They were fields a file could set on a
// body directly, which made them exactly the thing this format exists to prevent: a
// coordinate frozen by assertion rather than by an object in the scene. A pinned
// body is now one with a double-welded rod to the ground, and a reservoir is a
// vessel with a strut inside it -- both ordinary constraints, both visible, both
// deletable (constraints.js §06.2b).
const SCENE_VERSION = 2;

// ---- §17.1 · the ledger ----
// One row per kind of thing a scene can contain. Each row names:
//
//   list    the world array it lives in (§04.2)
//   match   which objects of that array are this kind
//   id      whether the line carries the object's id -- true only for bodies,
//           the only things anything else refers to
//   ends    positional endpoint slots, written  A -- B  ahead of the keyed fields
//   fields  the serialized fields, IN APPLICATION ORDER (see below)
//   build   the constructor call -- the one place this kind is created from a file
//   finish  ordered post-build work the per-field setters cannot express
//
// A field is one of three kinds, which is the whole of SCENE.md §S.3:
//
//   * DERIVED fields are simply absent from the table. mass and inertia on a
//     vessel, invM/invI/mu, the adiabat invariant, every solver scratch field, and
//     -- since version 2 -- `static` and `lenLock`, which are read off the
//     constraints present rather than set (constraints.js §06.2b) -- all recomputed
//     on load. Writing them out would let a file disagree with
//     itself and force the reader to pick a winner.
//   * AUTHORED fields carry a `def` and are written only when they differ from
//     it, so a default body is a short line. `def` may be a function of the
//     other fields -- `q(name)` reads one -- which is how "mass defaults to the
//     density-1 value for this radius" is stated once and read by both directions.
//   * CAPTURED fields carry `always:true` and are written every time. These are
//     read off the geometry at creation and never recomputed since (a rod's rest
//     length, a weld's rest angle, a belt's phase, a cable's paid-out length), so
//     the pose in the file does not imply them. Silently recapturing them on load
//     would be the format quietly editing the physics.
//
// `fields` is applied in declaration order on import -- a vessel's shell mass and
// gamma land before its gas is set, because the gas depends on both.
const F_POSE = () => ({
  th: {t:'num', def:0, get:b=>b.th, set:(b,v)=>{b.th=v;}},
  vx: {t:'num', def:0, get:b=>b.vx, set:(b,v)=>{b.vx=v;}},
  vy: {t:'num', def:0, get:b=>b.vy, set:(b,v)=>{b.vy=v;}},
  w:  {t:'num', def:0, get:b=>b.w,  set:(b,v)=>{b.w=v;}},
});

// A row's `state` list is the second thing the ledger defines: the fields a RUN can
// change, which is exactly what Reset has to put back (§16.1). It is a separate list
// from `fields` because the two answer different questions -- a body's radius is in
// the file and not in the snapshot; a vessel's adiabat invariant is in the snapshot
// and not in the file -- but they live here together so that adding a coordinate is
// one edit in one place. Getting that wrong is not hypothetical: Reset silently
// changed a scene's energy twice, once when the vessel's fourth coordinate and gas
// arrived and again when the bath total did, each time because the snapshot was a
// second list maintained somewhere else.
const S_POSE = () => [
  ['x',  b=>b.x,  (b,v)=>{b.x=v;}],   ['y',  b=>b.y,  (b,v)=>{b.y=v;}],
  ['th', b=>b.th, (b,v)=>{b.th=v;}],  ['vx', b=>b.vx, (b,v)=>{b.vx=v;}],
  ['vy', b=>b.vy, (b,v)=>{b.vy=v;}],  ['w',  b=>b.w,  (b,v)=>{b.w=v;}],
];

const SCENE_SCHEMA = [
  { kind:'body', list:'bodies', id:true, match:b=>b.shape==='circle',
    fields:{
      x:{t:'num', always:true, get:b=>b.x},
      y:{t:'num', always:true, get:b=>b.y},
      r:{t:'num', always:true, get:b=>b.r},
      // density 1, the disk factory's own convention (§05.2)
      mass:{t:'num', def:q=>Math.PI*q('r')*q('r'), get:b=>b.mass, set:(b,v)=>setBodyMass(b,v)},
      ...F_POSE(),
    },
    build:q=>makeBody(q('x'), q('y'), q('r')),
    finish:o=>refreshInertia(o),
    state:S_POSE() },

  { kind:'rect', list:'bodies', id:true, match:b=>b.shape==='rect',
    fields:{
      x:{t:'num', always:true, get:b=>b.x},
      y:{t:'num', always:true, get:b=>b.y},
      // full width/height, as the inspector shows them -- `w` is taken, it is the
      // angular velocity every body carries
      width:{t:'num', always:true, get:b=>b.hw*2},
      height:{t:'num', always:true, get:b=>b.hh*2},
      mass:{t:'num', def:q=>q('width')*q('height'), get:b=>b.mass, set:(b,v)=>setBodyMass(b,v)},
      ...F_POSE(),
    },
    build:q=>makeRectBody(q('x'), q('y'), q('width')/2, q('height')/2),
    finish:o=>refreshInertia(o),
    state:S_POSE() },

  { kind:'vessel', list:'bodies', id:true, match:b=>b.shape==='vessel',
    fields:{
      x:{t:'num', always:true, get:b=>b.x},
      y:{t:'num', always:true, get:b=>b.y},
      bore:{t:'num', always:true, get:b=>b.bore},
      // the fourth configuration coordinate (VESSEL.md §V.1), not a shape parameter
      len:{t:'num', always:true, get:b=>b.len},
      mShell:{t:'num', def:q=>VESSEL_DENSITY*q('bore')*q('len')*VESSEL_DEPTH,
              get:b=>b.mShell, set:(b,v)=>{b.mShell=v;}},
      gamma:{t:'num', def:GAS_AIR.gamma, get:b=>b.gas.gamma, set:(b,v)=>{b.gas.gamma=v;}},
      // The gas is written as pressure and temperature -- the two faces of it a
      // person reasons about, and a complete encoding at a known volume: mass and
      // the adiabat invariant both follow (§05.2d setVesselGasPT). Applied together
      // in finish(), after gamma, since kap depends on it. Both are `always` here
      // because that is what finish() below actually consumes -- but a line need
      // not literally write them: resolveVesselFill (§17.4) runs ahead of this
      // table and fills in whichever of len/P/T a line left for it to compute from
      // the other two of {P, T, gasMass}, so by the time `always` is checked they
      // are already there.
      P:{t:'num', always:true, get:b=>gasP(b), set:null},
      T:{t:'num', always:true, get:b=>gasT(b), set:null},
      // A third way to name the gas's fill, alongside P and T -- consumed straight
      // off the raw parsed fields by resolveVesselFill, so it needs no real get/set
      // of its own here; the entry only exists so readKeyed (§17.4) recognizes the
      // key. get/def are fixed and equal so emitFields never writes it back out: P
      // and T alone are already this format's complete, canonical encoding of the
      // gas (SCENE.md S.3) -- gasMass is something a person may write, never
      // something the tool emits.
      gasMass:{t:'num', def:0, get:()=>0, set:null},
      ...F_POSE(),
      vlen:{t:'num', def:0, get:b=>b.vlen, set:(b,v)=>{b.vlen=v;}},
    },
    build:q=>makeVessel(q('x'), q('y'), q('bore'), q('len')),
    finish:(o,f,q)=>{ setVesselGasPT(o, q('P'), q('T')); refreshVessel(o); },
    // The gas is snapshotted in the form the vessel STORES -- the adiabat invariant
    // and the mass -- not in the pressure and temperature the file writes. Those are
    // a change of variables away from it (§05.2d), and Reset runs on every press of
    // R: routing it through the readable view would drift the gas by a rounding
    // error every time, for nothing.
    state:[ ...S_POSE(),
            ['len',  v=>v.len,  (v,x)=>{v.len=x;}],
            ['vlen', v=>v.vlen, (v,x)=>{v.vlen=x;}],
            ['gas',  v=>[v.gas.kap, v.gas.mass], (v,a)=>{v.gas.kap=a[0]; v.gas.mass=a[1];}] ],
    restore:v=>{ v._vlen0=v.vlen; refreshVessel(v); } },

  { kind:'pin', list:'constraints', match:c=>c.type==='pin',
    ends:[['a','ep-body'], ['b','ep-body']],
    fields:{},
    build:(q,e)=>makePinCon(e.a, e.b) },

  { kind:'rod', list:'constraints', match:c=>c.type==='rod',
    ends:[['a','ep'], ['b','ep']],
    fields:{
      len:{t:'num', always:true, get:c=>c.len, set:(c,v)=>{c.len=v;}},
      weld:{t:'ends', def:'none', get:c=>endsWord(c.weldA,c.weldB)},
      restAngA:{t:'num', always:true, when:c=>c.weldA, get:c=>c.restAngA, set:(c,v)=>{c.restAngA=v;}},
      restAngB:{t:'num', always:true, when:c=>c.weldB, get:c=>c.restAngB, set:(c,v)=>{c.restAngB=v;}},
    },
    build:(q,e)=>{ const [A,B]=endsFlags(q('weld')); return makeRodCon(e.a, e.b, A, B); } },

  { kind:'slot', list:'constraints', match:c=>c.type==='slot',
    ends:[['a','ep'], ['b','ep']],
    fields:{
      lock:{t:'ends', def:'none', get:c=>endsWord(c.prismaticA,c.prismaticB)},
      restAngA:{t:'num', always:true, when:c=>c.prismaticA, get:c=>c.restAngA, set:(c,v)=>{c.restAngA=v;}},
      restAngB:{t:'num', always:true, when:c=>c.prismaticB, get:c=>c.restAngB, set:(c,v)=>{c.restAngB=v;}},
    },
    build:(q,e)=>{ const [A,B]=endsFlags(q('lock')); return makeSlotCon(e.a, e.b, A, B); } },

  { kind:'belt', list:'constraints', match:c=>c.type==='belt',
    ends:[['a','id'], ['b','id']],
    fields:{
      rA:{t:'num', always:true, get:c=>c.rA, set:(c,v)=>{c.rA=v;}},
      rB:{t:'num', always:true, get:c=>c.rB, set:(c,v)=>{c.rB=v;}},
      crossed:{t:'flag', def:false, get:c=>c.sense<0},
      restPhase:{t:'num', always:true, get:c=>c.restPhase, set:(c,v)=>{c.restPhase=v;}},
    },
    build:(q,e)=>makeBeltCon(e.a.id, e.b.id, q('crossed')?-1:1) },

  { kind:'cvt', list:'constraints', match:c=>c.type==='cvt',
    ends:[['a','id'], ['b','id']],
    fields:{},
    build:(q,e)=>makeCvtCon(e.a.id, e.b.id) },

  // 'a' is an ordinary endpoint: the rack is welded to that body, so its anchor
  // rides the body frame like every other {id, off} in the format. `angle` is the
  // rack's heading in that body's OWN frame -- AUTHORED, not captured: the
  // constructor's 0 (along the body's local +x) is a real default rather than
  // something read off the pose at creation, so a line that omits it means that.
  { kind:'rack', list:'constraints', match:c=>c.type==='rack',
    ends:[['a','ep'], ['b','id']],
    fields:{
      angle:{t:'num', def:0, get:c=>c.angle, set:(c,v)=>{c.angle=v;}},
    },
    build:(q,e)=>makeRackCon(e.a.id, e.a.off, e.b.id) },

  { kind:'knife', list:'constraints', match:c=>c.type==='knife',
    ends:[['a','ep-body']],
    fields:{ dir:{t:'vec2', always:true, get:c=>c.dir} },
    build:(q,e)=>makeKnifeCon(e.a, q('dir')) },

  { kind:'cable', list:'cables', match:()=>true,
    ends:[['tether','ep'], ['spool','id']],
    fields:{
      Ltot:{t:'num', always:true, get:c=>c.Ltot, set:(c,v)=>{c.Ltot=v;}},
      localAngle:{t:'num', always:true, get:c=>c.localAngle, set:(c,v)=>{c.localAngle=v;}},
      spoolAngle:{t:'num', def:0, get:c=>c.spoolAngle, set:(c,v)=>{c.spoolAngle=v;}},
    },
    build:(q,e)=>makeCableCon(e.tether, e.spool.id),
    state:[ ['spoolAngle', c=>c.spoolAngle, (c,x)=>{c.spoolAngle=x;}] ] },

  { kind:'spring', list:'springs', match:()=>true,
    ends:[['a','ep'], ['b','ep']],
    fields:{
      restLen:{t:'num', always:true, get:s=>s.restLen, set:(s,v)=>{s.restLen=v;}},
      k:{t:'num', def:SPRING_DEFAULT_K, get:s=>s.k, set:(s,v)=>{s.k=v;}},
    },
    build:(q,e)=>makeSpringCon(e.a, e.b) },

  { kind:'rotspring', list:'rotSprings', match:()=>true,
    ends:[['a','id-bg'], ['b','id']],
    fields:{
      restAngle:{t:'num', always:true, get:s=>s.restAngle, set:(s,v)=>{s.restAngle=v;}},
      k:{t:'num', def:ROTSPRING_DEFAULT_K, get:s=>s.k, set:(s,v)=>{s.k=v;}},
    },
    build:(q,e)=>makeRotSpringCon(e.a.id, e.b.id) },

  // Heat and mass interactions name their two sides by key rather than
  // positionally: they are not symmetric (one side is always the mediating body,
  // the other the vessel or the background) and saying so in the file is worth
  // four characters. See VESSEL.md §V.10.
  //
  // `body` is `ep-centre`, not a plain `ref`: "every constraint, spring and cable
  // endpoint is an {id, off} pair" (geometry.js §05.2c) and an interaction's
  // mediating side is no exception -- it just has nowhere yet to put a nonzero
  // one, so it round-trips as ordinary body-frame coordinates at (0,0), same as a
  // pin or rod anchored on a body's own centre (fmtEp omits the redundant
  // "@(0,0)"; `id@(0,0)` on import is accepted as identical to a bare id). See
  // resolveVesselFill above for why `vessel` stays a plain ref-bg: the background
  // side names no specific point, only "the ambient," so it has no anchor to
  // spell out in the first place.
  { kind:'heat', list:'interactions', match:i=>i.type==='heat',
    fields:{
      body:{t:'ep-centre', always:true, get:i=>i.body},
      vessel:{t:'ref-bg', always:true, get:i=>i.vessel.id},
      k:{t:'num', def:1000, get:i=>i.k, set:(i,v)=>{i.k=v;}},
    },
    build:q=>makeInteraction('heat', q('body').id, q('vessel')) },

  { kind:'flow', list:'interactions', match:i=>i.type==='flow',
    fields:{
      body:{t:'ep-centre', always:true, get:i=>i.body},
      vessel:{t:'ref-bg', always:true, get:i=>i.vessel.id},
      k:{t:'num', def:1e-5, get:i=>i.k, set:(i,v)=>{i.k=v;}},
    },
    build:q=>makeInteraction('flow', q('body').id, q('vessel')) },
];

// The two singletons. Solver tuning (h, beta, reg, maxSub) and view toggles
// (showForces, showGrid) are deliberately NOT here: they are how the engine is run
// and what is drawn, not what the scene is. The ambient and the bath total are, and
// so is the camera -- where an example wants you to be looking is part of it.
const SIM_FIELDS = {
  // always written, default or not: whether gravity is on is the single most
  // consequential fact about a scene, and it keeps the sim line from being bare
  gravity:{t:'onoff', always:true, def:true, get:s=>s.gravity, set:(s,v)=>{s.gravity=v;}},
  g:{t:'num', def:9.8, get:s=>s.g, set:(s,v)=>{s.g=v;}},
  'bg.P':{t:'num', def:101325, get:s=>s.bg.P, set:(s,v)=>{s.bg.P=v;}},
  'bg.T':{t:'num', def:293.15, get:s=>s.bg.T, set:(s,v)=>{s.bg.T=v;}},
  bathQ:{t:'num', def:0, get:s=>s.bathQ, set:(s,v)=>{s.bathQ=v;}},
};
// `always` here means "always written"; the `def` is what an absent key falls back
// to on import. The two singletons are the one place those differ, because sim and
// cam are not rebuilt by a constructor -- see applyFieldsFresh below.
const CAM_FIELDS = {
  x:{t:'num', always:true, def:0, get:c=>c.x, set:(c,v)=>{c.x=v;}},
  y:{t:'num', always:true, def:2.2, get:c=>c.y, set:(c,v)=>{c.y=v;}},
  scale:{t:'num', always:true, def:64, get:c=>c.scale, set:(c,v)=>{c.scale=v;}},
};

// The one constructor interactions did not have. Kept here rather than in §06
// because an interaction is not a constraint -- it carries no row -- but it obeys
// the same one-constructor rule (SCENE.md §S.2); the tool dispatch calls it too.
// `body` carries an `off` like every other endpoint in the engine (geometry.js
// §05.2c) even though nothing yet sets it to anything but [0,0] -- see the heat/
// flow schema rows above.
function makeInteraction(type, bodyId, vesselId){
  return { id:uid++, type, body:{id:bodyId, off:[0,0]}, vessel:{id:vesselId},
           k: type==='heat'?1000:1e-5, sel:false };
}

const endsWord = (A,B) => A&&B ? 'both' : A ? 'A' : B ? 'B' : 'none';
const endsFlags = w => [w==='A'||w==='both', w==='B'||w==='both'];

// ---- §17.2 · values ----
class SceneError extends Error {
  constructor(line, msg){ super(line ? `line ${line}: ${msg}` : msg); this.line=line; }
}

// Significant digits every number is written to. Twelve is a measured choice, not a
// round one, and both directions off it were tried:
//
//   * Nine reads best -- a captured rod length prints as 2.5019992 -- but the ~1e-9
//     it throws away is amplified by the dynamics: an exported and reimported
//     four-bar had visibly diverged from the original after two seconds. A file
//     that does not reproduce the scene it was written from is not a scene file.
//   * Printing exactly (JavaScript's Number->String is the shortest decimal that
//     parses back to the identical double) makes every stored field bit-exact --
//     but a vessel sitting at ambient then exports as P=101325.00000000001, because
//     pressure and temperature are a change of variables away from what a vessel
//     actually stores (its gas mass and adiabat invariant, §05.2d). That exactness
//     is fake: the inverse map is not exact either, so the text was not even stable
//     under a second round trip.
//
// Twelve digits reads as the value in every case that has one (2.6, 800, 101325),
// keeps ~1e-12 of the state -- four orders below where any of this is visible --
// and is stable under repeated round trips, because rounding absorbs exactly the
// last-bit wobble that made exact printing unstable.
const SCENE_PRECISION = 12;
function fmtNum(v){
  if(!isFinite(v)) throw new SceneError(0, `cannot serialize a non-finite number (${v})`);
  const r = Number(v.toPrecision(SCENE_PRECISION));
  return String(r === 0 ? 0 : r);            // normalizes -0 to "0"
}
function numTok(tok, ln, what){
  const v = Number(tok);
  if(tok==='' || !isFinite(v)) throw new SceneError(ln, `${what}: expected a number, got "${tok}"`);
  return v;
}
function idTok(tok, ln, what){
  if(!/^[1-9][0-9]*$/.test(tok)) throw new SceneError(ln, `${what}: expected a body id, got "${tok}"`);
  return Number(tok);
}

// Endpoint syntax, the one nested thing in the format:
//   7            body 7, at its centre
//   7@(0.1,-0.2) body 7, at that offset in its own frame (a material label on a
//                vessel -- §05.2c, so f is a fraction of the length, not a metre)
//   bg(1.15,1.75) the fixed background, at that world point
// Spaces are not allowed inside the parentheses; the line tokenizes on whitespace.
function fmtEp(ep, spec){
  if(spec==='id') return String(ep.id);
  if(spec==='id-bg') return ep.id==null ? 'bg' : String(ep.id);
  if(ep.id==null) return `bg(${fmtNum(ep.off[0])},${fmtNum(ep.off[1])})`;
  const ox=ep.off?ep.off[0]:0, oy=ep.off?ep.off[1]:0;
  return (ox===0 && oy===0) ? String(ep.id) : `${ep.id}@(${fmtNum(ox)},${fmtNum(oy)})`;
}
function parseEp(tok, spec, ln, what){
  if(spec==='id') return {id:idTok(tok, ln, what)};
  if(spec==='id-bg') return {id: tok==='bg' ? null : idTok(tok, ln, what)};
  let m;
  if((m=/^bg\(([^,()]+),([^,()]+)\)$/.exec(tok))){
    if(spec==='ep-body') throw new SceneError(ln, `${what}: this end must be a body, not the background`);
    return {id:null, off:[numTok(m[1],ln,what), numTok(m[2],ln,what)]};
  }
  if((m=/^([1-9][0-9]*)@\(([^,()]+),([^,()]+)\)$/.exec(tok)))
    return {id:Number(m[1]), off:[numTok(m[2],ln,what), numTok(m[3],ln,what)]};
  if(/^[1-9][0-9]*$/.test(tok)) return {id:Number(tok), off:[0,0]};
  throw new SceneError(ln, `${what}: expected an endpoint (7, 7@(x,y) or bg(x,y)), got "${tok}"`);
}

// A field's value -> its token text, and back. `flag` fields have no token: they
// are the bare word itself, present or absent.
function fmtVal(fd, v){
  switch(fd.t){
    case 'num':    return fmtNum(v);
    case 'vec2':   return `(${fmtNum(v[0])},${fmtNum(v[1])})`;
    case 'onoff':  return v ? 'on' : 'off';
    case 'ends':   return v;
    case 'ref':    return String(v);
    case 'ref-bg': return v==null ? 'bg' : String(v);
    // A KEYED field (body=...), not one of the positional `ends` above, but the
    // same {id, off} shape and the same fmtEp -- so it reads and writes exactly
    // like every other body-frame anchor in the format (§17.2).
    case 'ep-centre': return fmtEp(v, 'ep-body');
  }
  throw new SceneError(0, `no formatter for field type ${fd.t}`);
}
function parseVal(fd, name, tok, ln){
  switch(fd.t){
    case 'num': return numTok(tok, ln, name);
    case 'vec2': {
      const m=/^\(([^,()]+),([^,()]+)\)$/.exec(tok);
      if(!m) throw new SceneError(ln, `${name}: expected (x,y), got "${tok}"`);
      return [numTok(m[1],ln,name), numTok(m[2],ln,name)];
    }
    case 'onoff':
      if(tok==='on') return true; if(tok==='off') return false;
      throw new SceneError(ln, `${name}: expected on or off, got "${tok}"`);
    case 'ends':
      if(['none','A','B','both'].includes(tok)) return tok;
      throw new SceneError(ln, `${name}: expected none, A, B or both, got "${tok}"`);
    case 'ref': return idTok(tok, ln, name);
    case 'ref-bg': return tok==='bg' ? null : idTok(tok, ln, name);
    // Nothing yet gives an interaction a real off-centre anchor (physics.js
    // §08.0b's contact area is a whole-body overlap, not a point), so a nonzero
    // offset here can't mean anything the engine can build -- reject it rather
    // than silently keep it and have it do nothing, the same rule §17 opens with.
    case 'ep-centre': {
      const ep = parseEp(tok, 'ep-body', ln, name);
      if(ep.off[0]!==0 || ep.off[1]!==0)
        throw new SceneError(ln, `${name}: an interaction only anchors at a body's centre (0,0) for now`);
      return ep;
    }
  }
  throw new SceneError(ln, `${name}: no parser for field type ${fd.t}`);
}

// Emit a field table for one object: captured fields always, authored fields only
// when they differ from their default, derived fields never (they are not here).
// "Is this still the default" is a tolerant comparison for numbers, not an exact
// one: a mass carried through a resize (which scales it by an area ratio to hold
// density) lands a bit or two off the density-1 value it is conceptually still at,
// and writing it out for that would be noise. The tolerance affects only whether a
// field is OMITTED -- a field that is written is written exactly.
function isDefault(fd, v, d){
  if(fd.t==='num') return v===d || Math.abs(v-d) <= 1e-12*Math.max(Math.abs(v), Math.abs(d));
  return fmtVal(fd, v) === fmtVal(fd, d);
}
function emitFields(fields, o){
  const out=[];
  const q = n => fields[n].get(o);
  for(const [name, fd] of Object.entries(fields)){
    if(fd.when && !fd.when(o)) continue;
    const v = fd.get(o);
    if(fd.t==='flag'){ if(v) out.push(name); continue; }
    if(!fd.always){
      const d = typeof fd.def==='function' ? fd.def(q) : fd.def;
      if(isDefault(fd, v, d)) continue;
    }
    out.push(`${name}=${fmtVal(fd,v)}`);
  }
  return out;
}
// A field table is a plain object literal, so a line claiming a field called
// "constructor" or "toString" would otherwise resolve through Object.prototype and
// walk straight past the unknown-key check that is this format's whole enforcement
// surface (SCENE.md §S.2).
const fieldOf = (fields, name) =>
  Object.prototype.hasOwnProperty.call(fields, name) ? fields[name] : undefined;

// Apply the fields a line actually carried, in declaration order. A field the line
// omitted keeps whatever the constructor gave it -- which is what makes a terse
// hand-written line legal and still mean the obvious thing.
function applyFields(fields, o, f){
  for(const [name, fd] of Object.entries(fields)){
    if(f[name]===undefined || !fd.set) continue;
    fd.set(o, f[name]);
  }
}
// The singleton counterpart, for `sim` and `cam`. They are not rebuilt from a
// constructor -- they are the same live objects the previous scene was using -- so
// "absent" cannot mean "keep what is there": a file with gravity on would not turn
// it back on after a scene that turned it off. An absent field is its DEFAULT here.
function applyFieldsFresh(fields, o, f){
  for(const [name, fd] of Object.entries(fields)){
    if(!fd.set) continue;
    // Both singleton tables use plain-value defaults; neither needs the q()
    // cross-field form the body rows use for mass.
    const v = f[name]!==undefined ? f[name] : fd.def;
    if(v!==undefined) fd.set(o, v);
  }
}
// Read a field for `build`: what the line said, else the field's default.
function fieldReader(fields, f, ln){
  const q = name => {
    const fd = fieldOf(fields, name);
    if(!fd) throw new SceneError(ln, `internal: no field "${name}"`);
    if(f[name]!==undefined) return f[name];
    if(fd.always) throw new SceneError(ln, `missing required field "${name}"`);
    return typeof fd.def==='function' ? fd.def(q) : fd.def;
  };
  return q;
}

// ---- §17.3 · exportScene (world -> text) ----
// Bodies first, then everything that refers to them. The reader does not require
// that order (it builds bodies in a first pass), but a file a person reads should
// introduce a thing before mentioning it.
function exportScene(){
  const rowFor = (list, o) => SCENE_SCHEMA.find(r => r.list===list && r.match(o));
  const emit = (list, o) => {
    const r = rowFor(list, o);
    if(!r) throw new SceneError(0, `nothing in the scene table matches a ${list} entry of type "${o.type||o.shape}"`);
    const parts=[r.kind];
    if(r.id) parts.push(String(o.id));
    if(r.ends){
      const toks = r.ends.map(([name,spec]) => fmtEp(o[name], spec));
      parts.push(toks.length>1 ? toks.join(' -- ') : toks[0]);
    }
    return parts.concat(emitFields(r.fields, o)).join(' ');
  };
  const L = [`scene ${SCENE_VERSION}`, ''];
  L.push(['sim', ...emitFields(SIM_FIELDS, sim)].join(' '));
  L.push(['cam', ...emitFields(CAM_FIELDS, cam)].join(' '));
  const section = (title, list, arr) => {
    if(!arr.length) return;
    L.push('', `# ${title}`);
    for(const o of arr) L.push(emit(list, o));
  };
  section('bodies', 'bodies', bodies);
  section('constraints', 'constraints', constraints);
  section('cables', 'cables', cables);
  section('springs', 'springs', springs);
  section('rotational springs', 'rotSprings', rotSprings);
  section('interactions', 'interactions', interactions);
  return L.join('\n') + '\n';
}

// A vessel's fourth coordinate (len) and its gas (P, T, mass) are related by
// exactly one equation at a fixed bore -- P*(bore*len) = mass*Rs*T -- so a line
// may give any TWO of {P, T, gasMass} alongside len (the third follows), or, with
// len left out entirely, all THREE of them (len is then exactly the length they
// imply). Anything short of that leaves the state underdetermined; anything past
// it over-determines it -- both are load-time errors, not a silent guess. Mutates
// the raw parsed fields in place so len, P and T (all `always` in the vessel row,
// §17.1) are unconditionally present by the time the ordinary fieldReader / build
// / finish pipeline runs below, whatever subset of the four the file actually
// wrote. Runs as its own pass after the whole file is read (not inline in the
// per-line loop) so a `sim` line naming a non-default ambient is already known
// here, wherever in the file it appears.
function resolveVesselFill(f, ln, bgP, bgT){
  if(f.bore===undefined) throw new SceneError(ln, `vessel: missing required field "bore"`);
  const has = k => f[k]!==undefined;
  const nGas = ['P','T','gasMass'].filter(has).length;
  if(has('len')){
    if(nGas===0){ f.P=bgP; f.T=bgT; return; }                 // ambient, same as a freshly placed vessel
    if(nGas===1) throw new SceneError(ln, `vessel: len given with only one of P, T, gasMass -- give two of them, or none for ambient`);
    if(nGas===3) throw new SceneError(ln, `vessel: len, P, T and gasMass together over-determine the gas -- give at most two of P, T, gasMass when len is set`);
    if(has('P') && has('T')) return;                          // already exactly what finish() below expects
    const V = f.bore*f.len*VESSEL_DEPTH;
    if(!(V>0)) throw new SceneError(ln, `vessel: bore and len must be positive`);
    if(has('T') && has('gasMass')) f.P = f.gasMass*GAS_AIR.Rs*f.T/V;
    else { if(!(f.gasMass>0)) throw new SceneError(ln, `vessel: gasMass must be positive to compute T from P and gasMass`);
      f.T = f.P*V/(f.gasMass*GAS_AIR.Rs); }
    return;
  }
  if(nGas!==3) throw new SceneError(ln, `vessel: no len given -- provide len, or all three of P, T and gasMass so length can be computed`);
  if(!(f.P>0)) throw new SceneError(ln, `vessel: P must be positive to compute len from P, T and gasMass`);
  const V = f.gasMass*GAS_AIR.Rs*f.T/f.P;
  if(!(V>0)) throw new SceneError(ln, `vessel: P, T and gasMass do not imply a positive length`);
  f.len = V/(f.bore*VESSEL_DEPTH);
}

// ---- §17.4 · importScene (text -> world) ----
// Two passes, and the split matters. parseScene reads the whole file and validates
// everything it can without touching the world -- syntax, unknown kinds and keys,
// duplicate and dangling body ids. Only once that returns does commitScene clear
// the bench. A bad file therefore leaves the current scene exactly as it was,
// rather than half-replacing it.
function parseScene(text){
  const out = { version:null, sim:{}, cam:{}, items:[] };
  const lines = String(text).split(/\r?\n/);
  const bodyIds = new Set();
  const kindRow = k => SCENE_SCHEMA.find(r => r.kind===k);

  for(let i=0;i<lines.length;i++){
    const ln = i+1;
    const raw = lines[i].split('#')[0].trim();
    if(!raw) continue;
    const tok = raw.split(/\s+/);
    const kind = tok.shift();

    if(out.version===null){
      if(kind!=='scene') throw new SceneError(ln, `a scene file must open with "scene ${SCENE_VERSION}"`);
      out.version = numTok(tok[0]||'', ln, 'scene version');
      if(out.version !== SCENE_VERSION)
        throw new SceneError(ln, `this is a version ${out.version} scene file; this build reads version ${SCENE_VERSION}`);
      continue;
    }
    if(kind==='scene') throw new SceneError(ln, 'duplicate "scene" line');

    if(kind==='sim' || kind==='cam'){
      const fields = kind==='sim' ? SIM_FIELDS : CAM_FIELDS;
      out[kind] = readKeyed(fields, tok, ln, kind);
      continue;
    }

    const row = kindRow(kind);
    if(!row) throw new SceneError(ln, `unknown kind "${kind}" -- a scene can only contain ${SCENE_SCHEMA.map(r=>r.kind).join(', ')}`);

    const item = { row, ln, id:null, ends:{}, f:null };
    if(row.id){
      const idt = tok.shift();
      if(idt===undefined) throw new SceneError(ln, `a ${kind} line needs an id`);
      item.id = idTok(idt, ln, `${kind} id`);
      if(bodyIds.has(item.id)) throw new SceneError(ln, `duplicate body id ${item.id}`);
      bodyIds.add(item.id);
    }
    if(row.ends){
      for(let e=0;e<row.ends.length;e++){
        const [name, spec] = row.ends[e];
        if(e>0){
          const sep = tok.shift();
          if(sep!=='--') throw new SceneError(ln, `expected "--" between the two ends of a ${kind}, got "${sep===undefined?'end of line':sep}"`);
        }
        const t = tok.shift();
        if(t===undefined) throw new SceneError(ln, `a ${kind} line needs ${row.ends.length} endpoint(s)`);
        item.ends[name] = parseEp(t, spec, ln, `${kind} end ${name}`);
      }
    }
    item.f = readKeyed(row.fields, tok, ln, kind);
    out.items.push(item);
  }

  if(out.version===null) throw new SceneError(0, 'empty scene file');

  // Resolve every vessel's fill now that the whole file (in particular a `sim`
  // line naming a non-default ambient, wherever it appears) has been read.
  {
    const bgP = out.sim['bg.P']!==undefined ? out.sim['bg.P'] : SIM_FIELDS['bg.P'].def;
    const bgT = out.sim['bg.T']!==undefined ? out.sim['bg.T'] : SIM_FIELDS['bg.T'].def;
    for(const it of out.items) if(it.row.kind==='vessel') resolveVesselFill(it.f, it.ln, bgP, bgT);
  }

  // Every body id anything names has to exist. Checked here, before the bench is
  // touched, so a typo in an endpoint is a message rather than a wrecked scene.
  for(const it of out.items){
    const refs=[];
    for(const [name] of (it.row.ends||[])) if(it.ends[name].id!=null) refs.push([it.ends[name].id, `end ${name}`]);
    for(const [name, fd] of Object.entries(it.row.fields)){
      if((fd.t==='ref' || fd.t==='ref-bg') && it.f[name]!=null) refs.push([it.f[name], name]);
      if(fd.t==='ep-centre' && it.f[name]!=null) refs.push([it.f[name].id, name]);
    }
    for(const [id, what] of refs)
      if(!bodyIds.has(id)) throw new SceneError(it.ln, `${it.row.kind} ${what} names body ${id}, which this file does not define`);
  }
  return out;
}
// The keyed tail of a line: key=value pairs and bare flag words, in any order.
function readKeyed(fields, tok, ln, kind){
  const f = Object.create(null);
  for(const t of tok){
    const eq = t.indexOf('=');
    const name = eq<0 ? t : t.slice(0, eq);
    const fd = fieldOf(fields, name);
    if(!fd) throw new SceneError(ln, `${kind}: unknown field "${name}" -- this kind takes ${Object.keys(fields).join(', ')||'no fields'}`);
    if(f[name]!==undefined) throw new SceneError(ln, `${kind}: "${name}" given twice`);
    if(fd.t==='flag'){
      if(eq>=0) throw new SceneError(ln, `${kind}: "${name}" is a flag -- write it on its own, with no value`);
      f[name] = true;
    } else {
      if(eq<0) throw new SceneError(ln, `${kind}: "${name}" needs a value, as ${name}=...`);
      f[name] = parseVal(fd, name, t.slice(eq+1), ln);
    }
  }
  return f;
}

// Empty the bench. Everything a scene owns goes, including the two running totals
// that belong to the scene that spent them: the bath's cumulative draw (§04.3) and
// the per-island energy bank (§08.6), whose key is a body id the next scene will
// reuse. Shared with §15 so there is one answer to "what is a fresh bench".
function clearScene(){
  bodies=[]; constraints=[]; cables=[]; springs=[]; rotSprings=[]; interactions=[];
  uid=1; sim.bathQ=0;
  ENERGY_BANK.clear();
  refreshFrozen();
  clearSelection();
  eHist.length=0;
}

// The world arrays are top-level `let` bindings (§04.2), which in a classic script
// live in the global declarative record and are NOT properties of `window` -- so a
// row's `list` name has to be dispatched, not looked up.
const SCENE_LISTS = {
  bodies:      o=>bodies.push(o),
  constraints: o=>constraints.push(o),
  cables:      o=>cables.push(o),
  springs:     o=>springs.push(o),
  rotSprings:  o=>rotSprings.push(o),
  interactions:o=>interactions.push(o),
};

function commitScene(parsed){
  clearScene();
  // Bodies first, whatever order the file used: every constructor below this line
  // resolves endpoints through the live `bodies` array (§06.1 epWorld), and a belt
  // or cable reads its spool's radius at construction.
  const build = it => {
    const q = fieldReader(it.row.fields, it.f, it.ln);
    const o = it.row.build(q, it.ends);
    if(it.row.id) o.id = it.id;             // the file's ids are the scene's ids
    applyFields(it.row.fields, o, it.f);
    if(it.row.finish) it.row.finish(o, it.f, q);
    SCENE_LISTS[it.row.list](o);
  };
  const isBody = it => it.row.list==='bodies';
  for(const it of parsed.items) if(isBody(it)) build(it);
  uid = bodies.reduce((m,b)=>Math.max(m,b.id), 0) + 1;
  for(const it of parsed.items) if(!isBody(it)) build(it);

  applyFieldsFresh(SIM_FIELDS, sim, parsed.sim);
  applyFieldsFresh(CAM_FIELDS, cam, parsed.cam);
  if(typeof document!=='undefined'){
    const gc=document.getElementById('tgGrav'); if(gc) gc.checked=sim.gravity;
  }

  // No position projection: the file's pose is the scene's pose. Projecting would
  // quietly move bodies off what the file says, which is the one thing a format
  // whose job is fidelity must not do.
  // Which coordinates the scene has pinned is derived from what it contains
  // (§06.2b) -- do it now so the first render, the first Reset baseline and any
  // inspector readout all see the same answer the substep will.
  refreshFrozen();
  clearSelection();
  saveState();
}

// The whole of it: parse (throws on anything wrong), then commit.
function importScene(text){ commitScene(parseScene(text)); }

// ---- §17.6 · the state snapshot (what Reset restores) ----
// Walks the same ledger, over each row's `state` list. Structure is not captured --
// Reset never adds or removes anything -- so a snapshot is one record per object,
// positional within its list, carrying a body's id so a structural edit between
// save and restore is skipped rather than applied to the wrong object.
//
// This is deliberately NOT the text format: it runs on every inspector keystroke
// (§16.1 saveState) and has to be exact, so it copies values rather than formatting
// them. What it shares with the format is the ledger, which is the part that was
// being maintained twice.
const SCENE_STATE_LISTS = () => [['bodies',bodies], ['constraints',constraints],
  ['cables',cables], ['springs',springs], ['rotSprings',rotSprings], ['interactions',interactions]];

function snapshotState(){
  const rec = {};
  for(const [list, arr] of SCENE_STATE_LISTS()){
    rec[list] = arr.map(o=>{
      const row = SCENE_SCHEMA.find(r => r.list===list && r.match(o));
      if(!row || !row.state) return null;
      return { id:o.id, v:row.state.map(([,get]) => get(o)) };
    });
  }
  rec.bathQ = sim.bathQ;
  return rec;
}
function applyState(rec){
  if(!rec) return;
  for(const [list, arr] of SCENE_STATE_LISTS()){
    const saved = rec[list]; if(!saved) continue;
    arr.forEach((o,i)=>{
      const s = saved[i]; if(!s) return;
      if(s.id!==undefined && o.id!==undefined && s.id!==o.id) return;  // structure moved under us
      const row = SCENE_SCHEMA.find(r => r.list===list && r.match(o));
      if(!row || !row.state) return;
      row.state.forEach(([, , set], k)=>set(o, s.v[k]));
      if(row.restore) row.restore(o);
    });
  }
  sim.bathQ = rec.bathQ || 0;
}

// ---- §17.5 · panel UI ----
// The card shows the LIVE bench. That matters more than it sounds: the textarea is
// what Download writes and what Import reads, so text that lags the scene is text
// that lies about it. It is regenerated on every panel render (which is every edit,
// and cheap at these sizes -- and the card only exists while nothing is selected).
//
// Two things override the live text, in order:
//   * a DRAFT -- anything the player typed or pasted -- which is theirs and is never
//     overwritten. Import consumes it; Export discards it.
//   * an ANNOTATED file, kept only while the bench still matches it line for line
//     with comments stripped. That is what lets clicking an example show the
//     example's own file, prose and all, and lets an imported file keep its
//     comments -- while the first edit that actually changes the scene drops
//     silently back to the plain export.
let sceneText = null;                       // the annotated file, while it still fits
let sceneDraft = null;                      // the player's own text, untouched
let sceneMsg = null;                        // {ok, text} shown under the card

// A scene file with its comments and blank lines removed -- the canonical content,
// which is what "does this file still describe the bench" compares. Shared with
// tools/scene-roundtrip.js.
const sceneStrip = t =>
  String(t).split('\n').filter(l=>l.trim() && !l.trim().startsWith('#')).join('\n')+'\n';

// exportScene() itself just walks the LIVE world arrays, which mid-run are the
// running pose, not the reset baseline (SCENE.md S.3: "export should write the
// reset baseline... offer 'export current state' as an explicit second command if
// it is wanted" -- Capture, below, is that command). To write the baseline instead,
// swap `saved` onto the live objects, export, then swap the live pose straight
// back. That round trip only ever touches the STATE-list fields (§17.6) -- pose,
// velocity, a vessel's gas -- never structure, so nothing else the format writes
// (shape, mass, weld flags, ...) can differ between the two swaps. It is safe
// across a running sim too: this all happens inside one synchronous click handler,
// so no frame or substep can land between the two applyState() calls and see the
// bench in its borrowed, mid-swap state.
function sceneBaselineText(){
  if(!saved) return exportScene();
  const live = snapshotState();
  applyState(saved);
  try { return exportScene(); }
  finally { applyState(live); }
}

// Pure -- it must not drop the annotated file as a side effect of being asked. This
// runs mid-import too, at the moment clearScene has emptied the bench and nothing
// matches anything; discarding then would throw away the file about to be loaded.
// Leaving it in place also means undoing an edit brings its comments back.
function sceneCardText(){
  if(sceneDraft!==null) return sceneDraft;
  let baseline;
  try { baseline = sceneBaselineText(); }
  catch(e){ return `# this bench cannot be written out: ${e.message||e}`; }
  return (sceneText!==null && sceneStrip(sceneText)===sceneStrip(baseline)) ? sceneText : baseline;
}

function sceneCardHTML(){
  return `
    <div class="card"><div class="cardhead">scene file</div>
      <textarea id="sc_text" class="scenebox" spellcheck="false"
        placeholder="This bench, as a scene file. Edit or paste one and press Import to load it — or drop a file on the canvas."></textarea>
      <div class="scenebtns">
        <button id="sc_export">Export</button>
        <button id="sc_capture">Capture</button>
        <button id="sc_import">Import</button>
        <button id="sc_copy">Copy</button>
        <button id="sc_save">Download</button>
      </div>
      ${sceneMsg ? `<p class="${sceneMsg.ok?'muted':'scerr'}" style="margin:8px 0 0">${escHtml(sceneMsg.text)}</p>` : ''}
      <p class="muted" style="margin:8px 0 0">A plain-text listing of every body, constraint, force element and interaction, with the ambient and the camera. Export (and the panel itself) hold the reset baseline — what R restores to — not the mid-run pose. Capture instead freezes the exact current state, mid-run included. Either way, pressing it discards your edits to the text and rewrites the field from the bench.</p>
    </div>`;
}
const escHtml = s => String(s).replace(/[&<>]/g, c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]));

function wireSceneCard(){
  const ta=document.getElementById('sc_text'); if(!ta) return;
  ta.value=sceneCardText();
  ta.oninput=()=>{ sceneDraft=ta.value; };
  const say=(ok,text)=>{ sceneMsg={ok,text}; renderInspector(); };
  document.getElementById('sc_export').onclick=()=>{
    sceneDraft=null; sceneText=null;          // back to the plain baseline export
    say(true, `${bodies.length} bodies and ${constraints.length+cables.length+springs.length+rotSprings.length+interactions.length} couplings, at the reset baseline.`);
  };
  document.getElementById('sc_capture').onclick=()=>{
    let text;
    try { text = exportScene(); }
    catch(e){ say(false, String(e.message||e)); return; }
    sceneDraft=text; sceneText=null;          // held as a draft so a later re-render can't fall back to the baseline
    say(true, `Captured the exact current state (${bodies.length} bodies) — not the reset baseline.`);
  };
  document.getElementById('sc_import').onclick=()=>{
    const text=ta.value;
    try { importScene(text); sceneDraft=null; sceneText=text; say(true, `Loaded ${bodies.length} bodies.`); }
    catch(e){ sceneDraft=text; say(false, String(e.message||e)); }
  };
  document.getElementById('sc_copy').onclick=()=>{
    if(navigator.clipboard) navigator.clipboard.writeText(ta.value).then(()=>say(true,'Copied.'), ()=>say(false,'Clipboard refused; select the text and copy it.'));
    else { ta.select(); say(true,'Selected — press Ctrl/Cmd-C.'); }
  };
  document.getElementById('sc_save').onclick=()=>{
    const blob=new Blob([ta.value], {type:'text/plain'});
    const a=document.createElement('a');
    a.href=URL.createObjectURL(blob); a.download='bench.scene';
    a.click(); setTimeout(()=>URL.revokeObjectURL(a.href), 1000);
  };
}
// Drop a scene file anywhere on the canvas to load it. The same strict reader, so
// a bad file reports its line and leaves the bench alone.
if(typeof cv!=='undefined' && cv){
  cv.addEventListener('dragover', e=>{ e.preventDefault(); });
  cv.addEventListener('drop', e=>{
    e.preventDefault();
    const file=e.dataTransfer && e.dataTransfer.files && e.dataTransfer.files[0]; if(!file) return;
    file.text().then(t=>{
      try { importScene(t); sceneDraft=null; sceneText=t; sceneMsg={ok:true, text:`Loaded ${file.name}.`}; }
      catch(err){ sceneDraft=t; sceneMsg={ok:false, text:`${file.name}: ${err.message||err}`}; }
      clearSelection();
    });
  });
}
