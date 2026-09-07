// Scene file round-trip -- SCENE.md §S.2.
//
// Unlike the vessel-check scripts beside it, this one DOES load the simulator: the
// claim under test is about the real reader and writer, so a reimplementation would
// test nothing. It loads the handful of source files the scene format touches into
// a bare context with a stub DOM, then for every bundled example asserts:
//
//   1. export -> import -> export is byte-identical. A canonical file round-trips
//      exactly, so the format loses nothing it claims to carry.
//   2. every authored and captured field survives, compared object by object
//      against the live world the example built (SCENE.md §S.3's taxonomy is the
//      list of what must match; derived fields are recomputed and not compared).
//   3. the reader accepts nothing outside the ledger -- a made-up kind, a made-up
//      key, a dangling body reference and a bad version each raise, and a rejected
//      file leaves the previous scene standing.
//
// (2) is the one that catches the drift this format exists to prevent: add a field
// to a scene object and forget the ledger, and the export comes back missing it.
const fs=require('fs'), path=require('path'), vm=require('vm');
const ROOT=path.join(__dirname,'..');

let pass=0, fail=0;
const ok=(name,good,detail)=>{ good?pass++:fail++;
  console.log((good?'  ok  ':'  FAIL'), name, good?'':('\n        '+(detail||''))); };

// ---- a DOM thin enough to load state.js and wide enough for the two guarded
// ---- getElementById calls the scene reader and the examples make.
const stubEl = () => new Proxy({}, { get:(t,k)=>
  k==='getContext' ? ()=>new Proxy({},{get:()=>()=>{}}) :
  k==='classList'  ? {add(){},remove(){},toggle(){}} :
  k in t ? t[k] : ()=>{},
  set:(t,k,v)=>{ t[k]=v; return true; } });
const ctx = vm.createContext({
  document:{ getElementById:()=>stubEl(), createElement:()=>stubEl(),
             querySelectorAll:()=>[], addEventListener(){} },
  window:{addEventListener(){}}, performance:{now:()=>0}, console,
  Math, JSON, Number, String, Object, Array, Map, Set, Error,
  requestAnimationFrame:()=>{},
});
ctx.globalThis = ctx;
// Stubs for the parts of the engine the scene path calls but does not need loaded:
// selection and the tool rail both live in DOM-heavy files, and nothing under test
// touches either. saveState/restoreState are NOT stubbed -- check (6) is about the
// real ones, so js/transport.js is loaded below like any other source file, and
// js/select.js with it, since both of those name the group selection it owns.
vm.runInContext(`
  function clearSelection(){} function renderInspector(){}
  function setTool(){} var TOOLS=[];
`, ctx);
// solver/physics/projection come along because check (4) below runs the real
// substep on both worlds -- a derived field the reader recomputed wrongly shows up
// as a diverging trajectory even when every serialized field matches.
for(const f of ['js/state.js','js/expr.js','js/geometry.js','js/constraints.js','js/solver.js',
                'js/physics.js','js/projection.js','js/loop.js','js/hud.js','js/scene.js','js/examples.js',
                'js/select.js','js/transport.js'])
  vm.runInContext(fs.readFileSync(path.join(ROOT,f),'utf8'), ctx, {filename:f});

const run = src => vm.runInContext(src, ctx);
const EXAMPLES = run('Object.keys(SCENES)');

// ---- the field list (2) compares, straight off the ledger, so it cannot go stale
function snapshot(){
  return run(`(()=>{
    const rowFor=(list,o)=>SCENE_SCHEMA.find(r=>r.list===list&&r.match(o));
    const grab=(list,arr)=>arr.map(o=>{
      const r=rowFor(list,o); const rec={_kind:r.kind};
      if(r.id) rec._id=o.id;
      for(const [n,spec] of (r.ends||[])) rec['end:'+n]=fmtEp(o[n],spec);
      for(const [n,fd] of Object.entries(r.fields)){
        if(fd.when && !fd.when(o)) continue;
        rec[n]=fd.t==='flag'?!!fd.get(o):fmtVal(fd,fd.get(o));
      }
      return rec;
    });
    return JSON.stringify({
      sim:Object.fromEntries(Object.entries(SIM_FIELDS).map(([n,fd])=>[n,fmtVal(fd,fd.get(sim))])),
      cam:Object.fromEntries(Object.entries(CAM_FIELDS).map(([n,fd])=>[n,fmtVal(fd,fd.get(cam))])),
      bodies:grab('bodies',bodies), constraints:grab('constraints',constraints),
      cables:grab('cables',cables), springs:grab('springs',springs),
      rotSprings:grab('rotSprings',rotSprings), interactions:grab('interactions',interactions),
    },null,1);
  })()`);
}

console.log('\n1. every example exports, and the export round-trips byte-for-byte');
const texts={};
for(const ex of EXAMPLES){
  let t1,t2,s1,s2,err=null;
  try {
    run(`loadExample(${JSON.stringify(ex)})`);
    t1=run('exportScene()'); s1=snapshot();
    run(`importScene(${JSON.stringify(t1)})`);
    t2=run('exportScene()'); s2=snapshot();
  } catch(e){ err=e; }
  texts[ex]=t1;
  if(err){ ok(ex.padEnd(12)+'exports and reimports', false, err.stack||String(err)); continue; }
  ok(ex.padEnd(12)+'export === reexport', t1===t2, firstDiff(t1,t2));
  ok(ex.padEnd(12)+'every ledger field survives', s1===s2, firstDiff(s1,s2));
}

console.log('\n2. every bundled example is a canonical scene file');
// The examples ARE scene files now (§15), so they can drift from what the exporter
// would write -- someone hand-edits one, or a ledger change alters the canonical
// form and the checked-in text is not regenerated. Stripped of its comments and
// blank lines, each one must be exactly the export. Run with --canon <name> to
// print the canonical text of one, which is how you regenerate after a change.
const strip = run('sceneStrip');   // the reader's own definition, §17.5
const canonArg = process.argv.indexOf('--canon');
if(canonArg>=0){
  const name = process.argv[canonArg+1];
  run(`loadExample(${JSON.stringify(name)})`);
  process.stdout.write(run('exportScene()'));
  process.exit(0);
}
for(const ex of EXAMPLES){
  run(`loadExample(${JSON.stringify(ex)})`);
  const written = run(`SCENES[${JSON.stringify(ex)}]`), canon = run('exportScene()');
  ok(ex.padEnd(12)+'is canonical as checked in', strip(written)===strip(canon), firstDiff(strip(written), strip(canon)));
}

console.log('\n3. the file really carries the scene (spot checks on what it says)');
const has=(ex,re,what)=>ok(`${ex.padEnd(12)}${what}`, re.test(texts[ex]||''),
  `not found in:\n${(texts[ex]||'').split('\n').map(l=>'        '+l).join('\n')}`);
has('heatpair', /^rod bg\(0,2\.15\) -- 1 .*weld=both/m, 'the plate is held by a ground weld, not a flag');
has('heatpair', /^rod 2@\(0,-0\.5\) -- 2@\(0,0\.5\) len=1\.8$/m, 'the reservoir is a vessel with a strut in it');
has('heatpair', /^heat body=\d+ vessel=\d+ k=2000$/m,'both heat interactions, with k');
has('heatpair', /^rod bg\(1\.15,1\.75\) -- \d+ len=\S+ weld=both/m, 'the anchoring rod, welded both ends');
has('pendulum', /^rod bg\(0,4\.4\) -- \d+ len=2\.6$/m,'an unwelded background rod (weld omitted = none)');
has('flowpair', /^vessel \d+ .* P=243180 /m,         'the reservoir is at 2.4 atm');
has('spinvessel',/^sim gravity=off/m,                'gravity is off');
has('spinvessel',/^vessel \d+ .* w=9$/m,              'the initial spin');
has('crank',    /^slot \d+ -- bg\([^)]*\) lock=B/m,  'the one-end-prismatic rail');
has('rack',     /^rack \d+@\([^)]*\) -- \d+ pt=\d+\/pinion$/m,
                'the rack: two pins naming the line, the pinion as a control point');
has('skate',    /^knife \d+@\(0\.42,0\) dir=\(1,0\)$/m,'the knife heading');
has('cable',    /^cable \d+ -- \d+ Ltot=\S+ localAngle=\S+$/m,'the cable, with its captured length');
has('cable',    /^rod bg\(0,5\.1\) -- 1 .*weld=both/m,  'the spool is grounded by a rod');

// Nothing anywhere may still carry the removed flags: they are derived now, and a
// file that set one would be freezing a coordinate by assertion again.
for(const ex of EXAMPLES){
  const bad = (texts[ex]||'').split('\n').filter(l=>!l.trim().startsWith('#') && /\b(static|lenlock)\b/.test(l));
  ok(ex.padEnd(12)+'freezes nothing by assertion', bad.length===0, bad.join('\n        '));
}

console.log('\n4. freezing is derived from the constraints, per coordinate');
// The rules (constraints.js §06.2b) and the cases that separate them. The vessel
// rows are the point: a rod welded to a MID-WALL pins the pose and leaves the
// length free, the same rod welded to a CAP pins neither, and only a strut between
// two of a vessel's own planes locks the length.
const frozen = () => run(`JSON.stringify(bodies.map(b=>[b.id,!!b.static,!!b.lenLock]))`);
const cases = [
  ['heatpair', 'plate on a ground weld is pinned',            b=>b[0][1]===true],
  ['heatpair', 'reservoir with a strut: length locked, pose free',
                                                              b=>b[1][1]===false && b[1][2]===true],
  ['heatpair', 'vessel welded at its mid-wall: pose pinned, length FREE',
                                                              b=>b[2][1]===true && b[2][2]===false],
  ['gasspring','vessel welded at its CAP: nothing frozen',    b=>b[0][1]===false && b[0][2]===false],
  ['cable',    'spool on a ground weld is pinned',            b=>b[0][1]===true],
  ['cable',    'the hanging mass is not',                     b=>b[1][1]===false],
  ['pendulum', 'a bob on a pin-ended rod is not pinned',      b=>b[0][1]===false],
  ['integrator','a disk on a ONE-end weld is not pinned',     b=>b[0][1]===false],
];
for(const [ex, what, pred] of cases){
  run(`loadExample(${JSON.stringify(ex)})`);
  const b=JSON.parse(frozen());
  ok((ex+' ').padEnd(12)+what, pred(b), JSON.stringify(b));
}
// Transitivity, and the constraint that does the freezing being compiled away.
run(`(()=>{ clearScene();
  const a=makeBody(0,1,0.3); bodies.push(a); const c=makeBody(1,1,0.3); bodies.push(c);
  constraints.push(makeRodCon({id:null,off:[0,0]},{id:a.id,off:[0,0]},true,true));
  constraints.push(makeRodCon({id:a.id,off:[0,0]},{id:c.id,off:[0,0]},true,true));
  refreshFrozen(); })()`);
ok('grounding is transitive through a double weld', frozen()==='[[1,true,false],[2,true,false]]', frozen());
ok('both grounding rods are compiled away', run('JSON.stringify(constraints.map(c=>!!c._compiled))')==='[true,true]');
// ...and that deleting the rod thaws the body again, which is the whole point of
// freezing being derived rather than stored.
run('constraints.length=0; refreshFrozen()');
ok('deleting the rods thaws both bodies', frozen()==='[[1,false,false],[2,false,false]]', frozen());

console.log('\n5. the reader accepts nothing the editor cannot build');
run(`loadExample('pendulum')`);
const before = run('exportScene()');
const rejects = [
  ['an unknown kind',            'scene 3\nrocket 1 x=0 y=0'],
  ['an unknown field',           'scene 3\nbody 1 x=0 y=0 r=1 charge=3'],
  ['a coordinate frozen by fiat','scene 3\nbody 1 x=0 y=0 r=1 xLocked'],
  ['a dangling body reference',  'scene 3\nbody 1 x=0 y=0 r=1\nrod 1 -- 7'],
  ['a future version',           'scene 4\nbody 1 x=0 y=0 r=1'],
  ['a version 1 file',           'scene 1\nbody 1 x=0 y=0 r=1'],
  ['a missing separator',        'scene 3\nbody 1 x=0 y=0 r=1\nbody 2 x=2 y=0 r=1\nrod 1 2'],
  ['a duplicate id',             'scene 3\nbody 1 x=0 y=0 r=1\nbody 1 x=2 y=0 r=1'],
  ['an authored static flag',    'scene 3\nbody 1 x=0 y=0 r=1 static'],
  ['an authored length lock',    'scene 3\nvessel 1 x=0 y=0 bore=1 len=1 lenlock'],
  ['a prototype key',            'scene 3\nbody 1 x=0 y=0 r=1 constructor=1'],
  ['a background pin',           'scene 3\nbody 1 x=0 y=0 r=1\npin 1 -- bg(0,1)'],
  // The extra-control-point grammar (constraints.js §06.2c) is validated per kind,
  // the same way a line's own keys are: what a point may say depends on what it is
  // attached to, and anything else is a load error rather than a field ignored.
  ['a pinion on a slot',         'scene 3\nbody 1 x=0 y=0 r=1\nbody 2 x=2 y=0 r=1\nbody 3 x=1 y=0 r=1\nslot 1 -- 2 pt=3/pinion'],
  ['a station on a pin',         'scene 3\nbody 1 x=0 y=0 r=1\nbody 2 x=2 y=0 r=1\nbody 3 x=1 y=0 r=1\npin 1 -- 2 pt=3/s=0.5'],
  ['a rod point with no station','scene 3\nbody 1 x=0 y=0 r=1\nbody 2 x=2 y=0 r=1\nbody 3 x=1 y=0 r=1\nrod 1 -- 2 len=2 pt=3'],
  ['a locked point with no rest angle',
                                 'scene 3\nbody 1 x=0 y=0 r=1\nbody 2 x=2 y=0 r=1\nbody 3 x=1 y=0 r=1\nslot 1 -- 2 pt=3/lock'],
  ['a dangling point reference', 'scene 3\nbody 1 x=0 y=0 r=1\nbody 2 x=2 y=0 r=1\nslot 1 -- 2 pt=9'],
];
for(const [what, text] of rejects){
  let msg=null;
  try { run(`importScene(${JSON.stringify(text)})`); } catch(e){ msg=e.message; }
  ok(('rejects '+what).padEnd(40), !!msg, 'accepted it');
  if(msg) console.log('        ->', msg);
}
ok('a rejected file leaves the bench standing', run('exportScene()')===before);

console.log('\n6. an imported scene runs the same as the scene it was exported from');
// Every check above compares what the file SAYS. This one compares what the file
// DOES: two seconds of the real substep on the example the tools built and on the
// world the reader rebuilt from its export, then the full state of both. A derived
// quantity the reader failed to recompute -- an inertia, an inverse mass, a gas
// adiabat -- is invisible to a field-by-field comparison and lands here.
const STATE = `JSON.stringify(bodies.map(b=>[b.x,b.y,b.th,b.vx,b.vy,b.w,b.len||0,b.vlen||0,
                b.mass,b.I,b.invM,b.invI,b.mu||0,b.invMu||0,b.gas?b.gas.kap:0,b.gas?b.gas.mass:0]))`;
function traceOf(setup, steps){
  run(setup);
  const out=[run(STATE)];                       // the state as loaded, before any step
  run(`projectPositions(20)`);
  for(let i=0;i<steps;i++){ run(`substep(sim.h)`); if(i%40===39) out.push(run(STATE)); }
  return out;
}
// Two claims, with two tolerances, because they are different claims. The loaded
// state must match to the format's own precision -- that is fidelity, and it covers
// the DERIVED fields (inertia, inverse masses, the adiabat invariant) that no
// field-by-field comparison of the file can reach. The trajectory afterwards is
// allowed to drift by the amplification a linkage applies to a 1e-12 seed over two
// seconds; the claim there is only that it is recognisably the same run.
for(const ex of EXAMPLES){
  const a = traceOf(`loadExample(${JSON.stringify(ex)})`, 240);
  const b = traceOf(`importScene(${JSON.stringify(texts[ex])})`, 240);
  const worstOver = (from, to) => {
    let worst=0, where='';
    for(let s=from;s<to;s++){
      const A=JSON.parse(a[s]), B=JSON.parse(b[s]);
      if(A.length!==B.length) return [Infinity, 'body count differs'];
      for(let i=0;i<A.length;i++) for(let j=0;j<A[i].length;j++){
        const d=Math.abs(A[i][j]-B[i][j])/Math.max(1,Math.abs(A[i][j]));
        if(d>worst){ worst=d; where=`body ${i}, field ${j}${s?`, at step ${s*40}`:' as loaded'}`; }
      }
    }
    return [worst, where];
  };
  const [w0, e0] = worstOver(0,1), [w1, e1] = worstOver(1,a.length);
  ok(ex.padEnd(12)+'loads to the same state', w0<1e-11, `worst ${w0.toExponential(2)} (${e0})`);
  ok(ex.padEnd(12)+'2 s of substeps agree  ', w1<1e-6,  `worst ${w1.toExponential(2)} (${e1})`);
}

console.log('\n7. Reset puts back exactly what was loaded');
// saveState/restoreState (§16.1) walk the same ledger as the file, over each row's
// `state` list. This is the check that list is complete: load a scene, run it, press
// R, and every field of every body -- including the derived ones the snapshot does
// not carry and the restore has to recompute -- must be back where it started. A
// state field missing from the ledger shows up here as a field that did not return.
for(const ex of EXAMPLES){
  run(`loadExample(${JSON.stringify(ex)})`);
  run('saveState()');
  const at0 = run(STATE), bath0 = run('sim.bathQ');
  run('projectPositions(20)');
  for(let i=0;i<240;i++) run('substep(sim.h)');
  const moved = run(STATE)!==at0;
  run('restoreState()');
  const back = run(STATE), bathBack = run('sim.bathQ');
  const A=JSON.parse(at0), B=JSON.parse(back);
  let worst=0, where='';
  for(let i=0;i<A.length;i++) for(let j=0;j<A[i].length;j++){
    const d=Math.abs(A[i][j]-B[i][j]);
    if(d>worst){ worst=d; where=`body ${i}, field ${j}: ${A[i][j]} -> ${B[i][j]}`; }
  }
  const bathOk = bath0===bathBack;
  // a scene that never moves would pass trivially; say so rather than counting it
  ok(ex.padEnd(12)+(moved?'restores exactly':'is static (nothing to restore)'),
     worst===0 && bathOk, bathOk ? where : `sim.bathQ ${bath0} -> ${bathBack}`);
}

function firstDiff(a,b){
  if(a===b) return '';
  const A=String(a).split('\n'), B=String(b).split('\n');
  for(let i=0;i<Math.max(A.length,B.length);i++)
    if(A[i]!==B[i]) return `first difference at line ${i+1}:\n        -  ${A[i]}\n        +  ${B[i]}`;
  return 'lengths differ';
}

console.log('\n8. a joint with extra control points round-trips');
// The `pt` key is the format's one repeatable field, and each token carries a whole
// point (constraints.js §06.2c). This is the check that all four kinds that take
// them write what they hold and read back the same thing -- including the captured
// station and rest angle, which the reader must NOT recompute.
const MULTI = [
  'scene 3',
  'sim gravity=off',
  'body 1 x=0 y=0 r=0.2',
  'body 2 x=2 y=0 r=0.2',
  'body 3 x=1 y=0 r=0.2',
  'body 4 x=3 y=0 r=0.2',
  'body 5 x=1 y=-1 r=0.4',
  'rod 1 -- 2 len=2 pt=3/s=-1',
  'slot 1 -- 2 pt=4/lock/restAng=0',
  'pin 1@(0.1,0) -- 2@(-1.9,0) pt=3@(-0.9,0)',
  'rack 1 -- 2 weld=A restAngA=0 pt=5/pinion pt=4/s=-3',
].join('\n')+'\n';
{
  let err=null, t1=null, t2=null;
  try { run(`importScene(${JSON.stringify(MULTI)})`); t1=run('exportScene()');
        run(`importScene(${JSON.stringify(t1)})`);   t2=run('exportScene()'); }
  catch(e){ err=e; }
  ok('a multi-point scene loads and re-exports', !err, err&&(err.stack||String(err)));
  if(!err){
    ok('and round-trips byte-for-byte', t1===t2, firstDiff(t1,t2));
    const shape = run(`JSON.stringify(constraints.map(c=>[c.type, conPoints(c).map(
      pt=>[pt.ep.id, pt.kind, pt.s===undefined?null:pt.s, pt.lock])]))`);
    ok('every point came back with its kind, station and lock',
       shape==='[["rod",[[3,"point",-1,false]]],["slot",[[4,"point",null,true]]],'
             +'["pin",[[3,"point",null,false]]],["rack",[[5,"pinion",null,false],[4,"point",-3,false]]]]',
       shape);
    for(const line of ['rod 1 -- 2 len=2 pt=3/s=-1',
                       'slot 1 -- 2 pt=4/lock/restAng=0',
                       'pin 1@(0.1,0) -- 2@(-1.9,0) pt=3@(-0.9,0)',
                       'rack 1 -- 2 weld=A restAngA=0 pt=5/pinion pt=4/s=-3'])
      ok(`the file writes back  ${line}`, t1.split('\n').includes(line),
         `not found in:\n${t1.split('\n').map(l=>'        '+l).join('\n')}`);
  }
}

console.log(`\n${pass} ok, ${fail} failed\n`);
process.exit(fail?1:0);
