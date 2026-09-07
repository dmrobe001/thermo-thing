// Posable rods -- the pose-time release (constraints.js §06.2d).
//
// Like multipoint-check.js and scene-roundtrip.js beside it, this one LOADS the
// simulator: the claim is about the real rows, the real projection and the real
// drag path, so a reimplementation would test nothing. What it asserts:
//
//   1. a plain rod holds its length through a pose drag; a posable one does not --
//      the dragged body slides along the bar and the rod keeps the length it was
//      posed to, rather than snapping back when the drag ends.
//   2. the release drops the WELDS too: a body welded to a posable rod turns freely
//      while it is dragged, which is what "all joined bodies pinned" means.
//   3b. it reaches exactly as far as the hand: a posable rod is released only when it
//      names the dragged body among its own ends, and one further out stays rigid.
//   3. an extra control point rides the released bar as a slot's rider does -- held
//      on the line, free to slide off its station -- and holds its new station once
//      the rod is rigid again.
//   4. a posable rod grounding a body releases that body for the drag and grounds it
//      again, at the posed geometry, the moment the drag is over.
//   5. the scene the drag leaves behind is SATISFIED -- so it becomes the reset
//      baseline (transport.js §16.1) instead of reading as a violation.
//   6. none of this reaches the running physics: a posable rod and a plain one
//      integrate the same trajectory, step for step.
//   7. `posable` round-trips through the scene file, and defaults to off.
//   8. the canvas shows a rod as a rail exactly while it is released, and never
//      otherwise -- a posable rod nobody is dragging is drawn as the rigid rod it is.
const fs=require('fs'), path=require('path'), vm=require('vm');
const ROOT=path.join(__dirname,'..');

// The TOOL layer is loaded (as multipoint-check.js loads it) because §1-§5 below are
// claims about the drag path, not just about the rows: poseDragTo (§13.6) is what
// opens the posing scope, and driving it is the only honest way to check that.
const stubEl = () => new Proxy({}, { get:(t,k)=>
  k==='getContext'       ? ()=>new Proxy({},{get:()=>()=>{}}) :
  k==='classList'        ? {add(){},remove(){},toggle(){}} :
  k==='querySelectorAll' ? ()=>[] :
  k==='style' || k==='dataset' ? {} :
  k in t ? t[k] : ()=>{},
  set:(t,k,v)=>{ t[k]=v; return true; } });
const ctx = vm.createContext({
  document:{ getElementById:()=>stubEl(), createElement:()=>stubEl(),
             querySelectorAll:()=>[], addEventListener(){} },
  window:{addEventListener(){}, devicePixelRatio:1}, performance:{now:()=>0}, console,
  Math, JSON, Number, String, Object, Array, Map, Set, Error,
  requestAnimationFrame:()=>{}, setTimeout:()=>{},
});
ctx.globalThis = ctx;
for(const f of ['js/state.js','js/expr.js','js/geometry.js','js/constraints.js','js/solver.js',
                'js/physics.js','js/projection.js','js/loop.js','js/render.js','js/hud.js',
                'js/tools.js','js/inspector.js','js/examples.js','js/scene.js','js/select.js','js/transport.js'])
  vm.runInContext(fs.readFileSync(path.join(ROOT,f),'utf8'), ctx, {filename:f});
const run = s => vm.runInContext(s, ctx);

let pass=0, fail=0;
const ok=(name,good,detail)=>{ good?pass++:fail++;
  console.log((good?'  ok  ':'  FAIL'), name, good?'':('\n        '+detail)); };
const near=(a,b,tol)=>Math.abs(a-b)<=tol;

// A pose drag, driven the way the pointer handlers drive it: grab a body at a world
// point, follow the cursor in small steps (§13.6 takes each pointermove as one
// increment, and the projection's delta form wants them small), then let go.
run(`var poseDrag = (id, from, to, steps) => {
  const bi = bodyIndex(id);
  drag = { bi, off: localOff(bi, from[0], from[1]) };
  beginPosing(id);                     // §13.5 does, alongside setting drag
  const n = steps || 20;
  for(let i=1;i<=n;i++){
    poseDragTo(from[0]+(to[0]-from[0])*i/n, from[1]+(to[1]-from[1])*i/n);
    saveState();                       // §13.6 does, after every move
  }
  drag = null; endPosing();            // §13.7 does, alongside clearing drag
};`);
// The world-space cursor cap (§05.4) is a SCREEN-space distance, so the camera has
// to be somewhere sane for a drag to reach where it is aimed.
run(`sim.running=false; cam.scale=64;`);

// Ground -- rod -- body, the anchoring arrangement the rod tool builds by default,
// with the far end left as a pin so the body swings. `posable` is the only thing
// that differs between the two worlds below.
const BENCH = posable => `(()=>{ clearScene(); sim.gravity=false; cam.scale=64;
  const b=makeBody(1,0,0.2); bodies.push(b);
  constraints.push(makeRodCon({id:null,off:[0,0]}, {id:b.id,off:[0,0]}, true, false, ${posable}));
  refreshFrozen(); saveState(true); })()`;

console.log('\n1. a posable rod is a rail while it is dragged: the length follows');
run(BENCH(false));
run(`poseDrag(1, [1,0], [2,0])`);
const rigidLen = +run(`constraints[0].len`), rigidX = +run(`bodies[0].x`);
ok('a plain rod holds its length -- the body cannot be pulled outward',
   near(rigidLen,1,1e-9) && near(Math.hypot(rigidX, run('bodies[0].y')),1,1e-6),
   `len ${rigidLen}, |r| ${Math.hypot(rigidX, run('bodies[0].y'))}`);

run(BENCH(true));
run(`poseDrag(1, [1,0], [2,0])`);
const posLen = +run(`constraints[0].len`), posX = +run(`bodies[0].x`);
ok('a posable one slides: the body reaches the cursor', near(posX,2,4e-3), `x ${posX}`);
ok('and the rod ends up at the length it was posed to', near(posLen,2,4e-3), `len ${posLen}`);
ok('with the rod rigid again -- a second drag off the line still holds the new length',
   (()=>{ run(`drag={bi:0, off:localOff(0,2,0)};`);
          // one step only: enough to see the distance row bite, not enough to matter
          run(`sim.running=false;`);
          run(`(()=>{ const G=bodies[0]; const q0=poseSnapshot();
                const temp={type:'dragpin', a:{id:G.id, off:[0,0]}, world:[2,0.5]};
                projectPositions(8,[temp]); })()`);
          const L=Math.hypot(run('bodies[0].x'), run('bodies[0].y'));
          run(`drag=null`);
          return near(L, posLen, 1e-3); })(),
   'the rod did not hold its recaptured length outside the drag');

console.log('\n2. the release drops the welds: joined bodies are pinned, not welded');
// Both ends welded, so the body's angle is locked to the rod's direction when the
// rod is rigid. Dragged sideways, a released rod must let it keep its own angle.
const WELDED = posable => `(()=>{ clearScene(); sim.gravity=false; cam.scale=64;
  const b=makeBody(1,0,0.2); bodies.push(b);
  constraints.push(makeRodCon({id:null,off:[0,0]}, {id:b.id,off:[0,0]}, true, true, ${posable}));
  refreshFrozen(); saveState(true); })()`;
run(WELDED(false));
const wasStatic = run(`bodies[0].static===true`);
ok('a double-welded rod grounds its far end when nothing is being dragged', wasStatic);
run(WELDED(true));
ok('and a posable one does too, until the drag starts', run(`bodies[0].static===true`));
ok('inside a posing gesture on its own body it grounds nothing',
   run(`(()=>{ beginPosing(1); const r=withPosing(()=>{ refreshFrozen(); return bodies[0].static===false; });
        endPosing(); return r; })()`));
run(`refreshFrozen()`);

// A body pinned to ground by one welded-background rod, dragged around the anchor:
// released, it must not be turned by the rod's own swing.
run(`(()=>{ clearScene(); sim.gravity=false; cam.scale=64;
  const b=makeBody(1,0,0.2); bodies.push(b);
  constraints.push(makeRodCon({id:null,off:[0,0]}, {id:b.id,off:[0,0]}, false, true, true));
  refreshFrozen(); saveState(true); })()`);
run(`poseDrag(1, [1,0], [0,1])`);
const thPosable = +run(`bodies[0].th`);
ok('a welded far end does not turn while the rod is released', near(thPosable,0,1e-6),
   `th ${thPosable}`);
ok('and the weld is recaptured, so the pose holds when the rod is rigid again',
   near(+run(`conMaxC(constraints[0])`), 0, 1e-9), run(`String(conMaxC(constraints[0]))`));

// Round the anchor and past the -x direction, which is where the segment angle's
// own branch cut sits (§06.1 unwraps phi against _phiRef precisely because of it).
// A recapture there has to leave the weld agreeing with the row that reads it --
// now, and again after a Reset, which throws the unwrapping anchor away.
run(`(()=>{ clearScene(); sim.gravity=false; cam.scale=64;
  const b=makeBody(1,0,0.2); bodies.push(b);
  constraints.push(makeRodCon({id:null,off:[0,0]}, {id:b.id,off:[0,0]}, false, true, true));
  refreshFrozen(); saveState(true); })()`);
run(`poseDrag(1, [1,0], [0,1], 20); poseDrag(1, [0,1], [-1,-0.15], 30);`);
ok('a pose swung past the branch cut leaves the weld satisfied',
   near(+run(`conMaxC(constraints[0])`), 0, 1e-9), run(`String(conMaxC(constraints[0]))`));
run(`saveState(); restoreState();`);
ok('and still satisfied after a Reset throws the unwrapping anchor away',
   near(+run(`conMaxC(constraints[0])`), 0, 1e-9), run(`String(conMaxC(constraints[0]))`));

console.log('\n3. an extra control point rides the released bar');
// Bar from ground to a free end, with a third body attached halfway along it. Drag
// the RIDER outward: on a rigid rod its station holds it, on a released one it
// slides along the line and stays on it.
const BAR = posable => `(()=>{ clearScene(); sim.gravity=false; cam.scale=64;
  const a=makeBody(0,0,0.2); bodies.push(a);
  const b=makeBody(2,0,0.2); bodies.push(b);
  const c=makeBody(1,0,0.2); bodies.push(c);
  constraints.push(makeRodCon({id:a.id,off:[0,0]}, {id:b.id,off:[0,0]}, false, false, ${posable}));
  constraints.push(makeRodCon({id:null,off:[0,0]}, {id:a.id,off:[0,0]}, true, true));
  makeConPoint(constraints[0], {id:c.id, off:[0,0]}, {});
  refreshFrozen(); saveState(true); })()`;
run(BAR(true));
const s0 = +run(`conPoints(constraints[0])[0].s`);
run(`poseDrag(3, [1,0], [1.5,0])`);
const rider = JSON.parse(run(`JSON.stringify([bodies[2].x, bodies[2].y, conPoints(constraints[0])[0].s])`));
ok('the rider slides along the bar instead of dragging it', near(rider[0],1.5,4e-3), `x ${rider[0]}`);
ok('and never leaves the line', near(rider[1],0,1e-6), `y ${rider[1]}`);
ok('its station is recaptured at the new place', !near(rider[2],s0,1e-3) && near(rider[2],-1.5,4e-3),
   `s ${s0} -> ${rider[2]}`);
run(BAR(false));
run(`poseDrag(3, [1,0], [1.5,0])`);
ok('on a plain rod the rider cannot slide at all -- its station holds it',
   near(+run(`conPoints(constraints[0])[0].s`), s0, 1e-9) && near(+run(`bodies[2].x`), 1, 1e-3),
   run(`JSON.stringify([conPoints(constraints[0])[0].s, bodies[2].x])`));

console.log('\n3b. the release reaches exactly as far as the hand does');
// Two posable rods, only one of them jointed to the body under the cursor. A drag on
// body 1 must release the rod that names it and leave the other one rigid -- length,
// welds and all -- however posable it is.
run(`(()=>{ clearScene(); sim.gravity=false; cam.scale=64;
  bodies.push(makeBody(1,0,0.2));                       // 1: dragged
  bodies.push(makeBody(2.5,0,0.2));                     // 2: one joint further out
  constraints.push(makeRodCon({id:null,off:[0,0]}, {id:1,off:[0,0]}, true, false, true));
  constraints.push(makeRodCon({id:1,off:[0,0]}, {id:2,off:[0,0]}, false, false, true));
  refreshFrozen(); saveState(true); })()`);
run(`var rowsUnder = (rootId, ci) => { beginPosing(rootId);
  const n = withPosing(()=>rowsFor(constraints[ci]).length); endPosing(); return n; };`);
ok('a rod naming the dragged body is released',
   run(`rowsUnder(1,0)===0`), run(`String(rowsUnder(1,0))`));
ok('and so is one that names it as its FAR end', run(`rowsUnder(1,1)===0`));
// Rigid, that rod is two rows: its distance, and the weld on its background end.
ok('a drag on the far body leaves the ground rod rigid',
   run(`rowsUnder(2,0)===2`), run(`String(rowsUnder(2,0))`));
// Three in a row, so there is a posable rod two joints away from the grab.
run(`(()=>{ clearScene(); sim.gravity=false; cam.scale=64;
  bodies.push(makeBody(1,0,0.2)); bodies.push(makeBody(2,0,0.2)); bodies.push(makeBody(3,0,0.2));
  constraints.push(makeRodCon({id:null,off:[0,0]}, {id:1,off:[0,0]}, true, false, true));
  constraints.push(makeRodCon({id:2,off:[0,0]}, {id:3,off:[0,0]}, false, false, true));
  refreshFrozen(); saveState(true); })()`);
const farLen0 = +run(`constraints[1].len`);
run(`poseDrag(1, [1,0], [1.7,0])`);
ok('a posable rod the drag never reached keeps its length',
   near(+run(`constraints[1].len`), farLen0, 1e-12),
   `${farLen0} -> ${run('String(constraints[1].len)')}`);
ok('and its bodies did not move',
   near(+run(`bodies[1].x`),2,1e-12) && near(+run(`bodies[2].x`),3,1e-12),
   run(`JSON.stringify([bodies[1].x, bodies[2].x])`));

console.log('\n4. a posable ground strut can be posed, and re-grounds where it lands');
run(WELDED(true));
run(`poseDrag(1, [1,0], [1.6,0.4])`);
const landed = JSON.parse(run(`JSON.stringify([bodies[0].x, bodies[0].y, bodies[0].static, constraints[0].len])`));
ok('the grounded body moved to where it was dragged',
   near(landed[0],1.6,4e-3) && near(landed[1],0.4,4e-3), JSON.stringify(landed));
ok('and is grounded again the moment the drag is over', landed[2]===true, JSON.stringify(landed));
ok('at the length the pose implies', near(landed[3], Math.hypot(1.6,0.4), 6e-3), `len ${landed[3]}`);

console.log('\n5. the posed scene is satisfied, so it becomes the reset baseline');
ok('every constraint holds within the drift tolerance', run(`constraintsSatisfied()`),
   run(`String(constraints.map(conMaxC))`));
run(`saveState(); bodies[0].x=9; bodies[0].y=9; restoreState();`);
ok('Reset puts back the posed pose, not the authored one',
   near(+run(`bodies[0].x`),1.6,4e-3) && near(+run(`bodies[0].y`),0.4,4e-3),
   run(`JSON.stringify([bodies[0].x, bodies[0].y])`));

console.log('\n6. the flag never reaches the running physics');
// Same machine twice, differing only in `posable`, integrated with no drag in sight.
const PENDULUM = posable => `(()=>{ clearScene(); sim.gravity=true; sim.g=9.81;
  const b=makeBody(1,0,0.2); bodies.push(b);
  constraints.push(makeRodCon({id:null,off:[0,0]}, {id:b.id,off:[0,0]}, true, false, ${posable}));
  refreshFrozen(); saveState(true); sim.running=true;
  for(let i=0;i<360;i++) substep(sim.h);
  sim.running=false;
  return JSON.stringify([bodies[0].x, bodies[0].y, bodies[0].th, bodies[0].vx, bodies[0].vy]); })()`;
const plainRun = run(PENDULUM(false)), posableRun = run(PENDULUM(true));
ok('three seconds of a posable pendulum is bit-identical to the plain one',
   plainRun===posableRun, `${plainRun}\n        ${posableRun}`);

console.log('\n7. the scene file carries it');
const FILE = ['scene 3','sim gravity=off',
              'body 1 x=1 y=0 r=0.2',
              'rod bg(0,0) -- 1 len=1 weld=A posable restAngA=0'].join('\n')+'\n';
{
  let err=null, t1=null, t2=null;
  try { run(`importScene(${JSON.stringify(FILE)})`); t1=run('exportScene()');
        run(`importScene(${JSON.stringify(t1)})`);   t2=run('exportScene()'); }
  catch(e){ err=e; }
  ok('a posable rod loads and re-exports', !err, err&&(err.stack||String(err)));
  if(!err){
    ok('the flag survived the read', run(`constraints[0].posable===true`));
    ok('and round-trips byte-for-byte', t1===t2, firstDiff(t1,t2));
    ok('the file writes back  rod bg(0,0) -- 1 len=1 weld=A posable restAngA=0',
       t1.split('\n').includes('rod bg(0,0) -- 1 len=1 weld=A posable restAngA=0'),
       t1.split('\n').map(l=>'        '+l).join('\n'));
  }
}
// Authored fields are written only when they differ from their default, so a rod
// nobody marked must not gain a key -- that is what keeps a default scene terse.
run(`importScene(${JSON.stringify(['scene 3','sim gravity=off','body 1 x=1 y=0 r=0.2',
                                   'rod bg(0,0) -- 1 len=1 weld=A restAngA=0'].join('\n')+'\n')})`);
ok('an unmarked rod defaults to off and writes no key',
   run(`constraints[0].posable===false`) && !run('exportScene()').includes('posable'),
   run('exportScene()'));

console.log('\n8. the canvas shows the rail only while the rod is one');
run(`(()=>{ clearScene(); sim.gravity=false; cam.scale=64;
  const b=makeBody(1,0,0.2); bodies.push(b);
  constraints.push(makeRodCon({id:null,off:[0,0]}, {id:b.id,off:[0,0]}, true, false, true));
  refreshFrozen(); selectConstraint(0); cancelSingle(); })()`);
ok('a posable rod nobody is dragging draws as a plain rod', run(`rodPosing(constraints[0])===false`));
run(`beginPosing(1)`);
ok('and as a rail for as long as the gesture on its body lasts', run(`rodPosing(constraints[0])===true`));
ok('but its ROWS are only released inside the scope the solver runs in',
   run(`rowsFor(constraints[0]).length===2 && withPosing(()=>rowsFor(constraints[0]).length)===0`),
   run(`JSON.stringify([rowsFor(constraints[0]).length, withPosing(()=>rowsFor(constraints[0]).length)])`));
run(`endPosing()`);
ok('the gesture ending puts the rail away', run(`rodPosing(constraints[0])===false`));
run(`render(); renderInspector(); updateInspectorLive();`);
ok('render and the inspector run clean over a posable rod', true);
// render.js loads BEFORE tools.js: anything it calls that lives in the tool layer is
// a forward reference, and one that fails to resolve throws inside render(), which
// kills the §10 rAF chain outright -- the page stops redrawing for good. Everything
// the canvas needs about a released rod therefore lives in §06.2d.
ok('the canvas reads the release from the constraint layer, not the tool layer',
   !fs.readFileSync(path.join(ROOT,'js/render.js'),'utf8').includes('poseDragRoot'));

function firstDiff(a,b){
  if(a===b) return '';
  const A=String(a).split('\n'), B=String(b).split('\n');
  for(let i=0;i<Math.max(A.length,B.length);i++)
    if(A[i]!==B[i]) return `first difference at line ${i+1}:\n        -  ${A[i]}\n        +  ${B[i]}`;
  return 'lengths differ';
}

console.log(`\n${pass} ok, ${fail} failed\n`);
process.exit(fail?1:0);
