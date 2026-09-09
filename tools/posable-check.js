// A POSABLE line (constraints.js §06.2d/§06.2f) -- the pose-time release.
//
// While the player drags a body with the sim PAUSED, a line marked `posable` and
// touching that body holds nothing for the length of the gesture: every joint slides,
// nothing is welded, and it grounds nothing. It is rigid again the moment the drag
// step is over, at the geometry the drag reached. What this asserts:
//
//   1. a plain bar holds its held distances through a pose drag; a posable one does
//      not -- the dragged body slides along it, and the bar keeps what it was posed
//      to once the drag is over.
//   2. the release drops the WELDS too: a body welded to a posable bar turns freely
//      while it is dragged.
//   3. it reaches exactly as far as the hand: a posable line is released only when
//      the dragged body is one it touches, and a posable line nobody is dragging is
//      never released at all.
//   4. a posable bar GROUNDING a body releases that body for the drag and grounds it
//      again where it lands -- a ground strut that still pinned the body it is meant
//      to slide along would be a contradiction.
//   5. between drag steps the bar is rigid, so nothing reads as violated: the pose
//      the drag leaves is a satisfied one, and becomes the reset baseline.
//   6. none of it reaches the running physics: a posable bar and a plain one integrate
//      identically.
//   7. `posable` round-trips through the scene file, and defaults to off.
//   8. the canvas shows the bar as a rail exactly while it is released.
const fs=require('fs'), path=require('path'), vm=require('vm');
const ROOT=path.join(__dirname,'..');
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

// Ground -- BAR -- body: the anchoring arrangement the line tool builds, ground joint
// welded so it is a rigid strut, far joint free so the body swings. `posable` is the
// only thing that differs between the two benches.
run(`var theLine = () => constraints.find(isLine);
var span = () => { const f=lineFrame(theLine()); return Math.hypot(f.wax-f.wbx, f.way-f.wby); };
var bar = (specs, posable)=>{
  const L=makeLine(); L.posable=!!posable; constraints.push(L);
  const vs=[];
  for(const [ep,o] of specs){
    const v=makeVertex(null); makeVertexOn(v,ep,{join:true}); constraints.push(v);
    makeVertexOn(v,{id:L.id},{join:true, slide:!!o.slide}); vs.push([v,o]); }
  for(const [v,o] of vs){ if(!o.weld) continue;
    for(const e of vertexOns(v)) setVertexWeld(v,e,true); }
  refreshFrozen(); return L; };
// A pose drag, exactly as the tool layer runs it (§13.6 poseDragTo): a gesture is a
// RUN of pointermoves, and the projection's soft drag row closes over the run, so a
// check that teleports in one jump is testing something the player never does.
var poseTo = (bi, wx, wy) => {
  drag={bi, off:[0,0]}; beginPosing(bodies[bi].id);
  const x0=bodies[bi].x, y0=bodies[bi].y;
  for(let i=1;i<=12;i++) poseDragTo(x0+(wx-x0)*i/12, y0+(wy-y0)*i/12);
  endPosing(); drag=null; };`);

const BENCH = posable => `(()=>{ clearScene(); sim.gravity=false; cam.scale=64;
  const b=makeBody(1,0,0.2); bodies.push(b);
  bar([[{id:null,off:[0,0]},{slide:false,weld:true}],
       [{id:b.id,off:[0,0]},{slide:false}]], ${posable}); })()`;

console.log('\n1. a posable bar is a rail while it is dragged');
run(BENCH(false)); run(`poseTo(0, 2, 0)`);
const rigidLen=+run('span()'), rigidX=+run('bodies[0].x');
ok('a plain bar holds its distance -- the body cannot be pulled outward',
   near(rigidLen,1,1e-6) && rigidX<1.2, `span ${rigidLen} x ${rigidX}`);
run(BENCH(true)); run(`poseTo(0, 2, 0)`);
const posLen=+run('span()'), posX=+run('bodies[0].x');
ok('a posable one slides: the body reaches the cursor', near(posX,2,4e-3), `x ${posX}`);
ok('and the bar ends up at the distance it was posed to', near(posLen,2,4e-3), `span ${posLen}`);
// Rigid again BETWEEN drags: the distance the pose left is what the running sim then
// holds. (Another drag would release it again -- that is what marking it posable
// means -- so the claim is about what happens when the hand lets go.)
run(`(()=>{ bodies[0].vy=0.6; setRunning(true); for(let i=0;i<240;i++) substep(sim.h); })()`);
ok('rigid again once the hand lets go -- it holds what the pose left',
   near(+run('span()'), posLen, 1e-3), `${run('String(span())')} vs ${posLen}`);

console.log('\n2. the release drops the welds too');
run(`(()=>{ clearScene(); sim.gravity=false; cam.scale=64;
  const b=makeBody(1,0,0.2); bodies.push(b);
  bar([[{id:null,off:[0,0]},{slide:false,weld:true}],
       [{id:b.id,off:[0,0]},{slide:false,weld:true}]], true); })()`);
run(`poseTo(0, 1.4, 1.4)`);
ok('a body welded to a posable bar turns freely while it is dragged',
   Math.abs(+run('bodies[0].th'))>1e-6 || Math.abs(+run('bodies[0].y')-1.4)<0.05,
   run('JSON.stringify([bodies[0].x,bodies[0].y,bodies[0].th])'));

console.log('\n3. it reaches exactly as far as the hand');
run(`(()=>{ clearScene(); sim.gravity=false; cam.scale=64;
  const a=makeBody(1,0,0.2); bodies.push(a);
  const c=makeBody(3,0,0.2); bodies.push(c);
  bar([[{id:null,off:[0,0]},{slide:false,weld:true}],[{id:a.id,off:[0,0]},{slide:false}]], true);
  const far = bar([[{id:a.id,off:[0,0]},{slide:false}],[{id:c.id,off:[0,0]},{slide:false}]], true);
  refreshFrozen(); })()`);
const farBefore=+run(`(()=>{ const L=constraints.filter(isLine)[1], f=lineFrame(L);
  return Math.hypot(f.wax-f.wbx,f.way-f.wby); })()`);
ok('a posable line the drag never reached is not released', run(`(()=>{
  // Drag the FAR body: the second bar names it, the first does not.
  const near_=constraints.filter(isLine)[0], far=constraints.filter(isLine)[1];
  drag={bi:1, off:[0,0]}; beginPosing(bodies[1].id);
  const r = withPosing(()=>[lineReleased(near_), lineReleased(far)]);
  endPosing(); drag=null;
  return r[0]===false && r[1]===true; })()`)===true,
  'the release must reach exactly as far as the hand');
ok('and with nothing being dragged, nothing is released', run(`(()=>{
  return constraints.filter(isLine).some(L=>linePosing(L)); })()`)===false);

console.log('\n4. a posable ground strut releases the body it grounds');
run(`(()=>{ clearScene(); sim.gravity=false; cam.scale=64;
  const b=makeBody(1,0,0.2); bodies.push(b);
  bar([[{id:null,off:[0,0]},{slide:false,weld:true}],
       [{id:b.id,off:[0,0]},{slide:false,weld:true}]], true); })()`);
ok('grounded before the drag', run('bodies[0].static')===true);
ok('...free inside it', run(`(()=>{ drag={bi:0,off:[0,0]}; beginPosing(bodies[0].id);
  const st = withPosing(()=>{ refreshFrozen(); return bodies[0].static; });
  endPosing(); drag=null; refreshFrozen(); return st; })()`)===false);
run(`poseTo(0, 2.5, 0)`);
// The drag is a SOFT pull (§09.1), so where it lands is near the cursor rather than
// exactly on it; what matters is that it moved there and is pinned again.
ok('...and grounded again where it lands', run('bodies[0].static')===true
   && near(+run('bodies[0].x'), 2.5, 2e-2), run('JSON.stringify([bodies[0].static,bodies[0].x])'));

console.log('\n5. the pose it leaves is a satisfied one');
ok('nothing reads as violated after a drag', near(+run('Math.max(...constraints.map(conMaxC))'), 0, 1e-6),
   run('String(Math.max(...constraints.map(conMaxC)))'));
ok('so it becomes the reset baseline', run(`constraintsSatisfied()`)===true);

console.log('\n6. none of it reaches the running physics');
{
  const traj = posable => { run(BENCH(posable));
    return run(`(()=>{ bodies[0].vy=0.7; setRunning(true);
      for(let i=0;i<240;i++) substep(sim.h);
      return JSON.stringify([bodies[0].x,bodies[0].y,bodies[0].th]); })()`); };
  ok('a posable bar and a plain one integrate identically', traj(true)===traj(false),
     traj(true)+' vs '+traj(false));
}

console.log('\n7. the file');
{
  const T=['scene 5','sim gravity=off','cam x=0 y=0 scale=64','body 1 x=1 y=0 r=0.2',
           'line 2 posable','vertex A on=bg(0,0)/join on=2/join/fix/s=0',
           'vertex B on=1/join on=2/join/fix/s=-1'].join('\n')+'\n';
  run(`importScene(${JSON.stringify(T)})`);
  ok('posable round-trips', run('theLine().posable')===true);
  ok('and the export writes it', run('exportScene()').includes('line 2 posable'));
  run(`importScene(${JSON.stringify(T.replace(' posable',''))})`);
  ok('...and defaults to off', run('theLine().posable')===false);
}

console.log('\n8. the canvas shows it as a rail exactly while released');
run(BENCH(true));
ok('not while nothing is dragged', run('linePosing(theLine())')===false);
ok('yes while its own body is', run(`(()=>{ drag={bi:0,off:[0,0]}; beginPosing(bodies[0].id);
  const r=linePosing(theLine()); endPosing(); drag=null; return r; })()`)===true);

console.log(`\n${pass} ok, ${fail} failed\n`);
process.exit(fail?1:0);
