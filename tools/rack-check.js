// Rack and pinion, and rolling rows under the position projection.
//
// Like scene-roundtrip.js beside it (and unlike the vessel-check scripts), this one
// LOADS the simulator: the claims are about the real rows and the real projection,
// so a reimplementation would test nothing. What it asserts:
//
//   1. the rack rides its body's frame -- turn the body and the rack's world
//      heading turns with it, which is the whole difference between a rack welded
//      to a part and a direction fixed in the world.
//   2. the row it builds is the right one: no slip at the mesh while running, and
//      a pinion spinning at (rack speed / pitch radius).
//   3. a PAUSED drag articulates the pair. Rolling rows have no position invariant,
//      so the projection enforces them on the position DELTA of the edit instead
//      (projection.js §09.1) -- this is the check that the delta form actually
//      rolls, and rolls by the right amount.
//   4. the same now holds for the CVT, including the case that must NOT roll:
//      sliding a ball-and-disk follower radially re-ratios the pair and turns
//      nothing, because the row's translation columns are tangential.
//   5. an off-centre anchor couples the rack body's SPIN to the pinion -- the
//      -(r.n) angular column, which a rack through the body's centre never exercises.
const fs=require('fs'), path=require('path'), vm=require('vm');
const ROOT=path.join(__dirname,'..');

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
vm.runInContext(`function clearSelection(){} function renderInspector(){}
                 function setTool(){} var TOOLS=[];`, ctx);
for(const f of ['js/state.js','js/geometry.js','js/constraints.js','js/solver.js',
                'js/physics.js','js/projection.js','js/loop.js','js/hud.js','js/scene.js',
                'js/examples.js','js/transport.js'])
  vm.runInContext(fs.readFileSync(path.join(ROOT,f),'utf8'), ctx, {filename:f});
const run = s => vm.runInContext(s, ctx);

let pass=0, fail=0;
const ok=(name,good,detail)=>{ good?pass++:fail++;
  console.log((good?'  ok  ':'  FAIL'), name, good?'':('\n        '+detail)); };
const near=(a,b,tol)=>Math.abs(a-b)<=tol;

console.log('\n1. the rack rides its body\'s frame (not the world)');
run(`loadExample('rack')`);
// world heading of the rack, before and after turning the rack's body
const head0 = run(`rackFrame(constraints.find(c=>c.type==='rack')).ang`);
run(`bodies[0].th = 0.5;`);
const head1 = run(`rackFrame(constraints.find(c=>c.type==='rack')).ang`);
ok('rack heading = body angle + local angle', near(head0,0,1e-12) && near(head1,0.5,1e-12),
   `got ${head0} then ${head1}, expected 0 then 0.5`);
run(`bodies[0].th = 0;`);

console.log('\n2. the row is satisfied while running, at the right ratio');
run(`loadExample('rack'); saveState(); projectPositions(20);`);
// pitch radius is the pinion's perpendicular distance to the rack line
const rho = run(`rackFrame(constraints.find(c=>c.type==='rack')).rho`);
ok('pitch radius = 1.0 (signed -1: pinion below the rack)', near(Math.abs(rho),1.0,1e-9),
   `got ${rho}`);
for(let i=0;i<120;i++) run('substep(sim.h)');
const st = run(`(()=>{const c=constraints.find(x=>x.type==='rack');
  const f=rackFrame(c); const A=bodies[0], B=bodies[1];
  // residual of the row itself: rack material speed along u minus the pinion's
  const rackV = A.vx*f.ux + A.vy*f.uy;      // anchor at A's centre, so no w term
  const pinV  = B.vx*f.ux + B.vy*f.uy + B.w*f.rho;
  return {vx:A.vx, w:B.w, rho:f.rho, slip:rackV-pinV, th:A.th};})()`);
ok('no slip at the mesh', near(st.slip,0,1e-9), `slip ${st.slip}`);
ok('pinion spin matches cart speed / pitch radius',
   near(st.w, -st.vx/Math.abs(st.rho), 1e-6), `w=${st.w}, vx=${st.vx}, rho=${st.rho}`);
ok('cart stays level (slot locks its orientation, so the rack stays level)',
   near(st.th,0,1e-9), `th=${st.th}`);

console.log('\n3. a PAUSED drag articulates the pair (the delta-form projection)');
run(`loadExample('rack')`);
const before = run(`JSON.stringify([bodies[0].x, bodies[1].th])`);
// exactly what tools.js §13.6 does for a non-static drag: a soft dragpin goal
// pulling the cart's centre 0.4 to the right, articulated by projectPositions.
run(`(()=>{ const G=bodies[0];
  const temp={type:'dragpin', a:{id:G.id, off:[0,0]}, world:[G.x+0.4, G.y]};
  for(let k=0;k<12;k++) projectPositions(8,[temp]); })()`);
const after = run(`JSON.stringify([bodies[0].x, bodies[1].th])`);
const [x0,t0]=JSON.parse(before), [x1,t1]=JSON.parse(after);
const dx=x1-x0, dth=t1-t0;
ok('the dragged cart actually moved', dx>0.3, `dx=${dx}`);
ok('the pinion rolled while paused', Math.abs(dth)>1e-3, `dth=${dth}`);
ok('and rolled by dx / pitch radius, the right way',
   near(dth, -dx/1.0, 2e-2), `dth=${dth}, expected ${-dx/1.0}`);

console.log('\n4. the same now holds for the CVT (nonholonomic too)');
run(`loadExample('integrator')`);
// Sliding the follower radially is NOT a rolling motion -- the row's translation
// columns are tangential, so radial travel induces no slip. That is the whole
// point of a ball-and-disk integrator: sliding the follower re-ratios the pair
// without turning either body. The rolling DOF is a TURN, so turn the disk.
const slideBefore = run(`JSON.stringify([bodies[0].th, bodies[1].th])`);
run(`(()=>{ const q0=poseSnapshot(); bodies[1].x -= 0.3; projectPositions(8,null,q0); })()`);
const slideAfter = run(`JSON.stringify([bodies[0].th, bodies[1].th])`);
ok('sliding the follower re-ratios without turning anything (as it should)',
   JSON.parse(slideBefore).every((v,i)=>near(v,JSON.parse(slideAfter)[i],1e-9)),
   `${slideBefore} -> ${slideAfter}`);
run(`loadExample('integrator')`);
const cvtBefore = run(`bodies[1].th`);
run(`(()=>{ const q0=poseSnapshot(); bodies[0].th += 0.3; projectPositions(8,null,q0); })()`);
const cvtAfter = run(`bodies[1].th`);
ok('turning the CVT disk now turns the follower while paused',
   Math.abs(cvtAfter-cvtBefore)>1e-3, `follower dth=${cvtAfter-cvtBefore}`);

console.log('\n5. an off-centre rack anchor lets the body\'s SPIN drive the pinion');
// The anchor is on A's rim, so the rack line misses A's centre: turning A now
// sweeps the rack along itself and must roll the pinion (the -(r.n) column).
run(`(()=>{ clearScene();
  const a=makeBody(0,0,0.5); bodies.push(a);
  const b=makeBody(0,-1,0.3); bodies.push(b);
  constraints.push(makeRackCon(a.id,[0,0.5],b.id));   // anchor at A's top
  refreshFrozen(); })()`);
const spinBefore = run(`bodies[1].th`);
run(`(()=>{ const q0=poseSnapshot(); bodies[0].th += 0.2; projectPositions(8,null,q0); })()`);
const spinAfter = run(`bodies[1].th`);
ok('turning the rack body rolls the pinion', Math.abs(spinAfter-spinBefore)>1e-3,
   `dth=${spinAfter-spinBefore}`);

console.log(`\n${pass} ok, ${fail} failed\n`);
process.exit(fail?1:0);
