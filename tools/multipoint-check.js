// Extra control points -- a joint with more than two ends (constraints.js §06.2c).
//
// Like rack-check.js and scene-roundtrip.js beside it, this one LOADS the simulator:
// the claims are about the real rows and the real projection. What it asserts:
//
//   1. a rod's extra point is a RIGID attachment to the bar -- on the line, at its
//      captured station -- and stays one when the bar is moved or turned.
//   2. a locked extra point also turns with the bar, exactly as a welded end does.
//   3. a slot's rider is held on the rail and NOTHING else: it slides freely along.
//   4. a pin's extra end brings a third body to the same pivot.
//   5. a rack's jointed point carries its body along with the rack.
//   6. the lock a new point inherits: all-alike is copied, a mix reads non-rotating.
//   7. every extra point couples its body into the same island as the joint's ends.
//   8. the placement gesture that creates them: clicking an existing joint of the
//      tool's own kind adds a point to it, and on a rack a click near a disk's rim
//      makes that disk a pinion where a click through a body makes it a joint.
//   9. the editing surface holds up -- handles name each point, and deleting a
//      point's body drops the point while deleting an END drops the whole joint.
const fs=require('fs'), path=require('path'), vm=require('vm');
const ROOT=path.join(__dirname,'..');

// Wider than the other scripts' stubs: this one loads the TOOL, RENDER and
// INSPECTOR layers too, because §8 and §9 below are claims about the placement
// gesture and the editing surface, not just about the rows.
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

// The point's own invariants, read back out of the live geometry: how far off the
// line it has drifted, and how far its station has moved from what it captured.
const OFFSETS = `(con,k)=>{
  const f=twoPointFrame(con), pt=conPoints(con)[k];
  const [wx,wy]=epWorld(pt.ep);
  const dx=wx-f.wax, dy=wy-f.way;
  return [f.nx*dx+f.ny*dy, (f.ux*dx+f.uy*dy)-(pt.s||0)];
}`;
run(`var offsets = ${OFFSETS};`);
const offs = (ci,k) => JSON.parse(run(`JSON.stringify(offsets(constraints[${ci}],${k}))`));

console.log('\n1. a rod\'s extra point is a rigid attachment to the bar');
// A bar from a grounded pin to a free end, with a third body hung on it halfway.
run(`(()=>{ clearScene();
  const a=makeBody(0,0,0.2); bodies.push(a);
  const b=makeBody(2,0,0.2); bodies.push(b);
  const c=makeBody(1,0,0.2); bodies.push(c);
  const rod=makeRodCon({id:a.id,off:[0,0]}, {id:b.id,off:[0,0]}, false, false);
  makeConPoint(rod, {id:c.id, off:[0,0]}, {});
  constraints.push(rod); refreshFrozen(); })()`);
ok('the station is captured where the point was placed', near(run(`constraints[0].pts[0].s`), -1, 1e-12),
   `s=${run('constraints[0].pts[0].s')}`);
let [lat,sta]=offs(0,0);
ok('and it starts exactly on the line at that station', near(lat,0,1e-12) && near(sta,0,1e-12),
   `lateral ${lat}, station error ${sta}`);
// Move BOTH ends: the bar swings and lengthens nowhere (the rod holds its length),
// and the third body has to follow it.
const cBefore = run(`JSON.stringify([bodies[2].x, bodies[2].y])`);
run(`(()=>{ const q0=poseSnapshot(); bodies[1].y += 1.2; projectPositions(24,null,q0); })()`);
const cAfter = run(`JSON.stringify([bodies[2].x, bodies[2].y])`);
[lat,sta]=offs(0,0);
ok('the third body moved when the bar did',
   Math.hypot(JSON.parse(cAfter)[0]-JSON.parse(cBefore)[0],
              JSON.parse(cAfter)[1]-JSON.parse(cBefore)[1])>0.1, `${cBefore} -> ${cAfter}`);
ok('and is still on the line, at the same station', near(lat,0,1e-6) && near(sta,0,1e-6),
   `lateral ${lat}, station error ${sta}`);
ok('the constraint reads as satisfied', run(`conMaxC(constraints[0])`)<1e-6,
   `maxC=${run('conMaxC(constraints[0])')}`);

console.log('\n2. a LOCKED extra point turns with the bar, a free one does not');
const freeTh = run(`bodies[2].th`);
ok('an unlocked point left the third body free to keep its own angle',
   near(freeTh, 0, 1e-9), `th=${freeTh}`);
run(`(()=>{ clearScene();
  const a=makeBody(0,0,0.2); bodies.push(a);
  const b=makeBody(2,0,0.2); bodies.push(b);
  const c=makeBody(1,0,0.2); bodies.push(c);
  const rod=makeRodCon({id:a.id,off:[0,0]}, {id:b.id,off:[0,0]}, false, false);
  makeConPoint(rod, {id:c.id, off:[0,0]}, {lock:true});
  constraints.push(rod); refreshFrozen(); })()`);
const phi0 = run(`twoPointFrame(constraints[0]).phi`);
run(`(()=>{ const q0=poseSnapshot(); bodies[1].y += 1.2; projectPositions(24,null,q0); })()`);
const phi1 = run(`twoPointFrame(constraints[0]).phi`);
const lockTh = run(`bodies[2].th`);
ok('the bar actually turned', Math.abs(phi1-phi0)>0.1, `phi ${phi0} -> ${phi1}`);
ok('and the locked body turned with it', near(lockTh, phi1-phi0, 1e-5),
   `dth=${lockTh}, bar turned ${phi1-phi0}`);

console.log('\n3. a slot\'s rider is held on the rail and slides freely along it');
// A rail tilted 45 degrees between two background points, with a rider on it and
// gravity on: the rider must slide DOWN the rail and never leave it.
run(`(()=>{ clearScene();
  const r=makeBody(1,1,0.2); bodies.push(r);
  const slot=makeSlotCon({id:null,off:[0,0]}, {id:null,off:[4,4]}, true, true);
  makeConPoint(slot, {id:r.id, off:[0,0]}, {});
  constraints.push(slot); sim.gravity=true; refreshFrozen(); })()`);
ok('a slot rider holds no station (a rider slides)', run(`constraints[0].pts[0].s===undefined`),
   `s=${run('constraints[0].pts[0].s')}`);
run(`saveState(); projectPositions(20);`);
for(let i=0;i<200;i++) run('substep(sim.h)');
[lat,sta]=offs(0,0);
ok('the rider never left the rail', near(lat,0,1e-6), `lateral ${lat}`);
ok('but it did slide down it', run(`bodies[0].x`)<0.9, `x=${run('bodies[0].x')}`);

console.log('\n4. a pin\'s extra end brings a third body to the same pivot');
run(`(()=>{ clearScene();
  const a=makeBody(0,0,0.3); bodies.push(a);
  const b=makeBody(0.4,0,0.3); bodies.push(b);
  const c=makeBody(0,0.4,0.3); bodies.push(c);
  const pin=makePinCon({id:a.id,off:[0,0]}, {id:b.id,off:[-0.4,0]});
  makeConPoint(pin, {id:c.id, off:[0,-0.4]}, {});
  constraints.push(pin); refreshFrozen(); })()`);
run(`projectPositions(30)`);
const P = run(`JSON.stringify([epWorld(constraints[0].a), epWorld(constraints[0].b),
                               epWorld(constraints[0].pts[0].ep)].map(p=>[p[0],p[1]]))`);
const pts = JSON.parse(P);
ok('all three ends coincide', near(pts[0][0],pts[2][0],1e-6) && near(pts[0][1],pts[2][1],1e-6)
   && near(pts[0][0],pts[1][0],1e-6) && near(pts[0][1],pts[1][1],1e-6), P);

console.log('\n5. a rack\'s jointed point carries its body along with the rack');
// Both pins on a cart moving right, plus a body jointed to the rack a metre away:
// the joint is a rigid attachment to the rack, so it must travel with the cart.
run(`(()=>{ clearScene(); sim.gravity=false;
  const cart=makeBody(0,0,0.3); bodies.push(cart); cart.vx=1;
  const j=makeBody(1,0,0.2); bodies.push(j);
  const c=makeRackCon({id:cart.id,off:[0,0]}, {id:cart.id,off:[-0.5,0]}, false, false);
  makeConPoint(c, {id:j.id, off:[0,0]}, {});
  constraints.push(c); refreshFrozen(); })()`);
run(`saveState(); projectPositions(20);`);
for(let i=0;i<120;i++) run('substep(sim.h)');
const carry = run(`JSON.stringify([bodies[0].x, bodies[1].x, bodies[0].vx, bodies[1].vx])`);
const [cx,jx,cvx,jvx]=JSON.parse(carry);
ok('the cart travelled', cx>0.5, `cart x=${cx}`);
ok('and the jointed body travelled with it, keeping its station',
   near(jx-cx, 1, 1e-6) && near(jvx, cvx, 1e-6), carry);

console.log('\n6. the lock a newly added point inherits');
const inherit = (wa,wb,extra)=>run(`(()=>{ clearScene();
  const a=makeBody(0,0,0.2); bodies.push(a); const b=makeBody(2,0,0.2); bodies.push(b);
  const rod=makeRodCon({id:a.id,off:[0,0]},{id:b.id,off:[0,0]},${wa},${wb});
  ${extra ? `const c=makeBody(1,0,0.2); bodies.push(c);
             makeConPoint(rod,{id:c.id,off:[0,0]},{lock:${extra}});` : ''}
  return conNewPointLock(rod); })()`);
ok('both ends free -> the new point is free',   inherit(false,false,null)===false);
ok('both ends welded -> the new point is welded',inherit(true,true,null)===true);
ok('a mix -> the new point is non-rotating',    inherit(true,false,null)===true);
ok('an existing point counts in the vote too',  inherit(false,false,true)===true,
   'two free ends and one locked point is a mix, so the next point locks');

console.log('\n7. an extra point couples its body into the joint\'s island');
run(`(()=>{ clearScene(); sim.gravity=false;
  const a=makeBody(0,0,0.2); bodies.push(a);
  const b=makeBody(2,0,0.2); bodies.push(b);
  const c=makeBody(1,0,0.2); bodies.push(c);
  const lone=makeBody(9,9,0.2); bodies.push(lone);
  const rod=makeRodCon({id:a.id,off:[0,0]}, {id:b.id,off:[0,0]}, false, false);
  makeConPoint(rod, {id:c.id, off:[0,0]}, {});
  constraints.push(rod); refreshFrozen(); })()`);
const isl = JSON.parse(run(`JSON.stringify(computeIslands().map(i=>i.bodyIdx))`));
ok('the bar and all three of its bodies are one island, the loose body another',
   isl.length===2 && isl.some(g=>g.length===3) && isl.some(g=>g.length===1),
   JSON.stringify(isl));

console.log('\n8. the placement gesture: a click on a joint of the tool\'s own kind');
// The rack is the three-click case AND the one where the click is read two ways, so
// it carries this section. cam.scale matters: the pinion test is a screen-space
// tolerance around the rim (tools.js §13.5 PINION_RIM_PX).
run(`(()=>{ clearScene(); sim.gravity=false; cam.scale=64;
  bodies.push(makeBody(0,0,0.3)); bodies.push(makeBody(1,-1,0.4)); setTool('rack'); })()`);
run(`runToolClick(0,0)`); run(`runToolClick(0.5,0)`); run(`runToolClick(1,-1)`);
ok('three clicks build a two-pin rack carrying one pinion',
   run(`constraints.length===1 && constraints[0].type==='rack'
        && conPoints(constraints[0]).length===1 && conPoints(constraints[0])[0].kind==='pinion'`),
   run('JSON.stringify(constraints)'));
// A disk whose RIM the rack line crosses at (2,0) -- centre (2,0.6), radius 0.6.
run(`bodies.push(makeBody(2,0.6,0.6)); pending=null; runToolClick(2,0);`);
ok('a click near a disk\'s rim on the rack line makes it a pinion',
   run(`conPoints(constraints[0]).length===2 && conPoints(constraints[0])[1].kind==='pinion'`),
   run('JSON.stringify(conPoints(constraints[0]))'));
// A body the rack line runs straight through the middle of.
run(`bodies.push(makeBody(-2,0,0.5)); pending=null; runToolClick(-2,0);`);
ok('a click through a body makes it a joint instead',
   run(`conPoints(constraints[0]).length===3 && conPoints(constraints[0])[2].kind==='point'`),
   run('JSON.stringify(conPoints(constraints[0])[2])'));

run(`(()=>{ clearScene();
  bodies.push(makeBody(0,0,0.2)); bodies.push(makeBody(2,0,0.2)); bodies.push(makeBody(1,0,0.2));
  constraints.push(makeRodCon({id:1,off:[0,0]},{id:2,off:[0,0]},true,true));
  setTool('rod'); pending=null; })()`);
run(`runToolClick(1,0)`);
ok('a rod-tool click on an existing rod adds a point rather than starting a new rod',
   run(`constraints.length===1 && conPoints(constraints[0]).length===1
        && conPoints(constraints[0])[0].ep.id===3`), run('JSON.stringify(constraints)'));
ok('and the point inherited the rod\'s both-welded state',
   run(`conPoints(constraints[0])[0].lock===true`),
   run('JSON.stringify(conPoints(constraints[0])[0])'));
run(`pending=null; runToolClick(1,0)`);
ok('clicking again, over a body the joint already holds, adds nothing',
   run(`conPoints(constraints[0]).length===1`), run('JSON.stringify(conPoints(constraints[0]))'));

console.log('\n9. the editing surface: handles, rendering, and deletion');
ok('conHandles lists the base pair and then every point',
   run(`JSON.stringify(conHandles(constraints[0]).map(h=>h.which))`)==='["A","B","pt"]',
   run(`JSON.stringify(conHandles(constraints[0]).map(h=>h.which))`));
// A rack whose pinion came in from a scene FILE, where a pinion is a bare body id:
// its handle still has somewhere to be, which is what makeConPoint's normalization
// of the endpoint shape buys.
run(`importScene([
  'scene 3','sim gravity=off',
  'body 1 x=0 y=0 r=0.2','body 2 x=2 y=0 r=0.2','body 3 x=1 y=-1 r=0.4',
  'rack 1 -- 2 pt=3/pinion'].join('\\n'))`);
ok('a file-loaded pinion resolves to a handle position',
   run(`(()=>{ const h=conHandles(constraints[0]);
     return h.length===3 && h[2].which==='pt' && isFinite(h[2].x) && isFinite(h[2].y); })()`),
   run(`JSON.stringify(conHandles(constraints[0]))`));
run(`(()=>{ clearScene();
  bodies.push(makeBody(0,0,0.2)); bodies.push(makeBody(2,0,0.2)); bodies.push(makeBody(1,0,0.2));
  constraints.push(makeRodCon({id:1,off:[0,0]},{id:2,off:[0,0]},true,true));
  makeConPoint(constraints[0], {id:3, off:[0,0]}, {lock:true}); })()`);
run(`selectConstraint(0)`);
ok('the handle picker finds the point\'s handle and names its index',
   run(`(()=>{ const h=pickHandle(1,0); return !!h && h.which==='pt' && h.k===0; })()`));
run(`render(); renderInspector(); updateInspectorLive();`);
ok('render and the inspector run clean over a multi-point joint', true);
run(`dropBodyFromConstraints(3)`);
ok('deleting a POINT\'s body drops just the point',
   run(`constraints.length===1 && conPoints(constraints[0]).length===0`));
run(`dropBodyFromConstraints(1)`);
ok('deleting an END\'s body takes the whole joint', run(`constraints.length===0`));

console.log(`\n${pass} ok, ${fail} failed\n`);
process.exit(fail?1:0);
