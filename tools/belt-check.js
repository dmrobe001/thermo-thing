// The belt -- constraints.js §06.2e, DEVELOPMENT.md §4.1c.
//
// Like scene-roundtrip.js, rack-check.js and the other tool-layer checks beside it,
// this one LOADS the simulator: the claims are about the real rows, the real path
// geometry and the real reader, so a reimplementation would test nothing. It also
// checks the belt against a CLOSED FORM worked out here and nowhere in the engine --
// the two-pulley tangent length and wrap angles out of the textbook -- so the path
// the rows measure has an independent witness.
//
// What it asserts:
//
//   1. the path: a two-wheel belt's length is the classical 2*sqrt(d^2-(rA-rB)^2)
//      plus the two wrap arcs, and flipping one wheel's wrap gives the crossed
//      belt's own closed form. The rest length a fresh belt captures is that number.
//   2. the ratio: rA*wA = rB*wB open, and rA*wA = -rB*wB crossed, exactly, held
//      through a run -- and with an untied eyelet bending the path, unchanged, since
//      an eyelet routes the belt and does not grip it.
//   3. the length: an INEXTENSIBLE belt holds its loop's total length, so a free
//      wheel thrown away from a fixed one is stopped by the belting.
//   4. a TIED eyelet is carried by the belt, and a WELDED one turns its body with
//      the belt's local direction.
//   5. softness is a modulus: tension is stretch/soft, it is tension-only (a slack
//      belt pushes nothing), the strain is in the energy ledger, and the ledger
//      holds flat over a run -- which is what says the force and the potential are
//      the same thing.
//   6. `posable` takes the belt out of the rows entirely while a body it runs on is
//      dragged, and puts it back re-fitted to the pose the drag leaves.
//   7. the scene file carries every node and every captured constant, round-trips
//      byte-for-byte, and refuses a node that claims a word of the other kind.
//   8. the tool layer builds one: a click on a disk's EDGE makes a wheel and a click
//      anywhere else an eyelet, and a click on the belting splices a node into the
//      span it landed on.
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
                'js/tools.js','js/inspector.js','js/examples.js','js/scene.js','js/select.js',
                'js/transport.js'])
  vm.runInContext(fs.readFileSync(path.join(ROOT,f),'utf8'), ctx, {filename:f});
const run = s => vm.runInContext(s, ctx);

let pass=0, fail=0;
const ok=(name,good,detail)=>{ good?pass++:fail++;
  console.log((good?'  ok  ':'  FAIL'), name, good?'':('\n        '+detail)); };
const near=(a,b,tol)=>Math.abs(a-b)<=(tol===undefined?1e-9:tol);

// A pair of wheels, each pinned at its centre by a short rod welded only at its
// background end -- the holding the wheel integrator and the rack's pinion use, and
// the only thing that pins anything (§06.2b). `free` leaves the second wheel loose.
run(`sim.running=false;
var PAIR = (rA, rB, d, wrapB, soft, free) => { clearScene(); sim.gravity=false; sim.h=1/480;
  const A=makeBody(0,0,rA), B=makeBody(d,0,rB); bodies.push(A,B);
  constraints.push(makeRodCon({id:null,off:[0,-3]}, {id:A.id,off:[0,0]}, true, false));
  if(!free) constraints.push(makeRodCon({id:null,off:[d,-3]}, {id:B.id,off:[0,0]}, true, false));
  const belt=makeBeltCon([{id:A.id,off:[0,0],kind:'wheel'},
                          {id:B.id,off:[0,0],kind:'wheel',wrap:wrapB||1}], {soft:soft||0});
  constraints.push(belt);
  refreshFrozen(); saveState(true);
  return belt;
};
var BELT = () => constraints.find(c=>c.type==='belt');
var STEP = n => { for(let i=0;i<n;i++) substep(sim.h); };
var maxC = () => Math.max(0, ...constraints.map(c=>conMaxC(c)));
`);

console.log('\n1. the path: the textbook two-pulley belt');
{
  // Open belt, centres d apart, radii rA >= rB: two external tangents of length
  // sqrt(d^2-(rA-rB)^2), the big wheel wrapped through pi+2*asin((rA-rB)/d) and the
  // small one through pi-2*asin((rA-rB)/d).
  const rA=0.5, rB=0.25, d=2;
  const g=Math.asin((rA-rB)/d);
  const openL = 2*Math.sqrt(d*d-(rA-rB)*(rA-rB)) + rA*(Math.PI+2*g) + rB*(Math.PI-2*g);
  run(`PAIR(${rA},${rB},${d},1)`);
  ok('an open belt is the classical tangents plus wraps',
     near(run('beltLength(BELT())'), openL, 1e-12), `${run('beltLength(BELT())')} vs ${openL}`);
  ok('...and that is the rest length it captured',
     near(run('beltRestLen(BELT())'), openL, 1e-9), String(run('beltRestLen(BELT())')));
  ok('...so a fresh belt is unstressed', run('conMaxC(BELT())')<1e-12, String(run('conMaxC(BELT())')));
  // Crossed belt: two internal tangents of length sqrt(d^2-(rA+rB)^2), each wheel
  // wrapped through pi+2*asin((rA+rB)/d).
  const gc=Math.asin((rA+rB)/d);
  const crossL = 2*Math.sqrt(d*d-(rA+rB)*(rA+rB)) + (rA+rB)*(Math.PI+2*gc);
  run(`PAIR(${rA},${rB},${d},-1)`);
  ok('flipping one wheel gives the crossed belt, to its own closed form',
     near(run('beltLength(BELT())'), crossL, 1e-12), `${run('beltLength(BELT())')} vs ${crossL}`);
}

console.log('\n2. the ratio, and what an eyelet does to it');
{
  run('PAIR(0.5,0.25,2,1); bodies[0].w=1;');
  run('STEP(480)');
  ok('an open belt holds rA*wA = rB*wB exactly',
     near(run('0.5*bodies[0].w - 0.25*bodies[1].w'), 0, 1e-12),
     String(run('0.5*bodies[0].w - 0.25*bodies[1].w')));
  ok('...and both wheels turn the same way', run('bodies[0].w')>0 && run('bodies[1].w')>0,
     run('[bodies[0].w,bodies[1].w].join(" ")'));
  run('PAIR(0.5,0.25,2,-1); bodies[0].w=1;');
  run('STEP(480)');
  ok('a crossed belt holds rA*wA = -rB*wB, and reverses the driven wheel',
     near(run('0.5*bodies[0].w + 0.25*bodies[1].w'), 0, 1e-12) && run('bodies[1].w')<0,
     run('[bodies[0].w,bodies[1].w].join(" ")'));
  // An untied eyelet on the background, well below the lower run, bends the path.
  run(`PAIR(0.5,0.25,2,1);
       var belt=BELT(); makeConPoint(belt, {id:null,off:[1,-1.4],kind:'eyelet'}, {kind:'eyelet'});
       beltNodes(belt).splice(2,0,beltNodes(belt).pop());
       beltRefresh(belt); bodies[0].w=1; saveState(true);`);
  const bent = run('beltLength(BELT())');
  ok('an eyelet lengthens the path it bends', bent > 6.4, String(bent));
  run('STEP(480)');
  ok('...and leaves the ratio alone: it routes the belt, it does not grip it',
     near(run('0.5*bodies[0].w - 0.25*bodies[1].w'), 0, 1e-11),
     String(run('0.5*bodies[0].w - 0.25*bodies[1].w')));
}

console.log('\n3. an inextensible belt holds the loop\'s length');
{
  run('PAIR(0.4,0.4,2,1,0,true); bodies[1].vx=1;');
  const L0=run('beltLength(BELT())');
  run('STEP(480)');
  ok('a free wheel thrown outward is stopped by the belting',
     near(run('bodies[1].x'), 2, 1e-6) && Math.abs(run('bodies[1].vx'))<1e-9,
     `x=${run('bodies[1].x')} vx=${run('bodies[1].vx')}`);
  ok('...and the loop kept its length exactly',
     near(run('beltLength(BELT())'), L0, 1e-12), String(run('beltLength(BELT())')-L0));
}

console.log('\n4. a tied eyelet is carried, a welded one is turned');
{
  run(`clearScene(); sim.gravity=false; sim.h=1/480;
    var A=makeBody(0,0,0.5), B=makeBody(3,0,0.5), C=makeBody(1.5,-0.5,0.1);
    bodies.push(A,B,C);
    constraints.push(makeRodCon({id:null,off:[0,-3]},{id:A.id,off:[0,0]},true,false));
    constraints.push(makeRodCon({id:null,off:[3,-3]},{id:B.id,off:[0,0]},true,false));
    constraints.push(makeBeltCon([{id:A.id,off:[0,0],kind:'wheel'},
                                  {id:B.id,off:[0,0],kind:'wheel'},
                                  {id:C.id,off:[0,0],kind:'eyelet',tied:true}]));
    bodies[0].w=1; refreshFrozen(); saveState(true);`);
  const E0=run('energy().tot');
  run('STEP(480)');
  // The lower run travels with the driving wheel's rim: wA = 1 rad/s on r = 0.5 is
  // 0.5 m/s of belting, and the bottom of a wheel turning the positive way goes -x.
  ok('the tied body is carried along the belt, at the belt\'s own speed',
     near(run('bodies[2].vx'), -0.5, 0.08) && run('bodies[2].x') < 1.2,
     `x=${run('bodies[2].x')} vx=${run('bodies[2].vx')}`);
  ok('...with the belt still satisfied', run('conMaxC(constraints[2])')<1e-5,
     String(run('conMaxC(constraints[2])')));
  ok('...and the ledger flat', near(run('energy().tot'), E0, 1e-9),
     `${E0} -> ${run('energy().tot')}`);
  // Weld it, and the body's own angle follows the belt's local direction there. The
  // eyelet rides the belting round the driving wheel, so its direction turns.
  const rows0 = run('rowsFor(constraints[2]).length');
  run(`var nd=beltNodes(constraints[2])[2]; restoreState();
       setBeltLock(constraints[2], nd, true); saveState(true);`);
  ok('three gripping nodes are three segment rows, and a welded eyelet adds a fourth',
     rows0===3 && run('rowsFor(constraints[2]).length')===4,
     `${rows0} -> ${run('rowsFor(constraints[2]).length')}`);
  const th0=run('bodies[2].th');
  run('STEP(480)');
  ok('...and turns its body as the belt\'s local direction turns',
     Math.abs(run('bodies[2].th')-th0) > 0.05, `${th0} -> ${run('bodies[2].th')}`);
}

console.log('\n5. softness is a modulus');
{
  run('PAIR(0.4,0.4,2,1,0.01,true)');
  run('beltSetRestLen(BELT(), beltLength(BELT())-0.2); saveState(true);');
  ok('tension is stretch / softness', near(run('beltTension(BELT())'), 20, 1e-9),
     String(run('beltTension(BELT())')));
  ok('...and the strain energy is stretch^2 / (2*softness)',
     near(run('beltEnergy(BELT())'), 2, 1e-9), String(run('beltEnergy(BELT())')));
  ok('...which the ledger carries as spring potential',
     near(run('energy().SPE'), 2, 1e-9), String(run('energy().SPE')));
  const E0=run('energy().tot');
  run('STEP(960)');
  ok('the ledger holds flat while the belt pulls', near(run('energy().tot'), E0, 1e-9),
     `${E0} -> ${run('energy().tot')}`);
  run('beltSetRestLen(BELT(), beltLength(BELT())+1)');
  ok('a belt slack of its rest length carries nothing',
     run('beltTension(BELT())')===0 && run('beltEnergy(BELT())')===0, 'slack belt pushed');
  // A soft belt still transmits its ratio exactly: no-slip is a row either way.
  run('PAIR(0.5,0.25,2,1,0.01); bodies[0].w=1; STEP(480);');
  ok('a soft belt still holds rA*wA = rB*wB exactly',
     near(run('0.5*bodies[0].w - 0.25*bodies[1].w'), 0, 1e-11),
     String(run('0.5*bodies[0].w - 0.25*bodies[1].w')));
}

console.log('\n6. posable');
{
  run('PAIR(0.4,0.4,2,1); BELT().posable=true;');
  ok('a posable belt is an ordinary belt when nothing is being dragged',
     run('rowsFor(BELT()).length')===2, String(run('rowsFor(BELT()).length')));
  ok('...and no rows at all while a body it runs on is dragged',
     run('beginPosing(bodies[0].id); const n=withPosing(()=>rowsFor(BELT()).length); endPosing(); n')===0,
     'the released belt still had rows');
  ok('...and none of its tension either',
     run(`beltSetRestLen(BELT(), beltLength(BELT())-0.2); BELT().soft=0.01;
          beginPosing(bodies[0].id); const T=withPosing(()=>beltTension(BELT())); endPosing(); T`)===0,
     'the released belt still pulled');
  // The drag moves the body itself; the recapture then re-fits the belt to it.
  run(`BELT().soft=0; beginPosing(bodies[1].id); bodies[1].x=2.6;
       withPosing(()=>{}); recapturePosable(); endPosing();`);
  ok('...and the pose the drag leaves is the belt\'s new rest length',
     near(run('beltRestLen(BELT())'), run('beltLength(BELT())'), 1e-12) && run('conMaxC(BELT())')<1e-12,
     `rest=${run('beltRestLen(BELT())')} len=${run('beltLength(BELT())')}`);
}

console.log('\n7. the scene file');
{
  run(`clearScene(); sim.gravity=false;
    var A=makeBody(0,0,0.5), B=makeBody(3,0,0.25), C=makeBody(1.5,-0.9,0.1);
    bodies.push(A,B,C);
    constraints.push(makeBeltCon([{id:A.id,off:[0,0],kind:'wheel'},
                                  {id:B.id,off:[0,0],kind:'wheel',wrap:-1},
                                  {id:C.id,off:[0,0],kind:'eyelet',tied:true,lock:true},
                                  {id:null,off:[0.5,1.2],kind:'eyelet'}], {soft:0.002, posable:true}));`);
  const text = run('exportScene()');
  const line = text.split('\n').find(l=>l.startsWith('belt'));
  ok('the belt line names every node, in order, with its captured constants',
     /^belt soft=0\.002 posable pt=1\/wheel\/r=0\.5\/restSeg=[-\d.]+ pt=2\/wheel\/r=0\.25\/wrap=-1\/restSeg=[-\d.]+ pt=3\/tied\/restSeg=[-\d.]+\/lock\/restAng=[-\d.]+ pt=bg\(0\.5,1\.2\)$/.test(line),
     line);
  run(`importScene(${JSON.stringify(text)})`);
  ok('...and it round-trips byte-for-byte', run('exportScene()')===text, run('exportScene()'));
  ok('...to a belt that is still exactly satisfied', run('conMaxC(constraints[0])')<1e-9,
     String(run('conMaxC(constraints[0])')));
  // A belt with no gripping node writes its own rest length, since it has no segment
  // constant to hang it off -- and holds its one closed loop's length.
  run(`clearScene(); var C0=makeBody(0,0,0.1), D0=makeBody(2,0,0.1); bodies.push(C0,D0);
       constraints.push(makeBeltCon([{id:C0.id,off:[0,0],kind:'eyelet'},{id:D0.id,off:[0,0],kind:'eyelet'}]));`);
  const t2=run('exportScene()');
  ok('a belt with nothing gripping it writes restLen instead',
     /^belt restLen=4 pt=1 pt=2$/.test(t2.split('\n').find(l=>l.startsWith('belt'))),
     t2.split('\n').find(l=>l.startsWith('belt')));
  ok('...and holds exactly one row: the loop\'s own length',
     run('rowsFor(constraints[0]).length')===1, String(run('rowsFor(constraints[0]).length')));
  const bad = [
    ['a wheel claiming an eyelet\'s word', 'belt pt=1/wheel/tied'],
    ['an eyelet claiming a rim',           'belt pt=1/r=0.5'],
    ['a segment constant on a node that does not grip', 'belt pt=1/restSeg=1 pt=2'],
    ['a rest angle with no lock',          'belt pt=1/restAng=1 pt=2'],
  ];
  for(const [what, tail] of bad){
    const src = 'scene 4\nbody 1 x=0 y=0 r=0.5\nbody 2 x=2 y=0 r=0.5\n'+tail+'\n';
    let msg=null;
    try { run(`importScene(${JSON.stringify(src)})`); } catch(e){ msg=String(e.message||e); }
    ok('rejects '+what, msg!==null, 'accepted it');
  }
}

console.log('\n8. the tool layer builds one');
{
  run(`clearScene(); cam.x=0; cam.y=0; cam.scale=64; sim.gravity=false;
       var A2=makeBody(0,0,0.5), B2=makeBody(2,0,0.5); bodies.push(A2,B2);`);
  const rim = run('JSON.stringify(beltNodeAt(0.5, 0))');
  const mid = run('JSON.stringify(beltNodeAt(0.25, 0.05))');
  const ctr = run('JSON.stringify(beltNodeAt(0.02, 0))');
  ok('a click on a disk\'s EDGE names a wheel', JSON.parse(rim).kind==='wheel', rim);
  ok('a click through its middle names an eyelet on that body', JSON.parse(mid).kind==='eyelet', mid);
  ok('...and so does a click at its centre', JSON.parse(ctr).kind==='eyelet', ctr);
  // A disk smaller than the snap radius must still be able to take an eyelet: a rim
  // test alone would read every click on one as "wrap it".
  run(`var TINY=makeBody(5,5,0.08); bodies.push(TINY);`);
  ok('a small disk\'s centre is still an eyelet, not a rim everywhere',
     run('beltNodeAt(5,5).kind')==='eyelet', run('JSON.stringify(beltNodeAt(5,5))'));
  run('bodies.pop();');
  ok('a click in empty space names an eyelet on the background',
     run('beltNodeAt(0,3).id')===null, String(run('beltNodeAt(0,3).id')));
  run(`setTool('belt'); runToolClick(0.5, 0); runToolClick(1.5, 0);`);
  ok('two clicks make a belt of two wheels',
     run('constraints.length')===1 && run('beltNodes(constraints[0]).length')===2 &&
     run('beltNodes(constraints[0]).every(n=>n.kind==="wheel")'),
     run('JSON.stringify(constraints.map(c=>c.type))'));
  // The lower run of that belt sits at y = -0.5, between the two wheels: a click
  // there splices a node into that span rather than starting a new belt.
  run(`var C2=makeBody(1,-0.5,0.08); bodies.push(C2); runToolClick(1, -0.5);`);
  const nodes = run('JSON.stringify(beltNodes(constraints[0]).map(n=>[n.ep.id,n.kind]))');
  ok('a click on the belting splices a node into the span it landed on, in order',
     run('constraints.length')===1 && run('beltNodes(constraints[0]).length')===3 &&
     JSON.parse(nodes)[1][0]===run('C2.id') && JSON.parse(nodes)[1][1]==='eyelet', nodes);
  ok('...and the belt is still exactly satisfied afterwards',
     run('conMaxC(constraints[0])')<1e-9, String(run('conMaxC(constraints[0])')));
  ok('render and the inspector run clean over a belt',
     run(`(()=>{ try{ selectConstraint(0); render(); renderInspector(); updateInspectorLive(); return 'ok'; }
                catch(e){ return String(e.message||e); } })()`)==='ok',
     run(`(()=>{ try{ selectConstraint(0); render(); renderInspector(); return 'ok'; } catch(e){ return String(e.stack||e); } })()`));
  ok('deleting a node\'s body drops that node and leaves the belt standing',
     run(`dropBodyFromConstraints(C2.id); constraints.length===1 && beltNodes(constraints[0]).length===2`),
     run('JSON.stringify(beltNodes(constraints[0]||{}).map(n=>n.ep.id))'));
  ok('...and deleting one more takes the belt with it: a loop of one is no loop',
     run(`dropBodyFromConstraints(B2.id); constraints.length===0`),
     run('JSON.stringify(constraints.map(c=>c.type))'));
}

console.log(`\n${pass} ok, ${fail} failed\n`);
process.exit(fail?1:0);
