// The VERTEX (constraints.js §06.2e) -- a named point, and the bodies it touches.
//
// Phase 1 of the vertex/line plan (VERTEX.md §X.2, §X.3, §X.12). What it asserts:
//
//   1. the rows: two per joined incidence past the primary, one per welded
//      incidence past the first, and -- the case that surprises -- NONE for a single
//      weld, because welding ties two frames together and one frame has nothing to
//      tie to.
//   2. it holds what it claims to: two bodies brought to one point stay there, a
//      third arm joins the same point, welded bodies turn together while merely
//      joined ones hinge, and a vertex joined to the background is a ground pin --
//      position held, rotation free, which the bench could not express before.
//   3. the primary: the background speaks for a vertex's position before any body
//      does, so a ground pin holds the BODY to the world point and not the other way
//      round.
//   4. the invariants: one incidence per body, a weld only on a joined body, a
//      vertex with nothing left to be a point on goes when its last body does.
//   5. the captures never snap: ticking join or weld on holds the pose it found.
//   6. the file: it round-trips byte-for-byte, its incidences come back with their
//      flags and rest angles, and a scaled selection box leaves it assembled.
//   7. the tool: one tap plants a vertex, and the same tap again in the same place
//      reaches through to join the body underneath -- which is how a hinge is made.
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
const J = expr => JSON.parse(run(`JSON.stringify(${expr})`));

// Two disks whose rims touch at the origin, and a vertex holding them there.
const PAIR = `(()=>{ clearScene(); sim.gravity=false;
  const a=makeBody(-0.3,0,0.3); bodies.push(a);
  const b=makeBody(0.3,0,0.3);  bodies.push(b);
  const v=makeVertex(null);
  makeVertexOn(v,{id:a.id,off:[0.3,0]},{join:true});
  makeVertexOn(v,{id:b.id,off:[-0.3,0]},{join:true});
  constraints.push(v); refreshFrozen(); return v; })()`;

console.log('\n1. the rows a vertex builds');
run(PAIR);
ok('two joined bodies: 2 rows', run('rowsFor(constraints[0]).length')===2);
run(`makeVertexOn(constraints[0],{id:null,off:[0,0]},{join:true});`);
ok('a third joined incidence: 4 rows', run('rowsFor(constraints[0]).length')===4);
run(`setVertexWeld(constraints[0], vertexOns(constraints[0])[0], true);`);
ok('ONE weld holds nothing -- still 4 rows', run('rowsFor(constraints[0]).length')===4,
   'a weld ties two frames together, and one frame has nothing to tie to');
run(`setVertexWeld(constraints[0], vertexOns(constraints[0])[1], true);`);
ok('a second weld adds exactly one row', run('rowsFor(constraints[0]).length')===5);
run(`setVertexWeld(constraints[0], vertexOns(constraints[0])[2], true);`);
ok('a third weld adds one more', run('rowsFor(constraints[0]).length')===6);
run(`(()=>{ const v=constraints[0]; for(const e of vertexOns(v)) setVertexJoin(v,e,false); })()`);
ok('joined to nothing: no rows at all', run('rowsFor(constraints[0]).length')===0);

console.log('\n2. what it holds');
run(PAIR); run('setRunning(true); for(let i=0;i<300;i++) substep(sim.h);');
{
  const p=J('[epWorld(vertexOns(constraints[0])[0]), epWorld(vertexOns(constraints[0])[1])]');
  ok('two bodies stay at one point', near(p[0][0],p[1][0],1e-6) && near(p[0][1],p[1][1],1e-6),
     JSON.stringify(p));
}
run(`(()=>{ const c=makeBody(0,0.6,0.3); bodies.push(c);
  makeVertexOn(constraints[0], {id:c.id, off:[0,-0.6]}, {join:true}); refreshFrozen(); })()`);
run('for(let i=0;i<300;i++) substep(sim.h);');
{
  const p=J('vertexOns(constraints[0]).map(e=>epWorld(e).slice(0,2))');
  ok('a third arm joins the same point',
     p.every(q=>near(q[0],p[0][0],1e-6) && near(q[1],p[0][1],1e-6)), JSON.stringify(p));
}
// Merely joined: the bodies hinge. Welded: they turn as one.
run(`(()=>{ clearScene(); sim.gravity=false;
  const a=makeBody(0,0,0.3); a.w=1.5; bodies.push(a);
  const b=makeBody(0,0,0.3); bodies.push(b);
  const v=makeVertex(null);
  makeVertexOn(v,{id:a.id,off:[0,0]},{join:true});
  makeVertexOn(v,{id:b.id,off:[0,0]},{join:true});
  constraints.push(v); refreshFrozen(); setRunning(true);
  for(let i=0;i<200;i++) substep(sim.h); })()`);
ok('joined but unwelded, the second body does not turn',
   Math.abs(run('bodies[1].th'))<1e-9, run('String(bodies[1].th)'));
run(`(()=>{ clearScene(); sim.gravity=false;
  const a=makeBody(0,0,0.3); a.w=1.5; bodies.push(a);
  const b=makeBody(0,0,0.3); bodies.push(b);
  const v=makeVertex(null);
  const e1=makeVertexOn(v,{id:a.id,off:[0,0]},{join:true});
  const e2=makeVertexOn(v,{id:b.id,off:[0,0]},{join:true});
  constraints.push(v);
  setVertexWeld(v,e1,true); setVertexWeld(v,e2,true);   // both at th = 0
  refreshFrozen(); setRunning(true);
  for(let i=0;i<200;i++) substep(sim.h); })()`);
{
  const d=Math.abs(run('bodies[0].th-bodies[1].th'));
  ok('welded, they turn together', d<1e-6 && Math.abs(run('bodies[0].th'))>0.1,
     'dth='+d+' th0='+run('String(bodies[0].th)'));
}
// ...and what a weld holds is the OFFSET it was captured at, not zero: weld two
// bodies that already disagree and they keep disagreeing by exactly that much.
run(`(()=>{ clearScene(); sim.gravity=false;
  const a=makeBody(0,0,0.3); a.th=0.7; a.w=1.5; bodies.push(a);
  const b=makeBody(0,0,0.3); b.th=-0.4; bodies.push(b);
  const v=makeVertex(null);
  const e1=makeVertexOn(v,{id:a.id,off:[0,0]},{join:true});
  const e2=makeVertexOn(v,{id:b.id,off:[0,0]},{join:true});
  constraints.push(v);
  setVertexWeld(v,e1,true); setVertexWeld(v,e2,true);
  refreshFrozen(); setRunning(true);
  for(let i=0;i<200;i++) substep(sim.h); })()`);
ok('a weld holds the offset it was captured at',
   Math.abs(run('bodies[0].th-bodies[1].th') - 1.1) < 1e-6,
   'dth='+run('String(bodies[0].th-bodies[1].th)')+', captured 1.1');
// A ground pin: the point is held, the rotation is not. The bench could not say this
// before -- pinning to ground took a welded background rod, which killed the spin too.
run(`(()=>{ clearScene(); sim.gravity=true;
  const a=makeBody(1,2,0.3); a.w=2.0; bodies.push(a);
  const v=makeVertex(null);
  makeVertexOn(v,{id:null,off:[1,2]},{join:true});
  makeVertexOn(v,{id:a.id,off:[0,0]},{join:true});
  constraints.push(v); refreshFrozen(); setRunning(true);
  for(let i=0;i<300;i++) substep(sim.h); })()`);
ok('a ground pin holds the point', near(run('bodies[0].x'),1,1e-9) && near(run('bodies[0].y'),2,1e-9),
   run('bodies[0].x+","+bodies[0].y'));
ok('...and leaves the rotation free', Math.abs(run('bodies[0].w')-2.0)<1e-9, run('String(bodies[0].w)'));

console.log('\n3. the primary speaks for the position');
ok('the background outranks a body', run(`(()=>{ clearScene();
  const a=makeBody(5,5,0.3); bodies.push(a);
  const v=makeVertex(null);
  makeVertexOn(v,{id:a.id,off:[0,0]},{join:true});      // body FIRST in the list
  makeVertexOn(v,{id:null,off:[1,2]},{join:true});
  constraints.push(v);
  return vertexPrimary(v).id===null; })()`)===true,
  'a ground pin must hold the body to the world point, not the point to the body');

console.log('\n4. the invariants');
ok('one incidence per body', run(`(()=>{ clearScene();
  const a=makeBody(0,0,0.3); bodies.push(a);
  const v=makeVertex(null);
  makeVertexOn(v,{id:a.id,off:[0,0]},{join:true});
  return makeVertexOn(v,{id:a.id,off:[0.2,0]},{join:true})===null && vertexOns(v).length===1; })()`)===true);
ok('a weld needs a join', run(`(()=>{ const v=constraints[0]||null;
  clearScene(); const a=makeBody(0,0,0.3); bodies.push(a);
  const w=makeVertex(null); makeVertexOn(w,{id:a.id,off:[0,0]},{join:false});
  setVertexWeld(w, vertexOns(w)[0], true);
  return vertexOns(w)[0].weld===false; })()`)===true);
ok('unjoining drops the weld with it', run(`(()=>{ clearScene();
  const a=makeBody(0,0,0.3); bodies.push(a);
  const v=makeVertex(null); const e=makeVertexOn(v,{id:a.id,off:[0,0]},{join:true});
  setVertexWeld(v,e,true);
  setVertexJoin(v,e,false);
  return e.weld===false && e.restAng===undefined; })()`)===true);
ok('a vertex goes when its last body does', run(`(()=>{ clearScene();
  const a=makeBody(0,0,0.3); bodies.push(a);
  const b=makeBody(1,0,0.3); bodies.push(b);
  const v=makeVertex(null);
  makeVertexOn(v,{id:a.id,off:[0,0]},{join:true});
  makeVertexOn(v,{id:b.id,off:[-1,0]},{join:true});
  constraints.push(v);
  dropBodyFromConstraints(a.id);
  const after1 = constraints.length;                    // one body left: still a vertex
  dropBodyFromConstraints(b.id);
  return after1===1 && constraints.length===0; })()`)===true);

console.log('\n5. the captures never snap');
ok('ticking join on re-reads the offset', run(`(()=>{ clearScene();
  const a=makeBody(0,0,0.3); bodies.push(a);
  const b=makeBody(2,0,0.3); bodies.push(b);
  const v=makeVertex(null);
  makeVertexOn(v,{id:a.id,off:[0.3,0]},{join:true});
  const e=makeVertexOn(v,{id:b.id,off:[9,9]},{join:false});   // a spot nowhere near
  constraints.push(v);
  setVertexJoin(v,e,true);
  return Math.abs(conMaxC(v)) < 1e-12; })()`)===true,
  'the tick must hold the pose it found, not drag the body to a stale offset');
ok('ticking weld on captures the live angle', run(`(()=>{ clearScene();
  const a=makeBody(0,0,0.3); a.th=0.7; bodies.push(a);
  const b=makeBody(0,0,0.3); b.th=-0.4; bodies.push(b);
  const v=makeVertex(null);
  const e1=makeVertexOn(v,{id:a.id,off:[0,0]},{join:true});
  const e2=makeVertexOn(v,{id:b.id,off:[0,0]},{join:true});
  constraints.push(v);
  setVertexWeld(v,e1,true); setVertexWeld(v,e2,true);
  return Math.abs(conMaxC(v)) < 1e-12; })()`)===true);

console.log('\n6. the file');
const FILE = [
  'scene 5','sim gravity=off','cam x=0 y=0 scale=64',
  'body 1 x=0 y=0 r=0.3','body 2 x=1 y=0 r=0.3','body 3 x=2 y=0 r=0.3',
  'vertex A on=1@(0.3,0)/join/weld/restAng=0 on=2@(-0.7,0)/join/weld/restAng=0 on=3@(-1.7,0)',
  'vertex B on=bg(0,-1)/join on=1@(0,-0.3)/join',
].join('\n')+'\n';
{
  let err=null, t1=null, t2=null;
  try { run(`importScene(${JSON.stringify(FILE)})`); t1=run('exportScene()');
        run(`importScene(${JSON.stringify(t1)})`); t2=run('exportScene()'); }
  catch(e){ err=e; }
  ok('a vertex scene loads and re-exports', !err, err&&(err.stack||String(err)));
  if(!err){
    ok('and round-trips byte-for-byte', t1===t2, `\n${t1}\n---\n${t2}`);
    const shape=J(`constraints.filter(isVertex).map(v=>[v.label, vertexOns(v).map(e=>[e.id,e.join,e.weld,e.restAng===undefined?null:e.restAng])])`);
    ok('every incidence came back with its flags',
       JSON.stringify(shape)==='[["A",[[1,true,true,0],[2,true,true,0],[3,false,false,null]]],["B",[[null,true,false,null],[1,true,false,null]]]]',
       JSON.stringify(shape));
    for(const line of ['vertex A on=1@(0.3,0)/join/weld/restAng=0 on=2@(-0.7,0)/join/weld/restAng=0 on=3@(-1.7,0)',
                       'vertex B on=bg(0,-1)/join on=1@(0,-0.3)/join'])
      ok(`the file writes back  ${line}`, t1.split('\n').includes(line),
         t1.split('\n').map(l=>'        '+l).join('\n'));
  }
}
// A scaled box spreads the bodies without resizing them, so a vertex's two body-frame
// offsets stop naming one point -- and the re-read is what keeps the machine assembled.
run(`importScene([
 'scene 5','sim gravity=off','cam x=0 y=0 scale=64',
 'body 1 x=0 y=0 r=0.3','body 2 x=1 y=0 r=0.3',
 'vertex A on=1@(0.3,0)/join on=2@(-0.7,0)/join'].join('\\n'));`);
run(`selectGroup(new Set([1,2])); selGroup.s=2.5; groupApply(selGroup);`);
ok('a scaled selection leaves the vertex assembled', run('conMaxC(constraints[0])')<1e-9,
   run('String(conMaxC(constraints[0]))'));
ok('...and the bodies spread rather than grew',
   Math.abs(run('bodies[1].x-bodies[0].x')-2.5)<1e-9 && run('bodies[0].r')===0.3,
   run('bodies.map(b=>b.x+"/r"+b.r).join(" ")'));
// A copied selection is the part as it was, with names of its own.
run(`clearSelection(); selectGroup(new Set([1,2]));
     const frag=selectionFragment(); clearSelection(); pasteFragment(frag);`);
{
  const labels=J('constraints.filter(isVertex).map(v=>v.label)');
  ok('a pasted vertex takes a fresh label', labels.length===2 && labels[0]!==labels[1],
     JSON.stringify(labels));
}

console.log('\n7. the tool');
// One tap plants a vertex; the same tap again in the same place reaches THROUGH the
// body it landed on to the one underneath, which is the whole gesture for a hinge.
// Two overlapping disks, and a point well clear of both centres and both rims, so
// the tap falls through snapAnchor to the plain topmost pick rather than to a snap.
run(`(()=>{ clearScene(); sim.gravity=false;
  bodies.push(makeBody(0,0,1.0));
  bodies.push(makeBody(0.5,0,0.8));    // overlapping, and on top
  setTool('vertex'); cam.scale=64; })()`);
run(`runToolClick(0.45, 0.3)`);
{
  const vs=J('constraints.filter(isVertex).map(v=>vertexOns(v).map(e=>e.id))');
  ok('one tap plants a vertex on the topmost body',
     vs.length===1 && vs[0].length===1 && vs[0][0]===2, JSON.stringify(vs));
}
run(`runToolClick(0.45, 0.3)`);
{
  const vs=J('constraints.filter(isVertex).map(v=>vertexOns(v).map(e=>e.id))');
  ok('the same tap again joins the body underneath',
     vs.length===1 && vs[0].length===2 && vs[0][1]===1, JSON.stringify(vs));
}
run(`runToolClick(0.45, 0.3)`);
{
  const vs=J('constraints.filter(isVertex).map(v=>vertexOns(v).map(e=>e.id))');
  ok('a third tap finds nothing left to join and adds nothing',
     vs.length===1 && vs[0].length===2, JSON.stringify(vs));
}
run(`(()=>{ clearScene(); setTool('vertex'); runToolClick(3,4); })()`);
{
  const vs=J('constraints.filter(isVertex).map(v=>vertexOns(v).map(e=>[e.id,e.off]))');
  ok('a tap on empty space anchors to the background there',
     JSON.stringify(vs)==='[[[null,[3,4]]]]', JSON.stringify(vs));
}

console.log(`\n${pass} ok, ${fail} failed\n`);
process.exit(fail?1:0);
