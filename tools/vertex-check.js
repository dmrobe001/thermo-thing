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
//   7. the tool: one tap plants a POINT -- a background incidence, unjoined, holding
//      the coordinates of the place tapped, whatever body it landed on -- and the same
//      tap again reaches through to join the bodies under it, topmost first, which is
//      how a hinge is made by hand. Plus what that buys: a body dragged out from under
//      an unjoined vertex leaves it where it was, and so does letting go of the last
//      body it was held to.
//   8. the two lists the panels show: EXTENT, not incidence. A vertex lists every
//      body whose outline covers it -- the background always, and the one underneath
//      exactly like the one on top -- and a body lists every vertex inside it. Ticking
//      `joined` off leaves the body listed holding nothing, which is why there is no
//      longer a "remove" button and nothing a press could lose.
//   9. what a coordinate in one of those rows commits: the ANCHOR where an incidence
//      holds one, the VERTEX where nothing does, and a solve attempt either way.
//  10. the same from a line's side: a vertex merely lying on the bar is listed by it
//      and has a station, ticking `joined` makes it a slider, and a held joint takes
//      the station it is given.
//  11. what "lying on the bar" means, which is a question about the PICTURE: the two
//      glyphs touching, so the same 4 cm is on the line at one zoom and off it at the
//      next. Then the gesture the lists exist for -- put a point down, tick it onto a
//      line and onto a body -- landing on the same assembled bench in either order.
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
// One tap plants a POINT: a background incidence, unjoined, holding the world
// coordinates of the place tapped -- and nothing else, whatever body happens to be
// under it. Which body that would be is a question about draw order, and a point must
// not take its frame from whatever is on top. Tapping the same place again reaches
// through to join a body, topmost first, which is still how a hinge is made by hand.
// Two overlapping disks, and a point well clear of both centres and both rims, so the
// tap falls through snapAnchor to the plain pick rather than to a snap.
run(`(()=>{ clearScene(); sim.gravity=false;
  bodies.push(makeBody(0,0,1.0));
  bodies.push(makeBody(0.5,0,0.8));    // overlapping, and on top
  setTool('vertex'); cam.scale=64; })()`);
run(`runToolClick(0.45, 0.3)`);
{
  const vs=J('constraints.filter(isVertex).map(v=>vertexOns(v).map(e=>[e.id,e.off,!!e.join]))');
  ok('one tap plants a point in the BACKGROUND frame, joined to nothing',
     JSON.stringify(vs)==='[[[null,[0.45,0.3],false]]]', JSON.stringify(vs));
}
{
  // ...and it stays where it was put when the disk it is sitting in is dragged away.
  run(`bodies[1].x = 4`);
  const w=J('vertexWorld(constraints[0])');
  ok('dragging the body it is inside does not move it',
     near(w[0],0.45,1e-12) && near(w[1],0.3,1e-12), JSON.stringify(w));
  run(`bodies[1].x = 0.5`);
}
run(`runToolClick(0.45, 0.3)`);
{
  const vs=J('constraints.filter(isVertex).map(v=>vertexOns(v).map(e=>e.id))');
  ok('the same tap again joins the topmost body',
     vs.length===1 && vs[0].length===2 && vs[0][1]===2, JSON.stringify(vs));
}
run(`runToolClick(0.45, 0.3)`);
{
  const vs=J('constraints.filter(isVertex).map(v=>vertexOns(v).map(e=>e.id))');
  ok('...and again reaches the body underneath',
     vs.length===1 && vs[0].length===3 && vs[0][2]===1, JSON.stringify(vs));
}
run(`runToolClick(0.45, 0.3)`);
{
  const vs=J('constraints.filter(isVertex).map(v=>vertexOns(v).map(e=>e.id))');
  ok('a fourth tap finds nothing left to join and adds nothing',
     vs.length===1 && vs[0].length===3, JSON.stringify(vs));
}
run(`(()=>{ clearScene(); setTool('vertex'); runToolClick(3,4); })()`);
{
  const vs=J('constraints.filter(isVertex).map(v=>vertexOns(v).map(e=>[e.id,e.off,!!e.join]))');
  ok('a tap on empty space is the same thing -- a point, not a ground pin',
     JSON.stringify(vs)==='[[[null,[3,4],false]]]', JSON.stringify(vs));
}
{
  // Letting go of the last body a vertex is held to leaves it exactly where it was:
  // the background incidence takes over, re-read at the point rather than at whatever
  // it last happened to hold.
  run(`(()=>{ clearScene(); sim.gravity=false;
    const b=makeBody(0,0,0.5); bodies.push(b);
    const v=makeVertex(null); makeVertexOn(v,{id:b.id,off:[0.2,0.1]},{join:true});
    constraints.push(v); bodies[0].x=3; })()`);
  const before=J('vertexWorld(constraints[0])');
  run(`setIncidenceJoin(constraints[0], 1, false)`);
  const after=J('vertexWorld(constraints[0])');
  ok('unticking the last join leaves the point where it stood',
     near(before[0],after[0],1e-12) && near(before[1],after[1],1e-12), JSON.stringify([before,after]));
  run(`bodies[0].x = 9`);
  const later=J('vertexWorld(constraints[0])');
  ok('...and the body may then move without taking it along',
     near(after[0],later[0],1e-12) && near(after[1],later[1],1e-12), JSON.stringify(later));
}


// ---------------------------------------------------------------------------
console.log('\n8. the two lists: extent, not incidence (constraints.js §06.2e, inspector.js §14.2c)');
// Two overlapping disks and a vertex joined to ONE of them, at a point inside both.
const OVERLAP = `(()=>{ clearScene(); sim.gravity=false;
  const a=makeBody(0,0,1.0);   bodies.push(a);
  const b=makeBody(0.5,0,0.8); bodies.push(b);
  const v=makeVertex(null);
  makeVertexOn(v,{id:b.id,off:[-0.05,0.3]},{join:true});
  constraints.push(v); refreshFrozen(); return v; })()`;
run(OVERLAP);
{
  const sites=J('vertexSites(constraints[0]).map(s=>[s.id, !!s.e])');
  ok('a vertex lists the background, and BOTH bodies it is inside',
     JSON.stringify(sites)==='[[null,false],[1,false],[2,true]]', JSON.stringify(sites));
}
ok('...and the one underneath is listed exactly like the one on top',
   J('vertexSites(constraints[0]).map(s=>s.id)').includes(1),
   'depth decides what a click lands on, not what a point is inside');
{
  // Ticking `joined` on a body that was only listed: it takes the incidence at the
  // place the vertex already occupies, so nothing moves.
  const before=J('vertexWorld(constraints[0])');
  run(`setIncidenceJoin(constraints[0], 1, true)`);
  const after=J('vertexWorld(constraints[0])');
  const e=J('vertexOns(constraints[0]).find(x=>x.id===1)');
  ok('ticking joined on a listed body makes the incidence, and snaps nothing',
     near(before[0],after[0],1e-12) && near(before[1],after[1],1e-12) && e && e.join===true,
     JSON.stringify([before,after,e]));
}
{
  // ...and ticking it off leaves the body ON the list. That is the whole reason the
  // panel has no "remove" button: there is nothing a press could take away.
  run(`setIncidenceJoin(constraints[0], 1, false)`);
  const sites=J('vertexSites(constraints[0]).map(s=>s.id)');
  ok('unticking joined leaves the body listed', sites.includes(1), JSON.stringify(sites));
  ok('...holding nothing', J('vertexOns(constraints[0]).find(x=>x.id===1).join')===false);
}
{
  // The same relation from the body's side. Disk 1 holds nothing here, so the two
  // readings differ, and the difference is exactly what the panel needed: `verticesOn`
  // is the incidence relation the row assembly and the scene walk read, and
  // `verticesInExtent` is what the point is inside.
  run(OVERLAP);
  ok('a body lists a vertex inside it that it holds nothing of',
     JSON.stringify(J('verticesInExtent(1).map(v=>v.label)'))==='["A"]');
  ok('...where the INCIDENCE relation, which the rows read, is still empty',
     JSON.stringify(J('verticesOn(1).map(v=>v.label)'))==='[]');
  run(`bodies[0].x = 8`);                       // walk the disk away from the vertex
  ok('a body that no longer covers the vertex stops listing it',
     JSON.stringify(J('verticesInExtent(1).map(v=>v.label)'))==='[]');
  ok('...and one that holds an incidence keeps listing it wherever it goes',
     JSON.stringify(J('verticesInExtent(2).map(v=>v.label)'))==='["A"]',
     'an incidence is a stored fact, not a geometric one');
}

console.log('\n9. what a coordinate in one of those rows commits');
run(PAIR);
{
  // A joined incidence: the number IS the anchor, so committing it moves the anchor
  // and the assembly has to follow. Both disks are free, so the solve brings them
  // back into coincidence at the new material point.
  run(`commitIncidenceOff(constraints[0], 1, 0.3, 0.15)`);
  const e=J('vertexOns(constraints[0]).find(x=>x.id===1)');
  ok('editing the coordinates of a joined body writes the anchor',
     JSON.stringify(e.off)==='[0.3,0.15]', JSON.stringify(e.off));
  ok('...and the solve puts the assembly back together',
     run('conMaxC(constraints[0])')<1e-8, String(run('conMaxC(constraints[0])')));
}
run(OVERLAP);
{
  // A body the vertex is merely INSIDE has no anchor to move, so the number says
  // where in that body's frame to put the vertex -- the same edit its own x/y field
  // makes, said in another frame.
  run(`commitIncidenceOff(constraints[0], 1, 0.4, -0.2)`);
  const w=J('vertexWorld(constraints[0])');
  ok('editing a body it is only inside moves the VERTEX there',
     near(w[0],0.4,1e-9) && near(w[1],-0.2,1e-9), JSON.stringify(w));
  ok('...and the body it IS joined to re-read its own anchor, so it did not move',
     near(J('bodies[1].x'),0.5,1e-9) && near(J('bodies[1].y'),0,1e-9));
}

console.log('\n10. a line lists the vertices on it, joint or not');
run(`(()=>{ clearScene(); sim.gravity=false;
  const L=makeLine(); constraints.push(L);
  for(const x of [-1,1]){
    const v=makeVertex(null); makeVertexOn(v,{id:null,off:[x,0]},{join:true});
    constraints.push(v); makeVertexOn(v,{id:L.id},{join:true,slide:false}); }
  // A third vertex sitting ON the bar, pinned to a disk, joined to nothing else.
  const b=makeBody(0.25,0,0.2); bodies.push(b);
  const v=makeVertex(null); makeVertexOn(v,{id:b.id,off:[0,0]},{join:true});
  constraints.push(v);
  refreshFrozen(); })()`);
{
  const on=J('verticesInExtent(constraints.find(isLine).id).map(v=>v.label)');
  ok('a vertex lying on the bar is listed by it, with no joint at all',
     on.length===3, JSON.stringify(on));
  // Stations run from the origin -- the first HELD joint in joint order, the one at
  // x=-1 -- along the line's own u, which points from the second end to the first.
  const st=run(`incidenceStation(constraints[3], constraints.find(isLine))`);
  ok('...and has a station, read off the geometry', near(st, -1.25, 1e-9), String(st));
}
{
  run(`setIncidenceJoin(constraints[3], constraints.find(isLine).id, true)`);
  const e=J('vertexOns(constraints[3]).find(x=>x.kind==="line")');
  ok('ticking joined on a line makes a SLIDING joint', !!e && e.join===true && e.slide===true,
     JSON.stringify(e));
  run(`setVertexSlide(constraints[3], vertexOns(constraints[3]).find(x=>x.kind==="line"), false)`);
  run(`commitIncidenceStation(constraints[3], constraints.find(isLine), 0.5)`);
  const s2=J('vertexOns(constraints[3]).find(x=>x.kind==="line").s');
  ok('...and a held joint takes the station it is given', near(s2,0.5,1e-12), String(s2));
  // Station 0.5 from an origin at x=-1, along a u that points toward -x: the disk is
  // pulled to x=-1.5, which is the solve doing what the number asked for.
  ok('...which the solve made true of the disk carrying it',
     near(J('bodies[0].x'), -1.5, 1e-6) && near(J('bodies[0].y'), 0, 1e-6),
     JSON.stringify([J('bodies[0].x'), J('bodies[0].y')]));
}


console.log('\n11. a line lists what TOUCHES it on screen, and the tick that follows');
// A held bar along y=0, and a vertex a little above it. Whether that vertex is "on"
// the line is a question about the picture -- do the two glyphs touch -- so the answer
// has to change with the zoom, and a fixed distance in metres cannot give it.
const BAR = `(()=>{ clearScene(); sim.gravity=false; cam.scale=100;
  const mk=(x,y)=>{ const v=makeVertex(null); makeVertexOn(v,{id:null,off:[x,y]},{join:true}); constraints.push(v); return v; };
  const L=makeLine(); constraints.push(L);
  for(const v of [mk(-1,0),mk(1,0)]) makeVertexOn(v,{id:L.id},{join:true,slide:false});
  refreshFrozen(); return L; })()`;
const theBar = 'constraints.find(isLine)';
run(BAR);
run(`(()=>{ const v=makeVertex(null); makeVertexOn(v,{id:null,off:[0,0.04]},{join:false});
  constraints.push(v); })()`);
{
  // 0.04 m off the bar. At 100 px/m that is 4 px -- the dot and the stroke touch.
  run(`cam.scale=100`);
  ok('a vertex 4px off the bar is listed by it',
     J(`verticesInExtent(${theBar}.id).length`)===3, JSON.stringify(J(`verticesInExtent(${theBar}.id).map(v=>v.label)`)));
  // Zoom in and the same 0.04 m is 8 px: the glyphs have come apart.
  run(`cam.scale=200`);
  ok('...and zooming IN, at the same 0.04m, drops it',
     J(`verticesInExtent(${theBar}.id).length`)===2, String(J(`verticesInExtent(${theBar}.id).length`)));
  // Zoom out and it is well inside the stroke again. A tolerance in metres could not
  // do this: it would be the same distance at every zoom, which is not what a person
  // tapping two glyphs together is judging.
  run(`cam.scale=40`);
  ok('...and zooming OUT brings it back', J(`verticesInExtent(${theBar}.id).length`)===3);
  run(`cam.scale=100`);
  ok('the vertex says the same thing from its own side',
     J(`vertexSites(constraints[2]).map(s=>s.id)`).includes(J(`${theBar}.id`)),
     JSON.stringify(J(`vertexSites(constraints[2]).map(s=>s.id)`)));
}
{
  // The gesture the lists exist for: put a point down, then tick it onto things. A
  // line is never a locator (§X.3), so joining the line alone makes no joint at all --
  // and the tick that gives the vertex a body is what brings the row to life and
  // closes the gap the point was left at.
  run(BAR);
  run(`(()=>{ bodies.push(makeBody(0.25, 1.0, 0.2)); refreshFrozen();
    const v=makeVertex(null); makeVertexOn(v,{id:null,off:[0.25,0.04]},{join:false});
    constraints.push(v); })()`);
  const V='constraints[constraints.length-1]';
  run(`setIncidenceJoin(${V}, ${theBar}.id, true)`);
  ok('joining only the line makes no joint -- nothing says where the point is',
     J(`lineJoints(${theBar}).length`)===2, String(J(`lineJoints(${theBar}).length`)));
  run(`setIncidenceJoin(${V}, bodies[0].id, true)`);
  ok('...and ticking a body at it makes the joint real',
     J(`lineJoints(${theBar}).length`)===3, String(J(`lineJoints(${theBar}).length`)));
  ok('...with the solve having closed the gap it was placed at',
     run(`conMaxC(${theBar})`)<1e-8, String(run(`conMaxC(${theBar})`)));
  ok('...by bringing the DISK down, the bar being pinned at both ends',
     near(J('bodies[0].y'), 1.0-0.04, 1e-6), String(J('bodies[0].y')));
}
{
  // The same two ticks the other way round must land in the same place.
  run(BAR);
  run(`(()=>{ bodies.push(makeBody(0.25, 1.0, 0.2)); refreshFrozen();
    const v=makeVertex(null); makeVertexOn(v,{id:null,off:[0.25,0.04]},{join:false});
    constraints.push(v); })()`);
  const V='constraints[constraints.length-1]';
  run(`setIncidenceJoin(${V}, bodies[0].id, true)`);
  run(`setIncidenceJoin(${V}, ${theBar}.id, true)`);
  ok('body first, then line, ends at the same assembled bench',
     J(`lineJoints(${theBar}).length`)===3 && run(`conMaxC(${theBar})`)<1e-8
       && near(J('bodies[0].y'), 1.0-0.04, 1e-6), String(J('bodies[0].y')));
}

console.log(`\n${pass} ok, ${fail} failed\n`);
process.exit(fail?1:0);
