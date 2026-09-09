// The LINE (constraints.js §06.2f) -- a straight massless bar whose frame is derived
// from the vertices joined to it.
//
// Phase 2 of the vertex/line plan (VERTEX.md §X.4, §X.12). One object replaces four,
// so the first thing to check is that it still does what each of them did:
//
//   1. the rows, and the three shapes they make. Two sliding joints are a drawn
//      guide with NO rows; two held joints hold their distance (a rod); a rail with
//      riders holds them on the line and nothing else (a slot).
//   2. the four behaviours it absorbed, each against what it means rather than
//      against the code it replaced: a pendulum swings at a fixed radius, a welded
//      ground bar pins a body outright, a slider rides a rail and keeps its angle,
//      and a disk meshing with a bar rolls without slip at its own live pitch radius.
//   3. compliance: a two-joint soft line is a linear spring, its strain energy is in
//      the ledger, and the station rows it replaces are gone.
//   4. the origin and the extent, both derived: stations are measured from the first
//      held joint in joint order, so adding a joint cannot reinterpret them, and a
//      line is a finite BAR exactly when nothing on it slides.
//   5. the heading cannot flip: a joint added beyond the far end leaves every weld
//      on the bar holding what it held.
//   6. freezing, restated: a welded ground bar grounds its far body and is compiled
//      away, a strut between two of a vessel's planes locks its length, and deleting
//      it thaws them again.
//   7. the file and the tool: a line round-trips with its joints, and the tool's two
//      modes build one.
//   8. a bar with ONE joint left rides it: it keeps the heading it was left with,
//      measured in that joint's frame, so the body it hangs off carries it -- turning
//      and all -- instead of the bar pivoting about a mark left behind on the
//      background. Dragging its loose end aims it, since nothing else places it, and
//      the heading goes into the file because nothing else can work it out.
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

// A line through the given anchors. `o` per joint: {slide, weld}. Welds go on LAST,
// once every joint exists -- a weld captures the line's own heading, and a line with
// fewer than two joints has none yet.
run(`var mkline = (specs)=>{
  const L=makeLine(); constraints.push(L);
  const vs=[];
  for(const [ep,o] of specs){
    const v=makeVertex(null); makeVertexOn(v,ep,{join:true}); constraints.push(v);
    makeVertexOn(v,{id:L.id},{join:true, slide:!!o.slide});
    vs.push([v,o]);
  }
  for(const [v,o] of vs){ if(!o.weld) continue;
    for(const e of vertexOns(v)) setVertexWeld(v,e,true); }
  refreshFrozen(); return L; };
var theLine = () => constraints.find(isLine);`);

console.log('\n1. the rows, and the three shapes they make');
run(`(()=>{ clearScene(); sim.gravity=false;
  const a=makeBody(0,0,0.2); bodies.push(a);
  const b=makeBody(2,0,0.2); bodies.push(b);
  mkline([[{id:a.id,off:[0,0]},{slide:true}],[{id:b.id,off:[0,0]},{slide:true}]]); })()`);
ok('two sliding joints: no rows at all, a drawn guide', run('rowsFor(theLine()).length')===0,
   'the two that place a line cannot also be held by it');
run(`(()=>{ clearScene(); sim.gravity=false;
  const a=makeBody(0,0,0.2); bodies.push(a);
  const b=makeBody(2,0,0.2); bodies.push(b);
  mkline([[{id:a.id,off:[0,0]},{slide:false}],[{id:b.id,off:[0,0]},{slide:false}]]); })()`);
ok('two held joints: one row, the distance', run('rowsFor(theLine()).length')===1);
run(`(()=>{ clearScene(); sim.gravity=false;
  const b=makeBody(0,2,0.25); bodies.push(b);
  mkline([[{id:null,off:[-3,2]},{slide:true}],[{id:null,off:[3,2]},{slide:true}],
          [{id:b.id,off:[0,0]},{slide:true}]]); })()`);
ok('a rail with a rider: one row, on the line', run('rowsFor(theLine()).length')===1);
run(`makeVertexOn(constraints.filter(isVertex)[2], {id:theLine().id}, {join:true});`);
// A station is a distance from ANOTHER held joint, so one held joint on a line holds
// nothing -- it is the origin, and an origin has nothing to be measured against. The
// same shape as one weld at a vertex holding nothing, and for the same reason: both
// are relations, and a relation needs two parties.
ok('one held joint on a rail holds nothing -- it IS the origin', run(`(()=>{
  clearScene(); sim.gravity=false;
  const b=makeBody(0,2,0.25); bodies.push(b);
  mkline([[{id:null,off:[-3,2]},{slide:true}],[{id:null,off:[3,2]},{slide:true}],
          [{id:b.id,off:[0,0]},{slide:false}]]);
  return rowsFor(theLine()).length; })()`)===1);
ok('...and a SECOND held joint is what pins it', run(`(()=>{
  clearScene(); sim.gravity=false;
  const b=makeBody(0,2,0.25); bodies.push(b);
  mkline([[{id:null,off:[-3,2]},{slide:false}],[{id:null,off:[3,2]},{slide:true}],
          [{id:b.id,off:[0,0]},{slide:false}]]);
  const n=rowsFor(theLine()).length;
  bodies[0].vx=1; setRunning(true); for(let i=0;i<120;i++) substep(sim.h);
  return n===2 && Math.abs(bodies[0].x)<1e-9; })()`)===true,
  'held against a background joint, the rider stops sliding');

console.log('\n2. the four behaviours it absorbed');
ok('a pendulum swings at a fixed radius', run(`(()=>{ clearScene(); sim.gravity=true;
  const b=makeBody(2.6,4.4,0.38); bodies.push(b);
  mkline([[{id:null,off:[0,4.4]},{slide:false}],[{id:b.id,off:[0,0]},{slide:false}]]);
  setRunning(true); for(let i=0;i<240;i++) substep(sim.h);
  const r=Math.hypot(bodies[0].x, bodies[0].y-4.4);
  return Math.abs(r-2.6)<1e-4 && Math.abs(bodies[0].x-2.6)>0.5; })()`)===true);
ok('a welded ground bar pins its far body outright', run(`(()=>{ clearScene(); sim.gravity=true;
  const b=makeBody(1,1,0.3); bodies.push(b);
  mkline([[{id:null,off:[0,1]},{slide:false,weld:true}],[{id:b.id,off:[0,0]},{slide:false,weld:true}]]);
  setRunning(true); for(let i=0;i<240;i++) substep(sim.h);
  return Math.abs(bodies[0].x-1)<1e-9 && Math.abs(bodies[0].y-1)<1e-9 && Math.abs(bodies[0].th)<1e-9; })()`)===true);
ok('a slider rides a rail and keeps its angle', run(`(()=>{ clearScene(); sim.gravity=true;
  const b=makeBody(0,1,0.3); bodies.push(b);
  mkline([[{id:null,off:[-3,1]},{slide:true}],[{id:null,off:[3,1]},{slide:true}],
          [{id:b.id,off:[0,0]},{slide:true,weld:true}]]);
  bodies[0].vx=0.5; setRunning(true); for(let i=0;i<240;i++) substep(sim.h);
  return Math.abs(bodies[0].y-1)<1e-9 && Math.abs(bodies[0].th)<1e-9 && bodies[0].x>0.5; })()`)===true);
{
  // Rolling: the two materials in contact have the same speed along the bar, so with
  // the disk's centre pinned its spin is exactly the bar's own speed over the pitch
  // radius -- signed by the frame, which is what the row actually says.
  const r = run(`(()=>{ clearScene(); sim.gravity=false;
    const cart=makeBody(0,0,0.3); bodies.push(cart); cart.vx=1;
    const pin=makeBody(0,0.4,0.4); bodies.push(pin);
    const L=mkline([[{id:cart.id,off:[-1,0]},{slide:false}],[{id:cart.id,off:[1,0]},{slide:false}]]);
    L.mesh=[pin.id];
    const v=makeVertex(null);
    makeVertexOn(v,{id:null,off:[0,0.4]},{join:true});
    makeVertexOn(v,{id:pin.id,off:[0,0]},{join:true});
    constraints.push(v); refreshFrozen();
    setRunning(true); for(let i=0;i<10;i++) substep(sim.h);
    const f=lineFrame(L);
    const rho=(bodies[1].x-f.wax)*f.nx + (bodies[1].y-f.way)*f.ny;
    const along=bodies[0].vx*f.ux + bodies[0].vy*f.uy;
    return JSON.stringify([bodies[1].w, along/rho, Math.abs(bodies[0].vx)]); })()`);
  const [w, want, moved] = JSON.parse(r);
  ok('a meshing disk rolls without slip at its live pitch radius',
     near(w, want, 1e-9) && moved>0.1, `w=${w} vs ${want}`);
}

console.log('\n3. compliance');
{
  // The claim worth checking is not that it is springy but that it is THE spring:
  // a two-joint compliant line has one stretch, no riders, and nothing else to hold.
  const y = run(`(()=>{ clearScene(); sim.gravity=true;
    const b=makeBody(0,1,0.3); bodies.push(b);
    const L=mkline([[{id:null,off:[0,2]},{slide:false}],[{id:b.id,off:[0,0]},{slide:false}]]);
    L.soft=1/50;
    setRunning(true); for(let i=0;i<240;i++) substep(sim.h); return bodies[0].y; })()`);
  ok('a compliant line oscillates about its rest length', y>0.5 && y<1.5, String(y));
  ok('and it builds no station row -- the distance is a force now',
     run('rowsFor(theLine()).length')===0);
  ok('its strain energy is in the ledger', run(`(()=>{
    const before=energy().SPE;
    bodies[0].y -= 0.3;
    return energy().SPE > before + 1e-6; })()`)===true,
     'without this, §08.6 would read the store as a leak and "correct" it');
  ok('so the total holds flat over two seconds', run(`(()=>{ clearScene(); sim.gravity=true;
    const b=makeBody(0,1,0.3); bodies.push(b);
    const L=mkline([[{id:null,off:[0,2]},{slide:false}],[{id:b.id,off:[0,0]},{slide:false}]]);
    L.soft=1/50; setRunning(true);
    const e0=energy().tot; for(let i=0;i<240;i++) substep(sim.h);
    return Math.abs((energy().tot-e0)/e0) < 1e-9; })()`)===true);
}

console.log('\n4. the origin and the extent are derived');
ok('stations are measured from the first HELD joint in joint order', run(`(()=>{
  clearScene(); sim.gravity=false;
  const a=makeBody(0,0,0.2); bodies.push(a);
  const b=makeBody(2,0,0.2); bodies.push(b);
  mkline([[{id:a.id,off:[0,0]},{slide:false}],[{id:b.id,off:[0,0]},{slide:false}]]);
  const ss=constraints.filter(isVertex).map(v=>vertexOns(v).find(isLineOn).s);
  return JSON.stringify(ss); })()`)==='[0,-2]');
ok('a line is a BAR exactly when nothing on it slides', run(`(()=>{
  const L=theLine(), v=constraints.filter(isVertex)[1];
  const wasBar=lineIsBar(L);
  setVertexSlide(v, vertexOns(v).find(isLineOn), true);
  const nowRail=!lineIsBar(L);
  setVertexSlide(v, vertexOns(v).find(isLineOn), false);
  return wasBar && nowRail && lineIsBar(L); })()`)===true);

console.log('\n5. the heading cannot flip under a new joint');
ok('a joint added beyond the far end leaves the welds holding what they held', run(`(()=>{
  clearScene(); sim.gravity=false;
  const a=makeBody(0,0,0.2); bodies.push(a);
  const b=makeBody(2,0,0.2); bodies.push(b);
  const L=mkline([[{id:a.id,off:[0,0]},{slide:false,weld:true}],
                  [{id:b.id,off:[0,0]},{slide:false,weld:true}]]);
  const before=lineFrame(L).phi;
  // A third body PAST the first joint, on the other side: the pair that places the
  // line changes, and without the continuity anchor the heading would turn end for end.
  const c=makeBody(-3,0,0.2); bodies.push(c);
  const v=makeVertex(null); makeVertexOn(v,{id:c.id,off:[0,0]},{join:true}); constraints.push(v);
  makeVertexOn(v,{id:L.id},{join:true, slide:true});
  const after=lineFrame(L).phi;
  return Math.abs(after-before) < 1e-9; })()`)===true);

console.log('\n6. freezing, restated for the line');
ok('a welded ground bar grounds its far body, and is compiled away', run(`(()=>{
  clearScene(); sim.gravity=true;
  const b=makeBody(1,1,0.3); bodies.push(b);
  const L=mkline([[{id:null,off:[0,1]},{slide:false,weld:true}],[{id:b.id,off:[0,0]},{slide:false,weld:true}]]);
  return bodies[0].static===true && L._compiled===true; })()`)===true);
ok('...and deleting it thaws the body again', run(`(()=>{
  constraints=constraints.filter(c=>!isLine(c)); refreshFrozen();
  return bodies[0].static===false; })()`)===true);
ok('a strut between two of a vessel\'s planes locks its length, not its pose', run(`(()=>{
  clearScene(); sim.gravity=false;
  const v=makeVessel(0,0,0.5,1.8); bodies.push(v);
  mkline([[{id:v.id,off:[0,-0.5]},{slide:false}],[{id:v.id,off:[0,0.5]},{slide:false}]]);
  return bodies[0].lenLock===true && bodies[0].static===false; })()`)===true);
ok('an unwelded ground bar grounds nothing', run(`(()=>{
  clearScene(); sim.gravity=true;
  const b=makeBody(1,1,0.3); bodies.push(b);
  mkline([[{id:null,off:[0,1]},{slide:false}],[{id:b.id,off:[0,0]},{slide:false}]]);
  return bodies[0].static===false; })()`)===true);

console.log('\n7. the file, and the tool');
{
  const FILE = [
    'scene 5','sim gravity=on','cam x=0 y=0 scale=64',
    'body 1 x=0 y=0 r=0.2','body 2 x=2 y=0 r=0.2','body 3 x=1 y=0 r=0.15',
    'line 4 soft=0.02 posable',
    'vertex A on=1/join on=4/join/fix/s=0',
    'vertex B on=2/join on=4/join/fix/s=-2',
    'vertex C on=3/join on=4/join',
  ].join('\n')+'\n';
  let err=null,t1=null,t2=null;
  try { run(`importScene(${JSON.stringify(FILE)})`); t1=run('exportScene()');
        run(`importScene(${JSON.stringify(t1)})`);   t2=run('exportScene()'); }
  catch(e){ err=e; }
  ok('a line scene loads and re-exports', !err, err&&(err.stack||String(err)));
  if(!err){
    ok('and round-trips byte-for-byte', t1===t2);
    ok('the line came back with its compliance and its flag',
       t1.split('\n').includes('line 4 soft=0.02 posable'), t1);
  }
}
// The tool: NEW places a line through the first two vertices tapped, and every tap
// after joins another. Nothing exists after ONE tap -- one point has no direction.
run(`(()=>{ clearScene(); sim.gravity=false;
  bodies.push(makeBody(0,0,0.2)); bodies.push(makeBody(2,0,0.2)); bodies.push(makeBody(1,1,0.2));
  setTool('line'); setLineMode(false); cam.scale=64; })()`);
run(`runToolClick(0,0)`);
ok('one tap makes no line -- a point has no direction', run('constraints.filter(isLine).length')===0);
run(`runToolClick(2,0)`);
ok('the second tap places the line through both', run('constraints.filter(isLine).length')===1
   && run('lineJoints(theLine()).length')===2);
run(`runToolClick(1,1)`);
ok('a third tap joins another vertex to the SAME line',
   run('constraints.filter(isLine).length')===1 && run('lineJoints(theLine()).length')===3);
ok('...and the quick solve brought it onto the line',
   Math.abs(run('conMaxC(theLine())'))<1e-6, run('String(conMaxC(theLine()))'));
ok('every joint the tool makes slides',
   J('lineJoints(theLine()).map(K=>!!K.e.slide)').every(Boolean));
run(`(()=>{ setLineMode(true); pending=null;
  bodies.push(makeBody(3,3,0.2)); })()`);
{
  // Tap a point that is actually ON the line -- the quick solve moved the bodies to
  // make them colinear, so where the third one started is not where it ended up.
  const f=J('(()=>{ const f=lineFrame(theLine()); return [f.wax,f.way,f.wbx,f.wby]; })()');
  run(`runToolClick(${(f[0]+f[2])/2}, ${(f[1]+f[3])/2})`);
}
run(`runToolClick(3,3)`);
ok('EXTEND joins a further vertex to the line it started on',
   run('constraints.filter(isLine).length')===1 && run('lineJoints(theLine()).length')===4);

console.log('\n8. a bar with one joint left rides it');
// Ground -- BAR -- body, both joints held: an anchored arm. Then let the ground go,
// which is the gesture that leaves the bar with one joint. What used to happen is
// that the mark the loose end left behind went on placing the bar, so it pivoted
// about a point on the background that nothing held and that no panel listed.
const ARM = `(()=>{ clearScene(); sim.gravity=false; cam.scale=64;
  const b=makeBody(1,0,0.3); bodies.push(b);
  const A=makeVertex(null); makeVertexOn(A,{id:null,off:[0,0]},{join:true}); constraints.push(A);
  const B=makeVertex(null); makeVertexOn(B,{id:b.id,off:[0,0]},{join:true}); constraints.push(B);
  const L=makeLine(); constraints.push(L);
  makeVertexOn(A,{id:L.id},{join:true,slide:false});
  makeVertexOn(B,{id:L.id},{join:true,slide:false});
  refreshFrozen(); })()`;
const loose='constraints.filter(isVertex)[0]', held='constraints.filter(isVertex)[1]';
const arm = () => { const a=J(`vertexWorld(${loose})`), b=J(`vertexWorld(${held})`);
  return { len:Math.hypot(a[0]-b[0],a[1]-b[1]), ang:Math.atan2(a[1]-b[1],a[0]-b[0]), at:a }; };
// Headings compare modulo a whole turn: atan2 has a branch cut and a swing may cross it.
const angNear=(a,b,tol)=>{ let d=(a-b)%(2*Math.PI);
  if(d> Math.PI) d-=2*Math.PI;
  if(d<-Math.PI) d+=2*Math.PI;
  return Math.abs(d)<=tol; };
run(ARM);
run(`setIncidenceJoin(${loose}, null, false)`);
ok('letting the ground go leaves the bar placed, and holding nothing',
   J(`!!linePlacement(theLine())`)===true && J(`!!lineFrame(theLine())`)===false
     && J(`rowsFor(theLine()).length`)===0);
ok('...with the loose end carried by the bar', J(`!!vertexRide(${loose})`)===true);
{
  const before=arm();
  run(`bodies[0].x += 1; bodies[0].y += 1;`);
  const after=arm();
  ok('moving the body carries the whole arm with it',
     near(after.len, before.len, 1e-9) && angNear(after.ang, before.ang, 1e-9)
       && near(after.at[0], before.at[0]+1, 1e-9) && near(after.at[1], before.at[1]+1, 1e-9),
     JSON.stringify([before, after]));
  ok('...and the mark it left on the background is no longer a pin',
     Math.hypot(after.at[0], after.at[1])>1e-6,
     'the bar used to pivot about the point the ground pin was at');
}
{
  // ...and TURNING the body turns the arm, which is the half a stale world point can
  // never do: the heading is held in the body's frame, not the world's.
  const before=arm();
  run(`bodies[0].th += Math.PI/2;`);
  const after=arm();
  ok('turning the body swings the arm with it',
     near(after.len, before.len, 1e-9) && angNear(after.ang, before.ang+Math.PI/2, 1e-9),
     JSON.stringify([before.ang, after.ang]));
  run(`bodies[0].th -= Math.PI/2;`);
}
{
  // Dragging the loose end: nothing else places this bar, so the point IS its free
  // end and the drag says both things at once -- which way the bar points, and how
  // far along it the point sits.
  run(ARM);
  run(`setIncidenceJoin(${loose}, null, false)`);
  const at=J(`vertexWorld(${loose})`);
  run(`(()=>{ const h=pickHandle(${at[0]}, ${at[1]}); if(!h) throw new Error('no handle');
    applyHandle(h, 1.6, 1.2); })()`);
  const now=J(`vertexWorld(${loose})`);
  ok('dragging the loose end puts it exactly where the hand asked',
     near(now[0],1.6,1e-9) && near(now[1],1.2,1e-9), JSON.stringify(now));
  ok('...which is an aim and a length, both, since both are the point\'s to say',
     near(arm().len, Math.hypot(1.6-1, 1.2), 1e-9), String(arm().len));
  const before=arm();
  run(`bodies[0].th += 0.4;`);
  ok('...and the aim rides the body from then on',
     angNear(arm().ang, before.ang+0.4, 1e-9), JSON.stringify([before.ang, arm().ang]));
}
{
  // The file. Nothing else can work the heading out -- the marks the loose ends stand
  // on are where they WERE, and the body has moved since -- so the bar writes it.
  const txt=run(`exportScene()`);
  ok('a bar riding one joint writes its heading', /^line \d+ ang=/m.test(txt),
     txt.split('\n').filter(l=>l.startsWith('line')).join(' | '));
  const was=arm();
  run(`importScene(${JSON.stringify('')} + ${JSON.stringify(txt)})`);
  const back=arm();
  ok('...and comes back with the arm exactly where it was',
     near(back.at[0], was.at[0], 1e-9) && near(back.at[1], was.at[1], 1e-9)
       && near(back.len, was.len, 1e-9), JSON.stringify([was.at, back.at]));
  ok('...byte for byte', run(`exportScene()`)===txt);
  // ...and a bar its joints still derive holds no heading, so it writes none: a
  // capture nothing reads is a number the file would have to keep honest for nothing.
  run(ARM);
  ok('a bar two joints still place writes none', !/ang=/.test(run(`exportScene()`)),
     run(`exportScene()`).split('\n').filter(l=>l.startsWith('line')).join(' | '));
}
{
  // Three points on the arm, one of them held by the body: dragging the far one turns
  // the whole bar about that joint, and the one in the middle stays where it is ALONG
  // the bar -- it has a station of its own, and a drag on somebody else's end is not
  // an instruction about it. Which side of the held joint each of them is on is not
  // the drag's to change either: a point taken across it would swing every other one
  // through half a turn to follow.
  run(`(()=>{ clearScene(); sim.gravity=false; cam.scale=64;
    const b=makeBody(0,0,0.3); bodies.push(b);
    const A=makeVertex(null); makeVertexOn(A,{id:b.id,off:[0,0]},{join:true}); constraints.push(A);
    const B=makeVertex(null); makeVertexOn(B,{id:null,off:[1,0]},{join:false}); constraints.push(B);
    const C=makeVertex(null); makeVertexOn(C,{id:null,off:[2,0]},{join:false}); constraints.push(C);
    const L=makeLine(); constraints.push(L);
    for(const v of [A,B,C]) makeVertexOn(v,{id:L.id},{join:true,slide:false});
    for(const v of [A,B,C]) settleVertex(v, ...vertexWorld(v));
    captureLineHeading(L); refreshFrozen(); })()`);
  const at = lab => J(`vertexWorld(constraints.find(c=>isVertex(c)&&c.label==='${lab}'))`);
  run(`(()=>{ const p=vertexWorld(constraints.find(c=>isVertex(c)&&c.label==='C'));
    const h=pickHandle(p[0],p[1]); if(!h) throw new Error('no handle'); applyHandle(h, 0, 2.5); })()`);
  const A=at('A'), B=at('B'), C=at('C');
  ok('dragging one end of a three-point arm turns the whole bar about its held joint',
     near(C[0],0,1e-9) && near(C[1],2.5,1e-9) && near(Math.hypot(B[0]-A[0],B[1]-A[1]), 1, 1e-9),
     JSON.stringify([A,B,C]));
  ok('...with the middle point still between them, on the side it was on',
     near(B[0],0,1e-9) && near(B[1],1,1e-9), JSON.stringify(B));
  run(`bodies[0].x += 1; bodies[0].th += Math.PI/2;`);
  const A2=at('A'), B2=at('B'), C2=at('C');
  ok('...and the whole arm rides the body, in order and to scale',
     near(Math.hypot(B2[0]-A2[0],B2[1]-A2[1]), 1, 1e-9)
       && near(Math.hypot(C2[0]-A2[0],C2[1]-A2[1]), 2.5, 1e-9)
       && near((B2[0]-A2[0])*(C2[0]-A2[0])+(B2[1]-A2[1])*(C2[1]-A2[1]), 2.5, 1e-9),
     JSON.stringify([A2,B2,C2]));
}

console.log(`\n${pass} ok, ${fail} failed\n`);
process.exit(fail?1:0);
