// Selection groups, the transform box, and widgets -- select.js §18, scene.js §17.7.
//
// Like scene-roundtrip.js, multipoint-check.js and posable-check.js beside it, this
// one LOADS the simulator: the claims are about the real membership rule, the real
// captured fields and the real reader, so a reimplementation would test nothing.
// What it asserts:
//
//   1. the membership rule: a coupling joins the selection when every body it names
//      is in it, a background anchor comes along, and one foot outside leaves it out.
//   2. the lasso catches a body by its CENTRE, and the tap toggles one in and out.
//   3. a rigid transform (translate + turn) turns every selected body by the box's
//      own delta, holds every distance between them, and leaves every member
//      coupling exactly as satisfied and as stressed as it was -- including the two
//      that measure against the fixed world, a belt's phase and a background
//      rotational spring's rest angle.
//   4. ...and a coupling with a foot outside the selection is NOT carried, which is
//      what makes the box honest about cutting through a machine.
//   5. scaling spreads the parts without resizing them, and the machine stays
//      assembled -- the captured lengths and stations are re-read, not multiplied.
//   6. the box is a frame, not an accumulation: angle 0 and scale 1 put the
//      selection back exactly where it was picked.
//   7. a selection exports as a fragment that pastes back congruent, with fresh body
//      ids, leaving the original untouched; and the same text loads standalone as an
//      ordinary scene file.
//   8. a fragment naming a body it does not define is refused, and the refusal
//      leaves the bench exactly as it was.
//   9. Reset still restores a pasted widget -- the paste built real scene objects,
//      not a copy the ledger does not know about.
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
for(const f of ['js/state.js','js/geometry.js','js/constraints.js','js/solver.js',
                'js/physics.js','js/projection.js','js/loop.js','js/render.js','js/hud.js',
                'js/tools.js','js/inspector.js','js/examples.js','js/scene.js','js/select.js',
                'js/transport.js'])
  vm.runInContext(fs.readFileSync(path.join(ROOT,f),'utf8'), ctx, {filename:f});
const run = s => vm.runInContext(s, ctx);

let pass=0, fail=0;
const ok=(name,good,detail)=>{ good?pass++:fail++;
  console.log((good?'  ok  ':'  FAIL'), name, good?'':('\n        '+detail)); };
const near=(a,b,tol)=>Math.abs(a-b)<=(tol===undefined?1e-9:tol);

// The bench every check below runs on. Bodies 1-3 are the widget: 1 is pinned to the
// ground by a double-welded rod (the only thing that pins anything, §06.2b), 2 hangs
// off it on a plain rod and carries a spring to a fixed point, 3 is belted to 2 and
// held by a rotational spring against the world's theta = 0. Body 4 is OUTSIDE, and
// a rod from 3 reaches it -- the coupling that must not be carried.
run(`sim.running=false;
var BENCH = () => { clearScene(); sim.gravity=false; cam.x=0; cam.y=2; cam.scale=64;
  const b1=makeBody(0,2,0.3), b2=makeBody(1,2,0.3), b3=makeBody(2.4,2,0.4), b4=makeBody(5,2,0.3);
  bodies.push(b1,b2,b3,b4);
  constraints.push(makeRodCon({id:null,off:[0,3]}, {id:b1.id,off:[0,0]}, true, true));
  constraints.push(makeRodCon({id:b1.id,off:[0,0]}, {id:b2.id,off:[0.1,0]}, false, true));
  constraints.push(makeBeltCon(b2.id, b3.id, 1));
  constraints.push(makeRodCon({id:b3.id,off:[0,0]}, {id:b4.id,off:[0,0]}, false, false));
  springs.push(makeSpringCon({id:b2.id,off:[0,0]}, {id:null,off:[1,3.2]}));
  rotSprings.push(makeRotSpringCon(null, b3.id));
  interactions.push(makeInteraction('heat', b1.id, null));
  refreshFrozen(); saveState(true);
  return [b1.id,b2.id,b3.id,b4.id];
};
var WIDGET = () => { const ids=BENCH(); selectGroup(ids.slice(0,3)); return ids; };
var poseOf = () => bodies.map(b=>[b.id,b.x,b.y,b.th]);
var maxC = () => Math.max(0, ...constraints.map(c=>conMaxC(c)));
var memberMaxC = () => Math.max(0, ...selGroup.m.constraints.map(c=>conMaxC(c)));
var rsTorque = rs => rs.k*(rs.restAngle - rotSpringRelAngle(rs));
var beltC = c => (c.rA*bodies[bodyIndex(c.a.id)].th - c.sense*c.rB*bodies[bodyIndex(c.b.id)].th) - c.restPhase;
`);

console.log('\n1. the membership rule');
{
  run('WIDGET()');
  const m = run(`(()=>{const m=selGroup.m; return {b:m.bodies.map(x=>x.id),
    cons:m.constraints.map(c=>c.type+':'+conEndpoints(c).map(e=>e.id===null?'bg':e.id).join('-')),
    springs:m.springs.length, rot:m.rotSprings.length, it:m.interactions.length};})()`);
  ok('the three picked bodies are the group', JSON.stringify(m.b)==='[1,2,3]', JSON.stringify(m.b));
  ok('the grounding rod comes along (background end and all)',
     m.cons.includes('rod:bg-1'), m.cons.join(' | '));
  ok('the rod between two selected bodies comes along', m.cons.includes('rod:1-2'), m.cons.join(' | '));
  ok('the belt between two selected bodies comes along', m.cons.includes('belt:2-3'), m.cons.join(' | '));
  ok('the rod reaching body 4 does NOT', !m.cons.includes('rod:3-4'), m.cons.join(' | '));
  ok('the background-anchored spring comes along', m.springs===1, String(m.springs));
  ok('the background-referenced rotational spring comes along', m.rot===1, String(m.rot));
  ok('the background heat interaction comes along', m.it===1, String(m.it));
  const solo = run(`(()=>{ selectGroup([3]); const m=selGroup.m;
    return {cons:m.constraints.map(c=>c.type), rot:m.rotSprings.length}; })()`);
  ok('selecting body 3 alone drops the belt (body 2 is out)',
     !solo.cons.includes('belt') && solo.rot===1, JSON.stringify(solo));
}

console.log('\n2. the lasso');
{
  run('BENCH()');
  // a loop around x in [-1, 3], y in [1, 3]: bodies 1, 2 and 3, not 4
  const got = run(`(()=>{ const g=lassoSelect([[-1,1],[3,1],[3,3],[-1,3]]);
    return g ? [...g.ids].sort((a,b)=>a-b) : null; })()`);
  ok('the loop catches every body whose centre is inside it',
     JSON.stringify(got)==='[1,2,3]', JSON.stringify(got));
  // body 3's centre is at x = 2.4 with r = 0.4: a loop ending at 2.2 overlaps it but
  // does not contain its centre, and the rule is the centre.
  const tight = run(`(()=>{ const g=lassoSelect([[-1,1],[2.2,1],[2.2,3],[-1,3]]);
    return g ? [...g.ids].sort((a,b)=>a-b) : null; })()`);
  ok('a body the loop only grazes is not caught -- the test is its centre',
     JSON.stringify(tight)==='[1,2]', JSON.stringify(tight));
  const added = run(`(()=>{ lassoToggle(2.4,2); return [...selGroup.ids].sort((a,b)=>a-b); })()`);
  ok('a tap on a body adds it to the group', JSON.stringify(added)==='[1,2,3]', JSON.stringify(added));
  const dropped = run(`(()=>{ lassoToggle(2.4,2); return [...selGroup.ids].sort((a,b)=>a-b); })()`);
  ok('...and a second tap takes it out again', JSON.stringify(dropped)==='[1,2]', JSON.stringify(dropped));
}

console.log('\n3. a rigid transform: the box carries the bodies');
{
  run('WIDGET()');
  const before = run(`(()=>({ pose:poseOf(), c:maxC(), belt:beltC(constraints[2]),
     tau:rsTorque(rotSprings[0]), rest:springs[0].restLen, rodLen:constraints[1].len,
     ang:constraints[1].restAngB }))()`);
  const DTH=0.9;
  run(`groupSetFrame(selGroup.cx+0.7, selGroup.cy-0.4, ${DTH}, 1)`);
  const after = run(`(()=>({ pose:poseOf(), c:memberMaxC(), belt:beltC(constraints[2]),
     tau:rsTorque(rotSprings[0]), rest:springs[0].restLen, rodLen:constraints[1].len }))()`);
  const dth = after.pose.filter(p=>p[0]<4).map((p,i)=>p[3]-before.pose[i][3]);
  ok('every selected body turned by the box\'s own change in angle',
     dth.every(d=>near(d,DTH)), JSON.stringify(dth));
  ok('body 4, outside the selection, did not move',
     near(after.pose[3][1],before.pose[3][1]) && near(after.pose[3][3],before.pose[3][3]),
     JSON.stringify([before.pose[3], after.pose[3]]));
  const dist=(p,q)=>Math.hypot(p[1]-q[1], p[2]-q[2]);
  ok('the distances between the selected bodies are unchanged',
     near(dist(after.pose[0],after.pose[1]), dist(before.pose[0],before.pose[1]), 1e-12) &&
     near(dist(after.pose[1],after.pose[2]), dist(before.pose[1],before.pose[2]), 1e-12),
     'pairwise distance moved');
  ok('every member coupling is still satisfied', after.c<1e-9, String(after.c));
  ok('the belt did not slip: its phase followed the turn',
     near(after.belt, before.belt, 1e-12), `${before.belt} -> ${after.belt}`);
  ok('the background rotational spring carries the same torque',
     near(after.tau, before.tau, 1e-12), `${before.tau} -> ${after.tau}`);
  ok('the spring\'s rest length is untouched by a rigid move',
     near(after.rest, before.rest, 1e-15), `${before.rest} -> ${after.rest}`);
  ok('the rod\'s captured length is untouched by a rigid move',
     near(after.rodLen, before.rodLen, 1e-12), `${before.rodLen} -> ${after.rodLen}`);
}

console.log('\n4. what the box does not carry');
{
  const cut = run(`conMaxC(constraints[3])`);
  ok('the rod reaching outside the selection now reads as violated', cut>0.1, String(cut));
  ok('...so the reset baseline was not taken from that pose', run('constraintsSatisfied()')===false, 'satisfied');
}

console.log('\n5. scaling spreads the parts without resizing them');
{
  run('WIDGET()');
  const S=1.7;
  const before = run(`(()=>({ pose:poseOf(), r:bodies.map(b=>b.r), rodLen:constraints[1].len,
     rest:springs[0].restLen }))()`);
  run(`groupSetFrame(selGroup.cx, selGroup.cy, 0, ${S})`);
  const after = run(`(()=>({ pose:poseOf(), r:bodies.map(b=>b.r), rodLen:constraints[1].len,
     rest:springs[0].restLen, c:memberMaxC() }))()`);
  const dist=(p,q)=>Math.hypot(p[1]-q[1], p[2]-q[2]);
  ok('the centres spread by exactly the scale',
     near(dist(after.pose[0],after.pose[1]), S*dist(before.pose[0],before.pose[1]), 1e-12),
     `${dist(before.pose[0],before.pose[1])} -> ${dist(after.pose[0],after.pose[1])}`);
  ok('no body changed size', after.r.every((r,i)=>near(r, before.r[i], 0)), JSON.stringify(after.r));
  ok('the machine is still assembled -- the captured geometry was re-read',
     after.c<1e-9, String(after.c));
  ok('the rod\'s length is the distance its ends actually sit at, not the old one times the scale',
     !near(after.rodLen, S*before.rodLen, 1e-9) && after.rodLen>before.rodLen,
     `${before.rodLen} * ${S} = ${S*before.rodLen}, got ${after.rodLen}`);
  ok('the spring\'s rest length -- a length the element owns -- scales instead',
     near(after.rest, S*before.rest, 1e-12), `${before.rest} -> ${after.rest}`);
}

console.log('\n6. the box is a frame, not an accumulation');
{
  run('WIDGET()');
  const before = run('JSON.stringify(poseOf())');
  run(`groupSetFrame(selGroup.cx+2, selGroup.cy+1, 1.3, 2.2)`);
  run(`groupSetFrame(selGroup.cx-2, selGroup.cy-1, 0, 1)`);
  const after = run('JSON.stringify(poseOf())');
  ok('angle 0 and scale 1 put the selection back exactly where it was picked',
     before===after, `${before}\n        ${after}`);
  ok('...and every constraint with it', run('maxC()')<1e-9, run('String(maxC())'));
}

console.log('\n7. a selection copies, pastes and stands alone');
{
  run('WIDGET()');
  const text = run('selectionFragment()');
  ok('the fragment carries no sim and no cam line',
     !/^\s*(sim|cam)\b/m.test(text), text);
  ok('...and opens as a scene file', /^scene 3/.test(text), text.split('\n')[0]);
  const shape = run(`(()=>{ const g=selGroup;
    const P=g.m.bodies.map(b=>[b.x,b.y,b.th]);
    return JSON.stringify([Math.hypot(P[0][0]-P[1][0],P[0][1]-P[1][1]),
                           Math.hypot(P[1][0]-P[2][0],P[1][1]-P[2][1])]); })()`);
  const res = run(`pasteWidget(${JSON.stringify(text)}, 0, -3)`);
  ok('the paste reports what it built', res.ok && /3 bodies/.test(res.text), JSON.stringify(res));
  const outcome = run(`(()=>{
    const ids=[...selGroup.ids].sort((a,b)=>a-b);
    const P=selGroup.m.bodies.map(b=>[b.x,b.y,b.th]);
    return { n:bodies.length, ids, cons:constraints.length, springs:springs.length,
      rot:rotSprings.length, it:interactions.length,
      shape:JSON.stringify([Math.hypot(P[0][0]-P[1][0],P[0][1]-P[1][1]),
                            Math.hypot(P[1][0]-P[2][0],P[1][1]-P[2][1])]),
      cx:selGroup.cx, cy:selGroup.cy, c:maxC(), oldMoved:[bodies[0].x,bodies[0].y] }; })()`);
  ok('the bench grew by three bodies', outcome.n===7, String(outcome.n));
  ok('the pasted bodies took fresh ids, none of them the originals\'',
     outcome.ids.length===3 && outcome.ids.every(id=>id>4), JSON.stringify(outcome.ids));
  ok('every coupling of the widget came with it (5 constraints, 1 spring, 1 rotspring, 1 interaction)',
     outcome.cons===4+3 && outcome.springs===2 && outcome.rot===2 && outcome.it===2,
     JSON.stringify(outcome));
  ok('the copy is congruent to the original', outcome.shape===shape, `${shape}\n        ${outcome.shape}`);
  ok('the copy landed where it was asked to', near(outcome.cx,0,1e-9) && near(outcome.cy,-3,1e-9),
     `${outcome.cx}, ${outcome.cy}`);
  ok('the original did not move', near(outcome.oldMoved[0],0) && near(outcome.oldMoved[1],2),
     JSON.stringify(outcome.oldMoved));
  ok('and every constraint in the bench is satisfied', outcome.c<1e-9, String(outcome.c));
  const alone = run(`(()=>{ importScene(${JSON.stringify(text)});
    return {n:bodies.length, cons:constraints.length, c:Math.max(0,...constraints.map(conMaxC))}; })()`);
  ok('the same text loads on its own as an ordinary scene',
     alone.n===3 && alone.cons===3 && alone.c<1e-9, JSON.stringify(alone));
}

console.log('\n8. a fragment that names a body it does not define is refused');
{
  run('BENCH()');
  const before = run('JSON.stringify(poseOf())');
  const bad = 'scene 3\n\n# bodies\nbody 1 x=0 y=0 r=0.2\n\n# constraints\nrod 1 -- 9 len=1\n';
  const caught = run(`(()=>{ try{ pasteFragment(${JSON.stringify(bad)}); return null; }
    catch(e){ return String(e.message||e); } })()`);
  ok('the paste raises, naming the line and the missing body',
     caught && /^line \d+:/.test(caught) && /body 9/.test(caught), String(caught));
  ok('and the bench is exactly as it was', run('JSON.stringify(poseOf())')===before, 'the bench moved');
}

console.log('\n9. Reset restores a pasted widget too');
{
  run('WIDGET()');
  const text = run('selectionFragment()');
  run(`pasteWidget(${JSON.stringify(text)}, 0, -3); clearSelection(); saveState(true);`);
  const at = run('JSON.stringify(poseOf())');
  run('setRunning(true); for(let i=0;i<40;i++) substep(sim.h); setRunning(false);');
  const moved = run('JSON.stringify(poseOf())');
  run('restoreState()');
  ok('the run moved things', moved!==at, 'nothing moved, so the check proves nothing');
  ok('Reset puts the pasted bodies back with the rest', run('JSON.stringify(poseOf())')===at,
     'a pasted body did not come back');
}

console.log(`\n${pass} ok, ${fail} failed\n`);
process.exit(fail?1:0);
