// The LINE FRAME (constraints.js §06.1b) -- a frame with no coordinates of its own.
//
// A rod, a slot and a rack are all one straight massless bar, and phase 0 of the
// vertex/line plan (VERTEX.md §X.5, §X.12) factors that bar out as a FRAME: it has a
// heading and a material point at every station, but no coordinates, so its columns
// are a combination of the columns of the two endpoints that define it. Everything
// downstream then talks to a line through the same interface epFrame gives a body
// and the background -- which is the one change the plan needs the row builders to
// make, so it is the one thing worth checking before anything is built on it.
//
// Like rack-check.js and multipoint-check.js beside it, this one LOADS the
// simulator: the claim is about the real rows.
//
//   1. the frame's angCols really is d(phi)/dt -- checked against a finite
//      difference of the live geometry, for a plain pair, for a pair with one end on
//      the background, and for a vessel endpoint (whose columns carry the fourth,
//      length coordinate, and which is the case an inlined copy of this arithmetic
//      would be most likely to get wrong).
//   2. the frame's material point at (station s, lateral t) moves as a point of a
//      rigid body does -- also against a finite difference, at several stations and
//      both on and off the line.
//   3. every row every constraint kind builds carries a ROLE tag, and the tags are
//      the ones §09.3 asks for. An untagged row is a multiplier the reaction readout
//      can no longer find, which is exactly the failure the tags exist to prevent.
//   4. the reaction readout finds a real multiplier for every joint that carries
//      one -- no silent undefined where a force used to be reported.
const fs=require('fs'), path=require('path'), vm=require('vm');
const ROOT=path.join(__dirname,'..');
const stubEl = () => new Proxy({}, { get:(t,k)=>
  k==='getContext' ? ()=>new Proxy({},{get:()=>()=>{}}) :
  k==='classList'  ? {add(){},remove(){},toggle(){}} :
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

// ---- shared helpers, defined once inside the context -------------------------
// A column set dotted with the world's live velocities: what the row says the
// quantity's rate of change is. The fourth entry is a vessel's length rate.
run(`var dotCols = cols => { let s=0;
  for(const [i,cx,cy,cw,cl] of cols){ const b=bodies[i];
    s += cx*b.vx + cy*b.vy + cw*b.w + (cl||0)*(b.vlen||0); }
  return s; };
// Step every body forward by dt at its current velocity -- a pure kinematic move,
// no forces, no solve, so the finite difference measures the frame and nothing else.
var glide = dt => { for(const b of bodies){
    b.x+=b.vx*dt; b.y+=b.vy*dt; b.th+=b.w*dt;
    if(b.shape==='vessel'){ b.len+=(b.vlen||0)*dt; refreshVessel(b); } } };
`);

console.log('\n1. the line frame\'s angCols is d(phi)/dt');
// Three benches: two free bodies, one end on the background, and a vessel endpoint.
const ANG = [
  ['two free bodies', `
    const a=makeBody(0,0,0.2); a.vx=0.13; a.vy=-0.07; a.w=0.9; bodies.push(a);
    const b=makeBody(2.1,0.4,0.2); b.vx=-0.05; b.vy=0.21; b.w=-0.4; bodies.push(b);
    constraints.push(makeRodCon({id:a.id,off:[0.05,-0.03]},{id:b.id,off:[-0.02,0.06]},false,false));`],
  ['one end on the background', `
    const b=makeBody(1.7,-0.6,0.25); b.vx=0.22; b.vy=0.31; b.w=1.2; bodies.push(b);
    constraints.push(makeRodCon({id:null,off:[0,0]},{id:b.id,off:[0.04,0.01]},false,false));`],
  ['a vessel endpoint (the length column)', `
    const v=makeVessel(0,0,0.5,1.4); v.vx=0.09; v.vy=0.02; v.w=0.55; v.vlen=0.37; bodies.push(v);
    const b=makeBody(2.3,0.5,0.2); b.vx=-0.11; b.vy=0.04; b.w=0.3; bodies.push(b);
    constraints.push(makeRodCon({id:v.id,off:[0.1,0.4]},{id:b.id,off:[0,0]},false,false));`],
];
for(const [what, build] of ANG){
  run(`(()=>{ clearScene(); ${build} refreshFrozen(); })()`);
  // twoPointFrame unwraps phi against con._phiRef, so read both ends through it and
  // the difference is the real turn rather than an atan2 branch jump.
  const rate = Number(run(`(()=>{
    const con=constraints[0];
    const f=twoPointFrame(con);
    return dotCols(lineFrameOf(f).angCols());
  })()`));
  const fd = Number(run(`(()=>{
    const con=constraints[0], dt=1e-6;
    const p0=twoPointFrame(con).phi;
    glide(dt);
    const p1=twoPointFrame(con).phi;
    glide(-dt);
    return (p1-p0)/dt;
  })()`));
  ok(`${what}: angCols.v = ${rate.toFixed(9)} vs finite difference ${fd.toFixed(9)}`,
     Math.abs(rate-fd) < 1e-5*Math.max(1,Math.abs(fd)), `${rate} vs ${fd}`);
}

console.log('\n2. the frame\'s material point moves as a rigid body\'s does');
// P(s,t) = P_a + s*u + t*n. Its velocity, probed along u and along n, is what
// minusPointAlong/Across return (negated -- they are built for rows that measure
// something RELATIVE to the bar). Checked away from the line as well as on it,
// because the off-line term is the one a transient actually exercises.
for(const [s,t] of [[0,0],[0.7,0],[-1.3,0],[0.9,0.4],[-0.5,-0.25]]){
  run(`(()=>{ clearScene();
    const a=makeBody(0,0,0.2); a.vx=0.13; a.vy=-0.07; a.w=0.9; bodies.push(a);
    const b=makeBody(2.1,0.4,0.2); b.vx=-0.05; b.vy=0.21; b.w=-0.4; bodies.push(b);
    constraints.push(makeRodCon({id:a.id,off:[0.05,-0.03]},{id:b.id,off:[-0.02,0.06]},false,false));
    refreshFrozen(); })()`);
  const got = JSON.parse(run(`(()=>{
    const f=twoPointFrame(constraints[0]), LF=lineFrameOf(f);
    // minusPoint* are negated and returned as term LISTS -- merge and flip to read
    // the material point's own velocity along each probe direction.
    const vAlong  = -dotCols(mergeCols(LF.minusPointAlong(${s},${t})));
    const vAcross = -dotCols(mergeCols(LF.minusPointAcross(${s},${t})));
    return JSON.stringify([vAlong, vAcross]);
  })()`));
  const fd = JSON.parse(run(`(()=>{
    const con=constraints[0], dt=1e-6;
    const at = () => { const f=twoPointFrame(con);
      return [f.wax + ${s}*f.ux + ${t}*f.nx, f.way + ${s}*f.uy + ${t}*f.ny, f.ux, f.uy, f.nx, f.ny]; };
    const p0=at(); glide(dt); const p1=at(); glide(-dt);
    const vx=(p1[0]-p0[0])/dt, vy=(p1[1]-p0[1])/dt;
    return JSON.stringify([vx*p0[2]+vy*p0[3], vx*p0[4]+vy*p0[5]]);
  })()`));
  const near=(a,b)=>Math.abs(a-b) < 1e-5*Math.max(1,Math.abs(b));
  ok(`station ${s}, lateral ${t}: v.u and v.n match the finite difference`,
     near(got[0],fd[0]) && near(got[1],fd[1]),
     `frame [${got}]  vs  finite difference [${fd}]`);
}

console.log('\n3. every row carries a role, and only roles §09.3 knows');
// One bench per kind, each carrying every optional row that kind can build.
const KINDS = `scene 3
sim gravity=on
cam x=0 y=0 scale=64
body 1 x=0 y=0 r=0.2
body 2 x=3 y=0.1 r=0.2
body 3 x=1 y=0.03 r=0.15
body 4 x=2 y=0.07 r=0.15
body 5 x=1.5 y=0.5 r=0.4
vessel 6 x=6 y=0 bore=0.5 len=1.2 P=101325 T=293.15
rod 1 -- 2 len=3.0016662 weld=both restAngA=0.2 restAngB=-0.3 pt=3/s=2/lock/restAng=0.1 pt=4/s=1
slot 1 -- 2 lock=both restAngA=0.1 restAngB=0.1 pt=3/lock/restAng=0.05 pt=4
rack 1 -- 2 weld=both restAngA=0.1 restAngB=0.2 pt=5/pinion pt=4/s=1
pin 1@(0.5,0) -- 2@(-2.5,0) pt=3@(-0.5,0)
belt 1 -- 2 rA=0.2 rB=0.2 restPhase=0
cvt 1 -- 2
knife 3 dir=(1,0)
`;
{
  const KNOWN = ['dist','weld','online','station','lateral','mesh','pin','belt','cvt','knife','drag'];
  run(`importScene(${JSON.stringify(KINDS)}); setRunning(true); substep(sim.h);`);
  const seen = JSON.parse(run(`JSON.stringify(constraints.map(c=>[c.type, rowsFor(c).map(r=>r.role||null)]))`));
  const untagged = seen.filter(([,rs])=>rs.some(r=>r===null)).map(([t])=>t);
  ok('no kind builds an untagged row', untagged.length===0, 'untagged rows on: '+untagged.join(', '));
  const unknown = [...new Set(seen.flatMap(([,rs])=>rs))].filter(r=>r && !KNOWN.includes(r));
  ok('and no kind builds a role §09.3 does not know', unknown.length===0, 'unknown roles: '+unknown.join(', '));
  // The roles physics records beside the multipliers must line up one-for-one with
  // the rows -- that pairing is what the lookup indexes through.
  const paired = run(`constraints.every(c=>c._compiled || (c._roles.length===c._rows.length
                       && c._roles.length===rowsFor(c).length))`);
  ok('and the recorded roles pair one-for-one with the multipliers', paired===true, String(paired));
}

console.log('\n4. the reaction readout finds a real multiplier');
{
  // Every joint that carries rows must report a finite reaction -- a role looked up
  // and not found reads as undefined, which arithmetic turns into NaN rather than
  // into an error, so this is the check that a missing tag cannot pass quietly.
  const bad = JSON.parse(run(`(()=>{
    const out=[];
    for(const c of constraints){
      if(c._compiled || !c._lam || !c._lam.length) continue;
      const r=reactionOf(c); if(!r){ out.push([c.type,'no readout']); continue; }
      for(const k of ['x','y','fx','fy','tau','val'])
        if(r[k]!==undefined && !isFinite(r[k])) out.push([c.type, k+'='+r[k]]);
    }
    return JSON.stringify(out);
  })()`));
  ok('every joint reports a finite reaction', bad.length===0, JSON.stringify(bad));
  // ...and the torque a line joint reports is its own END's lock, never a control
  // point's, which is a distinction the old row-order counting made by construction
  // and a role lookup has to be told.
  const tau = JSON.parse(run(`(()=>{
    clearScene();
    const a=makeBody(0,0,0.2); bodies.push(a);
    const b=makeBody(2,0,0.2); bodies.push(b);
    const c=makeBody(1,0,0.15); bodies.push(c);
    const rod=makeRodCon({id:a.id,off:[0,0]},{id:b.id,off:[0,0]},false,false);  // NEITHER end welded
    makeConPoint(rod, {id:c.id, off:[0,0]}, {lock:true});                       // but the point IS
    constraints.push(rod); refreshFrozen(); setRunning(true); substep(sim.h);
    return JSON.stringify(reactionOf(constraints[0]).tau===undefined);
  })()`));
  ok('a rod with no welded end reports no torque, though its point is locked', tau===true, String(tau));
}

console.log(`\n${pass} ok, ${fail} failed\n`);
process.exit(fail?1:0);
