// The APP surface: every example through the real render and the real inspector.
//
// Every other script here stubs those two out -- `function renderInspector(){}` and no
// canvas -- because their claims are about rows and files. That leaves a gap nothing
// covered, and it is not hypothetical: retiring the rod, slot, rack and spring left
// four separate paint- and panel-path faults that every one of the eleven other
// scripts passed straight over, because none of them ever called `render()` on a
// bundled example or opened a panel on a selected joint.
//
// So this one loads the whole engine against a stub DOM wide enough to swallow
// anything, and for every bundled example: renders it, runs it, renders it again,
// opens the panel on every object in it one at a time, resets, and renders once more.
// Then it sets every tool and hovers with each. It asserts nothing about what any of
// that DRAWS -- there is no canvas to look at -- only that none of it throws, which is
// the failure mode a headless check can honestly catch and the one that takes the
// page down: an exception inside render() kills the rAF chain outright (code §10), and the
// bench stops redrawing for good.
const fs=require('fs'), path=require('path'), vm=require('vm');
const ROOT=path.join(__dirname,'..');
// A DOM stub with one real property: it knows which ids EXIST. Every panel is drawn
// by assigning `innerHTML`, and the code then asks for its fields by id -- so a
// getElementById that answers for everything hides exactly the mistakes this script is
// here to find (a live readout reaching for a field the panel it is looking at does
// not have). The stub therefore tracks the ids the markup currently mentions -- the
// static ones from index.html, plus whatever the last innerHTML wrote -- and returns
// null for anything else, which is what a browser does.
const STATIC_IDS = new Set(
  [...fs.readFileSync(path.join(ROOT,'index.html'),'utf8').matchAll(/\bid="([^"]+)"/g)].map(m=>m[1]));
let liveIds = new Set(STATIC_IDS);
const stubEl = (id) => new Proxy({}, { get:(t,k)=>
  k==='getContext'       ? ()=>new Proxy({},{get:()=>()=>{}}) :
  k==='classList'        ? {add(){},remove(){},toggle(){}} :
  k==='querySelectorAll' ? ()=>[] :
  k==='style' || k==='dataset' ? {} :
  k in t ? t[k] : ()=>{},
  set:(t,k,v)=>{
    // Whatever the markup just written mentions is now live, and nothing else is.
    if(k==='innerHTML'){
      liveIds = new Set(STATIC_IDS);
      for(const m of String(v).matchAll(/\bid="([^"]+)"/g)) liveIds.add(m[1]);
    }
    t[k]=v; return true; } });
const byId = id => liveIds.has(id) ? stubEl(id) : null;
const ctx = vm.createContext({
  document:{ getElementById:byId, createElement:()=>stubEl(),
             querySelectorAll:()=>[], addEventListener(){} },
  window:{addEventListener(){}, devicePixelRatio:1}, performance:{now:()=>0}, console,
  Math, JSON, Number, String, Object, Array, Map, Set, Error, RegExp,
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
const tries = (name, src) => {
  let err=null;
  try { run(src); } catch(e){ err=e; }
  ok(name, !err, err && ((err.stack||String(err)).split('\n').slice(0,4).join('\n')));
};

console.log('\n1. every example: render, run, select everything, reset');
for(const ex of JSON.parse(run('JSON.stringify(Object.keys(SCENES))'))){
  tries(ex.padEnd(12)+'loads and renders',
    `loadExample(${JSON.stringify(ex)}); render(); updateHUD();`);
  // Every object in it, one panel at a time. A panel is drawn from the object it is
  // given, so a kind whose panel still names something retired throws here.
  tries(ex.padEnd(12)+'opens a panel on every object',
    `(()=>{
       for(let i=0;i<constraints.length;i++){ selectConstraint(i); render(); updateInspectorLive(); }
       for(let i=0;i<bodies.length;i++){ selectBody(i); render(); updateInspectorLive(); }
       for(let i=0;i<cables.length;i++){ selectCable(i); render(); updateInspectorLive(); }
       for(let i=0;i<rotSprings.length;i++){ selectRotSpring(i); render(); updateInspectorLive(); }
       for(let i=0;i<interactions.length;i++){ selectInteraction(i); render(); updateInspectorLive(); }
       clearSelection(); })()`);
  tries(ex.padEnd(12)+'runs, renders while running, and resets',
    `(()=>{ setRunning(true);
       for(let i=0;i<60;i++){ substep(sim.h); }
       render(); updateHUD();
       setRunning(false); restoreState(); render(); updateHUD(); })()`);
}

console.log('\n2. every tool sets, hovers and draws');
for(const t of JSON.parse(run('JSON.stringify(TOOLS.map(t=>t.id))'))){
  tries(`${t.padEnd(12)}sets and hovers`,
    `loadExample('rack'); setTool(${JSON.stringify(t)});
     updateHover(0.3,0.4); render();
     updateHover(-2,1.5); render();`);
}
run(`setTool('select')`);

console.log('\n3. the empty bench, and the panels that only exist there');
tries('an empty bench renders', `clearScene(); render(); updateHUD(); renderInspector();`);
tries('...and the scene card writes its own text', `sceneCardText().length>0 || (()=>{throw new Error('empty')})()`);
tries('a lasso selection opens the group panel',
  `loadExample('fourbar'); selectGroup(bodies.map(b=>b.id)); render(); renderInspector(); clearSelection();`);

console.log(`\n${pass} ok, ${fail} failed\n`);
process.exit(fail?1:0);
