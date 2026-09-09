// The APP surface: every example through the real render and the real inspector.
//
// Every other script here stubs those two out -- `function renderInspector(){}` and no
// canvas -- because their claims are about rows and files. That leaves a gap nothing
// covered, and it is not hypothetical: retiring the rod, slot, rack and spring left
// four separate paint- and panel-path faults that every one of the eleven other
// scripts passed straight over, because none of them ever called `render()` on a
// bundled example or opened a panel on a selected joint.
//
// So this one loads the whole engine against a stub DOM, and for every bundled
// example: renders it, runs it, renders it again, opens the panel on every object in
// it one at a time, resets, and renders once more. Then it drives every tool through
// its whole gesture, redrawing after each tap. It asserts nothing about what any of
// that DRAWS -- there is no canvas to look at -- only that none of it throws, which is
// the failure mode a headless check can honestly catch and the one that takes the
// page down: an exception inside render() kills the rAF chain outright (code §10), and the
// bench stops redrawing for good.
//
// A stub that swallows ANYTHING cannot catch that, and twice now it has not. Two
// things it therefore does not swallow:
//
//   * the canvas has a SIZE. `W()`/`H()` read cv.clientWidth/Height, and undefined
//     there makes every derived coordinate NaN -- which silently skips the grid, the
//     rail drawn across the viewport and a static body's hatch, three of the loops
//     most able to run away. It is a phone: 414x760 css px at dpr 2.
//   * the 2d context REFUSES what a browser refuses. A negative arc radius throws
//     IndexSizeError on a real canvas and does nothing on a permissive stub, and the
//     same goes for a non-finite gradient and an out-of-range colour stop. Drawing
//     ops are also counted, so a loop that would hang the tab fails here instead.
//
// The gesture pass is the other half of it. A tool's SECOND tap can leave state the
// next frame chokes on -- that is exactly how the line tool once left a `pending`
// with no world point in it, after which every frame threw and the bench never
// painted again while the panel and the scene box carried on working.
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
// ---- the canvas that refuses what a browser refuses ----
const CSS_W=414, CSS_H=760, DPR=2;               // a phone, which is where it was found
const OP_CAP=4e6;                                // one frame's worth, generously
let ops=0;
const draw = () => { if(++ops > OP_CAP) throw new Error(`runaway draw: over ${OP_CAP} canvas ops in one frame`); };
const finite = (...a) => a.every(v => typeof v==='number' && isFinite(v));
const GRAD = { addColorStop(o){ if(!(typeof o==='number' && isFinite(o) && o>=0 && o<=1))
                 throw new Error(`IndexSizeError: colour stop at ${o}`); } };
const CTX2D = {
  arc(x,y,r){ draw(); if(typeof r==='number' && r<0) throw new Error(`IndexSizeError: arc radius ${r}`); },
  ellipse(x,y,rx,ry){ draw(); if(rx<0||ry<0) throw new Error('IndexSizeError: ellipse radius'); },
  arcTo(x1,y1,x2,y2,r){ draw(); if(r<0) throw new Error(`IndexSizeError: arcTo radius ${r}`); },
  roundRect(x,y,w,h,r){ draw(); for(const v of (Array.isArray(r)?r:[r]))
    if(typeof v==='number' && v<0) throw new Error(`RangeError: roundRect radius ${v}`); },
  createLinearGradient(...a){ if(!finite(...a)) throw new Error(`NotSupportedError: gradient ${a}`); return GRAD; },
  createRadialGradient(x0,y0,r0,x1,y1,r1){ if(!finite(x0,y0,r0,x1,y1,r1)) throw new Error('NotSupportedError: gradient');
    if(r0<0||r1<0) throw new Error('IndexSizeError: gradient radius'); return GRAD; },
  measureText(t){ draw(); return {width:String(t).length*6, actualBoundingBoxAscent:8, actualBoundingBoxDescent:2}; },
  getImageData(){ return {data:[0,0,0,0], width:1, height:1}; },
  getLineDash(){ return []; },
};
const ctx2d = new Proxy(CTX2D, { get:(t,k)=> k in t ? t[k] : draw, set:(t,k,v)=>{ t[k]=v; return true; } });
const els = new Map();
function stubEl(id){
  const el = { id, clientWidth:CSS_W, clientHeight:CSS_H,
               width:Math.round(CSS_W*DPR), height:Math.round(CSS_H*DPR),
               style:{}, dataset:{}, value:'', textContent:'', innerHTML:'',
               getContext:()=>ctx2d, classList:{add(){},remove(){},toggle(){}},
               querySelectorAll:()=>[] };
  return new Proxy(el, { get:(t,k)=> k in t ? t[k] : ()=>{},
    set:(t,k,v)=>{
      // Whatever the markup just written mentions is now live, and nothing else is.
      if(k==='innerHTML'){
        liveIds = new Set(STATIC_IDS);
        for(const m of String(v).matchAll(/\bid="([^"]+)"/g)) liveIds.add(m[1]);
      }
      t[k]=v; return true; } });
}
// One object per id, kept: `cv` is captured once at load (code §04.1) and render()
// writes its width back, so a fresh proxy per lookup would lose the size it just set.
const byId = id => { if(!liveIds.has(id)) return null;
  if(!els.has(id)) els.set(id, stubEl(id));
  return els.get(id); };
const ctx = vm.createContext({
  document:{ getElementById:byId, createElement:()=>stubEl('x'),
             querySelectorAll:()=>[], addEventListener(){}, activeElement:null },
  window:{addEventListener(){}, devicePixelRatio:DPR}, performance:{now:()=>0}, console,
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
  ops=0;
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

// ---- 2b · the whole GESTURE, with a frame drawn between every tap ----
// A tool is not one click. The two- and three-tap tools leave state behind between
// taps -- a `pending` first pick, a `bodyPreview` -- and that state is DRAWN on every
// frame until the gesture finishes. So each tool is tapped five times over a bench it
// can act on, with a hover and a render after every tap, which is the frame a real
// user would get. Five is past the longest gesture (the rack's three), so every tool
// also runs off the end of its own cycle and starts another, which is where state left
// over from the previous one shows up.
//
// Rendering is not the whole of the check, because a painter that has been taught to
// tolerate a malformed pending (render.js §11.7 does, since an exception there is
// fatal to the page) would hide the tool that made one. So the CONTRACT is asserted
// where it is made: a pending first pick carries the world point of the pick, because
// that is the dot drawn and the end of the lead line to the cursor. A tool that leaves
// one without it fails here whether or not anything downstream survives it.
console.log('\n2b. every tool, tapped through its whole gesture, drawing every frame');
const TAPS = [[0.3,0.4],[-1.4,0.9],[0.9,-0.6],[-2.0,1.5],[1.6,1.1]];
const PENDING_OK = `(()=>{ if(!pending) return null;
  const w=pending.wp;
  return (w && w.length===2 && Number.isFinite(w[0]) && Number.isFinite(w[1]))
    ? null : 'pending {'+Object.keys(pending).join(',')+'} carries no world point'; })()`;
const gesture = (t, open, tail) =>
  `(()=>{ ${open} setTool(${JSON.stringify(t)});
     const bad=[];
     for(const [x,y] of ${JSON.stringify(TAPS)}){
       updateHover(x,y); render();
       runToolClick(x,y);
       const b=${PENDING_OK}; if(b && !bad.includes(b)) bad.push(b);
       ${tail}
     }
     if(bad.length) throw new Error(bad.join('; ')); })()`;
for(const t of JSON.parse(run('JSON.stringify(TOOLS.map(t=>t.id))'))){
  tries(`${t.padEnd(12)}five taps, a frame after each`,
    gesture(t, `loadExample('rack');`, `updateHover(x,y); render(); updateHUD(); renderInspector();`));
}
// ...and the same on an EMPTY bench, where a tool has nothing to pick and every
// gesture has to plant what it needs from nothing. Both orders matter: the line tool's
// fault only appeared once its second tap had actually made a line.
for(const t of JSON.parse(run('JSON.stringify(TOOLS.map(t=>t.id))'))){
  tries(`${t.padEnd(12)}...from an empty bench`,
    gesture(t, `clearScene();`, `updateHover(x,y); render(); updateHUD();`));
}
// A scene load in the middle of a gesture: whatever the first tap picked is about to
// be thrown away, and the frame after the load must not go looking for it.
tries('a scene load mid-gesture leaves nothing dangling',
  `(()=>{ loadExample('rack'); setTool('line'); runToolClick(0.3,0.4); render();
     loadExample('fourbar'); render(); updateHUD();
     clearScene(); render(); updateHUD(); })()`);
run(`setTool('select')`);

console.log('\n3. the empty bench, and the panels that only exist there');
tries('an empty bench renders', `clearScene(); render(); updateHUD(); renderInspector();`);
tries('...and the scene card writes its own text', `sceneCardText().length>0 || (()=>{throw new Error('empty')})()`);
tries('a lasso selection opens the group panel',
  `loadExample('fourbar'); selectGroup(bodies.map(b=>b.id)); render(); renderInspector(); clearSelection();`);

console.log(`\n${pass} ok, ${fail} failed\n`);
process.exit(fail?1:0);
