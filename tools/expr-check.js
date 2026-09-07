// Expressions in numeric fields -- §19, SCENE.md §S.10.
//
// Like scene-roundtrip.js beside it, this one DOES load the simulator: the claim is
// about the real language, the real reader and the real panel field, so a
// reimplementation would test nothing. It loads the sources into a bare context
// with a stub DOM and asserts, in four groups:
//
//   1. the language -- precedence, associativity, the function and constant table,
//      and that every malformed or non-finite expression is an error rather than a
//      quiet NaN;
//   2. a file -- an expression is legal wherever a number is (fields, endpoint
//      offsets, a vec2, a point's station), names resolve against the file itself
//      (including forwards, to a body defined further down, and to a ledger default
//      the line never wrote), and a vessel's fill answers whichever two of its four
//      the line happened to give;
//   3. that expressions are NOT STORED -- what a file is read into is numbers, the
//      export is digits, and a value typed against another body does not follow that
//      body afterwards. That is the whole boundary between this and a constraint;
//   4. refusals -- an unknown name, an unknown property, a body the file does not
//      define, a value defined in terms of itself, division by zero, an ambient
//      written in terms of a body -- each raising, and each leaving the bench that
//      was loaded before it exactly as it was;
//   5. the panel field -- numVal reads arithmetic, marks what it cannot use and
//      keeps that text uncommitted, and the arrow keys step the value.
const fs=require('fs'), path=require('path'), vm=require('vm');
const ROOT=path.join(__dirname,'..');

let pass=0, fail=0;
const ok=(name,good,detail)=>{ good?pass++:fail++;
  console.log((good?'  ok  ':'  FAIL'), name, good?'':('\n        '+(detail||''))); };
const near=(a,b,tol)=>Math.abs(a-b)<=(tol===undefined?1e-9:tol)*Math.max(1,Math.abs(b));

// ---- a DOM thin enough to load the panel, and wide enough to answer for one field
// ---- at a time: elements are registered by id and handed back by both lookups.
const REG = new Map();
const mkEl = (id, attrs={}) => {
  const cls = new Set();
  const el = { id, value:'', title:'', dataset:{}, changes:0,
    classList:{ add:c=>cls.add(c), remove:c=>cls.delete(c),
                toggle:(c,on)=>{ on?cls.add(c):cls.delete(c); }, contains:c=>cls.has(c) },
    dispatchEvent(){ el.changes++; if(el.onchange) el.onchange(); return true; },
    querySelectorAll:()=>[], addEventListener(){}, getContext:()=>({}) };
  Object.assign(el, attrs);
  REG.set(id, el);
  return el;
};
const stubEl = () => new Proxy({}, { get:(t,k)=>
  k==='getContext'      ? ()=>new Proxy({},{get:()=>()=>{}}) :
  k==='classList'       ? {add(){},remove(){},toggle(){}} :
  k==='querySelectorAll'? ()=>[] :
  k in t ? t[k] : ()=>{},
  set:(t,k,v)=>{ t[k]=v; return true; } });
const ctx = vm.createContext({
  document:{ getElementById:id=>REG.get(id) || stubEl(), createElement:()=>stubEl(),
             querySelectorAll:()=>[...REG.values()].filter(e=>e.dataset && e.dataset.step!==undefined),
             addEventListener(){} },
  window:{addEventListener(){}}, performance:{now:()=>0}, console,
  Math, JSON, Number, String, Object, Array, Map, Set, Error, Event:class{constructor(t){this.type=t;}},
  isFinite, NaN, requestAnimationFrame:()=>{},
});
ctx.globalThis = ctx;
vm.runInContext(`function setTool(){} var TOOLS=[];`, ctx);
for(const f of ['js/state.js','js/expr.js','js/geometry.js','js/constraints.js','js/solver.js',
                'js/physics.js','js/projection.js','js/loop.js','js/hud.js','js/render.js',
                'js/inspector.js','js/scene.js','js/examples.js','js/select.js','js/transport.js'])
  vm.runInContext(fs.readFileSync(path.join(ROOT,f),'utf8'), ctx, {filename:f});
const run = src => vm.runInContext(src, ctx);
// Evaluate, or return the message it failed with -- the two things every check below
// wants to tell apart.
const val = src => { try { return run(`evalExpr(${JSON.stringify(src)})`); }
                     catch(e){ return String(e.message||e); } };
const load = text => { try { run(`importScene(${JSON.stringify(text)})`); return null; }
                       catch(e){ return String(e.message||e); } };

console.log('\n1. the language');
for(const [src, want] of [
  ['1+2*3', 7], ['(1+2)*3', 9], ['2-3-4', -5], ['8/4/2', 1],
  ['2^3^2', 512],                       // right-associative
  ['-2^2', -4],                         // a leading minus is weaker than the power
  ['2^-1', 0.5],                        // ...and a trailing one is an exponent
  ['-(3)*-2', 6], ['+4', 4],
  ['pi', Math.PI], ['tau', 2*Math.PI], ['e', Math.E], ['2*pi/3', 2*Math.PI/3],
  ['sin(30*deg)', 0.5], ['cos(pi)', -1], ['atan2(1,1)', Math.PI/4],
  ['sqrt(2)', Math.SQRT2], ['cbrt(27)', 3], ['ln(e)', 1], ['log(1000)', 3], ['log2(8)', 3],
  ['abs(-2.5)', 2.5], ['sign(-3)', -1], ['floor(-1.5)', -2], ['ceil(1.2)', 2], ['round(2.5)', 3],
  ['pow(2,10)', 1024], ['mod(-1,3)', 2], ['clamp(5,0,1)', 1],
  ['min(3,1,2)', 1], ['max(3,1,2)', 3], ['hypot(3,4)', 5],
  ['1e-3', 0.001], ['.5', 0.5], ['2 * 3', 6],   // a panel field may carry spaces
]) ok(`${src} = ${want}`, near(val(src), want, 1e-12), `got ${val(src)}`);

for(const [src, want] of [
  ['', 'empty expression'], ['2+', 'expected a number'], ['2 3', 'unexpected'],
  ['(1+2', 'expected ")"'], ['1/0', 'division by zero'], ['sqrt(-1)', 'not a number'],
  ['1e400', 'not a finite number'], ['nope', 'unknown name "nope"'],
  ['nope(1)', 'unknown function "nope"'], ['sin(1,2)', 'sin takes 1 argument'],
  ['hypot()', 'at least one argument'], ['sin', 'is a function'],
  ['x[0]', 'unexpected character'], ['1;2', 'unexpected character'],
]) ok(`${JSON.stringify(src)} is refused (${want})`, String(val(src)).includes(want), `got ${val(src)}`);
// The grammar is the whole of what a field may say: there is no way through it to
// anything the host offers, which is what makes reading a scene file safe.
for(const src of ['globalThis', 'this', 'bodies', 'sim', 'constructor', 'toString',
                  'Math.PI', 'process', 'eval("1")', 'pi.constructor'])
  ok(`${src} reaches nothing`, /unknown (name|function)|unexpected|is a function/.test(String(val(src))), `got ${val(src)}`);

console.log('\n2. a file reads its own names');
const S = `scene 3

sim gravity=off bg.P=2*101325 bg.T=250+50
cam x=0 y=2 scale=64

# bodies
body 1 x=0 y=2 r=0.5*sqrt(2)
body 2 x=b1.x+2*b1.r y=b1.y-1 r=b1.r/2 mass=b1.mass
rect 3 x=b4.x+1 y=0.5 width=2*pi height=0.1*2
vessel 4 x=-1 y=1 bore=0.4 P=bg.P T=bg.T gasMass=0.02
vessel 5 x=1 y=1 bore=b4.bore len=b4.len*2 T=b4.T P=b4.P

# constraints
rod bg(b1.x,b1.y+3) -- 1 len=hypot(3,0) pt=2/s=1/2
knife 2 dir=(cos(pi/3),sin(pi/3))
spring 1@(b1.r/2,0) -- 2 restLen=2*b1.r k=5*2
`;
ok('the file loads', load(S)===null, load(S));
const B = i => run(`JSON.stringify(bodies[${i}])`) && JSON.parse(run(`JSON.stringify(bodies[${i}])`));
const b1=B(0), b2=B(1), b3=B(2), b4=B(3), b5=B(4);
ok('r = 0.5*sqrt(2)', near(b1.r, 0.5*Math.SQRT2));
ok('a name resolves backwards (b1.x+2*b1.r)', near(b2.x, 0 + 2*0.5*Math.SQRT2));
ok('...and forwards, to a body defined below (b4.x+1)', near(b3.x, 0));
// b1's line never wrote a mass, so `b1.mass` is the ledger's default for it --
// the density-1 disk the constructor would have made, which is the same answer as
// asking the built body.
ok('...and to a default the line never wrote (b1.mass)', near(b2.mass, Math.PI*b1.r*b1.r) && near(b2.mass, Math.PI*0.5));
ok('width = 2*pi', near(b3.hw*2, 2*Math.PI));
ok('the ambient is the file\'s own (bg.P=2*101325)', near(run('sim.bg.P'), 202650) && near(run('sim.bg.T'), 300));
ok('a vessel filled from bg.P/bg.T and a gas mass', near(run(`gasP(bodies[3])`), 202650) && near(run(`gasT(bodies[3])`), 300));
ok('...whose implied len is what the other vessel doubled', near(b5.len, b4.len*2));
ok('an endpoint offset is an expression', near(run('constraints[0].a.off[0]'), 0) && near(run('constraints[0].a.off[1]'), 5));
ok('a rod length is hypot(3,0)', near(run('constraints[0].len'), 3));
ok('a point station survives the slash (s=1/2)', near(run('conPoints(constraints[0])[0].s'), 0.5));
ok('a vec2 is two expressions', near(run('constraints[1].dir[0]'), Math.cos(Math.PI/3)));
ok('a spring anchor and its rest length', near(run('springs[0].a.off[0]'), b1.r/2) && near(run('springs[0].restLen'), 2*b1.r));
ok('and its k', near(run('springs[0].k'), 10));

console.log('\n3. an expression is not stored');
const out = run('exportScene()');
ok('the export is digits, not arithmetic', !/[*^]|sqrt|hypot|\bpi\b|b[0-9]+\./.test(out), out);
ok('the export reads back to the same scene', (()=>{
  const e1=run('exportScene()'); if(load(e1)) return false;
  return run('exportScene()')===e1;
})());
// The point of the boundary: a pose typed against another body is a number from
// then on. Move that body and nothing follows it -- following is what a constraint
// is for, and this is deliberately not one.
run('bodies[0].x = 10; bodies[0].y = 10;');
ok('a value typed against a body does not follow it', near(B(1).x, 2*0.5*Math.SQRT2));

console.log('\n4. what a file may not say');
run('loadExample("fourbar")');
const before = run('exportScene()');
for(const [text, want] of [
  ['body 1 x=b1.y y=b1.x r=0.5',                       'defined in terms of itself'],
  ['body 1 x=0 y=b9.x r=0.5',                          'defines no body 9'],
  ['body 1 x=0 y=0 r=0.5\nbody 2 x=b1.nope y=0 r=0.5', 'has no property "nope"'],
  ['rect 1 x=0 y=0 width=1 height=1\nbody 2 x=b1.r y=0 r=0.5', 'has no property "r"'],
  ['body 1 x=0 y=0 r=1/0',                             'division by zero'],
  ['body 1 x=0 y=0 r=2*+',                             'expected a number'],
  ['body 1 x=0 y=0 r=sqrt(-1)',                        'not a number'],
  ['sim bg.P=b1.r\nbody 1 x=0 y=0 r=0.5',              'unknown name "b1.r"'],
  ['body 1 x=0 y=0 r=0.5 mass=b1.zz',                  'has no property "zz"'],
]){
  const msg = load('scene 3\n'+text);
  ok(`refused: ${want}`, msg!==null && msg.includes(want), `got ${msg}`);
}
ok('every refusal left the bench standing', run('exportScene()')===before);

console.log('\n5. the panel field');
run('loadExample("pendulum")');
const el = mkEl('t_num', {dataset:{step:'0.25', min:'0'}});
const read = (text, ok_) => { el.value=text; ctx.__ok = ok_; return run(`numVal('t_num', __ok)`); };
ok('a field reads arithmetic', near(read('2*pi/3'), 2*Math.PI/3));
ok('...and the bench\'s own names', near(read('b1.x+b1.r'), run('bodies[0].x+bodies[0].r')));
ok('...and the ambient', near(read('bg.T'), run('sim.bg.T')));
ok('a plain number still reads', near(read('2.5'), 2.5));
ok('a bad expression is NaN, marked, and keeps its text',
   Number.isNaN(read('2*')) && el.classList.contains('bad') && el.value==='2*' && /expected a number/.test(el.title));
ok('a value the field cannot take is refused the same way',
   Number.isNaN(read('-1', v=>v>0)) && el.classList.contains('bad'));
ok('and a good one clears the mark', near(read('3'), 3) && !el.classList.contains('bad'));
ok('a name the bench does not have is refused', /no body 99/.test(String((()=>{ read('b99.x'); return el.title; })())));
run('wireNumIns()');
run(`function __key(k, shift){ document.getElementById('t_num').onkeydown({key:k, shiftKey:shift, preventDefault(){}}); }`);
el.value='2*pi'; el.changes=0;
run(`__key('ArrowUp', false)`);
ok('an arrow key steps the value it evaluates to', near(Number(el.value), 2*Math.PI+0.25, 1e-9), el.value);
el.value='1'; run(`__key('ArrowDown', true)`);
ok('shift steps by ten, and min clamps it', Number(el.value)===0, el.value);
ok('stepping fires the edit', el.changes>0);

console.log(`\n${pass} ok, ${fail} failed\n`);
process.exit(fail?1:0);
