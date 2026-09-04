// ============================================================================
//  §19 · EXPRESSIONS
//  A number you type may be arithmetic instead of a digit string: `2*pi/3`,
//  `bg.P*0.5`, `b3.x+b3.r`. This file is the whole language -- a tokenizer, a
//  recursive-descent parser and an evaluator, in the arithmetic every calculator
//  agrees on, with a table of functions and constants and one hook for names the
//  caller supplies.
//
//    §19.1  the vocabulary  (EXPR_CONSTS, EXPR_FUNCS)
//    §19.2  tokenize        (source text -> tokens)
//    §19.3  parse           (tokens -> a tree)
//    §19.4  evaluate        (a tree + an environment -> one number)
//
//  Two things this deliberately is not.
//
//  It is not a stored formula. `evalExpr` returns a number and the caller keeps
//  the number: the inspector writes it into the body, the scene reader builds
//  with it, and the exporter writes digits back out. Nothing anywhere holds the
//  text. A value that should FOLLOW another value as the scene changes is a
//  constraint or an interaction -- that is what those are for, and they are
//  solved rather than re-substituted. Expressions exist so the geometry you
//  declare up front can be declared exactly (`2.5*cos(30*deg)` rather than a
//  rounded 2.165), which is a statement about the moment of authoring only.
//
//  It is not `eval`. The grammar below is the whole of what a scene file or a
//  panel field may say: no property access beyond the names an environment
//  offers, no calls beyond EXPR_FUNCS, no assignment, no strings. That matters
//  because scene text arrives from files, from the clipboard and from the widget
//  stash -- reading one must not be able to run anything.
//
//  Who supplies the names: an ENVIRONMENT is a function `name -> number`, given
//  by the caller (scene.js §17.8 builds the two the bench uses -- the live world
//  for the inspector, and the file's own text for the reader). It returns
//  undefined for a name it does not know, or throws an ExprError of its own when
//  the name is nearly right and the reason is worth saying.
// ============================================================================
class ExprError extends Error {}

// ---- §19.1 · the vocabulary ----
// Constants are checked before the environment, so nothing a scene or a bench can
// name is able to shadow pi. `deg` is the radians-per-degree factor rather than a
// conversion function, because it reads as the unit it is: sin(30*deg).
const EXPR_CONSTS = {
  pi:  Math.PI,
  tau: 2*Math.PI,
  e:   Math.E,
  deg: Math.PI/180,
};

// n is the arity, or -1 for "one or more". `log` is base 10 and `ln` is natural,
// the calculator convention rather than the C one -- a field that wants a natural
// log in a physical formula is rarer than one that wants a decade, and `ln` is
// unambiguous either way.
const EXPR_FUNCS = {
  sin:  [1, Math.sin],   cos:  [1, Math.cos],   tan:  [1, Math.tan],
  asin: [1, Math.asin],  acos: [1, Math.acos],  atan: [1, Math.atan],
  sinh: [1, Math.sinh],  cosh: [1, Math.cosh],  tanh: [1, Math.tanh],
  atan2:[2, Math.atan2],
  sqrt: [1, Math.sqrt],  cbrt: [1, Math.cbrt],  exp:  [1, Math.exp],
  ln:   [1, Math.log],   log:  [1, Math.log10], log2: [1, Math.log2],
  abs:  [1, Math.abs],   sign: [1, Math.sign],
  floor:[1, Math.floor], ceil: [1, Math.ceil],  round:[1, Math.round],
  pow:  [2, Math.pow],
  // floored modulo, so mod(-1,3) is 2 -- the one an angle wants
  mod:  [2, (a,b)=>a-b*Math.floor(a/b)],
  clamp:[3, (x,lo,hi)=>Math.min(Math.max(x,lo),hi)],
  min:  [-1, Math.min],  max:  [-1, Math.max],  hypot:[-1, Math.hypot],
};

// ---- §19.2 · tokenize ----
// Names are dotted -- `bg.P`, `b3.len` -- because the two things worth naming in
// this bench are already spelled that way: `bg.P` is a key in the scene file's own
// sim line, and a body's properties read as a body's properties. A dot is part of
// a name, never an operator, so there is nothing to reach through and nothing to
// call on the far side of it.
const EXPR_NAME = /^[A-Za-z_][A-Za-z0-9_]*(\.[A-Za-z_][A-Za-z0-9_]*)*/;
const EXPR_NUM  = /^(?:[0-9]+(\.[0-9]*)?|\.[0-9]+)([eE][+-]?[0-9]+)?/;

function exprTokens(src){
  const s = String(src);
  const out = [];
  let i = 0;
  while(i < s.length){
    const c = s[i];
    if(c===' ' || c==='\t'){ i++; continue; }
    if('+-*/^(),'.includes(c)){ out.push({t:c, at:i}); i++; continue; }
    let m;
    if((m = EXPR_NUM.exec(s.slice(i)))){ out.push({t:'num', v:Number(m[0]), at:i}); i += m[0].length; continue; }
    if((m = EXPR_NAME.exec(s.slice(i)))){ out.push({t:'name', v:m[0], at:i}); i += m[0].length; continue; }
    throw new ExprError(`unexpected character "${c}"`);
  }
  out.push({t:'end', at:s.length});
  return out;
}

// ---- §19.3 · parse ----
// Ordinary precedence climbing, one function per level, lowest first:
//
//   sum     := product (('+'|'-') product)*
//   product := unary (('*'|'/') unary)*
//   unary   := ('-'|'+') unary | power
//   power   := atom ['^' unary]              right-associative, binds tighter
//                                            than a leading minus: -2^2 is -4
//   atom    := number | name | name '(' sum (',' sum)* ')' | '(' sum ')'
//
// The tree is plain objects -- {k:'num'|'name'|'call'|'un'|'bin', ...} -- and is
// thrown away as soon as it is evaluated. It exists as a tree at all only so that
// precedence is stated once, here, rather than in an evaluator's bookkeeping.
function parseExpr(src){
  const tk = exprTokens(src);
  let p = 0;
  const peek = () => tk[p];
  const take = () => tk[p++];
  const show = t => t.t==='end' ? 'the end of the expression'
                  : t.t==='num' ? `"${t.v}"` : t.t==='name' ? `"${t.v}"` : `"${t.t}"`;

  function sum(){
    let a = product();
    for(;;){
      const t = peek();
      if(t.t!=='+' && t.t!=='-') return a;
      take();
      a = {k:'bin', op:t.t, a, b:product()};
    }
  }
  function product(){
    let a = unary();
    for(;;){
      const t = peek();
      if(t.t!=='*' && t.t!=='/') return a;
      take();
      a = {k:'bin', op:t.t, a, b:unary()};
    }
  }
  function unary(){
    const t = peek();
    if(t.t==='-'){ take(); return {k:'un', a:unary()}; }
    if(t.t==='+'){ take(); return unary(); }
    return power();
  }
  function power(){
    const a = atom();
    if(peek().t!=='^') return a;
    take();
    return {k:'bin', op:'^', a, b:unary()};   // right-associative, and 2^-1 is legal
  }
  function atom(){
    const t = take();
    if(t.t==='num') return {k:'num', v:t.v};
    if(t.t==='('){
      const a = sum();
      const c = take();
      if(c.t!==')') throw new ExprError(`expected ")" but found ${show(c)}`);
      return a;
    }
    if(t.t==='name'){
      if(peek().t!=='(') return {k:'name', v:t.v};
      take();
      const args=[];
      if(peek().t===')') take();
      else for(;;){
        args.push(sum());
        const c = take();
        if(c.t===')') break;
        if(c.t!==',') throw new ExprError(`expected "," or ")" in ${t.v}(...) but found ${show(c)}`);
      }
      return {k:'call', v:t.v, args};
    }
    throw new ExprError(`expected a number, a name or "(" but found ${show(t)}`);
  }

  if(tk.length===1) throw new ExprError('empty expression');
  const tree = sum();
  const t = peek();
  if(t.t!=='end') throw new ExprError(`unexpected ${show(t)}`);
  return tree;
}

// ---- §19.4 · evaluate ----
// One tree, one environment, one number. Every failure is an ExprError carrying a
// sentence the caller can show as-is: the scene reader wraps it in a line number
// (§17.2 numTok) and the inspector puts it on the field (§14.0).
function evalTree(n, env){
  switch(n.k){
    case 'num': return n.v;
    case 'un':  return -evalTree(n.a, env);
    case 'bin': {
      const a = evalTree(n.a, env), b = evalTree(n.b, env);
      switch(n.op){
        case '+': return a+b;
        case '-': return a-b;
        case '*': return a*b;
        case '/':
          if(b===0) throw new ExprError('division by zero');
          return a/b;
        case '^': return Math.pow(a, b);
      }
      throw new ExprError(`no such operator "${n.op}"`);
    }
    case 'name': {
      if(Object.prototype.hasOwnProperty.call(EXPR_CONSTS, n.v)) return EXPR_CONSTS[n.v];
      if(Object.prototype.hasOwnProperty.call(EXPR_FUNCS, n.v))
        throw new ExprError(`"${n.v}" is a function -- it needs arguments, as ${n.v}(...)`);
      const v = env ? env(n.v) : undefined;
      if(v===undefined) throw new ExprError(`unknown name "${n.v}"`);
      if(typeof v!=='number') throw new ExprError(`"${n.v}" is not a number`);
      return v;
    }
    case 'call': {
      const fn = Object.prototype.hasOwnProperty.call(EXPR_FUNCS, n.v) ? EXPR_FUNCS[n.v] : null;
      if(!fn) throw new ExprError(`unknown function "${n.v}"`);
      const [arity, f] = fn;
      if(arity<0 ? n.args.length<1 : n.args.length!==arity)
        throw new ExprError(`${n.v} takes ${arity<0 ? 'at least one argument' : arity===1 ? '1 argument' : arity+' arguments'}, got ${n.args.length}`);
      return f(...n.args.map(a=>evalTree(a, env)));
    }
  }
  throw new ExprError(`no such node "${n.k}"`);
}

// The whole of it, and the only entry point anything outside this file needs:
// text in, one finite number out, an ExprError with a readable sentence otherwise.
// A non-finite result is a failure rather than a value -- sqrt(-1) and 1e400 are
// both a typo, and nothing downstream (a pose, a length, a pressure) has a
// meaning for them.
function evalExpr(src, env){
  const v = evalTree(parseExpr(src), env);
  if(typeof v!=='number' || !isFinite(v))
    throw new ExprError(Number.isNaN(v) ? 'not a number' : 'not a finite number');
  return v;
}

// The names this build understands, as one sorted list -- for the panel's help
// text and for error messages that want to say what is available.
const exprVocabulary = () =>
  [...Object.keys(EXPR_CONSTS), ...Object.keys(EXPR_FUNCS).map(n=>n+'()')].sort();
