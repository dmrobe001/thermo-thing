// ============================================================================
//  §04 · WORLD STATE & GLOBALS
//  The single source of truth the rest of the file reads and mutates.
//    §04.1  canvas / context handles
//    §04.2  world arrays (bodies, constraints, gases, cables)
//    §04.3  sim parameters & camera
// ============================================================================

// ---- §04.1 · canvas / context handles ----
const cv = document.getElementById('c');
const ctx = cv.getContext('2d');
const spark = document.getElementById('spark');
const sctx = spark.getContext('2d');

// ---- §04.2 · world arrays ----
let bodies = [];        // {x,y,th, vx,vy,w, mass, I, invM, invI, r, static, sel}
let constraints = [];   // typed; see makers below
let gases = [];         // gas volumes -- vessels/pistons with n,T,gamma (see gasFrame, constraints.js §06.2)
let heatInteractions = [];  // body<->gas/background heat couplings, see physics.js §08.5
let flowInteractions = [];  // body<->gas/background mass-flow couplings, see physics.js §08.5
let cables = [];        // radial-ratchet cable elements (unilateral)
let springs = [];       // linear (Hookean) spring force elements, see constraints.js §06.6
let rotSprings = [];    // rotational (torsional) spring force elements, see constraints.js §06.6
let uid = 1;

// ---- §04.3 · sim parameters & camera ----
// h: fixed step. maxSub: max substeps per frame. beta: Baumgarte gain (spec §3.4).
// A gain much below 1 under-corrects each step's position drift at this h, which
// bleeds kinetic energy every substep -- most visibly on multi-body chains (e.g. the
// double pendulum) right at the extremes of a swing, where drift is largest. beta=1
// (correct ~all of the drift within one substep) was found empirically to minimize
// that leak across the example mechanisms while staying well under the ~2.5-3
// instability threshold for kb·h at this step size. This leak still scales up with
// chain length (a 5+-link chain visibly loses energy even at beta=1) -- physics.js
// §08.6 closes that gap exactly with a post-solve energy-conservation rescale, so
// beta only needs to keep per-step position drift small enough for that rescale's
// keTarget to stay non-negative, not to be leak-free on its own.
// reg: Tikhonov term added to the Schur diagonal so redundant rows stay solvable.
// bg: the background, which counts as an infinite-capacity gas (spec: "the
// background counts as a gas, with some global temperature and pressure") --
// any heat/flow interaction whose gasId is null couples to this instead of a
// real gases[] entry, and it never itself changes from that exchange.
const sim = { running:false, gravity:true, g:9.8, showForces:true, showGrid:true,
              h:1/120, maxSub:6, beta:1.0, reg:1e-8, forceRef:1,
              bg:{T:1.0, P:1.0, gamma:1.4} };

// camera: world point at screen centre, px per world unit
const cam = { x:0, y:2.2, scale:64 };
let violCount=0;        // constraints exceeding drift tolerance this frame (HUD, §12)
