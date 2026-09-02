// ============================================================================
//  §04 · WORLD STATE & GLOBALS
//  The single source of truth the rest of the file reads and mutates.
//    §04.1  canvas / context handles
//    §04.2  world arrays (bodies, constraints, cables, springs)
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
let cables = [];        // radial-ratchet cable elements (unilateral)
let springs = [];       // linear (Hookean) spring force elements, see constraints.js §06.6
let rotSprings = [];    // rotational (torsional) spring force elements, see constraints.js §06.6
let interactions = []; // heat & mass exchange couplings, see physics.js §08.0b
let uid = 1;
// Gas vessels live in `bodies` alongside disks and rectangles -- they are not a
// separate array. A vessel is an ordinary body carrying a *fourth* configuration
// coordinate, its length (`len`/`vlen`), plus the gas state sealed inside it; see
// VESSEL.md and geometry.js §05.2d. Keeping them in `bodies` is what makes islands,
// save/restore, selection, deletion and every existing constraint work on them with
// no per-kind branching.
// `interactions` are the thermodynamic counterpart of a constraint: each one names
// one solid body and one vessel (a null vessel id reads as the background), and a
// PAIR of them sharing the same body couples the two things they name *through*
// that body. They carry no force and no constraint row -- the pass that consumes
// them (physics.js §08.0b) moves only heat and gas mass, at frozen geometry, before
// any force this substep is applied. See VESSEL.md §V.10.

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
// bg: the ambient atmosphere every vessel's caps push against and every gas is
// created at -- 1 atm and 20 C, in SI (Pa, K). The world is SI throughout: metres,
// kilograms, seconds, newtons, joules, with an implicit 1 m out-of-plane depth
// (geometry.js §05.2d VESSEL_DEPTH) turning planar areas into volumes.
// bathQ: cumulative energy the background has delivered *into* the world through
// heat/mass exchange (physics.js §08.0b), in joules. It is the one channel by which
// the scene's own total legitimately moves, because the background is an infinite
// reservoir rather than a tracked body -- so the ledger (hud.js §12.1) carries -bathQ
// as its own row and the running total stays flat, exactly as it does with no
// exchange at all. Exchange between two real vessels never touches it: those two dU
// are equal and opposite by construction.
const sim = { running:false, gravity:true, g:9.8, showForces:true, showGrid:true,
              h:1/120, maxSub:6, beta:1.0, reg:1e-8, forceRef:1,
              bathQ:0, bg:{ P:101325, T:293.15 } };

// camera: world point at screen centre, px per world unit
const cam = { x:0, y:2.2, scale:64 };
let violCount=0;        // constraints exceeding drift tolerance this frame (HUD, §12)
