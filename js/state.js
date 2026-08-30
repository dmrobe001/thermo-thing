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
let gases = [];         // gas-piston force elements (see gasFrame / substep step 5)
let cables = [];        // radial-ratchet cable elements (unilateral)
let uid = 1;

// ---- §04.3 · sim parameters & camera ----
// h: fixed step. maxSub: max substeps per frame. beta: Baumgarte gain (spec §3.4).
// reg: Tikhonov term added to the Schur diagonal so redundant rows stay solvable.
const sim = { running:false, gravity:true, g:9.8, showForces:true, showGrid:true,
              h:1/120, maxSub:6, beta:0.15, reg:1e-8, forceRef:1 };

// camera: world point at screen centre, px per world unit
const cam = { x:0, y:2.2, scale:64 };
let violCount=0;        // constraints exceeding drift tolerance this frame (HUD, §12)
