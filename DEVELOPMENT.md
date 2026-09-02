# Development Reference

Technical design of the physics engine, constraint library, instrumentation, and editing model. For implementation status and pending work see `ROADMAP.md`; for codebase navigation see `AGENT.md`.

## 3. Physics engine

### 3.1 Coordinate formulation

The engine uses **maximal coordinates**: each body holds its full `(x, y, theta)` independently, and the system mass matrix `M` is block-diagonal with a per-body block `diag(m, m, I)`. This is deliberately the opposite of a minimal- or generalized-coordinate (Featherstone-style) engine. The reason is compositional: adding a body is adding a 3×3 mass block, and adding a constraint is appending rows to the constraint Jacobian `J`, with no kinematic tree to build, no loops to cut, and no structure to rebuild when the player edits the machine mid-simulation. Runtime editability -- the entire point of a sandbox -- is nearly free in this formulation and awkward in the minimal one. The size disadvantage of maximal coordinates (more coordinates than degrees of freedom) is real but is a constant factor, and it is eroded in practice because real mechanisms are full of closed loops that a minimal engine must itself resolve with loop-closure multipliers.

### 3.2 The per-step solve

Each step assembles the applied generalized force vector `f` (gravity, spring elements, motor drives) and solves the constrained equations of motion as a saddle-point (KKT) system:

```
[ M   J^T ] [ a ]   [ f          ]
[ J   0  ] [ lambda ] = [ -J_dot·v - beta*phi  ]
```

Here `a` is the vector of body accelerations, `lambda` is the vector of Lagrange multipliers, `J` is the stacked constraint Jacobian, `phi` is the vector of position/velocity constraint residuals, and `beta` is a stabilization gain (see §3.4). Integrate `a` to advance velocities and positions; optionally run a velocity projection pass to remove residual constraint drift.

Because constraints are enforced through multipliers, they are satisfied at the acceleration level to the precision of the linear solve -- this is the *exact* enforcement the design requires, not a penalty approximation.

> **Status (as built):** Implemented in equivalent velocity-impulse form. Code §08.1 integrates the applied forces to candidate velocities; code §08.3 then solves the Schur complement (code §07) for the multipliers `lambda` and applies `M^-1 J^T lambda` as a single velocity projection. This is the discrete counterpart of the acceleration-level system above: the explicit `J_dot·v` term is absorbed into that projection rather than assembled, and `beta*phi` is the Baumgarte term `sim.beta/h · C`. Baumgarte alone is only an approximate velocity projection -- its leak grows with per-substep drift and with how many rows are coupled (visibly worse on a 5-link chain than a double pendulum for the same `beta`). Code §08.6 closes that gap exactly, independent of chain length: absent a live drag (the one legitimate energy source/sink), the mechanical total entering the substep is treated as an invariant, and any post-solve discrepancy is folded back with a single uniform velocity rescale -- the same idea as a velocity-rescaling thermostat.
>
> That per-substep rescale has one gap of its own: it's a *multiplicative* correction (`v *= sqrt(target/actual)`), so it's only well-defined while there's actual KE to scale. Right at any island's own turning point -- a pendulum at the top of its swing -- KE passes through zero for real, and both branches of code §08.6 fall back to leaving velocities untouched for that one substep rather than divide by ~0. The gap that fallback couldn't resolve doesn't vanish on its own: the next substep's `preE` is read straight off whatever state was left standing, so an uncorrected gap becomes the new baseline permanently rather than something a later substep makes up for -- confirmed on the plain pendulum example, whose entire slow multi-minute energy loss landed exactly on the substeps where this fallback fired, with every other substep's own delta at floating-point zero. `ENERGY_BANK` (code §08.0, keyed by an island's own body-id signature) defers that gap instead of dropping it, folding it into the very next substep where the same island's KE is large enough to actually rescale -- so the running total now stays exactly conserved across a turning point, not just between them.
>
> A *free* island has a harder version of the same problem, this time with no `ENERGY_BANK`-style fix available: its rigid (COM) motion is fully determined by momentum/angular-momentum targets that are already exact (impulse-momentum is exact for a constant force regardless of step size), which leaves *no* spare freedom in velocity to also hit the energy target -- only the internal/shape field (§08.6's `keInt`) has that freedom, since it alone carries zero net momentum. A free island with *no* internal motion at all has nothing for that internal channel to work with, so the shortfall goes unresolved forever rather than merely at a turning point: confirmed on the simplest possible case, a single free body with gravity and nothing else at all, whose energy decreased *linearly*, unboundedly, at exactly `0.5*M*g^2*h^2` per substep -- the textbook secular drift of a symplectic-Euler position update (`x += h*v_new`) under *constant* (non-oscillating) acceleration, which the usual "bounded energy error" guarantee for symplectic integrators only covers for bounded/oscillatory motion. Code §08.6 applies a second additive fix, this time to *position*: the island's COM is shifted (uniformly across every body, so it can't touch the internal field) to the exact trapezoidal displacement `0.5*(V0+Vt)*h` implied by the same momentum history the velocity fix already uses -- the standard leapfrog/Verlet correction for constant acceleration, applied here only to the rigid/COM part rather than switching the whole engine's integrator. It has to run *before* `post.pe` is read for that same substep's `keTarget`, or the internal-motion rescale "discovers" the same gravity-vs-position mismatch a second time and injects real energy correcting it twice.

### 3.3 The constraint class boundary

The library is restricted to constraints that are **linear in the velocities** -- the Pfaffian form

```
A(q) · v = b(q, t)
```

where `A` depends only on the current configuration, `v` stacks the involved bodies' velocities, and `b` is zero, a constant, or a driven signal. Every mechanical joint of interest -- pin, rod (with its optional per-end weld), slot, prismatic, gear, belt, rolling wheel, variable-ratio transmission -- is a small number of rows of this shape. The coefficients of `A` are trigonometric/geometric terms the engine computes from the parts' current poses; **the player never writes an expression in `x`, `y`, or `theta`.** This is the line to draw: players compose geometric features and select relations among them, and the velocity-linear class is exactly what the KKT solver consumes as rows of `J`. Constraints outside this class (nonlinear in velocity) are neither common in real machines nor cheap to solve, and are out of scope.

### 3.4 Holonomic vs. nonholonomic handling

Both kinds of bilateral constraint enter the solver as identical-looking Jacobian rows, but they differ in one respect the engine must handle:

- A **holonomic** constraint has an underlying position invariant `g(q) = 0`. The solver enforces its differentiated form at the velocity/acceleration level, which lets the position residual drift under integration error. These constraints therefore carry a Baumgarte stabilization term (the `beta*phi` above) and/or a post-step position projection to pull `g` back toward zero.
- A **nonholonomic** constraint (`A·v = 0` with no integrable `g`) has *no* position invariant to drift from. Started on the constraint manifold, enforcing `d/dt(A·v) = 0` keeps it satisfied, needing at most a cheap velocity projection. The velocity-only constraints are thus the *simpler* case here, not the exotic one.

### 3.5 Islanding

The constraint graph partitions into connected components ("islands") of bodies coupled by constraints; disconnected machines are independent solves. Per-step cost is therefore governed by the largest single connected mechanism, not the total part count on the canvas. Islands are recomputed only when the topology changes (a constraint added or removed), not every step.

> **Status (as built):** Not yet implemented. Code §08.3 assembles one global constraint system each step over every row on the canvas -- there is no island partition and no topology-change cache. Results are correct, but per-step cost currently scales with the *total* constraint count rather than the largest mechanism. Islanding is the principal pending performance work.

### 3.6 Solver ladder and performance

Implement the cheapest solver that meets the scale, and climb only as needed:

1. **Dense KKT per island** -- simplest, exact, adequate up to ~100-200 coordinates in one connected machine (a ~150×150 solve is well under a millisecond in optimized WASM). Start here.
2. **Sparse Cholesky on the Schur complement** `J M^-1 J^T` -- same exactness, extends to large single machines; the symbolic factorization is reusable until the player edits the topology, with numeric refactoring each step.
3. **Sequential impulse (Box2D-style)** -- matrix-free iteration over constraints; scales to hundreds of bodies but relaxes constraints approximately, trading exactness for iteration count.

Precision-critical mechanisms (any loop whose exactness matters, e.g. a governed transmission stage) should stay on a direct solver. Because islands are independent, a precise machine and a sloppy iterative one can coexist without interfering.

> **Status (as built):** The engine sits between rungs 1 and 2. Code §07 forms the dense Schur complement `J M^-1 J^T` (rung 2's operator) but factors it with dense Gauss-Jordan partial-pivot elimination rather than sparse Cholesky, and does so once globally rather than per island (see §3.5). Sequential impulse (rung 3) is absent. A small Tikhonov term (`sim.reg`, added to the diagonal) keeps redundant or overconstrained row sets solvable. This is exact and comfortably fast at the current example scale; the sparse/per-island refactor is the natural next rung.

### 3.7 Unilateral constraints (separate path)

Inequality constraints -- contacts, one-way ratchets and clutches, cables that carry tension but go slack in compression, hard end-stops on slots and rotations -- break out of the linear equality solve into a complementarity (LCP) step. Keep these architecturally separate from the bilateral core so that joints and nonholonomic rows remain in the clean, fast equality solve. Unilateral support can be added after the bilateral engine is working; the reference machine (§9) needs it only for end-stops and can be built without it initially.

## 4. Constraint library

Every bilateral constraint in the library is one instance of a single atomic operation: **measure a scalar velocity -- a projection defined by a geometric feature -- and constrain it to zero, to another measurement, or to a driven signal.** The catalog below is that operation applied to different features.

### 4.1 Holonomic bilateral (position invariant; stabilized)

| Constraint | Condition (velocity form) | Rows |
|---|---|---|
| Pin / revolute | relative velocity of a shared point is zero | 2 |
| Rod / distance | relative velocity *along* the connecting line is zero | 1 (+1 per welded end) |
| Slot / rail (point-on-line) | across-line drift zero once both ends are locked | 0-3, see below |
| Prismatic slider | across-line relative velocity zero **and** relative omega zero | see below (slot, both ends locked) |
| Gear (fixed ratio) | weighted sum of angular rates is zero | 1 |
| Belt / cable (inextensible) | rim tangential speeds equal | 1 |

The rod and the slot are conceptual complements -- distance-along-a-line vs. drift-across-one -- but are no longer built symmetrically: a rod always carries its base (distance) row, while a slot's base row is optional (see below).

**Weld and "prismatic" are the same per-endpoint operation, applied to two different base constraints.** Rod and slot both connect two endpoints, either of which can be a body or attached directly to the fixed background (`id === null`, mirroring the convention already used by cable tethers). Any endpoint can independently be a freely-rotating *pin*, or *locked* -- a row pinning that endpoint's body angle (or, for a background endpoint, the fixed world frame) to the live direction of the segment joining the two endpoints. For a rod this lock is called a **weld**; for a slot, **prismatic**. The two constraints differ in what's active when neither end is locked and in what a lock implies:

- **Rod** always carries its 1-row distance constraint, regardless of weld state. A rod with both ends welded to the background is rigid in both position and orientation, which is how a body is now anchored to the world (there is no standalone "static" toggle in the editor); a rod with only its background end welded reproduces the old ground-pin (a fixed pivot the far body spins freely about); a rod with no welds at either end is the original free-swinging distance joint.
- **Slot** carries *no* row at all until at least one end is prismatic -- two pins is a purely visual rail (drawn through the two points, no physics). One locked end adds just the angle-lock row (rotation only) -- *unless* that end is the background, in which case the endpoint's own angle is a fixed constant, so the row degenerates into pinning the entire rail's position (a fixed ray from that point) with zero rotation lock on the other end; this is how a slider gets confined to a straight line while still spinning freely. Only once *both* ends are prismatic does a third row appear -- the classic point-stays-on-the-rail lock, canceling lateral drift -- reconstructing the full rigid prismatic joint (no drift, no relative rotation) that `lockRot` used to give directly.

### 4.2 Nonholonomic bilateral (velocity-only; no stabilization)

| Constraint | Condition | Rows |
|---|---|---|
| No-side-slip wheel | contact-point velocity *across* heading is zero | 1 |
| Rolling wheel | across-heading zero **and** along-heading = r·spin | 2 |
| Variable-ratio transmission (disk-on-disk) | tangential speeds equal at a contact point whose radius is itself a coordinate | 1 |

These three are the same primitive -- "tangential velocities match at a parameterized contact point" -- with different features attached. Building the variable-radius contact once yields the rolling wheel, the ball-and-disk integrator, and the continuously variable transmission as configurations of one object. A gear whose ratio depends on configuration *is* nonholonomic; this is why the CVT lives in this group and not with the fixed-ratio gear above.

### 4.3 Unilateral (inequality; LCP path)

Contacts, ratchets/clutches (one-way angular coupling), slack-capable cables (tension only), and hard stops (end-of-travel on slots and rotations). Each is a one-sided version of a bilateral row, active only while its complementarity condition holds.

> **Status (as built):** Only the slack-capable cable exists, and as a specific *tetherball* element (a straight tangent to a spool plus a wound remainder of fixed total length, with reversible winding). It is handled by an active-set toggle -- code §08.2 decides each step whether the cable is taut, and only then adds its row to the bilateral equality solve -- rather than by a general LCP step. Contacts between bodies, ratchets/clutches, and hard end-stops on slots/rotations are not implemented. This matches the spec's staging: the reference machine needs unilateral support only for end-stops and can be built without it initially, and the separate LCP path remains future work. See `CABLE.md` for the cable's own design note (why the wrapping cable is energy-conserving, the per-end-radius abstraction, and per-line as-built status); the point<->spool case there is implemented, the spool<->spool generalization is not.

### 4.4 Driven and modulated relations

Any bilateral row may carry a nonzero right-hand side `b`. If `b` is a constant or a function of time, the constraint is a simple **driven** constraint -- a motor holding `theta_dot` at a setpoint, an actuator extending a slot. If `b` is instead *another body's measured state*, the relation is **modulated** and has crossed into the control layer (§5). Driven and modulated relations are not new physics; they are ordinary constraint rows with their right-hand side sourced differently.

> **Status (as built):** Not yet implemented. Every row the code builds drives its residual to zero (`b = 0`); there is no nonzero-RHS driven path and no modulated (signal-sourced) path. Consequently the current tool set has no motors, actuators, or driven CVT ratios -- the plumbing (per-row RHS) is a small addition to code §06.5 and code §08.3, but it is not wired up.

## 5. Signal layer

The one genuinely new first-class object beyond physical constraints is the **signal wire**: it reads a scalar measurement off the mechanism (a flyball radius, a shaft angle, a body speed) and feeds it into a constraint's parameter (a CVT ratio, a motor setpoint, an actuator target). Signal wires are first-class but must be presented as *visibly distinct* from physical constraints -- a wire, not a joint -- because they carry information, not force, and because conflating the two would obscure the energy accounting the sandbox exists to expose. The flyball governor, for instance, requires no new constraint type: it is pins, rods, and springs, with its centrifugal behavior emerging from ordinary rigid-body dynamics. Only its *coupling* to the load -- the measurement it drives -- is a signal wire.

> **Status (as built):** Not yet implemented. There is no signal-wire object and no modulated parameter anywhere in the code; this layer depends on the driven/modulated RHS path (§4.4), which is also pending. The flyball governor's *mechanism* can be built today from pins, rods, and springs, but the wire that couples its measurement to a load cannot, so the load-controlled cycle is currently unreachable.

## 6. Force elements

### 6.1 Springs

A spring is an ordinary Hookean force element, not a constraint: it contributes to the applied force vector `f` (§3.2) rather than a row of `J`, so it is solved to whatever precision the force integration gives, not enforced exactly the way a rod is. A **linear** spring connects two endpoints (the same `{id, off}` shape as a rod -- either end may be background-anchored) and applies `F = k(restLen - L)` along the line between them, `L` the live distance. A **rotational** spring connects two bodies' frame angles directly (background reads as a fixed `theta = 0`, mirroring the null-id convention used elsewhere) and applies a pure couple `tau = k(restAngle - (theta_A - theta_B))`, with no point of application. Both default their rest value to whatever the live measurement is at creation, so a freshly-placed spring starts unstressed.

Because a spring genuinely stores and releases mechanical energy, its potential energy (`0.5 k (L - restLen)^2` linear, `0.5 k (relAngle - restAngle)^2` rotational) is counted alongside kinetic and gravitational energy in the system total (§7) -- omitting it would make code §08.6's energy-conservation rescale read the spring's own KE<->PE conversion as drift and cancel it out.

> **Status (as built):** Implemented (code §06.6, applied in §08.1). Both spring types are pure force elements with no damping term -- lossless by construction, consistent with the engine's exact-constraint philosophy (`CABLE.md` §C.1 makes the same choice for the cable, for the same reason). A linear spring, when selected, shows a capped rest-length indicator parallel to itself with a draggable control point (constraints.js `springRestHandlePos`); the spring constant is inspector-only. A rotational spring renders as a belt between the two rims when they don't fully overlap, or as a decorative spiral (to the smaller body's rim, or to the body's centre when attached to the background, or when the bodies fully overlap) otherwise -- purely a rendering choice (`rotSpringVisualMode`/`rotSpringSpiralGeom`), not a change in the underlying torque law.

## 7. Instrumentation and state exposure

Instrumentation is not an add-on; it is a consequence of the solver. Every bilateral constraint's Lagrange multiplier `lambda` **is** the reaction force or torque that joint carries, and `rate × lambda` is the mechanical power flowing through it -- both available for free from the solve the engine already performs. The system must expose, per object:

- Per body: `(x, y, theta, x_dot, y_dot, theta_dot)`, kinetic energy.
- Per constraint: its multiplier `lambda` (reaction force/torque) and the power crossing it.
- System-level: total energy by category, with a running balance so that dissipation-free operation is visibly conservative.

This turns the constraint library into a measurement layer and is most of what makes the sandbox convincing as an honest instrument rather than a plausible-looking animation.

> **Status (as built):** Partially surfaced. Live today: system energy by category -- kinetic, potential, and spring potential (§6.1) -- with a running total and sparkline (code §12); per-constraint reaction force/torque `lambda/h`, both in the inspector and as on-canvas arrows (code §09.3, §11.6). Not yet surfaced: the power crossing each constraint (`rate × lambda`). This is recoverable from quantities the solve already computes; only the readout is missing.

## 8. Editing model and UX

**Features on bodies.** Constraints attach not to bodies directly but to *features* the player drops onto bodies by clicking: anchor points, axes/directions, rim circles, slot lines. A feature is a named geometric handle in a body's local frame.

**Constraint creation is "select two features, pick a relation."** The relation menu is filtered to what is geometrically sensible for the selected features: two points offer *coincide* (pin) or *hold at distance* (rod); two rim circles offer *gear* or *belt*; a point and a body direction offer *slot*; a rim and a face offer the variable-ratio contact. The player never sees a coordinate expression.

**Per-constraint controls are few and consistent.** Each constraint exposes at most: its principal parameter (rest length / ratio / target), an enable toggle, a sense or direction for unilateral types, and -- the important one -- a switch marking a parameter as **live-driven by a signal wire** rather than fixed.

**Three states must read apart at a glance.** A hard constraint (fixed relation), a driven constraint (constant or time-function target), and a modulated constraint (parameter fed by a signal wire) should be visually distinct, because the difference between them is the difference between structure, actuation, and control -- exactly the distinctions a player is trying to reason about.

**Signal wires are drawn as wires**, visibly separate from physical constraints, connecting a measurement source to a modulated parameter.

> **Status (as built):** The current editor is tool-first rather than feature-first. You select a tool from the rail (code §13.1) and click bodies; anchors snap to body centres and edges (code §13.2). There is no persistent, named-feature object dropped onto bodies, and no relation-filter menu -- the tool palette stands in for "select two features, pick a relation," and each tool constructs one constraint type directly (code §13.5). The three-way hard/driven/modulated visual distinction is moot until driven and modulated constraints exist (§4.4, §5); today every constraint is "hard." Signal wires, likewise, are not drawn because they do not yet exist.
