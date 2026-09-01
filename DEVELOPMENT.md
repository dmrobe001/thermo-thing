# Development Reference

Technical design of the physics engine, constraint library, thermodynamic elements, instrumentation, and editing model. For implementation status and pending work see `ROADMAP.md`; for codebase navigation see `AGENT.md`.

## 3. Physics engine

### 3.1 Coordinate formulation

The engine uses **maximal coordinates**: each body holds its full `(x, y, theta)` independently, and the system mass matrix `M` is block-diagonal with a per-body block `diag(m, m, I)`. This is deliberately the opposite of a minimal- or generalized-coordinate (Featherstone-style) engine. The reason is compositional: adding a body is adding a 3×3 mass block, and adding a constraint is appending rows to the constraint Jacobian `J`, with no kinematic tree to build, no loops to cut, and no structure to rebuild when the player edits the machine mid-simulation. Runtime editability -- the entire point of a sandbox -- is nearly free in this formulation and awkward in the minimal one. The size disadvantage of maximal coordinates (more coordinates than degrees of freedom) is real but is a constant factor, and it is eroded in practice because real mechanisms are full of closed loops that a minimal engine must itself resolve with loop-closure multipliers.

### 3.2 The per-step solve

Each step assembles the applied generalized force vector `f` (gravity, spring elements, gas pressure forces, motor drives) and solves the constrained equations of motion as a saddle-point (KKT) system:

```
[ M   J^T ] [ a ]   [ f          ]
[ J   0  ] [ lambda ] = [ -J_dot·v - beta*phi  ]
```

Here `a` is the vector of body accelerations, `lambda` is the vector of Lagrange multipliers, `J` is the stacked constraint Jacobian, `phi` is the vector of position/velocity constraint residuals, and `beta` is a stabilization gain (see §3.4). Integrate `a` to advance velocities and positions; optionally run a velocity projection pass to remove residual constraint drift.

Because constraints are enforced through multipliers, they are satisfied at the acceleration level to the precision of the linear solve -- this is the *exact* enforcement the design requires, not a penalty approximation.

> **Status (as built):** Implemented in equivalent velocity-impulse form. Code §08.1 integrates the applied forces to candidate velocities; code §08.3 then solves the Schur complement (code §07) for the multipliers `lambda` and applies `M^-1 J^T lambda` as a single velocity projection. This is the discrete counterpart of the acceleration-level system above: the explicit `J_dot·v` term is absorbed into that projection rather than assembled, and `beta*phi` is the Baumgarte term `sim.beta/h · C`. Baumgarte alone is only an approximate velocity projection -- its leak grows with per-substep drift and with how many rows are coupled (visibly worse on a 5-link chain than a double pendulum for the same `beta`). Code §08.6 closes that gap exactly, independent of chain length: absent a live drag or an active heat/flow interaction (both legitimate energy sources/sinks), the mechanical+gas total entering the substep is treated as an invariant, and any post-solve discrepancy is folded back with a single uniform velocity rescale -- the same idea as a velocity-rescaling thermostat.

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

Precision-critical mechanisms (any loop whose exactness matters, e.g. a transmission on a thermodynamic branch) should stay on a direct solver. Because islands are independent, a precise machine and a sloppy iterative one can coexist without interfering.

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

**Weld and "prismatic" are the same per-endpoint operation, applied to two different base constraints.** Rod and slot both connect two endpoints, either of which can be a body or attached directly to the fixed background (`id === null`, mirroring the convention already used by gas heads and cable tethers). Any endpoint can independently be a freely-rotating *pin*, or *locked* -- a row pinning that endpoint's body angle (or, for a background endpoint, the fixed world frame) to the live direction of the segment joining the two endpoints. For a rod this lock is called a **weld**; for a slot, **prismatic**. The two constraints differ in what's active when neither end is locked and in what a lock implies:

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

> **Status (as built):** Only the slack-capable cable exists, and as a specific *tetherball* element (a straight tangent to a spool plus a wound remainder of fixed total length, with reversible winding). It is handled by an active-set toggle -- code §08.2 decides each step whether the cable is taut, and only then adds its row to the bilateral equality solve -- rather than by a general LCP step. Contacts between bodies, ratchets/clutches, and hard end-stops are not implemented. This matches the spec's staging: the reference machine needs unilateral support only for end-stops and can be built without it initially, and the separate LCP path remains future work. See `CABLE.md` for the cable's own design note (why the wrapping cable is energy-conserving, the per-end-radius abstraction, and per-line as-built status); the point<->spool case there is implemented, the spool<->spool generalization is not.

### 4.4 Driven and modulated relations

Any bilateral row may carry a nonzero right-hand side `b`. If `b` is a constant or a function of time, the constraint is a simple **driven** constraint -- a motor holding `theta_dot` at a setpoint, an actuator extending a slot. If `b` is instead *another body's measured state*, the relation is **modulated** and has crossed into the control layer (§5). Driven and modulated relations are not new physics; they are ordinary constraint rows with their right-hand side sourced differently.

> **Status (as built):** Not yet implemented. Every row the code builds drives its residual to zero (`b = 0`); there is no nonzero-RHS driven path and no modulated (signal-sourced) path. Consequently the current tool set has no motors, actuators, or driven CVT ratios -- the plumbing (per-row RHS) is a small addition to code §06.5 and code §08.3, but it is not wired up.

## 5. Signal layer

The one genuinely new first-class object beyond physical constraints is the **signal wire**: it reads a scalar measurement off the mechanism (a flyball radius, a gas pressure, a shaft angle, a body speed) and feeds it into a constraint's parameter (a CVT ratio, a motor setpoint, an actuator target). Signal wires are first-class but must be presented as *visibly distinct* from physical constraints -- a wire, not a joint -- because they carry information, not force, and because conflating the two would obscure the energy accounting the sandbox exists to expose. The flyball governor, for instance, requires no new constraint type: it is pins, rods, and springs, with its centrifugal behavior emerging from ordinary rigid-body dynamics. Only its *coupling* to the load -- the measurement it drives -- is a signal wire.

> **Status (as built):** Not yet implemented. There is no signal-wire object and no modulated parameter anywhere in the code; this layer depends on the driven/modulated RHS path (§4.4), which is also pending. The flyball governor's *mechanism* can be built today from pins, rods, and springs, but the wire that couples its measurement to a load cannot, so the load-controlled cycle is currently unreachable.

## 6. Thermodynamic and force elements

### 6.1 Gas vessels and pistons

A gas element is a rectangular vessel that maintains its equation of state `P = nT/V` at all times (instantaneous equilibrium; `R = 1` in these abstract units). It is defined by two endpoints, exactly the `{id, off}` shape every other feature uses: `head` (a corner fixed in some body's frame, or the world, carrying the rectangle's axis direction) and `piston` (the far corner along that axis) -- the same two-corner selection a rectangular body placement uses. `piston` is either a real body -- a movable wall, making this a piston -- or `null`, in which case the rectangle's length is a fixed constant (`len`): a sealed vessel with no moving part at all, useful purely as a heat/flow node (§6.2). Either way `V = bore · length`.

**The background counts as a gas too** (`sim.bg`: a fixed, player-set temperature and pressure, infinite capacity) -- it is the implicit far side of any heat/flow interaction that names no gas (§6.2), and the pressure a piston's open face pushes against.

**The piston is two bodies.** The movable wall (`piston`) feels the net force of internal gas pressure against the background's, normal to its face: `F = (P_gas - P_bg)·bore`, along the vessel's axis; the rest of the vessel (`head`'s body, or nothing if head is world-fixed) feels the equal and opposite reaction. Placing a gas whose far corner lands on a body auto-creates a **mutually prismatic joint** between that body and the head -- an ordinary `slot` constraint with both ends locked (§4.1), marked `hidden` so it never renders or gets picked, deleted only by deleting the gas. This is bookkeeping the gas needs to exist as two rigid bodies at all, not a joint the player composed, hence not visualized.

### 6.2 Heat and flow interactions

Reservoirs are gone as a per-gas bolt-on; heat and mass exchange are now **interactions**, first-class objects each naming one solid body and one gas-or-background (`gasId === null` reads as the background, mirroring the null-id convention used everywhere else). A lone interaction moves nothing -- exchange only happens when **two** interactions share the same body, one on each side of it: the body is a wall between two gas-like things, and the pair couples whatever they each name *through* that body. This is deliberately symmetric with how a piston's two bodies work: a body-mediated relation between two other things, not a special reservoir type.

The rate depends on geometry the same way for both kinds: the **contact area** between the body's own outline and each named gas's rectangle (`geometry.js` §05.2c, a general convex-polygon clip -- a circle is approximated as a 20-gon so one routine handles every body shape), and the **smaller of the pair's two areas** is what limits the exchange (conduction/flow through a wall is bottlenecked by whichever side touches less of it). Each interaction carries its own conductivity/flow-restriction `k`; a pair's combined rate uses them in series, `1/k_eff = 1/k_1 + 1/k_2`, like two conductors back to back.

**Heat.** `Q_dot = k_eff · area_min · (T_other - T_self)`, exactly Newton's law of cooling generalized to two finite-capacity sides (`C = n/(gamma-1)` per gas; the background is infinite capacity and never itself moves). Rather than integrate that ODE forward one `Q_dot·h` step at a time -- which can overshoot equilibrium at a large `k·h` -- both sides' temperatures are solved **exactly** for the substep: the temperature difference decays as `D(t) = D_0 · exp(-lambda t)` (`lambda = k_eff·area_min·(1/C_1 + 1/C_2)`) while the weighted sum `C_1 T_1 + C_2 T_2` stays exactly conserved, so both `T`s land in closed form, unconditionally stable for any step size -- literally the "approached exponentially" the spec calls for, not a design choice layered on top of an Euler step.

**Flow** is the same closed-form relaxation with pressure and moles playing temperature and capacity's roles: holding `T`/`V` fixed over the substep (the same instantaneous-equilibrium stance `P=nT/V` already takes), each side's "pressure per mole" `s = T/V` is constant, so `P_A - P_B` decays exponentially exactly like the heat difference did, converging on the pressure-equalizing split of the two sides' combined moles (or, against the background, on whatever `n` gives the vessel `P_bg`). Moles that cross carry the *source* gas's own internal energy with them (`dn · cv_src · T_src`), absorbed into the destination at its own `cv` -- mirroring real mixing -- while the gas left behind keeps its own `T`, since removing gas at a given temperature doesn't change the temperature of what remains.

**Forces.** Flowing mass also pushes back on the vessels it crosses -- a thrust-like `|deltaP|`-scaled recoil along each side's own axis, applied at the same head/piston attachment points as the pressure force, equal-and-opposite between two real vessels and single-sided against the background (spec: "gas mass flowing from one vessel to another should apply appropriate equal and opposite forces on the vessels... flow to/from the background just applies a force to the vessel"). Its mechanical work is credited back into the same non-mechanical energy ledger the moles' own carried internal energy already uses (physics.js §08.0b/§08.1), so the exact energy-conservation rescale (§3.2's status note, §08.6) treats it as legitimate input rather than erasing it as drift.

When no interaction touches a gas at all it is isolated and traverses an adiabat automatically by energy conservation (the mechanical `P·dV` term of §6.1, alone) -- **adiabatic branches are not built, they emerge**, exactly as before; the gas-spring example is precisely this case.

> **Status (as built):** Implemented as described (code §08.0b). Every interaction's rate law and both relaxations are exact closed-form solutions, not stepped ODEs, so they stay stable regardless of `sim.h`/`k`. Multiple simultaneous pairs on one substep are resolved by sequential operator splitting (each pair's relaxation sees the previous pair's already-updated state) rather than one joint solve -- fine at this step size, and the per-pair math is still exactly energy/mole-conserving on its own.

### 6.3 Emergent process branches

This is the modeling stance for the whole thermodynamic side: idealized process branches are *consequences* of the force law and the coupling, not trajectories imposed on the machine. An isothermal branch is the gas held at constant `T_w` by a heat interaction to a much larger (near-infinite-capacity) gas or the background, which for an ideal gas is identical to `P is proportional to 1/V`, which is identical to *constant mechanical power* delivered by the gas (`DeltaU = 0`, so `P·V_dot = Q_dot = k_eff·area(T_bath - T_w)` = const). "Hold the isotherm" and "hold the gas power constant" and "hold `V_dot/V` constant" are three faces of one condition; the last two are things a governor can regulate directly, whereas temperature is not mechanically measurable in this world. A load-controlled cycle realizes the isotherm by regulating rate, not by tracing a shape.

### 6.4 Springs

A spring is an ordinary Hookean force element, not a constraint: it contributes to the applied force vector `f` (§3.2) rather than a row of `J`, so it is solved to whatever precision the force integration gives, not enforced exactly the way a rod is. A **linear** spring connects two endpoints (the same `{id, off}` shape as a rod -- either end may be background-anchored) and applies `F = k(restLen - L)` along the line between them, `L` the live distance. A **rotational** spring connects two bodies' frame angles directly (background reads as a fixed `theta = 0`, mirroring the null-id convention used elsewhere) and applies a pure couple `tau = k(restAngle - (theta_A - theta_B))`, with no point of application. Both default their rest value to whatever the live measurement is at creation, so a freshly-placed spring starts unstressed.

Because a spring genuinely stores and releases mechanical energy, its potential energy (`0.5 k (L - restLen)^2` linear, `0.5 k (relAngle - restAngle)^2` rotational) is counted alongside kinetic, gravitational, and gas-internal energy in the system total (§7) -- omitting it would make code §08.6's energy-conservation rescale read the spring's own KE<->PE conversion as drift and cancel it out.

> **Status (as built):** Implemented (code §06.6, applied in §08.1). Both spring types are pure force elements with no damping term -- lossless by construction, consistent with the engine's exact-constraint philosophy (`CABLE.md` §C.1 makes the same choice for the cable, for the same reason). A linear spring, when selected, shows a capped rest-length indicator parallel to itself with a draggable control point (constraints.js `springRestHandlePos`); the spring constant is inspector-only. A rotational spring renders as a belt between the two rims when they don't fully overlap, or as a decorative spiral (to the smaller body's rim, or to the body's centre when attached to the background, or when the bodies fully overlap) otherwise -- purely a rendering choice (`rotSpringVisualMode`/`rotSpringSpiralGeom`), not a change in the underlying torque law.

## 7. Instrumentation and state exposure

Instrumentation is not an add-on; it is a consequence of the solver. Every bilateral constraint's Lagrange multiplier `lambda` **is** the reaction force or torque that joint carries, and `rate × lambda` is the mechanical power flowing through it -- both available for free from the solve the engine already performs. The system must expose, per object:

- Per body: `(x, y, theta, x_dot, y_dot, theta_dot)`, kinetic energy.
- Per constraint: its multiplier `lambda` (reaction force/torque) and the power crossing it.
- Per gas element: `P, V, T, n`, heat+flow rate `Q_dot`, work rate `P·V_dot`, cumulative heat and work.
- Per heat/flow interaction: live contact area, and the rate it's currently moving.
- System-level: total energy by category, with a running balance so that dissipation-free operation is visibly conservative.

This turns the constraint library into a measurement layer and is most of what makes a thermodynamics demonstrator convincing.

> **Status (as built):** Partially surfaced. Live today: system energy by category -- kinetic, potential, gas internal, and spring potential (§6.4) -- with a running total and sparkline (code §12); per-constraint reaction force/torque `lambda/h`, both in the inspector and as on-canvas arrows (code §09.3, §11.6); per-gas `P, V, T, n, Q_dot` (code §14.3); and per-interaction live contact area (code §14.3). Not yet surfaced: the power crossing each constraint (`rate × lambda`), the gas work rate `P·V_dot`, cumulative heat and work, and a split of `Q_dot` into its heat vs. flow-carried parts (today it's their sum). These are all recoverable from quantities the solve already computes; only the readout is missing.

## 8. Editing model and UX

**Features on bodies.** Constraints attach not to bodies directly but to *features* the player drops onto bodies by clicking: anchor points, axes/directions, rim circles, slot lines. A feature is a named geometric handle in a body's local frame.

**Constraint creation is "select two features, pick a relation."** The relation menu is filtered to what is geometrically sensible for the selected features: two points offer *coincide* (pin) or *hold at distance* (rod); two rim circles offer *gear* or *belt*; a point and a body direction offer *slot*; a rim and a face offer the variable-ratio contact. The player never sees a coordinate expression.

**Per-constraint controls are few and consistent.** Each constraint exposes at most: its principal parameter (rest length / ratio / target), an enable toggle, a sense or direction for unilateral types, and -- the important one -- a switch marking a parameter as **live-driven by a signal wire** rather than fixed.

**Three states must read apart at a glance.** A hard constraint (fixed relation), a driven constraint (constant or time-function target), and a modulated constraint (parameter fed by a signal wire) should be visually distinct, because the difference between them is the difference between structure, actuation, and control -- exactly the distinctions a player is trying to reason about.

**Signal wires are drawn as wires**, visibly separate from physical constraints, connecting a measurement source to a modulated parameter.

> **Status (as built):** The current editor is tool-first rather than feature-first. You select a tool from the rail (code §13.1) and click bodies; anchors snap to body centres and edges (code §13.2). There is no persistent, named-feature object dropped onto bodies, and no relation-filter menu -- the tool palette stands in for "select two features, pick a relation," and each tool constructs one constraint type directly (code §13.5). The three-way hard/driven/modulated visual distinction is moot until driven and modulated constraints exist (§4.4, §5); today every constraint is "hard." Signal wires, likewise, are not drawn because they do not yet exist.
