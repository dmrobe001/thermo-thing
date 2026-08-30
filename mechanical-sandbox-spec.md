# 2D Mechanical Sandbox — Design Specification

## 1. Purpose

The sandbox is a 2D, top-down environment in which a player builds arbitrary machinery by placing rigid bodies and coupling them with constraints drawn from a fixed library. Its distinguishing goal is **legible energy flow**: the system exists to make the honest dynamics of mechanisms — and especially of thermodynamic machines — visible and instrumentable, rather than to render impressive-looking motion. The motivating target is a physically accurate finite-time heat engine that draws from reservoirs with real heat capacity and conductance, does work against a real load, and can be tuned toward its maximum-power operating point. Everything in this specification is chosen so that a machine of that kind can be assembled from primitives and simulated with complete physical honesty, subject to a small, clearly stated set of idealizing assumptions.

The central architectural commitment is that a constraint is a first-class, composable object the player manipulates directly, and that the simulator solves the resulting system *exactly* (to linear-solve and integration tolerance) rather than approximating constraints with stiff penalty forces. A second commitment follows from the first: because the same solver that enforces constraints also produces the reaction forces carried by them, the constraint library doubles as an instrumentation layer.

## 2. World model and assumptions

The world is planar. Every rigid body carries three configuration coordinates `(x, y, θ)` and their time derivatives `(ẋ, ẏ, θ̇)`. There is no third spatial dimension and no out-of-plane motion.

The following idealizations are load-bearing and should be treated as invariants of the design, not defaults to be relaxed later:

**Nothing interacts unless the player says it does.** Bodies do not collide, overlap-resolve, or exert forces on one another except through constraints and force elements the player explicitly creates. This removes the broad-phase/narrow-phase collision burden from the common case and makes the default world sparse and cheap.

**Friction is either absent or perfectly static.** There is no kinetic friction model. Where "friction" is wanted, it appears as a *constraint* (a static no-slip condition) rather than as a dissipative force law. This keeps the engine's force vocabulary small and keeps energy accounting clean.

**Gases are in instantaneous internal equilibrium.** A gas element always satisfies its equation of state for its current volume; it has no internal transient, no spatial structure, and no messiness when its container moves or rotates. Only the boundary heat exchange is finite-rate.

These assumptions are what make the project tractable at full honesty. The remainder of the specification assumes them throughout.

## 3. Physics engine

### 3.1 Coordinate formulation

The engine uses **maximal coordinates**: each body holds its full `(x, y, θ)` independently, and the system mass matrix `M` is block-diagonal with a per-body block `diag(m, m, I)`. This is deliberately the opposite of a minimal- or generalized-coordinate (Featherstone-style) engine. The reason is compositional: adding a body is adding a 3×3 mass block, and adding a constraint is appending rows to the constraint Jacobian `J`, with no kinematic tree to build, no loops to cut, and no structure to rebuild when the player edits the machine mid-simulation. Runtime editability — the entire point of a sandbox — is nearly free in this formulation and awkward in the minimal one. The size disadvantage of maximal coordinates (more coordinates than degrees of freedom) is real but is a constant factor, and it is eroded in practice because real mechanisms are full of closed loops that a minimal engine must itself resolve with loop-closure multipliers.

### 3.2 The per-step solve

Each step assembles the applied generalized force vector `f` (gravity, spring elements, gas pressure forces, motor drives) and solves the constrained equations of motion as a saddle-point (KKT) system:

```
[ M   Jᵀ ] [ a ]   [ f          ]
[ J   0  ] [ λ ] = [ -J̇·v - βφ  ]
```

Here `a` is the vector of body accelerations, `λ` is the vector of Lagrange multipliers, `J` is the stacked constraint Jacobian, `φ` is the vector of position/velocity constraint residuals, and `β` is a stabilization gain (see §3.4). Integrate `a` to advance velocities and positions; optionally run a velocity projection pass to remove residual constraint drift.

Because constraints are enforced through multipliers, they are satisfied at the acceleration level to the precision of the linear solve — this is the *exact* enforcement the design requires, not a penalty approximation.

> **Status (as built):** Implemented in equivalent velocity-impulse form. Code §08.1 integrates the applied forces to candidate velocities; code §08.3 then solves the Schur complement (code §07) for the multipliers `λ` and applies `M⁻¹Jᵀλ` as a single velocity projection. This is the discrete counterpart of the acceleration-level system above: the explicit `J̇·v` term is absorbed into that projection rather than assembled, and `βφ` is the Baumgarte term `sim.beta/h · C`. (Code sections are the `§NN` markers in `sandbox.html`; see this document's §11.)

### 3.3 The constraint class boundary

The library is restricted to constraints that are **linear in the velocities** — the Pfaffian form

```
A(q) · v = b(q, t)
```

where `A` depends only on the current configuration, `v` stacks the involved bodies' velocities, and `b` is zero, a constant, or a driven signal. Every mechanical joint of interest — pin, weld, rod, slot, prismatic, gear, belt, rolling wheel, variable-ratio transmission — is a small number of rows of this shape. The coefficients of `A` are trigonometric/geometric terms the engine computes from the parts' current poses; **the player never writes an expression in `x`, `y`, or `θ`.** This is the line to draw: players compose geometric features and select relations among them, and the velocity-linear class is exactly what the KKT solver consumes as rows of `J`. Constraints outside this class (nonlinear in velocity) are neither common in real machines nor cheap to solve, and are out of scope.

### 3.4 Holonomic vs. nonholonomic handling

Both kinds of bilateral constraint enter the solver as identical-looking Jacobian rows, but they differ in one respect the engine must handle:

- A **holonomic** constraint has an underlying position invariant `g(q) = 0`. The solver enforces its differentiated form at the velocity/acceleration level, which lets the position residual drift under integration error. These constraints therefore carry a Baumgarte stabilization term (the `βφ` above) and/or a post-step position projection to pull `g` back toward zero.
- A **nonholonomic** constraint (`A·v = 0` with no integrable `g`) has *no* position invariant to drift from. Started on the constraint manifold, enforcing `d/dt(A·v) = 0` keeps it satisfied, needing at most a cheap velocity projection. The velocity-only constraints are thus the *simpler* case here, not the exotic one.

### 3.5 Islanding

The constraint graph partitions into connected components ("islands") of bodies coupled by constraints; disconnected machines are independent solves. Per-step cost is therefore governed by the largest single connected mechanism, not the total part count on the canvas. Islands are recomputed only when the topology changes (a constraint added or removed), not every step.

> **Status (as built):** Not yet implemented. Code §08.3 assembles one global constraint system each step over every row on the canvas — there is no island partition and no topology-change cache. Results are correct, but per-step cost currently scales with the *total* constraint count rather than the largest mechanism. Islanding is the principal pending performance work.

### 3.6 Solver ladder and performance

Implement the cheapest solver that meets the scale, and climb only as needed:

1. **Dense KKT per island** — simplest, exact, adequate up to ~100–200 coordinates in one connected machine (a ~150×150 solve is well under a millisecond in optimized WASM). Start here.
2. **Sparse Cholesky on the Schur complement** `J M⁻¹ Jᵀ` — same exactness, extends to large single machines; the symbolic factorization is reusable until the player edits the topology, with numeric refactoring each step.
3. **Sequential impulse (Box2D-style)** — matrix-free iteration over constraints; scales to hundreds of bodies but relaxes constraints approximately, trading exactness for iteration count.

Precision-critical mechanisms (any loop whose exactness matters, e.g. a transmission on a thermodynamic branch) should stay on a direct solver. Because islands are independent, a precise machine and a sloppy iterative one can coexist without interfering.

> **Status (as built):** The engine sits between rungs 1 and 2. Code §07 forms the dense Schur complement `J M⁻¹ Jᵀ` (rung 2's operator) but factors it with dense Gauss–Jordan partial-pivot elimination rather than sparse Cholesky, and does so once globally rather than per island (see §3.5). Sequential impulse (rung 3) is absent. A small Tikhonov term (`sim.reg`, added to the diagonal) keeps redundant or overconstrained row sets solvable. This is exact and comfortably fast at the current example scale; the sparse/per-island refactor is the natural next rung.

### 3.7 Unilateral constraints (separate path)

Inequality constraints — contacts, one-way ratchets and clutches, cables that carry tension but go slack in compression, hard end-stops on slots and rotations — break out of the linear equality solve into a complementarity (LCP) step. Keep these architecturally separate from the bilateral core so that joints and nonholonomic rows remain in the clean, fast equality solve. Unilateral support can be added after the bilateral engine is working; the reference machine (§9) needs it only for end-stops and can be built without it initially.

## 4. Constraint library

Every bilateral constraint in the library is one instance of a single atomic operation: **measure a scalar velocity — a projection defined by a geometric feature — and constrain it to zero, to another measurement, or to a driven signal.** The catalog below is that operation applied to different features.

### 4.1 Holonomic bilateral (position invariant; stabilized)

| Constraint | Condition (velocity form) | Rows |
|---|---|---|
| Pin / revolute | relative velocity of a shared point is zero | 2 |
| Weld | shared-point relative velocity zero **and** relative ω zero | 3 |
| Rod / distance | relative velocity *along* the connecting line is zero | 1 |
| Slot / rail (point-on-line) | relative velocity *across* the line is zero | 1 |
| Prismatic slider | across-line relative velocity zero **and** relative ω zero | 2 |
| Gear (fixed ratio) | weighted sum of angular rates is zero | 1 |
| Belt / cable (inextensible) | rim tangential speeds equal | 1 |

The rod and the slot are exact complements: one kills the along-line component, the other the across-line component.

### 4.2 Nonholonomic bilateral (velocity-only; no stabilization)

| Constraint | Condition | Rows |
|---|---|---|
| No-side-slip wheel | contact-point velocity *across* heading is zero | 1 |
| Rolling wheel | across-heading zero **and** along-heading = r·spin | 2 |
| Variable-ratio transmission (disk-on-disk) | tangential speeds equal at a contact point whose radius is itself a coordinate | 1 |

These three are the same primitive — "tangential velocities match at a parameterized contact point" — with different features attached. Building the variable-radius contact once yields the rolling wheel, the ball-and-disk integrator, and the continuously variable transmission as configurations of one object. A gear whose ratio depends on configuration *is* nonholonomic; this is why the CVT lives in this group and not with the fixed-ratio gear above.

### 4.3 Unilateral (inequality; LCP path)

Contacts, ratchets/clutches (one-way angular coupling), slack-capable cables (tension only), and hard stops (end-of-travel on slots and rotations). Each is a one-sided version of a bilateral row, active only while its complementarity condition holds.

> **Status (as built):** Only the slack-capable cable exists, and as a specific *tetherball* element (a straight tangent to a spool plus a wound remainder of fixed total length, with reversible winding). It is handled by an active-set toggle — code §08.2 decides each step whether the cable is taut, and only then adds its row to the bilateral equality solve — rather than by a general LCP step. Contacts between bodies, ratchets/clutches, and hard end-stops are not implemented. This matches the spec's staging: the reference machine (§9) needs unilateral support only for end-stops and can be built without it initially, and the separate LCP path remains future work.

### 4.4 Driven and modulated relations

Any bilateral row may carry a nonzero right-hand side `b`. If `b` is a constant or a function of time, the constraint is a simple **driven** constraint — a motor holding `θ̇` at a setpoint, an actuator extending a slot. If `b` is instead *another body's measured state*, the relation is **modulated** and has crossed into the control layer (§5). Driven and modulated relations are not new physics; they are ordinary constraint rows with their right-hand side sourced differently.

> **Status (as built):** Not yet implemented. Every row the code builds drives its residual to zero (`b = 0`); there is no nonzero-RHS driven path and no modulated (signal-sourced) path. Consequently the current tool set has no motors, actuators, or driven CVT ratios — the plumbing (per-row RHS) is a small addition to code §06.5 and code §08.3, but it is not wired up.

## 5. Signal layer

The one genuinely new first-class object beyond physical constraints is the **signal wire**: it reads a scalar measurement off the mechanism (a flyball radius, a gas pressure, a shaft angle, a body speed) and feeds it into a constraint's parameter (a CVT ratio, a motor setpoint, an actuator target). Signal wires are first-class but must be presented as *visibly distinct* from physical constraints — a wire, not a joint — because they carry information, not force, and because conflating the two would obscure the energy accounting the sandbox exists to expose. The flyball governor, for instance, requires no new constraint type: it is pins, rods, and springs, with its centrifugal behavior emerging from ordinary rigid-body dynamics. Only its *coupling* to the load — the measurement it drives — is a signal wire.

> **Status (as built):** Not yet implemented. There is no signal-wire object and no modulated parameter anywhere in the code; this layer depends on the driven/modulated RHS path (§4.4), which is also pending. The flyball governor's *mechanism* can be built today from pins, rods, and springs, but the wire that couples its measurement to a load cannot, so the load-controlled cycle of §9.2 is currently unreachable.

## 6. Thermodynamic and force elements

### 6.1 Gas piston

A gas element maintains its equation of state `P = nRT/V` at all times (instantaneous equilibrium, §2). Its volume `V` is a geometric function of the mechanism's state (piston displacement × area). Each step it computes its pressure from current volume and temperature and applies the resulting force `P·A` to the piston follower, contributing to `f` before the constraint solve. It is otherwise an ordinary force element.

### 6.2 Reservoir and heat exchange

A reservoir has a finite heat capacity and a temperature `T_res` that drifts as heat is drawn from or dumped into it. Heat crosses the gas boundary at finite rate through a finite conductance:

```
Q̇ = κ (T_res − T_gas)
```

The gas temperature evolves by the first law with instantaneous equilibrium:

```
n c_v Ṫ = κ (T_res − T_gas) − P·V̇
```

Reservoir connection is switchable (which reservoir, if any, the gas is currently coupled to). When no reservoir is connected the gas is isolated and traverses an adiabat automatically by energy conservation — **adiabatic branches are not built, they emerge.**

> **Status (as built):** Simplified. In code §08.5, `T_res` is a fixed setpoint (a slider) with conductance `κ` — i.e. an *infinite*-capacity reservoir that neither drifts nor depletes — and a single reservoir attaches per gas via a `connected` toggle rather than being switchable among several. The finite-capacity, depleting reservoir and multi-reservoir switching described above are pending (and note that §9.2's "optimal setpoint drifts as reservoirs deplete" depends on them). The `κ = 0` adiabatic limit, by contrast, *is* implemented and emerges correctly — that is exactly the gas-spring example.

### 6.3 Emergent process branches

This is the modeling stance for the whole thermodynamic side: idealized process branches are *consequences* of the force law and the coupling, not trajectories imposed on the machine. An isothermal branch is the gas held at constant `T_w` by reservoir coupling, which for an ideal gas is identical to `P ∝ 1/V`, which is identical to *constant mechanical power* delivered by the gas (`ΔU = 0`, so `P·V̇ = Q̇ = κ(T_res − T_w)` = const). "Hold the isotherm" and "hold the gas power constant" and "hold `V̇/V` constant" are three faces of one condition; the last two are things a governor can regulate directly, whereas temperature is not mechanically measurable in this world. A load-controlled cycle realizes the isotherm by regulating rate, not by tracing a shape.

## 7. Instrumentation and state exposure

Instrumentation is not an add-on; it is a consequence of the solver. Every bilateral constraint's Lagrange multiplier `λ` **is** the reaction force or torque that joint carries, and `rate × λ` is the mechanical power flowing through it — both available for free from the solve the engine already performs. The system must expose, per object:

- Per body: `(x, y, θ, ẋ, ẏ, θ̇)`, kinetic energy.
- Per constraint: its multiplier `λ` (reaction force/torque) and the power crossing it.
- Per gas element: `P, V, T`, heat flow `Q̇`, work rate `P·V̇`, cumulative heat and work.
- Per reservoir: `T_res`, remaining stored energy, cumulative heat exchanged.
- System-level: total energy by category, with a running balance so that dissipation-free operation is visibly conservative.

This turns the constraint library into a measurement layer and is most of what makes a thermodynamics demonstrator convincing.

> **Status (as built):** Partially surfaced. Live today: system energy by category — kinetic, potential, and gas internal — with a running total and sparkline (code §12); per-constraint reaction force/torque `λ/h`, both in the inspector and as on-canvas arrows (code §09.3, §11.6); and per-gas `P, V, T, Q̇` (code §14.3). Not yet surfaced: the power crossing each constraint (`rate × λ`), the gas work rate `P·V̇`, cumulative heat and work, and per-reservoir stored/exchanged energy. These are all recoverable from quantities the solve already computes; only the readout is missing.

## 8. Editing model and UX

**Features on bodies.** Constraints attach not to bodies directly but to *features* the player drops onto bodies by clicking: anchor points, axes/directions, rim circles, slot lines. A feature is a named geometric handle in a body's local frame.

**Constraint creation is "select two features, pick a relation."** The relation menu is filtered to what is geometrically sensible for the selected features: two points offer *coincide* (pin) or *hold at distance* (rod); two rim circles offer *gear* or *belt*; a point and a body direction offer *slot*; a rim and a face offer the variable-ratio contact. The player never sees a coordinate expression.

**Per-constraint controls are few and consistent.** Each constraint exposes at most: its principal parameter (rest length / ratio / target), an enable toggle, a sense or direction for unilateral types, and — the important one — a switch marking a parameter as **live-driven by a signal wire** rather than fixed.

**Three states must read apart at a glance.** A hard constraint (fixed relation), a driven constraint (constant or time-function target), and a modulated constraint (parameter fed by a signal wire) should be visually distinct, because the difference between them is the difference between structure, actuation, and control — exactly the distinctions a player is trying to reason about.

**Signal wires are drawn as wires**, visibly separate from physical constraints, connecting a measurement source to a modulated parameter.

> **Status (as built):** The current editor is tool-first rather than feature-first. You select a tool from the rail (code §13.1) and click bodies; anchors snap to body centres and edges (code §13.2). There is no persistent, named-feature object dropped onto bodies, and no relation-filter menu — the tool palette stands in for "select two features, pick a relation," and each tool constructs one constraint type directly (code §13.5). The three-way hard/driven/modulated visual distinction is moot until driven and modulated constraints exist (§4.4, §5); today every constraint is "hard." Signal wires, likewise, are not drawn because they do not yet exist.

## 9. Reference machine and build order

The validation target is a finite-time heat engine tunable toward its maximum-power (Curzon–Ahlborn) operating point: a gas piston exchanging heat with hot and cold reservoirs of finite capacity and conductance, doing work against a flywheel load, with a rate governor selecting the operating point. Build it in an order that is itself the pedagogical arc:

1. **Constant-ω circular-crank engine.** Kinematically drive the piston through a fixed volume waveform; switch reservoir exposure by crank angle. This machine has a real maximum-power point but *cannot* reach the CA bound — a fixed sinusoidal `V(t)` cannot take the isothermal (hyperbolic) shape the optimum requires, so its efficiency at max power sits below `η_CA`. Displaying that shortfall next to the `η_CA` line is the lesson.
2. **Load-controlled engine with governed isotherm.** Replace the rigid crank with a compliant, variable-ratio coupling (the CVT primitive) so the piston is force-balanced rather than position-driven, and regulate `V̇/V` with a governor acting through the CVT — realized cleanly by placing the governor on a shaft whose angle is `ln V`, generated by the wheel-on-disk integrator. The isotherm then appears as the stable attractor of the rate-regulated dynamics rather than as an imposed shape, and the operating point climbs toward the CA bound. With finite reservoirs, the optimal setpoint drifts as they deplete — a feature to watch, not a bug.

Both machines are assembled entirely from library primitives: gas piston, slot/rack, CVT (variable-ratio nonholonomic contact), wheel-on-disk integrator (the same primitive wired for `ln V`), flyball governor (pins/rods/springs), flywheel, and a single signal wire carrying the governor's measurement to the CVT ratio.

> **Status (as built):** Neither reference machine is assembled yet. Machine 1 is close in principle — its parts (crank, slot/rack, gas piston, reservoirs by `connected` toggle) all exist — but a kinematic constant-`ω` crank drive and reservoir switching by crank angle are not wired. Machine 2 is blocked on the signal layer (§5), the driven/modulated RHS (§4.4), and finite reservoirs (§6.2). What the example bench (code §15) does ship is the set of building blocks and validators: rigid and double pendulum, four-bar linkage, slider-crank + piston, gas spring, knife-edge skate, wheel-on-disk integrator (CVT), and a cable ratchet. Together these exercise every implemented primitive, but stop short of the full finite-time heat engine.

## 10. Scope boundaries

**In scope:** planar rigid bodies in maximal coordinates; exact bilateral constraints (holonomic and nonholonomic) in the velocity-linear class; a unilateral/LCP path for contacts, ratchets, slack cables, and stops; spring, gravity, and gas-pressure force elements; finite-capacity, finite-conductance reservoirs with switchable coupling; signal wires modulating constraint parameters; per-object force, energy, and power instrumentation; runtime composition and editing of all of the above.

**Out of scope by design:** out-of-plane motion; default collision between unrelated bodies; kinetic friction and dissipative contact models; spatially resolved or non-equilibrium gas behavior; constraints nonlinear in velocity; player-authored coordinate expressions. Each exclusion is what keeps a corresponding part of the engine simple, and none of them blocks the reference machine.

## 11. Navigating the implementation

The implementation is a single self-contained file, `sandbox.html`: the document head, then all the CSS, then the entire simulator as one script. **Do not read it top to bottom.** It is organized for search, not for linear reading, and a linear pass will bury the structure that the section map surfaces in seconds.

**Read the SOURCE MAP first.** The very top of `sandbox.html` is a comment block titled `MECHANISM BENCH — SOURCE MAP`. It states the marker convention and indexes all sixteen top-level sections. Every section header in the file carries a token — `§NN` for a top-level section, `§NN.M` for a sub-section — and every parent header repeats the local sub-index for its own children. To reach any part of the code:

1. Open the SOURCE MAP and find the top-level section you need (for example, `§08 PHYSICS SUBSTEP`).
2. Search the file for that token (`§08`). You land on the section header, which lists its sub-sections.
3. Search the narrower token (`§08.5`) to jump straight to that one sub-section.

You should almost never scan code you did not search your way to. Two properties make this reliable. Top-level numbers are **zero-padded** (`§04`, never `§4`), so a search for `§04` never also stops on `§14`. And because the code markers are padded, any *unpadded* section reference in the code — a comment reading `(see §3.4)` — points back into *this specification*, not into the file. The markers also cross-link: a routine's comment will name `§07` or `§09` to point at a related routine, so the map doubles as a lightweight citation graph.

When you add or move code, give it a home in an existing section (and register it in that section's sub-index) or open a new section and add it to the SOURCE MAP. A stale map is worse than none; keeping it truthful is the price of working in this file.

### 11.1 Implementation status at a glance

The core is live and honest: maximal-coordinate rigid bodies; the velocity-linear bilateral constraints (pin, ground, rod, weld, slot/prismatic, belt, and the nonholonomic knife-edge and CVT); the Baumgarte-stabilized Schur-complement solve; position projection; the reaction-force readout that turns each multiplier into the force its joint carries; the gas piston with finite-rate boundary heat exchange; and a unilateral tetherball cable.

The gaps are catalogued in **Status (as built)** notes throughout this document, and none is blocked by the architecture — each is additive:

- **Islanding and the higher solver rungs** — one global solve today, no partition, no sparse/iterative path (§3.5, §3.6).
- **Driven and modulated constraints** — every row currently drives its residual to zero; no motors, actuators, or driven ratios (§4.4).
- **The signal-wire layer** — absent entirely, which also blocks the flyball-governed cycle (§5).
- **Finite, switchable reservoirs** — the reservoir is an infinite-capacity fixed setpoint; it neither depletes nor switches among several (§6.2).
- **Power/heat/work instrumentation** — forces are exposed; power crossing joints, gas work rate, and cumulative heat/work are not yet surfaced (§7).
- **The feature-first editing model** — the editor is tool-first, with no persistent named features or relation menu (§8).
- **The two reference machines** — neither is assembled; machine 2 waits on the signal layer and finite reservoirs (§9).
