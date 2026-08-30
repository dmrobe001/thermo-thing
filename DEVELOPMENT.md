# Development Reference

Technical design of the physics engine, constraint library, thermodynamic elements, instrumentation, and editing model. For implementation status and pending work see `ROADMAP.md`; for codebase navigation see `AGENT.md`.

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

> **Status (as built):** Implemented in equivalent velocity-impulse form. Code §08.1 integrates the applied forces to candidate velocities; code §08.3 then solves the Schur complement (code §07) for the multipliers `λ` and applies `M⁻¹Jᵀλ` as a single velocity projection. This is the discrete counterpart of the acceleration-level system above: the explicit `J̇·v` term is absorbed into that projection rather than assembled, and `βφ` is the Baumgarte term `sim.beta/h · C`.

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

> **Status (as built):** Only the slack-capable cable exists, and as a specific *tetherball* element (a straight tangent to a spool plus a wound remainder of fixed total length, with reversible winding). It is handled by an active-set toggle — code §08.2 decides each step whether the cable is taut, and only then adds its row to the bilateral equality solve — rather than by a general LCP step. Contacts between bodies, ratchets/clutches, and hard end-stops are not implemented. This matches the spec's staging: the reference machine needs unilateral support only for end-stops and can be built without it initially, and the separate LCP path remains future work.

### 4.4 Driven and modulated relations

Any bilateral row may carry a nonzero right-hand side `b`. If `b` is a constant or a function of time, the constraint is a simple **driven** constraint — a motor holding `θ̇` at a setpoint, an actuator extending a slot. If `b` is instead *another body's measured state*, the relation is **modulated** and has crossed into the control layer (§5). Driven and modulated relations are not new physics; they are ordinary constraint rows with their right-hand side sourced differently.

> **Status (as built):** Not yet implemented. Every row the code builds drives its residual to zero (`b = 0`); there is no nonzero-RHS driven path and no modulated (signal-sourced) path. Consequently the current tool set has no motors, actuators, or driven CVT ratios — the plumbing (per-row RHS) is a small addition to code §06.5 and code §08.3, but it is not wired up.

## 5. Signal layer

The one genuinely new first-class object beyond physical constraints is the **signal wire**: it reads a scalar measurement off the mechanism (a flyball radius, a gas pressure, a shaft angle, a body speed) and feeds it into a constraint's parameter (a CVT ratio, a motor setpoint, an actuator target). Signal wires are first-class but must be presented as *visibly distinct* from physical constraints — a wire, not a joint — because they carry information, not force, and because conflating the two would obscure the energy accounting the sandbox exists to expose. The flyball governor, for instance, requires no new constraint type: it is pins, rods, and springs, with its centrifugal behavior emerging from ordinary rigid-body dynamics. Only its *coupling* to the load — the measurement it drives — is a signal wire.

> **Status (as built):** Not yet implemented. There is no signal-wire object and no modulated parameter anywhere in the code; this layer depends on the driven/modulated RHS path (§4.4), which is also pending. The flyball governor's *mechanism* can be built today from pins, rods, and springs, but the wire that couples its measurement to a load cannot, so the load-controlled cycle is currently unreachable.

## 6. Thermodynamic and force elements

### 6.1 Gas piston

A gas element maintains its equation of state `P = nRT/V` at all times (instantaneous equilibrium). Its volume `V` is a geometric function of the mechanism's state (piston displacement × area). Each step it computes its pressure from current volume and temperature and applies the resulting force `P·A` to the piston follower, contributing to `f` before the constraint solve. It is otherwise an ordinary force element.

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

> **Status (as built):** Simplified. In code §08.5, `T_res` is a fixed setpoint (a slider) with conductance `κ` — i.e. an *infinite*-capacity reservoir that neither drifts nor depletes — and a single reservoir attaches per gas via a `connected` toggle rather than being switchable among several. The finite-capacity, depleting reservoir and multi-reservoir switching described above are pending. The `κ = 0` adiabatic limit *is* implemented and emerges correctly — that is exactly the gas-spring example.

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
