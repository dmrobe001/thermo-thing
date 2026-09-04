# Roadmap

This document covers the validation target (the reference machine and its build order), the project's scope boundaries, and a concise snapshot of what is implemented and what is pending.

## 9. Reference machine and build order

The validation target is a **flyball-governed continuously-variable drive**: a load flywheel driven through a variable-ratio transmission (the CVT primitive) whose ratio is regulated by a flyball governor reading the load shaft's own spin rate, holding output speed steady against a varying drive. Build it in an order that is itself the pedagogical arc:

1. **Fixed-ratio drive.** A crank or falling-weight-on-cable drive turns a load flywheel through an ordinary fixed-ratio belt or gear. This establishes the drivetrain and load, and makes the problem visible: the load's speed sags and surges as the drive's own torque/speed varies, with nothing correcting it.
2. **Governed drive.** Replace the fixed ratio with the CVT primitive, and add a flyball governor -- pins, rods, and springs whose flyweights fly outward with the load shaft's spin rate -- feeding its measured radius through a signal wire into the CVT's ratio. The governor then holds the load's speed steady as the drive varies, the classic Watt-governor regulation problem realized entirely from library primitives.

Both machines are assembled entirely from library primitives: a crank or cable-and-falling-weight drive, belt/gear (machine 1) or CVT (machine 2), flyball governor (pins/rods/springs), a flywheel, and a single signal wire carrying the governor's measurement to the CVT ratio.

> **Status (as built):** Neither reference machine is assembled yet. Machine 1's parts all exist (crank, cable, belt, flywheel body) but are not wired together into this specific drivetrain. Machine 2 is additionally blocked on the signal layer (§5) and the driven/modulated RHS (§4.4) -- the governor's *mechanism* can be built today from pins, rods, and springs, but the wire that couples its measurement to the CVT ratio cannot. What the example bench (code §15) does ship is the set of building blocks and validators: rigid and double pendulum, four-bar linkage, slider-crank, knife-edge skate, wheel-on-disk integrator (CVT), a cable ratchet, a gas spring, a free spinning vessel, and the two exchange benches (a hot reservoir driving a working vessel through a plate; a pressurized reservoir driving one through a port). Together these exercise every implemented primitive, but stop short of the full governed drive.

## 10. Scope boundaries

**In scope:** planar rigid bodies in maximal coordinates, plus the gas vessel (a body with one extra length coordinate, DEVELOPMENT.md §6.2); exact bilateral constraints (holonomic and nonholonomic) in the velocity-linear class; a unilateral/LCP path for contacts, ratchets, slack cables, and stops; spring, gravity and gas-pressure force elements; heat and mass interactions between vessels (DEVELOPMENT.md §6.3); signal wires modulating constraint parameters; per-object force and energy instrumentation; runtime composition and editing of all of the above.

**Out of scope by design:** out-of-plane motion; default collision between unrelated bodies; kinetic friction and dissipative contact models; constraints nonlinear in velocity; player-authored coordinate expressions. Each exclusion is what keeps a corresponding part of the engine simple, and none of them blocks the reference machine.

## 11. Implementation status at a glance

The core is live and honest: maximal-coordinate rigid bodies; gas vessels -- a fourth (length) coordinate per vessel, its gas an exactly-conservative potential force element on it, with material attachment points that make cap, wall and interior anchoring one uniform case, and heat/mass interactions coupling one vessel to another or to the background through a shared body (`VESSEL.md`); the velocity-linear bilateral constraints (pin; rod and slot/prismatic, both two-endpoint constraints where either end may anchor to the fixed background instead of a body and independently lock into a rotation-locked joint there -- a rod always holds its distance, a slot only gains rows once at least one end is locked; belt; and the nonholonomic knife-edge and CVT); the Baumgarte-stabilized Schur-complement solve; position projection; the reaction-force readout that turns each multiplier into the force its joint carries; spring and rotational-spring force elements; and a unilateral tetherball cable.

The gaps are catalogued in **Status (as built)** notes throughout `DEVELOPMENT.md`, and none is blocked by the architecture -- each is additive:

- **Islanding and the higher solver rungs** -- one global solve today, no partition, no sparse/iterative path (DEVELOPMENT.md §3.5, §3.6).
- **Driven and modulated constraints** -- every row currently drives its residual to zero; no motors, actuators, or driven ratios (DEVELOPMENT.md §4.4).
- **The signal-wire layer** -- absent entirely, which also blocks the flyball-governed drive (DEVELOPMENT.md §5).
- **Power instrumentation** -- forces are exposed; power crossing joints is not yet surfaced (DEVELOPMENT.md §7).
- **The feature-first editing model** -- the editor is tool-first, with no persistent named features or relation menu (DEVELOPMENT.md §8). Bulk editing has since landed on top of it: a lasso selection with a transform box, and a stash of reusable widgets held as scene fragments (`SCENE.md` §S.9).
- **Locating a flow port** -- mass exchange conserves linear momentum exactly but not angular momentum, because nothing yet names the point the gas crossed at (`VESSEL.md` §V.11, §V.12); the mediating body already is that point, so routing the transfer's impulse through its endpoint columns would close it.
- **A load-controlled cycle** -- heat and mass exchange are live, and a reservoir-driven working vessel already turns heat into stroke, but running a *cycle* means switching the couplings in step with the motion, which waits on the signal layer (§5) exactly as the governed drive does.
- **The reference machine** -- neither stage is assembled; machine 1 needs its drivetrain wired, machine 2 additionally waits on the signal layer (§9 above).
