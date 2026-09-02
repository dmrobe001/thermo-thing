# 2D Mechanical Sandbox

A 2D, top-down environment in which a player builds arbitrary machinery by placing rigid bodies and coupling them with constraints drawn from a fixed library.

## Purpose

The distinguishing goal is **legible energy flow**: the system exists to make the honest dynamics of mechanisms visible and instrumentable, rather than to render impressive-looking motion. The central architectural commitment is that a constraint is a first-class, composable object the player manipulates directly, and that the simulator solves the resulting system *exactly* (to linear-solve and integration tolerance) rather than approximating constraints with stiff penalty forces. A second commitment follows from the first: because the same solver that enforces constraints also produces the reaction forces carried by them, the constraint library doubles as an instrumentation layer.

## World model and assumptions

The world is planar. Every rigid body carries three configuration coordinates `(x, y, theta)` and their time derivatives `(x_dot, y_dot, theta_dot)`. There is no third spatial dimension and no out-of-plane motion.

The following idealizations are load-bearing and should be treated as invariants of the design, not defaults to be relaxed later:

**Nothing interacts unless the player says it does.** Bodies do not collide, overlap-resolve, or exert forces on one another except through constraints and force elements the player explicitly creates. This removes the broad-phase/narrow-phase collision burden from the common case and makes the default world sparse and cheap.

**Friction is either absent or perfectly static.** There is no kinetic friction model. Where "friction" is wanted, it appears as a *constraint* (a static no-slip condition) rather than as a dissipative force law. This keeps the engine's force vocabulary small and keeps energy accounting clean.

These assumptions are what make the project tractable at full honesty. The rest of the design assumes them throughout.

## Getting started

Open `index.html` in a browser. Use the tool rail on the left to place bodies and constraints; load a prebuilt example from the inspector panel on the right to see a working machine immediately.

**Key controls:** Space -- play/pause · R -- reset · wheel -- zoom · middle-drag or Alt-drag -- pan · keys 1-9, b/k/v/c -- select tools.

## Project layout

```
index.html              entry point (HTML shell, all CSS, DOM markup)
js/                     simulator source, one file per section (see AGENT.md)
README.md               this file
AGENT.md                codebase navigation guide for developers and AI agents
DEVELOPMENT.md          physics engine and constraint library design
CABLE.md                design note for the winding-cable constraint (slots alongside DEVELOPMENT.md)
VESSEL.md               design note for the gas vessel (proposed, not yet implemented)
ROADMAP.md              reference machines, scope boundaries, and implementation status
```
