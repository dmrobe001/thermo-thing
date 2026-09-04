# 2D Mechanical Sandbox

A 2D, top-down environment in which a player builds arbitrary machinery by placing rigid bodies and coupling them with constraints drawn from a fixed library.

## Purpose

The distinguishing goal is **legible energy flow**: the system exists to make the honest dynamics of mechanisms visible and instrumentable, rather than to render impressive-looking motion. The central architectural commitment is that a constraint is a first-class, composable object the player manipulates directly, and that the simulator solves the resulting system *exactly* (to linear-solve and integration tolerance) rather than approximating constraints with stiff penalty forces. A second commitment follows from the first: because the same solver that enforces constraints also produces the reaction forces carried by them, the constraint library doubles as an instrumentation layer.

## World model and assumptions

The world is planar and its units are SI. Every rigid body carries three configuration coordinates `(x, y, theta)` and their time derivatives `(x_dot, y_dot, theta_dot)`. There is no third spatial dimension and no out-of-plane motion, though an implicit 1 m depth gives planar areas a volume -- which is what lets a **gas vessel** (a body with one extra coordinate, its length, and a gas sealed inside it) work in real pascals and kelvin. Vessels exchange heat and gas only through **interactions** the player places -- a pair of them sharing a body couples what they each name through it. See `VESSEL.md`.

The following idealizations are load-bearing and should be treated as invariants of the design, not defaults to be relaxed later:

**Nothing interacts unless the player says it does.** Bodies do not collide, overlap-resolve, or exert forces on one another except through constraints and force elements the player explicitly creates. This removes the broad-phase/narrow-phase collision burden from the common case and makes the default world sparse and cheap. Two vessels sitting on top of one another share nothing: heat and gas cross only where an interaction says they do, and the one outline-overlap test in the engine runs only on the pair that names it.

**Friction is either absent or perfectly static.** There is no kinetic friction model. Where "friction" is wanted, it appears as a *constraint* (a static no-slip condition) rather than as a dissipative force law. This keeps the engine's force vocabulary small and keeps energy accounting clean.

These assumptions are what make the project tractable at full honesty. The rest of the design assumes them throughout.

## Getting started

Open `index.html` in a browser. Use the tool rail on the left to place bodies and constraints; load a prebuilt example from the inspector panel on the right to see a working machine immediately.

Nothing in a scene is fixed by a checkbox. A body is held still by a rod welded at both ends to fixed ground; a vessel's length is held by a rod between two of its own caps. The engine still freezes those coordinates internally -- that is what makes a fixed body a wall that islands split at -- but it *derives* which ones from the constraints you placed, so deleting the rod frees the body. See `SCENE.md` §S.8.

Bodies can be selected in bulk: pick the **lasso** (l), draw a loop around the part of
the bench you want, and a box appears around everything caught. Drag inside it to move
the selection, a corner to scale it, the stem above it to turn it -- every selected body
turns by the box's own change in angle, and the couplings between them come along and
are re-read where the transform changed their geometry. A selection can be copied
(Ctrl/Cmd-C), placed again (Ctrl/Cmd-V), or kept in the **widget stash** as a named part
to drop into any later bench. A widget is just a scene fragment, so a scene string
pasted into the stash card becomes a part too. See `SCENE.md` §S.9.

Numbers can be typed as arithmetic, in the inspector fields and in a scene file
alike: `2*pi/3`, `0.4*sqrt(2)`, `bg.P/2`, `b3.x+b3.r` -- the constants, the usual
mathematical functions, the ambient pressure and temperature, and any body's own
properties by the same names the scene file uses. It is a way to declare *initial*
geometry exactly rather than as whatever it rounded to; what gets stored is the
number, so nothing keeps following anything afterwards. Keeping two things related
as the scene moves is what constraints and interactions are for. See `SCENE.md`
§S.10.

Any bench can be written out as a **scene file** -- a plain-text listing of every body with its state, every constraint, force element and interaction, plus the ambient and the camera. Export, import and download it from the scene-file card at the bottom of that same panel, or drop a scene file onto the canvas. The bundled examples *are* scene files, so clicking one shows you its source. The format is also the definitive statement of what a scene may contain: it can express exactly what the tools can build, and nothing else. See `SCENE.md`.

**Key controls:** Space -- play/pause · R -- reset · wheel -- zoom · middle-drag or
Alt-drag -- pan · Ctrl/Cmd-C, Ctrl/Cmd-V -- copy and place a selection · Delete --
remove it · keys 1-9, b/f/g/h/k/l/t/v/c/q -- select tools.

## Project layout

```
index.html              entry point (HTML shell, all CSS, DOM markup)
js/                     simulator source, one file per section (see AGENT.md)
README.md               this file
AGENT.md                codebase navigation guide for developers and AI agents
DEVELOPMENT.md          physics engine and constraint library design
CABLE.md                design note for the winding-cable constraint (slots alongside DEVELOPMENT.md)
VESSEL.md               design note for the gas vessel (slots alongside DEVELOPMENT.md)
SCENE.md                design note for the scene file format and the constructible set
ROADMAP.md              reference machines, scope boundaries, and implementation status
```
