# Navigating the Implementation

This document is the primary reference for developers and AI agents working in this codebase. Read it before touching any code.

## File layout

The simulator is split into a thin HTML shell and one JavaScript file per logical section:

```
index.html          §01 document head · §02 all CSS · §03 body markup
js/state.js         §04 world state & globals
js/geometry.js      §05 geometry helpers
js/constraints.js   §06 constraint rows / Jacobian builders
js/solver.js        §07 dense linear solve
js/physics.js       §08 physics substep
js/projection.js    §09 position projection & reaction readout
js/loop.js          §10 fixed-step main loop
js/render.js        §11 canvas rendering
js/hud.js           §12 energy ledger, status line, sparkline
js/tools.js         §13 tool palette & pointer handling
js/inspector.js     §14 selection state & right-panel inspector
js/examples.js      §15 prebuilt example machines
js/transport.js     §16 transport controls, snapshots, keyboard shortcuts, boot
```

All scripts are plain globals -- no ES modules. Load order matters; `index.html` loads them in dependency order.

## Units

The world is **SI**: metres, kilograms, seconds, newtons, joules, pascals, kelvin,
with an implicit 1 m out-of-plane depth so a planar area reads directly as a volume.
Ambient is 101325 Pa at 293.15 K (`sim.bg`, §04.3).

## Section markers

Every section header carries a token -- `§NN` for a top-level section, `§NN.M` for a sub-section. The same token appears in exactly three places: the file's own section header, any parent section's sub-index, and cross-references in other routines.

**Do not read a file top to bottom.** Navigate by marker:

1. Find the top-level section you need (e.g. `§08 PHYSICS SUBSTEP`).
2. Search the file for `§08`. You land on the section header, which lists its sub-sections.
3. Search the narrower token (`§08.5`) to jump straight to that one sub-section.

**Zero-padding matters.** Top-level tokens are zero-padded (`§04`, never `§4`). This means searching `§04` never also stops on `§14`. Any *unpadded* section reference in a comment -- e.g. `(see §3.4)` -- points into `DEVELOPMENT.md`, not into the code. The padded markers also cross-link: a routine's comment will name `§07` or `§09` to point at a related routine, making the map a lightweight citation graph.

## Key data structures

All mutable world state lives in `js/state.js` (§04):

- `bodies` -- array of body objects `{id, x, y, th, vx, vy, w, mass, I, invM, invI, r|hw,hh, static, sel}`. Three shapes: `'circle'`, `'rect'`, and `'vessel'` -- a gas vessel, which is an ordinary body carrying a **fourth configuration coordinate**, its length (`len`/`vlen`), plus the gas sealed inside it (`gas:{mass,gamma,Rs,kap}`) and that coordinate's own generalized mass `mu`. Vessels live here, not in a separate array, so islands, save/restore, selection and every constraint work on them unchanged. See `VESSEL.md`.
- `constraints` -- array of typed joint objects; each carries `type`, endpoint refs (`a`, `b`), type-specific parameters, and transient solver outputs (`_lam`, `_rows`).
- `cables` -- array of unilateral tetherball cable elements.
- `springs` -- array of linear (Hookean) spring force elements, `{type:'spring', a, b, restLen, k, sel}` (`a`/`b` are rod-style `{id,off}` endpoints).
- `rotSprings` -- array of rotational (torsional) spring force elements, `{type:'rotspring', a:{id}, b:{id}, restAngle, k, sel}`.
- `sim` -- simulation parameters (`h`, `beta`, `reg`, `running`, `gravity`, `g`, ...).
- `cam` -- camera state (`x`, `y`, `scale`).

## Adding or moving code

Give new code a home in an existing section (and register it in that section's sub-index) or open a new section and update the list above. A stale map is worse than none.

## Section quick-reference

| File | Section | What it does |
|---|---|---|
| `js/state.js` | §04 | Canvas handles; `bodies`, `constraints`, `cables`, `springs`, `rotSprings`; `sim` (incl. `sim.bg`, the ambient atmosphere); `cam` |
| `js/geometry.js` | §05 | `R` (rotation), `worldPt`, `makeBody`, `refreshInertia`, `setBodyMass`, `w2s`/`s2w`; §05.2d `makeVessel`/`refreshVessel`/gas state, §05.2c `epLocal`/`epWorldPt`/`epOffOf` (material endpoint offsets) |
| `js/constraints.js` | §06 | `bodyIndex`, `epWorld`, `epFrame` (endpoint velocity columns, incl. a vessel's length column), `twoPointFrame`, `cableFrame`, `rowsFor`, `makeSpringCon`, `makeRotSpringCon`, `rotSpringSpiralGeom` |
| `js/solver.js` | §07 | `solveLinear` -- dense Gauss-Jordan on the Schur complement |
| `js/physics.js` | §08 | `substep` -- forces (gravity, drag, springs, vessel centrifugal) -> §08.1b `vesselGasStep` -> constraint solve -> position integration -> energy-conservation rescale |
| `js/projection.js` | §09 | `projectPositions`, `conMaxC`, `reactionOf` |
| `js/loop.js` | §10 | `frame` -- fixed-step accumulator, calls `substep` -> `render` -> `updateHUD` |
| `js/render.js` | §11 | `render` orchestrator; `drawBody`, `drawVessel`, `drawConstraint`, `drawCable`, `drawSpring`, `drawRotSpring`, `drawReaction`, ... |
| `js/hud.js` | §12 | `energy` (incl. spring PE, gas internal energy and atmospheric potential), `updateHUD`, `drawSpark` |
| `js/tools.js` | §13 | `TOOLS`, `setTool`, `pickBody`, `snapAnchor`, `conHandles`, pointer handlers |
| `js/inspector.js` | §14 | `clearSelection`, `select*`, `renderInspector`, §14.2b `renderVesselInspector`, `updateInspectorLive` |
| `js/examples.js` | §15 | `loadExample` -- assembles prebuilt machines from library primitives |
| `js/transport.js` | §16 | `saveState`, `restoreState`, `setRunning`, keyboard shortcuts, boot |
