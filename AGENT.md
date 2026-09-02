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

## Section markers

Every section header carries a token -- `§NN` for a top-level section, `§NN.M` for a sub-section. The same token appears in exactly three places: the file's own section header, any parent section's sub-index, and cross-references in other routines.

**Do not read a file top to bottom.** Navigate by marker:

1. Find the top-level section you need (e.g. `§08 PHYSICS SUBSTEP`).
2. Search the file for `§08`. You land on the section header, which lists its sub-sections.
3. Search the narrower token (`§08.5`) to jump straight to that one sub-section.

**Zero-padding matters.** Top-level tokens are zero-padded (`§04`, never `§4`). This means searching `§04` never also stops on `§14`. Any *unpadded* section reference in a comment -- e.g. `(see §3.4)` -- points into `DEVELOPMENT.md`, not into the code. The padded markers also cross-link: a routine's comment will name `§07` or `§09` to point at a related routine, making the map a lightweight citation graph.

## Key data structures

All mutable world state lives in `js/state.js` (§04):

- `bodies` -- array of rigid-disk objects `{id, x, y, th, vx, vy, w, mass, I, invM, invI, r, static, sel}`.
- `constraints` -- array of typed joint objects; each carries `type`, endpoint refs (`a`, `b`), type-specific parameters, and transient solver outputs (`_lam`, `_rows`). A gas's auto-created `gasmount` (its `com` body's rigid mount to the head frame) carries `hidden:true` and `gasLink:<gasId>` -- excluded from rendering/picking, has no `a`/`b` endpoints at all (only `gasLink`), and is deleted only via its gas.
- `gases` -- array of gas vessels/pistons `{id, head:{id,off,dir}, com:{id}|null, piston:{id,off}|null, sep, sepRate, len (if no piston), bore, mass, T, gamma, lockedField, sel}` (`mass` is the amount of gas -- `n` renamed, molar mass = 1; `lockedField` is inspector-only UI state, never read by physics.js). For a movable piston, `com` is a real, dynamical `bodies[]` entry (the gas's own center of mass, mass forced every substep to the gas's *full* `mass`, `geometry.js`'s `syncVesselComMass`) and `piston` is a *static*, kinematically-slaved marker body (repositioned from `com`+`sep` every substep, `syncVesselMarkers`) -- both `synthetic:true`, hidden from `drawBody`/`pickBody`/the body inspector since the vessel renders and picks as one unified rectangle (`drawVessel`/`gasHit`, DEVELOPMENT.md §6.1). `sep`/`sepRate` (the vessel's own axial separation and its rate) are a genuine extra scalar coordinate in the solver's own coordinate space (`geometry.js` §05.2d), decoupled from `com`'s translation with its own effective inertia `mass/12` -- this is the exact decomposition of a uniformly-distributed gas column's kinetic energy (`mass·(v1²+v1·v2+v2²)/6` in head/cap velocities, `(1/2)·mass·vc²+(1/2)·(mass/12)·vr²` in COM/relative velocities), replacing an earlier, provably-wrong per-body mass split. A rod/pin/spring endpoint may also be `{vesselId, frac, lat}` -- a point *inside* the vessel (constraints.js §06.2d) -- instead of the usual `{id, off}`. `sim.bg` (state.js §04.3) is the background, which also counts as a gas (infinite capacity, fixed T/P) for heat/flow purposes.
- `heatInteractions`, `flowInteractions` -- arrays of `{bodyId, gasId (null = background), k, sel}`. Two entries sharing a `bodyId` couple whatever gas/background each names, through that body (constraints.js gasFrame/gasPolygon, physics.js §08.0b).
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
| `js/state.js` | §04 | Canvas handles; `bodies`, `constraints`, `gases`, `heatInteractions`, `flowInteractions`, `cables`, `springs`, `rotSprings`; `sim` (incl. `sim.bg`); `cam` |
| `js/geometry.js` | §05 | `R` (rotation), `worldPt`, `makeBody`, `refreshInertia`, `setBodyMass`, `bodyPolygon`/`gasPolygon`/`clipPoly`/`bodyGasOverlapArea`, `w2s`/`s2w` |
| `js/constraints.js` | §06 | `bodyIndex`, `epWorld`, `twoPointFrame`, `gasFrame`, `gasCentroid`, `cableFrame`, `rowsFor`, `makeSpringCon`, `makeRotSpringCon`, `rotSpringSpiralGeom` |
| `js/solver.js` | §07 | `solveLinear` -- dense Gauss-Jordan on the Schur complement |
| `js/physics.js` | §08 | `substep` -- heat/flow interactions -> forces (incl. springs, gas) -> constraint solve -> position integration -> gas P·dV work |
| `js/projection.js` | §09 | `projectPositions`, `conMaxC`, `reactionOf` |
| `js/loop.js` | §10 | `frame` -- fixed-step accumulator, calls `substep` -> `render` -> `updateHUD` |
| `js/render.js` | §11 | `render` orchestrator; `drawBody`, `drawConstraint`, `drawGas`, `drawHeatInteraction`/`drawFlowInteraction`, `drawSpring`, `drawRotSpring`, `drawReaction`, ... |
| `js/hud.js` | §12 | `energy` (incl. spring PE), `updateHUD`, `drawSpark` |
| `js/tools.js` | §13 | `TOOLS`, `setTool`, `pickBody`, `snapAnchor`, `conHandles`, `purgeGas`, pointer handlers |
| `js/inspector.js` | §14 | `clearSelection`, `select*`, `renderInspector`, `updateInspectorLive` |
| `js/examples.js` | §15 | `loadExample` -- assembles prebuilt machines from library primitives |
| `js/transport.js` | §16 | `saveState`, `restoreState`, `setRunning`, keyboard shortcuts, boot |
