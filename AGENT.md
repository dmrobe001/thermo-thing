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
js/scene.js         §17 the scene file (ledger, exportScene, importScene)
js/select.js        §18 selection groups, the transform box, the widget stash
js/transport.js     §16 transport controls, snapshots, keyboard shortcuts, boot
```

`js/scene.js` and `js/select.js` load before `js/transport.js` despite their higher
numbers: transport's boot calls `renderInspector()`, which draws the scene-file card
and the stash card. Load order is dependency order; the section numbers are names.

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
- `constraints` -- array of typed joint objects; each carries `type`, endpoint refs (`a`, `b`), type-specific parameters, and transient solver outputs (`_lam`, `_rows`). A pin, rod, slot or rack may also carry `pts` -- its **extra control points**, the third and further ends attached to the same joint (`constraints.js` §06.2c). `conEndpoints(con)` is the one answer to "which bodies does this couple". A rod may also carry `posable`, the one joint field that says nothing about the running physics: it releases the rod to a bare rail while the player drags a body the rod is jointed to, with the sim paused (`constraints.js` §06.2d).
- `cables` -- array of unilateral tetherball cable elements.
- `springs` -- array of linear (Hookean) spring force elements, `{type:'spring', a, b, restLen, k, sel}` (`a`/`b` are rod-style `{id,off}` endpoints).
- `rotSprings` -- array of rotational (torsional) spring force elements, `{type:'rotspring', a:{id}, b:{id}, restAngle, k, sel}`.
- `interactions` -- array of heat and mass-exchange couplings, `{type:'heat'|'flow', body:{id}, vessel:{id}, k, sel}` (`vessel.id === null` is the background). They carry no force and no constraint row; two of the same kind sharing a `body` are a *pair* and couple what they each name through it. See `VESSEL.md` §V.10 and `js/physics.js` §08.0b.
- `sim` -- simulation parameters (`h`, `beta`, `reg`, `running`, `gravity`, `g`, `bathQ`, ...).
- `cam` -- camera state (`x`, `y`, `scale`).
- `selGroup` -- the many-body selection, or null (`select.js` §18.1). It is a
  *selection*, not a scene object: nothing about it is serialized or solved. While it
  is up it owns the pose of every body it holds -- each body's position and angle are
  fixed to the box's frame, so the box's translation, rotation and uniform scale write
  them from one capture taken when the selection was made. Which couplings come with
  the bodies is derived by one rule: **every body an element names must be in the
  selection**; a background anchor does not disqualify it and travels with the box.
  Body sizes never change, so a scale spreads the parts apart, and the captured
  geometry of the members is re-read rather than multiplied (§18.2).

## Scenes are built from the scene file, not from code

A scene is described by the text format in `js/scene.js` (§17), and that format is
also the definitive statement of what a scene may contain -- the reader builds only
by calling the constructors the tool dispatch calls, and rejects any key its ledger
does not list. Two rules follow, and they are what keep the bench honest:

- **Every kind of scene object has exactly one constructor** (`makeBody`,
  `makeRectBody`, `makeVessel` in §05.2; `makeRodCon`, `makeSlotCon`, `makePinCon`,
  `makeBeltCon`, `makeCvtCon`, `makeRackCon`, `makeKnifeCon`, `makeCableCon`,
  `makeSpringCon`, `makeRotSpringCon` in §06.1/§06.2, `makeConPoint` in §06.2c;
  `makeInteraction` in §17.1),
  called from the tool dispatch (§13.5) and the scene reader (§17.4) and nowhere
  else. Do not build one from an object literal.
- **A new field on a scene object needs a row in `SCENE_SCHEMA`**, classified as
  authored (has a default, written when it differs), captured (`always:true`,
  written every time, because the pose does not imply it), or derived (absent from
  the table, recomputed on load). `node tools/scene-roundtrip.js` fails if you skip
  this.
- **A new field a *run* can change also needs an entry in that row's `state` list**,
  which is what Reset restores (§17.6, walked by §16.1). It is a separate list from
  `fields` because the two answer different questions -- a radius is in the file and
  not in the snapshot; a vessel's adiabat invariant is in the snapshot and not in
  the file -- but they sit together so adding a coordinate is one edit. The
  validator's Reset check fails if you skip this.

**No coordinate is frozen by assertion.** `static` and `lenLock` still exist, but as
*derived* fields recomputed every substep from the constraints present
(`constraints.js` §06.2b `refreshFrozen`): a body is pinned by a rod welded at both
ends to fixed ground (or to an already-pinned body), and a vessel's length is locked
by a rod between two of its own material planes. Zeroing an inverse mass is an
optimization -- it removes the coordinate from the system and lets islands split
there -- and the constraint that earned it is compiled away (`_compiled`) rather than
left as a row of zeros. Nothing may set either flag: not a tool, not the inspector,
not a scene file. See `SCENE.md` §S.8.

## Adding or moving code

Give new code a home in an existing section (and register it in that section's sub-index) or open a new section and update the list above. A stale map is worse than none.

## Section quick-reference

| File | Section | What it does |
|---|---|---|
| `js/state.js` | §04 | Canvas handles; `bodies`, `constraints`, `cables`, `springs`, `rotSprings`, `interactions`; `sim` (incl. `sim.bg`, the ambient atmosphere, and `sim.bathQ`); `cam` |
| `js/geometry.js` | §05 | `R` (rotation), `worldPt`, `makeBody`, `refreshInertia`, `setBodyMass`, `w2s`/`s2w`; §05.2d `makeVessel`/`refreshVessel`/gas state, §05.2c `epLocal`/`epWorldPt`/`epOffOf` (material endpoint offsets), §05.2e `bodyPolygon`/`clipPoly`/`contactArea` (interaction contact area) |
| `js/constraints.js` | §06 | `bodyIndex`, `epWorld`, `epFrame` (endpoint velocity columns, incl. a vessel's length column), `twoPointFrame`, `cableFrame`, `rowsFor`, §06.2b `recaptureConAngles`/`recaptureConPose` (re-read what a line joint holds off the live geometry), §06.2c extra control points (`conPoints`, `makeConPoint`, `linePointRows`, `conEndpoints`), §06.2d posable rods (`beginPosing`/`endPosing`, `withPosing`, `rodPosing`/`rodReleased`, `recapturePosable`), `makeSpringCon`, `makeRotSpringCon`, `rotSpringSpiralGeom` |
| `js/solver.js` | §07 | `solveLinear` -- dense Gauss-Jordan on the Schur complement |
| `js/physics.js` | §08 | `substep` -- §08.0b `vesselExchangeStep` (heat & mass, at frozen geometry, ahead of everything) -> forces (gravity, drag, springs, vessel centrifugal) -> §08.1b `vesselGasStep` -> constraint solve -> position integration -> energy-conservation rescale |
| `js/projection.js` | §09 | `projectPositions`, `conMaxC`, `reactionOf` |
| `js/loop.js` | §10 | `frame` -- fixed-step accumulator, calls `substep` -> `render` -> `updateHUD` |
| `js/render.js` | §11 | `render` orchestrator; `drawBody`, `drawVessel`, `drawConstraint`, `drawCable`, `drawSpring`, `drawRotSpring`, §11.4c `drawInteraction`, `drawReaction`, ... |
| `js/hud.js` | §12 | `energy` (incl. spring PE, gas internal energy and atmospheric potential), §12.1b `bathTotal` (net of the background bath), `updateHUD`, `drawSpark` |
| `js/tools.js` | §13 | `TOOLS` (incl. the heat/mass interaction tools), `setTool`, `pickBody`, `pickVessel`, `pickInteraction`, `dropInteractionsOn`, `snapAnchor`, `conHandles`, pointer handlers |
| `js/inspector.js` | §14 | `clearSelection`, `select*`, `renderInspector` (incl. the interaction panel), §14.2b `renderVesselInspector`, `updateInspectorLive` |
| `js/examples.js` | §15 | `SCENES` -- every prebuilt machine as scene-file text, with its own reasoning as `#` comments; `loadExample` is `importScene` and nothing else |
| `js/scene.js` | §17 | `SCENE_SCHEMA` (the ledger: one row per scene-object kind, carrying both the serialized `fields` and the `state` a run can change), `exportScene`/`importScene`, `clearScene`, the scene-file panel card, §17.6 `snapshotState`/`applyState`, §17.7 fragments (`exportFragment`/`pasteFragment` -- part of a bench, out and back in) |
| `js/select.js` | §18 | §18.1 the group (`selGroup`, `groupMembers`, `makeGroup`, `selectGroup`), §18.2 the transform box (`groupApply`, `groupRecapture`, the handles and their drag), §18.3 the lasso (`lassoSelect`, `lassoToggle`), §18.4 widgets (`selectionFragment`, `copySelection`, `pasteWidget`, the stash), §18.5 the group inspector and stash cards |
| `js/transport.js` | §16 | `saveState`/`restoreState` (the ledger walk of §17.6 plus solver-scratch clearing), `setRunning`, keyboard shortcuts (incl. copy/paste, §18.4), boot |
