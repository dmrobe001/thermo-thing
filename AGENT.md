# Navigating the Implementation

This document is the primary reference for developers and AI agents working in this codebase. Read it before touching any code.

## File layout

The simulator is split into a thin HTML shell and one JavaScript file per logical section:

```
index.html          §01 document head · §02 all CSS · §03 body markup
js/state.js         §04 world state & globals
js/expr.js          §19 the expression language for numeric fields
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

`js/expr.js` loads early despite its higher number: it depends on nothing (it is the
language, not the bindings -- those are `js/scene.js` §17.8), and both the scene
reader and the inspector call it. `js/scene.js` and `js/select.js` load before
`js/transport.js` despite their higher numbers: transport's boot calls `renderInspector()`, which draws the scene-file card
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
- `constraints` -- array of **couplings**: everything that produces solver ROWS, as opposed to `bodies`, which is everything that produces COLUMNS. Each carries `type` and transient solver outputs (`_lam`, `_rows`, `_roles` -- what each of those rows *is*, so §09.3 can look a multiplier up by name). Two of the kinds are the scene model's spine and neither has the `a`/`b` endpoints the others do:
  - a **vertex** (`type:'vertex'`, §06.2e) is a named point carrying a list of **incidences**, one per body it touches, each saying where the vertex sits in that body's frame and whether it is `join`ed and `weld`ed there -- and, for a line, whether it may `slide`. What the *panels* list is wider than that and is not stored anywhere: every body whose **extent** covers the point, joined or not, with `joined` as the only control either list needs (`VERTEX.md` §X.11). It compiles to two rows per joined incidence past the primary and one per welded incidence past the first. It replaced the pin.
  - a **line** (`type:'line'`, §06.2f) is a straight massless bar: a body to anything naming one, with a numeric `id` from the same allocator bodies use, but with no coordinates of its own. Its frame, its extent, its origin and its joints' stations are all derived from the vertices joined to it, so it holds only `soft` (a compliance), `posable` and its meshing disks. It replaced the rod, the slot, the rack and the linear spring.

  Both live here because a coupling is what they are, which is why islands, the row assembly, the position projection, Reset and the selection's membership rule work on them with no per-kind branching. See `VERTEX.md`. `conEndpoints(con)` is the one answer to "which bodies does this couple". A line may also carry `posable`, the one field that says nothing about the running physics: it releases the bar to a bare rail while the player drags a body it touches, with the sim paused (`constraints.js` §06.2d).
- `cables` -- array of unilateral tetherball cable elements.
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
  `makeRectBody`, `makeVessel` in §05.2; `makeBeltCon`, `makeCvtCon`, `makeKnifeCon`, `makeCableCon`,
  `makeRotSpringCon` in §06.2, `makeVertex`/`makeVertexOn` in §06.2e, `makeLine` in
  §06.2f; `makeInteraction` in §17.1),
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

## Numbers are expressions, and they are not stored

Every numeric field -- in the inspector panel and in a scene file alike -- is parsed
as arithmetic (`js/expr.js` §19): `2*pi/3`, `0.4*sqrt(2)`, `bg.P/2`, `b3.x+b3.r`.
Two rules bound it, and both matter more than the feature does:

- **The vocabulary is the ledger.** What a name may mean is bound in `js/scene.js`
  §17.8, and a body's properties are exactly the numeric fields its `SCENE_SCHEMA`
  row lists -- so a disk offers `x y r mass th vx vy w`, the ambient is `bg.P`/`bg.T`
  because that is what the `sim` line calls it, and adding a field to the ledger adds
  it to the vocabulary with no second table to update. There are two environments and
  one vocabulary: `worldExprEnv()` reads the live bench (what a panel field is typed
  against), `sceneExprEnv()` reads the file's own text (so a scene stays a
  self-contained document, and may name a body defined further down).
- **What is kept is the number.** Nothing stores the text and nothing re-evaluates
  it. Type `b3.x` into a body's x and it lands where body 3 is *now* and stays there.
  A value that has to keep following another value is a constraint or an
  interaction -- that is what those are, and they are solved rather than
  re-substituted. Expressions exist so that *initial* geometry can be declared
  exactly. See `SCENE.md` §S.10.

## Adding or moving code

Give new code a home in an existing section (and register it in that section's sub-index) or open a new section and update the list above. A stale map is worse than none.

## Section quick-reference

| File | Section | What it does |
|---|---|---|
| `js/expr.js` | §19 | The expression language every numeric field is read through: `parseExpr`/`evalExpr`, `EXPR_CONSTS`, `EXPR_FUNCS`. Knows nothing of the world -- names come from an environment (§17.8) |
| `js/state.js` | §04 | Canvas handles; `bodies`, `constraints` (the couplings: vertices, lines, belt, cvt, knife), `cables`, `rotSprings`, `interactions`; `sim` (incl. `sim.bg`, the ambient atmosphere, and `sim.bathQ`); `cam` |
| `js/geometry.js` | §05 | `R` (rotation), `worldPt`, `makeBody`, `refreshInertia`, `setBodyMass`, `w2s`/`s2w`; §05.2d `makeVessel`/`refreshVessel`/gas state, §05.2c `epLocal`/`epWorldPt`/`epOffOf` (material endpoint offsets), §05.2e `bodyPolygon`/`clipPoly`/`contactArea` (interaction contact area) |
| `js/constraints.js` | §06 | `bodyIndex`, `epWorld`, `epFrame` (an endpoint's velocity columns, incl. a vessel's length column), §06.1b `lineFrameOf`/`frameAngleRow` (a bar's own frame -- the third frame kind beside a body's and the background's, owning no coordinates), §06.2e the vertex (`makeVertex`/`makeVertexOn`/`vertexWorld`/`verticesOn`, plus `vertexSites`/`verticesInExtent` -- the same relation widened to what the point is *inside*, which is what the panels list), §06.2f the line (`makeLine`/`lineFrame`/`lineJoints`/`lineIsBar`/`lineSegments`/`lineStationAt`), `cableFrame`, `rowsFor` (every row tagged with its `role`), §06.2b `recaptureConAngles`/`recaptureConPose` (re-read what a line joint holds off the live geometry), §06.2d the pose-time release (`beginPosing`/`endPosing`, `withPosing`, `linePosing`/`lineReleased`, `recapturePosable`), `makeRotSpringCon`, `rotSpringSpiralGeom` |
| `js/solver.js` | §07 | `solveLinear` -- dense Gauss-Jordan on the Schur complement |
| `js/physics.js` | §08 | `substep` -- §08.0b `vesselExchangeStep` (heat & mass, at frozen geometry, ahead of everything) -> forces (gravity, drag, springs, vessel centrifugal) -> §08.1b `vesselGasStep` -> constraint solve -> position integration -> energy-conservation rescale |
| `js/projection.js` | §09 | `projectPositions`, `conMaxC`, `reactionOf` (reads a multiplier by its row's `role`, not by counting row order) |
| `js/loop.js` | §10 | `frame` -- fixed-step accumulator, calls `substep` -> `render` -> `updateHUD` |
| `js/render.js` | §11 | `render` orchestrator; `drawBody`, `drawVessel`, `drawVertex`/`drawLabel`, `drawConstraint` (incl. the line: a bar between its extremes, a rail across the viewport), `drawCable`, `drawRotSpring`, §11.4c `drawInteraction`, `drawReaction`, ... |
| `js/hud.js` | §12 | `energy` (incl. spring PE, gas internal energy and atmospheric potential), §12.1b `bathTotal` (net of the background bath), `updateHUD`, `drawSpark` |
| `js/tools.js` | §13 | `TOOLS` (incl. the vertex and line tools and the heat/mass interaction tools), `setTool`/`setLineMode`, `pickBody`, `pickVertexAt`, `joinVertexToLine`, `pickVessel`, `pickInteraction`, `dropInteractionsOn`, `snapAnchor`, `conHandles`, pointer handlers |
| `js/inspector.js` | §14 | §14.0 `numRow`/`numVal`/`wireNumIns` (one editable number: arithmetic in, a number committed, the arrow keys stepping the value); `clearSelection`, `select*`, `renderInspector` (incl. the interaction panel, the vertex panel and the vertex list a body shows), §14.2b `renderVesselInspector`, §14.2c the incidence row (`incidenceRow`/`wireIncidenceRows`/`setIncidenceJoin`/`commitIncidenceOff`/`commitIncidenceStation`) -- the one row a vertex's panel, a body's and a line's are all built out of, `updateInspectorLive` |
| `js/examples.js` | §15 | `SCENES` -- every prebuilt machine as scene-file text, with its own reasoning as `#` comments; `loadExample` is `importScene` and nothing else |
| `js/scene.js` | §17 | §17.8 `worldExprEnv`/`sceneExprEnv` (what a name in a numeric field means -- the ledger is the vocabulary); `SCENE_SCHEMA` (the ledger: one row per scene-object kind, carrying both the serialized `fields` and the `state` a run can change), `exportScene`/`importScene`, `clearScene`, the scene-file panel card, §17.6 `snapshotState`/`applyState`, §17.7 fragments (`exportFragment`/`pasteFragment` -- part of a bench, out and back in) |
| `js/select.js` | §18 | §18.1 the group (`selGroup`, `groupMembers`, `makeGroup`, `selectGroup`), §18.2 the transform box (`groupApply`, `groupRecapture`, the handles and their drag), §18.3 the lasso (`lassoSelect`, `lassoToggle`), §18.4 widgets (`selectionFragment`, `copySelection`, `pasteWidget`, the stash), §18.5 the group inspector and stash cards |
| `js/transport.js` | §16 | `saveState`/`restoreState` (the ledger walk of §17.6 plus solver-scratch clearing), `setRunning`, keyboard shortcuts (incl. copy/paste, §18.4), boot |
