# Scene File & the Constructible Set -- Design Note

This note works out a strategy for two things that turn out to be one thing:

1. A **human-readable import/export format** for a scene -- every body with its
   state, every constraint, force element and interaction, the sim parameters and
   the camera.
2. An **architecture that discourages scenes containing features no player could
   build**. Twice now the bench has carried an example that reached past the tool
   palette -- a coordinate frozen by assignment rather than by a constraint object,
   a field set that no tool sets, a primitive that was later removed.

The connection is the whole idea. A serialization format is a written-down
statement of what a scene *is*. If it is also the only way a scene can be built,
then "can this be expressed in the file" and "can a player build this" become the
same question, and the second problem is closed by construction rather than by
discipline.

---

## S.1 Audit: what the examples do today that the editor cannot

Before designing anything, here is the actual divergence between `js/examples.js`
(§15) and what the tool rail (§13) plus the inspector (§14) can produce. This is
narrower than expected, which is good news for the plan.

**One genuine capability gap.**

| What the example writes | Where | Editor path |
|---|---|---|
| `static: true` on a body | `heatpair` plate, `flowpair` port, `cable` spool | **None.** `makeBody`/`makeRectBody` take the flag; both tools hard-code `false` (§13.5); no inspector checkbox exists. |

Three of the ten examples build a body a player cannot build. That is the entire
list of hard-unreachable state, and it is a ten-line fix (§S.6, phase 0).

**Reachable, but only by editing after creation.** These read as "the example
reached past the tool," but each corresponds to a real two-step player action --
place the thing, then change one field in the inspector:

| What the example writes | Tool default | Inspector path |
|---|---|---|
| `rodBG(...)` -- background rod, both ends *unwelded* | welded both ends (a rigid strut) | `f_weldA` / `f_weldB` checkboxes |
| `makeSlotCon(..., false, true)` -- one end prismatic | both ends prismatic | `f_lockA` / `f_lockB` checkboxes |
| `res.lenLock = true` | off | `v_lock` checkbox |
| `setVesselGasMT(res, m, 800)` / `setVesselGasPT(res, 2.4*P, T)` | ambient | `v_T` / `v_P` fields |
| `v.w = 9.0` | 0 | `v_w` field |
| `interactions.push({... k: 2000 ...})` | 1000 / 1e-5 | `i_k` field |
| `sim.gravity = false`, camera | on | header toggle, wheel/drag |

**Reachable in kind, not to an exact value.** `knife` with `dir:[1,0]` -- the tool
derives the heading from a drag, so a player gets *a* heading, not that one. Minor,
and the file format fixes it anyway by making the direction an authored number.

**The pattern the user named, found in the wild.** "A coder constrains a coordinate
to be constant without representing that with a constraint object" already exists
twice in sanctioned form: `body.static` freezes `(x, y, th)` and `vessel.lenLock`
freezes `len`, both by zeroing an inverse mass in `refreshInertia`/`refreshVessel`
rather than by adding a constraint row. Both are defensible -- `static` is the most
common single fact in any scene and paying three solver rows for it in every scene
is a bad trade -- but they should be named as **the two sanctioned exceptions**,
made first-class (serialized, inspector-editable, documented), and the door shut
behind them: no new coordinate may be frozen by assignment.

---

## S.2 The lever: the file is the only constructor

The strategy rests on one move.

> **`js/examples.js` stops being code that builds scenes and becomes data that
> describes them.** `loadExample(kind)` becomes `importScene(SCENES[kind])`.

After that move, an example *cannot* contain an unbuildable feature, because the
only vocabulary available to it is the format's vocabulary. There is no `push` to
reach for. The failure mode is not caught by review; it is unrepresentable.

This is only sufficient if a second property holds:

> **The format's vocabulary and the editor's capabilities are one list, kept one
> by a single table** (§S.4). The importer builds exclusively by calling the same
> constructors the tools call -- `makeBody`, `makeRectBody`, `makeVessel`,
> `makeRodCon`, `makeSlotCon`, `makeSpringCon`, `makeRotSpringCon` -- and then
> applies only fields the table lists, rejecting anything else as a parse error.

An importer that ends with a generic `Object.assign(body, parsed)` gives the whole
thing away: the file could then carry any field at all and the format would stop
being a statement about what is buildable. Strictness is the feature.

And a third, to keep the two from drifting apart:

> **A round-trip validator** in `tools/`, in the shape of the existing
> `tools/vessel-check-*.js` node scripts: for every example, import -> export ->
> import -> export and assert the two exports are identical, and assert the
> resulting world is bit-identical in every field the table calls authored. A
> field added to a scene object but not to the table fails this immediately.

---

## S.3 What a scene file has to hold, and what it must not

The single hardest correctness question in the format is not the grammar. It is
which fields are *authored*, which are *captured*, and which are *derived*.

**Derived -- never serialize; recompute on load.** `mass`/`I`/`invM`/`invI` for a
vessel, `mu`, `Alat`, `invMu`, `hw`, `hh` (`refreshVessel`, §05.2d); `I`, `invM`,
`invI` for an ordinary body (`refreshInertia`); `gas.kap`, `gas.mass` given P and T
(`setVesselGasPT`); every transient solver output (`_lam`, `_rows`, `_phiRef`,
`_active`, `_C`, `_cols`, `_Lallow`, `_spoolAngle`, `_vlen0`, `_rate`); and `sel`.
Writing any of these out invites a file whose derived fields disagree with its
authored ones, and then the importer has to decide which one is the truth.

**Authored -- serialize.** Pose and velocity (`x, y, th, vx, vy, w`, and a vessel's
`len, vlen`); shape parameters (`r`, `hw`, `hh`, `bore`); `mass` for an ordinary
body and `mShell` for a vessel; `static`; `lenLock`; gas `gamma` and any two of
(P, T, m) -- **write T and mass**, since that is the pair `setVesselGasMT` takes
and it makes a hand-edited temperature do the obvious thing; every stiffness and
conductivity (`spring.k`, `rotspring.k`, `interaction.k`); every weld/prismatic
flag; every endpoint `{id, off}`; `sim` parameters; the camera.

**Captured -- serialize, and this is the subtle class.** These are quantities read
off the geometry *at the moment the object was created or a lock was toggled*, and
never recomputed since. The pose no longer implies them:

- `rod.len` and `spring.restLen` -- the length at creation. Move a body afterward
  and the rod is under load; that load is the scene.
- `rod.restAngA` / `restAngB`, `slot.restAngA` / `restAngB` -- captured by
  `captureRestAngle` on creation and on each lock toggle, and (verified) at *no
  other site*: dragging a welded body does not recapture. So a welded rod's rest
  angle is genuinely independent of the exported pose.
- `belt.restPhase`, `belt.rA`, `belt.rB`, `belt.sense`.
- `rotspring.restAngle`.
- `cable.Ltot`, `cable.localAngle`, `cable.spoolAngle`.
- `knife.dir` (body-frame).
- `sim.bathQ` -- the running total drawn from the background bath, and the
  ledger's counterweight to the gas state being written alongside it. Omit when 0.

Round-tripping a scene while *silently recapturing* the captured class would be
the format quietly editing the physics. The importer must therefore accept them
explicitly and only fall back to recapture-from-pose when a key is absent -- which
is what makes a terse hand-written file (§S.5) still legal and still do the
sensible thing.

**Which state: live or the reset baseline?** Export should write **the reset
baseline** -- what `R` restores to (`transport.js` §16.1) -- not the mid-run pose,
because that is what an author means by "the scene," and because otherwise
exporting a running sim yields a file whose own Reset behaves differently from the
sim it came from. Offer "export current state" as an explicit second command if it
is wanted; do not make it the default.

---

## S.4 The ledger

One table, in one file (`js/scene.js`), is the single definition. One row per
scene-object kind; each row names its constructor, its authored keys with defaults,
its captured keys, and the refresh call to run after loading. Export walks it.
Import walks it. The schema section of this note is generated from reading it.

Sketched:

```js
const SCENE_SCHEMA = {
  body:   { make:(f)=>makeBody(f.x,f.y,f.r,f.static),
            keys:{ x:0, y:0, r:0.38, mass:null, static:false,
                   th:0, vx:0, vy:0, w:0 },
            after:refreshInertia },
  rect:   { ... hw, hh ... },
  vessel: { make:(f)=>makeVessel(f.x,f.y,f.bore,f.len,f.static),
            keys:{ ..., mShell:null, lenLock:false, gamma:1.4, T:293.15, gm:null,
                   vlen:0 },
            after:refreshVessel },
  rod:    { make:(f)=>makeRodCon(f.a,f.b,f.weldA,f.weldB),
            keys:{ weldA:false, weldB:false },
            captured:{ len:null, restAngA:null, restAngB:null } },
  // slot, pin, belt, cvt, knife, cable, spring, rotspring, heat, flow
};
```

Two payoffs beyond export/import.

**It subsumes `saveState`.** `transport.js` §16.1 already maintains a second,
parallel definition of "the scene's state" -- and its own comments record that it
has had to be patched twice as state was added (the vessel's fourth coordinate and
gas, then `sim.bathQ`), each time after a bug where Reset silently changed the
scene's energy. That is the same list, maintained twice. Point `saveState` at the
ledger's state-class fields and the duplication is gone. (Use the field list, not
the serialized text: `saveState` runs on every inspector keystroke, and there is no
reason to pay string formatting for it.)

**It gives the "is this buildable" check something to run against.** A scene object
kind with no editor path is a row with no tool -- visible in one place, not spread
across three files.

---

## S.5 Format: a line-oriented DSL, not JSON

Constraints on the choice: no build step and no dependencies (the project is plain
globals; `index.html` opens straight off disk), no network fetch (under `file://`,
`fetch` of a sibling file fails -- so scene text ships embedded as string literals
in a JS file, and the *file* half of import/export is paste, drag-drop, and a
`Blob` download), diffable in git, and readable and writable by a person.

- **JSON** is free (`JSON.parse`), but has no comments, quotes every key, and diffs
  badly. A scene is a parts list; JSON renders it as a wall.
- **YAML** would read well and needs a parser dependency the project does not have
  and should not acquire for this.
- **A flat `key=value` DSL** costs perhaps eighty lines of parser and gets: one
  object per line (so a diff shows exactly which part changed), `#` comments (so an
  example can carry the paragraph of reasoning that today lives in `examples.js`
  -- and that reasoning is worth keeping), and a file that reads like a bill of
  materials.

Every scene object in the data model is flat except for `{id, off}` endpoints and
`gas`, so the grammar stays small. Recommend the DSL; JSON-lines is the fallback if
the parser proves annoying.

Sketch, `heatpair` as it would be written:

```
# thermo-scene 1
# A hot reservoir warming a working vessel through a fixed plate. What couples
# the two gases is a PAIR of heat interactions sharing an ordinary static plate.
sim    gravity=off g=9.8 bg.P=101325 bg.T=293.15
cam    x=0 y=2.6 scale=64

rect   1  x=0     y=2.6  w=2.50 h=0.12  static
vessel 2  x=-1.25 y=2.6  bore=0.55 len=1.80  T=800  lenlock
vessel 3  x=1.15  y=2.6  bore=0.90 len=0.90
rod    4  bg(1.15,1.75) -- 3@(0,0)  weld=both  len=0.85
heat   5  body=1 vessel=2 k=2000
heat   6  body=1 vessel=3 k=2000
```

Grammar notes:

- `<kind> <id> <field>=<value> ...`; bare words are boolean flags (`static`,
  `lenlock`). Ids are preserved literally -- they are what the inspector shows and
  what other lines reference -- and `uid` resumes at `max + 1`.
- Endpoints: `<id>@(ox,oy)` for a body-local offset, `bg(x,y)` for the background,
  `--` between the two ends of a two-endpoint object. A bare `<id>` means `@(0,0)`.
- Omitted keys take the ledger's default; omitted *captured* keys recapture from
  the pose. So the terse hand-written form above is legal, and a full export is the
  same file with every captured key written out.
- Bodies before anything that references them; the exporter emits in that order and
  the importer requires it (a forward reference is an error with a line number, not
  a silent null).
- `# thermo-scene 1` is required. An unknown major version is refused outright.
- **Any unrecognized key or kind is an error**, reported with its line. This is the
  enforcement surface -- see §S.2.

---

## S.6 Phasing

**Phase 0 -- close the gap and write the ledger.** Add the `static` checkbox to the
body, rectangle and vessel inspectors (with the `refreshInertia`/`refreshVessel`
call and a `saveState`, exactly like the existing `lenLock` toggle). Write
`SCENE_SCHEMA` in a new `js/scene.js` (§17). No behavior change; every current
example becomes legal.

**Phase 1 -- the writer.** `exportScene()` walking the ledger, and an Export button
that drops the text into a textarea with a copy button (and a `Blob` download
alongside). Low risk, immediately useful, nothing else changes. Read the output of
all ten examples by eye; that is the format review.

**Phase 2 -- the reader.** `importScene(text)` built strictly on the constructors,
with line-numbered errors and hard rejection of unknown keys. Import UI: paste into
the same textarea, plus drag-drop a file onto the canvas.

**Phase 3 -- flip the examples, and shut the door.** Convert each example to scene
text in `js/scenes.js` (embedded string literals, `file://`-safe), keeping the
explanatory comments as `#` lines. `loadExample` becomes a two-line dispatch. Add
`tools/scene-roundtrip.js` per §S.2. Then state the invariant in `AGENT.md` where
future agents will read it: *outside `js/geometry.js`, `js/constraints.js` and the
tool dispatch, no file pushes onto `bodies`, `constraints`, `cables`, `springs`,
`rotSprings` or `interactions`; scenes are built from scene text.*

**Phase 4 -- fold in `saveState`.** Point `transport.js` §16.1 at the ledger's
state-class fields, deleting the parallel definition described in §S.4.

Phases 0-2 are independently useful and independently shippable. Phase 3 is where
the architectural claim gets paid for; Phase 4 is cleanup that only becomes cheap
once the ledger exists.

---

## S.7 What this does not fix

- **It does not stop a future tool from being removed** and leaving a scene file
  that references it. It converts that from a silently-wrong scene into a
  load-time error naming the line -- which is the fix that matters, but the example
  still has to be rewritten by hand.
- **It does not make the two sanctioned coordinate freezes** (`static`, `lenLock`)
  into constraint objects. §S.1 argues they should stay properties; the strategy
  makes them explicit and serialized rather than eliminating them.
- **It is not a save format for a running simulation.** Exporting the reset
  baseline (§S.3) is a deliberate narrowing; mid-run capture is a separate command
  if it is ever wanted.
- **It does not anticipate the signal layer** (`DEVELOPMENT.md` §5). When signal
  wires arrive they are a new ledger row and a new line kind -- which is the test
  of whether the ledger was factored right.
