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
| `rod ... posable` -- a rod released to a rail while posing | off | `f_posable` checkbox |
| `makeSlotCon(..., false, true)` -- one end prismatic | both ends prismatic | `f_lockA` / `f_lockB` checkboxes |
| `res.lenLock = true` | off | `v_lock` checkbox |
| `setVesselGasMT(res, m, 800)` / `setVesselGasPT(res, 2.4*P, T)` | ambient | `v_T` / `v_P` fields |
| `v.w = 9.0` | 0 | `v_w` field |
| `interactions.push({... k: 2000 ...})` | 1000 / 1e-5 | `i_k` field |
| `sim.gravity = false`, camera | on | header toggle, wheel/drag |

**Reachable in kind, not to an exact value.** `knife` with `dir:[1,0]` -- the tool
derives the heading from a drag, so a player gets *a* heading, not that one. Minor,
and the file format fixes it anyway by making the direction an authored number.

**The pattern named in the brief, found in the wild.** "A coder constrains a
coordinate to be constant without representing that with a constraint object"
already existed twice: `body.static` froze `(x, y, th)` and `vessel.lenLock` froze
`len`, both by zeroing an inverse mass rather than by adding a constraint row.

This note originally argued they should stay, as two sanctioned exceptions made
first-class. That was wrong, and §S.8 replaces it: they are now derived from the
constraints present. The optimization survives -- it is the reason to want them --
but nothing asserts it.

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
  `captureRestAngle` on creation and on each lock toggle. Dragging a welded body
  does not recapture, so a welded rod's rest angle is genuinely independent of the
  exported pose -- with two deliberate exceptions, both of them rods that are being
  moved *by hand* through geometry the solver is no longer holding: a **grounded**
  body's anchors (`recaptureGrounding`, whose rows are compiled away, §S.8) and a
  **posable** rod's own length, welds and stations, re-read after each step of a
  pose drag that reached it -- because for that step the rod was released and held
  none of them (`constraints.js` §06.2d). Both are the editor writing the scene, which is what
  an editor is for; neither happens while the sim runs.
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
  rod:    { make:(f)=>makeRodCon(f.a,f.b,f.weldA,f.weldB,f.posable),
            keys:{ weldA:false, weldB:false, posable:false },
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

`heatpair`, exactly as the exporter writes it:

```
scene 1

sim gravity=off
cam x=0 y=2.6 scale=64

# bodies
rect 1 x=0 y=2.6 width=2.5 height=0.24 static
vessel 2 x=-1.25 y=2.6 bore=0.55 len=1.8 P=276513.730172 T=800 lenlock
vessel 3 x=1.15 y=2.6 bore=0.9 len=0.9 P=101325 T=293.15

# constraints
rod bg(1.15,1.75) -- 3 len=0.85 weld=both restAngA=1.57079632679 restAngB=1.57079632679

# interactions
heat body=1 vessel=2 k=2000
heat body=1 vessel=3 k=2000
```

Grammar notes:

- `<kind> [<id>] [<endpoints>] <field>=<value> ...`; bare words are boolean flags
  (`crossed`, `posable`, and -- in the version-1 sketch above -- `static`, `lenlock`). Only **bodies** carry an id, because they are the
  only things anything refers to -- nouns are named, relations are anonymous, which
  is the data model exactly. Body ids are preserved literally (they are what the
  inspector shows) and `uid` resumes at `max + 1`.
- Endpoints: `7` for body 7 at its centre, `7@(0.1,-0.2)` for a body-local offset (a
  *material* label on a vessel, so the second number is a fraction of the length),
  `bg(x,y)` for the fixed background, `--` between the two ends. No spaces inside
  the parentheses -- a line tokenizes on whitespace.
- **`pt` is the one repeatable key.** A pin, rod, slot or rack may carry extra
  control points beyond its two named ends (`js/constraints.js` §06.2c), and a line
  writes one `pt=` per point, in order. Each token is a whole point in one word --
  the endpoint syntax above, then slash-separated options: `s=` its captured station
  along the line, `lock` plus `restAng=` its rotation lock, or the bare word `pinion`
  on a rack. Which of those a token may say depends on the kind it sits on, and
  saying anything else is a load error, exactly as an unknown key on the line itself
  is: `pt=3/pinion` on a slot and `pt=3/s=0.5` on a pin are both refused.
- **Every number is an expression** (§S.10): `r=0.25*2`, `len=hypot(3,4)`,
  `P=bg.P/2`, `x=b3.x+b3.r`. A line tokenizes on whitespace, so an expression in a
  file carries none. What is stored is the number it works out to; the exporter
  writes digits.
- A rectangle's dimensions are `width`/`height`, not `w`/`h`: `w` is the angular
  velocity every body carries.
- A vessel's gas is written as pressure and temperature -- the two faces of it a
  person reasons about, and a complete encoding at a known volume, since the gas
  mass and the adiabat invariant both follow.
- Omitted keys take the ledger's default, which for an object built by a constructor
  *is* what the constructor produced -- so the terse hand-written form is legal and
  means the obvious thing. `sim` and `cam` are the exception (§S.4): they are not
  rebuilt, so an omitted key there is the default, not "keep what is there".
- Line order does not matter: the reader builds bodies in a first pass. The exporter
  still writes them first, because a file a person reads should introduce a thing
  before mentioning it.
- `scene 3` must be the first non-comment line. An unknown version is refused.
  Version 2 dropped `static` and `lenlock` (§S.8); version 3 added `pt` and, with
  it, rewrote the rack -- `rack <pin> -- <pin> pt=<pinion>/pinion` where a version 2
  file wrote `rack <anchor> -- <pinion> angle=...`. That one had to move the version
  number rather than just extend the ledger, because the old two-token form still
  parses under the new reading and would silently mean something else.
- **Any unrecognized key or kind is an error**, reported with its line, and nothing
  is touched until the whole file has parsed -- so a bad file leaves the current
  bench exactly as it was. This is the enforcement surface; see §S.2.

**Numeric precision** was the one thing the plan did not anticipate, and it took
three attempts. Numbers are written to **12 significant digits**. Nine reads better
on the captured fields (`len=2.5019992`) but throws away ~1e-9, which the dynamics
amplify: an exported and reimported four-bar had visibly diverged after two seconds,
and a file that does not reproduce the scene it was written from is not a scene
file. Printing *exactly* -- JavaScript's `Number`-to-`String` is the shortest decimal
that parses back to the identical double -- makes every stored field bit-exact, but
a vessel sitting at ambient then exports as `P=101325.00000000001`, because pressure
and temperature are a change of variables away from what a vessel stores; that
exactness is fake, and the text was not even stable under a second round trip.
Twelve digits reads as the value in every case that has one, keeps ~1e-12 of the
state, and is stable under repeated round trips because the rounding absorbs exactly
the last-bit wobble that made exact printing unstable.

---

## S.6 Phasing

**Phase 0 -- close the gap and write the ledger. `[done]`** The `static` checkbox is
on the body, rectangle and vessel inspectors, going through a new `setBodyStatic`
(§05.2) that also zeroes the velocities the substep was about to discard anyway.
`SCENE_SCHEMA` lives in `js/scene.js` (§17.1). Every current example is now legal.

Phase 0 also turned up something the audit missed: five constraint kinds -- pin,
belt, CVT, knife and cable -- had *no* constructor at all, only object literals at
each call site. §S.2's "the reader builds only through the constructors the tools
call" was not enforceable against a kind that had none, so they were added
(`js/constraints.js` §06.2, plus `makeInteraction` in §17.1) and the tool dispatch
and the examples both switched onto them. Every kind now has exactly one
constructor, called from exactly two places.

**Phase 1 -- the writer. `[done]`** `exportScene()` (§17.3) walks the ledger; the
inspector's empty-bench panel carries a scene-file card with Export, Import, Copy
and Download.

**Phase 2 -- the reader. `[done]`** `importScene(text)` (§17.4) is a parse pass that
validates everything it can without touching the world -- syntax, unknown kinds and
keys, duplicate and dangling body ids -- and only then a commit pass that clears the
bench and builds. Errors carry their line number. A scene file can also be dropped
onto the canvas. `tools/scene-roundtrip.js` is the validator, and it checks four
things over all eleven examples: the export round-trips byte-for-byte; every ledger
field survives; the reader rejects an unknown kind, an unknown key, a dangling
reference, a duplicate id, a bad version, a flag given a value, a prototype key and
a background pin, each leaving the bench standing; and -- the one that catches a
*derived* field the reader recomputed wrongly, which no comparison of the file
itself can see -- two seconds of the real `substep` on the world the tools built and
on the world rebuilt from its export agree.

**Phase 3 -- flip the examples, and shut the door. `[done]`** All eleven examples
are scene text in `js/examples.js` (§15) -- embedded string literals, so no `fetch`
and no build step -- and `loadExample` is `importScene` and nothing else. A separate
`js/scenes.js` turned out not to be worth it: §15 was already "the examples", and
now it holds them.

Three things fell out that the plan did not call:

- **"Clear bench" stopped being a special case.** It is the scene with nothing in
  it. The loader has no branch for it.
- **The examples' prose moved into the files.** Every paragraph that explained why
  the gas spring is not a gas-spring primitive, or what makes the heat pair a heat
  engine with nothing that knows what a heat engine is, is now a `#` comment in the
  scene text. Exporting an example hands you the explanation along with the
  machine, and clicking one drops its file into the scene-file card -- which is the
  shortest introduction to the format there is.
- **Canonicality needs its own guard.** A checked-in scene file can drift from what
  the exporter would write, either by hand-editing or because a ledger change moved
  the canonical form. The validator now asserts that each example, stripped of
  comments and blank lines, is exactly the export; `node tools/scene-roundtrip.js
  --canon <name>` prints the canonical text, which is how you regenerate one.

The migration was verified against the code it replaced: every one of the twelve
scenes is byte-identical to what the old imperative `loadExample` produced.

The invariant now holds and is stated in `AGENT.md`: *outside the constructors in
`js/geometry.js` and `js/constraints.js`, only the tool dispatch (§13.5) and the
scene reader (§17.4) push onto `bodies`, `constraints`, `cables`, `springs`,
`rotSprings` or `interactions`.* Every kind of scene object has exactly one
constructor, and it is called from exactly those two places.

**Phase 4 -- fold in `saveState`. `[done]`** `transport.js` §16.1 is now four lines
of `snapshotState()` / `applyState()` (§17.6) plus the transient-clearing that is
genuinely transport's own. The parallel definition described in §S.4 is gone.

The plan said to reuse "the ledger's state-class fields", which turned out to be
wrong in one specific way worth recording: *what a run can change* and *what the
file writes* are related lists but not the same list. A body's radius is in the file
and not in the snapshot; a vessel's adiabat invariant is in the snapshot and not in
the file, because the file writes the readable view (pressure and temperature) and
Reset must not go through a change of variables it runs on every press of R. So each
row carries a `state` list alongside its `fields` -- two lists, but adjacent, in one
place, so adding a coordinate is one edit.

That refactor also made a check possible that the codebase never had: load a scene,
run it, press R, and assert every field is exactly back -- including the derived ones
the snapshot does not carry and the restore has to recompute. A state field missing
from the ledger fails it. This is the check that would have caught both historical
bugs the §16.1 comments describe.

All four phases are done. The architectural claim is paid for: there is no longer a
code path by which a scene can contain something the editor cannot build.

---

## S.7 What this does not fix

- **It does not stop a future tool from being removed** and leaving a scene file
  that references it. It converts that from a silently-wrong scene into a
  load-time error naming the line -- which is the fix that matters, but the example
  still has to be rewritten by hand.
- **It does not compute the general frozen set.** §S.8 recognizes two structural
  patterns, not every arrangement that pins a body.
- **It is not a save format for a running simulation.** Exporting the reset
  baseline (§S.3) is a deliberate narrowing; mid-run capture is a separate command
  if it is ever wanted.
- **It does not anticipate the signal layer** (`DEVELOPMENT.md` §5). When signal
  wires arrive they are a new ledger row and a new line kind -- which is the test
  of whether the ledger was factored right.

> The first such test has since been run, and it was not the signal layer: bulk
> selection needed to write out *part* of a bench and read it back into another one.
> That turned out to be the same ledger, the same reader and one new function on each
> side of it (§S.9) -- no new fields, no second format.



---

## S.8 Freezing is derived, not asserted

Version 2 of the format removed `static` and `lenlock`. They were the last two
fields a file could use to freeze a coordinate by saying so, which made them exactly
what the rest of this note exists to prevent. What replaced them keeps the
optimization and drops the assertion: both flags are still there, but **derived**,
recomputed every substep from the constraints actually present (`constraints.js`
§06.2b `refreshFrozen`).

### The two patterns

- **A rod welded at both ends grounds its far end.** Both ends welded pins distance,
  direction and orientation, so applied between fixed ground and a body it removes
  all three of that body's coordinates. Transitively: a body double-welded to an
  already-grounded body is grounded too.
- **A rod with both ends on the same vessel locks its length.** Its two ends ride
  different material planes, so its pose columns cancel exactly (which is also why
  the same rod on a *rigid* body is degenerate, and why the tool refuses it there)
  and what it holds is `len`. A reservoir is a vessel with a strut inside it.

Both are **structural**: they depend on what is attached, never on the current
configuration. Nothing freezes or thaws as a mechanism swings through a pose.

The one thing that suspends either is a rod marked **posable**, and what suspends it
is the *gesture*, never the configuration: while the player drags a body with the sim
paused, a posable rod jointed to that body is released to a bare rail and holds
nothing -- so it grounds nothing and locks no length either (`constraints.js` §06.2d).
It is rigid again, and freezing again, the moment the drag step is over, at the
geometry the drag reached. A posable ground strut whose body stayed frozen would be a
contradiction: the whole point of marking it is to slide that body along it.

### Freezing is per coordinate, and a vessel is where that shows

A vessel's fourth coordinate moves its own material: a point at material fraction
`f` sits `f*len` from the centre. So a double-welded ground rod pins a vessel's pose
only at the **mid-plane**, `f = 0`, whose world position has no length dependence.
Welded to a cap, it fixes the cap and not the body -- the centre still rides the
length, which is the entire mechanism of the gas spring.

This also fixed a real bug the brief pointed at. `refreshVessel` zeroed `invMu` on
`static || lenLock`, so any fixed vessel also had a frozen length. It is now
`lenLock` alone, and `invMdiag` no longer short-circuits all four coordinates on one
flag. The heat pair's working vessel -- pose pinned, length entirely free -- could
not have existed under the old rule; it is now what the mid-plane weld means.

### The freezing constraint is compiled away

A constraint that has frozen the coordinates it touches has nothing left to solve:
every column it would write is zero. Left in, it is a row of zeros that only the
Tikhonov term keeps solvable, reporting a reaction read off the regularizer rather
than off the mechanism. So it is marked `_compiled` and skipped by both the substep's
row assembly and the position projection. That is the "remove them from the equation"
the brief asked for, applied to the row as well as the coordinate.

Two consequences worth stating plainly:

- **A grounding constraint reports no reaction.** It is not solved for, so there is
  no multiplier to read. This is a real loss against a project whose stated
  commitment is that the constraint library doubles as an instrumentation layer, and
  it is the price of the optimization. It could be recovered -- for a fully frozen
  body with a single anchor, the anchor carries exactly the net of everything else
  acting on it, which the substep already computes -- but that is not built. The
  affected joints are exactly the ground welds and vessel struts.
- **A frozen body moved by hand needs its anchors recaptured.** Nothing in the solver
  will pull them back, because the rows that would have are gone. `recaptureGrounding`
  (§06.2b) does it, called from the drag path and the inspector's pose fields.

### What is not recognized

Other arrangements genuinely pin a body -- three pin-ended rods to the ground, or one
pin-ended rod and one weld. They are simply not optimized: the solver holds them
exactly as it always has, at the cost of the rows and the island split.

Recognizing the general case means asking which coordinates lie outside the nullspace
of the constraint Jacobian, which is a rank computation over the whole system every
step. Three things make that the wrong trade here. It is **configuration-dependent**:
a four-bar at a singular pose momentarily loses a degree of freedom, and freezing it
would be wrong the instant it moves off. It cannot use the **nonholonomic** rows
(knife, CVT) at all, which restrict velocity without restricting position. And it is
**expensive** in exactly the place this project's dense solver is already the
bottleneck. The structural rules above cost one pass over the constraint list.

### Verified

`tools/posable-check.js` covers the suspension: that a posable ground strut is frozen
until a drag on its own body starts, free inside that posing scope, and frozen again
-- at the posed geometry -- as soon as it ends, while a posable rod the drag never
reached is never released at all.

`tools/scene-roundtrip.js` §4 checks each rule and each case that separates them --
the mid-wall weld that pins a pose and leaves a length free, the cap weld that pins
neither, the strut that locks a length without pinning a pose, the one-ended weld
that pins nothing, transitivity, compilation, and that deleting the rod thaws the
body again. The migration was checked against the behaviour it replaced: nine of the
eleven examples are bit-identical over three seconds of substeps, and the two that
differ are the vessels that became pose-frozen, where the freeze is *more* exact than
the solve it replaced (x = 1.15 rather than 1.1499999999958).

---

## S.9 Fragments: a widget is part of a scene, written in the same format

Bulk selection wanted three things the format did not yet have a name for: keep this
part of a bench, put it down again somewhere else, and hold a library of such parts.
All three are the same object -- a **fragment**: a scene file with no `sim` and no
`cam` line, listing only some of the bench's objects.

### Why not a second format

The obvious alternative is a purpose-built clipboard structure -- deep-copy the
selected objects, keep them in an array, re-id them on paste. It was rejected for the
reason §S.2 exists at all. A second serialization is a second answer to "what can a
scene object hold", and it drifts: the field that gets forgotten in the copy path is
found the day someone pastes a widget and its welds come back unwelded. Reusing the
ledger means a widget's contents are checked by the same reader, against the same one
list of fields, as everything else. A field added to `SCENE_SCHEMA` is in the
clipboard the same afternoon, and `tools/scene-roundtrip.js` still guards it.

It also buys three properties that were not the point but turn out to matter:

- **A fragment is a legal scene file.** Drop one on the canvas and it loads as a
  scene, with `sim` and `cam` at their defaults. Paste one into the scene card and
  press Import and it replaces the bench. The reader cannot tell the difference,
  because there is none.
- **A whole scene file is a legal fragment.** That is the whole implementation of
  "import a scene string as a new widget": the stash card takes any text the reader
  accepts, and a paste simply ignores whatever `sim` and `cam` lines it carried. A
  paste adds parts; it does not take over the world they land in.
- **A widget is readable and mailable.** It is the same text the scene card shows, so
  a part can be pasted into a message, kept in a file, or hand-edited.

### What must be renumbered, and what must not

Body ids are the one thing a fragment cannot carry verbatim: they are whatever the
bench it was cut from was using, and the bench it lands in has its own. So a paste
renumbers -- each body takes a fresh id from `uid`, and every reference follows
(`remapItem`, §17.7). The places a reference can hide are exactly the ones the
reader's own dangling-id check already walks, which is why that check and the remap
are the same short list rather than two lists that could disagree.

Everything else is carried literally, captured fields included. A rod's rest length
travels with the rod; a belt's phase travels with the belt. A pasted widget is the
part as it was, not the part as its new pose would imply -- which is the same rule
`always:true` states for a file (§S.3), applied to a piece of one.

### What comes with the bodies

A selection is a set of BODIES. Which couplings travel with them is derived, by one
rule: **a constraint, cable, spring or interaction belongs to the selection when
every body it names is in it.** A background anchor does not disqualify anything --
the background is not a body, it is a point, and that point travels with the widget.

The narrower reading considered first was "at least two of its bodies are in the
selection", which is what this rule reduces to for anything that names no background.
The two part ways on exactly the elements anchored to ground -- and those are the ones
that matter most, because in this engine a rod welded at both ends to the background
is *the* way anything is pinned (§S.8). Under the narrower rule, stashing a pendulum
would hand you back a loose disk. Under this one you get the pendulum, hanging from a
ground point that moved with it.

The cost is that a selection can move a "pinned" body by moving what pins it. That is
the same thing the single-body pose drag already does (it moves the body and
recaptures its grounding rod, `constraints.js` §06.2b), and it is what makes a
grounded machine placeable at all.

### The transform, and what it re-reads

A selection carries a box, and while the box is up every body in it has its pose
*fixed to the box's frame* -- translation, uniform scale and rotation, with each body
turning by exactly the box's own change in angle. Body **sizes never change**: a
radius, a rectangle's half-extents, a vessel's bore and length are the parts
themselves, and scaling spreads the parts apart rather than growing them.

That last decision is what makes the couplings' captured geometry a real question,
because a scale is then not a similarity of the whole scene: the centres spread while
the body-frame anchor offsets did not, so the distance between two anchor points is
*not* the old distance times the scale. Three cases, and `select.js` §18.2 is where
they live:

| the transform | what happens to captured fields |
|---|---|
| translate | nothing. Every distance and relative angle between the points a member names is invariant, because they all moved together. |
| rotate | invariant too, with two exceptions that measure against the fixed world rather than against the selection: a belt's phase (`rA*thA - sense*rB*thB`, an angle sum with unequal weights) and a background-referenced rotational spring's rest angle. Both shift analytically from the capture, so an authored stress survives. The rest angles of welds and prismatic locks are re-read -- not because their value changes, but because they are measured against a raw `atan2` whose branch the turn may have crossed, and the re-read re-seeds both sides of that comparison together. |
| scale | every joint's geometry is re-read from the live pose: a rod's length, a control point's station, every rotation lock's rest angle. The new value the geometry shows is the only correct one, and it is exactly what building the joint there would have captured -- which is what keeps a scaled mechanism assembled instead of snapping the moment it runs. A spring's rest length and a cable's paid-out length are the opposite case: lengths the *element* owns rather than distances the pose implies, so they scale, and an authored stretch or slack survives. |

Two consequences worth stating plainly, because both are choices and not accidents:

- **The box is a frame, not an accumulation.** Every transform is computed from one
  capture taken when the selection was made, so setting the angle back to 0 and the
  scale back to 1 puts the scene back exactly as it was picked -- captured lengths
  included, because once a box has been scaled it keeps re-reading them, on the way
  back as well as out.
- **A turn or a scale settles any drift the members were carrying.** A joint that was
  authored unsatisfied comes out of a transform satisfied. A selection box is an
  authoring gesture, and the pose it leaves is the pose it authors.

A coupling with one foot outside the selection is not a member: it is neither carried
nor re-read, and after a transform it reads as violated -- the honest report that the
selection cut through a machine rather than around one. It also keeps that pose from
becoming the reset baseline, exactly as any other unsatisfied edit does (§16.1).

### Verified

`tools/select-check.js` covers the membership rule (including the grounding rod that
comes along and the rod that does not), the lasso's centre test, a rigid transform
that turns every body by the box's delta and leaves every member exactly as stressed
as it was, a scale that spreads without resizing and leaves the machine assembled, the
frame's reversibility, and the copy/paste round trip -- congruent, freshly numbered,
original untouched, and loadable on its own as a scene.

---

## S.10 Expressions: exact geometry at the moment of authoring

A number typed into a panel field, or written into a scene file, is read as
arithmetic: `2*pi/3`, `0.4*sqrt(2)`, `bg.P/2`, `b3.x+b3.r`. The parser is
`js/expr.js` (§19) -- a tokenizer, a recursive-descent parser and an evaluator,
about two hundred lines, no dependency -- and the names it can see are bound in
`js/scene.js` §17.8.

### The problem it solves, and the one it does not

Declaring geometry precisely is a different job from keeping it consistent, and the
bench already does the second one. A crank at exactly 30 degrees, a rod exactly
`sqrt(2)` long, a vessel charged to exactly half the ambient: writing those as
`0.5235987755982988`, `1.4142135623730951` and `50662.5` is not the same statement --
it is that statement rounded, and the rounding is invisible in the file afterwards.
An expression states the intent, and the *file* is where the intent was.

What it is emphatically not is a stored formula. **The number is what is kept**, at
the moment it is typed or read. Put `b3.x` into a body's x and it lands where body 3
is now, and stays there when body 3 moves. That boundary is not a limitation to
relax later; it is the same one the rest of the design turns on. A value that must
keep following another value as the scene changes is a *constraint* or an
*interaction* -- a first-class object the solver enforces, that carries a reaction
force you can read, that participates in islands and in the energy ledger, and that
you can see drawn on the canvas. A spreadsheet formula hidden inside a field would
be a second, invisible coupling mechanism with none of those properties, and the
project's whole claim is that couplings are visible objects.

So: expressions author the *initial* state. Constraints keep it.

### One vocabulary, and it is the ledger

There are two environments -- the live bench (what a panel field is typed against)
and the file being read (what that file's own text says) -- and deliberately one
vocabulary, taken from `SCENE_SCHEMA` itself:

```
pi tau e deg            constants; `deg` is radians per degree, so 30*deg
air.Rs air.gamma        the working gas's two numbers
g bg.P bg.T bathQ       the `sim` line's numeric keys, by their own names
b7.x b7.r b7.len ...    body 7's fields, by the names the format writes
sin cos tan asin acos atan atan2 sinh cosh tanh sqrt cbrt exp ln log log2
abs sign floor ceil round pow mod clamp min max hypot
```

A body's properties are exactly the numeric fields its ledger row lists -- a disk
has `x y r mass th vx vy w` because those are the fields the format gives a disk --
so the vocabulary cannot drift from the format, and a field added to the ledger is
a name the moment it exists. Ask for one that is not there and the error says what
the kind does have. `log` is base 10 and `ln` is natural; angles are radians.

### A file resolves its own names

A file's names resolve against **the file**, never against the bench it is landing
on. Two things follow. A scene is self-contained -- it means the same thing on any
bench, and loading it twice loads the same scene -- and it is order-free, exactly as
line order already was: a body may be placed relative to one defined further down,
because the reader now scans the whole file as text before evaluating any of it.
Resolution is lazy and memoized, so a value defined in terms of itself, directly or
around a ring, is a load error rather than a hang. A field the line omitted resolves
to its ledger default, including the computed ones -- `b3.mass` on a disk whose line
never wrote a mass is the density-1 value the constructor would give it. A vessel is
the one place this is not a plain field read: `b5.P` asks the resolved fill (§17.4
`resolveVesselFill`), so it answers even when the line that wrote it chose to say
`len`, `T` and `gasMass` instead.

The `sim` and `cam` lines are the exception, and evaluate against constants only:
what everything else reads as `bg.P` has to be one settled number before any body is
evaluated, so the ambient cannot be written in terms of a vessel that is filled from
the ambient.

### What the reader had to become

The reader was two passes (parse everything, then commit) and is now three, because
an expression may name a body defined further down:

```
scanScene   the file as TEXT -- unknown kinds and keys, a key twice, a flag with a
            value, duplicate ids, the `--` between two ends
evalScene   text -> numbers: expressions, endpoints, points, and the check that
            every body anything names exists
commitScene clear the bench and build -- only once both have returned
```

The guarantee that made the old split worth having is unchanged and now covers more:
a bad file, whether the fault is a misspelled key or a misspelled name inside an
expression, leaves the current bench exactly as it was.

Three grammar consequences fell out, all in §17.2. A coordinate pair used to be two
runs of characters that were neither comma nor paren; since either half may now
bring its own parentheses, `bg(b3.x,b3.y+0.5)` splits on the commas at depth zero
instead. A point's options are slash-separated and slash is also division, so
`pt=3/s=1/2` splits only at the slashes followed by one of the format's own option
words -- `s=1/2` is one option, a station of a half. And a body id stays a literal
everywhere it appears: an id is a name, the one thing other lines point at, and only
the offsets around it are arithmetic.

### It is a grammar, not `eval`

Scene text arrives from files, from the clipboard and from the widget stash. The
grammar in §19 is the whole of what such text may say: no property access beyond the
names an environment offers, no calls beyond the function table, no assignment, no
strings, and no way through a name to anything the host holds -- `globalThis`,
`constructor`, `Math.PI` and `eval("1")` are each just an unknown name or a syntax
error. `tools/expr-check.js` asserts that directly.

### Panel fields

The inspector's numeric inputs (`js/inspector.js` §14.0) are `type="text"`, not
`type="number"`: a number input hands back an empty string for anything it cannot
read as a literal, so `2*pi` would be gone before the panel saw it. The arrow keys
are wired back on by hand and step the value the field currently evaluates to, which
is why stepping a field holding `2*pi` leaves a number behind. Text a field cannot
use -- a malformed expression, or a value outside what that field accepts -- marks
the field, says why on hover, and is *not* committed: the panel keeps what you typed
so you can fix it, rather than silently reverting to the old value.

### Verified

`tools/expr-check.js`: the language (precedence, associativity, the function and
constant tables, and that every malformed or non-finite expression is an error
rather than a quiet NaN); a file (an expression wherever a number is -- fields,
endpoint offsets, a vec2, a point's station -- names resolving backwards, forwards
and to ledger defaults, and a vessel's fill answering whichever two of its four the
line gave); that expressions are not stored (the export is digits, and a value typed
against another body does not follow it); the refusals, each leaving the bench that
was loaded before it exactly as it was; and the panel field, including the arrow-key
step and the marking of text it cannot use.
