# Vertices, Lines and Strands -- Design Note

A plan for the next version of the scene model and the editor built on it. The
physics engine does not change: the solver (§07), the substep (§08), the position
projection (§09), the islands, the energy ledger and the expression language are all
untouched. What changes is **the scene, and the pass that turns a scene into rows** --
`rowsFor` (§06.5) and the objects it dispatches on.

This note uses `§X.n` for its own sections (`V` is the vessel note's) and the
codebase's own `§NN` tokens for code, per `AGENT.md`.

> **Status:** design only. Nothing in this note is built. `§X.12` phases it.

---

## X.1 The complaint, stated precisely

The bench has ten constraint kinds. Look at what four of them actually hold and the
duplication is embarrassing:

| Kind | Its two ends | Third-and-later ends | The rotation lock is called |
|---|---|---|---|
| `pin` | coincide | coincide | -- |
| `rod` | distance held | on the line, at a station | `weldA` / `weldB` / `pt.lock` |
| `slot` | (nothing) | on the line, sliding | `prismaticA` / `prismaticB` / `pt.lock` |
| `rack` | line origin + aim | station, or a meshing pinion | `weldA` / `weldB` / `pt.lock` |
| `spring` | distance, softly | -- | -- |

Every one of those rotation locks is the same row -- `pointAngleLockRow` (§06.1),
"hold this frame's angle to the line's heading" -- under four names. Every "on the
line at a station" is `linePointRows` (§06.2c). A rod and a slot differ by exactly
one bit per end (does the end hold its station, or slide?), and a spring differs from
a rod by exactly one number (is the axial relation a row, or a force?). A rack is a
rod whose second end slides, plus a list of disks meshing with it.

Three further problems follow from the same root, which is that **the shared point has
no name**:

- **A pivot is stored once per arm.** A pin between two bodies stores two offsets
  that are supposed to name the same world point; a three-armed hinge stores three.
  Nothing in the model says they are one thing, so nothing can list them, label them,
  or move them together, and the canvas draws a stack of endpoints where a person
  sees one joint.
- **Attachment is a property of the tool that made it.** Whether a body riding a bar
  slides is decided when you place it and is edited from the *joint's* panel, in the
  joint's vocabulary. There is no way to ask "what is body 3 attached to?"
- **`DEVELOPMENT.md` §8's feature-first model has nothing to be a feature.** "Drop a
  named geometric handle onto a body, then pick a relation" needs the handle to be an
  object. It is not one.

The fix is one object: **the vertex**.

---

## X.2 The vertex

> A **vertex** is a named point. It carries a label, and a list of the bodies it
> touches -- its **incidences**. It has no coordinates of its own.

That last clause is the whole reason the physics is untouched. A vertex is not a
particle: it has no mass, contributes no columns, and is never solved for. It is a
*name for a coincidence*, and it compiles to the coincidence rows the `pin` already
built. What it buys is that the coincidence is now a thing you can select, label,
list, drag, and hang a body list off.

**Bodies, for the purposes of a vertex's list, are: disks, rectangles, vessels,
lines, and the background.** The background is a body with a fixed frame -- position
`(0,0)`, angle `0`, no coordinates -- which is exactly what `epFrame`'s `id==null`
branch (§06.1) already makes it. Naming it a body is the last step of a move the code
has already made twice.

---

## X.3 The incidence

One row of a vertex's list. It says where the vertex sits in that body's frame, and
what -- if anything -- holds it there.

```
{ body,          a body id, or the background
  off,           where the vertex is, in that body's frame
  join,          held to the vertex, or merely marking a spot on it
  weld,          held to the vertex's FRAME as well as its point
  slide }        line bodies only: may travel along the line
```

**`off`** is the existing `{id, off}` offset, unchanged, including the material
`(lat, f)` labelling a vessel uses (§05.2c). That is what "where the vertex lies in
its body frame" means for a vessel, and it is why a vertex on a cap restrains the
breathing fully and one on the mid-wall not at all.

**`join` off** is a *feature point*: a named spot on a body that nothing holds the
vertex to. It is authored, not derived -- it means "this material place on that
body", and the vertex is simply not tied to it yet. Ticking `join` on recaptures the
offset from the live geometry first, so the tick never snaps anything -- the same
discipline `captureRestAngle` and `setConPointLock` already follow (§06.1, §06.2c).

**`weld`** is the one that repays the most. Today it appears as `weldA`, `weldB`,
`prismaticA`, `prismaticB` and `pt.lock`; here it is one flag with one meaning:

> A vertex has an **angular frame** as well as a point. `join` pins a body's material
> point to the vertex's point. `weld` additionally pins that body's angle to the
> vertex's frame.

Bodies joined but not welded at a vertex turn freely about it -- that is a hinge.
Bodies welded at a vertex turn together -- that is a rigid connection. The background
welded at a vertex ties everything welded there to the world frame, which is exactly
what `restAngA` on a background rod end means today.

A consequence worth stating plainly because it will surprise: **a single weld at a
vertex holds nothing.** There is nothing to hold it to. The inspector should say so
rather than pretend. `§X.5` shows why this is not a wart but the correct reading, and
`§X.9` gives the default that makes the obvious gesture do the obvious thing.

**Where a vertex is.** Its **primary** incidence gives its world position, and the
primary is chosen in a fixed order: the background if it is joined there, else the
first joined real body (disk, rect, vessel), else the first joined line, else the
first incidence at all. Background first because the background does not move; real
bodies before lines because a line's own frame is derived from vertices, and the
order is what keeps that derivation acyclic (`§X.5`).

Every vertex has a background incidence in its list, always, and it always reads
out the live world position. Ticking `join` on it nails the vertex to that world
point, which is a **ground pin** -- 2 rows, position held, rotation free. The bench
does not have that today: pinning a point to ground currently takes a rod with a
welded background end, which also removes the rotation. Getting it for free is a
small sign the factoring is right.

---

## X.4 The line

> A **line** is a body: a straight, massless, infinite bar. Its frame is not stored;
> it is **derived** from the vertices joined to it.

A line has a label, a set of joints (which are just the vertices whose list contains
it), an inverse elastic modulus, and a `posable` flag. It has no endpoints, no
length field, no rest angles, and no `pts` array. Everything that used to live on a
`rod`, `slot` or `rack` lives on the incidences instead.

**Order along the line is geometric, not stored.** Joints are ordered by their
**station** -- signed distance from the line's origin along its heading. That is
already the quantity `pt.s` captures (§06.2c); making it the ordering means the file
carries no arbitrary index, a joint dragged past another simply reorders, and
"consecutive" in the inspector means what it looks like it means.

**The frame.** Two joints define it:

- the **origin** `O` -- the first non-sliding joint in station order. It is the one
  material point of the bar that is located. (If every joint slides, the bar is
  undetermined; see below.)
- the **aim** `A` -- the joint furthest from `O` in station order. It gives the
  heading. The longest available baseline, which is strictly better conditioned than
  today's rod, whose frame is whichever two ends the tool happened to place first.

This is the rack's asymmetry (§06.2, "a pins the rack, b only aims it") promoted to
the general rule, and it is *forced*: a station is a distance from a material origin,
so an origin that slides would make every station meaningless.

**Rows.** With joints `J1..Jn` in station order:

| Joint | Rows |
|---|---|
| `O` | none |
| `A` | 1 station row, if `A` does not slide |
| any other `Jk` | 1 on-line row; + 1 station row if it does not slide |
| a meshing disk | 1 nonholonomic mesh row |

`A` gets no on-line row because it is a tautology -- the line is drawn *through* `A`.
That single omission is the whole difference between today's rod (`O` and `A` both
held: 1 distance row) and today's slot (`A` slides: 0 rows, "purely visual"), and the
degenerate case falls out with no branch: **a line all of whose joints slide has no
origin, no rows, and no defined pose -- it is inert, exactly as a two-pin slot is
today.**

The rows themselves are `linePointRows` (§06.2c) with `(O, A)` in place of `(a, b)`,
unchanged. The mesh row is the rack's pinion row (§06.5), unchanged, and it still
reads the rack's tangential speed off `O`'s columns -- which is correct precisely
because `O` is now *defined* to be non-sliding.

**Weld rows do not appear in this table**, and that is the point: a line's welds are
its incidences' welds, held by the vertices they sit at (`§X.5`). The four names for
one row collapse to zero names.

**The segment distances.** Consecutive non-sliding joints have a fixed distance
between them -- `station(k) - station(k-1)` -- and that is what the inspector lists
and what the file writes. Stations are relative, so the first non-sliding joint is
station 0 by definition and the segments give the rest. Editing a segment shifts
every station after it. A rod's `len` is the one-segment case of this.

**The line's own angle.** `phi`, the `O -> A` heading, unwrapped against `_phiRef`
exactly as `twoPointFrame` does today (§06.1) and for exactly the same reason.

---

## X.5 The one change to the translation into math

Everything above compiles through a single new idea, and it is small:

> **A frame is one of three things: a body, the background, or a line. All three
> answer the same two questions -- `velCols(dx, dy)` for the velocity columns of a
> point moving with the frame, and `angCols()` for the columns of its angle.**

`epFrame` (§06.1) already implements two of the three, and already routes a vessel's
fourth coordinate through the same closure so that no row above it needs vessel-
specific algebra. The change is to add the third:

```
body        velCols -> that body's own columns          angCols -> [[idx,0,0,1]]
background  velCols -> []                               angCols -> []
line        velCols -> a station-weighted blend of      angCols -> n.(v_O - v_A)/L,
            O's and A's columns                                    O's and A's columns
```

A line's `angCols` is not new arithmetic: it is the `k*nx, k*ny` combination
`pointAngleLockRow` already builds. A line's `velCols` is the bar-rotation term
`linePointRows` already builds. Both are being *named* rather than invented, and once
named, the rest of the engine talks to a line through the same interface it talks to
a disk through.

**A line is a derived body: it has a frame but not its own coordinates.** No column,
no mass, no island membership of its own, nothing in the snapshot, nothing for Reset
to restore. The Schur assembly, `invMdiag`, the projection, the energy ledger: all
untouched.

With that, the whole row catalogue is two short rules.

**A vertex, with incidences `I1..Im`:**

- let `P` be the primary joined incidence, `W` the first welded one;
- each *joined* incidence except `P`: **2 rows** -- its point coincides with `P`'s;
- each *welded* incidence except `W`: **1 row** -- its angle equals `W`'s angle plus a
  rest angle captured when the weld went on.

**A line**: the table in `§X.4`.

That is all of `pin`, `rod`, `slot`, `rack` and `spring`, and it is the reason a
single weld holds nothing: with `m` welded incidences at a vertex there are `m-1`
rows, because welding is a relation between frames and one frame has nothing to
relate to. It is the same arithmetic that gives a two-pin slot no rows.

**Acyclicity.** A line's frame depends on its origin and aim vertices, which depend
on their primary incidences, which may in principle be another line. The primary
ordering in `§X.3` (background, then real body, then line) makes a cycle impossible
unless a line's frame rests on a line whose frame rests on it. That is a graph check
run where the expression evaluator already runs one (§17.4, lazy + memoized with
cycle detection), reported as a load-time or edit-time error rather than a hang.

**Reactions get more honest, not less.** `reactionOf` (§09.3) currently walks row
order by hand, per kind, counting `weldA?` and `weldB?` and `pt.lock?` to find an
index. That arithmetic is duplicated four times and is the most fragile code in the
file. Under the new model each row is tagged with its role and the joint it belongs
to -- `{role:'station', at:k}` -- and the readout looks its multiplier up instead of
counting to it. A line then reports the axial force *in each segment*, which is more
than a rod reports today.

---

## X.6 Compliance: where the spring goes

> A line carries an **inverse elastic modulus**, `soft`. Zero -- the default -- means
> the axial relation is a constraint. Above zero it is a force.

With `soft > 0`, every **station** row is dropped and replaced by an axial force
element between consecutive non-sliding joints, `F = (L - L0) / (soft * L0)`, in the
solve's applied-force pass (§08.1) beside the springs, with its strain energy in the
ledger (§12) so the per-island conservation target stays honest. The **on-line** rows
stay: a soft line still is a line, and its riders still ride it.

Two consequences fall out, and both are the good kind:

- **A two-joint soft line is exactly today's spring.** Two joints means `O` and `A`
  and nothing else; `A`'s on-line row is the tautology, its station row has gone
  soft, and what remains is one axial force between two body-frame anchors. That is
  `makeSpringCon` (§06.6), field for field.
- **A three-joint soft line is something the bench cannot build today**: a sprung bar
  carrying a rider, or two bodies sprung apart along a rail they both sit on. Getting
  it for nothing is the usual sign.

`soft` is `1/(EA)` -- per unit length, so a long line is softer at the same modulus,
which is what calling it a modulus commits us to. `§X.13` lists the alternative.

The draggable rest-length handle (`springRestHandlePos`, §06.6) generalizes to one
handle per segment.

---

## X.7 The strand: cables, belts and rotational springs

This is the half the brief calls less clear, and it should be built last, but it is
worth writing down now because it constrains nothing above and it explains one
otherwise-arbitrary decision in `§X.4`.

The line's dual is a **strand**: a path of belting through an ordered list of nodes,
holding a length budget.

|  | line | strand |
|---|---|---|
| the path | straight, infinite | a polyline over wheel arcs, open or closed |
| a node | a vertex, sliding or at a station | a **wheel** the belting wraps, or an **eyelet** it passes through |
| a node that grips | non-sliding: holds its station | tied: grips the belting at one material point |
| a node that does not | sliding: rides the line | untied: routes the path and nothing else |
| welding | the body's angle follows the line's heading | the body's angle follows the belting's local direction |
| axial | rigid (a rod) or compliant (a spring) | inextensible (a belt, a cable) or compliant |
| one-sided | -- | tension-only (a cable) |
| node order | derived, by station | **stored** -- the order is the path |

Under that reading:

- a **belt** is a closed strand over two wheels, `soft = 0`;
- a **cable** is an open strand from an eyelet to a wheel, `soft = 0`, tension-only;
- a **rotational spring** is a strand over two wheels with `soft > 0`. A torsional
  rate `k_th = r^2 / soft` follows, and so does the reason `rotSpringVisualMode`
  (§06.6) already picks between drawing a *belt* and drawing a *spiral*: the code
  guessed the unification before the model had it.

This is not speculative. It was built once, on this branch's history, and reverted
whole: commit `03c163b`, "Redesign the belt as a closed loop of wheels and eyelets"
-- nodes in `con.pts`, one material-balance row per segment, a per-node `wrap` side
replacing the single `crossed` flag, compliance turning the rows into a tension force
element, and `tools/belt-check.js` checking the path against the textbook two-pulley
closed form. `CABLE.md` §C.2 and §C.5 already argue the same collapse from the other
end (a bare tether is a spool of radius 0; a point-to-point cable is a rod). The work
to do here is to rebuild that on the vertex model -- nodes are vertices -- rather than
to invent it.

The decision it explains: a line's joint order is derived from geometry and a
strand's is stored, because on a line "which side of you is it" is a fact about
where things are, and on a loop it is the object itself.

---

## X.8 Labels and identity

> Every vertex and every body carries a **label**. It is the identifier: what the
> file references, what the canvas draws, and what an expression may name.

- Vertices default to `A`, `B`, ... `Z`, `AA`, ...; bodies default to `1`, `2`, ...
  The background's label is `bg` and is not editable.
- Labels are editable in the inspector, unique across one namespace (vertices and
  bodies together, so an expression name is unambiguous), and must match
  `[A-Za-z_][A-Za-z0-9_]*` or be a bare integer. A rename rewrites nothing: the label
  *is* the reference.
- Drawn to the **top right** of the thing they name. A line's label is drawn where
  the line leaves the viewport, top-right-most crossing, which is also the only place
  an infinite object has room for one.
- Paste renumbers (`§17.7 remapItem`) exactly as it renumbers ids today, and for the
  same reason: a widget's labels belong to the bench it was cut from.

Labels join the expression vocabulary automatically, because the vocabulary *is* the
ledger (`SCENE.md` §S.10): a vertex row exposing `x` and `y` makes `A.x` a name the
moment the row exists. `line 5 -- 2` becomes readable as `A -- B`, which is most of
why this is worth doing at all.

---

## X.9 Freezing, re-derived

`refreshFrozen` (§06.2b) recognizes two structural patterns today. Both restate
cleanly, and the restatement is strictly more general -- it recognizes arrangements
the current rules miss, without going anywhere near the configuration-dependent rank
computation `SCENE.md` §S.8 rules out. The rules stay structural: they read only what
is attached, never where anything is.

- a **vertex is fixed** if it is joined to the background, or joined to a grounded
  body;
- a **line's frame is fixed** if two of its joints are fixed, or it is welded at a
  fixed vertex where the background (or a grounded body) is also welded;
- a **body is grounded** if two of its joined vertices are fixed and distinct in its
  own frame, or one is fixed and the body is welded there alongside a fixed line;
- a **vessel's length is locked** if two of its joined vertices sit at different
  material fractions and are both held -- a strut between two of its own planes,
  unchanged in substance.

Iterate to a fixed point as today (grounding is transitive), compile away the rows
that did the freezing as today, and release everything a `posable` line touches
during a pose drag as today. What is new is that "two ground pins hold a body still"
is now recognized, which is both obvious and currently missed.

`posable` ports directly: while a body a posable line touches is dragged, every
joint on that line becomes a sliding, unwelded rider, and the line re-reads its
stations, segments and rest angles from the pose the drag left (§06.2d
`recapturePosable`).

---

## X.10 The scene format, version 4

Two new kinds, five retired. The grammar is untouched -- one object per line, `#`
comments, `key=value`, every number an expression, the repeatable-key mechanism the
`pt=` field already established (§17.2).

```
vertex <label> on=<incidence> on=<incidence> ...
line   <label> [soft=<num>] [posable] [seg=<num> ...] [mesh=<body> ...]
```

An incidence token is one word, on the `pt=` pattern -- an endpoint, then
slash-separated options:

```
on=1@(0.2,-0.1)/join/weld       a body-frame point, held, and rigid
on=5/join/slide                 on line 5, free to travel along it
on=bg(0,4.4)/join               a ground pin
on=3@(0,0.5)                    a feature point: marked, not held
on=5/join/weld/restAng=1.5708   a weld, with its captured rest angle
```

`seg=` is repeatable and gives the distances between consecutive non-sliding joints,
in station order -- the same list the inspector shows. `mesh=` is repeatable and
names the disks meshing with the line, replacing `pt=<body>/pinion`.

Classification, in `SCENE.md` §S.3's terms: an incidence's `off`, `join`, `weld` and
`slide` are **authored**; `restAng` and `seg` are **captured** (`always:true`,
written every time -- they are read off the geometry at creation and the pose no
longer implies them); a line's frame, its joint order, its stations, every world
position and every rest length derived from a segment are **derived** and never
written.

Retired: `pin`, `rod`, `slot`, `rack`, `spring`. Version 4 must move rather than
extend, because `rod A -- B len=2.6` still parses under a reader that has only lines
and would mean something else. `tools/scene-convert.js` is a one-shot v3 -> v4
translator, used once to regenerate the bundled examples and then kept for anyone
holding an old file. The eleven examples are regenerated through it and re-canonicalized
(`tools/scene-roundtrip.js --canon`), and the migration is verified the way the last
two were: the world the converter produces must agree with the world the old reader
produced, field for field, and over three seconds of real substeps.

The pendulum, as version 4 writes it:

```
scene 4
# A disk on a rigid bar, swinging from a fixed point. Nothing is welded, so the bar
# hinges at both ends -- which is now the default rather than something to untick.
sim gravity=on
cam x=0 y=2.6 scale=64

# bodies
disk 1 x=2.6 y=4.4 r=0.38
line 2

# vertices
vertex A on=bg(0,4.4)/join on=2/join/weld
vertex B on=1/join on=2/join/weld
```

and the slider-crank's rail, which the version-3 file has to place ten metres out to
dodge a singularity (`js/examples.js`, `crank:`), stops needing the trick -- two
background vertices fix the rail's heading outright, so nothing is holding a raw
`atan2` any more:

```
line 6                                  # the rail
vertex D on=bg(-4,2.4)/join on=6/join/weld
vertex E on=bg(4,2.4)/join  on=6/join/weld
vertex C on=2/join/weld     on=6/join/slide/weld    # the piston, riding it
```

---

## X.11 The editor

**The rail loses four tools and gains two.** Out: pin, rod, slot, rack, spring. In:

- **vertex (`v`)** -- one tap places a vertex. On a body, it takes an incidence on
  that body, joined. On empty space, it takes a background incidence, joined -- a
  ground anchor, matching what a rod end clicked in empty space does today. On an
  existing vertex, it selects it.
- **line (`l`)**, with a **new / extend** toggle in the header, the way the transport
  controls already carry state:
  - **new** -- tap vertices in turn. The first two become the bar; each further tap
    joins that vertex to the line as a **sliding** rider and runs
    `projectPositions` to bring it onto the line, which is the "quick solve" the
    brief asks for and is code that already exists (§09.1). Tapping empty space
    places a vertex there first. Overlapping lines are ordinary -- three lines
    sharing four vertices is how you build rigid carriages on a shared rail.
  - **extend** -- tap an existing line, then tap vertices to join them to it, as
    sliding riders, same quick solve.
  - Every joint the tool creates is welded **on the line's own incidence** and not on
    the bodies'. This costs no rows (`§X.5`) and it is what makes the single tick a
    person expects do what they expect: ticking "welded" on *body 3* at a vertex
    where the line is already welded makes body 3 rigid with the bar immediately.
    Untick the line's own weld and everything at that vertex hinges again.

**The inspector is two lists, and they are one relation seen from both sides.**

- *A vertex selected*: its label, its world position, and one row per incidence --
  the body, the offset in that body's frame, `joined`, `welded`, and `slide` for line
  bodies. Plus a button to remove an incidence, and the honest note when fewer than
  two incidences are welded.
- *A body selected*: its label, its own properties as now, and one row per vertex on
  it -- the same relation, read the other way. Selecting a row selects that vertex.
- *A line selected*: its label, `soft`, `posable`, its joints in station order, and
  the distance between each consecutive non-sliding pair, editable. Plus its meshing
  disks.
- *The background selected*: reachable from the empty-bench panel, listing every
  vertex pinned to ground.

**Canvas.** Vertices draw as dots -- one dot per joint, not the stack of coincident
endpoints the pin draws today -- filled when joined, hollow when merely marking a
spot, with a square tick when welded. Labels top-right. Lines draw as they do now,
their label at the viewport edge. The existing handle/drag machinery (§13.3) moves
from "one handle per control point" to "one handle per vertex", which is fewer
handles doing more.

**Selection groups and widgets** (§18) need one restatement: the membership rule is
still "every body an element names must be in the selection", with a vertex coming
along when every body it is *joined* to is in the selection, and a line coming along
when all its joints do. A background incidence does not disqualify anything and
travels with the box, exactly as `SCENE.md` §S.9 already says. What a scaled box
re-reads gains two entries -- a line's segments and its joints' stations -- and loses
one, since rest angles now live on incidences.

---

## X.12 Phasing

Each phase leaves the bench working, the examples loading and the validators green.

**Phase 0 -- the frame seam.** Generalize `epFrame` (§06.1) into `frameOf`, with the
three cases of `§X.5`, and tag every row with its role and joint index so
`reactionOf` (§09.3) looks up instead of counting. No new objects, no format change,
no behaviour change. `tools/scene-roundtrip.js` and `tools/rack-check.js` must pass
untouched -- that is the phase's whole test.

**Phase 1 -- vertices.** The `vertices` array, incidences, labels on everything, the
vertex tool, the vertex and body panels, vertex rendering, the `vertex` line in the
format, and the compile of `§X.5`'s first rule. `pin` retires into it; the ground pin
arrives. Rod, slot, rack and spring stay as they are, but their endpoints become
vertex references, so there is exactly one place a shared point is stored. New
validator `tools/vertex-check.js`: the coincidence rows match the pin's exactly, the
primary ordering is acyclic, a weld count below two costs no rows, and a labelled
bench round-trips.

**Phase 2 -- lines.** The `line` body, the derived frame, the row table of `§X.4`,
compliance, the line tool with its two modes, the line panel, format version 4, the
converter, the regenerated examples, and the retirement of rod, slot, rack and
spring. `refreshFrozen` restated per `§X.9`. New validator `tools/line-check.js`;
`tools/multipoint-check.js`, `tools/posable-check.js`, `tools/rack-check.js` and
`tools/select-check.js` all ported. This is the large phase, and it is the one that
pays.

**Phase 3 -- strands.** Rebuild `03c163b` on vertices: a `strand` body, wheels and
eyelets as nodes, tied and untied, compliance, tension-only. Belt, cable and
rotational spring retire into it; format version 5; `tools/belt-check.js` restored
and `CABLE.md` §C.5's two-spool case falls out.

**Phase 4 -- optional, and genuinely speculative.** A rolling incidence: a vertex
that rolls between two frames rather than being joined to them. If it works, the
pinion, the CVT and the knife edge are three configurations of one incidence mode and
the `mesh=` list of `§X.10` disappears. If it does not, `mesh=` stays and nothing is
lost. Do not let phase 2's design bend to accommodate this.

---

## X.13 Decisions still open

1. **What a fresh line's joints default to.** The brief reads two ways: "each new
   selection constrains the new vertex to fall on the line" suggests every joint
   slides, and "for each pair of consecutive non-sliding joints the inspector lists
   the distance" suggests some do not. `§X.11` assumes **the first two joints do not
   slide and every later one does**, so two taps give a rod and three give a rod with
   a rider -- which is also the only reading under which "a quick solve makes them
   colinear" has anything to do (the first two are trivially colinear). Worth
   confirming, because it is the most-used gesture in the editor.
2. **Whether `soft` is a modulus or a compliance.** `§X.6` takes it as `1/(EA)`, so a
   long segment is softer at the same value. The alternative -- a plain `1/k` in m/N
   -- makes a segment's rate independent of its length, which is less physical but
   means that editing a rod's length does not change its stiffness.
3. **One namespace for labels, or two.** `§X.8` puts vertices and bodies in one, so
   expressions are unambiguous. Two namespaces would let a vertex be `A` and a body
   be `A`, at the cost of qualifying every expression name.
4. **How much of the strand to specify now.** `§X.7` sketches it and `03c163b`
   implements most of it, but the rotational-spring-as-compliant-strand claim in
   particular deserves a worked check (that `k_th = r^2/soft` reproduces the current
   element's energy and rate) before phase 3 commits to it.
5. **Whether a line should be finite.** Everything above treats a line as infinite,
   like today's slot and rack, and draws it to the viewport edge. A rod is finite. If
   lines should carry an extent, it is a pair of authored end stations -- which is
   also the natural home for the hard end-stops `DEVELOPMENT.md` §4.3 wants.

---

## X.14 What this does not touch

The solver (§07), the substep and its islands (§08), the position projection (§09),
the energy ledger (§12), the expression language (§19), the vessel and its gas
(§05.2d, `VESSEL.md`), the heat and mass interactions (§08.0b), the knife edge and
the CVT, and the scene file's grammar, its strictness, its three-pass reader and its
round-trip validator (`SCENE.md` §S.2, §S.5, §S.10). Phases 1 and 2 add rows of the
same shape the solver already takes, through closures it already calls, on a format
whose ledger already has a place for them.

That is the claim to check this plan against: if a phase needs a change under
`js/solver.js`, `js/physics.js` or `js/projection.js` beyond the role tags of phase
0, something in it is wrong.
