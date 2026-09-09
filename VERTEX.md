# Vertices, Lines and Strands -- Design Note

A plan for the next version of the scene model and the editor built on it. The
physics engine does not change: the solver (§07), the substep (§08), the position
projection (§09), the islands, the energy ledger and the expression language are all
untouched. What changes is **the scene, and the pass that turns a scene into rows** --
`rowsFor` (§06.5) and the objects it dispatches on.

This note uses `§X.n` for its own sections (`V` is the vessel note's) and the
codebase's own `§NN` tokens for code, per `AGENT.md`.

> **Status:** phases 0-2 are built -- the frame seam, the vertex, and the line. The
> strand is not. `§X.12` phases it and says where each phase stands.

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

> A **vertex** is a named point. It carries a label, a place, and a list of the bodies
> it touches -- its **incidences**. It has no *coordinates* -- no column of its own in
> the solve -- but it always has somewhere it is.

The first half of that is the whole reason the physics is untouched. A vertex is not a
particle: it has no mass, contributes no columns, and is never solved for. It is a
*name for a coincidence*, and it compiles to the coincidence rows the `pin` already
built. What it buys is that the coincidence is now a thing you can select, label,
list, drag, and hang a body list off.

The second half is what makes it an object rather than a body's property. Where
something places it -- a body it is joined to, the background, a line carrying it --
that relation says where it is; where nothing does, its own background mark does
(`§X.11`, "Where a vertex is"). So a vertex has a position under every arrangement,
including none, and it **outlives every body it ever touched**.

> This note originally said a vertex "has no coordinates of its own" flat, and phase 1
> was built on the strong reading: a vertex borrowed its position from a body and was
> deleted when the last body it could borrow from went. That is the body-owns-the-point
> model this whole design exists to be rid of, arriving through the back door -- and it
> showed. Deleting one disk took the vertex at its centre *and* the line between two of
> them, because each was resting its existence on the last. Having no column in the
> solve is one thing; having nowhere to be is another, and only the first was ever
> wanted. `§X.15` is the rule that replaced it.

> **As built (`js/constraints.js` §06.2e).** It lives in the `constraints` array,
> with `type:'vertex'`, because that array is the list of couplings and a vertex is
> one. Nothing else had to move: the island pass, the row assembly, the position
> projection, Reset's snapshot, transport's scratch clearing and the selection's
> membership rule all read a vertex through `conEndpoints` and `rowsFor` like every
> other coupling.

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

The two directions are not symmetric, and the asymmetry is the whole of what "not
held" means. Move the **body** and the spot is material: it goes with the body and
leaves the vertex standing, which is why an unjoined incidence never locates the
point. Move the **point** and every incidence re-reads its offset, this one included
-- the spot it names is *where the point is on that body*, and the point is now
somewhere else. A move that carries the point out of the body's extent ends the
incidence outright: there is nothing left for it to say, neither list will show it
(`§X.11`), and leaving it stored would be a coordinate in the file that no panel can
reach. A **joined** incidence is never dropped that way -- it is what holds the
vertex, and holding it from beyond its own outline is what a rod end at arm's length
always was.

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
vertex holds nothing.** There is nothing to hold it to. The inspector says so rather
than pretending. `§X.5` shows why this is not a wart but the correct reading, and
`§X.11` gives the line-tool default that will make the obvious gesture do the
obvious thing once lines exist.

A rest angle is captured as that body's **own absolute angle** at the moment its weld
went on, not as an offset from another incidence. So every capture is independent,
the row between two welded incidences is `(th_i - rest_i) - (th_j - rest_j)`, which
of them happens to be the reference cannot matter, and -- unlike a rod's weld, which
measures against an `atan2` -- there is no branch cut to unwrap.

**Where a vertex is.** What **grounds** it is its **primary** incidence, chosen in a
fixed order: the background if it is joined there, else the first joined real body
(disk, rect, vessel). Background first because the background does not move; a line
is never one of these at all, because a line's own frame is derived from vertices and
the exclusion is what keeps that derivation acyclic (`§X.5`).

A line may still **carry** a vertex that nothing grounds, and where neither does, the
vertex's own **background mark** says where it is. `§X.11` gives the three in order
and why they are one question with one answer. The mark is the bottom of that chain,
and it is what makes a vertex an object: it is planted, at the place the vertex was
already standing, whenever the relation that had been placing it goes away, so
letting go never moves anything and nothing is ever left with nowhere to be.

Joining a vertex to the background is a **ground pin**: 2 rows, position held,
rotation free. The bench could not say that before -- pinning a point to ground took
a rod with a welded background end, which removes the rotation along with the
position. Getting it for free is a small sign the factoring is right, and so is the
other thing that arrived with it: a body merely MARKED at a vertex, named at a
material spot without being held there, which is `DEVELOPMENT.md` §8's "feature" with
nothing else added.

---

## X.4 The line

> A **line** is a body: a straight, massless bar. Its frame is not stored; it is
> **derived** from the vertices joined to it.

A line has a label, a set of joints (which are just the vertices whose list contains
it), a compliance, and a `posable` flag. It has no endpoints, no
length field, no rest angles, and no `pts` array. Everything that used to live on a
`rod`, `slot` or `rack` lives on the incidences instead.

**Order along the line is geometric, not stored.** Joints are ordered by where they
project onto the line's own heading. That needs no material origin, so it is defined
for a line that has none; where there is one, it is the station `pt.s` already
captures (§06.2c). Ordering geometrically means the file carries no arbitrary index,
a joint dragged past another simply reorders, and "consecutive" in the inspector
means what it looks like it means.

**Two things are derived from the joints, and they are not the same thing.** Every
joined joint lies *on* the line -- that is what joining to a line means -- and
`slide` says only whether it additionally holds a **station**, a material position
along the bar. So:

- the line's **placement** -- where it is and which way it points -- comes from the
  two joints furthest apart, `P` and `Q`, sliding or not. The longest available
  baseline, which is better conditioned than today's rod, whose frame is whichever
  two ends the tool happened to place first. (Below a tolerance apart, the next
  furthest pair; a line with fewer than two placed joints has no direction, and no
  rows, which is not the same as no existence -- `§X.15`.)
- the line's **material origin** `O` -- the first non-sliding joint in station
  order -- exists only when some joint does not slide, and is what stations are
  measured from. A station is a distance from a material point, so an origin that
  slid would make every station meaningless. This is the rack's asymmetry
  (§06.2, "a pins the rack, b only aims it") promoted to the general rule.

**Rows.** With joints `J1..Jn` in station order:

| Joint | Rows |
|---|---|
| `P`, `Q` -- the two that place the line | none: they *are* the line |
| any other joint | 1 on-line row |
| each non-sliding joint except `O` | 1 station row -- its distance from `O` is held |
| a meshing disk | 1 nonholonomic mesh row |

**Every joint slides by default.** A fresh two-joint line is `P` and `Q` and nothing
else: **zero rows, a drawn guide** -- exactly what a two-pin slot is today. A third
tap adds one on-line row, and that is a rail with a rider. Untick `slide` on two
joints and the distance between them is held, and that is a rod. The three cases
are one object with no branch between them, and the common one -- laying out a rail
and dropping things onto it -- is the one that needs no ticking and asks the solver
for nothing.

**Extent is derived, and there are no end stops.** All joints non-sliding: the line
is a **bar**, drawn finite between its extreme joints. Any joint slides: it is a
**rail**, drawn infinite to the viewport edge. No field says which. Nothing clamps
travel either -- a slider may always pass a non-slider, or a slider that happens to
be stationary.

The rows themselves are `linePointRows` (§06.2c) with `(O, P..Q)` in place of
`(a, b)`, unchanged. The mesh row is the rack's pinion row (§06.5), unchanged, and it
still reads the rack's tangential speed off the material origin -- which is correct
precisely because `O` is now *defined* to be non-sliding.

**Weld rows do not appear in this table**, and that is the point: a line's welds are
its incidences' welds, held by the vertices they sit at (`§X.5`). The four names for
one row collapse to zero names.

**The segment distances.** Consecutive non-sliding joints have a fixed distance
between them -- `station(k) - station(k-1)` -- and that is what the inspector lists.
Stations are relative, so `O` is station 0 by definition and the segments give the
rest. Editing a segment shifts every station after it. A rod's `len` is the
one-segment case of this. The *file* writes the stations rather than the segments
(`§X.10`): the number belongs to the joint, and a segment is a difference of two.

**The line's own angle.** `phi`, the `P -> Q` heading, unwrapped against `_phiRef`
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

> A line carries a **compliance**, `soft` -- the inverse of a stiffness. Zero -- the
> default -- means the axial relation is a constraint. Above zero it is a force.

With `soft > 0`, every **station** row is dropped and replaced by an axial force
element between consecutive non-sliding joints, `F = (L - L0) / soft`, in the
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

`soft` is a plain **compliance**, `1/k` in m/N -- not a true modulus. A modulus
(`1/(EA)`, so a long segment is softer at the same value) is the more physical
reading, and it was rejected because a value you can think about beats one you have
to derive: `soft = 0.01` is a segment that gives a centimetre per newton, whatever
it is a segment of, and editing a rod's length does not quietly change its
stiffness.

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

An earlier attempt at the belt half of this exists in the branch's history and was
reverted whole. **It is not a foundation and must not be read as one** -- neither its
diff nor its reasoning should inform this design, because a flawed derivation that
gets copied forward is worse than no derivation at all. The strand is to be worked
out from first principles when phase 3 comes up, against `CABLE.md` §C.2 and §C.5,
which already argue the same collapse from the other end (a bare tether is a spool of
radius 0; a point-to-point cable is a rod) and which were written independently of
that attempt.

That is also why phase 3 is last and why nothing in phases 0-2 depends on it.

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

**Still not landed.** Freezing is an OPTIMIZATION, not the physics (`SCENE.md` §S.8):
an arrangement the rules do not recognize is held exactly as it always was, at the
cost of the rows and the island split. Phase 1 left this alone deliberately, and
phase 2 -- which was meant to bring it, the line being the vocabulary it is written
in -- **ported** the two old rod rules into line terms instead (`lineGrounds`,
`lineLocksLength`). So the restatement above is outstanding work rather than a
decision against it, and a body held by two ground-pinned vertices still works and is
still not compiled away.

`posable` ports directly: while a body a posable line touches is dragged, every
joint on that line becomes a sliding, unwelded rider, and the line re-reads its
stations and rest angles from the pose the drag left (§06.2d `recapturePosable`).

---

## X.10 The scene format, versions 4 and 5

Version 4 landed the first half of this with phase 1: the `vertex` line, the `on=`
incidence token, labels, and the retirement of `pin`. What follows is the second
half, which is version 5.

```
vertex <label> on=<incidence> on=<incidence> ...     [version 4, built]
line   <label> [soft=<num>] [posable] [mesh=<body> ...]
```

The grammar is untouched -- one object per line, `#` comments, `key=value`, every
number an expression, the repeatable-key mechanism `pt=` established and `on=` now
shares (§17.2).

An incidence token is one word, on the `pt=` pattern -- an endpoint, then
slash-separated options:

```
on=1@(0.2,-0.1)/join/weld       a body-frame point, held, and rigid
on=5/join                       on line 5, free to travel along it (the default)
on=5/join/fix                   ...held at its place along line 5 instead
on=bg(0,4.4)/join               a ground pin
on=3@(0,0.5)                    a feature point: marked, not held
on=5/join/weld/restAng=1.5708   a weld, with its captured rest angle
```

The word is `fix`, not `slide`, because sliding is the default (`§X.4`) and the
format writes what differs from the default. `mesh=` is repeatable and names the
disks meshing with the line, replacing `pt=<body>/pinion`.

> **As built, the distances live on the joints, not on the line.** This section
> planned a repeatable `seg=` on the `line` giving the distance between each
> consecutive pair of held joints. What shipped writes each held joint's own
> **station** instead, as `s=` inside its `on=` token, and a segment is the
> difference of two of them -- which is what the inspector shows and what a segment
> edit shifts. One number per joint rather than one per gap: the joint is where the
> number belongs, because a joint added or released changes what its own station is
> measured from and changes no one else's.

Classification, in `SCENE.md` §S.3's terms: an incidence's `off`, `join`, `weld` and
`fix` are **authored**; `restAng` and `s` are **captured** (`always:true`, written
every time -- they are read off the geometry at creation and the pose no longer
implies them); a line's frame, its joint order, its extent, its stations, every world
position and every rest length derived from a segment are **derived** and never
written.

Retired: `rod`, `slot`, `rack`, `spring` (`pin` went with version 4). Version 5 must
move rather than extend, because `rod A -- B len=2.6` still parses under a reader that
has only lines and would mean something else. `tools/scene-convert.js` is a one-shot
v4 -> v5 translator, used once to regenerate the bundled examples and then kept for
anyone holding an old file. The examples are regenerated through it and re-canonicalized
(`tools/scene-roundtrip.js --canon`), and the migration is verified the way the last
two were: the world the converter produces must agree with the world the old reader
produced, field for field, and over three seconds of real substeps.

The pendulum, as version 5 writes it. Both joints are `fix`, which is what makes the
line a bar rather than a rail; nothing is welded, so it hinges at both ends -- which
is now the default rather than something to untick:

```
scene 5
sim gravity=on
cam x=0 y=2.6 scale=64

# bodies
body 1 x=2.6 y=4.4 r=0.38

# lines
line 2

# vertices
vertex A on=bg(0,4.4)/join on=2/join/fix/s=0
vertex B on=1/join on=2/join/fix/s=-2.6
```

and the slider-crank's rail, which the version-3 file had to place ten metres out to
dodge a singularity (`js/examples.js`, `crank:`), stops needing the trick. Two
background vertices fix the rail's heading outright, so nothing is holding a raw
`atan2` any more -- and since every joint on it slides, not one of them writes an
option at all:

```
line 6                                  # the rail -- nothing is fixed to it
vertex D on=bg(-4,2.4)/join on=6/join
vertex E on=bg(4,2.4)/join  on=6/join
vertex C on=2/join/weld     on=6/join/weld    # the piston, riding it, angle held
```

> **Not taken.** The bundled `crank` still carries the version-3 arrangement, a
> single background vertex welded to the rail ten metres out, because the migration
> reproduced the old world field for field and nothing has rebuilt it since. The
> two-background-vertex form above is available and is the better scene; swapping it
> in is a change to the example, not to the model.

---

## X.11 The editor

**The rail loses four tools and gains two.** Out: pin, rod, slot, rack, spring. In:

- **vertex (`v`)** -- one tap places a **point**: a background incidence, unjoined,
  holding the world coordinates of the place tapped (the snap is still honoured, so a
  tap near a rim or a centre lands on it). Nothing else, whatever body it landed on.
  Which body that would be is a question about draw order, and a point must not take
  its frame from whatever happens to be on top -- nor be dragged around by a body it
  is not held to. Attaching is a tick in the panel, where every body the point is
  inside is already listed. A tap on an EXISTING vertex still reaches through to join
  a body, topmost first, which is the two-tap hinge by hand.
- **line (`l`)**, with a **new / extend** toggle in the header, the way the transport
  controls already carry state:
  - **new** -- tap vertices in turn. Every joint is created **sliding**, so the
    first two place the line and ask nothing of the solver, and each further tap
    runs `projectPositions` to bring that vertex onto the line -- the "quick solve",
    and code that already exists (§09.1). Only from the third tap is there anything
    to solve, which is also the only point at which a vertex might be somewhere the
    line is not: drop the bodies roughly where you want them, then say they are in
    a line. Tapping empty space places a vertex there first. Overlapping lines are
    ordinary -- three lines sharing four vertices is how you build rigid carriages
    on a shared rail.
  - **extend** -- tap an existing line, then tap vertices to join them to it, as
    sliding riders, same quick solve.
  - Every joint the tool creates is welded **on the line's own incidence** and not on
    the bodies'. This costs no rows (`§X.5`) and it is what makes the single tick a
    person expects do what they expect: ticking "welded" on *body 3* at a vertex
    where the line is already welded makes body 3 rigid with the bar immediately.
    Untick the line's own weld and everything at that vertex hinges again.

**The inspector is two lists, and they are one relation seen from both sides.** What
each list holds is the **extent** relation, not the incidence one: everything the
point is *inside*, and not just what has been joined to it. A vertex sitting in a
body is at a place on that body whether or not anything has said so yet, so the list
is a fact about the geometry rather than a record of the editing history -- which
means the body underneath is listed exactly like the one on top, the background is in
every list (its extent is the whole plane, which is what a body with a fixed frame
comes to), and `joined` is the only control such a list needs. Ticking it on makes
the incidence, at the place the vertex already occupies, so it snaps nothing; ticking
it off releases it and *leaves the body listed*, holding nothing. There is therefore
no "remove" button on either side, and nothing a press could lose.

A body's extent is its own outline, so `bodyContains` answers it exactly. A line has
no width, so "on it" has to be a tolerance rather than a test -- and the tolerance is
a question about the **drawing**: the vertex is on the line when its dot touches the
line's stroke, which is what a person tapping the two together is judging. So it is
measured in **pixels** and converted through the camera, like every other hit test in
`§13.2`. A fixed distance in metres would mean something different at every zoom, and
would make the obvious gesture -- tap a spot on the line, then tick `joined` -- work
or not work depending on how far you had scrolled.

- *A vertex selected*: its label, its world position, and one row per **site** -- the
  body, where the vertex sits in that body's frame, `joined`, `welded`, and `slide`
  for line bodies. Plus the honest note when fewer than two incidences are welded.
- *A body selected*: its label, its own properties as now, and one row per vertex
  **inside** it -- the same relation, read the other way, with the same coordinates
  and the same ticks. Selecting a row selects that vertex.
- *A line selected*: its label, `soft`, `posable`, every vertex on it in station
  order -- its joints and the ones merely lying on it alike -- and the distance
  between each consecutive non-sliding pair, editable. Plus its meshing disks.
- *The background selected*: reachable from the empty-bench panel, listing every
  vertex pinned to ground.

**Where a vertex is** is one question with one answer, and the answer is whichever of
these comes first:

| | |
|---|---|
| what **grounds** it | a joined body, or the background |
| what **carries** it | a line it is joined to, at its station |
| its own mark | the background incidence, which is simply its world coordinates |

A *joined* incidence always wins, which is the whole of it: joined to a body, the
vertex moves with the body; joined only to a line, it moves with the line. An
incidence on a body the vertex is **not** joined to never locates it -- that marks a
material spot, saying where the vertex *was* -- so releasing the last join grounds the
vertex where it stood, and a completely unjoined vertex has its coordinates in the
background frame, the one frame that cannot move out from under it.

A line **carrying** a vertex is not a second locator sneaking past `§X.3`. It does not
place the bar -- it is not in `lineJoints` and it builds no rows -- so the line's own
frame is still derived without ever consulting it, and the definition bottoms out.
Which is what the physics says too: a bare vertex has no mass and no forces on it, so
nothing can drive it along its slot. Whether the joint is ticked to `slide` makes no
difference until something with mass is joined at it, and then it slides freely. So a
vertex joined to a line moves with the line exactly as a joint that could not slide
would. The one thing that has to be remembered is its **station** -- a point on a rail
with nothing saying where along it is not a position -- and that station is written to
the file whether or not the joint is `fix`ed, because for a rider it is not the
constraint `fix` names but the whole of what says where the point is.

**Every coordinate in one of those rows is editable, and committing one is a solve
attempt.** Where an incidence holds the vertex there, the number *is* the anchor, so
the anchor moves and the assembly has to follow; where nothing holds it -- a body
merely listed, and a body *marked* but not joined alike, since a mark holds nothing
either -- there is no anchor to move, so the vertex goes to the point named and its
own joins are re-read -- the same edit its own x/y field makes, said in another
body's frame. A line's coordinate is its station: a held joint owns one, so the
number is stored and the bar rearranges around it, while a slider -- or a vertex
with no joint at all -- owns none, so the number says where along the bar to put the
point. Either way it ends in
`projectPositions`, which is an *attempt*: a distance the mechanism cannot take
leaves the bench wherever the solver could get to, exactly as typing a body's pose
does. Ticking `joined` on ends in the same projection. The tick itself is exact where
it stands, so on its own that is a no-op; what it is for is the rows the tick has just
brought to life, so that ticking a line and a body at one vertex lands on the same
assembled bench in either order.

**A row reads the point, not the editing history.** Move the vertex -- drag its
handle, type a coordinate, commit one in any of these frames -- and every row on
every panel that lists it says the new place, at once and while the gesture is still
going on. That is one point read in several frames, so any two rows disagreeing would
be one of them having stopped keeping up. Three things make it hold: a move re-reads
each frame's own coordinate (`§X.3`), a row with no anchor behind it is read live off
the geometry, and rows that *appear* and *disappear* -- the point dragged into a
disk's outline, or out of it -- rebuild the panel, since a refreshed number cannot add
a row. A **station** is included in that, and the drag is where it is *set*: a
station is where a joint sits along the bar, so dragging the joint by its handle
re-reads it and the bar takes the length the hand left -- which is the rod's endpoint
drag said in the line's vocabulary, and the same "the geometry the gesture left is
the geometry to hold" rule a pose drag and a scaled box already follow. A number
*typed* into the panel is the other thing and stays the other thing: a solve attempt
the mechanism may refuse. And every station on that line is re-read, not just the
dragged joint's, because a station is a distance from the **origin** -- drag the
joint at station 0 and it stays at 0 while every other station moves, the origin
having gone somewhere else. A vertex the line merely *carries* keeps its station
through all of it: for a rider the station is not a length being set but its
position, so it rides the bar wherever the drag takes it.

**A bar with one joint left rides it.** Under-determined is a state, not a failure
(`§X.15`), and the state a line is in when only one of its joints is still grounded
has an answer: it keeps the **heading** it was left with, measured in that joint's
frame, and everything else on it rides at its station. So the body it hangs off
carries the whole arm, turning and all, and nothing outside the assembly is consulted
-- which is the point. What it used to consult was the *mark* the loose end left
behind, and a mark is where a point **was**: the bar pivoted about a spot on the
background that nothing held, that no panel listed, and that the body walked away
from. That is the same rule as `§X.3`'s, one level up: a mark never locates anything,
and a line placed by one is a line located by a point nothing holds.

The heading is a **capture**, exactly like a weld's rest angle: taken at the moment
the joint set changes and holds what it finds. A line two joints still place holds
none -- it derives one, and a capture kept beside a derivation only goes stale behind
it -- and a line with nothing grounding it has no frame to hold one in, so there the
marks are all there is and the old reading stands. Only the middle state writes it to
the file (`ang=`, `§X.10`), because only there can nothing else work it out.

It builds no rows either way. A bar with one joint has nothing to hold, and a loose
end has no mass to hold it with -- which is the same reason a vertex a line carries
asks nothing of the bar. **Dragging that loose end aims the bar**: nothing else places
it, so the point is its free end and the drag says both things at once -- which way
the bar points, and how far along it the point sits. On a bar something else places,
a carried point still runs *along* it, because there the bar's own place is not the
point's to set.

Dragging the vertex is the *only* gesture that sets a station, and the case that says
so is a **body** drag. A body a line grounds is pinned -- that is what grounding is,
and `§X.9`'s arrangement leaves it no freedom at all -- so the hand moves it nowhere
rather than moving it and restretching the bar behind it, which is what re-reading
the bar's own captures after a kinematic move used to come to. Posing such a
mechanism is what the line's `posable` tick is for (`§X.6`), and unticking a weld or
a `slide` is the other way to say the same thing. Every other body drag is solved
rather than assumed, so the bars hold their lengths through it already.

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
re-reads gains one entry -- its joints' stations, which is what a line's segments are
differences of -- and loses one, since rest angles now live on incidences.

---

## X.12 Phasing

Each phase leaves the bench working, the examples loading and the validators green.

**Phase 0 -- the frame seam. `[done]`** `lineFrameOf` (§06.1b) is the line's own
frame: `angCols` for its heading, and its material point at any `(station, lateral)`,
both derived from the two endpoints and owning no coordinates. `frameAngleRow` is the
one row that ties two frames' angles, which is what `weldA`, `weldB`, `prismaticA`,
`prismaticB` and `pt.lock` all were. Every row now carries a `role` and, where a kind
has more than one of a role, an `at`; physics records them beside the multipliers and
`reactionOf` (§09.3) looks one up instead of re-deriving row order from the joint's
flags in a second file. New validator `tools/frame-check.js`, which checks the frame
against a finite difference of the live geometry.

No new objects, no format change, and the arithmetic is unchanged but for one thing
worth recording: the weld row used to form the reciprocal as `(-1/L)*n` and the
point-on-line rows as `n/L`, two roundings against one. Unified on the single-rounding
form, weld rows moved by **under one ulp** -- verified row by row across every bundled
example and five synthetic benches covering every weld, lock, pinion and vessel
endpoint, with everything else bit-identical. The reaction readout was checked
separately and harder: for every joint on every bench, the role lookup resolves to the
*same multiplier index* the old counting reached.

**Phase 1 -- vertices. `[done]`** The vertex (§06.2e), its incidences, its label, the
vertex tool, the vertex panel and the vertex list a body shows, the canvas dot and
label, the `vertex` line in format version 4, and the compile of `§X.5`'s first rule.
`pin` retired into it, and two things it could not say arrived: a point pinned to the
background with its rotation left free, and a body merely MARKED at a vertex rather
than held there. New validator `tools/vertex-check.js`, and a new bundled example
(`hinge`) that is a ground pin and a weld and nothing else.

One correction made after it shipped: **a vertex had nowhere to be of its own**, and
was deleted when the last body it could borrow a position from went. `§X.15` is what
replaced that, and why. Two corrections made while building it:

- **There is no `vertices` array.** A vertex lives in `constraints`, with
  `type:'vertex'`, because that is the list of couplings and a vertex is one -- the
  pin generalized, and the pin lived there. The payoff is that islands, the row
  assembly, the position projection, Reset, the scratch clearing and the selection's
  membership rule needed no change at all, which is what `§X.14` asks of every phase
  and what a second array would have cost. A `line`, in phase 2, is a *body* and goes
  in `bodies`: bodies have coordinates, couplings have rows, and each list holds the
  things that behave alike.
- **Rod, slot, rack and spring keep their own `{id, off}` endpoints.** The plan said
  to convert them to vertex references here, "so there is exactly one place a shared
  point is stored". But once the pin is gone, the only SHARED points in a scene are
  vertices already -- a rod end that nothing else touches is not a shared point --
  so the goal is met without the conversion, and doing it now would be phase 2's
  blast radius without phase 2's payoff, done twice. It moves to phase 2, where the
  line is the thing that actually wants it.

Body labels moved with it. Giving bodies their own would change every reference in
the format, and that is a version bump the line is going to force anyway.

**Phase 2 -- lines. `[done]`** The `line` (§06.2f) is a straight massless bar with a
frame and no coordinates: it lives in `constraints` beside the vertex, because that
array is the list of things that produce ROWS and `bodies` is the list of things that
produce COLUMNS. Its joints are the vertices naming it, and it holds only a
compliance, a `posable` flag and its meshing disks -- its frame, its extent, its
origin and its stations are all derived. Rod, slot, rack and the linear spring are
retired into it, along with the whole extra-control-point machinery of §06.2c, which
was what a joint needed before its joints were objects. `refreshFrozen` is **ported**
to the line rather than restated per `§X.9`, `posable` moved from the rod to the
line, and format version 5 is the `line` line plus `on=`'s line-joint options.

Body and line **labels** landed with it after all, and the endpoint conversion did
not: rod/slot/rack/spring are gone, so there are no `{id, off}` endpoints left to
convert -- a line's joints ARE vertices. Labels are editable names drawn beside the
thing; the file still references bodies and lines by their numeric id and writes a
`label` only when it differs. Making the label the file's reference is a further step
and is not taken here.

The migration was verified against the engine it replaces, which is the strongest
evidence available: every bundled example was rebuilt through the line/vertex
constructors and run beside the old world for three seconds of real substeps. **All
fourteen are bit-identical.** Two things had to be got right for that, and both are
recorded in §06.2f: the station ORIGIN is the first held joint in joint order rather
than in station order (or adding a joint silently reinterprets every station), and the
placement pair's orientation is anchored so a joint added beyond the far end cannot
turn the bar end for end and half-turn every weld on it.

New validator `tools/line-check.js`; `tools/multipoint-check.js` retired with its
subject; `frame-check`, `posable-check`, `rack-check`, `select-check`, `expr-check`
and `scene-roundtrip` all ported.

One correction made after it shipped: **a line was deleted once it was down to fewer
than two joints**, which -- with the vertex rule above -- made deleting one disk take a
vertex and a line with it. `§X.15` is what replaced that, and why. Two things this
phase settled that the plan had not:

- **One held joint on a line holds nothing.** A station is a distance from the origin,
  so a lone held joint IS the origin and has nothing to be measured against. That is
  the same shape as one weld at a vertex holding nothing, and it has the same reason:
  both are relations, and a relation needs two parties. To pin a rider in the world,
  hold one of the joints that places the line as well. The panel says so.
- **A compliant line's stations are a length the ELEMENT owns**, so a selection box
  scales them rather than re-reading them, and an authored pre-stretch survives -- the
  rule `SCENE.md` §S.9 states for a spring's rest length, applied to what replaced it.
  A rigid line's stations are captured geometry and are re-read like everything else.


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

## X.13 Decisions, settled and open

Five questions this note originally left open. Four are answered and folded into the
sections above; the reasoning is kept here because each one moves the design.

1. **What a fresh line's joints default to: SLIDING, all of them** (`§X.4`). Rails
   get placed about as often as rods, so neither should be the privileged one; a
   line that starts by asking the solver for nothing is the calmer thing to drop
   onto a bench; and it puts the quick solve exactly where it earns its keep -- the
   third joint and after, which is the only one that can be somewhere the line is
   not.
2. **`soft` is a compliance, `1/k`, not a modulus** (`§X.6`). Easier to think about,
   and a rod's stiffness stops depending on its length.
3. **One label namespace** for vertices and bodies (`§X.8`), so an expression name is
   unambiguous.
4. **Extent is derived and there are no end stops** (`§X.4`): a line with only
   non-sliding joints is a finite bar, a line with any slider is an infinite rail,
   and a moving slider may always pass anything on the line -- including a
   non-slider, and including a slider that happens to be stationary.

One remains open, and it belongs to the phase that is furthest out:

5. **How much of the strand to specify before building it.** `§X.7` sketches it and
   says why the earlier attempt is not to be built on. The
   rotational-spring-as-compliant-strand claim in particular needs a worked check --
   that a torsional rate falls out of a wrapped compliant strand with the energy and
   the rate the current element has -- before phase 3 commits to it.

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

---

## X.15 Nothing owns anything: deletion, and the under-determined state

The rule, stated once so every phase can be checked against it:

> A body, a vertex and a line are objects of equal standing. A constraint between them
> is a statement **about** them. Deleting an object removes the relations that named
> it and **nothing else**. No object is ever deleted for having lost one.

Phase 1 and phase 2 both broke it, in the same way and for the same reason, and it is
worth recording how because the mistake is an easy one to make twice. Both had a rule
that read like tidiness -- *a vertex with nothing left to be a point on has nothing to
say; a line with one joint is not a line* -- and each was true about **rows** and
false about **objects**. Compiled to a cascade in `dropBodyFromConstraints`, run to a
fixed point, they made a disk the owner of the vertex at its centre and of the line
between two of them: delete one circle and a vertex and a line went with it, silently,
with nothing on the canvas having said they were dependents. That is the model the
vertex was introduced to replace, re-entering through the deletion path.

The two ideas the fix separates:

| | what it means | what happens when it fails |
|---|---|---|
| **placed** | something says where this object is | it falls back: a vertex on its own background mark, a line on the two furthest joints wherever they are |
| **holding** | it produces rows | it produces none, and says so |

Neither is existence. An object that is placed but holding nothing is an ordinary,
useful thing to have on a bench -- it is what every arrangement looks like halfway
through being built. So:

- **A vertex is never deleted with a body.** Its incidences lose the row that named
  the body, and `settleVertex` runs first, on the pose still on screen, so it comes to
  ride a line it is joined to or leaves a mark where it stood. It does not move.
- **A line is never deleted with a body or with a vertex.** Its joints are the
  vertices naming it, and those vertices are still there.
- **A line with fewer than two grounded joints is still drawn and still pickable.**
  Its *kinematic* frame (`lineFrame`, what the rows are built from) needs joints with
  columns and is honestly null without them; its *placement* (`linePlacement`, what
  the canvas, the picker and the panel read) needs only joints with a place, and a
  vertex always has one. An object that stops being drawn when a neighbour is deleted
  is only a slower way of deleting it.
- **A carried vertex is placed by the bar only while the bar can place itself.** When
  it cannot, the station goes rather than being left behind: a number measured against
  a bar with no origin would put the point back at station 0 the moment the bar could
  be placed again, on top of whatever already sits there. Two vertices whose disks are
  both deleted must end up two metres apart, not on top of each other.
- **One delete path, not four.** `deleteConstraint` is the single door: it strips a
  line's joints' incidences before the line goes, so no `on=` is left pointing at an
  id the file no longer defines. Three call sites had drifted apart -- the delete
  tool, the keyboard and the two panels -- and two of them wrote scene files the
  reader would not take back.

Placement excludes the ride, deliberately and for the same reason `lineJoints` demands
a primary: a line cannot be placed by a point it is itself carrying. That is what
keeps the whole definition from closing on itself, and it is why `lineJointsAll` reads
each vertex's **anchor** -- what grounds it, else its mark -- and never its station.

What this does **not** claim is that a line owns a placement the way a vertex owns a
mark. A line with **one** joint, or none, still has nothing to fall back on and is
invisible; it can only be reached from the file. Giving a line its own remembered
heading is the symmetric move and is left open deliberately -- it is a stored quantity
where everything about a line is currently derived, and it should be decided with
phase 3, where the strand asks the same question about a path.
