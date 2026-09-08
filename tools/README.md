# Verification scripts

Standalone `node` scripts that check claims made in the design notes. They do not
import the simulator -- each one integrates the minimal system the claim is about, in
the same SI units the engine uses, so it can be read and re-run independently of the
implementation and then compared against it.

| script | checks | design note |
|---|---|---|
| `vessel-check-dynamics.js` | a free spinning/breathing vessel conserves energy and angular momentum to machine precision under the four-coordinate model | `VESSEL.md` §V.6 |
| `vessel-check-stiff.js` | the discrete-gradient gas force conserves energy exactly at any step size and never crosses zero volume, where an explicit force pass diverges | `VESSEL.md` §V.7 |
| `vessel-check-massmatrix.js` | the centred one-body formulation reproduces the exact gas mass matrix, where a fixed `m/3` cap mass is right only when the head is welded | `VESSEL.md` §V.3 |
| `vessel-check-exchange.js` | the heat and mass relaxations are the exact solutions of their ODEs at any step size, a finite mass transfer is the exact integral of the differential one, and a transfer conserves energy and linear momentum | `VESSEL.md` §V.10 |
| `scene-roundtrip.js` | every bundled example is a canonical scene file that round-trips byte-for-byte, reloads to the same state and runs the same; the reader rejects everything outside its ledger; and Reset puts back exactly what was loaded | `SCENE.md` §S.2, §S.6 |
| `rack-check.js` | a line carrying a mesh rides its body's frame when both joints sit on it, meshes without slip at the right ratio, aims from its far joint when that one rides something else, and -- with every other rolling row -- articulates a *paused* edit through the projection's delta form, including the ball-and-disk case that must re-ratio without turning anything | `DEVELOPMENT.md` §4.2 |
| `posable-check.js` | a `posable` line touching the dragged body is a bare rail for that drag -- distances free, welds off, every joint sliding -- keeps whatever pose the drag leaves it at and re-grounds where it lands, while one the drag never reached, or any posable line with nothing being dragged, is the rigid bar it draws and integrates as | `DEVELOPMENT.md` §4.1, `SCENE.md` §S.8 |
| `select-check.js` | a lasso catches bodies by their centres; a coupling joins the selection when every body it names is in it (a background anchor comes along, a foot outside does not); the box turns every selected body by its own change in angle and leaves every member exactly as stressed as it was; a scale spreads the parts without resizing them and leaves the machine assembled; the frame is reversible; and a selection copies, pastes back congruent with fresh ids, and loads on its own as a scene | `SCENE.md` §S.9 |
| `expr-check.js` | numeric fields are arithmetic and nothing more: the language's precedence, functions and refusals; a scene file resolving its own names backwards, forwards and to ledger defaults; that what is stored is the number and not the text; that a bad expression leaves the bench standing; and the panel field, its marking and its arrow-key step | `SCENE.md` §S.10 |
| `frame-check.js` | the line frame is a real frame with no coordinates of its own: its `angCols` is the live `d(phi)/dt` (for a plain pair, a background end, and a vessel endpoint whose columns carry the length coordinate), its material point at any station and lateral offset moves as a rigid body's does, every constraint kind tags every row it builds with a role the reaction readout knows, and every joint that carries rows reports a finite reaction | `VERTEX.md` §X.5 |
| `line-check.js` | one object replaces four, so: the three row-shapes a line makes (a drawn guide, a bar, a rail with riders), the four behaviours it absorbed checked against what they mean (a pendulum's fixed radius, a welded ground bar pinning a body, a slider keeping its angle, a disk rolling without slip at its live pitch radius), compliance being the spring it replaced with its strain energy in the ledger, the derived origin and extent, that the heading cannot flip under a new joint, freezing restated, and the tool's two modes | `VERTEX.md` §X.4, §X.6 |
| `vertex-check.js` | a vertex is a named point and the bodies it touches: the rows it builds (two per joined incidence past the primary, one per welded incidence past the first, and none at all for a single weld), what it holds (a shared point, a rigid weld, and a ground pin whose rotation stays free), the primary ordering, the one-incidence-per-body invariant, that ticking join or weld on never snaps the pose, the file round trip, a scaled selection box, the tool gesture -- one tap plants a point in the background frame joined to nothing, and taps after that reach through to the bodies under it -- and the two lists the panels show -- extent rather than incidence, so a body lists every vertex inside it and a vertex every body it is inside, with `joined` the only control, a line's share of the extent measured in pixels so the answer follows the picture, every coordinate in such a row a solve attempt, and a vertex nothing else holds carried by the line it is joined to -- riding the bar at its station, asking nothing of it, and becoming an ordinary joint the moment a body is ticked at it | `VERTEX.md` §X.2, §X.3 |
| `app-check.js` | the paint and panel paths themselves: every bundled example rendered, run, re-rendered, reset and re-rendered, with the inspector opened on every object in it one at a time against a DOM stub that answers only for ids the markup actually wrote, then every tool driven through its whole five-tap gesture with a frame drawn after each -- on a canvas that has a real size (so the grid, a rail across the viewport and a static body's hatch actually run) and that refuses what a browser refuses (a negative arc radius, a non-finite gradient, a runaway op count), and with the contract asserted that a tool's pending first pick carries the world point that is drawn for it -- the surface the other scripts stub out, where an exception inside `render()` kills the redraw chain (code §10) for good | `DEVELOPMENT.md` §8 |

Run any of them with `node tools/<script>`.

`scene-roundtrip.js`, `rack-check.js`, `line-check.js`, `posable-check.js`, `select-check.js`, `frame-check.js`, `vertex-check.js`, `expr-check.js` and `app-check.js` are the exceptions to the "do not import the simulator" rule
above: its claim is about the real reader and writer, so a reimplementation would
test nothing. It loads the engine's source files into a bare context with a stub
DOM. Its last check runs the real `substep` on both the example the tools built and
the world rebuilt from its export -- which is how a *derived* field the reader
failed to recompute gets caught, since no comparison of the file itself can see one. The last check runs a scene, presses Reset, and asserts every
field is back -- which is what fails if a field a run can change is missing from its
row's `state` list. `--canon <name>` prints an example's canonical text, which is how
you regenerate a bundled scene after a ledger change.

`vessel-check-dynamics.js` doubles as an independent check on the engine itself: it
integrates the same system as the bundled **spinning vessel** example with RK4 at
`h = 1e-5`, and lands on the same behaviour the simulator produces at `h = 1/120` --
a length swinging between about 1.00 and 1.18 m while the spin decays from 9 to
roughly 6.8 rad/s. Two integrators, four orders of magnitude apart in step size,
agreeing on the trajectory.
