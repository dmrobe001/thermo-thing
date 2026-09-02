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

Run any of them with `node tools/<script>`.

`vessel-check-dynamics.js` doubles as an independent check on the engine itself: it
integrates the same system as the bundled **spinning vessel** example with RK4 at
`h = 1e-5`, and lands on the same behaviour the simulator produces at `h = 1/120` --
a length swinging between about 1.00 and 1.18 m while the spin decays from 9 to
roughly 6.8 rad/s. Two integrators, four orders of magnitude apart in step size,
agreeing on the trajectory.
