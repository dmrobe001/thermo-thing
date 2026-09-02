# Verification scripts

Standalone `node` scripts that check claims made in the design notes. They do not
import the simulator -- each one integrates the minimal system the claim is about, so
it can be read and re-run independently of the engine.

| script | checks | design note |
|---|---|---|
| `vessel-check-dynamics.js` | a free spinning/breathing vessel conserves energy and angular momentum to machine precision under the four-coordinate model | `VESSEL.md` §V.6 |
| `vessel-check-stiff.js` | the discrete-gradient gas force conserves energy exactly at any step size and never crosses zero volume, where an explicit force pass diverges | `VESSEL.md` §V.7 |
| `vessel-check-massmatrix.js` | the centred one-body formulation reproduces the exact gas mass matrix, where a fixed `m/3` cap mass is right only when the head is welded | `VESSEL.md` §V.3 |

Run any of them with `node tools/<script>`.
