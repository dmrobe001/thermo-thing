# Gas Vessels -- Design Note

Design of the gas vessel: a body of fixed bore and variable length whose interior
holds a gas obeying the ideal equation of state. This note is written to slot
alongside `DEVELOPMENT.md`; it references that document's section numbers (§3.2 the
KKT solve, §3.3 the velocity-linear constraint class, §4.1 rod/slot rows, §6.1
springs as force elements, §7 instrumentation) rather than repeating them.

> **Status (as built):** Implemented -- §V.1 through §V.9, plus the length-locked
> reservoir of §V.8. Code: `geometry.js` §05.2d (the vessel, its inertia and the gas
> state), §05.2c (material endpoint offsets); `constraints.js` §06.1 (`epFrame`'s
> length column, and the rod/slot rows rebuilt on it); `physics.js` §08.1b (the gas
> force) and §08.1/§08.3/§08.4/§08.6; `render.js` §11.3 (`drawVessel`); `tools.js`
> §13.5 (placement, corner resize); `inspector.js` §14.2b. Heat and mass exchange
> (§V.10) are not built. The repository previously carried a *different* gas system
> (two synthetic boundary bodies coupled by a hidden prismatic joint, stripped in
> commit `97f7a72`); §V.3 and §V.9 explain what that design got right, what it got
> wrong, and why this one is not a revival of it.

The short version: **a vessel is one body with four configuration coordinates
`(x, y, theta, len)`, and the gas inside it is a force element with a potential
energy function -- an ordinary nonlinear spring acting on the fourth coordinate.**
Everything else in this note is a consequence of those two sentences.

## V.1 The modeling choice is the deformation field, and it is the only one

A vessel is not a rigid body, so it has no mass matrix until we say how its material
moves. Fix that first and every other quantity -- inertia, forces, energy,
attachment Jacobians -- follows by differentiation, with nothing left to choose and
nothing to reconcile by hand.

Work in **material coordinates**. Label a point of the vessel (shell or gas) by
`(f, lat)`: `f` in `[-1/2, +1/2]` is the fraction of the way along the axis, `lat`
its fixed offset across the bore. The caps are the material planes `f = -1/2` and
`f = +1/2`; the walls are the material lines `lat = +-bore/2`. A material label
never changes -- it is glued to the material -- so the vessel's mass distribution in
`(f, lat)` is a **constant**, independent of `len`.

The deformation is the uniform axial stretch about the centre. The axis is the
body's **local +y**, so a freshly placed vessel has its caps facing up and down and
its bore across, and `(bore, len)` map onto a rectangle's `(width, height)`:

```
r(lat, f) = c + Rot(theta) * (lat, f*len)
```

with `c = (x, y)` the centre point. An endpoint offset on a vessel is that `(lat, f)`
pair, in that order -- `geometry.js` §05.2c `epLocal`/`epOffOf` is the only place the
conversion to a local point lives.

**Units are SI throughout** -- metres, kilograms, seconds, pascals, kelvin, joules --
with an implicit **1 m out-of-plane depth**, so a planar area reads directly as a
volume and the bore reads directly as a cap area. Ambient is 101325 Pa at 293.15 K.
The gas carries a *specific* gas constant (`Rs`, J/(kg*K)) rather than the universal
one, which keeps its mass -- the quantity the vessel needs anyway, for `mu` -- as the
primary field, with no mole count or molar mass to carry alongside. Differentiating once gives the velocity of any
material point, which is the whole physics content of the model:

```
rdot = cdot + omega x Rot(theta)*(f*len, lat) + f*lendot * u        u = Rot(theta)*(1,0)
```

The last term is the deformation: a linear ramp in `f`, zero at the centre, `+-lendot/2`
at the caps. This is the standard quasi-one-dimensional expansion field, and it is
the same assumption that makes `P` uniform through the vessel -- we are already
committed to it the moment we write one pressure per vessel.

Choosing the **centre** as the reference point is not cosmetic; §V.3 shows it is what
makes the mass matrix diagonal.

## V.2 The mass matrix that falls out

Kinetic energy is `T = 1/2 * integral |rdot|^2 dm` over the material. Expanding, the
cross terms carry the factor `integral f dm`, which vanishes whenever the mass
distribution is **symmetric about the vessel's centre** -- true for the two caps
(equal, at `f = -+1/2`), for uniform walls, and for uniform gas. The `lendot`-vs-`omega`
cross term carries `integral f*lat dm`, which vanishes because the bore is
symmetric. So all three coordinates decouple:

```
T = 1/2*M*|cdot|^2  +  1/2*I(len)*omega^2  +  1/2*mu*lendot^2

M   = integral dm                    total mass (shell + gas)
mu  = integral f^2 dm                axial second moment  <- the generalized mass of `len`
A   = integral lat^2 dm              lateral second moment
I(len) = A + mu*len^2
```

Three constants, `M`, `A`, `mu`, plus one length-dependent inertia. Note that **`mu`
is simultaneously the generalized mass of the length coordinate and the coefficient
of `len^2` in the moment of inertia** -- one number governs both, which is why §V.6's
spin behaviour comes out right for free rather than needing its own model.

For a uniform distribution (the sane default, and the one that matches how the
editor already treats a rectangle):

```
mu = M/12        A = M*bore^2/12        I(len) = M*(bore^2 + len^2)/12
```

That last expression is *identical* to `makeRectBody`'s `mass*(hw^2+hh^2)/3` with
`hw = bore/2, hh = len/2` (code §05.2). A vessel is exactly a rectangular body whose
height is a state variable. If thick caps are ever wanted, put a fraction `cap` of
the mass in the two end planes and the rest uniform: `mu = M*(1 + 2*cap)/12`, ranging
from `M/12` (uniform) to `M/4` (all mass in the caps). Nothing else in the engine
changes.

### This is the answer to "if the caps are massless the acceleration is infinite"

`mu = integral f^2 dm` runs over **all** the material, and the gas is material. Even a
vessel with a weightless shell has `mu >= m_gas/12 > 0`, so a pressure difference
produces a finite `lendot` acceleration. There is nothing to track separately and no
inelegance to swallow: the gas's own inertia was always there, and writing the
kinetic energy of the deformation field is what surfaces it. The only degenerate case
is a vessel with no shell mass *and* no gas, which is a massless object and should be
rejected in the editor the way a zero-mass body already is.

## V.3 Why the centre, and what the old design got wrong

Take the vessel's own head-and-cap description: head at `f = -1/2` moving at
`v_head`, separation `x = len`. The exact gas kinetic energy in those coordinates is

```
T_gas = 1/2 * [v_head, xdot] * [[m, m/2], [m/2, m/3]] * [v_head, xdot]^T
```

-- a mass matrix with a genuine off-diagonal term, because head translation and
stretching share material. The previously-stripped implementation modeled this by
giving the moving cap body an effective mass `m/3` and leaving the head body with
none of the gas. That reproduces the `xdot^2` entry exactly and **both other entries
wrong**: total mass `m/3` instead of `m`, cross term `m/3` instead of `m/2`. It is
correct only when `v_head = 0`, i.e. only when the head is welded to the world:

```
  v_head  xdot |   exact      old(m/3 cap)   new(centered)
      0     1 |     0.166667     0.166667     0.166667     <- head welded: agrees
      1     0 |     0.500000     0.166667     0.500000
      1     1 |     1.166667     0.666667     1.166667
      2    -3 |     0.500000     0.166667     0.500000
```

The centred formulation has no cross term to get wrong, and transforming it back into
`(v_head, xdot)` reproduces the exact matrix above, including `m/3` when the head is
pinned. **`m/3` is not the gas's inertia; it is the one-end-welded reduction of it.**
Centring the coordinate is what turns a special case into the general one.

## V.4 The gas is a potential, not a bookkeeping loop

For a gas of fixed mass exchanging no heat, `dU = -P dV` and `P V^gamma` is constant.
So along any mechanical motion the internal energy is a **function of the current
geometry alone**. Store the adiabat invariant `kap = P * V^gamma` instead of storing
`T`, and everything is closed form (`R = 1` and molar mass `= 1`, as before, so gas
`mass` plays the role of a mole count):

```
V(len) = bore * len
P      = kap * V^-gamma
T      = kap * V^(1-gamma) / mass
U(len) = P*V/(gamma-1) = kap * V^(1-gamma) / (gamma-1)
```

Add the atmosphere's own potential and the vessel is exactly a nonlinear spring:

```
Upot(len) = U(len) + P_atm * bore * len
Q_len     = -dUpot/dlen = (P - P_atm) * bore
```

which is the force law the spec asks for -- bore times the pressure difference --
recovered as the gradient of a potential rather than asserted. Check it against the
caps directly: each cap feels `(P - P_atm)*bore` outward, and a force at material
point `f` contributes `f * (F . u)` to `Q_len`, so the two caps give
`(1/2 + 1/2)*(P - P_atm)*bore`. They also give **zero** net force and zero net torque
on the pose, which is correct -- a sealed vessel does not propel or spin itself.

### What this buys

The energy ledger (code §12.1) gains `Upot(len)` in its potential bucket, handled
exactly the way spring potential energy already is (§6.1). Then:

* **§08.6's rescale needs no gas-specific channel at all.** No `_Watm` term for the
  atmosphere's flow work, no `_Qstep`, no per-substep `dU = -P*dV` integration, no
  rule about which substeps may credit a `dV`. The old implementation's most
  intricate bugs -- documented at length in the stripped `DEVELOPMENT.md` §6.1 -- were
  every one of them artifacts of integrating `dU` incrementally alongside a rescale
  that also had to be told which increments were legitimate. A state function has no
  increments to misattribute.
* **Adiabatic branches still emerge rather than being built.** An isolated vessel
  moves along `kap = const` because nothing changes `kap`; that is the definition of
  the adiabat, not a special case.
* **`T` stays the natural inspector field.** Display `T` derived from `kap`; editing
  `T` at fixed `V` re-derives `kap`, which is a deliberate heat injection by the
  player and should be logged as one, exactly as editing a body's velocity injects
  kinetic energy today.

Heat and mass exchange are then the *only* things that touch `kap` and `mass`, which
is precisely the separation §V.10 wants.

## V.5 Attachment points: material by default

An endpoint on a vessel is `{id, off: [f, lat]}` read in **material** coordinates.
Its world position and velocity Jacobian follow from §V.1 by differentiation:

```
world:  (x,y) + Rot(theta)*(lat, f*len)
velCols(dx,dy) = [[ i, dx, dy, dx*(-ry)+dy*rx, f*(dx*ux + dy*uy) ]]
                  with (rx,ry) = Rot(theta)*(lat, f*len),  (ux,uy) = Rot(theta)*(0,1)
```

The fifth entry is the new `len` column; everything left of it is the ordinary rigid
body form. Because `epFrame` (code §06.1) already hands `rowsFor` an opaque `velCols`
closure, **pin, rod, and spring work on vessel endpoints with no changes to
`rowsFor` at all**.

This dissolves the "weird cases" rather than special-casing them, because the
`f`-dependence does the reasoning automatically:

| attachment | `f` | effect |
|---|---|---|
| a cap | `-+1/2` | full `+-lendot/2` coupling -- pinning both caps locks the length |
| mid-wall | `0` | no `len` column -- holds the vessel in place, lets it breathe symmetrically |
| anywhere on a wall | `f` | partially restrains length, in exact proportion to `f` |
| interior ("in the gas") | any | a marker frozen in the flow, riding the expansion |

One cap welded to the ground and the other free is not a puzzle: the centre simply
translates as `len` changes. No degree of freedom is lost by having one length
coordinate instead of two cap positions -- the map `(c_axial, len) <-> (cap1, cap2)`
is a bijection. Two caps welded to each other is a rod between `f = -1/2` and
`f = +1/2`, which constrains `len` directly, and the reaction it carries is readable
through the existing `reactionOf` instrumentation (code §09.3) -- the force in the
vessel's own walls, for free.

So the answer to "should things only attach to the caps?" is **no, attach anywhere**;
and the answer to "should a wall attachment leave the caps unconstrained?" is **not
quite** -- a wall attachment at `f != 0` genuinely does restrain length, and declaring
otherwise would break the virtual-work identity the energy accounting rests on.

### The exception that was considered and not built: a spatial collar

A "clamp that grips the tube while material slides past it" -- which is what a wall
constraint that must *not* touch the length would physically be -- is a different,
equally well-defined endpoint kind: a **fixed offset from the centre** rather than a
material fraction, whose `len` column is zero by construction. It is consistent (an
exact Jacobian either way, and a workless constraint stays workless), but it is
**not implemented**: material attachment already covers every case reached in
practice, including the one that motivated the question, since an anchor at `f = 0`
holds the vessel without restraining its breathing at all.

## V.6 Rotation

Nothing extra is needed, provided the `len` equation of motion keeps the term that
falls out of `I(len)` depending on `len`:

```
mu * lenddot = (P - P_atm)*bore  +  mu * len * omega^2
                                    ^^^^^^^^^^^^^^^^^^
                                    = 1/2 * dI/dlen * omega^2
```

This is the centrifugal generalized force: a spinning vessel stretches, and as it
stretches `I` grows and `omega` falls. Angular momentum and total energy are then
**exactly** conserved with no help from anything else. Integrating the free
spinning-and-breathing vessel with RK4 at `h = 1e-5` over 20 simulated seconds:

```
energy   relative drift:  -4.3e-14
ang.mom. relative drift:  -1.1e-13
```

Two consequences worth stating plainly:

* **Omitting that term breaks conservation.** Treating `len` as an ordinary body
  coordinate with a constant mass and no `omega^2` coupling is the failure mode to
  avoid; it is also the one that a two-bodies-and-a-prismatic implementation gets
  right only by accident, through whatever inertia it happened to assign the caps.
* **A fast spin genuinely flings a soft vessel out.** In the run above the vessel
  breathed between `len = 0.99` and `len = 7.5`. That is real physics (a spinning
  elastic rod stretches), not instability, and it is the reason to expose a maximum
  length, not a reason to damp anything.

The `lendot` mode carries **zero net linear momentum** (`integral f dm = 0`) and
**zero net angular momentum** (`integral f*lat dm = 0`) about the centre. That makes
it a clean internal ("shape") degree of freedom in exactly the sense §08.6's
free-island branch needs -- and it incidentally hands that branch the internal channel
whose absence its own comment records as unresolvable.

Gravity contributes nothing to `Q_len`, since the gravitational potential
`g*(M*c_y + len*u_y*integral f dm)` is independent of `len` for a symmetric
distribution. A hanging vessel therefore does not sag under its own weight. That is
an honest consequence of reducing the shell to one length coordinate, not a bug.

## V.7 The `len -> 0` limit needs no floor

`P ~ len^-gamma` diverges as the vessel closes, and the work to reach zero volume is
infinite for `gamma > 1`, so no piston with finite momentum ever gets there in
continuous time. Only a fixed step can integrate past it. The stripped implementation
fought this with a clamped pressure formula, then a Baumgarte row, then an
anticipatory elastic-reflection impulse plus rules about phantom volume changes.

None of that is necessary once the gas is a potential. Use a **discrete gradient**
for the gas force -- choose the force over the step so that the work it does equals
the potential drop exactly -- paired with a trapezoidal position update:

```
f_gas  = -(Upot(len_new) - Upot(len)) / (len_new - len)
v_new  = v + h * f_gas / mu
len_new = len + h * (v + v_new)/2
```

Every *other* generalized force on the length coordinate rides along inside the same
solve (`physics.js` §08.1b takes them as `Fother`), which makes the identity cover
the whole force set rather than the gas alone: for a total force held constant across
the step,

```
d(1/2 mu v^2) = 1/2*h*f_tot*(v + v_new) = f_tot * dlen = Fother*dlen - dUpot
```

identically, for **any** step size -- so kinetic plus potential is conserved to
rounding and the only net change is the work the other forces genuinely did. Solve the scalar residual by bisection: as `len_new -> 0+`, `Upot -> +inf`
so the residual `-> -inf`, which means **the root is always strictly positive -- the
step cannot cross zero volume, at any speed.** Tested at `h = 1/120` against a plain
explicit force pass, slamming the piston shut at increasing closing speeds:

```
 closing speed | explicit force pass      | discrete gradient
      2        | survives, drift 8.4e-4   | min len 1.2e-1, drift -5.5e-14
      6        | BLEW UP at step 20       | min len 2.9e-3, drift -1.8e-14
     12        | BLEW UP at step 7744     | min len 1.2e-4, drift  1.6e-14
     40        | BLEW UP                  | min len 3.1e-7, drift -4.0e-14
    200        | BLEW UP                  | min len 1.1e-10, drift -1.9e-14
```

So: no `GAS_MIN_X`, no reflection impulse, no `_reflected` flag, no rule about which
substeps may credit a `dV`. Keep a minimum length for *rendering and hit-testing*
only.

Two honest caveats. The trapezoidal update is applied to the `len` coordinate only
(the other three keep the engine's existing symplectic-Euler step) -- which is the
same correction §08.6 already applies to a free island's centre of mass, so it is not
a new idea in this codebase. And the exactness above holds while nothing else writes
to `len` between the force pass and the position update; once a constraint row also
acts on it, the constraint impulse lands in that gap and the identity becomes
approximate. Constraint impulses are workless, so the error is second order, and
§08.6's rescale absorbs it exactly as it does for every other coordinate today.

One case genuinely falls outside the guarantee, and the code says so where it
happens: a vessel holding (near) **no gas at all**. A vacuum has no divergent
pressure to arrest its caps, so the root above can fall below the floor; the step
then lands on `VESSEL_MIN_LEN` and stops the closing motion instead of integrating
through it. That is the only path where the length coordinate's energy identity does
not hold, and §08.6 absorbs the difference like any other residue.

**As built, measured in the browser** (`h = 1/120`), against the peak kinetic energy
of each scene rather than the large constant `U + P_bg*V` offset:

| scene | 50 s of play |
|---|---|
| gas spring (vessel welded to the ground by one cap) | worst `\|dE\|` = 2.1e-9 J, 7.6e-10 % of peak KE; peak-to-peak length swing identical to 6 significant figures across four successive windows -- no numerical damping |
| free spinning vessel | worst `\|dE\|` = 5.9e-2 J (1.9e-3 % of peak KE) and **not secular** -- the excursion is the breathing turning point that `ENERGY_BANK` repays, with end-of-window drift 1e-10 J; angular momentum drift 5e-15 |
| vessel driving a disk through a rod, or loaded by a spring | worst `\|dE\|` under 1e-9 J |

## V.8 Per-vessel state

A vessel lives in `bodies` alongside disks and rectangles -- not in a separate array.
That is what makes islands, save/restore, selection, deletion and every existing
constraint work on it with no per-kind branching.

```
{
  id, kind:'body', shape:'vessel',
  x, y, th, len,                 // the four configuration coordinates
  vx, vy, w, vlen,               // their rates -- all inspector-editable
  bore,                          // fixed width
  mShell,                        // structural mass, user-editable like body.mass
  gas: { mass, gamma, Rs, kap }, // kap = P*V^gamma, the adiabat invariant (§V.4)
  lenLock, static,               // either one zeroes invMu: a fixed-volume reservoir
  // derived, refreshed each substep (geometry.js §05.2d refreshVessel):
  mass, mu, Alat, I, invM, invI, invMu, hw, hh
}
```

Derived: `mass = mShell + gas.mass`, `mu = mass/12`, `Alat = mass*bore^2/12`,
`I = Alat + mu*len^2`. `hw`/`hh` mirror `bore/2` and `len/2` so the rectangle-shaped
picking, snapping, corner-resize and hit-test helpers serve vessels unchanged
(`geometry.js` §05.2b `rectLike`).

The inspector (`inspector.js` §14.2b) exposes the geometry, the shell mass, the four
coordinates and their rates, and the gas. `P`, `T` and gas `mass` are three faces of
one state at a fixed volume, so each edit says what it holds fixed: **temperature**
holds the mass, **pressure** and **mass** hold the temperature. Each is a deliberate
player-authored change to the gas's energy, exactly as typing a velocity into a
body's panel is a deliberate change to its kinetic energy. Resizing keeps the gas
sealed -- mass and temperature carry over, so the pressure follows the new volume.

`lenLock` is the exact analogue of a body's `static` flag applied to one coordinate
instead of three, and a **reservoir** is just a vessel with `lenLock` set and a large
gas mass.

## V.9 What this costs the engine

The engine hardcodes three coordinates per body. Generalizing to a fourth is
smaller than it sounds, because two abstractions already in place absorb most of it:
`mergeCols` (code §06.1) centralizes column arithmetic, and `epFrame`'s `velCols`
closure already hides endpoint Jacobians from `rowsFor`.

A row column becomes `[idx, jx, jy, jw, jlen]`, with `jlen` absent meaning zero, so
**every existing `rowsFor` branch is untouched.** The sites that read columns
positionally are:

| site | change |
|---|---|
| `geometry.js` §05.2 `invMdiag` | return a 4th entry `invMu` |
| `constraints.js` §06.1 `mergeCols` | carry a 4th component |
| `physics.js` §08.1 spring force apply | accumulate into a new `FL[]` |
| `physics.js` §08.2 `rowJv` | add the `jlen*vlen` term |
| `physics.js` §08.3 `maps` / `Jv` / `K` / impulse apply | 4 sites, one extra term each |
| `physics.js` §08.4 position integration | trapezoidal `len` update per §V.7 |
| `physics.js` §08.0 `islandKinematics` | unchanged -- `vlen` carries no `P` or `L` |
| `physics.js` §08.6 internal field | include `vlen` in `keInt` and scale it by `s` |
| `hud.js` §12.1 `energy` | add `1/2*mu*vlen^2` and `Upot(len)` |
| `projection.js` §09.1 | same 3 changes as §08.3 |

That is roughly a dozen lines of arithmetic plus the vessel's own geometry, force,
render, pick, and inspector code -- and it is what the implementation actually cost.
Compare the alternative the spec floats -- two cap
bodies sharing a prismatic joint with a springlike force pair -- which needs six
coordinates plus two constraint rows to cancel the two spurious ones, needs the cap
bodies hidden from picking and the inspector (the stripped implementation's
`synthetic:true`), needs the gas inertia hand-assigned to a cap where §V.3 shows the
obvious assignment is wrong off the welded case, and still has to get §V.6's
centrifugal coupling right through whatever inertia those bodies happen to carry.
The four-coordinate body is both less code and more correct.

One piece of genuinely new row algebra was required, as anticipated:
`twoPointFrame` and `endpointAngleLockRow` built their columns by hand rather than
through `velCols`, so a rod weld or slot prismatic on a vessel endpoint would have
missed its length column. Rather than special-case it, all three of those row
builders (the rod distance row, the endpoint angle lock, and the slot's lateral lock)
were **rebuilt on `epFrame`'s closures**, which reproduces the previous hand-written
columns exactly for two plain bodies and gets the vessel case right for free. Every
bundled example still conserves energy to machine precision after that refactor,
which is the check that it was exact and not merely close.

## V.10 What this sets up for heat and mass flow

Deliberately out of scope here, but the shape of it matters for the choices above.
With `U` a state function of `(kap, mass, len)`, an exchange pass runs **before** the
force pass, at frozen geometry, and touches only `kap` and `mass`:

* **Heat.** Volumes are fixed during the pass, so the closed-form two-body
  exponential relaxation in `T` used previously carries over verbatim; recompute each
  side's `kap` from its new `T` at the unchanged `V`. Because `V` did not move, the
  two `dU` are exactly equal and opposite -- the pass is conservative by construction
  rather than by a correction term.
* **Mass flow.** Same relaxation with pressure and moles in place of temperature and
  capacity, with moles carrying the source gas's internal energy across.
* **Interactions** stay as the spec proposes -- first-class objects naming a body and
  a vessel, active in pairs sharing a body -- which is a good fit for the existing
  constraint/island machinery and keeps the overlap test to explicitly-paired objects.

The only new obligation on §08.6 is the one heat genuinely creates: the amount moved
this substep is a legitimate non-mechanical energy channel and must be added to the
island's invariant, exactly one term, computed once, with no interaction with
anything mechanical. That is a far smaller surface than the stripped implementation's
`_Qstep` / `_Watm` / `_reflected` triple.

## V.11 Where this model is honestly limited

* The gas velocity field is the uniform ramp; there is no acoustics, no shock, no
  pressure gradient within a vessel. Consistent with one `P` per vessel.
* The bore is rigid, so there is no radial gas inertia and no hoop dynamics.
* A vessel does not sag under its own weight (§V.6) and cannot bend -- `len` is the
  only internal coordinate.
* `mu = mass/12` assumes a uniform distribution. A vessel with heavy caps and a light
  gas is a different `mu` (§V.2), not a different model, but nothing in the editor
  exposes that choice today.
* Two vessels' gases only interact through explicit interactions (§V.10), never by
  geometric overlap -- unchanged from the sandbox's standing "nothing interacts unless
  the player says it does" invariant.
* Belt, CVT and cable-spool attachments are restricted to disks, as before; a vessel
  is not a valid pick for any of them. A cable *tether* on a vessel does work.
* A vessel's default shell density (`VESSEL_DENSITY`, 2000 kg/m^3) is a usability
  default, not a physical claim: at 1 atm and human-scale geometry a gas spring is
  genuinely stiff, and a much lighter shell simply rings faster than the eye
  resolves. It is stable either way -- §V.7's step is unconditionally so.

## V.12 Open items

1. Heat and mass exchange (§V.10) -- the reason the state was shaped this way, and
   the next thing to build.
2. Power instrumentation for a vessel: `P*dV/dt` alongside the per-constraint
   reaction readout the joints already have (`DEVELOPMENT.md` §7).
3. Whether a maximum length is worth exposing. Deliberately absent for now: `Upot`
   bounds the length from below, and nothing bounds it from above, so a hard enough
   spin stretches a vessel without limit (§V.6). That is real, and only becomes a
   usability problem if someone hits it.
4. A cap-mass fraction in the editor, if a vessel with heavy caps is ever wanted
   (§V.2 gives `mu = mass*(1 + 2*cap)/12`; nothing else in the engine changes).
5. Vessel-to-vessel and vessel-to-body *contact*, which remains out of scope with
   every other contact (`DEVELOPMENT.md` §3.7).
