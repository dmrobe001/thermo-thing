# Cable / Spool Constraint — Design Note

Design of the winding-cable constraint: a strand of fixed total length that wraps
around one or two spools, carries tension only, and conserves energy exactly. This
note is written to slot alongside `DEVELOPMENT.md`; it references that document's
section numbers (§4.1 belt/rod rows, §4.3 unilateral/hard-stops, §6.5 row assembly,
§8.3 the velocity-projection solve) rather than repeating them.

> **Status (as built):** The single-spool (point ↔ spool) case described below —
> §C.1 through §C.4, §C.6, §C.7 — is implemented (code §06.3 `cableFrame`, code
> §08.2). The two-spool generalization (§C.5) and the open items (§C.8) are not yet
> built.

## C.1 The claim: the ideal wrapping cable is energy-conserving

An inextensible, massless cable on a frictionless spool stores no energy and does no
net work. Its tension is a workless constraint force, exactly like a rod's. Any
kinetic energy lost by the current implementation is a formulation artifact, not
physics — so the cable does **not** need to be modeled as an idealized elastic, and
should not be. Elasticity would reintroduce stiff-ODE timestepping and a springy
energy channel that muddies the energy accounting the sandbox exists to expose, and
it would forfeit the finite, exactly-reachable extent that makes a rope feel like a
rope.

The reason it is conservative: a point tethered to a fixed spool of radius `R` by a
taut cable traces the **involute of the circle**. Parameterize the taut
configurations by the departure-point rim angle `β`:

```
p(β) = c + R·(cosβ, sinβ) + (L − R·β)·(−sinβ, cosβ)
```

with `c` the spool center, `L` the total length, and `ℓ = L − R·β` the free
(unwound) length. Differentiating:

```
dp/dβ = ℓ·(−cosβ, −sinβ)
```

The body's only permitted motion is **perpendicular to the string direction**
`(−sinβ, cosβ)`. Tension acts along the string; permitted motion is orthogonal to it;
therefore the tension does no work and kinetic energy is conserved as the cable winds
in and `ℓ` shrinks. (This is the classic tetherball result: constant speed, spiraling
radius, string always normal to the path.)

**Consequence for the solver.** The taut constraint is a single row that removes the
velocity component *along the current string direction* — a slot-like row `ŝ·v = 0`
with `ŝ` recomputed each step. Because the true motion is already perpendicular to
`ŝ`, the §8.3 projection removes a component that is ≈ 0, so the impulse it applies —
and the energy it removes — is near zero to integration order, exactly as for a rod.
If the current implementation instead phrases the row against distance-to-center or
distance-to-a-fixed-anchor, its direction is not perpendicular to the involute
motion, the projection fights real motion every step, and that is where the energy
leaks.

> **As built:** `cableFrame`'s row is the gradient of `totalUsed` (wound arc length +
> tangent segment length) with respect to the tether point's position, not a naive
> distance-to-center row. That gradient is exactly the tangent-line unit vector
> `ŝ = (T−Q)/|T−Q|` — the extra `rs·sign·Dy/Dx` terms in `Jx`/`Jy` are precisely the
> correction that makes it so, rather than the plain `(T−center)` direction a
> distance constraint would use. This is the energy-conserving row described above.

## C.2 One object, not several special cases

A cable is a **single length-conservation constraint on a strand that leaves each end
tangentially**, with the wrapped amount at each end carried as state. A bare tether
point is simply a spool of radius `R = 0`. Writing the departure logic *per end*
collapses the whole family:

| Ends | Radii | Result |
|---|---|---|
| point ↔ point | R₁ = R₂ = 0 | plain rod, no wrap (already in library, §4.1) |
| point ↔ spool | one R = 0 | tetherball / falling-weight cable |
| spool ↔ spool | both R > 0 | two-pulley belt-cable |

The two-spool case falls out of the one-spool code for free once the departure
geometry is per-end. This is §4's "one atomic operation applied to different
features," with the feature being a per-end radius.

> **As built:** Only point ↔ spool exists (`cb.tether` is always a bare point;
> `cb.spool` always carries a radius). Point ↔ point stays the separate `rod`
> constraint rather than a cable with both radii zero. Spool ↔ spool (§C.5) is not
> implemented — see the two-spool generalization below for what that would add.

### The wrap coordinate

Track, per end, an accumulated wrap angle `Δ ≥ 0`, so the free length is
`ℓ = L − R·Δ` (summed over both ends in the two-spool case). `Δ = 0` is fully
unwound (departure point coincides with the material anchor); increasing `Δ` is
winding on. `Δ` carries the **integer turn count**, which the instantaneous geometry
(known only mod 2π) cannot, so it is what makes multi-turn and reversible winding
well-defined.

The three awkward cases the cable must survive all map onto this coordinate:

- **Fully unwound, tangent would fall past the anchor** → the **lower end-stop**
  `Δ = 0`. Below it, the departure point cannot slide past the material tie-off, so
  the constraint pivots at the anchor and becomes an ordinary rod to a point on the
  spool. The transition is continuous: at `Δ = 0` the tangent point *is* the anchor,
  so `ŝ` does not jump.
- **Tether inside the spool perimeter** → can occur *only* in that unwound rod
  regime, where it is harmless (a rod needs no tangent). In the wrapped regime the
  free segment is a tangent ray, which only reaches points with `d ≥ R`, so a wrapped
  tether is automatically outside. Gating wrapping on `d ≥ R` means `sqrt(d² − R²)`
  is never taken of a negative number.
- **Both ends spools** → the common-tangent-of-two-circles construction: one length
  row, two wrap coordinates, reducing to the existing belt `R₁ω₁ = R₂ω₂` when both
  centers are fixed.

**Key decoupling for stability.** The string direction `ŝ` depends only on current
positions and the wind sign — *not* on how much is wrapped. So the taut velocity row
is local and cheap, while `Δ` is slow bookkeeping used only to detect the two
end-stops and to stabilize position drift. Keeping these separate is what makes
many-turn winding numerically calm.

> **As built:** `cb.spoolAngle` (the ABC angle at the spool center, from anchor to
> tether, unwrapped continuously) plays the role of `Δ`; `windAngle` (its
> tangent-wins/anchor-wins reduction) is the unsigned wrap amount, and
> `woundLength = |windAngle|·rs` is `R·Δ`. The anchor/rod-regime end-stop (`Δ = 0`)
> falls out of the same `tangentWins` comparison that picks the departure point, so
> there is no separate branch for it. The wrap coordinate now *does* freeze while
> slack (see the robustness note below) — it did not before this pass, which was the
> one place the as-built code deviated from this section's stability argument.

## C.3 Per-cable state

```
# per end (per spool); a bare tether point sets R = 0
c            spool center                     (world, or a dynamic body's frame)
R            spool radius                     (0 for a bare tether point)
sigma        wind sign, +1 / -1               (which tangent; undefined until first wrap)
anchor_a     rim angle where rope is tied,    (spool's local frame)
Delta        accumulated wrap angle >= 0      (0 = fully unwound at the anchor)
delta_prev   previous departure rim angle     (for continuous Delta accumulation)

# per cable
L            total length
active       taut / slack flag
```

> **As built:** `cb.localAngle` is `anchor_a`; `cb.spoolAngle`/`cb._spoolAngle` fold
> together `Delta` and the continuity role of `delta_prev` into one unwrapped signed
> angle (see §C.2's as-built note); `sigma` is derived each step from that angle's
> sign rather than stored separately; `cb.Ltot` is `L`; `cb._active` is `active`.

## C.4 Per-step algorithm — single-spool cable (point P, spool at c)

```
1. GEOMETRY
   d_vec = P - c ;  d = |d_vec| ;  phi = atan2(d_vec.y, d_vec.x)

2. REGIME SELECT
   if Delta <= 0 or d <= R:                 # rod regime (unwound / interior tether)
        A       = c + R * dir(anchor_a + theta_spool)   # material anchor point
        pivot   = A
        s_hat   = unit(P - A)               # string direction = along the rod
   else:                                    # wrapped regime
        gamma   = acos(R / d)               # tangent half-angle, well-defined (d > R)
        delta   = phi - sigma * gamma       # departure rim angle  (VERIFY sign)
        D       = c + R * dir(delta)        # tangent / departure point
        pivot   = D
        s_hat   = unit(P - D)

3. UPDATE WRAP
   Delta += sigma * wrap_to_pi(delta - delta_prev)   # continuous branch
   delta_prev = delta
   clamp Delta >= 0                          # reaching 0 => next step is rod regime
   ell = L - R * Delta                        # current length budget

4. SLACK / TAUT GATE  (unilateral)
   geometric_free = |P - pivot|
   if geometric_free < ell - tol:  active = false    # excess rope; freeze Delta
   else:                           active = true
   # Robust alternative: always include the row, solve, then drop it if lambda
   # returns the compressive sign (rope pushing). Uses the multiplier you already
   # compute; avoids taut/slack chatter better than a pure geometric gate.
   if not active: skip row; P moves freely; Delta frozen

5. TAUT ROW  (one Pfaffian row, §6.5 form)
   J:  s_hat . (v_P - v_pivot_material) = 0
       # v_pivot_material = spool translation + omega_spool x r  if the spool is
       # dynamic. The SAME lambda applies torque R*lambda back to the spool.
   Baumgarte RHS:  b = -(beta / h) * g,   with  g = geometric_free - ell

6. END-STOPS  (unilateral, §4.3 hard-stops)
   fully wound:    Delta >= L/R  (ell -> 0)  => pin body to rim; allow unwinding only.
                   Clamp ell to a small ell_min so s_hat stays defined.
   fully unwound:  Delta = 0                  => handled by rod regime in step 2.
```

> **As built:** Steps 1–2 are `cableFrame`'s tangent/anchor construction
> (`tangentWins` picks the regime; `Q` is `pivot`). Step 3's update now only commits
> when the row is active — see the robustness note below. Step 4's gate is
> `C>-1e-4 && (C>1e-4 || rowJv(f.cols)>0)`, which is the geometric gate plus exactly
> the "robust alternative" fallback in the dead zone (using the row's own current
> velocity rather than a solved multiplier, since the multiplier isn't known until
> after the row is included). Step 5 is `cableFrame`'s `cols` plus the shared
> Baumgarte scaling every row gets in §08.3. Step 6's fully-wound stop is a 2-DOF
> hinge pinning tether to anchor (code §08.2's `fullyWound` branch) rather than a
> clamped `ell_min`; the fully-unwound stop is the rod-regime branch, as specified.

### Robustness notes

- **The near-full-wrap singularity is physical, not fixable.** Speed is `ℓ·|Δ̇|`, so
  as `ℓ → 0` the winding rate diverges — the tether whips around the rim. Clamp
  `ell_min`, and substep when `ell` is small if needed. No reformulation removes this;
  a real tetherball has the same singularity.
- **Verify the `sigma` sign** against a known orbit: start a body in a circular orbit
  that should wind in, and confirm `Delta` increases. The `phi ∓ sigma·gamma`
  convention depends on the engine's angle handedness.
- **Freezing `Δ` while slack** is a deliberate modeling choice, and a principled one:
  a limp frictionless rope's wrap is indeterminate, and holding it is exactly a static
  no-slip condition on the wound portion — consistent with the "friction is absent or
  perfectly static, expressed as a constraint" invariant. Comment it as intent.
- **Direction reversal (σ flip)** can only occur while passing through `Δ = 0` (the
  rod regime), where σ is re-read from the bending direction as the body leaves the
  anchor.

> **As built:** The freezing note above is now implemented: `cableFrame` is still
> evaluated every step from the last-committed `cb._spoolAngle` (so the taut/slack
> gate and rendering always see live, correct geometry), but code §08.2 only writes
> the result back into `cb._spoolAngle`/`cb.spoolAngle` when the row is active that
> step. A slack cable's wound-length bookkeeping therefore holds exactly at its last
> taut value — a swinging tether can no longer silently rewind or overwind the spool
> while the rope is slack — and picks back up from live geometry the instant it goes
> taut again, matching this section's intent. The σ-flip-only-at-Δ=0 property and the
> fully-wound singularity guard were both already structurally guaranteed by the
> existing `tangentWins`/`fullyWound` construction and needed no change.

## C.5 Two-spool generalization

Replace the single tangent point with the **common tangent** of the two rim circles,
selected by the two wind signs:

```
same-handed winds     -> external tangent   (offset uses R1 - R2)
opposite-handed winds -> internal / crossed  (offset uses R1 + R2)

1. Find D1 on circle 1 and D2 on circle 2 by the standard common-tangent
   construction (unit vector between centers, rotated by an angle set by
   (R1 -/+ R2) / |c2 - c1|).  Requires |c2 - c1| > |R1 -/+ R2|.
2. s_hat = unit(D2 - D1) ;   free length  ell = |D2 - D1|
3. Length budget (one conserved row):
        L = R1*Delta1 + ell + R2*Delta2
4. Velocity row = "net rate of rope consumption is zero":
        R1*(spin at D1) + s_hat.(v_c2 - v_c1) + R2*(spin at D2) = 0
   The tangent-point sliding terms are perpendicular to s_hat and drop out; what
   remains is the along-segment center motion plus the two payout rates.
5. Update Delta1, Delta2 from each departure angle exactly as in C.4 step 3.
   Each end has its own anchor end-stop (Delta = 0) and its own fully-wound stop.
```

Fixed centers with both ends fully wrapped recovers `R₁ω₁ = R₂ω₂`: the existing belt
(§4.1) is the rigid special case of this constraint.

> **As built:** Not implemented. Cables remain point ↔ spool only.

## C.6 Degeneracy guards

- **No common tangent** (circles overlapping, or one inside the other): the wrapping
  model is ill-posed. Disable or clamp that step rather than solving garbage.
- **Strand passing through a disk** (possible for a short rope from a rim anchor to an
  interior tether): decide whether spools are "solid." With no collision model the
  honest default is to allow it, but flag the case so a guard can be added later.
- **Interior tether** (`d ≤ R`): keep firmly in the rod regime so the tangent
  construction is never evaluated inside the disk.

> **As built:** The first two guards are only relevant to the two-spool case and
> don't yet apply. The interior-tether guard is implemented: `cableFrame` short-
> circuits to `tetherInside`'s direct/rod branch whenever `d <= rs`, before any
> `acos(R/d)` is evaluated.

## C.7 Why this stays energy-clean

Every regime is one workless direction row carrying a single shared multiplier:
tension acts along the strand, permitted motion is perpendicular to it, and there is
no energy channel anywhere — including across the taut/slack and anchor/tangent
transitions, because position and velocity are continuous through both and the
constraint force is zero at the instant of switching. Nothing here kills kinetic
energy, and the elastic-rope idea remains unnecessary. If numerical conditioning near
degeneracy is wanted, add a small CFM / regularization term (the existing `sim.reg`),
which softens the row without storing energy — distinct from a physical spring and
free of the pre-extent springiness the design deliberately avoids.

## C.8 Open items

- Explicit Jacobian coefficients for the dynamic-spool `omega x r` payout term and
  the two-spool row (the trig is sign-error-prone and not expanded here).
- Choice of active-set discipline: geometric gate vs. multiplier-sign gate (C.4
  step 4), and whether to run cables inside the main equality solve with an active-set
  toggle or on the separate LCP path (§3.7) once that exists.
- Whether spools are solid (collision guard for C.6) — deferred until there is a
  contact model.
