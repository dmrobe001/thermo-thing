// ============================================================================
//  §15 · EXAMPLES
//  Prebuilt machines wired from the panel buttons (§03.4 / §14.2).
//
//  These are DATA, not code. Each example is a scene file (§17), and loading one
//  is `importScene` and nothing else -- so an example cannot contain a body,
//  constraint or interaction the tools cannot build, and cannot set a field the
//  format does not name. That is the point: the bench twice grew examples with
//  features no player could reproduce, and the fix is not review discipline but
//  removing the code path that allowed it. See SCENE.md §S.2.
//
//  Each scene's own reasoning travels with it, as `#` comments in the file -- so
//  exporting an example hands you the explanation along with the machine.
//  `loadExample(kind)` dispatches on the data-ex key; search e.g. `crank:`.
// ============================================================================
const SCENES = {

pendulum: `scene 2
# A disk on a rigid rod, swinging from a fixed point. The rod's background end is
# a plain pin, not the tool's welded default -- tap the end to free it, or untick
# "end A welded" in the inspector.
sim gravity=on
cam x=0 y=2.6 scale=64

# bodies
body 1 x=2.6 y=4.4 r=0.38

# constraints
rod bg(0,4.4) -- 1 len=2.6
`,

double: `scene 2
# Two links, both pinned at both ends. The classic chaotic pair, and the case the
# Baumgarte gain (§04.3) was tuned against: drift is largest at the extremes of a
# swing, which is where a low gain bleeds energy visibly.
sim gravity=on
cam x=0 y=2.6 scale=64

# bodies
body 1 x=1.8 y=4.6 r=0.32
body 2 x=3.6 y=4.6 r=0.32

# constraints
rod bg(0,4.6) -- 1 len=1.8
rod 1 -- 2 len=1.8
`,

fourbar: `scene 2
# Two grounded cranks joined by a coupler -- the fourth bar is the ground itself,
# the fixed distance between the two background anchors. Nothing states that
# distance: it is implied by where the two anchors are.
sim gravity=on
cam x=0 y=2.6 scale=64

# bodies
body 1 x=-1.2 y=2.8 r=0.3
body 2 x=1.3 y=2.9 r=0.3

# constraints
rod bg(-1.6,1.2) -- 1 len=1.64924225025
rod 1 -- 2 len=2.50199920064
rod bg(1.6,1.2) -- 2 len=1.72626765016
`,

crank: `scene 2
# Slider-crank: a crank pin, a connecting rod, and a piston confined to a
# horizontal line.
#
# The rail is a slot with ONLY its background end prismatic (lock=B). A single
# locked end pins the segment's angle phi = atan2(...) directly (§06.5), which is
# singular if the piston ever passes through the anchor -- so the anchor sits ten
# metres out, well outside its travel.
sim gravity=on
cam x=0 y=2.6 scale=64

# bodies
body 1 x=-1 y=2.4 r=0.24
body 2 x=1.7 y=2.4 r=0.34

# constraints
rod bg(-1.6,2.4) -- 1 len=0.6
rod 1 -- 2 len=2.7
slot 2 -- bg(-8.3,2.4) lock=B restAngB=0
`,

skate: `scene 2
# A knife-edge wheel in zero gravity: the contact point cannot move sideways, but
# slides freely along its heading and pivots freely about it. Nonholonomic -- the
# constraint is on velocity and has no position form to integrate.
sim gravity=off
cam x=0 y=2.6 scale=64

# bodies
body 1 x=0 y=2.6 r=0.45

# constraints
knife 1@(0.42,0) dir=(1,0)
`,

integrator: `scene 2
# A wheel-on-disk integrator: the follower rolls on the big disk's face, so the
# ratio is its distance from the centre. Slide it in or out and the ratio changes
# continuously -- a CVT with no gear teeth anywhere.
#
# The big disk is held by a short rod welded at its BACKGROUND end only. The weld
# fixes the rod's direction, so its free far end -- which sits exactly at the
# disk's centre -- is itself fixed in space, and the disk spins freely about it.
# That is what a ground pin is, built from a rod.
#
# The follower rides the same single-locked-background rail the crank uses.
sim gravity=off
cam x=0 y=2.6 scale=64

# bodies
body 1 x=0 y=2.6 r=0.95
body 2 x=1.17 y=2.6 r=0.22

# constraints
rod bg(-0.5,2.6) -- 1 len=0.5 weld=A restAngA=-3.14159265359
slot 2 -- bg(-10,2.6) lock=B restAngB=0
cvt 1 -- 2
`,

gear: `scene 2
# Rack-and-pinion, via the rolling/gear constraint (constraints.js §06.2, gearFrame).
# The cart's control point sits at its own centre (off is exactly (0,0), so it
# never visibly detaches as the cart translates -- a body's own centre is
# trivially unaffected by the point's own "never rotates with its body" rule
# anyway), and the traction line is frozen horizontal (angle=0). The gear's
# rotation is coupled to the cart's speed along that line at the LIVE traction
# radius -- the perpendicular distance from the gear's centre to the line, here
# 1.0, deliberately different from the gear's own drawn radius (0.4): the
# traction radius is a property of the geometry, not of the gear body itself.
#
# The gear is held in place exactly as the wheel integrator's disk is: a short
# rod to a fixed background point, welded only at that end, pins its centre
# while leaving it free to spin (see the integrator example's own comment).
sim gravity=off
cam x=-1 y=2 scale=64

# bodies
body 1 x=-1 y=2.6 r=0.3 vx=1.4
body 2 x=-1 y=1.6 r=0.4

# constraints
slot 1 -- bg(-10,2.6) lock=B restAngB=0
rod bg(-1.5,1.6) -- 2 len=0.5 weld=A restAngA=-3.14159265359
gear 1 -- 2 angle=0
`,

cable: `scene 2
# A mass hanging from a cable wound on a fixed spool. Tension only: the cable goes
# slack rather than pushing, and the wrap point tracks around the rim as it winds.
# Ltot is the free span at creation -- a captured field, which is why it is written
# out rather than re-derived from the pose (SCENE.md §S.3).
sim gravity=on
cam x=0 y=2.6 scale=64

# bodies
body 1 x=0 y=4.6 r=0.4
body 2 x=0.9 y=4.4 r=0.32

# constraints
# What holds the spool is this rod, welded at both ends to a fixed point. That is
# the whole of what "static" means -- there is no flag.
rod bg(0,5.1) -- 1 len=0.5 weld=both restAngA=-1.57079632679 restAngB=-1.57079632679

# cables
cable 2 -- 1 Ltot=0.830662386292 localAngle=-0.218668945874
`,

gasspring: `scene 2
# A vessel standing on the ground: its lower cap -- the material plane f = -1/2 --
# is welded to a fixed world point, which pins that cap and locks the vessel's
# rotation, leaving the length as the only free coordinate.
#
# Nothing here is a "gas spring" primitive. The oscillation is the vessel's own gas
# potential (geometry.js §05.2d) against the weight the constraint transfers onto
# the length coordinate: with the cap pinned, the centre of mass rises by half of
# any extension, so gravity acquires a generalized force on len that it does not
# have for a free vessel. The adiabat is not imposed either -- an isolated gas
# simply never changes its adiabat invariant.
sim gravity=on
cam x=0 y=2.6 scale=64

# bodies
vessel 1 x=0 y=1.1 bore=0.5 len=1.2 P=101325 T=293.15

# constraints
rod bg(0,0.15) -- 1@(0,-0.5) len=0.35 weld=both restAngA=1.57079632679 restAngB=1.57079632679
`,

spinvessel: `scene 2
# A free vessel with nothing attached, spinning in zero gravity. I(len) grows as it
# stretches, so the spin slows and the centrifugal generalized force (physics.js
# §08.1) trades against the gas -- the vessel breathes. Angular momentum and total
# energy both hold flat while it does, which is the point: the same constant mu
# governs the length inertia and the len^2 term in I.
sim gravity=off
cam x=0 y=2.6 scale=64

# bodies
vessel 1 x=0 y=2.6 bore=0.4 len=1 P=101325 T=293.15 w=9
`,

heatpair: `scene 2
# A hot reservoir warming a working vessel through a fixed plate. Nothing here is a
# "heat exchanger" primitive: the plate is an ordinary rectangle, pinned by an
# ordinary rod welded to the ground, and what
# couples the two gases is a PAIR of heat interactions sharing it (physics.js
# §08.0b). The rate is their conductivities in series times the SMALLER of the two
# plate-to-vessel contact areas, so dragging a vessel half off the plate visibly
# halves it.
#
# The reservoir has a strut inside it -- VESSEL.md §V.8's reservoir, a vessel whose
# fourth coordinate is held by a rod between its own two caps -- so it stores heat
# without storing work, and only its temperature tint moves. The working vessel
# is held by a rod welded at both ends to its own MID-WALL, the material point f = 0
# whose length column is zero (§V.5): its pose is fixed and its length entirely
# free. So the heat crossing the plate comes out as extension, against the
# atmosphere -- heat turning into mechanical work, with nothing in the scene that
# knows what a heat engine is.
#
# Both sides start mechanically balanced, so the vessel walks out quasi-statically
# (its gas-spring period is a fraction of a second, the thermal time constant a few
# seconds) instead of ringing. Total energy holds flat throughout: with the geometry
# frozen during the pass the two dU are equal and opposite exactly.
sim gravity=off
cam x=0 y=2.6 scale=64

# bodies
rect 1 x=0 y=2.6 width=2.5 height=0.24
vessel 2 x=-1.25 y=2.6 bore=0.55 len=1.8 P=276513.730172 T=800
vessel 3 x=1.15 y=2.6 bore=0.9 len=0.9 P=101325 T=293.15

# constraints
# The plate is held by a rod welded at both ends to fixed ground -- that, and
# nothing else, is what makes it static.
rod bg(0,2.15) -- 1 len=0.45 weld=both restAngA=1.57079632679 restAngB=1.57079632679
# The reservoir's strut: a rod from its lower cap to its upper one. Both ends ride
# the same body, so what it holds is the distance between two material planes --
# the length. That is the whole of what a reservoir is.
rod 2@(0,-0.5) -- 2@(0,0.5) len=1.8
# The working vessel's anchor is welded to its MID-WALL, f = 0. That material plane
# does not move with the length, so this pins the pose and leaves the length free.
# Welded to a cap instead -- as the gas spring is -- it would pin neither.
rod bg(1.15,1.75) -- 3 len=0.85 weld=both restAngA=1.57079632679 restAngB=1.57079632679

# interactions
heat body=1 vessel=2 k=2000
heat body=1 vessel=3 k=2000
`,

flowpair: `scene 2
# The mass-transfer counterpart, in the same layout: a pressurized strutted
# reservoir feeding a free vessel through a port body, with a pair of FLOW
# interactions on it. Gas crosses until the pressures match, carrying its source's
# enthalpy with it, so the emptying side cools along its own isentrope while the
# filling side heats -- both visible directly in the temperature tint.
#
# The receiving vessel simply extends: its own pressure is pinned near ambient by
# the atmosphere on the far side of its cap, so almost all of what arrives shows up
# as volume. That is a pneumatic actuator, assembled from a reservoir, a port and an
# anchor, with no actuator primitive anywhere.
#
# The reservoir's 243180 Pa is 2.4 atmospheres, at ambient temperature.
sim gravity=off
cam x=0 y=2.6 scale=64

# bodies
rect 1 x=0 y=2.6 width=2.5 height=0.24
vessel 2 x=-1.25 y=2.6 bore=0.55 len=1.8 P=243180 T=293.15
vessel 3 x=1.15 y=2.6 bore=0.9 len=0.9 P=101325 T=293.15

# constraints
rod bg(0,2.15) -- 1 len=0.45 weld=both restAngA=1.57079632679 restAngB=1.57079632679
rod 2@(0,-0.5) -- 2@(0,0.5) len=1.8
rod bg(1.15,1.75) -- 3 len=0.85 weld=both restAngA=1.57079632679 restAngB=1.57079632679

# interactions
flow body=1 vessel=2 k=0.00003
flow body=1 vessel=3 k=0.00003
`,

// An empty bench is not a special case in the loader -- it is a scene with nothing
// in it, which is exactly what "clear" means.
clear: `scene 2
sim gravity=on
cam x=0 y=2.6 scale=64
`,

};

// Loading an example is importing its file, and nothing else. The text is offered
// to the scene-file card on the way past (§17.5), which shows it -- prose and all --
// for as long as the bench still matches it, and drops back to a plain live export
// the moment you change something. So clicking an example shows you the file that
// made it, and editing the scene shows you the file you now have.
function loadExample(kind){
  const text = SCENES[kind];
  if(text===undefined) throw new Error(`no such example: "${kind}"`);
  // Offered BEFORE the import, because importScene re-renders the panel on its way
  // through and the card reads this then. If the import throws, the offer is
  // harmless: sceneCardText only shows an annotated file while the bench still
  // matches it, and a bench that failed to load does not.
  sceneText = text; sceneDraft = null;
  importScene(text);
}
