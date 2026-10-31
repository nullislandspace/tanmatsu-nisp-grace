# Implementation plan: Null Island Space Program

Living document: design, step-by-step status, findings and decisions.
Updated whenever a step starts or finishes, something is measured, or
something is decided. Game design brief received from the user on
2026-10-31; this document is part of the repository and is committed with
the work it describes.

**Game:** *Null Island Space Program* -- "To nowhere, and beyond!"
Kerbal Space Program 1 re-envisioned for the Tanmatsu: build a rocket as a
blueprint, fly it, plan maneuvers on a simplified map. Slug
`at.cavac.nisp`, SD card only, SynthEngine3D.

## Context

What exists today is a solid skeleton, not a game: SynthEngine3D as a
submodule (v2.7), the engine splash on boot, the logo blit on the title,
the Play / Settings / Quit bar, save slots, `settings.txt`, the menu music
(`brittle_rille.mp3`, looped), the livestream as a setting, and the device
test kit driven by a scripted flight path. `main/` is ~5,200 lines and
`make build` / `make verify` are clean.

What it flies over is a placeholder: a seeded heightmap island at 0N 0E,
an obelisk, six survey markers, a starfield, and a directional sun. That
placeholder is about to be replaced by the actual subject of the game --
a planet, a moon and a sun that a ship can orbit, leave and land on.

**The user's brief (2026-10-31), in short.**

- Three bodies for now: **Planet Bob** (a Kerbin analogue), its moon
  **Cheese** (a Mun analogue), and the sun **Frodo** (Kerbol). Size,
  gravity and orbit roughly Kerbal-scale (Part O).
- **No free-hand rocket building** -- there is no mouse. A rocket is a
  blueprint: a control capsule on top, then stages added one at a time
  downwards, each stage configured (diameter and length = how much fuel,
  engine type, RCS yes/no, side boosters, landing legs). Stage separators
  are implicit.
  Side boosters ignite with their stage's main engine and separate when
  burned out.
- The capsule always exists, always on top, and carries an integrated heat
  shield and parachute. Robotic only -- no crew for now.
- **A design has to be saved under a name before it can be launched.**
- **A rocket is one rigid body.** No joints, no wobble. Center of mass and
  center of thrust are computed over all connected parts and change as
  fuel burns and stages separate. This is the single biggest simplification
  in the whole project and it is what makes the physics affordable here.
- No science points, no money, no achievements. The goal is: build rockets
  that fly.
- Three flight modes: fully manual, stability assist, automatic guidance
  onto the next planned maneuver.
- **Simplified map / maneuver planner.** Either place a node on the
  current orbit and edit its six axes with six keys, or (TAB) switch to
  the auto-planner, pick a maneuver type -- *change apoapsis*, *change
  periapsis*, *change inclination* -- enter the parameter, and let the
  planner choose the time and the thrust vector for you.
- Menu, audio and display settings stay the SynthMiner shape (they are
  already that shape; see Part N).

**Follow-ups from the same conversation**, each of which became a decision
rather than a passing note: the build screen is **Engineering** (D-34); a part
that overheats explodes and **splits the stack**, and the capsule's fragment is
the mission (D-30..D-33); **basic aerodynamics** decides which way the air
points the rocket, and an empty stage still attached makes it fly like a dart
(D-35); the heat shield is a **factor, not immunity** (D-36); Engineering
prints part **ratings** and never a survivability verdict (D-37); stages can
carry **landing legs**, which is what makes Cheese landable (D-38); and music
is per context -- menu, Engineering, silence in the air, `impact_lento` above
it (D-28, D-29).

**What this plan is not.** It does not design the career, the tech tree,
parts beyond what flying needs, crew, docking, rovers, or other bodies.
Those are later documents; the numbering here leaves room for them.

## About this document

Parts L, O, A, V, B, F, M, N, G, H are the design. Part D is the
step-by-step plan with the status table. Part E is the findings and
decisions log.

Numbering starts at **F-01 / D-01** in this repository. Standing rules
carried over from SynthMiner:

- **Engine, graceloader or launcher problems: stop and ask.** No
  workarounds in app code.
- Builds run in the **foreground**, never backgrounded with sleep-polling.
- **Commit and push only when the user asks.**
- `make mode_badgelink` before any badgelink command: after the app exits
  the badge is back in the launcher in debug mode (`303a:1001`), and a
  BadgeLink command then fails for a reason that looks like a bug.
- `make push` for code, `make install` only when assets actually changed
  (~4 minutes for the 20 MB of music).
- Tight timeouts; no routine BadgeLink downloads.

---

## Part L: source layout

Three principles, in the same order as SynthMiner's:

1. **Pure modules first.** Anything that can compile with a plain host `cc`
   does: the orbital maths, the rocket maths, the maneuver planner. That is
   what makes the host harness (Part H) possible, and it is the only way
   orbital mechanics gets tested at all -- you cannot see a wrong
   inclination by looking at a badge.
2. **Data tables, not switch statements.** A new engine, tank size,
   booster or capsule is a table row, never an edit in five files.
3. **One directory per concern.**

`*` marks what exists today; everything else is the plan.

```
main/
* main.c                  app_main, se_app_config_t, the frame loop, the mode machine
  sim/                    THE SKY -- pure, host-compilable
    units.h               km / s / km^3 s^-2 everywhere (D-15)      (pure)
    bodies.{c,h}          BODY[] registry: Bob, Cheese, Frodo       (pure)
    kepler.{c,h}          state <-> elements, anomaly solver        (pure)
    patches.{c,h}         SOI switching, the future-patch chain     (pure)
    ephemeris.{c,h}       where Bob and Cheese are, now             (pure)
    atmosphere.{c,h}      density, pressure, drag, terminal v       (pure)
    surface.{c,h}         the ground: radius, landmarks, the pad    (pure)
  rocket/                 THE VEHICLE -- pure
    parts.{c,h}           PART[] registry: capsules, engines, tanks,
                          RCS, boosters, legs, separators           (pure)
    stack.{c,h}           a blueprint -> masses, CoM, CoT, inertia,
                          drag centre, static margin                (pure)
    budget.{c,h}          delta-v per stage, TWR, burn time         (pure)
    heat.{c,h}            per-part heat load, failure, the split    (pure)
    design.{c,h}          the blueprint <-> /sd/nisp/designs/*.nbp
  flight/                 THE FLIGHT -- engine-facing
    physics.{c,h}         rigid body: thrust, drag, torque, staging
    control.{c,h}         manual / SAS / auto-guidance
    warp.{c,h}            time factor, physics substeps
    mission.{c,h}         launch, orbit, impact, touchdown, recovery
  map/                    THE MAP -- mostly pure
    orbit_draw.{c,h}      the conic as a polyline                   (pure)
    node.{c,h}            a maneuver node: 6 axes, the burn vector (pure)
    planner.{c,h}         ap / pe / inclination auto-planner        (pure)
    map_view.{c,h}        the map screen: camera, bodies, node UI
  engineering/            THE BUILD SCREEN
    builder.{c,h}         focus, stage cursor, parameter ladders   (pure)
    sheet.{c,h}           the blueprint: paper, grid, pencil strokes,
                          dimension callouts, title block
    notes.{c,h}           the post-it stat cards
    icons.{c,h}           PART[] -> 32x32 pencil icons, same geometry
                          as the sheet -- no image files
  game/
*   input.{c,h}           se_bindings, the look abstraction
*   hud.{c,h}             the flight readouts (to be rewritten: orbital, not island)
*   scene.{c,h}           the world as drawn (to be rewritten: bodies, not island)
  ui/
*   menu.{c,h}            title bar, slots, settings, pause
*   settings.{c,h}        /sd/nisp/settings.txt
*   title.{c,h}           the wordmark, the motto, the logo
*   logo.{c,h}            the packed RGB565 logo blit
  save/
*   save.{c,h}            the three slots (to gain: mission + design state)
  common/
*   paths.h               /sd/nisp, /sd/nisp/saves, settings.txt
*   rng.{c,h}             xorshift64*, hash2/hash3                  (pure)
*   vfs_compat.{c,h}      the FatFs calls graceloader does not export
  audio/
*   music.{c,h}           the mixer, the per-context mp3 playlists
                          (menu / engineering / space), the procedural fallback
  testkit/
*   devtest.{c,h}         perf / shots over the debug console
*   profile.{c,h}         the phase timers
*   showtime.{c,h}        the kit's clock

tools/                    AT THE REPOSITORY ROOT, not under main/
    nispcheck.c           THE HOST HARNESS (Part H): host cc, links only
                          against main/sim and main/rocket
```

`main/game/scene.c` as it stands -- the heightmap island -- is not kept.
Bob is a sphere (Part A). The island's *idea* survives as the spaceport:
one marked point on Bob's equator with a pad, a launch mount and a
runway-less nothing around it.

---

## Part O: the sky

### O1 -- Units, because float32 has seven digits and this is a solar system

The toolchain is `rv32imafc`: hardware **single**-precision float, no `d`
extension. Doubles work (libgcc soft-float) but they are slow, and the
frame budget is not a place to spend them.

Metres are the problem, not the precision. Kerbin-scale distances are
1e7 m; a float32 at 1e7 has ~1 m of resolution, which is exactly the
resolution the orbit needs and therefore not enough margin anywhere.

**Decision (D-15): the simulation is in kilometres, seconds and
km^3 s^-2.** Bob's gravitational parameter becomes 3530.46 rather than
3.5316e12, an orbit radius becomes 680 rather than 680000, and every
number in the sim sits between 1e-3 and 1e5, where float32 is at its
best: ~1e-7 relative, i.e. **centimetre-level on a 600 km orbit**.
Time stays in seconds (float32 up to ~4e7 s covers a 427-day Bob year
with ~4 s of resolution -- which is why the *clock* the HUD shows is
kept as a double, as it already is in `testkit/showtime.h`).

Rendering is the other direction, and it has its own rule
(**D-16: camera-relative submission**). Every vertex the engine sees is
`body_position - camera_position`, in metres, so the renderer never has
to represent 1e7 and never loses the metre it needs to draw a rocket.
The conversion happens in `game/scene.c`, once per body per frame.

### O2 -- The three bodies

Derived from Kerbal Space Program 1's published parameters, rounded to
what a float32 sim can actually use, and cross-checked against the
numbers players quote (a 80 km circular orbit at 2,278 m/s, Kerbin's
stationary orbit at 2,863.9 km, its semi-synchronous orbit at 1,581.8 km
and 1,272.3 m/s). Those four are the anchors `nispcheck` tests against
(F-11).

| | **Bob** (planet) | **Cheese** (moon) | **Frodo** (sun) |
|---|---|---|---|
| radius | 600 km | 200 km | 2,616 km |
| mu (km^3/s^2) | 3530.46 | 65.136 | 1.17234e9 |
| surface gravity | 9.81 m/s^2 | 1.628 m/s^2 | 171,300 m/s^2 |
| escape velocity | 3,430 m/s | 807 m/s | 946.7 km/s |
| SOI | 84,160 km | 2,400 km | -- (root) |
| rotation period | 21,549.4 s (5:59:30) | tidally locked to Bob | -- |
| atmosphere | top 70 km, p0 101.325 kPa | none | -- |
| orbits | Frodo, a = 13,599,840 km | Bob, a = 12,000 km, i = 12 deg | -- |
| period | 9,203,545 s (106.5 days = 427 Bob rotations) | 139,000 s (38.6 h) | -- |

Bob's stationary orbit lands at 2,863.9 km altitude, Cheese orbits four
times further out than that, and Cheese's Hill sphere (3,170 km) contains
its SOI (2,400 km) -- the three numbers have to be consistent or the
patched conics leak, and `nispcheck` asserts exactly that (F-11).

Frodo's surface numbers are in the table for completeness and are used by
nothing: KSP's sun is a 2,616 km radius ball of 1.76e28 kg, which is a
body with a 946 km/s escape velocity, and the game never goes near its
surface. What matters from Frodo is its `mu` and Bob's distance from it.

Bodies are **data rows** (`sim/bodies.c`), so adding a second moon or a
gas giant later is a row plus a texture, not a code change.

### O3 -- Propagation: conics, not integration

Numerical integration of an orbit is the easy way and the wrong way: it
drifts, it needs a tiny dt near periapsis, and it makes warp 1000 cost a
thousand steps. KSP does not integrate coasting either.

**Coasting is analytic.** A state vector (r, v relative to the current
reference body) is converted to orbital elements -- a, e, i, Omega,
omega, M -- and the mean anomaly advances by n*dt exactly. Converting
back needs Kepler's equation solved:

- elliptic (e < 1): Newton-Raphson on E - e sin E = M, seeded from M,
  4 iterations, and the seed matters more than the iteration count near
  e -> 1.
- hyperbolic (e > 1): Newton on H - e sinh H = M, seeded from the
  asymptote. Escapes and intercepts are hyperbolic, so this is not an
  edge case -- it is every SOI entry.
- e within 1e-6 of 1: clamp. A parabolic orbit is a numerical accident
  waiting to happen and is not worth representing.

Elements are also what every readout wants: apoapsis, periapsis, period,
inclination, eccentricity, time to apsis. They are derived on demand and
cached per state, never stored as the truth (**D-17**: the state vector
is the truth; elements are a view of it).

**Powered flight is integrated.** While a stage is burning, or while
inside an atmosphere, the sim integrates the state with velocity-Verlet
at a fixed `dt_phys` (Part F), because thrust and drag are not conic.
The moment the stage cuts off and the ship is above the atmosphere, the
state is converted to elements and coasting becomes exact again.

### O4 -- Patched conics

One reference body at a time, exactly as KSP does:

- outside Bob's SOI: Frodo-centric conic.
- inside Bob's SOI: Bob-centric. Inside Cheese's: Cheese-centric.
- a *patch* is (body, state at entry, conic, exit state at the SOI
  boundary). The chain is capped at `NISP_PATCH_MAX` (4) and at a total
  predicted time, because the map and the planner only ever need to see
  a little way ahead, and an unbounded chain is how a 30 fps game
  becomes a 3 fps game.

SOI entry is found by intersecting the conic with the SOI sphere --
solved by bisection on true anomaly over the arc, not by stepping.
Exit likewise.

**What is deliberately not here:** n-body gravity, sphere-of-influence
wobble, the restricted three-body problem, atmospheric lift, body
oblativity, and the Mun's `J2`. KSP 1 does not have them either, and
their absence is the reason KSP 1's map mode is predictable enough to
plan with.

### O5 -- Where Bob and Cheese are

Both orbits are circular and fixed in an inertial frame
(`sim/ephemeris.c`): Bob goes around Frodo at 13,599,840 km, Cheese
around Bob at 12,000 km with 12 deg inclination, tidally locked. No
Keplerian elements for the bodies themselves, no eccentricity, no
precession -- which means transfer windows are trivially computable and
a `shots` test of a map screen is reproducible.

The player's frame of reference is Bob: the map mode is Bob-centric by
default, and Frodo only appears when the ship leaves Bob's SOI.

---

## Part A: ground, air, and getting off both

### A1 -- Bob is a sphere

Physics uses a perfect sphere of radius 600 km. Landing detection,
ground collision, horizon, altitude and the map all follow from `|r|`
against `R_body`, which is why the heightmap island goes away: a
heightmap is a *rendering* detail and every physics question here wants
a radius.

Visual surface: a sphere tessellated in LOD rings (Part G) plus a small
set of landmarks drawn as markers -- the **spaceport at 0N 0E** (the pad,
the launch mount, the flag), Cheese's landing sites later. A heightmap
for looks only may come back once landing is what people actually do;
it is explicitly not in the first playable build (**D-18**).

The pad is a point on the equator. At launch the ship's state is Bob's
surface velocity there (174.9 m/s east, from the rotation) plus zero
relative velocity -- which is why launches to the east are cheaper here
exactly as they are in KSP, and why `nispcheck` checks it.

### A2 -- Atmosphere

One model, exponential, because it is the model every drag and heating
question here needs and it costs three floating-point operations:

```
rho(h)  = 1.225 * exp(-h / 5600 m)      h above Bob's radius
p(h)    = 101.325 kPa * exp(-h / 5600 m)
```

70 km is where it stops being anything. KSP 1's real model is a
temperature-and-pressure curve rather than a single exponential; the
difference is a few percent of drag in the regime that matters and is
not worth a table (**D-19**). Cheese has none, which is what makes it a
different kind of landing rather than a harder one.

Drag is `0.5 * rho * v_rel^2 * Cd * A` against the whole rocket's
reference area and Cd from the blueprint (a stack is a blunt cylinder:
Cd ~ 0.6, a fairing-topped stack ~ 0.3). Terminal velocity, the parachute
and the heat load all come out of the same two numbers, `rho` and
`v_rel`.
The `Cd * A` for the whole stack is not a second guess: it is the sum of the
per-part `CdA` rows (Part V2), because Part A5 has to know not only how much
drag there is but where it acts.

Heating is one more reading of those two numbers, not a second model:
Part A4 is the heat model, one accumulated quantity per part with a limit
and a cooling term. It is still a rule rather than a thermal simulation
(**D-20**), but it is per part, it accumulates, and a part that burns
splits the stack (**D-30**..**D-33**).

### A3 -- Landing and recovery

Touchdown is relative speed under a threshold (soft: < 10 m/s on Bob,
< 20 m/s on Cheese at any orientation the capsule tolerates), a
parachute that deploys on a key and above a minimum dynamic pressure,
and a wreck otherwise. A landed mission is written into the save slot as
a completed mission with its mass, apoapsis and delta-v spent -- the
only "score" the game has until the spec says otherwise.

Those thresholds belong to the fragment that touches down, not to the body
it lands on: a stage with a deployed leg set tolerates a faster and less
vertical touchdown than one that lands on its engine bell (Part V7, D-38).
On Bob the chute usually decides the number; on Cheese, where there is no
chute to decide it, the legs are the landing system.

### A4 -- Heating, and what burns

Air does the heating, and one quantity does the air: the convective heat
load on a part goes as `rho * v^3`, density from Part A2 and speed
relative to the air. Reentry and a hard launch are the same equation read
at different numbers, so there is no separate "reentry heating" model
anywhere in the code (D-30).

Exact values on this atmosphere (D-19), as the anchors the thresholds get
tuned against:

| situation | altitude | speed | `rho` kg/m^3 | `rho v^3` |
|---|---|---|---|---|
| entry, periapsis 25 km | 30 km | 2,458 m/s | 0.0058 | 8.6e7 |
| entry, periapsis 25 km | 20 km | 2,458 m/s | 0.0344 | 5.1e8 |
| a hard launch | 10 km | 1,500 m/s | 0.2054 | 6.9e8 |
| a normal launch | 10 km | 1,200 m/s | 0.2054 | 3.5e8 |
| low ascent | 4 km | 900 m/s | 0.5997 | 4.4e8 |

The row that decides the model is the third one: **peak heating rate does
not separate a hard launch from a reentry** (F-15). A fast ascent at 10 km
is hotter per second than a reentry at 20 km. What separates them is
duration -- a reentry sits in the 1e8 band for a couple of minutes, a
launch crosses it in tens of seconds -- so heat must be an **accumulated
quantity per part with a limit**, not a threshold on speed or dynamic
pressure. A model that looked only at instantaneous `q` would either burn
every rocket on the way up or let every rocket survive a sloppy reentry,
and both are the bug this model has to avoid.

Per part, one float, updated in the same tick as the drag:

```
flow     = -normalize(v_rel)                  where the air is coming from
rate(p)  = K * rho * |v_rel|^3 * area(p) * expose(p) / capacity(p)
temp(p) += (rate(p) - temp(p) / TAU) * dt     TAU ~ 30 s, parts cool
fail(p)  when temp(p) >= 1
```

`expose(p)` is 1 for every part in the game, with exactly one exception:
the capsule's integrated heat shield, worth about 0.1 when the shield
faces the flow and worth nothing when it does not (D-31). There is no
shadowing model in the first cut -- a stage behind another stage heats as
if it were alone -- because the shield is the one thing the player has to
think about, and a shadowing model would let them stop thinking about it.

**The shield is a factor, not immunity** (D-36). The capsule is a part like
every other part in the table: it has a `capacity` and a `limit`, and
`temp >= 1` takes it out exactly as it takes out a stage. Shield-first at
2,458 m/s through 20 km is 5.1e7 in `expose`-weighted `rho v^3` -- an order
of magnitude below the unshielded number, and still enough to fill a
capsule's capacity in a couple of minutes. So the tuning target for the
`capacity` / `limit` columns is not "shield forward is safe", it is
**"shield forward is survivable only if you arrived slower than orbital"**:
a deorbit that has burned off a few hundred m/s and opens a chute at 3 km
comes home, and a coasting reentry that lets the air do all the decelerating
from 2,458 m/s does not, whichever way the capsule is pointing (F-16).

A shield that degrades -- a `shield` value that falls as the capsule's
`temp` rises, so a first reentry chars the pod and a second one does not --
is the obvious next refinement and is deliberately not in the first cut. It
softens the exact lesson the first cut exists to teach.

Heating is still a rule and not a simulation (D-20 stands): one float per
part, no temperature field, no conduction along the stack, no radiation
view factors, no solar heating. What changed is what a failure *does* --
Part F5.

### A5 -- Aerodynamics, because a rocket is also a dart

Part A2 answers how much the air pushes back. It does not answer the
question the flight model has to answer every substep: **which way does the
air want to turn this rocket?** That is the difference between a vehicle a
player can fly and a vehicle that tumbles, and it is why the capsule's
shield-first reentry attitude (Part A4) is an *outcome* of the model instead
of a rule written into it.

Per part there is one number, `CdA` (Part V2). The stack's drag is the sum
of those rows, and so is the point that drag acts through:

```
x_CoM = sum(m_i * x_i) / sum(m_i)        centre of mass        (Part V4)
x_CP  = sum(CdA_i * x_i) / sum(CdA_i)    drag centre
SM    = x_CP - x_CoM                     static margin, metres along the axis
```

`x` is measured along the stack axis, outward from the capsule, so
**positive static margin means the drag sits behind the mass**: a dart with
fletching at the back, an arrow, a weathervane. Negative margin means the
mass is behind the drag and the air will swing the tail around until the
vehicle flies the other way round. Margin near zero is mushy -- the rocket
neither seeks the wind nor flees it, and SAS spends its authority holding an
attitude the air has no opinion about.

Torque, at angle of attack `alpha` to the relative wind:

```
q        = 0.5 * rho * |v_rel|^2
N_i      = q * CdA_i * sin(alpha)             sideways force on part i
tau_aero = -sum( N_i * (x_i - x_CoM) )
         = -q * sin(alpha) * CdA_total * SM   restoring when SM > 0
```

divided by the transverse moment of inertia Part V4 already computes. It is
zero above the atmosphere and dominant inside it, and it is one equation
across the whole flight -- from a chute descent at ~200 Pa of dynamic
pressure to a fast low ascent at ~1.5e5 Pa, three orders of magnitude of
`q` and no change in the model.

**The worked case: the burnt-out stage is the fletching.** A 1.25 m stack,
`x` measured from the nose, capsule 1.0 t with `CdA` 1.0 m^2 spanning
0-2 m, a stage with `CdA` 6.0 m^2 spanning 2-12 m:

| stack | CoM | drag centre | static margin | what the air does |
|---|---|---|---|---|
| stage loaded, 8.0 t | 6.33 m | 6.14 m | **-0.19 m** | marginally unstable; SAS earns its keep |
| stage burnt out, 0.8 t | 3.67 m | 6.14 m | **+2.47 m** | a dart: capsule leads, empty stage trails |

Nothing in the geometry changed between the two rows -- the drag centre is
the same 6.14 m both times. What moved is the mass: burning 7.2 t out of the
tail walks the CoM forward by 2.7 m and flips the sign of the static margin.
So the situation a player creates by forgetting to stage -- heavy capsule up
front, a long empty booster still bolted on behind, light end facing away
from the airstream -- *is* the stable configuration, and the air will hold
it nose-first for the whole descent. No special case in the code has to
notice that situation; it is what the two sums already say.

Which is the trap, not the rescue. Held nose-first at reentry speed, the
capsule stays pointed into the heat for the entire descent, and the shield
is a factor of about 0.1 rather than immunity (D-36). A dart that comes home
at 2,400 m/s is a dart that comes home very hot, and it is the capsule that
is at the front of it. Surviving means having slowed down before the air did
the slowing -- a retro burn, a high periapsis, a chute at 3 km -- and that
judgement is the player's, which is exactly why Part B4 refuses to print it
(D-37).

**What is deliberately not here:** no fins, no airbrakes, no body flaps, no
per-part `Cd` varying with Mach or Reynolds number, no induced drag beyond
the `sin(alpha)` term, no wave drag, no cross-axis coupling, and no
aerodynamic heating beyond Part A4's `rho v^3`. Fins in particular are not
parts in this game -- the legend in Part B3 has no such row -- so the
stability a stack has is the stability its mass distribution and drag areas
give it, which is the thing the blueprint already draws. Giving the player a
fin part would mean giving Engineering a stability answer, and D-37 says
Engineering does not get one.

The HUD carries the answer instead (step 6.6): the static margin as a signed
number in metres, so the player can watch it cross zero as a stage empties
and jump as stages separate. It is the only aerodynamic number on screen,
and it is the one that explains why the rocket started doing what it did.

---

## Part V: the vehicle

### V1 -- A blueprint, not an assembly

The player cannot drag parts around. A rocket is a **stack described by
numbers**, built top-down:

```
capsule            always exactly one, always on top; RCS pods yes/no,
                   and 1..3 parachutes
  stage 1          the uppermost engine stage
  stage 2
  ...
  stage N          the bottom stage
each stage: diameter, length, engine type, engine count, RCS?,
            side boosters (0 / 2 / 4) and their size,
            landing legs (none / 3 / 4 / 6)
```

The capsule is the one part that is never in the legend and never
removable, and it is not a fixed part either: RCS pods and 1..3 parachutes
are the player's two knobs on it, and the only two things Engineering can
change about it (**D-51**). No pods and one canopy is the default, and the
default is also what a blueprint written before they existed means, so an
old rocket flies exactly as it did.

Stage separators are implicit: a stage boundary *is* a separator, and it
exists whether or not the player thought about it (**D-21**). Side
boosters belong to the stage they hang on: they ignite with that stage's
main engine and separate when they themselves are empty, not when the
stage is.

Stages are numbered from the top (stage 1 fires first), and staging is
one key press that fires the next stage down -- the KSP order, so that
anyone arriving from KSP already knows the controls.

### V2 -- What a stage is made of

A stage is a cylinder of propellant with engines bolted to its bottom:

```
propellant mass = pi (d/2)^2 * L * rho_prop        rho_prop = 400 kg/m^3
tank dry mass   = 0.08 * propellant mass           (a fixed structural fraction)
engine mass     = table row * count
RCS mass        = table row, when on
booster mass    = the same cylinder maths, per booster
leg mass        = table row * count (3 / 4 / 6)
separator mass  = table row, per boundary
```

Every part row also carries the three numbers the heat model needs
(Part A4) and the one number the aero torque needs (Part V4):

| field | meaning |
|---|---|
| `capacity` | how much heat the part can absorb before it fails |
| `limit` | its failure threshold, normalised so `temp >= 1` is always "this part is gone" |
| `shield` | 0 for everything except the capsule, whose shield is ~0.9 effective when it faces the flow |
| `CdA` | drag area, used for drag, for aero torque, and for the parachute check in Part A3 |

These are tuning targets like the engine table is: `nispcheck` case 9 is
what sets them, because "a normal launch survives, a bad reentry does not"
is a property of these four columns and nothing else.

Diameters are a fixed ladder (1.25 / 1.875 / 2.5 / 3.75 m) and lengths
step 0.625 m to 10 m, so the editor is a ladder of choices rather than a
numeric field -- on a keyboard with one alphabet, "pick the next size"
is a nicer interaction than typing 2.37.

A stage may have **zero engines**: a cargo section, a fuel-only stage or a
diameter adapter is still a stage -- it has mass, volume and a separator
above it, it just adds no thrust. That is what the legend's cargo row
inserts (Part B3), and `rocket/stack` treats it as any other stage so no
code path has to special-case it.

The engine table (first cut, four rows, all Kerbal-shaped):

| engine | thrust | Isp | dry mass | role |
|---|---|---|---|---|
| Flea | 30 kN | 240 s | 0.15 t | upper stage, small |
| Relion | 60 kN | 320 s | 0.30 t | vacuum upper stage |
| Skipper | 180 kN | 270 s | 0.90 t | first stage |
| Mainsail | 300 kN | 290 s | 1.20 t | heavy first stage |
| Booster (solid) | 400 kN | 180 s | sized | side boosters only |

These are tuning targets, not scripture; `budget.{c,h}` computes what a
stack can actually do and the table gets adjusted by the numbers, not by
feeling (F-12).

### V3 -- One rigid body

**The whole rocket is one body** (D-22). No joints, no springs, no
wobble, no part-level contacts. The state is:

```
r, v            centre of mass, in the current reference body's frame
q               attitude (unit quaternion)
omega           angular velocity
m               total mass
```

and the only things that change them are forces and torques applied to
the whole thing. This is the decision that makes the physics fit on the
badge: N parts become 1 body, and the only per-part work left is summing
masses and thrust vectors, which is O(parts) once per physics tick and
parts is under 30.

What that buys, and what it costs:

- **Buys:** no constraint solver, no contact pairs, no joint breakage,
  no fixed-step stability worries about stiff connections, no per-part
  collision. Decoupling is a mass and CoM change, not an impulse.
- **Costs:** no failure from loads, no part collisions, no docking physics,
  no leg that crushes or snaps on contact. Landing legs exist as a touchdown
  rating (Part V7, D-38), not as contact mechanics. All of those are out of
  scope for "build rockets that fly", and each one is a separate future
  document rather than a hidden hole here.
- **The one per-part state that is not a sum: heat.** Each part carries one
  float (Part A4), because "which part burns" is a question a whole-body model
  cannot answer, and it is the only genuinely per-part quantity in the game.
  When a part fails the body is cut at it (Part F5), which changes *which
  parts are in the sums* -- CoM, CoT, mass, inertia -- and not the physics.

### V4 -- CoM, CoT, and the torque that comes from them being different

Everything is measured along the stack axis, which is a 1-D problem:

```
CoM = sum(m_i * x_i) / sum(m_i)          x_i along the axis
CoT = sum(T_i * x_i) / sum(T_i)          only over engines IGNITING now
x_CP = sum(CdA_i * x_i) / sum(CdA_i)     drag centre (Part A5)
```

Both are recomputed every physics tick, because fuel burns and the
numbers move: a stage empties from the bottom of its own tank and the
CoM walks up the rocket. That is cheap (a pass over <= 30 rows) and it
is the whole reason a real KSP-style rocket here behaves like one --
a half-empty first stage is tail-heavy and wants to flip, exactly as it
should.

Torque:

- **thrust torque** = `(CoT - CoM) x F` -- the reason rockets need fins
  or gimbals, and the reason a badly stacked rocket is unflyable in
  manual mode.
- **aero torque** = dynamic pressure x reference area x lever arm,
  = `q * CdA_total * SM * sin(alpha)` (Part A5). It weathervanes the rocket
  into the relative wind when the static margin `SM` is positive and flips it
  when `SM` is negative. Zero above the atmosphere, dominant in it.
- **RCS torque** = a table torque budget per module, applied on demand
  by SAS or the player.
- **SAS torque** = a critically damped controller on the attitude error
  (Part F), which is what "stability assist" means.

Moment of inertia is computed from the same stack as a sum of solid
cylinders about the two transverse axes, recomputed when a stage
separates and cached otherwise.

**Why a lone capsule flies shield-first.** The aero torque above is a
weathervane, and a weathervane is stable when its drag centre sits
downstream of its CoM. The capsule is built so that is true: its dry mass
is biased toward the shield end (the shield and the chute are the heavy
things in it) while its drag centre sits near its geometric middle. So a
capsule with nothing else attached turns itself shield-into-the-wind
because of drag, not because a rule says it may (D-31).

Attach a loaded stage and the stack's CoM walks down toward the propellant
while its drag centre stays near the middle of the whole stack -- and the
same torque now holds the rocket the other way round, engines into the
wind, capsule shield facing away. That is not a special case bolted onto
the model; it is the model, and it is the difference between a reentry
that lands and one that loses the pod.

The two sums say the opposite thing once that stage is empty, and that is the
case flight cares about most: with the propellant gone the CoM walks back up
toward the capsule, the drag centre does not move at all, and the stack has
become a dart. Part A5's worked table is that arithmetic end to end.

### V5 -- Delta-v, TWR, burn time

`rocket/budget.{c,h}` answers, per stage and for the whole stack:

```
delta-v_i   = Isp_i * g0 * ln(m_i / m_{i+1})        (Kostya's rocket equation)
TWR_i       = T_i / (m_i * g0)
burn_i      = propellant_i / (T_i / (Isp_i * g0))
```

Engineering shows all three per stage and the total, because that is the
entire feedback loop the builder has: it is the number that tells you
whether your design can do the mission you are about to attempt, and it
is the number the maneuver planner later compares against (Part M).

`nispcheck` verifies the arithmetic against hand-computed stacks (F-12),
including the case that catches every implementation: a stage whose
TWR is under 1 on Bob cannot leave the ground, and a stack with a good
total delta-v but a first stage under 1.0 TWR is a rocket that never
gets anywhere.

### V6 -- Designs are files

`/sd/nisp/designs/<name>.nbp`, one blueprint per file, text with the
same tagged-field approach SynthMiner uses for save data: forward
compatible, unknown fields ignored, hand-editable on the card. A design
is **not** in a save slot -- the same rocket should be launchable from
any slot, and a slot should be able to have flown several.

**A design must be named before it can be launched** (the user's rule):
Engineering's Launch item is disabled while the design is unnamed, and the
name is what the mission log and the save record.

### V7 -- Landing legs

Legs are a **stage parameter**, in exactly the sense RCS and side boosters
are (D-26, extended by D-38): a stage either has a leg set or it does not,
there is no attach interaction, and the ladder row is
`legs: none / 3 / 4 / 6`. They are not a legend row (Part B3) for the same
reason RCS is not one.

**Why they exist at all: Cheese.** Bob has an atmosphere, so a capsule can
come home under a chute and legs are a nicety that softens the last metre.
Cheese has none (Part A2), so on a Cheese mission the only landing system is
the stack's own thrust and whatever it happens to touch down on. Without legs
that means landing on an engine bell at single-digit metres per second and
hoping, which is not a feature, it is an absence. Legs are what makes the
second body in the game actually reachable.

What a leg set is, as data:

| field | meaning |
|---|---|
| count | 3 / 4 / 6 -- the ladder, chosen to suit the stage's diameter |
| mass | a table row per leg, so a 6-leg 3.75 m stage carries more of them |
| `rated_v` | the touchdown speed this stage survives, metres per second |
| `rated_tilt` | how far off vertical the contact may be before it is a wreck |
| deployed | stowed or deployed; deployment is a key, and deploying late is a wreck |

A leg set changes which of Part A3's touchdown numbers apply to the fragment
that lands, and that is all it does. There is no leg crush, no leg break, no
spring absorbing an impact, no per-leg load: the rigid body (Part V3) still
has one body and one touchdown test, and the leg row says which thresholds
that test uses. "Landing legs as physics" stays out of scope (Part D);
"landing legs as a rating" is what is in.

**What they cost, which is the whole trade.** Legs go on the stage that has
to survive, which is usually the last stage, which is the stage whose delta-v
the player wanted for something else. A 4-leg set is on the order of a tonne,
and a tonne bolted to the last stage is a large slice of that stage's delta-v:
an 8 t upper stage with 6 t of propellant at Isp 320 is 4,350 m/s, and the
same stage carrying a tonne of legs is 3,450 m/s. The Engineering decision
legs create is the real one -- a rocket that can land on Cheese is a rocket
that carried a tonne of landing hardware to Cheese -- and the delta-v card is
where the player sees the price before paying it.

**Legs and the air.** Deployed legs put drag area at the very back of the
fragment, which moves the drag centre aft and therefore *increases* the static
margin (Part A5): a fragment with its legs out flies more like a dart than the
same fragment with them stowed. That is not a bonus to be farmed -- it can be
exactly the wrong thing, because a dart held shield-first at orbital speed is
the configuration Part A4 says will cook the capsule. It is modelled rather
than special-cased, which is the point of having a static margin at all.

**Legs and the drawing.** On the sheet a deployed set is three or four pencil
strokes splayed outward from the bottom of the stage; a stowed set is the same
strokes folded against the hull. Same geometry as the legend icon, same
renderer as the thumbnail (Part B5). It is the one stage parameter that is
visible in the drawing rather than only in a dimension callout, which is worth
having for the parameter that decides whether the picture ends in a landing or
a crater.

**Legs and the notes.** The selected-stage card gains `legs: 4, deployed` and
the rated touchdown speed, because a part's rating is data and data belongs on
a card (D-37). What the notes never do is promise that the flight will arrive
inside the rating.

---

## Part B: Engineering (the blueprint designer)

Engineering is not an assembly building and it is not a menu. It is
a **drawing sheet** (D-25): blue paper, white pencil technical drawing, a
legend of parts down the left, the rocket in the middle, and a few
stuck-on notes on the right that say whether any of it will fly.

```
 ┌──────────────┬─────────────────────────────────┬───────────────┐
 │ PARTS        │ the blueprint                   │ NOTES         │
 │ the legend   │ blue paper, white pencil        │ post-it cards │
 │ scrollable   │ scroll up/down, auto-fit        │ selected stage│
 │ part icons   │ dimension callouts              │ whole rocket  │
 │ 32x32 pencil │ title block, bottom right       │ 168+424+208 px│
 │ 168 px       │ 424 px                          │ 208 px        │
 └──────────────┴─────────────────────────────────┴───────────────┘
```

800 logical px wide, 480 tall: 168 + 424 + 208. The middle column *is*
the screen; the two flanks are margins. Nothing here is a 3-D scene --
the whole screen is 2-D direct-565 drawing plus Hershey text (F-13), so
it costs no scene triangles, no scene lines, no camera and no PPA
upscale.

### B1 -- The sheet: paper, grid, pencil

- **Paper** is the frame clear: `pax_background(fb, BLUEPRINT_ARGB)`, a
  saturated drafting blue. It is the same CPU clear the island already
  pays for, so the paper is free; the *look* comes from what is drawn on
  it.
- **Grid**: a faint engineering grid, minor every 20 px and major every
  100 px, in a dimmer tint than the strokes -- about 45 + 9 segments for
  the whole sheet. It sells "drawing" for essentially nothing, and it
  gives the eye a scale to read the rocket against.
- **Pencil**: white-ish (`0xE8EEF5`, not pure white -- pure white on blue
  reads as a wireframe, off-white reads as a pencil) 1 px
  `direct_565_line` strokes. Optional sketch feel: a second stroke offset
  by one pixel at ~60 % brightness, which doubles the segment count and
  is the kind of thing to add only after 4.8 has measured the sheet.
- **Dimension callouts**: diameter arrows left of the stack, length
  arrows right, with extension lines, arrowheads and Hershey numbers
  ("2.5 m", "18 m"). These are the blueprint's *look* and its *editor*
  at the same time: the number you are changing with the ladder is the
  number drawn on the drawing, which is why the sheet replaces the
  separate "preview" the first draft of this plan asked for.
- **Title block**, bottom right of the sheet, like a real drawing:
  design name, revision, stage count, scale (`1:250`). It is also where
  the unnamed state is visible, so "Launch is disabled until named"
  (D-11) has somewhere to show itself.
- **Scale and scroll**: px-per-metre is chosen to fit the whole stack in
  the middle column, clamped so a 2 m capsule is never 3 px; past the
  clamp the sheet scrolls with the stage cursor, and the title block
  reports the current scale.

### B2 -- Keyboard-only, and how to still do it

No mouse, so the screen has a **focus ring** rather than hit targets:
PARTS ⇄ BLUEPRINT. NOTES is read-only and never takes focus. TAB (or the
shoulder button) cycles.

```
BLUEPRINT focus   up/down     stage cursor; the sheet scrolls to keep it visible
                  Enter       open the parameter editor -- the cursor's
                              stage, or the pod when the cursor is above
                              every stage
                  add / del   insert a stage below the cursor / remove it
PARTS focus       up/down     part cursor in the legend, which scrolls
                  Enter       insert that part as a stage below the cursor
PARAMS (Enter)    up/down     pick the parameter row
                  left/right  step that parameter's ladder
```

Editing a parameter is a **ladder**, never a numeric field: diameters
step through the four sizes, lengths through their steps, engines through
the PART[] table, counts through 1..N, and RCS, side boosters and the leg
set toggle through their own ladders. The
only typed text in the whole screen is the design name, and that text entry
already exists in the menu code (the world-name field).

There is no "fuel" knob: **fuel is the size of the stage** (D-10) --
length x diameter *is* the propellant load. That is what makes a ladder
work on a keyboard, and what makes the drawing honest: the sheet redraws
on the frame the ladder steps, so changing a diameter *is* watching the
rocket get fatter. The editor and the picture are the same object, which
is the whole argument for the blueprint layout.

### B3 -- What the legend contains

The legend is the visible face of the PART[] registry (Part V2), and it
lists **stage types**, not attachments:

| legend row | what it inserts |
|---|---|
| Command pod | the top of every stack; always present, never removable, not in the scrollable list. Enter on it opens its two options (RCS pods, parachutes) rather than inserting anything |
| Main stage | engine + propellant: the workhorse |
| Upper stage | a vacuum-optimised engine, thin walls, small diameter |
| Booster stage | solid: heavy thrust, no throttle, burns out on its own |
| Cargo / adapter section | no engine: payload, fuel-only tank, or a diameter adapter |

RCS, side boosters and landing legs are **stage parameters**, not legend
rows (D-26, extended by D-38).
They are properties of a stage -- a stage either has RCS pods or does not,
either has N boosters or does not, either has a leg set or does not -- and
putting them in the legend would mean a keyboard dance to attach one to a
stage, which is exactly the mouse-shaped interaction this screen cannot have.
Keeping them as parameters also keeps the legend five rows long, which fits
without scrolling for the first build and scrolls the same way later.

Each legend row carries a 32x32 pencil icon, and the icon is the part's
own geometry drawn at a fixed scale by the same code that draws the
sheet (Part V2's table, Part B1's strokes). No PNGs, no sprite sheets,
no asset pipeline: the legend is a legend because it is drawn in the same
language as the drawing.

### B4 -- The post-it notes

Three to five cards on the right, manila / pink / yellow fills with a
darker corner so they read as stuck-on paper against the drafting sheet,
each tilted a degree or two. They are the only warm colour on a blue
screen, which is deliberate: the sheet is the work, the notes are the
verdict.

| card | contents |
|---|---|
| Selected stage | delta-v, TWR at Bob's surface, burn time, wet and dry mass, propellant kg, RCS yes/no, booster count, legs and their rated touchdown speed |
| Whole rocket | total mass, total delta-v, liftoff TWR, stage count, part count |
| Warning (conditional) | appears only when triggered: first-stage TWR < 1 at the pad; total delta-v under the reach-orbit line; a stage that outlives its own stage separator |

Notes are recomputed from `rocket/budget` **on every edit**, not on a
timer and not on save. A post-it that updates when you press a key is
the feedback loop the builder runs on; F-12's trap -- plenty of delta-v,
a first stage that cannot lift -- is worth a card of its own for exactly
that reason.

The reach-orbit line is a measured number, not a guess: the ideal
impulsive cost from the pad to a 100 km circular orbit is **2,605 m/s**
(2,517 to raise the apoapsis, 88 to circularise, before the 174.9 m/s of
rotational help and before gravity and drag losses). The card's threshold
is the number block 6 measures in flight, not the ideal one.

What the notes do **not** do: predict survivability (D-37). Whether a design
comes home is a function of its mass distribution as much as its parts
(Parts A5, V4), of how fast it arrives, and of whether the player staged when
the stage was empty. That is the player's judgement, and it is not a number
that can be printed here. A "will this survive reentry" card would be read as
a guarantee, and the game would be blamed every time the guarantee was wrong
for a reason the card could not see.

The line between the two kinds of number is worth drawing exactly, because it
is what the notes *may* say. A **part's rating** is data and belongs on a
card: a shield's factor, a leg set's rated touchdown speed, an engine's
thrust and Isp, a stage's delta-v. **What the assembled rocket does with those
ratings on a trajectory the player is flying** is not data and does not
appear: not a stability verdict, not a heat verdict, not a survivability
verdict. Engineering says what your parts are rated for. It does not say that
you will stay inside the rating -- the HUD does that, in flight, where the
number can still be acted on (step 6.6).

### B5 -- One renderer, three sizes

`engineering/sheet` draws a stack into a rectangle at a given scale. That is the
only drawing function, and it is used for the Engineering sheet, the design-list
thumbnail in the Spaceport (Part N1), and the pre-launch confirmation. A
thumbnail is the same call with a smaller rectangle, so a design looks
identical everywhere it appears -- and the designs list becomes a wall of
little blueprints, which is the point of calling them blueprints.

### B6 -- What it costs

Engineering is the cheapest screen this game will have, and it should be
measured like one: `perf scene=engineering`, target **>= 40 fps**, with the cost
expected to be a few hundred `direct_565_line` calls plus text. Two
consequences worth stating now:

- **Half resolution does not apply here.** Nothing in Engineering goes
  through the 3-D pipeline or the PPA upscale, so the Graphics setting
  has nothing to halve (F-03 is the same lesson from the other side).
- **The sheet is not where the frame budget goes.** Flight and map are
  (Part G). If Engineering is ever slow, the suspicion falls on the drawing
  being redrawn per frame instead of cached into a layer, and 4.8 is what
  decides whether caching is needed at all.
- **Its music is not this screen's cost.** Engineering loops `sneaky_adventure.mp3`
  (Part N4), and the decoder is the engine's own task on core 1 (F-14), so the
  track costs the frame loop nothing. Building a rocket is a long session and it
  should sound like one.

### B7 -- What survives from the first draft

Save / Save as / Load design / Launch stay as they were, and the rule
that the preview was there to serve stays too: the drawing is drawn from
the same stack maths the physics uses (Part V3), so it cannot lie about
mass, height or what is tail-heavy. What changed is that the stage list,
the readouts and the preview collapsed into one thing -- a blueprint you
edit in place.

---

## Part F: flight

### F1 -- The physics tick

Fixed `dt_phys = 0.02 s` (50 Hz) at warp 1. Warp multiplies simulated
time, not step size: at warp `w` the frame advances `w * dt_frame` of
simulated time, which is covered by

- **coasting above the atmosphere:** analytic propagation, one call,
  whatever the warp (Part O3). Warp 1000 costs the same as warp 1.
- **thrusting or in atmosphere:** `ceil(w * dt_frame / dt_phys)`
  substeps, capped at `NISP_SUBSTEP_MAX` (200). The cap is what keeps a
  warp-1000 burn from eating a frame, and when the cap binds the sim
  drops the effective warp rather than the accuracy -- visible, honest,
  and better than a wrong orbit.

### F2 -- Control modes

| mode | what it does | how it is entered |
|---|---|---|
| **manual** | the player's keys are torque commands; nothing is stabilised | default |
| **SAS** (stability assist) | damped attitude hold: kills rotation, holds the current heading, and can lock to prograde / retrograde / normal / radial | `S` toggles, `K`/`H` pick the reference direction |
| **auto-guidance** | steers to the next maneuver node's burn vector, warps to the node, ignites, cuts off when the node's delta-v is spent | chosen from the map screen |

SAS is a PD controller on the attitude error with the RCS torque budget
as its authority limit; when the budget is exhausted the ship simply
stops responding, which is the honest failure mode and the one that
teaches the player to put RCS on a stage.

**The controlled body is always the capsule** (D-33). Keys, SAS, throttle,
staging and auto-guidance all act on whichever fragment contains the
capsule, and on nothing else. Debris is not controllable, has no SAS, and
is not what the HUD is showing -- but it is still simulated, still drawn,
and still visible in map mode, because watching your spent first stage
fall away below you is the feedback that teaches staging.

### F3 -- Staging, decoupling, boosters

One key advances to the next stage. On a stage boundary:

1. the current stage's engines stop and its propellant is discarded with it,
2. the separator fires (a mass change and a small separation impulse so
   the spent stage does not sit inside the live one),
3. mass, CoM, CoT, inertia and the drag centre are recomputed -- which is
   also where the static margin jumps, sometimes across zero (Part A5),
4. side boosters that are already empty have separated earlier, on their
   own burn-out, not on the stage boundary.

### F4 -- Warp

Warp levels `1, 5, 10, 100, 1000`, and 10,000 only above 100 km altitude
and outside any atmosphere. Warp is refused while thrusting below the
substep cap's comfort zone, and refused inside an atmosphere above warp
10 -- the two places where a big dt is a wrong trajectory rather than a
slow one.

### F5 -- Failure: a part burns, and the stack splits

When a part's heat reaches its limit, **that part goes** (D-32). Not the
rocket, not the stage -- the part, and the stack is cut at it:

```
capsule | sep | stage 1 | sep | stage 2 | sep | stage 3   stage 2 fails
        ->  capsule | sep | stage 1        (controllable: has the capsule)
            stage 3                        (debris: ballistic, still heated)
```

- **A fragment is a range in the same array.** A stack is already a list of
  parts along one axis (Part V2), so a fragment is `(first, last)` into it.
  Mass, CoM, CoT and inertia come from the same code that already recomputes
  them at a stage boundary (Part V4) -- a burn-through is a staging event the
  player did not ask for, and it reuses the staging path rather than adding a
  second one.
  The drag centre comes from the same pass, so every fragment has its own
  static margin (Part A5): the shed stage flies like the dart it has become,
  and the capsule fragment flies like whatever its own mass distribution says.
- **The failed part is gone, its neighbours are not spared.** The fragments
  are now exposed on a fresh face, so they keep heating at the new frontal
  area. A rocket that starts breaking tends to finish breaking, which is the
  right behaviour and needs no extra rule.
- **The capsule fragment is the mission.** It has the heat shield, the
  parachute and the control, so a mission that loses everything below the
  capsule is a survivable mission that lands under chute -- smaller, sadder,
  and still a landing.
- **The capsule itself failing ends the mission** where it happens, in the
  air, at whatever speed and altitude. That is a wreck in the save slot with
  the altitude it happened at, because "you lost it at 22 km doing 2,400 m/s"
  is the lesson.
- **Debris is capped** at `NISP_DEBRIS_MAX` (8) bodies, each a point mass with
  the same heat model, each allowed to fail once. The cap is a frame-budget
  constraint (Part G), not a physics one, and when it binds the oldest debris
  is dropped rather than the sim slowing down.
- **Nothing fails for reasons other than heat** in this plan. Engines do not
  overheat from burning, parts do not break on contact, and a stage does not
  tear itself off from too much thrust (Part D's out-of-scope list). One
  failure cause is a rule a player can learn; four is a simulation.

---

## Part M: the map and the maneuver planner

### M1 -- The map screen

A 2-D view of the current SOI's plane: Bob's disc, Cheese's disc and its
orbit, the ship, the current conic drawn as a polyline (sampled from the
elements, not integrated -- 128 samples per orbit is plenty and it is
exact), the predicted path through the patch chain beyond the SOI,
apoapsis and periapsis markers, and the node.

TAB toggles flight <-> map at any time; the map is not a pause, the ship
keeps flying while it is up (that is what makes "warp to the node"
possible).

Line budget: `SE_SCENE_LINE_CAP` is 4,096. The orbit polyline is 128,
Cheese's orbit 128, the predicted path 256, node markers and body
outlines a few hundred. There is room for everything here and this is
one of the few places in the project where the engine's cap is not the
constraint (Part G).

### M2 -- Manual nodes: the six axes

A node is a delta-v vector applied at a point on the current orbit, in
the orbital frame:

```
prograde / retrograde   along the velocity
radial in / out         perpendicular, in the orbital plane
normal + / -            perpendicular to the orbital plane
time - / +              slide the node along the orbit
```

Six axes, six keys, each key nudging one component by a step (and a
modifier for a bigger step). The node's vector is drawn as an arrow at
the node, and the resulting predicted orbit is drawn live, so the
editing is visible rather than arithmetic.

The resulting burn is then *executed* by the flight computer: warp to
the node, turn to the vector (or let auto-guidance do it), burn, cut off
when the planned delta-v has been delivered.

### M3 -- The auto-planner

TAB switches to the planner. The player picks a maneuver type and types
the target; the planner produces the same node object a human would have
dialed in, with the time and the vector chosen properly:

| maneuver | burn point | delta-v |
|---|---|---|
| **change apoapsis** | periapsis, prograde (raise) or the apsis that is nearest the target (lower) | vis-viva: `v = sqrt(mu (2/r - 1/a))`, `a = (r_p + r_a')/2` |
| **change periapsis** | apoapsis, retrograde (lower) or prograde (raise) | same |
| **change inclination** | the ascending or descending node, whichever is sooner | `2 v sin(di/2)` at the node, normal to the plane |

and each one reports **time to burn**, **delta-v required**, and whether
the current stage's remaining delta-v covers it. A maneuver the rocket
cannot do is refused with the number that says so, not attempted.

Transfer / intercept ("get to Cheese") is the next planner item and is
explicitly **not** in this plan's scope: it needs a phasing-angle
calculation and a targeting UI, and it is much better designed after the
three simple ones have been flown.

### M4 -- What the planner must never do

It must not silently change the orbit while the player is looking at a
predicted one, it must not plan a burn inside an atmosphere or below the
surface, and it must not plan a burn at a point the rocket cannot reach
because the orbit will have been changed by an earlier burn. Every
planned node is validated against the *current* conic before it is
offered, and `nispcheck` has a case for each refusal.

---

## Part N: menus, settings, saves

### N1 -- The menu tree, and where the game hangs off it

The shape is already the SynthMiner shape and is kept:

```
title bar   Play / Settings / Quit
  Play      the three slots
    a world   Play / Rename / Delete        -> becomes the Spaceport hub
  Settings  Controls / Graphics / Audio / Display / Livestream
pause       Resume / Save / Settings / Save and quit / Quit
```

The one change the game needs is what happens when a slot is opened. A
slot is not a world you walk around in; it is a **mission**, so opening
one opens the Spaceport:

```
spaceport   Continue mission...     (which mission, when the slot has more than one)
            Build a rocket          (Engineering, on an unnamed working design)
            Launch...               (which saved design)
            Designs...              (open / rename / delete a design)
            Rename / Delete         (the slot itself)
```

The hub lists **functions, never functions-with-arguments** (D-24). A row
that needs a parameter opens a picker for it -- "Launch..." then asks
*which design*, "Continue mission..." then asks *which mission* when the
slot has more than one in progress. A row whose parameter is already
unambiguous skips the picker and goes straight through. So the hub is a
fixed-length menu no matter how many designs or missions the player has,
which is what keeps it drawable with the same fixed-height rows as every
other screen.

Everything else -- the pause menu, the settings screens, the livestream
being a *setting* and never a title-bar item -- stays as it is (D-05).

### N2 -- Settings parity with SynthMiner

Same file (`/sd/nisp/settings.txt`), same `key=value` format, same
screens, same names where the concept is the same:

| screen | rows | status |
|---|---|---|
| Controls | every action, remappable, swap-on-conflict, reset to defaults | done |
| Graphics | Half resolution, view/zoom | half-res done; "view distance" becomes **map zoom** and the body-LOD distance |
| Audio | Music on/off + volume, Effects on/off + volume, track list + attribution | done (menu music is `brittle_rille.mp3`, looped; per-context playlists in Part N4) |
| Display | device brightness / screen settings via `se_hw.h` | done |
| Livestream | on/off, applied by `main.c` because it owns the framebuffer | done |

Rows this game will add, in the same style: **SAS by default** (start a
mission in stability assist or in manual), **units** (metres vs kilometres
on the HUD), and later **gyro look** once the look abstraction gets a
second source (the abstraction is already there in `game/input.h`).

Volume and brightness are the device's, not the game's, and `se_hw.h`
owns them -- same rule as SynthMiner, same reason.

### N3 -- What persists

| path | what |
|---|---|
| `/sd/nisp/saves/save<n>.bin` | the slot: the seed field (kept so an old slot still loads; the world itself comes from the fixed internal seed, **D-50**), the mission state, the flown rocket's design id, apoapsis reached, missions completed |
| `/sd/nisp/designs/<name>.nbp` | rocket blueprints, shared between slots |
| `/sd/nisp/settings.txt` | settings and bindings |
| `/sd/nisp/test/` | test-kit shots |

A save holds **state, not geometry**: a state vector, an attitude, a
design id, a mission log. The universe is regenerated from the body
table, so a save stays a sentence however big the game gets -- the
SynthMiner lesson, kept.

The save schema gets a version field before any of this is added to it
(D-23), because a slot that cannot be recognised is a slot a player
deletes.

### N4 -- Music: which track plays where

Music follows the **context**, and the context is a screen except in
flight, where it is the sim (D-28):

| context | track | on the card |
|---|---|---|
| Title, menus, Spaceport, settings | `brittle_rille.mp3`, looped | `music/menu/` |
| Engineering (the blueprint designer) | `sneaky_adventure.mp3`, looped | `music/engineering/` |
| Flight, **inside** the atmosphere (pad, ascent, reentry, touchdown) | **silence** | -- |
| Flight, **above** the atmosphere | `impact_lento.mp3`, looped | `music/space/` |

Silence in the air is a choice, not an oversight (D-29). Launch, ascent
and reentry are the loudest moments the game has and they are the moments
the engine is busiest: the MP3 decoder is its own task on core 1
resampling into a ring buffer, and clearing the music slot during
atmospheric flight shuts that task down (F-14). So the phase that needs
the most CPU is the phase that gets the least audio, and the payoff is
not a frame rate number -- it is that the only music in flight is the
music that means something, and the quiet while the air is still going
past you is the effect.

Rules that fall out of the table:

- **The boundary is the atmosphere, not the screen.** Map mode keeps
  whatever the mission's altitude says: orbit in map view is still
  `impact_lento`, a ship sitting on the pad in map view is still silence.
- **The switch is edge-triggered with a hysteresis band**: space above
  70 km, back to silence below 60 km. Without the band, a suborbital hop
  or an orbit whose periapsis dips into the air would tear down and
  restart the decoder every pass (F-14), which is audible as a stutter
  and expensive as a card read.
- **Swaps fade, they do not cut.** ~0.5 s down on the old track, source
  swap, ~0.5 s up on the new one, via `audio_mixer_set_music_volume()`.
  A hard cut between two recordings on a badge with no volume ramp reads
  as a crash.
- **The user's switch outranks the game's.** Audio > Music off means no
  music in any context; the context rule decides *which track*, the
  setting decides *whether*. The procedural generator stays the fallback
  for a card with no music installed, and it is not used as
  atmospheric-phase filler -- that phase has no music by design.
- **Attribution follows the track.** `music_attribution()` already reads
  the `.txt` beside the playing file in its own directory, so all three
  tracks are attributed correctly with no new code. With nothing playing,
  the Audio screen says so rather than showing the last track.
- **A playlist, not a file.** `se_mp3` plays every `.mp3` in the directory
  it is handed, so each context's directory is its playlist: a player can
  drop their own track into `music/space/` and it joins the rotation.
  The Makefile's stem lists grow to match (`MUSIC_MENU_STEMS`,
  `MUSIC_ENGINEERING_STEMS`, `MUSIC_SPACE_STEMS`) and `metadata.json` lists the
  same files by hand, as it already does.

---

## Part G: render budget

Measured on the badge with the current build (v2.7 engine, 2026-10-31):

| scene | avg fps | worst frame | phase split |
|---|---|---|---|
| `perf scene=title secs=20` | 25.3 | 46.1 ms | logo 6.2 ms, rast 19.8 ms |
| `perf scene=space secs=20` | 28.9 | 40.8 ms | rast 23.3 ms, wait 6.1 ms |

Before the logo was packed for the blit the title ran at ~11 fps with
`draw 58.7 ms` (F-01). That is the lesson to carry into the game work:
the cost here is **pixels moved**, not geometry, and the two places
pixels get moved for free are (a) drawing something at a size it does
not need and (b) drawing it through a general-purpose path when a
specialised one exists.

Space is cheaper than the island in geometry and more expensive in
overlay:

- **Bodies.** Bob fills a large part of the screen near the surface: a
  sphere tessellated by screen size, LOD ladder 96x48 / 48x24 / 24x12 /
  12x6, chosen from the body's projected radius. A full-disc Bob at the
  highest LOD is ~9,000 triangles, which is over `SE_SCENE_TRI_CAP`
  (4,096) -- so the cap is a real constraint here and the LOD ladder is
  not optional. Measured, not guessed (step 2.4).
- **Orbit lines and map overlays** are a few hundred line segments
  against a 4,096 cap: not a problem.
- **HUD text** is Hershey-vector text, which is fast (the user's own
  experience, and F-02 confirmed it here): the HUD is not the budget
  problem and should not be optimised before it is measured.
- **Half resolution** scales the *scene* through the PPA and leaves the
  HUD at full resolution. It helps the rasterising phase and does nothing
  for the overlay -- which is exactly why turning it on did not fix the
  title (F-03).
- **Heat and debris are sim cost, not render cost.** One float per part per
  substep over <= 30 parts is nothing; the debris cap (8 bodies) exists to
  bound the *physics* substep loop and the number of orbit lines map mode has
  to draw, not the framebuffer. Debris is drawn as the same primitives as the
  rocket, so it adds no new render path.

Budget targets, to be measured rather than asserted: **>= 30 fps** in
flight near the surface at half resolution, **>= 25 fps** in map mode,
**>= 20 fps** with a full-disc Bob and a predicted path on screen. The
profiler phases get extended for this game (`sim`, `phys`, `map`, `hud`
alongside the existing `rast`, `logo`, `overlay`, `wait`) so a slow frame
can be attributed rather than argued about.

Every number in this part is a **badge** number. The virtual graceloader
(F-17) runs the same build on this PC at 7.1-8.0 fps on the same two scenes,
and that is the only thing it can say about time: it is an RV32 interpreter,
not a P4, and nothing about its frame rate transfers. What the emulator
answers is *what* is drawn and *how much* of it -- shot hashes, primitive
counts, which phase eats a frame. What it answers about speed is nothing
(D-39). A target in this part is met on a badge, or it is not met.

---

## Part H: the host harness

Orbital mechanics and rocket arithmetic are the two parts of this project
that can be **provably wrong without anything looking wrong**, and they
are the two parts that are pure. So they get a host binary, exactly as
SynthMiner's `worldcheck` does:

```
tools/nispcheck.c     compiled with plain host cc, links only against main/sim and main/rocket
make check            hostpurity + nispcheck, and `make build` depends on it
```

**Host purity is enforced by a grep, not by convention:** no file under
`main/sim/` or `main/rocket/` may include an engine, BSP, FreeRTOS or
graceloader header, and `make check` fails if one does. That is what
keeps the harness honest as the code grows.

What it checks, and each of these is a case that has a hand-computed
answer next to it:

1. **The four KSP anchors** (F-11): 80 km circular orbit at 2,278.6 m/s
   and 1,875 s; stationary orbit at 2,863.9 km; semi-synchronous at
   1,581.8 km and 1,272.3 m/s; surface escape 3,430.5 m/s. If the body
   table or the element conversion is wrong, one of these fails.
2. **Kepler propagation round trip**: state -> elements -> propagate a
   full period -> state, equal to the start within a metre over 10,000
   periods, for e = 0, 0.2, 0.7, 0.95, and the hyperbolic case.
3. **Patched conics**: a Cheese transfer orbit enters Cheese's SOI, the
   patch chain exits it again, and the exit state matches a brute-force
   integrated reference to a stated tolerance.
4. **The transfer numbers**: from an 80 km circular orbit, a Hohmann
   transfer to Cheese's orbit is 860 m/s, takes 7.95 h to arrival, and
   circularising at 12,600 km costs 360 m/s more. These are the numbers a
   player will look up on a wiki and compare against, so they are in the
   test rather than in a comment.
5. **Stack maths**: mass, CoM, CoT, inertia, drag centre and static margin for
   a hand-written stack, against arithmetic done in the source -- including
   Part A5's two-row example, because a sign error in the static margin is
   invisible in every other number and flips the whole flight model.
6. **Delta-v budget**: Kostya's equation per stage for a three-stage
   stack, plus the TWR-under-1 case, plus the case where the total
   delta-v is enough and the first stage is not.
7. **The planner**: for each maneuver type, the node the planner produces
   must actually produce the requested apoapsis / periapsis / inclination
   when propagated -- i.e. the planner is checked by *flying its answer*,
   not by checking its formula.
8. **Refusals**: a burn below the surface, a burn inside the atmosphere,
   a maneuver the stage cannot afford, a node placed where the orbit has
   already been changed.

9. **Heating and failure** (Part A4, Part F5): four scripted
   `(altitude, speed, attitude)` profiles driven into `rocket/heat` directly --
   the profiles are inputs, so the heat model stays pure and host-testable
   while `flight/` integrates them for real. A nominal launch must leave every
   part under 50 % of its limit. A nominal reentry from a 100 km orbit, capsule
   facing the flow, must lose at most the unprotected lower parts and keep the
   capsule. A steep reentry, or a stack flying engines-first, must destroy the
  capsule. And the fourth is the dart: Part A5's burnt-out stack, flying
  shield-first at reentry speed, must also destroy the capsule -- it is the
  stable attitude and it is still fatal, which is the case that keeps the
  shield from ever being implemented as immunity (F-16, D-36).
  These four cases are what the `capacity` / `limit` / `shield`
   columns mean, and they are tuned here rather than by flying a badge.
10. **The split**: a stack with a part removed in the middle produces two
    fragments whose masses sum to the original minus the lost part, whose CoM
    and CoT match hand arithmetic, and of which exactly one contains the
    capsule.
11. **Coast, warp and the handover**: an analytic coast against an RK4 reference
    at three altitudes, a warp ladder that advances exactly the seconds it
    claims, the refusals (periapsis in the air, periapsis underground), the
    per-frame substep cap, and Bob -> Cheese -> Bob with the state handed over
    in the right velocity frame.
12. **Authority and SAS**: the kN m a stack can actually spend, what it drops to
    after a stage and on the pod alone, a 30 degree error spent at exactly the
    limit, a hold that converges and then holds, and the three ways a ship can
    have no authority at all (RCS switch off, keys only, RCS never built).
13. **Missions**: a scripted launch to space, the same launch without RCS falling
    over, circularising at apoapsis to within 1.3 km of round, a chute landing
    at 8.8 m/s, a Cheese landing that survives on legs and is a crater without
    them, a 250 x 20 km reentry that kills the capsule and a 250 x 60 km one
    that never gets hot, and a hover that is really a control test.
14. **The blueprint as an editor**: where an inserted stage lands (below the
    cursor, at index 0 when the cursor is on the pod, appended past the
    bottom, refused when full), what the pod's two options weigh and drag
    (1.00 / 1.10 / 1.20 t, 300 / 900 m^2, terminal speed by the square root
    of the area they add), that they survive `encode -> decode -> encode`
    byte for byte, that a file without them means the default, and that the
    pod's own pods are a control module of their own after everything below
    has gone.

Cases 1-6 and 9-15 are in `tools/nispcheck.c` today (471 checks). Cases 7 and 8
are written down before they exist, because they are the acceptance test for the
planner in block 8 -- the planner gets written when its test is already
described.

### The three harnesses, and what each one may be believed about

| Harness | Command | May be believed about | Must not be asked |
|---|---|---|---|
| host binary | `make check` | the orbital and rocket arithmetic | anything drawn, anything about time |
| virtual graceloader | `make emu-run`, `make emu-test` | what a screen looks like, whether a flow works, whether a scene's primitive counts changed, whether a save round-trips | frame rate, and every number in Part G (F-17, D-39) |
| badge | `make cycle TEST="..."` | the numbers Part G is built on, and every Accept line that says "on the badge" | how fast an idea gets a first look |

The middle row is new (F-17, D-39) and it is the one most of blocks 4, 6,
7 and 9 will actually be iterated in: the same `app.so`, the same install
path, the same `@@SR-*@@` records, no badge in reach.

The reference set, from the manifest run of 2026-11-01 (the previous set was
superseded by the fixed internal seed, which regenerates the starfield every
screen draws its panel over, and by the sheet's new bottom edge): `engineering`
`e66d7b09` (the same hash at t = 0 and t = 1.5 s, i.e. the sheet is a still
drawing), `spaceport` `d4c881f4`, `designs` `187ab4c4`, `settings` `96b6ddd7`,
`graphics` `fe500731`, `audio` `7b6eccff`, `display` `3b272a56`. Two runs of
the manifest in a row produced all eight hashes identically. The title is not
in the set and cannot be until step 10.5 (F-19).

---

## Part D: step-by-step plan with status tracking

**Status values:** `todo` / `in progress` / `done` / `blocked (why)` /
`skipped (why)`. The Notes column records the result, with links to
findings (F-n) and decisions (D-n). A block is not finished until its
**Accept** line is true, not until its last sub-step was typed.

### Already in place (the skeleton, committed 2026-10-31)

| Item | Status | Notes |
|---|---|---|
| SynthEngine3D as a submodule, v2.7 | done | `make engine` updates it; the app does **not** call `se_stream_reserve_permanent_memory()` -- that was a SynthMiner need only (D-02) |
| Engine splash on boot | done | `se_splash()` is the first thing `on_init()` does, as in SynthMiner |
| Logo on the title, from the SD card | done | `nullislandlogo_ball.png` beside `app.so`, packed once into RGB565 column runs (D-03, F-01) |
| Title bar Play / Settings / Quit; livestream under Settings | done | the user's correction, kept (D-04, D-05) |
| Save slots, `settings.txt`, key bindings | done | `/sd/nisp`, outside the install directory |
| Menu music `brittle_rille.mp3`, looped, licenses shipped | done | `make install` uploads mp3 + txt; `make push` sends `app.so` only (D-08, D-09) |
| Test kit (`perf`, `shots`), profiler phases | done | `results/` holds the runs; `perf scene=title` 25.3 fps, `perf scene=space` 28.9 fps |
| Virtual graceloader as a test harness | done | `nisp` added to `../virtual-tanmatsu-graceloader/Makefile`; `make emu-run` for the window, `make emu-test` for the suite -- one fixed command, manifest in `tools/emutests.txt`, records in `results/emulator/latest.log` (F-17, D-39, D-40, D-41) |

### The plan

| # | Step | Status | Notes |
|---|---|---|---|
| **0** | **Scaffolding (host only, no badge)** | | |
| 0.1 | This document: design parts L O A V B F M N G H, the status table, F-01..F-19, D-01..D-41 | done | 2026-10-31. The game brief arrived the same day; the numbers in Part O are derived from KSP 1's published parameters and cross-checked against four player-quotable anchors (F-11). Design parts A5 (aerodynamics) and V7 (landing legs) came from the same conversation. |
| 0.2 | `tools/nispcheck.c` + `make check` = `hostpurity` + `nispcheck`, and `make build` depends on it | done | 2026-10-31. `tools/hostpurity.sh` greps `main/sim`, `main/rocket`, `main/flight` for engine/BSP/RTOS includes; `make check` builds `build/host/nispcheck` with `HOSTCC`/`HOSTCFLAGS` read out of `CMakeLists.txt` rather than written down twice. |
| 0.3 | `sim/units.h` and `sim/bodies.{c,h}`: the BODY[] registry, Bob / Cheese / Frodo as rows | done | 2026-10-31. Sim in km / km-per-s / km^3-per-s^2 (D-15); `nisp_body_g0` converts to m/s^2 only where a TWR is quoted. Hill radii checked against SOIs: Cheese 3,170.9 km > 2,400, Bob 196,393 km > 84,160. |
| 0.4 | CMake: the new directories in `APP_SOURCES` as they arrive; profiler phases `sim`, `phys`, `map` added to `testkit/profile.h` | done | 2026-10-31. The six `main/sim/*.c` files are in `APP_SOURCES`; `PROF_SIM`, `PROF_PHYS`, `PROF_MAP` appended before `PROF_COUNT` so existing phase numbers do not move. |
| 0.5 | `nisp` in the virtual graceloader's app list, the imports its trap table was missing, its debug console backed by stdin, and one fixed command that runs the whole suite | done | 2026-10-31. `make emu-run` / `make emu-test` work from this repo; `perf` and `shots` both come back with records, and two identical runs of the manifest produce the same record chain (F-17, F-18, D-41). |
| | **Accept:** `make check` green, `make build` clean, `make verify` passes | done | 2026-10-31: `hostpurity: ok (13 files)`, `nispcheck: ok (79 checks)`, `app.so` links clean at 171,632 bytes of text. |
| **1** | **The sky, as maths (host only)** | | |
| 1.1 | `sim/kepler.{c,h}`: state <-> elements; elliptic and hyperbolic anomaly solvers; derived readouts (Ap, Pe, period, inclination, time to apsis) | done | 2026-10-31. One conic formula for ellipse and hyperbola alike, from `p = h^2/mu`; elements double, state float (D-17). Found and fixed F-20 here. |
| 1.2 | `sim/ephemeris.{c,h}` + `sim/patches.{c,h}`: circular body orbits, SOI entry/exit by conic-sphere intersection, the future-patch chain capped at 4 | done | 2026-10-31. Leaving a sphere is solved in one line (`acos((p/soi - 1)/e)`); entering one is a 128-sample scan plus bisection, because it is genuinely a three-body question. |
| 1.3 | `sim/atmosphere.{c,h}` + `sim/surface.{c,h}`: exponential density, drag coefficient and reference area from the stack, the pad at 0N 0E with its 174.9 m/s rotational velocity | done | 2026-10-31. `sim/surface.c` is body-relative -- it does not add the body's own 13.6 M km inertial position, which is what the pad's 174.9 m/s depends on. |
| 1.4 | `nispcheck` cases 1-4: the four KSP anchors, the 10,000-period round trip, the patched-conic transfer against a brute-force integrated reference, the Hohmann numbers | done | 2026-10-31, 79 checks. 80 km circular 2,278.6 m/s / 1,875.1 s; stationary 2,863 km; surface escape 3,430.5 m/s; Hohmann 860.2 m/s out, 7.947 h, 359.9 m/s; pad -> 100 km 2,605.4 m/s. The transfer case burns at a searched window (F-22) and agrees with a 0.01 s RK4 to 1.1 m. |
| | **Accept:** every case in Part H 1-4 passes, with the tolerances printed | done | 2026-10-31: `nispcheck: ok (79 checks)`, every tolerance printed next to the number it allows. |
| **2** | **The sky, on the badge** (replaces the island) | | |
| 2.1 | `game/scene.c` rewritten: bodies as lit spheres, camera-relative submission in metres (D-16), LOD ladder by projected radius | done | 2026-11-01. `main/game/scene.c` (705 lines) draws Bob, Cheese and Frodo as lit spheres from the rows `sim/bodies.c` keeps, at the positions `sim/ephemeris.c` puts them. Ladder is 6 rungs, 40 rings x 28 segs down to 6 x 6, chosen by projected radius; the whole scene runs at 66 triangles on the title and 221 in a chase view, against `SE_SCENE_TRI_CAP` 4,096. |
| 2.2 | The sun as a direction: `normalize(Frodo - camera)` fed to the engine's directional light every frame | done | 2026-11-01. Frodo is drawn as an emissive disc at the bottom rung only to give the eye a place to put the light; the lighting itself is `se_light.h` directional, recomputed per frame from the camera. |
| 2.3 | Starfield kept; the title screen becomes the ship drifting past Bob with Cheese in the frame, logo and wordmark over it | done | 2026-11-01. `nisp_camera_title` is a closed-form drift, so the screen is a pure function of the clock; the logo is decoded once at start (940x937 -> 341x340 RGB565 column layer, 252 ms) and never rescaled per frame. The title's own hash still moves on every rebuild because it prints the build stamp (F-19); the menu screens reproduce exactly (`011bbebf` settings, `ea95f026` display). |
| 2.4 | Measure: `perf scene=title` and `perf scene=space`, phase split recorded here | done | 2026-11-01, in the emulator: title 30.39 fps / 66 tris (logo 6.0 ms), space 5.40 fps / 221 tris with `rast` 152.6 ms. The title is comfortably above the 25 fps baseline. The space number is the interpreter, not the badge (D-39) -- the badge run is still owed. |
| 2.5 | Delete the island: the heightmap, the obelisk, the beacons, `BENCH_SEED`'s island meaning | done | 2026-11-01. Gone from `main/game/scene.c`; `main/game/scene.h:8` says so out loud -- "no heightmap and no island". "Landing" and "seeing the ground" now read the same radius test as `sim/surface.c`. |
| | **Accept:** >= 25 fps with Bob full-disc and Cheese in frame; the island is gone | done | 2026-11-01: the island is gone, and the title runs 30.39 fps in the emulator. The badge number is still open (D-39) -- the emulator is not a speedometer. |
| **3** | **The rocket, as maths (host only)** | | |
| 3.1 | `rocket/parts.{c,h}`: the PART[] registry -- capsule, four engines, the solid booster, the cargo/adapter section, RCS, legs, separator; propellant density and structural fraction as constants | done | 2026-11-01. Four diameters (1.25 / 1.875 / 2.5 / 3.75 m), sixteen lengths (0.625 -> 10 m in 0.625 steps), propellant at 400 kg/m^3. Five rows (POD / MAIN / UPR / BST / CRG), five engines (Flea 30 kN 240 s, Relion 60 / 320, Skipper 180 / 270, Mainsail 300 / 290, solid Booster 400 / 180 sized by its own tank), three leg sets, RCS at 0.10 t and 15 kN m, separators at 0.05 t. `capacity_per_t` is the one column that was tuned rather than chosen (F-23). This table *is* the Engineering legend (Part B3). |
| 3.2 | `rocket/stack.{c,h}`: blueprint -> total mass, per-stage mass, CoM, CoT, inertia, drag centre, static margin; recomputed on stage separation and as fuel burns | done | 2026-11-01. `nisp_stack_build` lays out `[pod, sep0, body0, booster0, legs0, sep1, body1, ...]` on an axis measured from the nose; one pass gives mass, CoM, CoT (at `part->x1`), the geometric drag centre, transverse inertia and `static_margin_m = cp_x - com_x`. `com_bias` (0.45 for the pod, 0.5 elsewhere) is what makes a lone capsule shield-heavy and therefore a dart (F-16). `nisp_stack_recompute` is the single re-entry point for burning, staging and splitting. |
| 3.3 | `rocket/budget.{c,h}`: delta-v per stage, TWR, burn time, total | done | 2026-11-01. Two phases per stage: side boosters burn with the stage and shed their own dry mass mid-burn, then the core finishes (D-42). `reach_orbit_ok` is the 2,605.4 m/s figure from Part O5. |
| 3.4 | `rocket/design.{c,h}`: `/sd/nisp/designs/<name>.nbp`, the design list, rename, delete | done | 2026-11-01. NBT through `se_save.h`, list / rename / delete, name clamped to 16 characters, and the encode/decode path compiled on the host under `SM_HOST` so case 6 can prove the round trip byte-identical. Shared between slots, not inside a slot (Part N3). |
| 3.5 | `nispcheck` cases 5-6: the hand-computed stack, the delta-v budget, the TWR-under-1 trap | done | 2026-11-01. Case 5's hand table: CoM 6.3333 m, CoT 12.0 m, static margin -0.1905 m loaded and +2.4762 m burnt out, inertia 99.879 t m^2 wet and 23.176 dry. Case 6: 136.727 t at the pad, TWR 1.044, bottom stage 3,289.2 m/s against a 0.002 s stepped march that contains no rocket equation, 209.4 s burn, mixed-stage Isp 227.14 s, and the F-12 trap (enough delta-v, no liftoff). |
| 3.6 | `rocket/heat.{c,h}`: per-part accumulation, cooling, `temp >= 1` failure, the capsule shield factor | done | 2026-11-01. `rate = K rho v^3 area expose / capacity` with `K = 1.5e-11`, `expose = 1 - shield * facing`, and a 30 s cooling time constant, so heat is accumulated and not a speed threshold. Capacity is `capacity_per_t * (dry + propellant)` fixed when the stack is built (F-23). Separators carry mass and no thermal capacity (F-24); a booster group presents n cylinders (F-25). One float per part (Part A4). Pure, so the thresholds are tunable on a host. |
| 3.7 | `nispcheck` cases 9-10: the four heating profiles, the split arithmetic | done | 2026-11-01. Four profiles on one launcher: throttled launch peaks 0.125 (pod) to 0.329 (stages); nominal reentry 1.275 / 1.235 / 1.230 for the unshielded stages and 0.486 for the pod; engines-first 6.849 for the pod; the burnt-out dart 1.996 for the pod, shield-first and still fatal (F-16). `capacity_per_t` settled at POD 0.053 / MAIN 0.089 / UPR 0.161 / BST 0.087 / CRG 0.089 (F-23). Case 10: the split conserves mass, leaves the pod fragment with no lit engine, and gives each fragment its own CoM. |
| | **Accept:** Part H 5-6 and 9-10 pass; a named design round-trips through the file byte-identically; a nominal launch heats nothing past 50 % | done | 2026-11-01: `nispcheck: ok (196 checks)`, `hostpurity: ok`, `app.so` links clean. Design encode/decode round trip checked in case 6. Nominal launch peaks at a third of the limit on every part, and a nominal reentry still kills the stages that have no shield. |
| **4** | **Engineering** | | |
| 4.1 | `engineering/builder.{c,h}`: the working design in RAM, the stage cursor, add / remove stage, the PARTS legend and its scroll | done | 2026-11-01. `main/engineering/builder.c` (482 lines). Focus ring is three stops -- `NISP_FOCUS_PARTS` ⇄ `NISP_FOCUS_SHEET` ⇄ `NISP_FOCUS_ACTIONS` -- cycled by TAB (D-45); NOTES never takes focus, as planned. Columns 168 / 424 / 208 px across the 800 px screen, 22 px header, body from y 44. |
| 4.2 | Parameter ladders on the selected stage: diameter, length, engine, engine count, RCS, side boosters, leg set | done | 2026-11-01. Nine rows -- diameter, length, engine, count, RCS, boosters, booster length, legs, Remove. The editor is a sub-mode of the BLUEPRINT stop: ENTER opens it, ESC or TAB closes it, so it is a ladder over the drawing rather than a fourth focus stop. Every ladder step calls `nisp_stack_build` and `nisp_budget_compute`, and the sheet redraws in the same frame. |
| 4.3 | `engineering/sheet.{c,h}`: drafting-blue paper, minor/major grid, white pencil strokes from the stack maths, dimension callouts, title block, stowed and deployed leg strokes | done | 2026-11-01. `main/engineering/sheet.c` (369 lines). Paper `0xff1d4e89u`, pencil `0xffe8eef5u` with a dim variant for callouts, minor grid `0xff2a5f97u`, major `0xff3d75adu`, mark `0xfffff0c0u`. Scale is clamped to 4..26 px per metre so a 6-stage stack and a 2-stage one both read. Direct-565 runs only -- no `scene_line`, no 3-D submission (F-13, D-27). |
| 4.4 | `engineering/notes.{c,h}`: the post-it cards, recomputed from `rocket/budget` on every edit | done | 2026-11-01. `main/engineering/notes.c` (273 lines). Manila stage card `0xffefd7a6u`, pink total card `0xfff0b6c0u`, yellow warning `0xfff6e27cu`, each a sheared `direct_565_tri` card with a folded corner and ink `0xff2b2418u` text. Delta-v, mass and TWR per stage and in total; the warning card appears for a TWR under 1 and for a design that cannot reach orbit. Ratings only -- no survivability verdict (D-37). |
| 4.5 | `engineering/icons.{c,h}`: PART[] -> 32x32 pencil icons from the same geometry the sheet draws | done | 2026-11-01. `main/engineering/icons.c` (72 lines), `NISP_ICON_PX 32`, drawn with the same stroke routine as the sheet. No image files, no sprite sheet. |
| 4.6 | Save / Save as / Load design / Launch, with **Launch disabled until the design is named** | done | 2026-11-01. Save and Launch are both refused while the design is unnamed, and launching writes the blueprint to the card before the mission starts, so "saved before launch" is enforced by code rather than by the player remembering (D-48). The unnamed state shows in the sheet's title block. |
| 4.7 | The Spaceport hub screen, hung off the existing slot menu (Part N1) | done | 2026-11-01. Seven rows -- Continue mission, Build a rocket, Launch..., Designs..., Rename, Delete, Back -- with the pickers as their own screens (`SCR_LAUNCH`, `SCR_DESIGNS`, `SCR_DESIGN`) carrying 34x34 blueprint thumbnails. Opening a world opens its hub and launches nothing (D-49); the hub is a `main.c` mode (`MODE_WORLD`), not a menu detail. |
| 4.8 | Measure: `perf scene=engineering`, and decide there whether the sheet needs caching into a layer | done | 2026-11-01. `perf scene=engineering`: 29.76 fps average (27.3..32.7), 66 tris, `rast` 0 -- the cost is entirely `draw` at 9.9..13.6 ms, and the frame is identical at t=0 and t=1.5 s (`51270ab0`), so nothing is being recomputed per frame. No caching into a layer yet. The >= 40 fps target is unmeasured on the badge (D-39). |
| 4.9 | `music/engineering/` playlist: `sneaky_adventure.mp3` looped while Engineering is up, back to the menu track when it closes; Makefile stem lists and `metadata.json` extended | done | 2026-11-01. `MUSIC_ENGINEERING` is a context in `main/audio/music.h` alongside NONE / MENU / SPACE, set in `open_engineering()` and restored to MENU on the way out. `sneaky_adventure.mp3` and its license are in `music/engineering/` and go up with `make install`. |
| 4.10 | The pod's two options: RCS pods on/off, 1..3 parachutes, edited from the sheet and drawn on it | done | 2026-11-01. `pod_rcs` / `pod_chutes` in the design, the stack and the file; `nisp_pod_mass_t` and `nisp_chute_cd_a` in `parts.c` so the cost is one place; case 14 measures 1.00 / 1.10 / 1.20 t and 27.87 / 16.59 m/s. The decoder bug this exposed is F-38 (D-51). |
| 4.11 | The first badge hand-test of Engineering: four fixes and the fixed internal seed | done | 2026-11-01. The pod was not editable, the name dialog could not be closed, an inserted stage landed above the cursor, and the sheet's bottom edge collided with the actions. All four fixed on the badge report; `NISP_WORLD_SEED` replaces the typed seed (D-50, D-52, F-37, F-38). |
| | **Accept:** build a rocket on the badge that Part 3's budget says can reach 100 km, name it, launch it -- and the screen reads as a blueprint, not a menu | in progress | The blueprint half is settled, on hardware: the user built, named and edited rockets on a badge and the screen reads as a drawing rather than a list. What that test produced was 4.10 and 4.11 rather than a pass. Still owed: the >= 40 fps measurement on the badge (D-39), and a rocket that actually reaches 100 km from a badge launch, which needs block 6's HUD to fly. |
| **5** | **Flight physics** | | |
| 5.1 | `flight/physics.{c,h}`: rigid-body state, quaternion attitude, velocity-Verlet at `dt_phys` 0.02 s | done | 2026-11-01. `main/flight/physics.c` (904 lines) and `main/common/quat.c` (164 lines), `NISP_PHYS_DT 0.02`. One body per fragment (D-22); a split goes through the staging path, not a second code path (D-32). |
| 5.2 | Thrust at CoT, `(CoT - CoM) x F` torque, staging, separators, booster burn-out and separation | done | 2026-11-01. Thrust is applied at the stack's CoT and the torque is the lever arm crossed with it; CoM and CoT are recomputed every tick as propellant drains, so a stage that is half empty pushes differently than the same stage full. Side boosters burn out on their own clock and shed themselves mid-burn (F-25, D-42). |
| 5.3 | Drag and aero torque from `sim/atmosphere` and the stack's static margin, terminal velocity, the parachute | done | 2026-11-01. The weathervane torque is `q * CdA * static_margin`, capped so its natural frequency fits the tick (F-30), plus rotational damping so it settles instead of swinging forever (F-32). Chute: 300 m^2 sized backwards from the 10 m/s it has to deliver (F-29), gated at q >= 40 Pa and v <= 300 m/s. Case 13's chute drop settles at 8.806 m/s terminal from either 1,500 m or 5,000 m. |
| 5.4 | SOI switching in flight; analytic coasting above the atmosphere, integration while thrusting or in air | done | 2026-11-01. Handover converts velocity into the new body's frame, parent-relative (F-35). Case 11's Bob -> Cheese -> Bob round trip comes back 0.000487 km and 0.000000 km/s from where it started. |
| 5.5 | `flight/warp.{c,h}`: warp ladder, substep cap, the refusals (thrusting, in atmosphere) | done | 2026-11-01. `main/flight/warp.c` (114 lines). Ceiling 10 in air, 100 while thrusting or steering, 10 when the next periapsis is under the air (F-34), 1,000 below 100 km, 10,000 above. A coast is verified against a 0.01 s RK4 reference to 0.015 km, and a capped frame costs 4 s of ticks. |
| 5.6 | RCS torque budget + SAS (damped attitude hold, reference directions) | done | 2026-11-01. `main/flight/control.c` (153 lines). Authority is the RCS budget of the modules actually attached and nothing else: 45 kN m for a three-stage stack, 30 after a stage, 15 on the pod alone. SAS commands a rotation rate and chases it (F-36): a 30 degree error takes 629 frames at 10.5 s, peaks at 0.0862 rad/s -- exactly the authority limit -- and does not overshoot. |
| 5.7 | Leg deployment, per-fragment touchdown thresholds (`rated_v` / `rated_tilt`), impact, mission outcome | done | 2026-11-01. A leg set selects the rating; it adds no contact mechanic (D-45). Case 13 lands on Cheese at 11.007 m/s with legs and wrecks the same stack without them. Hovering is a control problem, not a landing problem: 3.983 TWR at ignition with RCS and a hold, and it drifts with either one missing. |
| 5.8 | Music context switch: silence inside the atmosphere, `music/space/` above it, edge-triggered with the 60/70 km hysteresis band and a 0.5 s fade | done | 2026-11-01. `NISP_MUSIC_SPACE_KM 70` / `NISP_MUSIC_AIR_KM 60` in `main/flight/mission.c`, edge-triggered off the same `nisp_atm_density` the physics uses. Silence on the pad and through the whole atmosphere; `impact_lento.mp3` only above the band. |
| 5.9 | Heat in the tick: `rocket/heat` fed `rho`, `v_rel` and the attitude every substep; a failed part cuts the stack through the staging path | done | 2026-11-01. Case 13's too-steep deorbit burns the pod and the mission ends in the air; the same flight with the stages shed survives. A burnt-out stage left attached makes the stack a dart and still cooks the capsule (case 10, F-16). |
| 5.10 | Debris: ballistic bodies, capped at `NISP_DEBRIS_MAX`, each still heated and each allowed to fail once | done | 2026-11-01. `NISP_DEBRIS_MAX 8` in `main/flight/mission.h`, oldest retired first. Debris is never controlled, so it coasts whenever it is free (F-31) and is heated on the same path as the ship. Drawn in the chase view; map mode arrives with Part 7. |
| | **Accept:** a scripted launch reaches a stable 100 km orbit; a planned deorbit lands under chute on Bob and survives; a Cheese landing survives **on legs** and the same stack without legs wrecks; a too-steep deorbit loses the pod and the mission ends in the air; a reentry with the burnt-out stage still attached is stable, dart-shaped, and still cooks the capsule | done | 2026-11-01, host-scripted (`nispcheck` case 13, 383 checks overall): a 61.228 t launcher at 1.499 TWR spends 7,697 m/s over 150.1 s to reach space, circularises with a 127.6 m/s burn at apoapsis into an orbit 1.269 km round at a 299.1 km periapsis, comes down under a canopy at 8.806 m/s, and lands on Cheese at 11.007 m/s -- survivable on legs, fatal without them. The dart case is case 10's, measured rather than asserted. |
| **6** | **HUD and controls** | | |
| 6.1 | Orbital readouts: altitude, surface and orbital speed, Ap / Pe, period, time to Ap / Pe, inclination, current SOI, warp factor | done | 2026-11-01. `main/game/hud.c` draws both columns: left is ALT / LOCAL / SPEED / V AIR or V GND / Q / MASS / TWR / MARGIN, right is APO / T+AP / PERI / T+PE / PERIOD / INCL / SOI / WARP, eight rows each at a fixed pitch so the numbers do not move when the flight changes. `nisp_time_to_apsis` feeds the two countdowns (`flight/physics.c`); LOCAL is `nisp_solar_time` coloured by `nisp_sun_elevation` (F-40). The clock is T+ from `mission.t_launch`, not the absolute sim clock. |
| 6.2 | Attitude indicator: prograde / retrograde / normal markers, the burn vector when a node is armed | in progress | 2026-11-01. PRO / RET / NRM are projected through the engine's own pinhole (`hud.c` `view_at` / `project_dir`), the camera basis rebuilt from yaw / pitch / roll with `se_scene.c`'s formulas and the local frame mapped as (east, up, -north) exactly as `game/scene.c` maps its geometry -- so a marker lands on the scene that was drawn with them. Behind the eye: not drawn. Exactly to the side: no finite projection, so it is placed along its screen bearing and clamped to the edge of the sky, dimmed, because "that way, further out" is the reading. Measured in the emulator on the bench, where the nose is along the velocity: PRO at the pinhole centre (400, 256), NRM clamped to the right edge, RET absent. The burn vector waits on block 7's nodes. |
| 6.3 | Throttle, staging, warp, SAS keys, all of them bindings in Settings -> Controls | done | 2026-11-01. KSP-like defaults -- WASD plus QE steer, L-Shift / L-Ctrl throttle, R-Shift boost, SPACE stage, X cut, C chute, G legs, T SAS, R RCS, M map, F3 / F4 warp, TAB focus, PgUp / PgDn time -- and 30 bindings in five rebinding sections (Flight 15, Map and warp 5, Engineering 2, Camera 4, System 4), reached Settings -> Controls -> section -> key (`ui/menu.c` `SCR_CONTROLS` / `SCR_KEYS`, D-54). `input_init` logs the per-section counts and warns about any action with no section, which is what catches a new action that forgot to be rebinding-able. |
| 6.4 | Mode indicators: SAS on/off and its reference, auto-guidance armed, delta-v remaining on the armed node | in progress | The status line reads `stage 2/3  SAS prograde  RCS on  chute  legs`: the mode, its reference, and what has already been let go, all in words. Auto-guidance armed and the armed node's delta-v are 8.3's, and there is no node to arm until block 7. |
| 6.5 | Heat readout: the hottest part as a bar against its limit, named when it passes 70 %, and a fragment count when the stack has split | done | 2026-11-01. The bar is off the screen below 2 % of the limit; past 70 % its value becomes `62% Main stage`, from `readout.heat_part` through `nisp_part_row(...)->name`, because which part is going is the part a player can still act on. The status line gains `N loose piece(s)` whenever `mission.debris_count` is non-zero (Part F5). |
| 6.6 | Stability readout: the static margin as a signed number in metres, with the drag centre and CoM marked on the attitude indicator | done | 2026-11-01. `MARGIN` is the signed metres; the picture is a nose-to-tail strip under the left column -- a square for the mass, a mark pointing down for the drag centre, one pointing up for the thrust, each in its own colour -- drawn only while there is air. A strip rather than marks on the ship: the chase camera is behind and above it, where marks on the stack would sit under the reticle and behind the silhouette (D-37's teaching still happens, as a pair of marks crossing). |
| | **Accept:** every number a flight needs is on screen, and every key that changes it is remappable | in progress | 2026-11-01, host + emulator: `nispcheck: ok (471 checks)`, `make emu-test` 12/12 ok, the flight bench still 221 triangles with the whole HUD on top of it, and two new screen references added to the manifest -- Controls `cda2f879`, Keys `c96e5467`. Owed: the badge run, which is the only frame rate that means anything for a HUD (D-39), and a look at the stability strip, which the bench never draws because the bench is above the air. |
| **7** | **Map mode and manual nodes** | | |
| 7.1 | `map/map_view.{c,h}`: TAB to map, the current SOI's plane, Bob and Cheese as discs, Cheese's orbit, the ship | todo | The ship keeps flying while the map is up. |
| 7.2 | `map/orbit_draw.{c,h}`: the conic as a polyline sampled from the elements (128 samples), plus the predicted path through the patch chain | todo | |
| 7.3 | `map/node.{c,h}`: node placement along the orbit and the six axes (prograde, radial, normal, time) on six keys with a big-step modifier | todo | |
| 7.4 | Execute a node: warp to it, align to its vector, burn, cut off at the planned delta-v | todo | |
| | **Accept:** a manually dialed node raises apoapsis to the dialed value within 1% when executed | todo | |
| **8** | **The auto-planner** | | |
| 8.1 | `map/planner.{c,h}`: change apoapsis, change periapsis -- burn point, delta-v, time to burn | todo | |
| 8.2 | Change inclination, at the sooner node, `2 v sin(di/2)` | todo | |
| 8.3 | Auto-guidance: steer to the planned vector, execute, cut off | todo | |
| 8.4 | Affordability and refusals: below the surface, inside the atmosphere, more delta-v than the stage has | todo | Refused with the number that says so (Part M4). |
| | **Accept:** Part H 7-8 pass on the host; on the badge, "apoapsis 150 km" from a 100 km orbit produces a burn that makes it so | todo | |
| **9** | **Mission lifecycle and persistence** | | |
| 9.1 | Save schema v2: mission state (body, state vector, attitude, stage index, fuel, **which parts of the design are still attached**), design id, mission log | todo | Versioned before the fields are added (D-23). A saved mid-flight mission may be a fragment, so the save says which range of the design is still flying. |
| 9.2 | Launch flow from the pad; the ship on the mount; nothing to count down | todo | |
| 9.3 | Landing, recovery, wreck (from touchdown **or** from the capsule burning up in flight), and what the slot remembers afterwards | todo | A wreck records where it happened -- altitude and speed are the lesson (Part F5). A Cheese touchdown is the case legs exist for (Part V7). |
| 9.4 | Design library: list, open, rename, delete, from the Spaceport | todo | |
| | **Accept:** title -> Spaceport -> build -> name -> launch -> orbit -> deorbit -> land -> save -> continue, with no step missing | todo | |
| **10** | **Test kit and regression** | | |
| 10.1 | Content scenes: `title`, `orbit`, `launch`, `transfer`, `map`, `engineering` -- each scripted, each a pure function of the clock | todo | A scripted flight is the only way `shots` can mean anything here (the existing bench flight is the pattern). |
| 10.2 | `make testrefs` / `make testcompare` references committed | todo | |
| 10.3 | `perf` scenes for flight and map, recorded in Part G | todo | |
| 10.4 | Host/device parity: the same sim numbers from `nispcheck` and from the badge's `perf` run, compared; and the same `shots` hashes from the emulator and from the badge | todo | float32 both sides, so this should be exact rather than close. If the emulator's hashes match the device's, the emulator carries the visual regression suite and the badge carries the timing one (F-17). |
| 10.5 | Make the shot set reproducible: freeze the build stamp while a test is running, or keep the title out of the shot set | todo | The title draws `APP_GIT_HASH  APP_BUILD_TIME`, so its framebuffer hash changes on every rebuild while nothing about the screen did (F-19). This blocks a committed `testrefs` reference for any screen that prints the stamp, on the badge as much as on the emulator. |
| | **Accept:** `make testrun` green on the badge; references in the repository | todo | |
| **11** | **Performance and polish** | | |
| 11.1 | Profiler phases for `sim`, `phys`, `map`, `hud`; a measured budget table in Part G | todo | |
| 11.2 | LOD ladder and star count tuned against the measured budget | todo | |
| 11.3 | Settings rows this game needs: SAS by default, HUD units, map zoom (replacing the island's view distance) | todo | |
| 11.4 | README and this document brought to the state the code is actually in | todo | |
| | **Accept:** Part G's targets met, or the shortfall written down with the numbers that explain it | todo | |

### Explicitly out of scope for this plan

Not deferred-by-accident; excluded on purpose, each one its own future
document if it ever happens:

- science points, money, contracts, achievements, a tech tree
- crew / kerbals, seats, IVA, landing legs **as contact physics** -- legs exist
  as a touchdown rating (Part V7, D-38); a leg that crushes, snaps or absorbs
  an impact does not
- docking, part collisions, wobble, and structural failure **from loads** --
  failure from heat is in (Part F5); failure from thrust, contact or bending
  is not
- more bodies, other star systems, transfer/intercept planning
- atmosphere rendering (clouds, sky colour, reentry glow), terrain heightmaps
- multiplayer, replays, mission editor

---

## Part E: findings and decisions log

### Findings

- **F-01** 2026-10-31: **the title screen's cost was the logo, and it was a
  layout problem, not a size problem.** The first measurement put the title at
  ~11 fps with `draw 58.7 ms` for a 341x340 artwork. The artwork was being
  decoded once and then blitted as ARGB8888 through `pax_draw_image()` --
  per-pixel alpha, per-pixel addressing -- every frame.
  What it is now: packed at load into the framebuffer's own RGB565 halfwords,
  **column-major**, each column trimmed to the rows the artwork touches, with
  its fully opaque middle recorded as a run. Drawing is 341 contiguous sweeps:
  plain halfword stores for the middle, a blend only for the soft edge, nothing
  at all for the transparent corners. 91,734 pixels in spans, 90,910 of them in
  opaque runs, 824 that actually need a blend. **6.2 ms**, and the title went
  11 -> 25.3 fps.
  The general rule for this game: the expensive thing is a general-purpose
  drawing path used where the data already knows how it wants to be drawn.

- **F-02** 2026-10-31: **Hershey vector text is not the bottleneck.** The user
  said so from experience across several games; the measurements agree (the
  title's text overlay is a few hundred pixels and does not register against a
  19.8 ms raster). Text is therefore *not* a thing to optimise here before it
  is measured, which matters because a HUD-heavy game invites exactly that
  guesswork.

- **F-03** 2026-10-31: **half resolution did not fix the title, and the reason
  is which pixels it scales.** Half-res renders the *scene* into a half-size
  layer and the PPA scales it up; the logo, the wordmark and the HUD are drawn
  at full resolution on top of the result. Turning it on while the cost was a
  full-resolution 341x340 blit changed nothing, which is what it should have
  done -- and is why the profiler has a separate `logo`/`overlay` phase outside
  `rast` rather than inside it.

- **F-04** 2026-10-31: **after the app exits, the badge is in the launcher in
  debug mode**, so the next badgelink command fails for a reason that looks
  like a bug in the app. `make mode_badgelink` first (BadgeLink mode shows as
  `16d0:0f9a`, debug as `303a:1001`). A second `mode_badgelink` that fails with
  `could not open port /dev/ttyACM0` usually means the first one already
  worked -- `make mode_badgelink` does not always succeed on the first try.

- **F-05** 2026-10-31: **the engine's MP3 source plays a directory, not a
  file.** `se_mp3.c` enumerates every `.mp3` it is handed and cannot be pointed
  at one track, which is why the menu's track lives in `music/menu/` while the
  rest of the pool sits in `music/`. A card with no music falls back to the
  procedural generator rather than to silence.

- **F-06** 2026-10-31: **v2.7's only API change is a SynthMiner-specific one.**
  The engine moved `bf39a61` -> `f21bc09` for `se_stream_reserve_permanent_memory()`;
  nothing in this app calls it. The memory reservation for streaming was a
  SynthMiner requirement (its `main/main.c`) and is **not** a general engine
  requirement, so it is not copied here.

- **F-07** 2026-10-31: **the baselines, with the phase split.** `perf scene=title
  secs=20`: 25.3 fps average, worst frame 46.1 ms, logo 6.2 ms, raster 19.8 ms,
  507 frames. `perf scene=space secs=20`: 28.9 fps, worst 40.8 ms, raster
  23.3 ms, wait 6.1 ms, 578 frames. Raster dominates both. Every future
  performance claim in this project is measured against these two numbers and
  recorded in Part G.

- **F-08** 2026-10-31: **a body table that is internally inconsistent is
  invisible until an orbit leaks.** Deriving the three bodies from KSP's
  published numbers produced one consistency test that no individual number
  can give: a moon's SOI has to fit inside its Hill sphere. Cheese's SOI is
  2,400 km and its Hill sphere at a = 12,000 km is 3,170 km -- consistent.
  The alternative reading of the same sources (Cheese at ~3,550 km, which is
  what a 6.2-hour period would imply) puts its Hill sphere at 936 km, *inside*
  its own SOI, which is impossible. That is the test that picked the orbit, and
  it is in `nispcheck` rather than in this paragraph.

- **F-09** 2026-10-31: **there is no hardware double here, and that is a design
  input, not an inconvenience.** The toolchain is `rv32imafc`: single-precision
  float in hardware, doubles through libgcc's soft-float. An orbital sim written
  in metres spends its precision on the exponent (1e7 m -> ~1 m resolution) and
  its time on soft-float. Written in kilometres it has centimetre-level
  resolution and stays in hardware float throughout. Hence D-15, decided before
  a line of orbital code exists.

- **F-10** 2026-10-31: **the engine's light is directional with no falloff**,
  and for this game that is not a limitation but the right model: from anywhere
  near Bob, the direction to Frodo is the same to well under a degree across the
  whole scene. The sun therefore becomes a per-frame direction
  (`normalize(Frodo - camera)`) rather than a light source with a position, and
  no point light is needed.

- **F-11** 2026-10-31: **four numbers players already know, used as anchors.**
  A body table can be wrong in a way that looks completely plausible on screen.
  The four Kerbal values that any player can quote -- 80 km circular orbit at
  2,278.6 m/s, stationary orbit at 2,863.9 km altitude, semi-synchronous orbit
  at 1,581.8 km and 1,272.3 m/s, surface escape velocity 3,430.5 m/s -- are
  reproduced by `mu = 3530.46 km^3/s^2`, `R = 600 km`, `T_rot = 21,549.4 s` to
  better than 0.1%. They are the first test in `nispcheck`, and they are why
  the derived Hohmann numbers (860 m/s out, 7.95 h coast, 360 m/s to
  circularise at Cheese) can be trusted before a single frame is drawn.

- **F-12** 2026-10-31: **the delta-v budget is the builder's only feedback, and
  the trap it has to catch is TWR, not delta-v.** A stack can have a textbook
  total delta-v and never leave the pad, because its first stage thrusts at
  0.8x its own weight. `rocket/budget.{c,h}` reports both per stage and in
  total, and `nispcheck` has exactly that case -- enough total delta-v, a first
  stage that cannot lift -- because it is the mistake a real player makes first.

- **F-13** 2026-10-31: **the engine already has the 2-D leaves a blueprint
  needs, so a 2-D screen needs no 3-D pipeline at all.** `se_direct565.h`
  exposes `direct_565_line`, `direct_565_tri`, `direct_565_vrun` and
  `direct_565_dim_rect` as inline primitives straight into the framebuffer,
  `se_text.h` is Hershey vector text, and `pax_background()` is a plain clear.
  A blueprint sheet is therefore a frame clear plus a few hundred line calls
  plus text: no camera, no depth buffer, no `SE_SCENE_*_CAP` involved, no PPA
  upscale. `main/ui/menu.c` and `main/ui/logo.c` already draw this way, so this
  is the app's existing path, not a new one.

- **F-14** 2026-10-31: **there is one music slot, and swapping it is not
  free.** `audio_mixer_set_music()` installs one `music_source_t` and fires the
  previous source's `shutdown()`; `se_mp3` runs minimp3 in its own task on core
  1, reading the card and resampling into a ring buffer. So a track change is a
  task teardown plus a card read plus a decoder start -- fine at a screen
  boundary, not something to do per frame. And `se_audio.h` is explicit that a
  source rendering silence still counts as active, so "no music" has to be a
  cleared slot (or the `_enabled` gate), not a source that plays nothing.
  Hence the hysteresis band in D-29: an orbit with a periapsis inside the
  atmosphere would otherwise swap the source once per pass.

- **F-15** 2026-10-31: **peak heating rate does not tell a launch from a
  reentry.** On this atmosphere `rho v^3` is 6.9e8 at 10 km doing 1,500 m/s -- a
  hard launch -- and 5.1e8 at 20 km doing 2,458 m/s -- a reentry from a 100 km
  orbit. The launch is the hotter instant. Only the *integral* separates them:
  a reentry spends minutes in that band, an ascent crosses it in tens of
  seconds. So any heat model keyed on instantaneous speed or dynamic pressure
  gets the two cases backwards, and the model has to accumulate per part with a
  cooling term. Hence D-30.

- **F-16** 2026-10-31: **a stable reentry attitude is not a survivable
  reentry.** With a burnt-out stage still attached, the static margin of a
  1.25 m stack goes from -0.19 m to +2.47 m (Part A5's table): the heavy capsule
  leads and the empty stage acts as fletching, so the air itself holds the pod
  shield-first for the whole descent. That is the *correct* aerodynamic answer
  and it is still fatal, because the shield is a factor of ~0.1 on a heat load
  of 5.1e8 and the capsule has a finite limit. Two consequences: the heat
  thresholds cannot be tuned against "shield forward means safe" (D-36), and a
  stability readout on its own can mislead -- the HUD has to show the margin and
  the heat, not one of them (steps 6.5, 6.6).

- **F-17** 2026-10-31: **the test kit runs end to end on this PC, with nobody
  at a device.** `../virtual-tanmatsu-graceloader` interprets the same
  `app.so`, staged from this repo's `metadata.json`, at the same install path
  it has on a card. Six imports its trap table did not have blocked the load
  outright: four `esp_h264_enc_*` (the livestream path), `pax_decode_png_fd`
  (the logo), `strftime`. The debug console was a stub returning
  `ESP_ERR_NOT_SUPPORTED`, so it is now backed by stdin
  (`src/shims/console_shim.c`) and a piped `RUN ...` is a command the app
  really receives. `perf` and `shots` come back with the same record chain
  `tools/testrun.py` parses off a badge, and the app exits by itself
  (`bsp_device_restart_to_launcher()`, which the emulator implements as
  `exit(0)`), so a run needs no window and no keyboard. Measured here:
  7.1-8.0 fps, `rast` 104-112 ms of a ~140 ms frame, logo 5.5 ms, 742 tris /
  24 lines on the title and 481 / 19 on the bench -- the same *shape* as the
  badge's 25.3 and 28.9 fps, at roughly a quarter of the speed, which is why
  the shape is the only thing carried across (D-39).

- **F-18** 2026-10-31: **the emulator cannot run sandboxed, and a command
  whose text varies asks for permission again every time.** Guest memory is a
  single `mmap(MAP_32BIT)` below 2 GiB, which the sandbox's seccomp filter
  kills with `Bad system call (core dumped)`; it opens an SDL window too, so
  the tooling classifies it as a GUI app. Both mean the run has to be
  escalated, and an escalation is remembered per exact command string: a
  different `TEST="..."`, a `| grep`, a different timeout -- each one a new
  prompt. Hence D-41: one command, always the same three words.

- **F-19** 2026-10-31: **a `shots` hash of the title screen is not
  reproducible, and cannot be while the title prints the build stamp.** Two
  identical runs of the same manifest gave `title_000000.png` hashes
  `5beae48b` and `ed4ac37e` with identical primitive counts (746 tris, 23
  lines) both times. The difference is `main/ui/title.c:89`, which draws
  `APP_GIT_HASH  APP_BUILD_TIME`, and `APP_BUILD_TIME` changes on every build
  -- the pixels differ because the binary was rebuilt, not because anything
  regressed. `settings_000000.png` hashed `d858e651` in both runs. Step 10.5
  is the fix; until then only stamp-free screens are valid references.

- **F-20** 2026-10-31: **a round-trip test cannot catch a wrong periapsis,
  and an equatorial orbit is where it hides.** `nisp_state_to_elements` had a
  degenerate branch for `inc == 0` (no node line, so `nlen == 0`) that read
  the periapsis angle as `atan2(e_y, e_x)` -- the eccentricity vector read in
  the X-Y plane instead of the X-Z one. Every orbit in the ecliptic, which is
  every orbit this game has, got its periapsis 90 degrees away. It survived
  the round trip because `aop` and `m0` are recovered together and are
  self-consistent: elements -> state -> elements -> state reproduces the
  state exactly with the wrong `aop`. Only propagating *forward in time* and
  comparing against an independent integrator showed it -- the analytic conic
  and the RK4 disagreed by 14 million km, because the analytic one was
  answering "where would this orbit be" for a different orbit. Fixed by using
  `atan2(ahead, along)` in both branches, where `along`/`ahead` are the
  eccentricity vector projected on `N` and `H x N`, which are defined even
  when the node line is not.

- **F-21** 2026-10-31: **the reference integrator sets the tolerance, so it
  has to be the accurate one.** Two separate mistakes here. First, the RK4
  wrote its result back into the float `nisp_state_t` every step, which
  accumulates about 250 m of quantisation over a 15,500 s leg -- bigger than
  the error the test is trying to see. Second, step size: RK4 is fourth
  order, and near periapsis (680 km, 2.3 km/s) a 0.05 s step leaves 17 m of
  its own error behind, while 0.0125 s leaves 4 m and 0.01 s leaves 1 m. A
  5 m tolerance at 0.05 s was measuring the reference, not the code. The
  reference now accumulates in double and runs at 0.01 s.

- **F-22** 2026-10-31: **a transfer to Cheese has a window, and it is
  narrow.** The 80 km -> 12,600 km Hohmann transfer is the right *shape* at
  every moment and the right *meeting* at only some of them. Scanning one
  synodic period (1,900.8 s) for the burn epoch, the best window in the whole
  period passes 1,908 km from Cheese's centre -- inside its 2,400 km sphere
  by 492 km, which is 20 % of the sphere. The reason is Cheese's 12 degrees
  of inclination: the transfer stays in the ecliptic, so the meeting is only
  close when Cheese is near one of its nodes. Two consequences. Part M3's
  auto-planner has to *search* for a window rather than compute a phase from
  a formula, and a planner that gets the phase wrong produces a perfectly
  valid conic that simply misses -- no error, no crash, a flyby that does not
  happen. Second, a real Cheese mission will want a plane change, which is
  the first reason this game needs the "change inclination" maneuver type at
  all.

- **F-23** 2026-11-01: **the heat constants are only meaningful together with
  the flight profile they are tuned against.** The first capacity table
  (0.133 / 0.242 / 0.133 t per tonne) left a nominal reentry peaking at 0.85
  on the unshielded stages: nothing ever burned, which makes the whole model
  decoration. Two things had to change together -- the column, down to
  0.089 / 0.161 / 0.087, and the launch profile, from a hard ascent that
  integrates 1.4e9 of rho*v^3 through max q to a throttled one at 9.9e8. The
  unthrottled number is not wrong: it is what a player who never pitches over
  actually earns. It is just not the profile a constant should be *tuned*
  against. Case 9 prints the peaks of all four profiles on every run, so a
  retune is a diff against a recorded profile instead of a re-derivation.

- **F-24** 2026-11-01: **a decoupler with an ordinary thermal capacity is the
  first casualty of every flight.** Separators are real rows in the stack
  (0.05 t each) and they sit in the flow. Scaled by mass like everything else,
  one would have a fortieth of its stage's thermal mass at the same exposed
  area, so every stage separation would start with the separator exploding and
  every rocket would come apart exactly where it was supposed to come apart.
  They carry mass and no heat capacity (`capacity = 0`), which extends Part
  A4's list of what is deliberately not modelled.

- **F-25** 2026-11-01: **side boosters are a group, not a part.** n boosters
  mean n cylinders of drag and n engines of thrust, and they run out on their
  own clock, not the stage's. The stack models them as one row with the
  cylinder area multiplied by n and the propellant summed, which is what makes
  "ignite with the stage, separate when spent" (Part V2) a property of the data
  structure rather than a rule the flight code has to remember.

- **F-26** 2026-11-01: **thrust cancels out of the rocket equation, and that
  is the F-12 trap in one line.** The first hand check for a two-Skipper stage
  multiplied its 360 kN of thrust by g0 and came out 33 % high; the answer is
  `Isp * g0 * ln(m0/m1)`, which does not care how the thrust is split. Delta-v
  comes from Isp and mass ratio; TWR and burn time come from thrust. Two
  different columns of the same table, which is precisely why a design can have
  plenty of delta-v and still sit on the pad.

- **F-27** 2026-11-01: **the emulator's trap table is a second export list, and
  it drifts from the device's.** `logf` -- used by `rocket/budget.c` for the
  Tsiolkovsky equation -- is in `../tanmatsu-graceloader/main/symbol_export/all`
  and links fine on the badge, but the emulator's `src/shims/trap_list.h` had
  no entry for it, so the app died at load there and nowhere else. The diff
  that finds these is `nm -u build/app.so` against the `X(...)` names in the
  trap list; it is worth running whenever a new libm call appears.

- **F-28** 2026-11-01: **air density has to be re-derived inside the Verlet
  step, not held across it.** The attitude and the frame's rotation are held
  over 0.02 s, which is fine; the altitude moved is a few tens of metres
  against a 5.6 km scale height, which is also fine, but only because it is
  *recomputed* rather than assumed equal. Holding the density too gives a
  second-order integrator a first-order error and quietly changes terminal
  velocity.

- **F-29** 2026-11-01: **the parachute's area was sized backwards from the
  number it has to produce.** Part A3 promises a 10 m/s soft touchdown, so
  `NISP_CHUTE_CDA_M2` is whatever delivers it: 300 m^2 gives a 1.0 t pod
  7.3 m/s at sea level and a 5.0 t fragment 16 m/s. That is the honest
  consequence of one canopy fitting every mass -- a heavy fragment needs legs
  or a retro burn, and the harness now prints 8.806 m/s rather than a
  hand-picked figure.

- **F-30** 2026-11-01: **every velocity- or angle-dependent torque needs a
  cap, or the tick stops meaning anything.** The weathervane torque is a
  spring; a spring whose natural frequency exceeds the physics tick integrates
  to noise, not to a turn. Each one is capped against `inertia *
  NISP_TORQUE_OMEGA_MAX^2`, which says out loud how fast the air is allowed to
  rotate the ship (about a second to line up with the flow). The same rule
  covers the drag clamp -- one tick may not take more of the relative velocity
  than the tick has -- and the last-resort rate cap in `flight/physics.c`.

- **F-31** 2026-11-01: **steering costs the same as burning.** A coast is
  analytic, and an analytic propagation applies no forces at all, so a ship
  whose SAS is fighting a rotation cannot coast: `can_coast` refuses it and the
  frame is paid for in ticks. The alternative -- a SAS that silently stops
  working the moment the engines cut -- is a bug a player could never name.
  Debris is never controlled, so it coasts whenever it is free.

- **F-32** 2026-11-01: **the weathervane is a spring with no damper, and a
  spring with no damper is a pendulum.** A capsule under a chute swung through
  vertical, past it, and back, forever, and case 13's chute drop landed at 43
  degrees of tilt on a perfectly good canopy. The damper is the same drag
  applied to the sideways motion rotation creates: `0.5 * rho * |v_rel| *
  sum(cd_a * d^2)`, which `rocket/stack.c` carries as `aero_damp_m4`.

- **F-33** 2026-11-01: **a near-circular orbit produced a NaN, and the guard
  after the square root could not catch it.** `1 + 2*eps*p/mu` is exactly zero
  in theory and about -1e-15 in float; `sqrt` of that is NaN, and NaN compares
  false against 0.0, so the check that used to sit after the root never fired
  and the NaN propagated through a whole flight. The argument is clamped
  before the root (`sim/kepler.c`).

- **F-34** 2026-11-01: **the edge of the air is inside a chunk, never on a
  chunk boundary.** A chunk exit is an SOI handover, and a warp-10,000 frame is
  167 simulated seconds, so an orbit can dive through an atmosphere and come
  back out without ever crossing a boundary -- the old code flew the whole way
  through and reported the ship underground. Two answers: bisect the conic
  inside the chunk, and refuse the high warp in the first place when the *next
  periapsis* is under the air. Being above the air right now is not the test.

- **F-35** 2026-11-01: **an SOI handover is a change of frame for the velocity
  as well as the position.** The stored velocity is relative to the body being
  left, so what has to be added is that body's velocity relative to the one
  taking over -- not its velocity in the sun's frame. Frodo is the root and its
  velocity is zero, which is why the leaving case looked right on its own; the
  entering case subtracted a child's heliocentric velocity from a
  parent-relative one and left Bob's own 9.3 km/s in the answer, turning a
  Cheese approach into a flyby at ten times escape speed.

- **F-36** 2026-11-01: **SAS has to command a rotation rate, not a spring
  force.** A spring/damper on the angle looks right on paper: 1,200 kN m/rad on
  a 30 degree error asks for 628 kN m, and a three-stage stack has 45. The
  command saturates, the ship slews at full authority, the angle term collapses
  on the way to the target while the ship is still rotating at 0.13 rad/s, and
  what is left of the damping cannot stop it -- a limit cycle, which is F-30 at
  scale. The switching law a real RCS runs (arrest rate `sqrt(2 * alpha *
  angle)`, chased) holds at exactly the authority limit: 0.0862 rad/s, 629
  frames, no overshoot.

- **F-37** 2026-11-01: **the bottom edge of the sheet was a fight for the same
  pixels.** The sheet used to run to the last row of the screen, so its title
  block sat at x = 408..584, y = 410..472 while the action row (`Save`, `Name`,
  `Launch`, `Back`) was drawn at y = 458 across x = 178..420 -- straight through
  the block's left border and its numbers. The parts column had the same
  problem in miniature: a 12 px help line of more than about 26 characters walks
  past x = 168 into the paper. Both are one mistake -- the drawing was allowed
  to be as tall as the screen, leaving everything that belongs *under* it
  nowhere to go. Fixed by `SHEET_H 400` (the paper now ends at y = 444 and the
  actions have a strip of their own), by filling the title block with the
  paper's own blue before its border and text (a wide stage's tail stops at the
  block instead of running through it), and by short help strings.

- **F-38** 2026-11-01: **a key at the start of a line was never found.** The
  blueprint decoder matched a field name only when the byte before it was a
  space, so any key that begins a line silently returned its default:
  `stages=`, `pod_rcs=`, `pod_chutes=`. The bug predates the pod options -- it
  just never mattered, because the fields that happened to start a line were
  ones whose default was also what was written. Found by case 14's
  `encode -> decode -> encode` check, which is the only test in the harness
  that asks the file a question rather than the maths.

- **F-39** 2026-11-01: **the emulator's card was part of the measurement.**
  The Spaceport hub names the slot and counts the blueprints, the Designs
  screen *is* the list, and the settings screens show the saved settings, so
  all three are drawn from whatever the card happens to hold. A save, a
  blueprint and a settings file left over from playing moved `spaceport` and
  `designs` with nothing at all changed in the code. `make emu-test` now
  clears `saves`, `designs` and `settings.txt` from the card before the
  manifest (`make emu-run` does not, because there the point of the card is
  that something is saved on it), and two runs in a row then produced all
  seven hashes identically.
- **F-40** 2026-11-01: **the solar system went the wrong way round, and only
  the length of a day noticed.** Bob's rotation carries the pad toward -z, and
  the orbits were written sweeping the other way: `pz = +a sin u cos i` with
  `vz = +a n cos u cos i`, which is a position sweeping -z toward +x and an
  angular momentum of -y. Every number that does not care about handedness was
  right with it -- radii, surface gravities, all three periods, the whole
  delta-v budget -- and the bug survived four blocks of tests. What caught it
  is day and night: a point on the surface was chasing its own sun, and Bob's
  solar day came out 21,499 s against the 21,600 s a Kerbin player can quote
  from memory. Flipping the `z` component of position and velocity together
  gives 21,599.98 s, a sidereal day of 21,549.4 s, the pad's first noon at
  10,799.1 s, and a point 90 degrees east of the pad at noon a quarter of a
  day earlier. `nispcheck` case 15 is those numbers, and it is the only case
  in the harness that could have seen this -- which says something about a
  harness that measures only magnitudes.


### Decisions

- **D-01** The game is **Null Island Space Program**, motto *"To nowhere, and
  beyond!"*, logo `nullislandlogo_ball.png`, slug `at.cavac.nisp`.
- **D-02** **No chunk streaming, and no stream memory reservation at boot.**
  Whatever SynthMiner needs there, this game does not: there is no paged world
  to page in.
- **D-03** The logo is **data on the card beside `app.so`**, not a blob in the
  binary: it can be redrawn without rebuilding, and `metadata.json` lists it as
  an asset so an install from the app repository brings it along.
- **D-04** The bottom bar is **Play / Settings / Quit**, as in SynthMiner.
  Play opens the slots. The livestream is **not** a title-bar item.
- **D-05** Settings, audio and display keep the **SynthMiner shape**: Controls /
  Graphics / Audio / Display / Livestream, one `settings.txt`, volume and
  brightness owned by the device through `se_hw.h`.
- **D-06** The mp3s and their license `.txt` files **are in git** and are
  uploaded to the card with the app. Music without its license file is not
  shippable music.
- **D-07** Menu music is `music/menu/brittle_rille.mp3`, **looped**, replacing
  the procedural generator (which stays as the fallback when the card has no
  music).
- **D-08** `make push` uploads `app.so` only; `make install` uploads everything
  and is for when the assets actually changed.
- **D-09** Player data lives in `/sd/nisp`, outside the install directory the
  launcher owns. Copying that directory is the whole backup.
- **D-10** **No free-hand rocket building.** A rocket is a blueprint of
  configured stages: there is no mouse, and pretending otherwise would be a
  worse game on this hardware.
- **D-11** **A design must be saved under a name before it can be launched.**
  Enforced in Engineering, not left to the player's diligence.
- **D-12** Building is **top-down**: the capsule is always the top, stages are
  added beneath it, and staging advances downward from stage 1.
- **D-13** **No science points, no money, no achievements, no crew.** Robotic
  capsules only. The whole scope right now is "build rockets that fly".
- **D-14** Three flight modes -- **manual, stability assist, automatic guidance
  onto the next node** -- and a **simplified map**: six-axis manual nodes, or
  TAB to the auto-planner with *change apoapsis / change periapsis / change
  inclination* for now.
- **D-15** The simulation is in **kilometres, seconds and km^3/s^2**, float32.
  Rendering converts to metres, camera-relative.
- **D-16** **Camera-relative submission**: the renderer never sees a coordinate
  above ~1e6 m.
- **D-17** The **state vector is the truth**; orbital elements are a view of it,
  derived on demand and cached.
- **D-18** **Bob is a sphere for physics.** A visual heightmap may come later;
  it is not in the first playable build, and nothing may depend on it.
- **D-19** One **exponential atmosphere** (scale height 5.6 km, top 70 km,
  101.325 kPa at the surface) rather than KSP's temperature-and-pressure curve.
- **D-20** **Heating is a rule, not a simulation.** Shield forward and on top
  survives; exposed at orbital speed in dense air does not.
  (Refined by D-30..D-33: the rule is per part, it accumulates, and a failure
  splits the stack rather than ending it.)
- **D-21** **Stage separators are implicit** -- a stage boundary is a separator.
  Side boosters ignite with their stage's main engine and separate when *they*
  are empty, not when the stage is.
- **D-22** **The rocket is one rigid body.** No joints, no wobble, no
  part-level contacts; CoM and CoT are computed over all connected parts and
  recomputed as fuel burns and stages separate.
  (D-32 is the one exception: a burn-through cuts the body, and each fragment
  is then rigid in the same sense.)
- **D-23** The save schema gets a **version field** before mission state is
  added to it.
- **D-24** The **Spaceport is a hub**: you pick the *function* first
  (continue / build / launch / manage designs / manage the slot), and only
  then the parameters that function needs -- which saved design to launch,
  which in-progress mission to continue. The hub itself never expands into
  one row per design or per mission.
- **D-25** Engineering is a **blueprint sheet, not an assembly view**: a
  scrollable part legend on the left (168 px), the drawing in the middle
  (424 px), post-it stat notes on the right (208 px). The drawing is a
  classic drafting-blue sheet with white pencil technical drawing, dimension
  callouts and a title block. The stage list, the readouts and the preview
  are the same object.
- **D-26** **RCS and side boosters are stage parameters, not legend parts.**
  A stage has them or does not; there is no attach interaction, because an
  attach interaction needs a pointer this device does not have.
  (Extended by D-38: landing legs are a third such parameter.)
- **D-27** The sheet is drawn with the **2-D direct-565 leaves**, not the 3-D
  scene pipeline (F-13). Half resolution has no effect on it, and it is
  expected to be the cheapest screen in the game -- measured in 4.8, not
  assumed.
- **D-30** Heating is **`rho * v^3` accumulated per part**, with a cooling time
  constant and a per-part limit, and it is the same model in the launch and in
  the reentry (Part A4, F-15). No temperature field, no conduction, no view
  factors, no solar heating.
- **D-31** **The capsule's heat shield is the only protection in the game**, and
  only while it faces the oncoming air. A capsule with nothing attached orients
  shield-first by drag, because its mass is biased toward the shield end; a
  stack with heavy stages attached may hold itself the other way round (Part V4).
  (Refined by D-36: "the only protection" is not "immunity".)
- **D-32** **An overheating part destroys that part**, and the stack splits at
  it into a capsule fragment and uncontrolled fragments (Part F5). Not "the
  whole rocket explodes" -- the failure unit is the part, and the split reuses
  the staging path.
- **D-33** **The actively controlled body is always the capsule's fragment.**
  Keys, SAS, throttle, staging and auto-guidance apply to it alone; debris is
  ballistic, uncontrolled, still simulated, still drawn, and capped.
- **D-28** Music is per context: **menus `brittle_rille.mp3`, Engineering
  `sneaky_adventure.mp3`, flight above the atmosphere `impact_lento.mp3`**, each
  looped, each in its own directory on the card (`music/menu/`, `music/engineering/`,
  `music/space/`) because `se_mp3` plays a directory rather than a file.
- **D-29** **No music during atmospheric flight** -- pad, ascent, reentry and
  touchdown are silent. The boundary is the atmosphere (70 km), the switch is
  edge-triggered with a 60/70 km hysteresis band, swaps fade ~0.5 s rather than
  cut, and the Audio setting outranks the context rule (Part N4, F-14).

- **D-34** The build screen is called **Engineering**, not a VAB or any other
  Kerbal name. The screen name, the source directory `main/engineering/`, the
  perf scene `perf scene=engineering` and the card directory
  `music/engineering/` all follow it, so there is one word for it everywhere.
  The Spaceport verb stays **"Build a rocket"**, because the hub lists verbs and
  the verb opens the Engineering screen (D-24).
- **D-35** Aerodynamics is **one static margin**: per-part `CdA`, a drag centre
  as the `CdA`-weighted position, `SM = x_CP - x_CoM`, and torque
  `q * CdA_total * SM * sin(alpha)` (Part A5). No fins, no airbrakes, no Mach-
  or Reynolds-dependent `Cd`, no wave drag, no cross-axis coupling. Stability is
  a property of the mass distribution the blueprint already draws, not a part
  the player can buy.
- **D-36** **The heat shield is a factor, not immunity.** The capsule is a part
  with a `capacity` and a `limit` like every other row, and going fast enough
  nose-first destroys it and ends the mission (Part A4, F-16). Shield-first is
  the *stable* attitude, not the safe one; survivability comes from arriving
  slower, which is the player's job. Shield degradation / ablation is a later
  refinement, deliberately not in the first cut.
- **D-37** **Engineering does not predict survivability.** The notes print part
  ratings (shield factor, rated touchdown speed, thrust, Isp, delta-v) and never
  a verdict about a flight the player has not flown yet -- no stability verdict,
  no heat verdict, no "will this reenter" card. Whether a design comes home
  depends on the mass distribution, the arrival speed and whether the player
  staged, and that judgement is not a number a post-it can carry (Part B4).
- **D-38** **Landing legs are a stage parameter and a touchdown rating.**
  `legs: none / 3 / 4 / 6` on a stage, a mass row per leg, and `rated_v` /
  `rated_tilt` selecting which of Part A3's thresholds apply to the fragment
  that lands (Part V7). They exist because Cheese has no atmosphere. No contact
  mechanics: no crush, no breakage, no per-leg load.

- **D-39** **The virtual graceloader is the default harness for everything
  that is not a timing question.** Menus, layout, flows, saves, scene
  composition, primitive counts: iterate them on the PC, where nothing has to
  be plugged in and nobody has to be at a badge. Frame rate, memory pressure,
  and every Accept line that says "on the badge" stay on a device. The
  emulator's fps gets recorded as a number about the interpreter and is never
  compared against a Part G target.

- **D-40** **One manifest, two repos.** This repo's Makefile knows the
  emulator's directories and the guest path; the emulator's Makefile knows
  this repo and stages whatever `metadata/metadata.json` lists. No asset is
  written down twice, so an install onto a card and an install into the
  emulator cannot disagree about what is on the card.

- **D-41** **One command, always the same text:** `make emu-test`. No
  arguments, no pipes, no per-run timeouts. The tests live in
  `tools/emutests.txt`, one console command per line, each line run in its own
  launch (the app exits to the launcher when a test ends), and everything the
  app printed is kept in `results/emulator/latest.log`. A run is reproducible,
  reviewable in the repo, and identical every time -- which is also what makes
  it approvable once instead of on every call (F-18). `make emu-run
  EMU_SECS=n` stays for the interactive window.

- **D-42** **Stages ignite bottom-up.** Part V1 says "stage 1 fires first,
  then the next stage down", which is a slip in ladder order: ladder index 0 is
  the *topmost* row, and the stage that lights is the bottom-most one. Written
  into `rocket/stack.h` and `rocket/budget.h` rather than left to be rediscovered,
  because the ladder index and the firing order are two numbers that look alike.

- **D-43** **Heat thresholds are set by `nispcheck`, not by flying a badge.**
  The four profiles in case 9 are the definition of "nominal" for this game. A
  constant changes by editing `rocket/parts.c` and reading the printed peaks,
  not by launching at a device and guessing which part died.

- **D-44** **A part that peaked over its limit is dead, even if it cooled
  afterwards.** The harness flies heat open loop and records the first breach;
  flight closes the loop and splits the stack the moment `nisp_heat_breach`
  answers. Peak-over-limit and breached-during-the-profile are the same fact
  seen at two different times, and case 9 checks the *order* things die in, not
  only the peaks.

- **D-45** **A touchdown rating belongs to the fragment, and legs only improve
  it.** Each fragment's rating is the best deployed leg set attached to it, or
  the capsule's bell rating if there is none; a leg set is never worse than the
  bell. It selects which thresholds apply -- it does not add a contact
  mechanic (Part V7, out of scope).

- **D-46** **Control authority is the RCS budget of the modules that are
  actually attached, and nothing hidden on top of it.** 45 kN m for a
  three-stage stack, 30 after a stage, 15 on the pod alone. A rocket with no
  RCS module anywhere has no authority at all, SAS included, and the SAS switch
  does not change that.

- **D-47** **A rocket with no RCS flips over in the air, and that is the
  lesson, not a bug.** The weathervane turns whatever is broadest to face the
  flow, which for a fat booster with a small pod is the wrong end forward. The
  HUD says which way the air is pushing (Part A5); the fix is to buy RCS in
  Engineering, not to ask the physics to be kinder.

- **D-48** **Launching from Engineering writes the blueprint to the card
  first.** "The player always has to save a design with a name before launch"
  is enforced by the code path rather than by the player remembering: Launch is
  refused while the design is unnamed, and a named-but-unsaved design is saved
  before the mission begins.

- **D-49** **Opening a world opens its Spaceport hub and launches nothing.**
  The hub is a mode of `main.c` (`MODE_WORLD`), not a detail of the menu,
  because it is where the game decides which mission or design the next verb
  applies to (Part N1, D-24).

- **D-50** **The world has one fixed internal seed, `NISP_WORLD_SEED`.** There
  is no seed to type and no seed screen: the sky is the same sky for every
  player, and the number exists so the starfield and the surface noise are
  reproducible rather than so they can vary. The seed field stays in the save
  file so a slot written before this decision still loads (Part N3).
- **D-51** **The command pod has exactly two options: RCS pods, and 1..3
  parachutes.** They are edited on the sheet, with the cursor parked above
  every stage, because the pod is not a stage -- no diameter, no length, no
  engine, nothing to remove. Each one is worth what it says it is: 0.10 t for
  the pods (one control module, 15 kN m, and the pod still has it after
  everything below has burnt away), 0.05 t per extra canopy for 300 m^2 apiece,
  which drops the touchdown speed by `sqrt(area)` -- 27.87 m/s under one canopy
  against 16.59 m/s under three. No pods and one canopy is the default, and the
  default is also what a blueprint written before they existed means, so an old
  rocket flies exactly as it did.
- **D-52** **The key press that closes a menu belongs to the menu.** A dialog
  that closes on ENTER must not hand that same ENTER to the mode underneath it,
  and one that closes on ESC must not travel two levels up in the same frame.
  `main.c` samples the keys only when the menu was closed at the start of the
  frame, which is what makes a dialog closeable at all.
- **D-53** **A launch happens in daylight, and the sky runs on the mission's
  clock while it is flying.** The world clock starts at the pad's first local
  noon (`nisp_local_noon`, `sim/ephemeris.c`), so the first thing a player sees
  through a chase camera is a lit planet rather than a black one, and no one
  has to wind a clock to test a launch. PgUp / PgDn move the epoch an hour, or
  six with the boost key, from the title and the world view. Once a ship is
  flying the sky is asked where the planets are at `mission.t`, not at the
  show clock: a warp of 10,000 that left the sun where it was would be a sky
  that disagrees with the position it is drawn against.

- **D-54** **Every key is rebinding-able, and the list is grouped.** As in
  SynthMiner, nothing about the controls is fixed; unlike SynthMiner the
  bindings are too many for one screen, so Settings -> Controls is a list of
  sections -- Flight, Map and warp, Engineering, Camera, System -- and each
  section opens its own scrolling list of actions. A section is stored as
  section + 1 in the action table, so an action that was never assigned reads
  as unassigned rather than as "Flight", and `input_init` logs a warning for
  it: the failure mode of an append-only action enum is a new action nobody
  can rebind, and that is a bug a log line can catch at boot.


---

## Verification

- **Host:** `make check` = `hostpurity` + `nispcheck` (Part H). Seconds, no
  badge, and it is what actually validates the orbital and rocket maths --
  nothing about a 38-hour transfer is visible on a 480x800 screen.
- **Device:** `make cycle TEST="perf scene=<name> secs=20"` for frame rate and
  phase split; `make testrefs` / `make testcompare` with
  `TEST="shots scene=<name> ms=..."` for framebuffer-hash regressions on the
  scripted flight scenes and each menu screen.
- **Virtual device:** `make emu-test` -- always those three words (D-41). It
  builds, stages onto `../virtual-tanmatsu-graceloader/sd`, runs every line of
  `tools/emutests.txt` in its own launch, and leaves the records in
  `results/emulator/latest.log`. Screens, flows, primitive counts; never frame
  rate (D-39).
- **Build hygiene:** `make build` clean with no new warnings, `make verify`
  (every undefined symbol exists in `fakelib`). `make format` needs a
  `clang-format` on PATH, which this machine does not have -- formatting is
  matched by hand against `.clang-format` instead.
- **By hand:** the user flies it. Each block's Accept line is a thing to do on
  a badge, not a thing to assert in code, and the first playable flight (block
  5) is the point at which the design either feels like flying or does not.

## Critical files

- `claudeplans/project.md` -- this document. It is updated as part of the work,
  not afterwards.
- `tools/emutests.txt` -- the emulator test manifest: what `make emu-test`
  runs. A test gets added by editing this file, never by typing a command.
- `../virtual-tanmatsu-graceloader/Makefile` -- `nisp` in its app list, and the
  staging that reads this repo's `metadata.json` (D-40).
- `../virtual-tanmatsu-graceloader/src/shims/console_shim.c` -- the emulator's
  USB-serial/JTAG backed by stdin, without which `make emu-test` could not
  reach the app at all (F-17).
- `main/main.c` -- the mode machine and the frame loop; everything that changes
  which world is open or whether the port is a video link happens here.
- `main/sim/` -- the sky: `bodies.c` is the table the whole game is built on,
  `kepler.c` is the arithmetic everything else depends on, `patches.c` is the
  SOI chain and the handover (F-35).
- `main/rocket/` -- `parts.c` is the parts table and the Engineering legend,
  `stack.c` is the rigid-body geometry and the static margin, `budget.c` is the
  number the builder lives by, `heat.c` is the one per-part state and the rule
  that splits a rocket, `design.c` is the blueprint in RAM and the file format
  that has to survive being read back (F-38).
- `main/flight/` -- `physics.c` is the tick (forces, torque caps, heat, SOI),
  `mission.c` is the ship plus its debris and its music context, `warp.c` is the
  ladder and its refusals, `control.c` is RCS authority and the SAS switching
  law (F-36).
- `main/engineering/` -- `builder.c` is the focus ring and the parameter ladders,
  `sheet.c` is the blueprint renderer: paper, pencil strokes, dimension callouts,
  title block. One function, three sizes (Part B5).
- `main/map/planner.c` (to be written) -- the auto-planner, and the place where
  "simplified compared to Kerbal Space Program" is actually implemented.
- `main/ui/menu.c` -- the title bar, the slots, the settings screens, the pause
  menu, and the Spaceport hub that the game hangs off.
- `main/game/scene.c` -- the world as drawn: three lit spheres and a starfield,
  camera-relative metres, the LOD ladder.
- `tools/nispcheck.c` -- the host harness, 15 cases, and the only thing standing
  between this project and a plausible-looking wrong solar system.
