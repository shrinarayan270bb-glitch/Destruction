# Destruction Sandbox

A playable prototype of a structural destruction system, in the spirit of
*The Finals*: buildings are graphs of chunks glued by bonds, gravity loads
flow through those bonds to the ground, and anything that loses its path to
the ground falls as a rigid body with its own mass and centre of mass.

Two versions share the same ideas:

* **3D** (`3d.html`, `src3d/`): 1 m voxel buildings, a member-level 6-DOF
  stress solver, a voxel rigid-body engine and a three.js view.
* **2D lab** (`index.html`, `src/`): a side section with 0.5 m cells where every
  mechanism is easy to see.

## Run it

```bash
npm install          # only needed for the headless tests (planck.js)
npm start            # serves the folder: http://localhost:8080/3d.html (3D), /index.html (2D)
npm test             # headless checks for both (npm run test:2d / test:3d)
```

The 2D page loads planck.js and the 3D page loads three.js from jsDelivr. ES modules need a web server, so
open it through `npm start` rather than as a `file://` URL.

**Controls:** click uses the tool (Blast, Drill, Weight, Inspect, keys `1`–`4`),
drag pans, wheel or pinch zooms. `S` stress, `G` support graph, `C` centre of
mass, `A` critical supports, `D` exaggerated deflection, `Space` pause, `R` rebuild.
The **Try a failure** buttons run scripted tests (undermine the house, cut the
office at the 3rd floor, blast an upper corner, remove a column, drop the crane
counterweight, undermine the leaning tower…).

## 3D version — `src3d/`

```
graph.js      voxel support graph, islands, members, support polygon (convex hull), articulation points
stress.js     member-level RBSM: 6 DOF per member, 6 springs per bond, block IC(0) PCG, time-sliced
physics.js    voxel rigid-body engine: sampled-surface contacts vs grid, sequential impulses, sleeping
sim.js        failure rules, impact crushing, debris loads, time-sliced stress jobs
structures.js house, office block, tower crane, leaning tower, construction site
render.js     three.js instanced voxels, stress colours, support polygons, COM markers, graph lines
```

**Two-level graph (the optimisation).** Every authored box is cut into
structural *members* no bigger than 4 voxels a side: slab panels between
column lines, storey-high column segments, wall panels, mast and jib sections.
The stress solve has one rigid node (6 DOF) per member; every voxel bond that
crosses a member boundary keeps its own six springs (normal, two shears,
torsion, two bending), so joint stiffness and capacity come from the real
interface geometry. The office goes from ~5000 unknowns to ~900 and a cold
solve takes ~11 ms (was ~200 ms per voxel); members fail at their joints as
whole panels, which is what makes collapses read as big pieces. Blasts and
impacts can still break bonds inside a member, which splits it.

**Failure rules** are the 2D ones (below) plus two that 3D plates need:
a connection between two members cracks as a unit, and when a region comes
loose, every other loose region attached to it through overloaded joints
joins the same falling body (otherwise parallel yield lines in a floor plate
peel it into strips).

**Voxel rigid bodies.** Each falling piece is one body. Its exposed faces
carry sample points on a half-voxel lattice; a point inside a voxel of the
standing grid or of another body is a contact pushed out through that voxel's
nearest exposed face. Contacts are solved with sequential impulses
(friction, warm starting, Baumgarte). A body sleeps when every point of it is
nearly still, so a tall crane starting to tip is not frozen. Contacts report
approach speed and impulse to the damage system.

**Time slicing.** Stress solves are generators that yield every few
iterations; the simulation runs them under a 12 ms per-frame budget, so a big
solve spreads over a few frames instead of stalling one. Jobs check that
their island has not changed before applying a result.

Measured headless: a full construction-site collapse (crane topple + office
bay) averages 2.6 ms per step.

## How it works (2D lab)

```
damage (blast / drill / impact)
   │  chunk HP and bond integrity drop
   ▼
break chunks / bonds ──► split islands (BFS over intact bonds)
   │                        │ no path to ground → dynamic rigid body
   ▼                        ▼   (mass, COM, inertia from its chunks)
stress solve on dirty anchored islands (time-sliced)
   │  overloaded bonds break → split again
   ▼
contacts from falling debris ──► impact damage + extra load on standing chunks
```

### Support graph — `src/graph.js`
* **Chunk** (node): grid cell with material, mass, HP, home position.
* **Bond** (edge): shared face between neighbours, with contact centroid,
  normal, length, integrity and limits (tension / compression / shear).
* **Ground** is a fixed node. Bonds to it are *footings*: `cast`, `bolted`
  or `resting` (zero tension, so a resting block only stands on its COM).
* **Islands** are connected components. An island is *anchored* when any of
  its chunks still has an intact footing.
* **Critical supports** are articulation points of the graph with the ground
  included (iterative Tarjan): remove one and something loses its support.

### Stress — `src/stress.js`
Rigid-Body-Spring Model. Each chunk has 3 DOF (x, y, rotation); each bond is a
normal, shear and rotational spring at its contact centroid. Solving
`K q = f` (gravity + debris loads) gives every bond its axial force `N`,
shear `V` and moment `M`. The solver is preconditioned conjugate gradient
with a block incomplete-Cholesky preconditioner. The grid has no triangles,
so IC(0) needs no fill. Warm starts reuse the last solution.

A bond fails when its utilisation exceeds 1, using a plastic section of length `L`:

| Mode | Utilisation |
|---|---|
| pulled + bent (N > 0) | `N/(t·L) + |M|/(t·L²/4)` |
| pressed + bent (N < 0) | `|M| / (t·L²/4 + |N|·L/2)`: compression adds bending capacity, which gives the no-tension (masonry) rule |
| crushed | `|N|/(c·L)` |
| sheared | `|V| / (s·L + 0.6·|N|)`: friction under compression |

### How things come apart — `src/sim.js → _fail`

This is what makes damage read like a building, not paper:

1. **Detach, don't shred.** On overload, release every bond that is
   seriously overloaded (at least half as overloaded as the worst) and find
   the region that would lose its path to the ground. Break **only the bonds
   on that region's border**. The whole region leaves as **one rigid body**;
   overloaded bonds inside it stay intact, because falling unloads them.
   If nothing would come loose, the load has another path: crack the single
   worst bond and re-solve.
2. **Take what goes with it.** Re-solve what would remain. Anything that would
   fail right next to the falling region, or would be left dangling, joins it.
   Take out a ground-floor column and the bay above drops as one frame
   through every floor, instead of floor by floor.
3. **Damage at the contact.** A landing spends its energy
   (`½·impulse·approach speed`) breaking the bonds nearest the contact
   point first; farther bonds cost more, so damage stays local.
4. **Snap where weak.** A hard landing also stress-checks the falling body:
   contacts become temporary supports and the deceleration (Δv over the
   *impact time*) acts like stronger gravity. Long overhangs and thin columns
   snap off as pieces; compact parts survive. Once a piece comes to rest it is
   checked again under plain gravity on whatever it landed on.
5. **Pancaking.** Debris resting *on* a standing structure feeds its weight
   into that structure's stress solve (averaged, capped at 1.5× the debris
   weight), so a dropped block can overload the floor below. Sideways shoves
   only crush locally.

The **Impact** slider scales steps 3 and 4.

### Centre of mass
`COM = Σ mᵢpᵢ / Σ mᵢ`, inertia by the parallel-axis theorem
(`graph.js → massProps`). Anchored islands show their support span on the
ground: green when the COM is safely inside, amber when close, red when
outside. Falling islands hand their chunks to planck.js as one compound body.
The tests confirm our COM and mass match the engine's to 1e-15.

### Physics bridge — `src/sim.js`
* Standing chunks are fixtures on one static body; a detached island becomes
  one dynamic body (inheriting velocity when a falling body splits).
* `pre-solve` records approach speed, `post-solve` records impulses (see
  steps 3–5 above). A piece that has just broken away gets no friction on its
  crack faces for 0.6 s, so it slides off instead of dragging its neighbours.
* Continuous collision is off: debris moves well under a cell per step, and
  CCD against hundreds of touching fixtures cost 20–40 ms per step.
* The stress solve runs under a per-frame time budget (6 ms by default).

### Optimisations in place
* Only *dirty* islands are re-solved or re-flood-filled.
* Warm-started PCG with block IC(0): about 120 iterations and 7 ms for the
  369-chunk office, versus about 900 iterations with plain Jacobi.
* Time-sliced stress queue, capped debris bodies, cosmetic dust kept out of
  the simulation.

## Files

| File | What it holds |
|---|---|
| `src/materials.js` | material and footing tables (density, strengths, stiffness, HP, toughness) |
| `src/graph.js` | `Structure`: chunks, bonds, islands, BFS split, mass properties, articulation points; `Builder` for authoring |
| `src/stress.js` | RBSM stress solver + failure rule |
| `src/structures.js` | presets: house, office block, tower crane, leaning tower, construction site |
| `src/sim.js` | simulation loop, damage, impacts, debris loads, planck.js bridge |
| `src/render.js`, `src/main.js`, `index.html` | canvas renderer and UI |
| `test/run-tests.js` | headless checks (statics, crane tipping, progressive collapse, COM, loads) |

## Taking it to a game engine

The core (`graph.js`, `stress.js`, failure rule) is engine-agnostic and maps
directly to 3D:

* chunks get 6 DOF and bonds get 6 springs (normal, 2 shear, torsion, 2 bending);
* fracture authoring (Voronoi or modular pieces) replaces the grid, and bond
  area comes from the shared face;
* the server owns damage, stress and connectivity, and replicates only
  `bond broken` / `chunk destroyed` / `island spawned` events;
* add hierarchical clusters (wall → floor → building) so stress runs on a
  coarse graph first and refines only near damage.

Next steps: move stress jobs to a Web Worker (or the game server), Voronoi or
authored modular pieces instead of voxels, and replicating `bond broken` /
`island spawned` events for multiplayer.
