# Destruction Sandbox

A playable prototype of a structural destruction system, in the spirit of
*The Finals*: buildings are graphs of chunks glued by bonds, gravity loads
flow through those bonds to the ground, and anything that loses its path to
the ground falls as a rigid body with its own mass and centre of mass.

It is a 2D side-section (0.5 m cells) so every mechanism is visible: stress
in each joint, the support graph, centres of mass and support bases.

## Run it

```bash
npm install          # only needed for the headless tests (planck.js)
npm start            # serves the folder on http://localhost:8080
npm test             # headless physics checks
```

The page loads planck.js from jsDelivr. ES modules need a web server, so
open it through `npm start` rather than as a `file://` URL.

**Controls:** click uses the tool (Blast, Drill, Weight, Inspect, keys `1`–`4`),
drag pans, wheel or pinch zooms. `S` stress, `G` support graph, `C` centre of
mass, `A` critical supports, `D` exaggerated deflection, `Space` pause, `R` rebuild.
The **Try a failure** buttons run scripted tests (drop the crane counterweight,
remove a column, undermine the leaning tower…).

## How it works

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

Only the worst few overloaded bonds break per solve, then the island is
re-solved. Load redistributes, so collapses progress realistically instead of
everything snapping at once.

### Centre of mass
`COM = Σ mᵢpᵢ / Σ mᵢ`, inertia by the parallel-axis theorem
(`graph.js → massProps`). Anchored islands show their support span on the
ground: green when the COM is safely inside, amber when close, red when
outside. Falling islands hand their chunks to planck.js as one compound body.
The tests confirm our COM and mass match the engine's to 1e-15.

### Physics bridge — `src/sim.js`
* Standing chunks are fixtures on one static body; a detached island becomes
  one dynamic body (inheriting velocity when a falling body splits).
* `pre-solve` records approach speed, `post-solve` records impulses.
  Fast impacts damage bonds. Resting contact forces become external loads
  on standing chunks, so debris piling on a slab can overload it.
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

Next steps: cluster hierarchy, resumable CG across frames to remove solve
spikes on very large islands, and debris sleeping/merging.
