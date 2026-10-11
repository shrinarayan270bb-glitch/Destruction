// Headless checks for the 3D destruction core. Run: npm run test:3d
import { Structure, Builder, GROUND, polygonMargin } from '../src3d/graph.js';
import { StressSolver } from '../src3d/stress.js';
import { Simulation } from '../src3d/sim.js';
import { GRAVITY } from '../src3d/materials.js';

let failed = 0;
function check(name, ok, detail = '') {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? '  — ' + detail : ''}`);
  if (!ok) failed++;
}
const upTo = (sim, t) => { while (sim.time < t) sim.step(); };
const pieces = (sim) => [...sim.st.islands.values()].filter(i => !i.anchored).map(i => sim.st.massProps(i.chunks).mass).sort((a, b) => b - a);
const standing = (sim) => [...sim.st.islands.values()].filter(i => i.anchored).reduce((m, i) => m + sim.st.massProps(i.chunks).mass, 0);
const total = (sim) => sim.st.chunks.reduce((m, c) => m + c.mass, 0);

// 1. Statics: an L-shaped cantilever off a steel column (bending, shear, torsion)
{
  const st = new Structure(1);
  const b = new Builder(st);
  b.foot('cast').box(0, 0, 0, 1, 4, 1, 'steel');
  b.box(1, 3, 0, 4, 1, 1, 'slab').box(4, 3, 1, 1, 1, 3, 'slab');
  st.finalize();
  new StressSolver(st).solve([...st.islands.values()][0]);
  const w = st.chunks.find(c => c.mat === 'slab').mass * GRAVITY;
  const root = st.bonds.find(x => x.b >= 0 && st.chunks[x.a].ix === 0 && st.chunks[x.a].iy === 3 && st.chunks[x.b].ix === 1);
  let T = 0; for (let k = 1; k <= 3; k++) T += w * k;
  let Mx = 0; for (let i = 0; i < 4; i++) Mx += w * (i + 0.5);
  Mx += 3 * w * 3.5;
  const M = Math.hypot(Mx, 0);
  check('3D cantilever: root shear equals the weight beyond it', Math.abs(root.V - 7 * w) / (7 * w) < 0.01, `V=${(root.V / 1000).toFixed(2)} kN, expected ${(7 * w / 1000).toFixed(2)}`);
  check('3D cantilever: root torsion from the side arm', Math.abs(root.T - T) / T < 0.01, `T=${(root.T / 1000).toFixed(2)} kN·m, expected ${(T / 1000).toFixed(2)}`);
  check('3D cantilever: root bending moment', Math.abs(root.M - M) / M < 0.01, `M=${(root.M / 1000).toFixed(2)} kN·m, expected ${(M / 1000).toFixed(2)}`);
  let Ny = 0; for (const x of st.bonds) if (x.b === GROUND) Ny += -x.N;
  const W = st.totalWeight(st.chunks.map(c => c.id));
  check('3D ground reaction equals weight', Math.abs(Ny - W) / W < 0.01, `${(Ny / 1000).toFixed(1)} kN vs ${(W / 1000).toFixed(1)} kN`);
}

// 2. every preset stands
for (const p of ['house', 'office', 'crane', 'tower', 'site']) {
  const sim = new Simulation(p);
  upTo(sim, 3);
  const c = sim.counts();
  check(`3D ${p}: stands for 3 s`, c.falling === 0 && sim.stats.bondsBroken === 0, `falling=${c.falling}, broken=${sim.stats.bondsBroken}, chunks=${c.alive}`);
}

// 3. crane: losing the counterweight tips it; losing the load does not
{
  const sim = new Simulation('crane');
  const isl0 = [...sim.st.islands.values()].find(i => i.anchored);
  const before = sim.islandSummary(isl0);
  sim.removeChunks(sim.st.chunks.filter(c => c.tag === 'crane-counterweight'));
  const isl1 = sim.st.islands.get(sim.st.chunks.find(c => c.mat === 'lattice').island);
  const after = sim.islandSummary(isl1);
  check('3D crane: COM leaves the footprint without the counterweight', before.margin > 0.3 && polygonMargin(before.poly, after.x, after.z) < 0, `margin ${before.margin.toFixed(2)} m -> ${polygonMargin(before.poly, after.x, after.z).toFixed(2)} m`);
  upTo(sim, 8);
  const mast = sim.st.chunks.find(c => c.mat === 'lattice' && c.iy === 20);
  const p = sim.chunkPos(mast);
  check('3D crane: tips over', Math.hypot(p.x - mast.x, p.y - mast.y) > 4, `mast point moved ${Math.hypot(p.x - mast.x, p.y - mast.y).toFixed(1)} m`);
}
{
  const sim = new Simulation('crane');
  sim.removeChunks([sim.st.chunks.filter(c => c.tag === 'crane-cable')[5]]);
  upTo(sim, 4);
  check('3D crane: survives losing its load', sim.st.islands.get(sim.st.chunks.find(c => c.mat === 'lattice' && c.iy === 20).island).anchored);
}

// 4. undermined house falls as one body, damaged not scattered
{
  const sim = new Simulation('house');
  const upper = sim.st.chunks.filter(c => c.iy >= 1).reduce((m, c) => m + c.mass, 0);
  sim.removeChunks(sim.st.chunks.filter(c => c.tag === 'house-base'));
  upTo(sim, 0.4);
  const f = pieces(sim)[0] / upper;
  upTo(sim, 4);
  const l = pieces(sim)[0] / upper;
  check('3D house: undermined, falls as one piece', f > 0.9, `${(f * 100).toFixed(0)}% in one body`);
  check('3D house: after landing it is damaged, not scattered', l > 0.6, `largest piece ${(l * 100).toFixed(0)}%`);
}

// 5. office cut at the 3rd floor: everything above falls as one block
{
  const sim = new Simulation('office');
  const upper = sim.st.chunks.filter(c => c.iy >= 10).reduce((m, c) => m + c.mass, 0);
  sim.removeChunks(sim.st.chunks.filter(c => c.tag === 'office-storey'));
  upTo(sim, 0.4);
  const f = pieces(sim)[0] / upper;
  check('3D office: storey cut, upper floors fall as one block', f > 0.9, `${(f * 100).toFixed(0)}% in one body`);
}

// 6. office column line: the bay above drops as a frame, the rest stands
{
  const sim = new Simulation('office');
  const T = total(sim);
  sim.removeChunks(sim.st.chunks.filter(c => c.tag === 'office-column'));
  upTo(sim, 1.5);
  const p = pieces(sim);
  check('3D office: lost column line drops a frame as one big piece', p[0] > 50000, `largest ${(p[0] / 1000).toFixed(1)} t`);
  check('3D office: the far bays keep standing', standing(sim) / T > 0.35, `${(100 * standing(sim) / T).toFixed(0)}% still standing`);
}

// 7. west columns of one storey removed: the hanging edge leaves as one piece
{
  const sim = new Simulation('office');
  const T = total(sim);
  sim.removeChunks(sim.st.chunks.filter(c => c.tag === 'office-edge'));
  upTo(sim, 1.5);
  const p = pieces(sim);
  const share = p.length ? p[0] / p.reduce((a, m) => a + m, 0) : 0;
  upTo(sim, 4);
  check('3D office: hanging west edge leaves as one main piece', p[0] > 10000 && share > 0.6, `largest ${((p[0] || 0) / 1000).toFixed(1)} t = ${(share * 100).toFixed(0)}% of what fell`);
  check('3D office: the building stays up', standing(sim) / T > 0.6, `${(100 * standing(sim) / T).toFixed(0)}% standing`);
}

// 8. leaning tower tips when its toe is cut
{
  const sim = new Simulation('tower');
  sim.removeChunks(sim.st.chunks.filter(c => c.tag === 'tower-toe'));
  upTo(sim, 8);
  const top = sim.st.chunks.filter(c => !c.destroyed).sort((a, b) => b.iy - a.iy)[0];
  check('3D tower: tips after losing its toe', sim.chunkPos(top).y < 12, `top block height ${sim.chunkPos(top).y.toFixed(1)} m`);
}

// 9. COM bookkeeping matches the rigid body
{
  const sim = new Simulation('house');
  sim.removeChunks(sim.st.chunks.filter(c => c.tag === 'house-base'));
  upTo(sim, 0.8);
  let worst = 0, n = 0;
  for (const isl of sim.st.islands.values()) {
    if (!isl.body || isl.chunks.length < 3) continue;
    const s = sim.islandSummary(isl), b = isl.body;
    worst = Math.max(worst, Math.hypot(s.x - b.pos[0], s.y - b.pos[1], s.z - b.pos[2]), Math.abs(s.mass - b.mass) / s.mass);
    n++;
  }
  check('3D falling bodies: graph COM and mass match the rigid body', n > 0 && worst < 1e-6, `${n} bodies, worst error ${worst.toExponential(1)}`);
}

// 10. debris resting on the roof is fed to the stress solve
{
  const sim = new Simulation('office');
  sim.dropWeight(2.5, 21.2, 2.5, 4000);
  upTo(sim, 4);
  let load = 0;
  for (const c of sim.st.chunks) if (!c.destroyed) load -= c.ext[1];
  const W = 4000 * GRAVITY;
  check('3D resting debris shows up as roof load', Math.abs(load - W) / W < 0.1, `${(load / 1000).toFixed(1)} kN vs ${(W / 1000).toFixed(1)} kN`);
}

// 11. solver speed on the office (member-level graph)
{
  const sim = new Simulation('office');
  const isl = [...sim.st.islands.values()].find(i => i.anchored);
  for (let i = 0; i < 2; i++) { sim.solver.memberQ.clear(); sim.solver.solve(isl); }
  const t0 = performance.now();
  sim.solver.memberQ.clear();
  sim.solver.solve(isl);
  const ms = performance.now() - t0;
  check('3D office cold stress solve under 60 ms', ms < 60, `${ms.toFixed(1)} ms, ${sim.solver.lastIters} iterations, ${isl.chunks.length} blocks`);
}

console.log(failed ? `\n${failed} check(s) failed` : '\nAll checks passed');
process.exit(failed ? 1 : 0);
