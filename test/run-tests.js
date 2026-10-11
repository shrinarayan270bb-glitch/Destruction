// Headless checks for the destruction core. Run: npm test
import * as planck from 'planck';
import { Structure, Builder, GROUND } from '../src/graph.js';
import { StressSolver } from '../src/stress.js';
import { Simulation } from '../src/sim.js';
import { GRAVITY } from '../src/materials.js';

let failed = 0;
function check(name, ok, detail = '') {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? '  — ' + detail : ''}`);
  if (!ok) failed++;
}
const run = (sim, seconds) => { for (let i = 0; i < seconds * 60; i++) sim.step(); };
const anchoredWith = (sim, pred) => [...sim.st.islands.values()].filter(i => i.anchored && i.chunks.some(id => pred(sim.st.chunks[id])));
const fallingMass = (sim) => [...sim.st.islands.values()].filter(i => !i.anchored).reduce((m, i) => m + sim.st.massProps(i.chunks).mass, 0);

// 1. Cantilever: steel wall with a concrete beam sticking out. Root moment
//    must match statics: M = w * l^2 / 2
{
  const st = new Structure(0.5, 0.5);
  const b = new Builder(st);
  b.foot('cast').rect(0, 0, 1, 6, 'steel');
  b.rect(1, 5, 8, 1, 'concrete');
  st.finalize();
  const solver = new StressSolver(st);
  solver.solveIsland([...st.islands.values()][0]);
  const root = st.bonds.find(bd => st.chunks[bd.a].ix === 0 && st.chunks[bd.a].iy === 5 && bd.b !== GROUND && st.chunks[bd.b].ix === 1);
  const w = st.chunks.find(c => c.mat === 'concrete').mass * GRAVITY; // per cell
  const n = 8, s = 0.5;
  // moment of 8 cells about the root face; cell i centre at (i + 0.5) * s
  let Mexp = 0; for (let i = 0; i < n; i++) Mexp += w * (i + 0.5) * s;
  const Vexp = w * n;
  check('cantilever root moment matches statics', Math.abs(Math.abs(root.M) - Mexp) / Mexp < 0.01, `M=${Math.abs(root.M).toFixed(0)} N·m, expected ${Mexp.toFixed(0)}`);
  check('cantilever root shear matches statics', Math.abs(Math.abs(root.V) - Vexp) / Vexp < 0.01, `V=${Math.abs(root.V).toFixed(0)} N, expected ${Vexp.toFixed(0)}`);
  // total ground reaction equals total weight
  let Ny = 0;
  for (const bd of st.bonds) if (bd.b === GROUND) Ny += -bd.N;
  const W = st.totalWeight(st.chunks.map(c => c.id));
  check('ground reaction equals weight', Math.abs(Ny - W) / W < 0.01, `${Ny.toFixed(0)} N vs ${W.toFixed(0)} N`);
}

// 2. Every preset stands on its own
for (const p of ['house', 'office', 'crane', 'tower', 'site']) {
  const sim = new Simulation(planck, p);
  run(sim, 3);
  const c = sim.counts();
  check(`${p}: stands for 3 s`, c.falling === 0 && sim.stats.bondsBroken === 0, `falling=${c.falling}, broken=${sim.stats.bondsBroken}, chunks=${c.alive}`);
}

// 3. Crane: remove the counterweight, the crane tips over (COM leaves the base)
{
  const sim = new Simulation(planck, 'crane');
  const isl = anchoredWith(sim, c => c.mat === 'lattice')[0];
  const before = sim.islandSummary(isl);
  for (const c of sim.st.chunks) if (c.mat === 'concrete' && c.iy > 10) sim.drill(c.x, c.y);
  const after = sim.islandSummary(isl);
  check('crane: COM moves toward the jib when counterweight is lost', after.x > before.x + 0.5, `COM x ${before.x.toFixed(2)} -> ${after.x.toFixed(2)} m, base ${before.span.lo.toFixed(2)}..${before.span.hi.toFixed(2)}`);
  run(sim, 6);
  const mast = sim.st.chunks.find(c => c.mat === 'lattice' && c.iy === 30);
  const p = sim.chunkPos(mast);
  check('crane: topples after losing counterweight', !sim.st.islands.get(mast.island).anchored && Math.abs(p.a) > 0.3, `mast angle ${(p.a * 180 / Math.PI).toFixed(0)}°`);
}

// 4. Crane: cut the hook cable, crane stays up
{
  const sim = new Simulation(planck, 'crane');
  const cable = sim.st.chunks.find(c => c.mat === 'cable' && c.iy === 30);
  sim.drill(cable.x, cable.y);
  run(sim, 4);
  const mast = sim.st.chunks.find(c => c.mat === 'lattice' && c.iy === 30);
  check('crane: survives losing its load', sim.st.islands.get(mast.island).anchored, `falling mass ${(fallingMass(sim) / 1000).toFixed(1)} t`);
}

// 5. Office: knock out an interior column line on the ground floor → progressive collapse
{
  const sim = new Simulation(planck, 'office');
  for (let y = 0; y < 6; y++) sim.drill(9 * 0.5 + 0.25, y * 0.5 + 0.25);
  run(sim, 6);
  const fm = fallingMass(sim);
  check('office: losing a column collapses floors above', fm > 20000, `falling mass ${(fm / 1000).toFixed(1)} t, bonds broken ${sim.stats.bondsBroken}`);
}

// 6. Leaning tower: cut the low corner, COM leaves the base, it tips
{
  const sim = new Simulation(planck, 'tower');
  for (const c of sim.st.chunks) if (c.ix >= 4 && c.iy <= 1 && !c.destroyed) sim.drill(c.x, c.y);
  run(sim, 6);
  const top = sim.st.chunks.find(c => c.iy === 35 && !c.destroyed);
  const p = sim.chunkPos(top);
  check('tower: tips after losing its low corner', p.y < 12, `top chunk height ${p.y.toFixed(1)} m`);
}

// 7. COM bookkeeping matches the physics engine for a falling island
{
  const sim = new Simulation(planck, 'house');
  sim.explode(1, 1.5, 2.2, 400);
  sim.explode(10.5, 1.5, 2.2, 400);
  run(sim, 0.5);
  let worst = 0, n = 0;
  for (const isl of sim.st.islands.values()) {
    if (!isl.body || isl.chunks.length < 3) continue;
    const s = sim.islandSummary(isl);
    const wc = isl.body.getWorldCenter();
    worst = Math.max(worst, Math.hypot(s.x - wc.x, s.y - wc.y));
    worst = Math.max(worst, Math.abs(s.mass - isl.body.getMass()) / s.mass);
    n++;
  }
  check('falling islands: our COM and mass match planck', n > 0 && worst < 1e-3, `${n} islands, worst error ${worst.toExponential(1)}`);
}

// 8. Debris landing on a slab is fed back as load to the stress solver
{
  const sim = new Simulation(planck, 'office');
  const slab = sim.st.chunks.find(c => c.mat === 'concrete' && c.iy === 41 && c.ix === 14);
  const roofUtil = () => Math.max(...sim.st.bonds.filter(bd => !bd.broken && sim.st.chunks[bd.a].iy === 41 && bd.b >= 0 && sim.st.chunks[bd.b].iy === 41 && sim.st.chunks[bd.a].ix < 19).map(bd => bd.util));
  const before = roofUtil();
  sim.dropWeight(slab.x - 0.6, slab.y + 0.8, 1500);
  sim.dropWeight(slab.x + 0.6, slab.y + 0.8, 1500);
  run(sim, 4);
  let load = 0;
  for (const c of sim.st.chunks) if (!c.destroyed) load += -c.ext[1];
  const W = 3000 * GRAVITY;
  check('resting debris shows up as slab load', Math.abs(load - W) / W < 0.1 && sim.counts().falling === 0, `${(load / 1000).toFixed(1)} kN on the roof, weights weigh ${(W / 1000).toFixed(1)} kN`);
  const after = roofUtil();
  check('roof slab stress rises under the load', after > before * 1.5, `max roof utilisation ${before.toFixed(2)} -> ${after.toFixed(2)}`);
}

// 10. Damaged supports: the part above falls as ONE body and is only damaged
//     where it lands, instead of scattering.
const pieces = (sim) => [...sim.st.islands.values()].filter(i => !i.anchored)
  .map(i => sim.st.massProps(i.chunks).mass).sort((a, b) => b - a);
const upTo = (sim, t) => { while (sim.time < t) sim.step(); };
{
  const sim = new Simulation(planck, 'house');
  const upper = sim.st.chunks.filter(c => c.iy >= 3).reduce((m, c) => m + c.mass, 0);
  sim.removeChunks(sim.st.chunks.filter(c => c.tag === 'house-base'));
  upTo(sim, 0.3);
  const falling = pieces(sim)[0] / upper;
  upTo(sim, 4);
  const landed = pieces(sim)[0] / upper;
  check('house: undermined, the upper house falls as one piece', falling > 0.9, `${(falling * 100).toFixed(0)}% of it in one body`);
  check('house: after landing it is damaged, not scattered', landed > 0.6, `largest piece ${(landed * 100).toFixed(0)}% of the upper house`);
}
{
  const sim = new Simulation(planck, 'office');
  const upper = sim.st.chunks.filter(c => c.iy >= 18).reduce((m, c) => m + c.mass, 0);
  sim.removeChunks(sim.st.chunks.filter(c => c.tag === 'office-storey'));
  upTo(sim, 0.3);
  const falling = pieces(sim)[0] / upper;
  check('office: storey cut, everything above falls as one piece', falling > 0.9, `${(falling * 100).toFixed(0)}% of the upper floors in one body`);
}
{
  const sim = new Simulation(planck, 'office');
  sim.removeChunks(sim.st.chunks.filter(c => c.tag === 'office-column'));
  upTo(sim, 0.3);
  const p = pieces(sim).filter(m => m >= 1000);
  check('office: missing column drops its bay as one frame, not floor by floor', p.length === 1 && p[0] > 30000, `${p.length} piece(s) over 1 t, largest ${(p[0] / 1000).toFixed(1)} t`);
}
{
  const sim = new Simulation(planck, 'office');
  const total = sim.st.chunks.reduce((m, c) => m + c.mass, 0);
  const t = sim.st.chunks.filter(c => c.tag === 'office-corner');
  sim.explode(t.reduce((a, c) => a + c.x, 0) / t.length, t.reduce((a, c) => a + c.y, 0) / t.length, 3, 400);
  upTo(sim, 0.3);
  const p = pieces(sim);
  upTo(sim, 4);
  let standing = 0;
  for (const i of sim.st.islands.values()) if (i.anchored) standing += sim.st.massProps(i.chunks).mass;
  const share = p[0] / p.reduce((a, m) => a + m, 0);
  check('office: blasted corner breaks away as one main piece', p[0] > 10000 && share > 0.7, `largest ${(p[0] / 1000).toFixed(1)} t = ${(share * 100).toFixed(0)}% of what fell (the rest is blast fragments)`);
  check('office: the rest keeps standing', standing / total > 0.7, `${(100 * standing / total).toFixed(0)}% still standing`);
}

// 9. Solver speed
{
  const sim = new Simulation(planck, 'office');
  const isl = [...sim.st.islands.values()].find(i => i.anchored && i.chunks.length > 100);
  for (const c of sim.st.chunks) { c.qx = c.qy = c.qt = 0; }
  const t0 = performance.now();
  for (let i = 0; i < 5; i++) { for (const c of sim.st.chunks) { c.qx = c.qy = c.qt = 0; } sim.solver.solveIsland(isl); }
  const ms = (performance.now() - t0) / 5;
  check('office cold solve under 25 ms', ms < 25, `${ms.toFixed(1)} ms, ${sim.solver.lastIters} CG iterations, ${isl.chunks.length} chunks`);
}

console.log(failed ? `\n${failed} check(s) failed` : '\nAll checks passed');
process.exit(failed ? 1 : 0);
