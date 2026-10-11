// 3D simulation: support graph + RBSM stress + voxel rigid bodies.
//
// Standing islands are cells of the physics engine's static grid. An island
// that loses its path to the ground becomes one rigid body (mass, COM and
// inertia from its voxels). Failure rules (same as the 2D lab):
//   1. Detach, don't shred: the region that loses support leaves as ONE
//      body; only the bonds on its border break.
//   2. Neighbouring regions that would fail in the same event join it.
//   3. Landings crush bonds near the contact with an energy budget.
//   4. Hard landings stress-check the body against its contacts (impact
//      deceleration as body load); it snaps where weak.
//   5. Debris resting on standing structure feeds its weight into the solve.
// Stress work runs as time-sliced jobs so a big solve never stalls a frame.

import { GROUND, polygonMargin } from './graph.js';
import { MATERIALS, GRAVITY } from './materials.js';
import { StressSolver } from './stress.js';
import { buildPreset } from './structures.js';
import { VoxelPhysics } from './physics.js';

const STEP = 1 / 60;
const FRACTURE_J_PER_TOUGHNESS = 3;
const DETACH_BAND = 0.5;
const MAX_BODIES = 220;

export class Simulation {
  constructor(preset = 'site') {
    this.settings = {
      stressBudgetMs: 12,
      fragility: 1,
      impactSpeed: 2.5,
      impactTime: 0.25,
      timeScale: 1,
      paused: false,
    };
    this.reset(preset);
  }

  reset(preset = this.preset) {
    this.preset = preset;
    this.st = buildPreset(preset);
    this.solver = new StressSolver(this.st);
    this.physics = new VoxelPhysics({ cell: this.st.cell, gravity: -GRAVITY });
    this.physics.contactFriction = (c) => this._friction(c);
    for (const c of this.st.chunks) this.physics.addStatic(c.ix, c.iy, c.iz, c.id);
    this.time = 0; this.acc = 0;
    this.events = []; this.particles = []; this.weights = [];
    this.jobs = []; this.jobKeys = new Set();
    this.loaded = new Set();
    this.topologyVersion = 0;
    this.lastContacts = [];
    this.stats = { solveMs: 0, physicsMs: 0, bondsBroken: 0, chunksDestroyed: 0, queue: 0, contacts: 0 };
    // settle resting footings before the first frame (uplift is silent)
    for (let guard = 0; guard < 400; guard++) {
      if (!this._scheduleDirty()) break;
      this._runJobs(Infinity, true);
    }
    this.stats.bondsBroken = 0;
    this.events = [];
    this.log(`${this.st.chunks.length} chunks, ${this.st.bonds.length} bonds, ${this.st.islands.size} islands`, 'info');
  }

  // ---- positions -------------------------------------------------------------

  chunkPos(c) {
    const isl = this.st.islands.get(c.island);
    if (isl && isl.body) { const p = this.physics.toWorld(isl.body, c.x, c.y, c.z); return { x: p[0], y: p[1], z: p[2] }; }
    return { x: c.x, y: c.y, z: c.z };
  }

  islandSummary(isl) {
    const st = this.st;
    const mp = st.massProps(isl.chunks, c => this.chunkPos(c));
    let poly = null, margin = null;
    if (isl.anchored) { poly = st.supportPolygon(isl); margin = poly ? polygonMargin(poly, mp.x, mp.z) : null; }
    return { ...mp, poly, margin };
  }

  counts() {
    let alive = 0, bonds = 0, anchored = 0, falling = 0;
    for (const c of this.st.chunks) if (!c.destroyed) alive++;
    for (const b of this.st.bonds) if (!b.broken) bonds++;
    for (const isl of this.st.islands.values()) isl.anchored ? anchored++ : falling++;
    return { alive, bonds, anchored, falling };
  }

  log(text, kind = 'info') {
    this.events.unshift({ t: this.time, text, kind });
    if (this.events.length > 40) this.events.pop();
  }

  // ---- actions -----------------------------------------------------------------

  explode(x, y, z, radius = 3, power = 160) {
    const st = this.st;
    const touched = new Set();
    const hits = [];
    for (const c of st.chunks) {
      if (c.destroyed) continue;
      const p = this.chunkPos(c);
      const d = Math.hypot(p.x - x, p.y - y, p.z - z);
      if (d <= radius) hits.push([c, power * Math.pow(1 - d / radius, 1.2)]);
    }
    let destroyed = 0;
    for (const [c, dmg] of hits) {
      if (c.destroyed) continue;
      c.health -= dmg;
      touched.add(c.island);
      this._bump(c.island);
      if (c.health <= 0) { this._destroyChunk(c, x, y, z); destroyed++; continue; }
      for (const bi of c.bonds) {
        const b = st.bonds[bi];
        if (b.broken) continue;
        b.integrity -= dmg / (MATERIALS[c.mat].hp * 1.5);
        if (b.integrity <= 0) this._breakBond(b, 'blast');
      }
    }
    this._resolve(touched);
    const R = radius * 1.6;
    for (const isl of st.islands.values()) {
      if (!isl.body) continue;
      for (const id of isl.chunks) {
        const c = st.chunks[id], p = this.chunkPos(c);
        const dx = p.x - x, dy = p.y - y, dz = p.z - z, d = Math.hypot(dx, dy, dz);
        if (d > R || d < 1e-4) continue;
        const dv = power * 0.1 * (1 - d / R) * c.mass;
        this.physics.applyImpulse(isl.body, [dx / d * dv, dy / d * dv, dz / d * dv], [p.x, p.y, p.z]);
      }
    }
    for (const w of this.weights) {
      const dx = w.pos[0] - x, dy = w.pos[1] - y, dz = w.pos[2] - z, d = Math.hypot(dx, dy, dz);
      if (d < R && d > 1e-4) { const dv = power * 0.1 * (1 - d / R) * w.mass; this.physics.applyImpulse(w, [dx / d * dv, dy / d * dv, dz / d * dv], w.pos); }
    }
    this.log(`Blast r=${radius.toFixed(1)} m: ${destroyed} chunks destroyed, ${hits.length - destroyed} damaged`, 'blast');
  }

  drill(c) {
    if (!c || c.destroyed) return false;
    const p = this.chunkPos(c);
    this._bump(c.island);
    this._destroyChunk(c, p.x, p.y + 1, p.z);
    this._resolve(new Set([c.island]));
    this.log(`Removed ${MATERIALS[c.mat].label.toLowerCase()} chunk (${fmtMass(c.mass)})`, 'blast');
    return true;
  }

  removeChunks(list) {
    const touched = new Set();
    let mass = 0;
    for (const c of list) {
      if (c.destroyed) continue;
      touched.add(c.island); this._bump(c.island);
      mass += c.mass;
      const p = this.chunkPos(c);
      this._destroyChunk(c, p.x, p.y + 1, p.z);
    }
    this._resolve(touched);
    this.log(`Removed ${list.length} chunk${list.length === 1 ? '' : 's'} (${fmtMass(mass)})`, 'blast');
  }

  dropWeight(x, y, z, mass = 3000) {
    const ix = Math.floor(x), iy = Math.max(0, Math.floor(y)), iz = Math.floor(z);
    const w = this.physics.createBody([{ id: -2, ix, iy, iz, mass }], null, { weight: true });
    this.weights.push(w);
    if (this.weights.length > 20) this.physics.removeBody(this.weights.shift());
    this.log(`Dropped a ${(mass / 1000).toFixed(1)} t weight`, 'info');
  }

  // ---- graph mutations ---------------------------------------------------------

  _bump(iid) { const isl = this.st.islands.get(iid); if (isl) isl.version++; }

  _destroyChunk(c, bx, by, bz) {
    if (c.destroyed) return;
    const p = this.chunkPos(c);
    const isl = this.st.islands.get(c.island);
    c.destroyed = true;
    for (const bi of c.bonds) this.st.bonds[bi].broken = true;
    if (!isl || !isl.body) this.physics.removeStatic(c.ix, c.iy, c.iz);
    else isl.bodyDirty = true;
    this.stats.chunksDestroyed++;
    this.topologyVersion++;
    this._spawnDust(p.x, p.y, p.z, MATERIALS[c.mat].color, bx, by, bz, 6);
  }

  _breakBond(b, cause) {
    if (b.broken) return;
    b.broken = true;
    b.cause = cause;
    this.stats.bondsBroken++;
    this.topologyVersion++;
    const c = this.st.chunks[b.a];
    this._bump(c.island);
    if (Math.random() < 0.5) { const p = this.chunkPos(c); this._spawnDust(p.x, p.y, p.z, '#8a8378', null, 0, 0, 1); }
  }

  _resolve(touchedIds) {
    const st = this.st;
    for (const iid of touchedIds) {
      const isl = st.islands.get(iid);
      if (!isl) continue;
      const res = st.splitIsland(isl);
      if (res.kept) {
        if (isl.anchored) { isl.dirty = true; continue; }
        if (isl.bodyDirty && isl.body) {
          const old = isl.body;
          if (isl.chunks.length) this._makeDynamic(isl, old, true);
          else isl.body = null;
          this.physics.removeBody(old);
          if (!isl.chunks.length) st.islands.delete(isl.id);
        }
        continue;
      }
      const old = res.old;
      for (const ni of res.created) {
        if (!ni.chunks.length) { st.islands.delete(ni.id); continue; }
        if (ni.anchored) { ni.dirty = true; continue; }
        this._makeDynamic(ni, old.body);
      }
      if (old.body) this.physics.removeBody(old.body);
    }
  }

  _makeDynamic(isl, fromBody, quiet = false) {
    const st = this.st;
    const voxels = [];
    for (const id of isl.chunks) {
      const c = st.chunks[id];
      if (!fromBody) { this.physics.removeStatic(c.ix, c.iy, c.iz); c.freshUntil = this.time + 0.6; }
      c.ext.fill(0);
      this.loaded.delete(id);
      voxels.push({ id, ix: c.ix, iy: c.iy, iz: c.iz, mass: c.mass });
    }
    isl.body = this.physics.createBody(voxels, fromBody, isl);
    isl.bodyDirty = false;
    isl.born = this.time;
    isl.restChecked = false;
    if (!fromBody && !quiet && isl.chunks.length > 2) {
      let m = 0; for (const id of isl.chunks) m += st.chunks[id].mass;
      this.log(`Collapse: ${isl.chunks.length} chunks (${fmtMass(m)}) lost support`, 'collapse');
    }
  }

  // ---- jobs (time-sliced stress work) -------------------------------------------

  _enqueue(key, gen) {
    if (this.jobKeys.has(key)) return;
    this.jobKeys.add(key);
    this.jobs.push({ key, gen });
  }

  _scheduleDirty() {
    let n = 0;
    for (const isl of this.st.islands.values()) {
      if (isl.anchored && isl.dirty && !this.jobKeys.has('s' + isl.id)) { this._enqueue('s' + isl.id, this._standingJob(isl)); n++; }
    }
    return n || this.jobs.length;
  }

  _runJobs(budgetMs, quiet = false) {
    const t0 = now();
    this._quiet = quiet;
    while (this.jobs.length && now() - t0 < budgetMs) {
      const j = this.jobs[0];
      const r = j.gen.next();
      if (r.done) { this.jobs.shift(); this.jobKeys.delete(j.key); }
    }
    this._quiet = false;
    this.stats.solveMs = now() - t0;
    this.stats.queue = this.jobs.length;
  }

  *_standingJob(isl) {
    if (this.st.islands.get(isl.id) !== isl) return;   // replaced while queued
    const v0 = isl.version;
    const valid = () => this.st.islands.get(isl.id) === isl && isl.version === v0;
    isl.dirty = false;
    const maxU = yield* this.solver.solveGen(isl);
    if (!valid()) { if (this.st.islands.get(isl.id) === isl) isl.dirty = true; return; }
    if (maxU <= 1) return;
    const plan = yield* this._failPlan(isl, null);
    if (!valid()) { if (this.st.islands.get(isl.id) === isl) isl.dirty = true; return; }
    this._applyPlan(isl, plan, 'stress');
  }

  *_bodyJob(isl, supports, accel, cause) {
    if (this.st.islands.get(isl.id) !== isl || !isl.body) return;
    const v0 = isl.version;
    const valid = () => this.st.islands.get(isl.id) === isl && isl.version === v0 && isl.body;
    const maxU = yield* this.solver.solveGen(isl, { supports, accel });
    if (!valid() || maxU <= 1) return;
    const plan = yield* this._failPlan(isl, new Set(supports.map(s => s.id)));
    if (!valid()) return;
    this._applyPlan(isl, plan, cause);
  }

  // Region that loses support if the seriously overloaded bonds let go,
  // grown by neighbours that would fail in the same event (standing only).
  *_failPlan(isl, supports) {
    const st = this.st;
    const first = this._looseRegion(isl.chunks, supports);
    if (!first) return null;
    if (first.crack) return first;
    // Grow the seed: release every overloaded bond and take each loose region
    // that is attached (through those bonds) to the seed. Parallel lines of
    // overloaded joints in a floor plate then fall as one panel, not strips,
    // while unrelated overloads elsewhere stay out of it.
    const fall = this._growLoose(isl.chunks, supports, first.fall);
    if (!supports) {
      for (let round = 0; round < 3; round++) {
        const rest = isl.chunks.filter(id => !fall.has(id) && !st.chunks[id].destroyed);
        let grew = false;
        for (const comp of st.components(rest)) {
          if (!comp.anchored) { for (const id of comp.chunks) fall.add(id); grew = true; continue; }
          if (comp.chunks.length < 2) continue;
          yield* this.solver.solveGen({ chunks: comp.chunks });
          // here every overloaded bond counts: whatever would fail once the
          // first region is gone, and is attached to it, goes with it
          const r = this._looseRegion(comp.chunks, null, 0);
          if (r && r.fall && this._touches(r.fall, fall)) { for (const id of r.fall) fall.add(id); grew = true; }
        }
        if (!grew) break;
      }
    }
    return { fall };
  }

  _looseRegion(ids, supports, band = DETACH_BAND) {
    const st = this.st;
    const inSet = new Set(ids);
    const within = (b) => b.b === GROUND || inSet.has(b.b);
    let maxU = 0, worst = null;
    for (const id of ids) for (const bi of st.chunks[id].bonds) {
      const b = st.bonds[bi];
      if (b.broken || b.a !== id || !within(b)) continue;
      if (b.util > maxU) { maxU = b.util; worst = b; }
    }
    if (maxU <= 1) return null;
    const cut = Math.max(1, maxU * band);
    const released = new Set();
    for (const id of ids) for (const bi of st.chunks[id].bonds) {
      const b = st.bonds[bi];
      if (!b.broken && b.a === id && within(b) && b.util >= cut) released.add(b.id);
    }
    const loose = st.components(ids, b => released.has(b.id), supports).filter(c => !c.anchored);
    if (!loose.length) {
      // A connection fails as a unit: crack every overloaded bond of the
      // joint between the two members that holds the worst bond, plus twin
      // joints within 10% of it (a symmetric frame cracks on both sides).
      const jointOf = (b) => b.b === GROUND ? st.chunks[b.a].member + ':g' : Math.min(st.chunks[b.a].member, st.chunks[b.b].member) + ':' + Math.max(st.chunks[b.a].member, st.chunks[b.b].member);
      const joints = new Set([jointOf(worst)]);
      for (const id of ids) for (const bi of st.chunks[id].bonds) {
        const b = st.bonds[bi];
        if (!b.broken && b.a === id && within(b) && b.util >= maxU * 0.9) joints.add(jointOf(b));
      }
      const crack = [];
      for (const id of ids) for (const bi of st.chunks[id].bonds) {
        const b = st.bonds[bi];
        if (!b.broken && b.a === id && within(b) && b.util > 1 && joints.has(jointOf(b))) crack.push(b);
      }
      crack.sort((p, q) => q.util - p.util);
      return { crack };
    }
    const fall = new Set();
    for (const comp of loose) for (const id of comp.chunks) fall.add(id);
    return { fall };
  }

  _growLoose(ids, supports, seed) {
    const st = this.st;
    const inSet = new Set(ids);
    const released = new Set();
    for (const id of ids) for (const bi of st.chunks[id].bonds) {
      const b = st.bonds[bi];
      if (!b.broken && b.a === id && (b.b === GROUND || inSet.has(b.b)) && b.util > 1) released.add(b.id);
    }
    const loose = st.components(ids, b => released.has(b.id), supports).filter(c => !c.anchored);
    const compOf = new Map();
    loose.forEach((c, i) => { for (const id of c.chunks) compOf.set(id, i); });
    const fall = new Set(seed);
    const take = new Set();
    const queue = [];
    for (const id of seed) { const ci = compOf.get(id); if (ci !== undefined && !take.has(ci)) { take.add(ci); queue.push(ci); } }
    while (queue.length) {
      const ci = queue.pop();
      for (const id of loose[ci].chunks) {
        fall.add(id);
        for (const bi of st.chunks[id].bonds) {
          const b = st.bonds[bi];
          if (b.broken || b.b === GROUND) continue;
          const nj = compOf.get(st.other(b, id));
          if (nj !== undefined && !take.has(nj)) { take.add(nj); queue.push(nj); }
        }
      }
    }
    return fall;
  }

  _touches(a, b) {
    for (const id of a) for (const bi of this.st.chunks[id].bonds) {
      const bd = this.st.bonds[bi];
      if (!bd.broken && bd.b !== GROUND && b.has(this.st.other(bd, id))) return true;
    }
    return false;
  }

  _applyPlan(isl, plan, cause) {
    if (!plan) return;
    const st = this.st;
    if (plan.crack) {
      const b = plan.crack[0];
      for (const x of plan.crack) this._breakBond(x, cause);
      if (!this._quiet && cause === 'stress') this.log(cap(`${where(st, b)} cracked in ${b.mode} (${(b.util * 100).toFixed(0)}% of capacity)`), 'stress');
      this._resolve(new Set([isl.id]));
      return;
    }
    let n = 0, worst = null, mass = 0;
    for (const id of plan.fall) {
      mass += st.chunks[id].mass;
      for (const bi of st.chunks[id].bonds) {
        const b = st.bonds[bi];
        if (b.broken) continue;
        const o = b.b === GROUND ? -1 : st.other(b, id);
        if (o >= 0 && plan.fall.has(o)) continue;
        if (!worst || b.util > worst.util) worst = b;
        this._breakBond(b, cause); n++;
      }
    }
    if (!this._quiet && worst) {
      const what = cause === 'impact' ? 'Impact snapped off' : 'Detached';
      this.log(`${what} ${plan.fall.size} chunks (${fmtMass(mass)}) as one piece: ${n} ${where(st, worst)}${n === 1 ? '' : 's'} failed in ${worst.mode}`, 'collapse');
    }
    this._resolve(new Set([isl.id]));
  }

  // ---- contacts ------------------------------------------------------------------

  _fresh(id) { return id >= 0 && this.st.chunks[id].freshUntil > this.time; }

  _friction(c) {
    // a piece that just broke away slides off its crack faces
    if (Math.abs(c.ny) < 0.6 && !c.ground && (this._fresh(c.chunkA) || this._fresh(c.chunkB))) return 0;
    return 0.6;
  }

  _processContacts(contacts) {
    const st = this.st;
    const touched = new Set();
    const vT = this.settings.impactSpeed;
    const accum = new Map();
    const hits = new Map();
    const loadsBy = new Map();
    const addHit = (body, cid, P, c, nSign) => {
      const isl = body && body.userData;
      if (!isl || !isl.chunks || cid < 0) return;
      let h = hits.get(isl.id);
      if (!h) { h = { isl, J: [0, 0, 0], supports: new Map() }; hits.set(isl.id, h); }
      h.J[0] += P[0]; h.J[1] += P[1]; h.J[2] += P[2];
      const prev = h.supports.get(cid);
      if (!prev || c.l[0] > prev.jn) h.supports.set(cid, { p: [c.px, c.py, c.pz], n: [-c.nx * nSign, -c.ny * nSign, -c.nz * nSign], jn: c.l[0] });
    };

    for (const c of contacts) {
      const [n, t1, t2] = c.dirs;
      const P = [n[0] * c.l[0] + t1[0] * c.l[1] + t2[0] * c.l[2], n[1] * c.l[0] + t1[1] * c.l[1] + t2[1] * c.l[2], n[2] * c.l[0] + t1[2] * c.l[1] + t2[2] * c.l[2]];
      addHit(c.A, c.chunkA, P, c, 1);
      if (c.B) addHit(c.B, c.chunkB, [-P[0], -P[1], -P[2]], c, -1);

      if (c.approach > vT && c.l[0] > 0) {
        const E = 0.5 * c.l[0] * (c.approach - vT) * this.settings.fragility;
        const sides = [c.chunkA, c.chunkB].filter(id => id >= 0 && !st.chunks[id].destroyed);
        for (const id of sides) this._crush(st.chunks[id], c.px, c.py, c.pz, E / sides.length, touched);
      }

      // debris weight resting ON standing structure (contact normal mostly vertical)
      const onStatic = !c.B && !c.ground && !c.sleeper && c.chunkB >= 0;
      if (onStatic && Math.abs(c.ny) > 0.7 && !this._fresh(c.chunkA) && c.A.userData) {
        let list = loadsBy.get(c.A);
        if (!list) { list = { sum: 0, items: [] }; loadsBy.set(c.A, list); }
        list.sum += Math.hypot(...P) / STEP;
        list.items.push([c, P]);
      }
    }
    // a body pushes on the structure with at most 1.5x its weight in total
    for (const [body, list] of loadsBy) {
      const f = Math.min(1, 1.5 * body.mass * GRAVITY / (list.sum || 1)) / STEP;
      for (const [c, P] of list.items) {
        addLoad(accum, c.chunkB, -P[0] * f, -P[1] * f, -P[2] * f, c.px, c.py, c.pz);
        st.chunks[c.chunkB].loadBody = body;
      }
    }
    this._resolve(touched);

    // hard landings: stress-check the falling body against its contacts
    const big = [...hits.values()]
      .filter(h => st.islands.get(h.isl.id) === h.isl && h.isl.body && h.isl.chunks.length >= 4)
      .map(h => ({ ...h, dv: Math.hypot(...h.J) / h.isl.body.mass }))
      .filter(h => h.dv > vT)
      .sort((p, q) => q.dv * q.isl.chunks.length - p.dv * p.isl.chunks.length)
      .slice(0, 2);
    for (const h of big) {
      const Jl = Math.hypot(...h.J);
      const a = h.dv / this.settings.impactTime * this.settings.fragility;
      this._queueBodyCheck(h.isl, h.supports, [-h.J[0] / Jl * a, -h.J[1] / Jl * a, -h.J[2] / Jl * a], 'impact');
      h.isl.restChecked = false;
    }
    this._restChecks(hits);

    // smooth loads; sleeping debris keeps its last load, scaled so a body
    // never presses down with more than its own weight
    for (const id of accum.keys()) this.loaded.add(id);
    const down = new Map();
    for (const id of this.loaded) {
      const c = st.chunks[id];
      if (c.loadBody && c.lastRaw) down.set(c.loadBody, (down.get(c.loadBody) || 0) - c.lastRaw[1]);
    }
    for (const id of [...this.loaded]) {
      const c = st.chunks[id];
      if (c.destroyed || st.islands.get(c.island)?.body) { this.loaded.delete(id); continue; }
      if (!accum.has(id) && c.loadBody && !c.loadBody.removed && c.loadBody.sleeping) {
        if (c.lastRaw && !c.frozen) {
          c.frozen = true;
          const k = Math.min(1, c.loadBody.mass * GRAVITY / Math.max(1, down.get(c.loadBody) || 0));
          for (let i = 0; i < 6; i++) c.ext[i] = c.lastRaw[i] * k;
          const isl = st.islands.get(c.island);
          if (isl && isl.anchored) isl.dirty = true;
        }
        continue;
      }
      c.frozen = false;
      const a = accum.get(id);
      const F = [0, 0, 0, 0, 0, 0];
      if (a) {
        F[0] = a[0]; F[1] = a[1]; F[2] = a[2];
        const w = a[9] || 1, rx = a[6] / w - c.x, ry = a[7] / w - c.y, rz = a[8] / w - c.z;
        F[3] = ry * F[2] - rz * F[1]; F[4] = rz * F[0] - rx * F[2]; F[5] = rx * F[1] - ry * F[0];   // torque about chunk centre
        c.lastRaw = F;
      }
      const k = 0.05;
      for (let i = 0; i < 6; i++) c.ext[i] += (F[i] - c.ext[i]) * k;
      const d = Math.hypot(c.ext[0] - c.extSolved[0], c.ext[1] - c.extSolved[1], c.ext[2] - c.extSolved[2]);
      const isl = st.islands.get(c.island);
      if (d > Math.max(800, 0.15 * Math.hypot(c.extSolved[0], c.extSolved[1], c.extSolved[2])) && isl && isl.anchored) isl.dirty = true;
      if (!a && Math.hypot(c.ext[0], c.ext[1], c.ext[2]) < 50) {
        c.ext.fill(0);
        this.loaded.delete(id);
        if (isl && isl.anchored && Math.hypot(c.extSolved[0], c.extSolved[1], c.extSolved[2]) > 50) isl.dirty = true;
      }
    }
  }

  _queueBodyCheck(isl, supportsMap, accelW, cause) {
    const body = isl.body;
    const supports = [];
    for (const [id, sp] of supportsMap) {
      const c = this.st.chunks[id];
      if (c.destroyed || c.island !== isl.id) continue;
      const h = this.physics.toHome(body, sp.p[0], sp.p[1], sp.p[2]);
      const n = this.physics.dirToHome(body, sp.n[0], sp.n[1], sp.n[2]);
      supports.push({ id, x: h[0], y: h[1], z: h[2], nx: n[0], ny: n[1], nz: n[2] });
    }
    if (!supports.length) return;
    const a = this.physics.dirToHome(body, accelW[0], accelW[1], accelW[2]);
    // impact checks replace a pending one for the same body
    const key = 'b' + isl.id;
    const i = this.jobs.findIndex(j => j.key === key);
    if (i > 0) { this.jobs.splice(i, 1); this.jobKeys.delete(key); }
    this._enqueue(key, this._bodyJob(isl, supports, { x: a[0], y: a[1], z: a[2] }, cause));
  }

  // once a fallen piece is at rest, check it can carry itself on what it landed on
  _restChecks(hits) {
    for (const h of hits.values()) {
      const isl = h.isl, body = isl.body;
      if (!body || isl.restChecked || isl.chunks.length < 6 || this.time - isl.born < 0.4) continue;
      if (Math.hypot(...body.v) > 0.4 || Math.hypot(...body.w) > 0.4) continue;
      isl.restChecked = true;
      this._queueBodyCheck(isl, h.supports, [0, -GRAVITY, 0], 'stress');
      return;
    }
  }

  // Spend impact energy on the bonds nearest the contact, nearest first.
  _crush(c, px, py, pz, E, touched) {
    if (E <= 0 || c.destroyed) return;
    const st = this.st;
    const isl = st.islands.get(c.island);
    if (!isl) return;
    const body = isl.body, R = 1.8;
    const near = [], seen = new Set([c.id]), queue = [c.id], seenB = new Set();
    while (queue.length) {
      const id = queue.pop();
      for (const bi of st.chunks[id].bonds) {
        const b = st.bonds[bi];
        if (b.broken || b.b === GROUND || seenB.has(bi)) continue;
        const w = body ? this.physics.toWorld(body, b.cx, b.cy, b.cz) : [b.cx, b.cy, b.cz];
        const d = Math.hypot(w[0] - px, w[1] - py, w[2] - pz);
        if (d > R) continue;
        seenB.add(bi);
        near.push([b, d]);
        const o = st.other(b, id);
        if (!seen.has(o)) { seen.add(o); queue.push(o); }
      }
    }
    near.sort((p, q) => p[1] - q[1]);
    for (const [b, d] of near) {
      const G = b.toughness * FRACTURE_J_PER_TOUGHNESS * (1 + d / 0.5);
      const need = G * b.integrity;
      if (E >= need) { E -= need; this._breakBond(b, 'impact'); touched.add(c.island); }
      else { b.integrity -= E / G; this._bump(c.island); touched.add(c.island); break; }
    }
  }

  // ---- main loop -------------------------------------------------------------------

  update(dtReal) {
    if (this.settings.paused) return;
    this.acc += Math.min(dtReal, 0.1) * this.settings.timeScale;
    let steps = 0;
    while (this.acc >= STEP && steps < 2) { this.step(); this.acc -= STEP; steps++; }
    if (steps === 2) this.acc = Math.min(this.acc, STEP);
    this._updateParticles(dtReal * this.settings.timeScale);
  }

  step() {
    const t0 = now();
    const contacts = this.physics.step(STEP);
    this.stats.contacts = contacts.length;
    this.time += STEP;
    this.stats.physicsMs = now() - t0;
    this._processContacts(contacts);
    this._scheduleDirty();
    this._runJobs(this.settings.stressBudgetMs);
    this._cleanup();
  }

  // run every queued stress job to completion (tests, settling)
  flush() { for (let i = 0; i < 50 && this._scheduleDirty(); i++) this._runJobs(Infinity); }

  _cleanup() {
    const dyn = [];
    for (const isl of this.st.islands.values()) {
      if (!isl.body) continue;
      if (isl.body.pos[1] < -40) { this._removeIsland(isl); continue; }
      dyn.push(isl);
    }
    if (dyn.length > MAX_BODIES) {
      dyn.sort((a, b) => a.chunks.length - b.chunks.length || a.born - b.born);
      for (const isl of dyn.slice(0, dyn.length - MAX_BODIES)) this._removeIsland(isl);
    }
  }

  _removeIsland(isl) {
    for (const id of isl.chunks) this.st.chunks[id].destroyed = true;
    if (isl.body) this.physics.removeBody(isl.body);
    this.st.islands.delete(isl.id);
    this.topologyVersion++;
  }

  // ---- cosmetic dust ------------------------------------------------------------

  _spawnDust(x, y, z, color, bx, by, bz, n) {
    if (this.particles.length > 700) return;
    for (let i = 0; i < n; i++) {
      let vx = (Math.random() - 0.5) * 4, vy = Math.random() * 3, vz = (Math.random() - 0.5) * 4;
      if (bx !== null) {
        const dx = x - bx, dy = y - by, dz = z - bz, d = Math.hypot(dx, dy, dz) || 1;
        vx += dx / d * 6 * Math.random(); vy += dy / d * 6 * Math.random(); vz += dz / d * 6 * Math.random();
      }
      this.particles.push({ x, y, z, vx, vy, vz, life: 1 + Math.random() * 0.8, color, s: 0.12 + Math.random() * 0.2 });
    }
  }

  _updateParticles(dt) {
    const ps = this.particles;
    for (let i = ps.length - 1; i >= 0; i--) {
      const p = ps[i];
      p.vy -= GRAVITY * dt;
      p.x += p.vx * dt; p.y += p.vy * dt; p.z += p.vz * dt;
      if (p.y < 0) { p.y = 0; p.vy *= -0.3; p.vx *= 0.6; p.vz *= 0.6; }
      p.life -= dt;
      if (p.life <= 0) ps.splice(i, 1);
    }
  }
}

function addLoad(map, id, fx, fy, fz, px, py, pz) {
  let a = map.get(id);
  if (!a) { a = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]; map.set(id, a); }
  // force, plus a force-weighted application point (torque is taken later)
  const w = Math.hypot(fx, fy, fz);
  a[0] += fx; a[1] += fy; a[2] += fz;
  a[6] += px * w; a[7] += py * w; a[8] += pz * w; a[9] += w;
}

export function fmtMass(kg) { return kg >= 1000 ? (kg / 1000).toFixed(1) + ' t' : kg.toFixed(0) + ' kg'; }
function where(st, b) { return b.b === GROUND ? 'footing' : `${MATERIALS[st.chunks[b.a].mat].label.toLowerCase()} joint`; }
function cap(s) { return s.charAt(0).toUpperCase() + s.slice(1); }
function now() { return (typeof performance !== 'undefined' ? performance : Date).now(); }
