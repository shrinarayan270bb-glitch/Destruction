// Simulation: ties the support graph + stress solver to a rigid-body engine
// (planck.js, a Box2D port). Anchored islands live as fixtures on one static
// body; an island that loses its path to the ground becomes a dynamic body
// carrying its own mass, centre of mass and inertia.
//
// Per fixed step:
//   1. apply queued player actions (blast / drill / weight)
//   2. physics step, collecting contact impulses
//   3. impact damage + debris loads on standing chunks
//   4. split islands whose bonds broke, spawn falling bodies
//   5. time-sliced stress solve on dirty anchored islands, break overloads

import { GROUND } from './graph.js';
import { MATERIALS, GRAVITY } from './materials.js';
import { StressSolver } from './stress.js';
import { buildPreset } from './structures.js';

const STEP = 1 / 60;
const BREAKS_PER_SOLVE = 6;
const MAX_BODIES = 320;

export class Simulation {
  constructor(planck, preset = 'site') {
    this.pl = planck;
    this.settings = {
      stressBudgetMs: 6,
      impactScale: 1,
      impactSpeed: 3.0,     // m/s approach speed before impacts do damage
      timeScale: 1,
      paused: false,
    };
    this.reset(preset);
  }

  reset(preset = this.preset) {
    const pl = this.pl;
    this.preset = preset;
    this.st = buildPreset(preset);
    this.solver = new StressSolver(this.st);
    this.world = new pl.World({ gravity: pl.Vec2(0, -GRAVITY) });
    this.time = 0;
    this.acc = 0;
    this.events = [];
    this.particles = [];
    this.weights = [];
    this.contacts = [];
    this.loaded = new Set();
    this.topologyVersion = 0;
    this.stats = { solveMs: 0, iters: 0, solves: 0, physicsMs: 0, bondsBroken: 0, chunksDestroyed: 0, queue: 0 };

    const ground = this.world.createBody({ type: 'static' });
    ground.createFixture({ shape: pl.Box(400, 2, pl.Vec2(0, -2)), friction: 0.9, userData: { kind: 'ground' } });
    this.staticBody = this.world.createBody({ type: 'static' });
    for (const c of this.st.chunks) this._addFixture(this.staticBody, c);

    this.world.on('pre-solve', (contact) => this._preSolve(contact));
    this.world.on('post-solve', (contact, impulse) => this._postSolve(contact, impulse));

    // let resting footings settle before the first frame (uplift is silent)
    let guard = 0;
    while (this._dirtyAnchored().length && guard++ < 400) this.stressStep(Infinity, true);
    this.stats.bondsBroken = 0;
    this.events = [];
    this.log(`${this.st.chunks.length} chunks, ${this.st.bonds.length} bonds, ${this.st.islands.size} islands`, 'info');
  }

  // ---- fixtures / bodies ----------------------------------------------------

  _addFixture(body, c) {
    const pl = this.pl;
    const h = this.st.cell / 2 - 0.004;
    const area = 4 * h * h;   // density from the shrunk box so mass stays exact
    c.fixture = body.createFixture({
      shape: pl.Box(h, h, pl.Vec2(c.local.x, c.local.y)),
      density: c.mass / area,
      friction: 0.7,
      restitution: 0.02,
      userData: { kind: 'chunk', id: c.id },
    });
  }

  _removeFixture(c) {
    if (!c.fixture) return;
    c.fixture.getBody().destroyFixture(c.fixture);
    c.fixture = null;
  }

  islandBody(isl) { return isl.anchored ? null : isl.body; }

  chunkPos(c) {
    const isl = this.st.islands.get(c.island);
    if (isl && isl.body) {
      const p = isl.body.getWorldPoint(c.local);
      return { x: p.x, y: p.y, a: isl.body.getAngle() };
    }
    return { x: c.x, y: c.y, a: 0 };
  }

  // ---- actions ---------------------------------------------------------------

  explode(x, y, radius = 3, power = 100) {
    const st = this.st;
    const touched = new Set();
    let destroyed = 0;
    const hits = [];
    for (const c of st.chunks) {
      if (c.destroyed) continue;
      const p = this.chunkPos(c);
      const d = Math.hypot(p.x - x, p.y - y);
      if (d > radius) continue;
      const dmg = power * Math.pow(1 - d / radius, 1.2);
      hits.push([c, dmg]);
    }
    for (const [c, dmg] of hits) {
      if (c.destroyed) continue;
      c.health -= dmg;
      touched.add(c.island);
      if (c.health <= 0) { this._destroyChunk(c, x, y); destroyed++; continue; }
      for (const bi of c.bonds) {
        const b = st.bonds[bi];
        if (b.broken) continue;
        b.integrity -= dmg / (MATERIALS[c.mat].hp * 1.5);
        if (b.integrity <= 0) this._breakBond(b, 'blast');
      }
    }
    this._resolve(touched);

    // push everything that is now dynamic
    for (const isl of st.islands.values()) {
      if (!isl.body) continue;
      for (const id of isl.chunks) {
        const c = st.chunks[id];
        const p = this.chunkPos(c);
        const dx = p.x - x, dy = p.y - y, d = Math.hypot(dx, dy);
        const R = radius * 1.6;
        if (d > R || d < 1e-4) continue;
        const dv = power * 0.12 * (1 - d / R);
        isl.body.applyLinearImpulse(this.pl.Vec2(dx / d * dv * c.mass, dy / d * dv * c.mass), this.pl.Vec2(p.x, p.y), true);
      }
    }
    for (const w of this.weights) {
      const p = w.getWorldCenter();
      const dx = p.x - x, dy = p.y - y, d = Math.hypot(dx, dy), R = radius * 1.6;
      if (d < R && d > 1e-4) w.applyLinearImpulse(this.pl.Vec2(dx / d * power * 0.12 * (1 - d / R) * w.getMass(), dy / d * power * 0.12 * (1 - d / R) * w.getMass()), p, true);
    }
    this.log(`Blast r=${radius.toFixed(1)} m: ${destroyed} chunks destroyed, ${hits.length - destroyed} damaged`, 'blast');
    return destroyed;
  }

  drill(x, y) {
    const c = this.pick(x, y);
    if (!c) return false;
    this._destroyChunk(c, x, y);
    this._resolve(new Set([c.island]));
    this.log(`Removed ${MATERIALS[c.mat].label.toLowerCase()} chunk (${c.mass.toFixed(0)} kg)`, 'blast');
    return true;
  }

  // Remove specific chunks outright (scenarios, scripted demolition)
  removeChunks(list) {
    const touched = new Set();
    let mass = 0;
    for (const c of list) {
      if (c.destroyed) continue;
      touched.add(c.island);
      mass += c.mass;
      const p = this.chunkPos(c);
      this._destroyChunk(c, p.x, p.y - 1);
    }
    this._resolve(touched);
    this.log(`Removed ${list.length} chunk${list.length === 1 ? '' : 's'} (${fmtMass(mass)})`, 'blast');
  }

  dropWeight(x, y, mass = 2500) {
    const pl = this.pl;
    const body = this.world.createBody({ type: 'dynamic', position: pl.Vec2(x, y) });
    body.createFixture({ shape: pl.Box(0.5, 0.5), density: mass, friction: 0.8, userData: { kind: 'weight' } });
    this.weights.push(body);
    if (this.weights.length > 24) this.world.destroyBody(this.weights.shift());
    this.log(`Dropped a ${(mass / 1000).toFixed(1)} t weight`, 'info');
  }

  pick(x, y) {
    const st = this.st;
    const h = st.cell / 2;
    // standing chunks: direct grid lookup
    const id = st._grid.get(Math.floor(x / st.cell) + ',' + Math.floor(y / st.cell));
    if (id !== undefined) {
      const c = st.chunks[id];
      if (!c.destroyed && st.islands.get(c.island)?.anchored) return c;
    }
    for (const isl of st.islands.values()) {
      if (!isl.body) continue;
      const lp = isl.body.getLocalPoint(this.pl.Vec2(x, y));
      for (const cid of isl.chunks) {
        const c = st.chunks[cid];
        if (Math.abs(lp.x - c.local.x) <= h && Math.abs(lp.y - c.local.y) <= h) return c;
      }
    }
    return null;
  }

  // ---- graph mutations --------------------------------------------------------

  _destroyChunk(c, bx, by) {
    if (c.destroyed) return;
    c.destroyed = true;
    for (const bi of c.bonds) this.st.bonds[bi].broken = true;
    const p = this.chunkPos(c);
    this._removeFixture(c);
    this.stats.chunksDestroyed++;
    this._spawnDust(p.x, p.y, MATERIALS[c.mat].color, bx, by, 7);
    this.topologyVersion++;
  }

  _breakBond(b, cause) {
    if (b.broken) return;
    b.broken = true;
    b.cause = cause;
    this.stats.bondsBroken++;
    this.topologyVersion++;
    const st = this.st;
    if (cause === 'stress') {
      const where = b.b === GROUND ? 'footing' : `${MATERIALS[st.chunks[b.a].mat].label.toLowerCase()} joint`;
      this._lastStressBreak = `${where} failed in ${b.mode} (${(b.util * 100).toFixed(0)}% of capacity)`;
    }
    this._spawnDust(b.cx, b.cy, '#8a8378', null, null, 2);
  }

  // Re-run connectivity on touched islands and create/destroy bodies.
  _resolve(touchedIds) {
    const st = this.st;
    for (const iid of touchedIds) {
      const isl = st.islands.get(iid);
      if (!isl) continue;
      const res = st.splitIsland(isl);
      if (res.kept) {
        if (isl.anchored) isl.dirty = true;
        else if (isl.body && isl.chunks.length === 0) { this.world.destroyBody(isl.body); isl.body = null; }
        continue;
      }
      const old = res.old;
      for (const ni of res.created) {
        if (ni.anchored) { ni.dirty = true; continue; }
        this._makeDynamic(ni, old);
      }
      if (old.body) this.world.destroyBody(old.body);
      // drop empty islands (every chunk destroyed)
      for (const ni of res.created) if (ni.chunks.length === 0) st.islands.delete(ni.id);
    }
  }

  _makeDynamic(isl, from) {
    const pl = this.pl;
    const st = this.st;
    const src = from.body;
    const body = this.world.createBody({
      type: 'dynamic',
      position: src ? src.getPosition() : pl.Vec2(0, 0),
      angle: src ? src.getAngle() : 0,
    });
    for (const id of isl.chunks) {
      const c = st.chunks[id];
      if (c.fixture) this._removeFixture(c);
      if (!src) c.local = { x: c.x, y: c.y };
      this._addFixture(body, c);
      c.ext[0] = c.ext[1] = c.ext[2] = 0;
      this.loaded.delete(id);
    }
    if (src) {
      body.setLinearVelocity(src.getLinearVelocityFromWorldPoint(body.getWorldCenter()));
      body.setAngularVelocity(src.getAngularVelocity());
    }
    isl.body = body;
    isl.born = this.time;
    if (!src && isl.chunks.length > 2) {
      const mp = st.massProps(isl.chunks);
      this.log(`Collapse: ${isl.chunks.length} chunks (${fmtMass(mp.mass)}) lost support`, 'collapse');
    }
  }

  _dirtyAnchored() {
    const out = [];
    for (const isl of this.st.islands.values()) if (isl.anchored && isl.dirty) out.push(isl);
    return out;
  }

  // ---- stress -----------------------------------------------------------------

  // Solve dirty anchored islands until the time budget runs out.
  stressStep(budgetMs = this.settings.stressBudgetMs, quiet = false) {
    const t0 = now();
    const queue = this._dirtyAnchored();
    let solved = 0;
    while (queue.length && now() - t0 < budgetMs) {
      const isl = queue.shift();
      if (!this.st.islands.has(isl.id) || !isl.dirty) continue;
      const maxU = this.solver.solveIsland(isl);
      solved++;
      this.stats.iters = this.solver.lastIters;
      if (maxU <= 1) { isl.dirty = false; continue; }
      const over = [];
      for (const id of isl.chunks) {
        for (const bi of this.st.chunks[id].bonds) {
          const b = this.st.bonds[bi];
          if (!b.broken && b.a === id && b.util > 1 && b.util >= maxU * 0.85) over.push(b);
        }
      }
      over.sort((p, q) => q.util - p.util);
      for (const b of over.slice(0, BREAKS_PER_SOLVE)) this._breakBond(b, 'stress');
      if (!quiet && this._lastStressBreak) this.log(cap(this._lastStressBreak), 'stress');
      this._lastStressBreak = null;
      this._resolve(new Set([isl.id]));
    }
    this.stats.solveMs = now() - t0;
    this.stats.solves = solved;
    this.stats.queue = this._dirtyAnchored().length;
  }

  // ---- contacts -----------------------------------------------------------------

  _preSolve(contact) {
    const fa = contact.getFixtureA(), fb = contact.getFixtureB();
    const ba = fa.getBody(), bb = fb.getBody();
    if (!ba.isDynamic() && !bb.isDynamic()) return;
    const wm = contact.getWorldManifold(null);
    if (!wm) return;
    let approach = 0;
    const n = wm.normal;
    for (let i = 0; i < contact.getManifold().pointCount; i++) {
      const p = wm.points[i];
      const va = ba.getLinearVelocityFromWorldPoint(p), vb = bb.getLinearVelocityFromWorldPoint(p);
      approach = Math.max(approach, -((vb.x - va.x) * n.x + (vb.y - va.y) * n.y));
    }
    contact.__approach = approach;
  }

  _postSolve(contact, impulse) {
    const fa = contact.getFixtureA(), fb = contact.getFixtureB();
    const ua = fa.getUserData(), ub = fb.getUserData();
    const ca = ua && ua.kind === 'chunk' ? ua.id : -1;
    const cb = ub && ub.kind === 'chunk' ? ub.id : -1;
    if (ca < 0 && cb < 0) return;
    const count = contact.getManifold().pointCount;
    if (!count) return;
    const wm = contact.getWorldManifold(null);
    let jn = 0, jt = 0, px = 0, py = 0;
    for (let i = 0; i < count; i++) {
      jn += impulse.normalImpulses[i];
      jt += impulse.tangentImpulses[i];
      px += wm.points[i].x / count; py += wm.points[i].y / count;
    }
    this.contacts.push({
      ca, cb, jn, jt, px, py, nx: wm.normal.x, ny: wm.normal.y,
      approach: contact.__approach || 0,
      staticA: !fa.getBody().isDynamic(), staticB: !fb.getBody().isDynamic(),
    });
  }

  _processContacts() {
    const st = this.st;
    const touched = new Set();
    const vT = this.settings.impactSpeed;
    const accum = new Map();
    for (const k of this.contacts) {
      // impact damage
      if (k.approach > vT && k.jn > 0) {
        const severity = k.jn * (1 - vT / k.approach) * this.settings.impactScale;
        for (const cid of [k.ca, k.cb]) {
          if (cid < 0) continue;
          const c = st.chunks[cid];
          if (c.destroyed) continue;
          for (const bi of c.bonds) {
            const b = st.bonds[bi];
            if (b.broken) continue;
            b.integrity -= severity / b.toughness;
            if (b.integrity <= 0) { this._breakBond(b, 'impact'); touched.add(c.island); }
          }
          if (st.islands.get(c.island)?.anchored) touched.add(c.island);
        }
      }
      // debris resting on standing structure becomes a load for the stress solver
      // normal points A -> B; impulse on B is +P, on A is -P
      const tx = k.ny, ty = -k.nx;
      const Px = k.jn * k.nx + k.jt * tx, Py = k.jn * k.ny + k.jt * ty;
      if (k.ca >= 0 && k.staticA && !k.staticB) addLoad(accum, k.ca, -Px / STEP, -Py / STEP, k.px, k.py);
      if (k.cb >= 0 && k.staticB && !k.staticA) addLoad(accum, k.cb, Px / STEP, Py / STEP, k.px, k.py);
    }
    this.contacts.length = 0;

    // smooth loads, decay the ones that went away, flag islands to re-solve
    for (const id of accum.keys()) this.loaded.add(id);
    for (const id of [...this.loaded]) {
      const c = st.chunks[id];
      if (c.destroyed) { this.loaded.delete(id); continue; }
      const a = accum.get(id) || [0, 0, 0, 0, 0];
      const fx = a[0], fy = a[1];
      const tq = (a[3] - c.x) * fy - (a[4] - c.y) * fx;   // torque about chunk centre
      const k = 0.12;
      c.ext[0] += (fx - c.ext[0]) * k;
      c.ext[1] += (fy - c.ext[1]) * k;
      c.ext[2] += (tq - c.ext[2]) * k;
      const d = Math.hypot(c.ext[0] - c.extSolved[0], c.ext[1] - c.extSolved[1]);
      if (d > Math.max(800, 0.15 * Math.hypot(c.extSolved[0], c.extSolved[1]))) {
        const isl = st.islands.get(c.island);
        if (isl && isl.anchored) isl.dirty = true;
      }
      if (!accum.has(id) && Math.hypot(c.ext[0], c.ext[1]) < 50) {
        c.ext[0] = c.ext[1] = c.ext[2] = 0;
        this.loaded.delete(id);
        const isl = st.islands.get(c.island);
        if (isl && isl.anchored && Math.hypot(c.extSolved[0], c.extSolved[1]) > 50) isl.dirty = true;
      }
    }
    this._resolve(touched);
  }

  // ---- main loop ------------------------------------------------------------------

  update(dtReal) {
    if (this.settings.paused) return;
    this.acc += Math.min(dtReal, 0.1) * this.settings.timeScale;
    let steps = 0;
    const t0 = now();
    while (this.acc >= STEP && steps < 3) {
      this.step();
      this.acc -= STEP;
      steps++;
    }
    if (steps === 3) this.acc = 0;
    this.stats.physicsMs = now() - t0 - this.stats.solveMs;
    this._updateParticles(dtReal * this.settings.timeScale);
  }

  step() {
    this.world.step(STEP, 8, 3);
    this.time += STEP;
    this._processContacts();
    this.stressStep();
    this._cleanup();
  }

  _cleanup() {
    const st = this.st;
    const dyn = [];
    for (const isl of st.islands.values()) {
      if (!isl.body) continue;
      if (isl.body.getPosition().y < -60) { this._removeIsland(isl); continue; }
      dyn.push(isl);
    }
    if (dyn.length > MAX_BODIES) {
      dyn.sort((a, b) => a.chunks.length - b.chunks.length || a.born - b.born);
      for (const isl of dyn.slice(0, dyn.length - MAX_BODIES)) this._removeIsland(isl);
    }
  }

  _removeIsland(isl) {
    for (const id of isl.chunks) { this.st.chunks[id].destroyed = true; this.st.chunks[id].fixture = null; }
    if (isl.body) this.world.destroyBody(isl.body);
    this.st.islands.delete(isl.id);
  }

  // ---- particles (cosmetic only, never simulated on the server in a real game) --

  _spawnDust(x, y, color, bx, by, n) {
    if (this.particles.length > 900) return;
    for (let i = 0; i < n; i++) {
      let vx = (Math.random() - 0.5) * 4, vy = Math.random() * 3;
      if (bx !== null && bx !== undefined) {
        const dx = x - bx, dy = y - by, d = Math.hypot(dx, dy) || 1;
        vx += dx / d * 6 * Math.random(); vy += dy / d * 6 * Math.random();
      }
      this.particles.push({ x, y, vx, vy, life: 1 + Math.random() * 0.8, color, s: 0.06 + Math.random() * 0.12 });
    }
  }

  _updateParticles(dt) {
    const ps = this.particles;
    for (let i = ps.length - 1; i >= 0; i--) {
      const p = ps[i];
      p.vy -= GRAVITY * dt;
      p.x += p.vx * dt; p.y += p.vy * dt;
      if (p.y < 0) { p.y = 0; p.vy *= -0.3; p.vx *= 0.6; }
      p.life -= dt;
      if (p.life <= 0) ps.splice(i, 1);
    }
  }

  // ---- read-outs --------------------------------------------------------------

  log(text, kind = 'info') {
    this.events.unshift({ t: this.time, text, kind });
    if (this.events.length > 40) this.events.pop();
  }

  islandSummary(isl) {
    const st = this.st;
    const mp = isl.body
      ? st.massProps(isl.chunks, c => isl.body.getWorldPoint(c.local))
      : st.massProps(isl.chunks);
    const span = isl.anchored ? st.supportSpan(isl) : null;
    let margin = null;
    if (span) margin = Math.min(mp.x - span.lo, span.hi - mp.x);
    return { ...mp, span, margin };
  }

  counts() {
    let alive = 0, bonds = 0, anchored = 0, falling = 0;
    for (const c of this.st.chunks) if (!c.destroyed) alive++;
    for (const b of this.st.bonds) if (!b.broken) bonds++;
    for (const isl of this.st.islands.values()) isl.anchored ? anchored++ : falling++;
    return { alive, bonds, anchored, falling };
  }
}

function addLoad(map, id, fx, fy, px, py) {
  let a = map.get(id);
  if (!a) { a = [0, 0, 0, 0, 0]; map.set(id, a); }
  // accumulate force and a force-weighted application point
  const w = Math.hypot(fx, fy);
  const tot = a[2] + w;
  if (tot > 0) { a[3] = (a[3] * a[2] + px * w) / tot; a[4] = (a[4] * a[2] + py * w) / tot; }
  a[0] += fx; a[1] += fy; a[2] = tot;
}

function fmtMass(kg) { return kg >= 1000 ? (kg / 1000).toFixed(1) + ' t' : kg.toFixed(0) + ' kg'; }
function cap(s) { return s.charAt(0).toUpperCase() + s.slice(1); }
function now() { return (typeof performance !== 'undefined' ? performance : Date).now(); }

export { fmtMass };
