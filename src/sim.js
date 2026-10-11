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
const FRACTURE_J_PER_TOUGHNESS = 3;
const DETACH_BAND = 0.5;              // bonds within this fraction of the worst fail together   // bond fracture energy (J) per unit toughness
const MAX_BODIES = 320;

export class Simulation {
  constructor(planck, preset = 'site') {
    this.pl = planck;
    this.settings = {
      stressBudgetMs: 6,
      fragility: 1,         // scales contact crushing and impact deceleration
      impactSpeed: 2.5,     // m/s approach speed before impacts do damage
      impactTime: 0.25,     // s over which a landing body decelerates (longer = gentler)
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
    // Debris moves well under a cell (0.5 m) per step, so discrete collision is
    // enough; continuous collision against hundreds of touching fixtures costs
    // 20-40 ms a step during big collapses.
    this.world = new pl.World({ gravity: pl.Vec2(0, -GRAVITY), continuousPhysics: false });
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

  _destroyBody(body) {
    body.__dead = true;   // lets loads that referenced it expire
    this.world.destroyBody(body);
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
    if (this.weights.length > 24) this._destroyBody(this.weights.shift());
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
        else if (isl.body && isl.chunks.length === 0) { this._destroyBody(isl.body); isl.body = null; }
        continue;
      }
      const old = res.old;
      for (const ni of res.created) {
        if (ni.anchored) { ni.dirty = true; continue; }
        this._makeDynamic(ni, old);
      }
      if (old.body) this._destroyBody(old.body);
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
      if (!src) { c.local = { x: c.x, y: c.y }; c.freshUntil = this.time + 0.6; }
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
      const res = this._fail(isl, null, 'stress');
      if (!quiet) this._logFailure(res, 'stress');
      this._resolve(new Set([isl.id]));
    }
    this.stats.solveMs = now() - t0;
    this.stats.solves = solved;
    this.stats.queue = this._dirtyAnchored().length;
  }

  // Failure rule, shared by standing structures and landing bodies.
  //
  // 1. Release every bond that is seriously overloaded (at least half as
  //    overloaded as the worst) and find the region that loses its support:
  //    ground footings, or `supports` for a body resting on something.
  //    If nothing comes loose the load has another path: crack the single
  //    worst bond and let the next solve redistribute.
  // 2. For standing structures, re-solve what would remain. Anything that
  //    would fail right next to the falling region, or that would be left
  //    dangling, joins it. Everything that loses support in one event leaves
  //    as ONE rigid body instead of peeling off floor by floor.
  // 3. Break only the bonds on that region's border. Overloaded bonds inside
  //    it stay intact, because falling unloads them.
  _fail(isl, supports, cause) {
    const st = this.st;
    const first = this._looseRegion(isl.chunks, supports);
    if (!first) return null;
    if (first.crack) {
      this._breakBond(first.crack, cause);
      return { detached: 0, bonds: 1, worst: first.crack };
    }
    const fall = first.fall;
    if (!supports) {
      for (let round = 0; round < 3; round++) {
        const rest = isl.chunks.filter(id => !fall.has(id) && !st.chunks[id].destroyed);
        let grew = false;
        for (const comp of st.components(rest)) {
          if (!comp.anchored) { for (const id of comp.chunks) fall.add(id); grew = true; continue; }
          if (comp.chunks.length < 2) continue;
          this.solver.solveIsland({ chunks: comp.chunks });
          const r = this._looseRegion(comp.chunks, null);
          if (r && r.fall && this._touches(r.fall, fall)) { for (const id of r.fall) fall.add(id); grew = true; }
        }
        if (!grew) break;
      }
    }
    let n = 0, worst = null;
    for (const id of fall) {
      for (const bi of st.chunks[id].bonds) {
        const b = st.bonds[bi];
        if (b.broken) continue;
        const o = b.b === GROUND ? -1 : st.other(b, id);
        if (o >= 0 && fall.has(o)) continue;
        if (!worst || b.util > worst.util) worst = b;
        this._breakBond(b, cause); n++;
      }
    }
    let mass = 0;
    for (const id of fall) mass += st.chunks[id].mass;
    return { detached: fall.size, mass, bonds: n, worst };
  }

  // The region that would lose support if the seriously overloaded bonds
  // among `ids` let go. Uses the utilisation from the last solve.
  _looseRegion(ids, supports) {
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
    const cut = Math.max(1, maxU * DETACH_BAND);
    const released = new Set();
    for (const id of ids) for (const bi of st.chunks[id].bonds) {
      const b = st.bonds[bi];
      if (!b.broken && b.a === id && within(b) && b.util >= cut) released.add(b.id);
    }
    const loose = st.components(ids, b => released.has(b.id), supports).filter(c => !c.anchored);
    if (!loose.length) return { crack: worst };
    const fall = new Set();
    for (const comp of loose) for (const id of comp.chunks) fall.add(id);
    return { fall };
  }

  _touches(a, b) {
    for (const id of a) for (const bi of this.st.chunks[id].bonds) {
      const bd = this.st.bonds[bi];
      if (!bd.broken && bd.b !== GROUND && b.has(this.st.other(bd, id))) return true;
    }
    return false;
  }

  _logFailure(res, kind) {
    if (!res || !res.worst) return;
    const b = res.worst, st = this.st;
    const where = b.b === GROUND ? 'footing' : `${MATERIALS[st.chunks[b.a].mat].label.toLowerCase()} joint`;
    if (res.detached > 1) {
      const what = kind === 'impact' ? 'Impact snapped off' : 'Detached';
      this.log(`${what} ${res.detached} chunks (${fmtMass(res.mass)}) as one piece: ${res.bonds} ${where}${res.bonds === 1 ? '' : 's'} failed in ${b.mode}`, 'collapse');
    } else if (kind === 'stress') {
      this.log(cap(`${where} cracked in ${b.mode} (${(b.util * 100).toFixed(0)}% of capacity)`), 'stress');
    }
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

    // A piece that just broke away slides off the faces it cracked from
    // instead of hanging on them by friction. Bearing contacts (mostly
    // vertical normal) keep friction, so it still lands normally.
    if (Math.abs(n.y) < 0.6) {
      const ua = fa.getUserData(), ub = fb.getUserData();
      const fresh = (u, body) => u && u.kind === 'chunk' && body.isDynamic() && this.st.chunks[u.id].freshUntil > this.time;
      if ((fresh(ua, ba) && !bb.isDynamic()) || (fresh(ub, bb) && !ba.isDynamic())) contact.setFriction(0);
    }
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
      mA: fa.getBody().getMass(), mB: fb.getBody().getMass(),
      bodyA: fa.getBody(), bodyB: fb.getBody(),
    });
  }

  _processContacts() {
    const st = this.st;
    const touched = new Set();
    const vT = this.settings.impactSpeed;
    const accum = new Map();
    const hits = new Map();   // falling island id -> impulse + contact supports

    const addHit = (cid, Px, Py, k, nx, ny) => {
      const c = st.chunks[cid];
      const isl = st.islands.get(c.island);
      if (!isl || !isl.body) return;
      let h = hits.get(isl.id);
      if (!h) { h = { isl, jx: 0, jy: 0, approach: 0, supports: new Map() }; hits.set(isl.id, h); }
      h.jx += Px; h.jy += Py;
      h.approach = Math.max(h.approach, k.approach);
      const prev = h.supports.get(cid);
      if (!prev || k.jn > prev.jn) h.supports.set(cid, { x: k.px, y: k.py, nx, ny, jn: k.jn });
    };

    for (const k of this.contacts) {
      // normal points A -> B; impulse on B is +P, on A is -P
      const tx = k.ny, ty = -k.nx;
      const Px = k.jn * k.nx + k.jt * tx, Py = k.jn * k.ny + k.jt * ty;
      if (k.ca >= 0 && !k.staticA) addHit(k.ca, -Px, -Py, k, k.nx, k.ny);
      if (k.cb >= 0 && !k.staticB) addHit(k.cb, Px, Py, k, -k.nx, -k.ny);

      // local crushing: the impact's energy breaks the bonds nearest the
      // contact first and runs out, so only that portion is damaged
      if (k.approach > vT && k.jn > 0) {
        const E = 0.5 * k.jn * (k.approach - vT) * this.settings.fragility;
        const sides = [k.ca, k.cb].filter(id => id >= 0 && !st.chunks[id].destroyed);
        for (const cid of sides) this._crush(st.chunks[cid], k.px, k.py, E / sides.length, touched);
      }

      // debris pressing on standing structure becomes a load for the stress
      // solver (a piece that has only just broken away does not count yet)
      // Landing jolts are handled by crushing above; what is fed in here is
      // debris weight resting ON the structure (mostly vertical contact),
      // capped at 1.5x the debris weight. Sideways shoves only crush locally.
      const fresh = (id) => id >= 0 && (st.chunks[id].freshUntil > this.time || Math.abs(k.ny) < 0.7);
      const capped = (m) => Math.min(1, 1.5 * m * GRAVITY / (Math.hypot(Px, Py) / STEP || 1)) / STEP;
      if (k.ca >= 0 && k.staticA && !k.staticB && !fresh(k.cb)) { const f = capped(k.mB); addLoad(accum, k.ca, -Px * f, -Py * f, k.px, k.py); st.chunks[k.ca].loadBody = k.bodyB; }
      if (k.cb >= 0 && k.staticB && !k.staticA && !fresh(k.ca)) { const f = capped(k.mA); addLoad(accum, k.cb, Px * f, Py * f, k.px, k.py); st.chunks[k.cb].loadBody = k.bodyA; }
    }
    this.contacts.length = 0;
    this._resolve(touched);
    touched.clear();

    // hard landings: solve the falling body against its contacts with the
    // impact deceleration as body load. It snaps where it is weak (a long
    // overhang, a thin column), the rest stays one piece.
    const big = [...hits.values()]
      .filter(h => st.islands.has(h.isl.id) && h.isl.body && h.isl.chunks.length >= 4)
      .map(h => ({ ...h, dv: Math.hypot(h.jx, h.jy) / h.isl.body.getMass() }))
      .filter(h => h.dv > vT)
      .sort((p, q) => q.dv * q.isl.chunks.length - p.dv * p.isl.chunks.length)
      .slice(0, 2);
    for (const h of big) {
      const J = Math.hypot(h.jx, h.jy);
      const a = h.dv / this.settings.impactTime * this.settings.fragility;
      // every chunk feels the deceleration as a body force opposite the impulse
      this._solveBody(h.isl, h.supports, -h.jx / J * a, -h.jy / J * a, 'impact');
      h.isl.restChecked = false;
    }

    this._restChecks();

    // smooth loads, decay the ones that went away, flag islands to re-solve
    for (const id of accum.keys()) this.loaded.add(id);
    for (const id of [...this.loaded]) {
      const c = st.chunks[id];
      if (c.destroyed) { this.loaded.delete(id); continue; }
      // a sleeping body reports no contacts but still rests here: keep its load
      if (!accum.has(id) && c.loadBody && !c.loadBody.__dead && !c.loadBody.isAwake()) {
        if (c.lastRaw && !c.frozen) {
          c.frozen = true;
          c.ext[0] = c.lastRaw[0]; c.ext[1] = c.lastRaw[1]; c.ext[2] = c.lastRaw[2];
          const isl = st.islands.get(c.island);
          if (isl && isl.anchored) isl.dirty = true;
        }
        continue;
      }
      c.frozen = false;
      const a = accum.get(id) || [0, 0, 0, 0, 0];
      const fx = a[0], fy = a[1];
      const tq = (a[3] - c.x) * fy - (a[4] - c.y) * fx;   // torque about chunk centre
      if (accum.has(id)) c.lastRaw = [fx, fy, tq];
      const k = 0.05;   // ~0.3 s averaging: sustained load, not the landing spike
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

  // Spend impact energy on the bonds nearest the contact point, nearest first.
  // Farther bonds cost more, so damage stays local unless the hit is huge.
  _crush(c, px, py, E, touched) {
    if (E <= 0 || c.destroyed) return;
    const st = this.st;
    const isl = st.islands.get(c.island);
    if (!isl) return;
    const body = isl.body;
    const R = 1.6;
    const near = [];
    const seen = new Set([c.id]);
    const queue = [c.id];
    while (queue.length) {
      const id = queue.pop();
      for (const bi of st.chunks[id].bonds) {
        const b = st.bonds[bi];
        if (b.broken || b.b === GROUND) continue;
        const w = body ? body.getWorldPoint(this.pl.Vec2(b.cx, b.cy)) : { x: b.cx, y: b.cy };
        const d = Math.hypot(w.x - px, w.y - py);
        if (d > R) continue;
        if (!near.includes(b)) near.push(Object.assign(b, { _d: d }));
        const o = st.other(b, id);
        if (!seen.has(o)) { seen.add(o); queue.push(o); }
      }
    }
    near.sort((p, q) => p._d - q._d);
    for (const b of near) {
      const G = b.toughness * FRACTURE_J_PER_TOUGHNESS * (1 + b._d / 0.4);   // energy to break this bond
      const need = G * b.integrity;
      if (E >= need) { E -= need; this._breakBond(b, 'impact'); touched.add(c.island); }
      else { b.integrity -= E / G; touched.add(c.island); break; }
    }
  }

  // Stress-check a falling body against temporary supports (world-space
  // contacts) with a body acceleration (world). Breaks with the shared rule.
  _solveBody(isl, supportsMap, axW, ayW, cause) {
    const body = isl.body;
    if (!body || !supportsMap.size) return null;
    const ang = body.getAngle(), cs = Math.cos(-ang), sn = Math.sin(-ang);
    const rot = (x, y) => ({ x: x * cs - y * sn, y: x * sn + y * cs });
    const supports = [];
    for (const [id, sp] of supportsMap) {
      if (this.st.chunks[id].destroyed || this.st.chunks[id].island !== isl.id) continue;
      const lp = body.getLocalPoint(this.pl.Vec2(sp.x, sp.y));
      const ln = rot(sp.nx, sp.ny);
      supports.push({ id, x: lp.x, y: lp.y, nx: ln.x, ny: ln.y });
    }
    if (!supports.length) return null;
    const maxU = this.solver.solveIsland(isl, { supports, accel: rot(axW, ayW) });
    if (maxU <= 1) return null;
    const res = this._fail(isl, new Set(supports.map(s => s.id)), cause);
    this._logFailure(res, 'impact');
    this._resolve(new Set([isl.id]));
    return res;
  }

  // Once a fallen piece comes to rest, check it can carry its own weight on
  // whatever it landed on (a slab lying across rubble may snap).
  _restChecks() {
    let budget = 1;
    for (const isl of this.st.islands.values()) {
      if (!budget) break;
      const body = isl.body;
      if (!body || isl.restChecked || isl.chunks.length < 6 || this.time - isl.born < 0.4) continue;
      if (body.getLinearVelocity().length() > 0.4 || Math.abs(body.getAngularVelocity()) > 0.4) continue;
      isl.restChecked = true;
      const supports = new Map();
      for (let ce = body.getContactList(); ce; ce = ce.next) {
        const ct = ce.contact;
        if (!ct.isTouching()) continue;
        const mineA = ct.getFixtureA().getBody() === body;
        const mine = mineA ? ct.getFixtureA() : ct.getFixtureB();
        const ud = mine.getUserData();
        if (!ud || ud.kind !== 'chunk') continue;
        const wm = ct.getWorldManifold(null);
        const cnt = ct.getManifold().pointCount;
        if (!wm || !cnt) continue;
        let x = 0, y = 0;
        for (let i = 0; i < cnt; i++) { x += wm.points[i].x / cnt; y += wm.points[i].y / cnt; }
        const s = mineA ? 1 : -1;
        supports.set(ud.id, { x, y, nx: wm.normal.x * s, ny: wm.normal.y * s, jn: 0 });
      }
      budget--;
      this._solveBody(isl, supports, 0, -GRAVITY, 'stress');
    }
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
    if (isl.body) this._destroyBody(isl.body);
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
