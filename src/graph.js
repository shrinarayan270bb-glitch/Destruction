// Support graph: chunks are nodes, bonds are edges, the ground is a special
// fixed node (index GROUND). Everything here is pure data + graph algorithms,
// no physics engine, so it runs headless in tests.

import { MATERIALS, FOOTINGS, GRAVITY } from './materials.js';

export const GROUND = -1;

export class Structure {
  constructor(cell = 0.5, depth = 0.5) {
    this.cell = cell;      // chunk edge length (m)
    this.depth = depth;    // out-of-plane thickness used for mass (m)
    this.chunks = [];
    this.bonds = [];
    this.islands = new Map();
    this.nextIslandId = 1;
    this._grid = new Map();
  }

  // ---- building -------------------------------------------------------

  addChunk(ix, iy, mat, footing = null, tag = null) {
    const key = ix + ',' + iy;
    if (this._grid.has(key)) {
      const c = this.chunks[this._grid.get(key)];
      c.mat = mat;
      c.footing = footing;
      c.tag = tag;
      return c.id;
    }
    const s = this.cell;
    const id = this.chunks.length;
    this.chunks.push({
      id, ix, iy, mat, footing, tag,
      x: (ix + 0.5) * s, y: (iy + 0.5) * s,   // home centre (m)
      mass: 0, health: 0, destroyed: false,
      bonds: [], groundBond: -1,
      island: 0,
      // stress solution (displacement + rotation), kept for warm starts
      qx: 0, qy: 0, qt: 0,
      // external load from resting debris, force at centre + torque
      ext: [0, 0, 0], extSolved: [0, 0, 0],
      // physics link
      local: { x: 0, y: 0 }, fixture: null,
      util: 0,
    });
    this._grid.set(key, id);
    return id;
  }

  removeCell(ix, iy) {
    // only valid before finalize()
    const key = ix + ',' + iy;
    if (!this._grid.has(key)) return;
    const id = this._grid.get(key);
    this._grid.delete(key);
    this.chunks[id].removed = true;
  }

  finalize() {
    // compact out cells removed during authoring
    const keep = this.chunks.filter(c => !c.removed);
    this.chunks = keep;
    this._grid.clear();
    keep.forEach((c, i) => { c.id = i; this._grid.set(c.ix + ',' + c.iy, i); });

    const s = this.cell;
    for (const c of this.chunks) {
      const m = MATERIALS[c.mat];
      c.mass = m.density * s * s * this.depth;
      c.health = m.hp;
      c.local = { x: c.x, y: c.y };
      const right = this._grid.get((c.ix + 1) + ',' + c.iy);
      const up = this._grid.get(c.ix + ',' + (c.iy + 1));
      if (right !== undefined) this._addBond(c.id, right, 1, 0);
      if (up !== undefined) this._addBond(c.id, up, 0, 1);
      if (c.iy === 0 && c.footing) {
        const b = this._addBond(c.id, GROUND, 0, -1);
        c.groundBond = b;
      }
    }
    this.rebuildIslands();
    return this;
  }

  _addBond(a, b, nx, ny) {
    const s = this.cell;
    const ca = this.chunks[a];
    const id = this.bonds.length;
    const limits = b === GROUND
      ? FOOTINGS[ca.footing]
      : combine(MATERIALS[ca.mat], MATERIALS[this.chunks[b].mat]);
    this.bonds.push({
      id, a, b, nx, ny,
      cx: ca.x + nx * s / 2, cy: ca.y + ny * s / 2,   // contact centroid (home)
      L: s,
      limits,
      toughness: b === GROUND ? 1e9 : Math.min(MATERIALS[ca.mat].toughness, MATERIALS[this.chunks[b].mat].toughness),
      integrity: 1, broken: false,
      N: 0, V: 0, M: 0, util: 0, mode: '',
    });
    ca.bonds.push(id);
    if (b !== GROUND) this.chunks[b].bonds.push(id);
    return id;
  }

  // ---- queries --------------------------------------------------------

  other(bond, c) { return bond.a === c ? bond.b : bond.a; }

  isAnchoredChunk(c) {
    return c.groundBond >= 0 && !this.bonds[c.groundBond].broken;
  }

  // ---- islands (connected components) ----------------------------------

  rebuildIslands() {
    this.islands.clear();
    const all = this.chunks.filter(c => !c.destroyed).map(c => c.id);
    for (const comp of this.components(all)) this._makeIsland(comp.chunks, comp.anchored);
  }

  // BFS over unbroken bonds restricted to `ids`. Returns [{chunks, anchored}].
  // skip(bond) lets callers ask "what if these bonds were gone?";
  // supports (a Set of chunk ids) count as anchored, used for bodies resting
  // on something instead of bonded to the ground.
  components(ids, skip = null, supports = null) {
    const allowed = new Set(ids);
    const seen = new Set();
    const out = [];
    for (const start of ids) {
      if (seen.has(start) || this.chunks[start].destroyed) continue;
      const comp = [];
      let anchored = false;
      const queue = [start];
      seen.add(start);
      while (queue.length) {
        const id = queue.pop();
        const c = this.chunks[id];
        comp.push(id);
        if (supports && supports.has(id)) anchored = true;
        for (const bi of c.bonds) {
          const bond = this.bonds[bi];
          if (bond.broken || (skip && skip(bond))) continue;
          if (bond.b === GROUND) { anchored = true; continue; }
          const n = this.other(bond, id);
          if (!seen.has(n) && allowed.has(n) && !this.chunks[n].destroyed) { seen.add(n); queue.push(n); }
        }
      }
      out.push({ chunks: comp, anchored });
    }
    return out;
  }

  _makeIsland(chunkIds, anchored) {
    const isl = { id: this.nextIslandId++, chunks: chunkIds, anchored, dirty: anchored, body: null, needsSplit: false };
    for (const id of chunkIds) this.chunks[id].island = isl.id;
    this.islands.set(isl.id, isl);
    return isl;
  }

  // Re-run connectivity on one island. Returns the list of resulting islands
  // (the original object is reused when nothing changed topologically).
  splitIsland(isl) {
    const live = isl.chunks.filter(id => !this.chunks[id].destroyed);
    const comps = this.components(live);
    if (comps.length === 1 && comps[0].anchored === isl.anchored) {
      isl.chunks = comps[0].chunks;
      return { kept: isl, created: [] };
    }
    this.islands.delete(isl.id);
    const created = comps.map(c => this._makeIsland(c.chunks, c.anchored));
    return { kept: null, created, old: isl };
  }

  // ---- mass properties -------------------------------------------------

  // posFn(chunk) -> {x, y} in world space (lets callers use body transforms)
  massProps(chunkIds, posFn = c => c) {
    let m = 0, mx = 0, my = 0;
    for (const id of chunkIds) {
      const c = this.chunks[id];
      if (c.destroyed) continue;
      const p = posFn(c);
      m += c.mass; mx += c.mass * p.x; my += c.mass * p.y;
    }
    if (m === 0) return { mass: 0, x: 0, y: 0, inertia: 0 };
    const cx = mx / m, cy = my / m;
    // parallel-axis theorem: square plate inertia + m d^2
    const s = this.cell;
    let I = 0;
    for (const id of chunkIds) {
      const c = this.chunks[id];
      if (c.destroyed) continue;
      const p = posFn(c);
      const dx = p.x - cx, dy = p.y - cy;
      I += c.mass * (s * s) / 6 + c.mass * (dx * dx + dy * dy);
    }
    return { mass: m, x: cx, y: cy, inertia: I };
  }

  // Ground contact span of an anchored island (the 2D "support polygon")
  supportSpan(isl) {
    let lo = Infinity, hi = -Infinity;
    const h = this.cell / 2;
    for (const id of isl.chunks) {
      const c = this.chunks[id];
      if (!c.destroyed && this.isAnchoredChunk(c)) { lo = Math.min(lo, c.x - h); hi = Math.max(hi, c.x + h); }
    }
    return lo <= hi ? { lo, hi } : null;
  }

  // ---- critical supports (articulation points, ground included) -------

  // Iterative Tarjan. A chunk is critical if removing it disconnects part of
  // the structure from the ground (or splits the structure).
  articulationPoints() {
    const n = this.chunks.length;
    const G = n; // ground node index in this local numbering
    const disc = new Int32Array(n + 1).fill(-1);
    const low = new Int32Array(n + 1);
    const result = new Set();
    let time = 0;

    const neighbours = (v) => {
      const list = [];
      if (v === G) {
        for (const c of this.chunks) if (!c.destroyed && this.isAnchoredChunk(c) && this.islands.get(c.island)?.anchored) list.push(c.id);
        return list;
      }
      for (const bi of this.chunks[v].bonds) {
        const b = this.bonds[bi];
        if (b.broken) continue;
        if (b.b === GROUND) { list.push(G); continue; }
        const o = this.other(b, v);
        if (!this.chunks[o].destroyed) list.push(o);
      }
      return list;
    };

    const roots = [G];
    for (const c of this.chunks) if (!c.destroyed && this.islands.get(c.island)?.anchored) roots.push(c.id);

    for (const root of roots) {
      if (disc[root] !== -1) continue;
      let rootChildren = 0;
      disc[root] = low[root] = time++;
      const stack = [{ v: root, parent: -2, nb: neighbours(root), i: 0 }];
      while (stack.length) {
        const fr = stack[stack.length - 1];
        if (fr.i < fr.nb.length) {
          const w = fr.nb[fr.i++];
          if (disc[w] === -1) {
            disc[w] = low[w] = time++;
            if (fr.v === root) rootChildren++;
            stack.push({ v: w, parent: fr.v, nb: neighbours(w), i: 0 });
          } else if (w !== fr.parent) {
            low[fr.v] = Math.min(low[fr.v], disc[w]);
          }
        } else {
          stack.pop();
          if (stack.length) {
            const p = stack[stack.length - 1];
            low[p.v] = Math.min(low[p.v], low[fr.v]);
            if (p.v !== root && low[fr.v] >= disc[p.v] && p.v !== G) result.add(p.v);
          }
        }
      }
      if (root !== G && rootChildren > 1) result.add(root);
    }
    return result;
  }

  totalWeight(chunkIds) {
    let m = 0;
    for (const id of chunkIds) if (!this.chunks[id].destroyed) m += this.chunks[id].mass;
    return m * GRAVITY;
  }
}

function combine(ma, mb) {
  return {
    tension: Math.min(ma.tension, mb.tension),
    compression: Math.min(ma.compression, mb.compression),
    shear: Math.min(ma.shear, mb.shear),
    stiffness: Math.min(ma.stiffness, mb.stiffness),
  };
}

// Small authoring helper used by the presets.
export class Builder {
  constructor(structure) { this.s = structure; this.footing = 'cast'; this.ox = 0; this.label = null; }
  at(ox) { this.ox = ox; return this; }
  foot(f) { this.footing = f; return this; }
  tag(t) { this.label = t; return this; }   // names chunks so scenarios can find them
  set(ix, iy, mat) { this.s.addChunk(this.ox + ix, iy, mat, iy === 0 ? this.footing : null, this.label); return this; }
  rect(ix, iy, w, h, mat) {
    for (let x = ix; x < ix + w; x++) for (let y = iy; y < iy + h; y++) this.set(x, y, mat);
    return this;
  }
  clear(ix, iy, w = 1, h = 1) {
    for (let x = ix; x < ix + w; x++) for (let y = iy; y < iy + h; y++) this.s.removeCell(this.ox + x, y);
    return this;
  }
}
