// 3D support graph. Chunks are 1 m voxels on an integer grid, bonds join
// face neighbours (6-connectivity), the ground is a fixed node.
// Pure data + graph algorithms; no physics or rendering.

import { MATERIALS, FOOTINGS, GRAVITY } from './materials.js';

export const GROUND = -1;
const key = (x, y, z) => x + ',' + y + ',' + z;

export class Structure {
  constructor(cell = 1) {
    this.cell = cell;
    this.chunks = [];
    this.bonds = [];
    this.islands = new Map();
    this.nextIslandId = 1;
    this.grid = new Map();
    this.nextMember = 1;
  }

  // ---- authoring -------------------------------------------------------

  addChunk(ix, iy, iz, mat, footing = null, tag = null, member = -1) {
    if (member < 0) member = this.nextMember++;
    const k = key(ix, iy, iz);
    if (this.grid.has(k)) {
      const c = this.chunks[this.grid.get(k)];
      Object.assign(c, { mat, footing, tag, member, removed: false });
      return c.id;
    }
    const s = this.cell;
    const id = this.chunks.length;
    this.chunks.push({
      id, ix, iy, iz, mat, footing, tag, member,
      x: (ix + 0.5) * s, y: (iy + 0.5) * s, z: (iz + 0.5) * s,   // home centre
      mass: 0, health: 0, destroyed: false,
      bonds: [], groundBond: -1, island: 0,
      q: new Float64Array(6),            // stress solution: displacement + rotation
      ext: new Float64Array(6),          // debris load: force + torque about centre
      extSolved: new Float64Array(6),
      util: 0,
    });
    this.grid.set(k, id);
    return id;
  }

  removeCell(ix, iy, iz) {
    const k = key(ix, iy, iz);
    if (!this.grid.has(k)) return;
    this.chunks[this.grid.get(k)].removed = true;
    this.grid.delete(k);
  }

  finalize() {
    const keep = this.chunks.filter(c => !c.removed);
    this.chunks = keep;
    this.grid.clear();
    keep.forEach((c, i) => { c.id = i; this.grid.set(key(c.ix, c.iy, c.iz), i); });
    const s = this.cell;
    for (const c of this.chunks) {
      const m = MATERIALS[c.mat];
      c.mass = m.density * s * s * s;
      c.health = m.hp;
      const nb = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
      for (const [dx, dy, dz] of nb) {
        const o = this.grid.get(key(c.ix + dx, c.iy + dy, c.iz + dz));
        if (o !== undefined) this._addBond(c.id, o, dx, dy, dz);
      }
      if (c.iy === 0 && c.footing) c.groundBond = this._addBond(c.id, GROUND, 0, -1, 0);
    }
    this.splitMembers(this.chunks.map(c => c.id));
    this.rebuildIslands();
    return this;
  }

  // Members are rigid structural pieces (slab panels, column segments, wall
  // panels). A member must be connected through its own intact bonds; any
  // piece cut off from the rest (by a blast, an impact, an opening) becomes
  // a member of its own. Returns true if anything was relabelled.
  splitMembers(ids) {
    const groups = new Map();
    for (const id of ids) {
      const c = this.chunks[id];
      if (c.destroyed) continue;
      if (!groups.has(c.member)) groups.set(c.member, []);
      groups.get(c.member).push(id);
    }
    let changed = false;
    for (const [mem, list] of groups) {
      if (list.length < 2) continue;
      const seen = new Set([list[0]]);
      const stack = [list[0]];
      while (stack.length) {
        const id = stack.pop();
        for (const bi of this.chunks[id].bonds) {
          const b = this.bonds[bi];
          if (b.broken || b.b === GROUND) continue;
          const o = this.other(b, id);
          if (!seen.has(o) && this.chunks[o].member === mem && !this.chunks[o].destroyed) { seen.add(o); stack.push(o); }
        }
      }
      if (seen.size === list.length) continue;
      // relabel the rest component by component
      changed = true;
      for (const id of list) {
        if (seen.has(id) || this.chunks[id].member !== mem) continue;
        const nm = this.nextMember++;
        const st2 = [id];
        this.chunks[id].member = nm;
        while (st2.length) {
          const x = st2.pop();
          for (const bi of this.chunks[x].bonds) {
            const b = this.bonds[bi];
            if (b.broken || b.b === GROUND) continue;
            const o = this.other(b, x);
            if (this.chunks[o].member === mem && !seen.has(o) && !this.chunks[o].destroyed) { this.chunks[o].member = nm; st2.push(o); }
          }
        }
      }
    }
    return changed;
  }

  _addBond(a, b, nx, ny, nz) {
    const s = this.cell;
    const ca = this.chunks[a];
    const limits = b === GROUND ? FOOTINGS[ca.footing] : combine(MATERIALS[ca.mat], MATERIALS[this.chunks[b].mat]);
    const id = this.bonds.length;
    this.bonds.push({
      id, a, b, nx, ny, nz,
      cx: ca.x + nx * s / 2, cy: ca.y + ny * s / 2, cz: ca.z + nz * s / 2,
      L: s, limits,
      toughness: b === GROUND ? 1e9 : Math.min(MATERIALS[ca.mat].toughness, MATERIALS[this.chunks[b].mat].toughness),
      integrity: 1, broken: false,
      N: 0, V: 0, T: 0, M: 0, util: 0, mode: '',
    });
    ca.bonds.push(id);
    if (b !== GROUND) this.chunks[b].bonds.push(id);
    return id;
  }

  // ---- queries -----------------------------------------------------------

  other(bond, c) { return bond.a === c ? bond.b : bond.a; }
  isAnchoredChunk(c) { return c.groundBond >= 0 && !this.bonds[c.groundBond].broken; }
  at(ix, iy, iz) { const id = this.grid.get(key(ix, iy, iz)); return id === undefined ? null : this.chunks[id]; }

  // ---- islands -------------------------------------------------------------

  rebuildIslands() {
    this.islands.clear();
    const all = this.chunks.filter(c => !c.destroyed).map(c => c.id);
    for (const comp of this.components(all)) this._makeIsland(comp.chunks, comp.anchored);
  }

  // Connected components of `ids` over intact bonds. skip(bond) hides bonds
  // ("what if these let go?"); `supports` chunk ids count as anchored.
  components(ids, skip = null, supports = null) {
    const allowed = new Set(ids);
    const seen = new Set();
    const out = [];
    for (const start of ids) {
      if (seen.has(start) || this.chunks[start].destroyed) continue;
      const comp = [];
      let anchored = false;
      const stack = [start];
      seen.add(start);
      while (stack.length) {
        const id = stack.pop();
        comp.push(id);
        if (supports && supports.has(id)) anchored = true;
        for (const bi of this.chunks[id].bonds) {
          const b = this.bonds[bi];
          if (b.broken || (skip && skip(b))) continue;
          if (b.b === GROUND) { anchored = true; continue; }
          const n = this.other(b, id);
          if (!seen.has(n) && allowed.has(n) && !this.chunks[n].destroyed) { seen.add(n); stack.push(n); }
        }
      }
      out.push({ chunks: comp, anchored });
    }
    return out;
  }

  _makeIsland(chunkIds, anchored) {
    const isl = { id: this.nextIslandId++, chunks: chunkIds, anchored, dirty: anchored, body: null, version: 0 };
    for (const id of chunkIds) this.chunks[id].island = isl.id;
    this.islands.set(isl.id, isl);
    return isl;
  }

  splitIsland(isl) {
    const live = isl.chunks.filter(id => !this.chunks[id].destroyed);
    const comps = this.components(live);
    if (comps.length === 1 && comps[0].anchored === isl.anchored) {
      isl.chunks = comps[0].chunks;
      return { kept: isl, created: [] };
    }
    this.islands.delete(isl.id);
    return { kept: null, created: comps.map(c => this._makeIsland(c.chunks, c.anchored)), old: isl };
  }

  // ---- mass properties -----------------------------------------------------

  // posFn(chunk) -> {x,y,z} world position. Inertia is about the COM, in the
  // same frame as posFn (cube inertia + parallel-axis term).
  massProps(ids, posFn = c => c) {
    let m = 0, mx = 0, my = 0, mz = 0;
    const P = [];
    for (const id of ids) {
      const c = this.chunks[id];
      if (c.destroyed) continue;
      const p = posFn(c);
      P.push([c, p]);
      m += c.mass; mx += c.mass * p.x; my += c.mass * p.y; mz += c.mass * p.z;
    }
    if (!m) return { mass: 0, x: 0, y: 0, z: 0, inertia: [0, 0, 0, 0, 0, 0, 0, 0, 0] };
    const cx = mx / m, cy = my / m, cz = mz / m;
    const s2 = this.cell * this.cell / 6;
    const I = [0, 0, 0, 0, 0, 0, 0, 0, 0];
    for (const [c, p] of P) {
      const dx = p.x - cx, dy = p.y - cy, dz = p.z - cz, d2 = dx * dx + dy * dy + dz * dz;
      const k = c.mass;
      I[0] += k * (s2 + d2 - dx * dx); I[4] += k * (s2 + d2 - dy * dy); I[8] += k * (s2 + d2 - dz * dz);
      I[1] -= k * dx * dy; I[2] -= k * dx * dz; I[5] -= k * dy * dz;
    }
    I[3] = I[1]; I[6] = I[2]; I[7] = I[5];
    return { mass: m, x: cx, y: cy, z: cz, inertia: I };
  }

  // Footprint of the intact footings as a convex polygon in XZ (the support
  // polygon), plus the signed distance of a point to its edge (+ inside).
  supportPolygon(isl) {
    const h = this.cell / 2;
    const pts = [];
    for (const id of isl.chunks) {
      const c = this.chunks[id];
      if (c.destroyed || !this.isAnchoredChunk(c)) continue;
      pts.push([c.x - h, c.z - h], [c.x + h, c.z - h], [c.x + h, c.z + h], [c.x - h, c.z + h]);
    }
    return pts.length ? convexHull(pts) : null;
  }

  totalWeight(ids) {
    let m = 0;
    for (const id of ids) if (!this.chunks[id].destroyed) m += this.chunks[id].mass;
    return m * GRAVITY;
  }

  // ---- critical supports (articulation points, ground included) ----------

  articulationPoints() {
    const n = this.chunks.length, G = n;
    const disc = new Int32Array(n + 1).fill(-1), low = new Int32Array(n + 1);
    const result = new Set();
    let time = 0;
    const standing = (c) => !c.destroyed && this.islands.get(c.island)?.anchored;
    const neighbours = (v) => {
      const list = [];
      if (v === G) { for (const c of this.chunks) if (standing(c) && this.isAnchoredChunk(c)) list.push(c.id); return list; }
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
    for (const c of this.chunks) if (standing(c)) roots.push(c.id);
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
          } else if (w !== fr.parent) low[fr.v] = Math.min(low[fr.v], disc[w]);
        } else {
          stack.pop();
          if (stack.length) {
            const p = stack[stack.length - 1];
            low[p.v] = Math.min(low[p.v], low[fr.v]);
            if (p.v !== root && p.v !== G && low[fr.v] >= disc[p.v]) result.add(p.v);
          }
        }
      }
      if (root !== G && rootChildren > 1) result.add(root);
    }
    return result;
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

// Andrew's monotone chain, counter-clockwise
export function convexHull(pts) {
  const p = pts.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lower = [], upper = [];
  for (const q of p) { while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], q) <= 0) lower.pop(); lower.push(q); }
  for (let i = p.length - 1; i >= 0; i--) { const q = p[i]; while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], q) <= 0) upper.pop(); upper.push(q); }
  upper.pop(); lower.pop();
  return lower.concat(upper);
}

// Signed distance from (x, z) to a CCW convex polygon's boundary (+ inside)
export function polygonMargin(poly, x, z) {
  if (!poly || poly.length < 3) return -Infinity;
  let inside = Infinity;
  for (let i = 0; i < poly.length; i++) {
    const [ax, az] = poly[i], [bx, bz] = poly[(i + 1) % poly.length];
    const ex = bx - ax, ez = bz - az, len = Math.hypot(ex, ez) || 1;
    // left-of-edge distance (CCW => inside is left)
    inside = Math.min(inside, (ex * (z - az) - ez * (x - ax)) / len);
  }
  return inside;
}

const MEMBER = 4;
function parts(n) {
  const p = Math.max(1, Math.ceil(n / MEMBER)), base = Math.floor(n / p), extra = n % p;
  const out = [];
  let a = 0;
  for (let i = 0; i < p; i++) { const len = base + (i < extra ? 1 : 0); out.push([a, a + len]); a += len; }
  return out;
}

export class Builder {
  constructor(structure) { this.s = structure; this.footing = 'cast'; this.ox = 0; this.oz = 0; this.label = null; }
  at(ox, oz = 0) { this.ox = ox; this.oz = oz; return this; }
  foot(f) { this.footing = f; return this; }
  tag(t) { this.label = t; return this; }
  set(x, y, z, mat, member = -1) { this.s.addChunk(this.ox + x, y, this.oz + z, mat, y === 0 ? this.footing : null, this.label, member); return this; }
  // A box is cut into structural members no larger than MEMBER voxels a side
  // (a floor becomes slab panels, a column becomes storey segments).
  box(x, y, z, w, h, d, mat) {
    const px = parts(w), py = parts(h), pz = parts(d);
    for (const [x0, x1] of px) for (const [y0, y1] of py) for (const [z0, z1] of pz) {
      const mem = this.s.nextMember++;
      for (let i = x0; i < x1; i++) for (let j = y0; j < y1; j++) for (let k = z0; k < z1; k++) this.set(x + i, y + j, z + k, mat, mem);
    }
    return this;
  }
  clear(x, y, z, w = 1, h = 1, d = 1) {
    for (let i = x; i < x + w; i++) for (let j = y; j < y + h; j++) for (let k = z; k < z + d; k++) this.s.removeCell(this.ox + i, j, this.oz + k);
    return this;
  }
}
