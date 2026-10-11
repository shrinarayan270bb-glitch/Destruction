// Rigid-body engine specialised for voxel bodies (like voxel destruction
// games use). A body is a set of 1 m voxels; the standing structure is a
// static voxel grid; the ground is the plane y = 0.
//
// Collision: every body carries sample points on its exposed faces (corners,
// edge midpoints and face centres, i.e. a half-voxel lattice). A point that
// ends up inside a voxel of the static grid or of another body is a contact,
// pushed out through that voxel's nearest exposed face. Corner points also
// test against the ground plane. This is cheap (a grid lookup per point) and
// scales to thousands of voxels per body.
//
// Dynamics: sequential impulses with friction, warm starting and Baumgarte
// position correction; bodies sleep when still. Every contact reports its
// approach speed and accumulated normal impulse so the destruction layer can
// turn impacts into damage.

const HALF = 1024, SPAN = 4096;
const cellKey = (x, y, z) => (x + HALF) + (y + HALF) * SPAN + (z + HALF) * SPAN * SPAN;

export class VoxelPhysics {
  constructor({ cell = 1, gravity = -9.81 } = {}) {
    this.cell = cell;
    this.gravity = gravity;
    this.static = new Map();          // cellKey -> chunk id
    this.bodies = [];
    this.nextId = 1;
    this.iterations = 10;
    this.friction = 0.6;
    this.beta = 0.2;
    this.slop = 0.01;
    this.warm = new Map();
    this.contacts = [];
    this.pool = [];
    this.contactFriction = null;      // optional (contact) -> mu
  }

  // ---- static grid ---------------------------------------------------------

  addStatic(ix, iy, iz, id) { this.static.set(cellKey(ix, iy, iz), id); }
  removeStatic(ix, iy, iz) {
    this.static.delete(cellKey(ix, iy, iz));
    const s = this.cell, cx = (ix + 0.5) * s, cy = (iy + 0.5) * s, cz = (iz + 0.5) * s;
    for (const b of this.bodies) {
      if (b.sleeping && Math.hypot(b.pos[0] - cx, b.pos[1] - cy, b.pos[2] - cz) < b.radius + 2 * s) this.wake(b);
    }
  }
  staticAt(ix, iy, iz) { const v = this.static.get(cellKey(ix, iy, iz)); return v === undefined ? -1 : v; }

  // ---- bodies --------------------------------------------------------------

  // voxels: [{id, ix, iy, iz, mass}] in home grid coordinates.
  // from: an existing body to inherit transform/velocity from (a split), else
  // the body starts where its voxels sit in the home grid.
  createBody(voxels, from = null, userData = null) {
    const s = this.cell;
    let m = 0, cx = 0, cy = 0, cz = 0;
    for (const v of voxels) { m += v.mass; cx += v.mass * (v.ix + 0.5) * s; cy += v.mass * (v.iy + 0.5) * s; cz += v.mass * (v.iz + 0.5) * s; }
    cx /= m; cy /= m; cz /= m;
    const I = [0, 0, 0, 0, 0, 0, 0, 0, 0];
    const k6 = s * s / 6;
    const cells = new Map();
    let radius = 0;
    for (const v of voxels) {
      const dx = (v.ix + 0.5) * s - cx, dy = (v.iy + 0.5) * s - cy, dz = (v.iz + 0.5) * s - cz, d2 = dx * dx + dy * dy + dz * dz;
      I[0] += v.mass * (k6 + d2 - dx * dx); I[4] += v.mass * (k6 + d2 - dy * dy); I[8] += v.mass * (k6 + d2 - dz * dz);
      I[1] -= v.mass * dx * dy; I[2] -= v.mass * dx * dz; I[5] -= v.mass * dy * dz;
      cells.set(cellKey(v.ix, v.iy, v.iz), v.id);
      radius = Math.max(radius, Math.sqrt(d2) + 0.87 * s);
    }
    I[3] = I[1]; I[6] = I[2]; I[7] = I[5];
    const body = {
      id: this.nextId++, voxels, cells, mass: m, invMass: 1 / m,
      comHome: [cx, cy, cz], Iinv: inv3(I), IinvW: new Float64Array(9), R: new Float64Array(9),
      pos: [cx, cy, cz], q: [1, 0, 0, 0], v: [0, 0, 0], w: [0, 0, 0],
      radius, sleeping: false, sleepTime: 0, userData,
    };
    if (from) {
      const R = from.R;
      const d = [cx - from.comHome[0], cy - from.comHome[1], cz - from.comHome[2]];
      const rd = mulv(R, d);
      body.pos = [from.pos[0] + rd[0], from.pos[1] + rd[1], from.pos[2] + rd[2]];
      body.q = from.q.slice();
      const wxr = crossv(from.w, rd);
      body.v = [from.v[0] + wxr[0], from.v[1] + wxr[1], from.v[2] + wxr[2]];
      body.w = from.w.slice();
    }
    this._buildPoints(body);
    this._updateRotation(body);
    this.bodies.push(body);
    return body;
  }

  removeBody(body) {
    const i = this.bodies.indexOf(body);
    if (i >= 0) this.bodies.splice(i, 1);
    body.removed = true;
  }

  // sample points on exposed faces, half-voxel lattice
  _buildPoints(body) {
    const pts = new Map();
    const dirs = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]];
    for (const v of body.voxels) {
      for (const [dx, dy, dz] of dirs) {
        if (body.cells.has(cellKey(v.ix + dx, v.iy + dy, v.iz + dz))) continue;
        // face lies on the plane through the voxel's side; enumerate 3x3 points
        const ax = dx !== 0 ? 0 : dy !== 0 ? 1 : 2;
        const base = [2 * v.ix, 2 * v.iy, 2 * v.iz];
        base[ax] += (dx + dy + dz) > 0 ? 2 : 0;
        const u = (ax + 1) % 3, w = (ax + 2) % 3;
        for (let i = 0; i <= 2; i++) for (let j = 0; j <= 2; j++) {
          const p = base.slice();
          p[u] += i; p[w] += j;
          const k = (p[0] + HALF) + (p[1] + HALF) * SPAN + (p[2] + HALF) * SPAN * SPAN;
          if (!pts.has(k)) pts.set(k, [p[0], p[1], p[2], v.id]);
        }
      }
    }
    const n = pts.size, s = this.cell;
    body.np = n;
    body.pl = new Float64Array(n * 3);        // local (relative to COM, home axes)
    body.pw = new Float64Array(n * 3);        // world, refreshed each step
    body.owner = new Int32Array(n);
    body.corner = new Uint8Array(n);
    let i = 0;
    for (const [x, y, z, id] of pts.values()) {
      body.pl[i * 3] = x * s / 2 - body.comHome[0];
      body.pl[i * 3 + 1] = y * s / 2 - body.comHome[1];
      body.pl[i * 3 + 2] = z * s / 2 - body.comHome[2];
      body.owner[i] = id;
      body.corner[i] = (x % 2 === 0 && y % 2 === 0 && z % 2 === 0) ? 1 : 0;
      i++;
    }
  }

  _updateRotation(b) {
    quatToMat(b.q, b.R);
    // IinvW = R Iinv R^T
    const R = b.R, I = b.Iinv, T = new Float64Array(9);
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) {
      let s = 0;
      for (let k = 0; k < 3; k++) s += R[i * 3 + k] * I[k * 3 + j];
      T[i * 3 + j] = s;
    }
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) {
      let s = 0;
      for (let k = 0; k < 3; k++) s += T[i * 3 + k] * R[j * 3 + k];
      b.IinvW[i * 3 + j] = s;
    }
  }

  _refreshPoints(b) {
    const R = b.R, pl = b.pl, pw = b.pw, [px, py, pz] = b.pos;
    for (let i = 0; i < b.np; i++) {
      const x = pl[i * 3], y = pl[i * 3 + 1], z = pl[i * 3 + 2];
      pw[i * 3] = px + R[0] * x + R[1] * y + R[2] * z;
      pw[i * 3 + 1] = py + R[3] * x + R[4] * y + R[5] * z;
      pw[i * 3 + 2] = pz + R[6] * x + R[7] * y + R[8] * z;
    }
  }

  // world position of a home-grid point carried by this body
  toWorld(b, hx, hy, hz) {
    const R = b.R, x = hx - b.comHome[0], y = hy - b.comHome[1], z = hz - b.comHome[2];
    return [b.pos[0] + R[0] * x + R[1] * y + R[2] * z, b.pos[1] + R[3] * x + R[4] * y + R[5] * z, b.pos[2] + R[6] * x + R[7] * y + R[8] * z];
  }
  // home-grid coordinates of a world point, in this body's frame
  toHome(b, wx, wy, wz) {
    const R = b.R, x = wx - b.pos[0], y = wy - b.pos[1], z = wz - b.pos[2];
    return [R[0] * x + R[3] * y + R[6] * z + b.comHome[0], R[1] * x + R[4] * y + R[7] * z + b.comHome[1], R[2] * x + R[5] * y + R[8] * z + b.comHome[2]];
  }
  dirToWorld(b, x, y, z) { const R = b.R; return [R[0] * x + R[1] * y + R[2] * z, R[3] * x + R[4] * y + R[5] * z, R[6] * x + R[7] * y + R[8] * z]; }
  dirToHome(b, x, y, z) { const R = b.R; return [R[0] * x + R[3] * y + R[6] * z, R[1] * x + R[4] * y + R[7] * z, R[2] * x + R[5] * y + R[8] * z]; }

  applyImpulse(b, J, p) {
    b.v[0] += J[0] * b.invMass; b.v[1] += J[1] * b.invMass; b.v[2] += J[2] * b.invMass;
    const r = [p[0] - b.pos[0], p[1] - b.pos[1], p[2] - b.pos[2]];
    const t = mulv(b.IinvW, crossv(r, J));
    b.w[0] += t[0]; b.w[1] += t[1]; b.w[2] += t[2];
    this.wake(b);
  }

  wake(b) { b.sleeping = false; b.sleepTime = 0; }

  // ---- step ----------------------------------------------------------------

  step(dt) {
    const g = this.gravity;
    for (const b of this.bodies) {
      if (b.sleeping) continue;
      b.v[1] += g * dt;
      this._refreshPoints(b);
    }
    this._collide();
    this._solve(dt);
    for (const b of this.bodies) {
      if (b.sleeping) continue;
      // cap spin for stability
      const wl = Math.hypot(b.w[0], b.w[1], b.w[2]);
      if (wl > 25) { b.w[0] *= 25 / wl; b.w[1] *= 25 / wl; b.w[2] *= 25 / wl; }
      b.pos[0] += b.v[0] * dt; b.pos[1] += b.v[1] * dt; b.pos[2] += b.v[2] * dt;
      const [qw, qx, qy, qz] = b.q, [wx, wy, wz] = b.w, h = 0.5 * dt;
      b.q[0] += h * (-wx * qx - wy * qy - wz * qz);
      b.q[1] += h * (wx * qw + wy * qz - wz * qy);
      b.q[2] += h * (wy * qw + wz * qx - wx * qz);
      b.q[3] += h * (wz * qw + wx * qy - wy * qx);
      const ql = Math.hypot(...b.q);
      for (let i = 0; i < 4; i++) b.q[i] /= ql;
      this._updateRotation(b);
      // asleep only when every point of the body is nearly still (a tall body
      // starting to tip turns slowly but its top already moves)
      const speed = Math.hypot(b.v[0], b.v[1], b.v[2]) + Math.hypot(b.w[0], b.w[1], b.w[2]) * b.radius;
      if (speed < 0.08) { b.sleepTime += dt; if (b.sleepTime > 0.5) { b.sleeping = true; b.v = [0, 0, 0]; b.w = [0, 0, 0]; } }
      else b.sleepTime = 0;
    }
    return this.contacts;
  }

  _newContact() {
    const c = this.pool.pop() || {};
    this.contacts.push(c);
    return c;
  }

  _collide() {
    for (const c of this.contacts) this.pool.push(c);
    this.contacts = [];
    const s = this.cell, inv = 1 / s;
    const bodies = this.bodies;
    for (const A of bodies) {
      if (A.sleeping) continue;
      const pw = A.pw;
      const nearGround = A.pos[1] - A.radius < 0.05;
      for (let i = 0; i < A.np; i++) {
        const x = pw[i * 3], y = pw[i * 3 + 1], z = pw[i * 3 + 2];
        if (nearGround && A.corner[i] && y < 0) {
          this._add(A, null, x, y, z, 0, 1, 0, -y, A.owner[i], -1, i, 0, true);
          continue;
        }
        const ix = Math.floor(x * inv), iy = Math.floor(y * inv), iz = Math.floor(z * inv);
        if (iy < 0) continue;
        const sid = this.static.get(cellKey(ix, iy, iz));
        if (sid === undefined) continue;
        const f = [x * inv - ix, y * inv - iy, z * inv - iz];
        const face = this._exitFace(f, (dx, dy, dz) => (iy + dy < 0) || this.static.has(cellKey(ix + dx, iy + dy, iz + dz)));
        if (!face) continue;
        this._add(A, null, x, y, z, face[0], face[1], face[2], face[3] * s, A.owner[i], sid, i, 0, false);
      }
    }
    // dynamic pairs
    for (let a = 0; a < bodies.length; a++) {
      const A = bodies[a];
      for (let b = a + 1; b < bodies.length; b++) {
        const Bd = bodies[b];
        if (A.sleeping && Bd.sleeping) continue;
        const dx = A.pos[0] - Bd.pos[0], dy = A.pos[1] - Bd.pos[1], dz = A.pos[2] - Bd.pos[2];
        const rr = A.radius + Bd.radius;
        if (dx * dx + dy * dy + dz * dz > rr * rr) continue;
        if (A.sleeping) this._refreshPoints(A);
        if (Bd.sleeping) this._refreshPoints(Bd);
        const before = this.contacts.length;
        this._pointsInto(A, Bd, 0);
        this._pointsInto(Bd, A, 1);
        if (this.contacts.length === before) continue;
        // A sleeping body is woken only by something moving into it with real
        // speed; a gently resting neighbour treats it as solid ground so the
        // pair can fall asleep together.
        const S = A.sleeping ? A : Bd.sleeping ? Bd : null;
        if (!S) continue;
        const W = S === A ? Bd : A;
        if (Math.hypot(...W.v) > 0.4 || Math.hypot(...W.w) > 0.4) { this.wake(S); continue; }
        for (let i = before; i < this.contacts.length; i++) {
          const c = this.contacts[i];
          if (c.A === S) {
            c.A = W; c.B = null; c.nx = -c.nx; c.ny = -c.ny; c.nz = -c.nz;
            const t = c.chunkA; c.chunkA = c.chunkB; c.chunkB = t;
          } else c.B = null;
          c.sleeper = S;
        }
      }
    }
  }

  // points of P inside voxels of Q. side 0: P is contact body A (normal out of
  // Q into P). side 1: P is B, Q is A (normal flipped).
  _pointsInto(P, Q, side) {
    const s = this.cell, inv = 1 / s;
    const pw = P.pw, r2 = (Q.radius + s) * (Q.radius + s);
    for (let i = 0; i < P.np; i++) {
      const x = pw[i * 3], y = pw[i * 3 + 1], z = pw[i * 3 + 2];
      const ddx = x - Q.pos[0], ddy = y - Q.pos[1], ddz = z - Q.pos[2];
      if (ddx * ddx + ddy * ddy + ddz * ddz > r2) continue;
      const h = this.toHome(Q, x, y, z);
      const ix = Math.floor(h[0] * inv), iy = Math.floor(h[1] * inv), iz = Math.floor(h[2] * inv);
      const qid = Q.cells.get(cellKey(ix, iy, iz));
      if (qid === undefined) continue;
      const f = [h[0] * inv - ix, h[1] * inv - iy, h[2] * inv - iz];
      const face = this._exitFace(f, (dx, dy, dz) => Q.cells.has(cellKey(ix + dx, iy + dy, iz + dz)));
      if (!face) continue;
      const m = this.dirToWorld(Q, face[0], face[1], face[2]);
      if (side === 0) this._add(P, Q, x, y, z, m[0], m[1], m[2], face[3] * s, P.owner[i], qid, i, 0, false);
      else this._add(Q, P, x, y, z, -m[0], -m[1], -m[2], face[3] * s, qid, P.owner[i], i, 1, false);
    }
  }

  // nearest exposed face of a unit cell for fractional position f.
  // blocked(dx,dy,dz) says whether the neighbour through that face is solid.
  _exitFace(f, blocked) {
    let best = null, bd = Infinity;
    const cand = [[-1, 0, 0, f[0]], [1, 0, 0, 1 - f[0]], [0, -1, 0, f[1]], [0, 1, 0, 1 - f[1]], [0, 0, -1, f[2]], [0, 0, 1, 1 - f[2]]];
    for (const c of cand) {
      if (c[3] >= bd) continue;
      if (blocked(c[0], c[1], c[2])) continue;
      best = c; bd = c[3];
    }
    return best;
  }

  _add(A, B, x, y, z, nx, ny, nz, depth, chunkA, chunkB, pi, side, ground) {
    const c = this._newContact();
    c.A = A; c.B = B; c.px = x; c.py = y; c.pz = z;
    c.nx = nx; c.ny = ny; c.nz = nz; c.depth = depth;
    c.chunkA = chunkA; c.chunkB = chunkB; c.ground = ground; c.sleeper = null;
    const bk = ground ? 0 : B ? B.id + 2 : 1;
    c.key = A.id * 1e10 + bk * 1e5 + pi * 2 + side;
  }

  _solve(dt) {
    const cs = this.contacts;
    const n = cs.length;
    const warm = this.warm, next = new Map();
    for (let i = 0; i < n; i++) {
      const c = cs[i];
      const A = c.A, B = c.B;
      const rA = [c.px - A.pos[0], c.py - A.pos[1], c.pz - A.pos[2]];
      const rB = B ? [c.px - B.pos[0], c.py - B.pos[1], c.pz - B.pos[2]] : null;
      const nrm = [c.nx, c.ny, c.nz];
      const t1 = perp(nrm), t2 = crossv(nrm, t1);
      c.dirs = [nrm, t1, t2];
      c.rAx = []; c.rBx = []; c.IA = []; c.IB = []; c.invK = [];
      for (let d = 0; d < 3; d++) {
        const dir = c.dirs[d];
        const ra = crossv(rA, dir), ia = mulv(A.IinvW, ra);
        let k = A.invMass + dot(ra, ia);
        c.rAx.push(ra); c.IA.push(ia);
        if (B) {
          const rb = crossv(rB, dir), ib = mulv(B.IinvW, rb);
          k += B.invMass + dot(rb, ib);
          c.rBx.push(rb); c.IB.push(ib);
        }
        c.invK.push(1 / k);
      }
      c.approach = -this._relVel(c, 0);
      c.target = Math.min(3, this.beta / dt * Math.max(c.depth - this.slop, 0));
      c.mu = this.contactFriction ? this.contactFriction(c) : this.friction;
      const w = warm.get(c.key);
      c.l = w ? [w[0], w[1], w[2]] : [0, 0, 0];
      for (let d = 0; d < 3; d++) if (c.l[d]) this._applyImp(c, d, c.l[d]);
    }
    for (let it = 0; it < this.iterations; it++) {
      for (let i = 0; i < n; i++) {
        const c = cs[i];
        for (let d = 1; d < 3; d++) {
          const vt = this._relVel(c, d);
          const lim = c.mu * c.l[0];
          const nl = Math.max(-lim, Math.min(lim, c.l[d] - vt * c.invK[d]));
          const dl = nl - c.l[d];
          c.l[d] = nl;
          if (dl) this._applyImp(c, d, dl);
        }
        const vn = this._relVel(c, 0);
        const nl = Math.max(0, c.l[0] + (c.target - vn) * c.invK[0]);
        const dl = nl - c.l[0];
        c.l[0] = nl;
        if (dl) this._applyImp(c, 0, dl);
      }
    }
    for (let i = 0; i < n; i++) { const c = cs[i]; next.set(c.key, c.l); }
    this.warm = next;
  }

  _relVel(c, d) {
    const A = c.A, B = c.B, dir = c.dirs[d], ra = c.rAx[d];
    let v = A.v[0] * dir[0] + A.v[1] * dir[1] + A.v[2] * dir[2] + A.w[0] * ra[0] + A.w[1] * ra[1] + A.w[2] * ra[2];
    if (B) { const rb = c.rBx[d]; v -= B.v[0] * dir[0] + B.v[1] * dir[1] + B.v[2] * dir[2] + B.w[0] * rb[0] + B.w[1] * rb[1] + B.w[2] * rb[2]; }
    return v;
  }

  _applyImp(c, d, l) {
    const A = c.A, B = c.B, dir = c.dirs[d], ia = c.IA[d];
    A.v[0] += dir[0] * l * A.invMass; A.v[1] += dir[1] * l * A.invMass; A.v[2] += dir[2] * l * A.invMass;
    A.w[0] += ia[0] * l; A.w[1] += ia[1] * l; A.w[2] += ia[2] * l;
    if (B) {
      const ib = c.IB[d];
      B.v[0] -= dir[0] * l * B.invMass; B.v[1] -= dir[1] * l * B.invMass; B.v[2] -= dir[2] * l * B.invMass;
      B.w[0] -= ib[0] * l; B.w[1] -= ib[1] * l; B.w[2] -= ib[2] * l;
    }
  }
}

// ---- small vector helpers --------------------------------------------------

function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
export function crossv(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }
export function mulv(M, v) { return [M[0] * v[0] + M[1] * v[1] + M[2] * v[2], M[3] * v[0] + M[4] * v[1] + M[5] * v[2], M[6] * v[0] + M[7] * v[1] + M[8] * v[2]]; }
function perp(n) {
  const a = Math.abs(n[0]) > 0.6 ? [0, 1, 0] : [1, 0, 0];
  const t = crossv(n, a), l = Math.hypot(...t);
  return [t[0] / l, t[1] / l, t[2] / l];
}
export function quatToMat(q, R) {
  const [w, x, y, z] = q;
  R[0] = 1 - 2 * (y * y + z * z); R[1] = 2 * (x * y - w * z); R[2] = 2 * (x * z + w * y);
  R[3] = 2 * (x * y + w * z); R[4] = 1 - 2 * (x * x + z * z); R[5] = 2 * (y * z - w * x);
  R[6] = 2 * (x * z - w * y); R[7] = 2 * (y * z + w * x); R[8] = 1 - 2 * (x * x + y * y);
  return R;
}
function inv3(m) {
  const [a, b, c, d, e, f, g, h, i] = m;
  const A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
  const det = a * A + b * B + c * C;
  const k = 1 / det;
  return new Float64Array([A * k, -(b * i - c * h) * k, (b * f - c * e) * k, B * k, (a * i - c * g) * k, -(a * f - c * d) * k, C * k, -(a * h - b * g) * k, (a * e - b * d) * k]);
}
