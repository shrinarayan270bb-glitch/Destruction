// 3D stress solver: Rigid-Body-Spring Model with 6 DOF per chunk
// (3 translations, 3 rotations). Each bond is six springs at its contact
// centroid: normal, two shears, torsion and two bending springs. Solving
// K q = f gives every bond its axial force N, shear V, torsion T and bending
// moment M, which are checked against a plastic section of side L:
//
//   pulled  (N > 0):  N/(t A) + M/(t L^3/4)
//   pressed (N < 0):  M / (t L^3/4 + |N| L/2)        compression helps
//   crushed:          |N|/(c A)
//   sheared:          (V/A + T/(0.208 L^3)) / (s + 0.6 |N|/A)
//
// K is assembled as 6x6 blocks and solved with conjugate gradient,
// preconditioned by block incomplete Cholesky (the 6-connected grid has no
// triangles, so IC(0) needs no fill). solveGen() is a generator that yields
// every few iterations so the caller can spread a solve across frames.

import { GROUND } from './graph.js';
import { GRAVITY } from './materials.js';

const B = 6, BB = 36;

export class StressSolver {
  constructor(structure) {
    this.st = structure;
    this.maxIter = 3000;
    this.tol = 1e-3;
    this.sliceIters = 6;
    this.lastIters = 0;
  }

  solve(isl, opts) {
    const g = this.solveGen(isl, opts);
    let r;
    while (!(r = g.next()).done);
    return r.value;
  }

  // opts.supports [{id, x,y,z, nx,ny,nz}] temporary supports (contacts) in the
  //               chunks' home frame; opts.accel {x,y,z} body acceleration felt
  //               by every chunk (default: gravity). Standing islands use
  //               neither and include debris loads.
  *solveGen(isl, opts = {}) {
    const st = this.st;
    const ids = isl.chunks.filter(id => !st.chunks[id].destroyed);
    st.splitMembers(ids);

    // ---- members of this island are the nodes (6 DOF each) --------------
    const memIndex = new Map();
    let mems = [];
    for (const id of ids) {
      const c = st.chunks[id];
      let mi = memIndex.get(c.member);
      if (mi === undefined) { mi = mems.length; memIndex.set(c.member, mi); mems.push({ id: c.member, chunks: [], mass: 0, x: 0, y: 0, z: 0 }); }
      const M = mems[mi];
      M.chunks.push(id); M.mass += c.mass; M.x += c.mass * c.x; M.y += c.mass * c.y; M.z += c.mass * c.z;
    }
    for (const M of mems) { M.x /= M.mass; M.y /= M.mass; M.z /= M.mass; }
    // top-down ordering gives the incomplete-Cholesky preconditioner the
    // fewest iterations on floor structures
    mems.sort((p, q) => q.y - p.y || p.x - q.x || p.z - q.z);
    const n = mems.length;
    const memOf = new Int32Array(st.chunks.length).fill(-1);
    mems.forEach((M, i) => { for (const id of M.chunks) memOf[id] = i; });

    // ---- rows: bonds between different members, footings, supports -------
    const rows = [];
    for (const id of ids) {
      for (const bi of st.chunks[id].bonds) {
        const b = st.bonds[bi];
        if (b.broken || b.a !== id) continue;
        if (b.b !== GROUND) {
          if (memOf[b.b] < 0) continue;
          if (memOf[b.b] === memOf[b.a]) { b.util = 0; continue; }   // inside a rigid member
        }
        rows.push(b);
      }
    }
    const mReal = rows.length;
    for (const sp of opts.supports || []) if (memOf[sp.id] >= 0) rows.push(supportRow(sp, st.cell));
    const m = rows.length;

    const A = new Int32Array(m), Bn = new Int32Array(m);
    const K = new Float64Array(m * 6);
    const J = new Float64Array(m * 6 * 12);
    const Kd = new Float64Array(n * BB);
    const Ko = new Float64Array(m * BB);
    const frame = new Float64Array(9);
    for (let r = 0; r < m; r++) {
      const b = rows[r];
      const a = memOf[b.a], bb = b.b === GROUND ? -1 : memOf[b.b];
      A[r] = a; Bn[r] = bb;
      const Ma = mems[a], Mb = bb >= 0 ? mems[bb] : null;
      basis(b.nx, b.ny, b.nz, frame);
      const ra = [b.cx - Ma.x, b.cy - Ma.y, b.cz - Ma.z];
      const rb = Mb ? [b.cx - Mb.x, b.cy - Mb.y, b.cz - Mb.z] : [0, 0, 0];
      const E = b.limits.stiffness * Math.max(b.integrity, 0.05);
      const L = b.L, Ar = L * L;
      const ks = [E * Ar, 0.4 * E * Ar, 0.4 * E * Ar, 0.4 * E * Ar * L * L / 6, E * Ar * L * L / 12, E * Ar * L * L / 12];
      for (let s = 0; s < 6; s++) {
        K[r * 6 + s] = ks[s];
        const d = [frame[(s % 3) * 3], frame[(s % 3) * 3 + 1], frame[(s % 3) * 3 + 2]];
        const o = (r * 6 + s) * 12;
        if (s < 3) {
          const xa = cross(ra, d), xb = cross(rb, d);
          J[o] = -d[0]; J[o + 1] = -d[1]; J[o + 2] = -d[2]; J[o + 3] = -xa[0]; J[o + 4] = -xa[1]; J[o + 5] = -xa[2];
          J[o + 6] = d[0]; J[o + 7] = d[1]; J[o + 8] = d[2]; J[o + 9] = xb[0]; J[o + 10] = xb[1]; J[o + 11] = xb[2];
        } else {
          J[o + 3] = -d[0]; J[o + 4] = -d[1]; J[o + 5] = -d[2];
          J[o + 9] = d[0]; J[o + 10] = d[1]; J[o + 11] = d[2];
        }
        const k = ks[s];
        for (let i = 0; i < 6; i++) {
          const jai = J[o + i], jbi = J[o + 6 + i];
          for (let j = 0; j < 6; j++) {
            Kd[a * BB + i * 6 + j] += k * jai * J[o + j];
            if (bb >= 0) Kd[bb * BB + i * 6 + j] += k * jbi * J[o + 6 + j];
          }
        }
        if (bb >= 0) {
          const hiIsA = a > bb;
          const oh = hiIsA ? 0 : 6, ol = hiIsA ? 6 : 0;
          for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) Ko[r * BB + i * 6 + j] += k * J[o + oh + i] * J[o + ol + j];
        }
      }
    }

    // ---- loads: body force on every chunk, debris loads, about member COM --
    const N6 = n * 6;
    const f = new Float64Array(N6), x = new Float64Array(N6);
    const acc = opts.accel || { x: 0, y: -GRAVITY, z: 0 };
    const standing = !opts.supports;
    if (!this.memberQ) this.memberQ = new Map();
    for (let i = 0; i < n; i++) {
      const M = mems[i];
      f[i * 6] = M.mass * acc.x; f[i * 6 + 1] = M.mass * acc.y; f[i * 6 + 2] = M.mass * acc.z;
      if (!standing) continue;
      for (const id of M.chunks) {
        const c = st.chunks[id], e = c.ext;
        if (e[0] || e[1] || e[2] || e[3] || e[4] || e[5]) {
          const rx = c.x - M.x, ry = c.y - M.y, rz = c.z - M.z;
          f[i * 6] += e[0]; f[i * 6 + 1] += e[1]; f[i * 6 + 2] += e[2];
          f[i * 6 + 3] += e[3] + ry * e[2] - rz * e[1];
          f[i * 6 + 4] += e[4] + rz * e[0] - rx * e[2];
          f[i * 6 + 5] += e[5] + rx * e[1] - ry * e[0];
        }
        for (let k = 0; k < 6; k++) c.extSolved[k] = e[k];
      }
      const q0 = this.memberQ.get(M.id);
      if (q0) for (let k = 0; k < 6; k++) x[i * 6 + k] = q0[k];   // warm start
    }

    const hi = new Int32Array(m), lo = new Int32Array(m);
    for (let r = 0; r < m; r++) { hi[r] = Bn[r] < 0 ? -1 : Math.max(A[r], Bn[r]); lo[r] = Bn[r] < 0 ? -1 : Math.min(A[r], Bn[r]); }
    // y = K v with 6x6 blocks, inner loops unrolled
    const offs = [];
    for (let r = 0; r < m; r++) if (hi[r] >= 0) offs.push(r);
    const mul = (v, out) => {
      for (let i = 0; i < n; i++) {
        const vi = i * 6;
        const x0 = v[vi], x1 = v[vi + 1], x2 = v[vi + 2], x3 = v[vi + 3], x4 = v[vi + 4], x5 = v[vi + 5];
        for (let a = 0, o = i * BB; a < 6; a++, o += 6) {
          out[vi + a] = Kd[o] * x0 + Kd[o + 1] * x1 + Kd[o + 2] * x2 + Kd[o + 3] * x3 + Kd[o + 4] * x4 + Kd[o + 5] * x5;
        }
      }
      for (let q = 0; q < offs.length; q++) {
        const r = offs[q];
        const h = hi[r] * 6, l = lo[r] * 6;
        const x0 = v[l], x1 = v[l + 1], x2 = v[l + 2], x3 = v[l + 3], x4 = v[l + 4], x5 = v[l + 5];
        let t0 = 0, t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0;
        for (let a = 0, o = r * BB; a < 6; a++, o += 6) {
          const k0 = Ko[o], k1 = Ko[o + 1], k2 = Ko[o + 2], k3 = Ko[o + 3], k4 = Ko[o + 4], k5 = Ko[o + 5];
          out[h + a] += k0 * x0 + k1 * x1 + k2 * x2 + k3 * x3 + k4 * x4 + k5 * x5;
          const ya = v[h + a];
          t0 += k0 * ya; t1 += k1 * ya; t2 += k2 * ya; t3 += k3 * ya; t4 += k4 * ya; t5 += k5 * ya;
        }
        out[l] += t0; out[l + 1] += t1; out[l + 2] += t2; out[l + 3] += t3; out[l + 4] += t4; out[l + 5] += t5;
      }
    };

    // tiny stiffness floor so a piece that has lost every connection (a
    // stale island mid-update) cannot make the system singular
    let dmax = 0;
    for (let i = 0; i < n; i++) for (let a = 0; a < 6; a++) dmax = Math.max(dmax, Kd[i * BB + a * 7]);
    for (let i = 0; i < n; i++) for (let a = 0; a < 6; a++) Kd[i * BB + a * 7] += dmax * 1e-9 + 1e-9;
    const precond = blockIC0(n, m, hi, lo, Kd, Ko);
    yield 0;

    const r_ = new Float64Array(N6), z = new Float64Array(N6), p = new Float64Array(N6), Ap = new Float64Array(N6);
    mul(x, Ap);
    let fn = 0;
    for (let i = 0; i < N6; i++) { r_[i] = f[i] - Ap[i]; fn += f[i] * f[i]; }
    fn = Math.sqrt(fn) || 1;
    precond(r_, z);
    let rz = 0;
    for (let i = 0; i < N6; i++) { p[i] = z[i]; rz += r_[i] * z[i]; }
    let it = 0;
    for (; it < this.maxIter; it++) {
      let rn = 0;
      for (let i = 0; i < N6; i++) rn += r_[i] * r_[i];
      if (Math.sqrt(rn) < this.tol * fn) break;
      mul(p, Ap);
      let pAp = 0;
      for (let i = 0; i < N6; i++) pAp += p[i] * Ap[i];
      if (!(pAp > 0)) break;
      const alpha = rz / pAp;
      for (let i = 0; i < N6; i++) { x[i] += alpha * p[i]; r_[i] -= alpha * Ap[i]; }
      precond(r_, z);
      let rz2 = 0;
      for (let i = 0; i < N6; i++) rz2 += r_[i] * z[i];
      const beta = rz2 / rz;
      rz = rz2;
      for (let i = 0; i < N6; i++) p[i] = z[i] + beta * p[i];
      if (it % this.sliceIters === this.sliceIters - 1) yield it;
    }
    this.lastIters = it;

    // ---- write back -------------------------------------------------------
    for (let i = 0; i < n; i++) {
      const M = mems[i], u = x.subarray(i * 6, i * 6 + 6);
      if (standing) this.memberQ.set(M.id, Float64Array.from(u));
      for (const id of M.chunks) {
        const c = st.chunks[id];
        c.util = 0;
        if (!standing) continue;
        // chunk displacement = member translation + rotation x offset (for display)
        const rx = c.x - M.x, ry = c.y - M.y, rz = c.z - M.z;
        c.q[0] = u[0] + u[4] * rz - u[5] * ry; c.q[1] = u[1] + u[5] * rx - u[3] * rz; c.q[2] = u[2] + u[3] * ry - u[4] * rx;
        c.q[3] = u[3]; c.q[4] = u[4]; c.q[5] = u[5];
      }
    }
    let maxUtil = 0;
    const d = new Float64Array(6);
    const memU = new Float64Array(n);
    for (let r = 0; r < mReal; r++) {
      const b = rows[r];
      const a = A[r] * 6, bb = Bn[r] * 6;
      for (let s = 0; s < 6; s++) {
        const o = (r * 6 + s) * 12;
        let v = 0;
        for (let k = 0; k < 6; k++) v += J[o + k] * x[a + k];
        if (bb >= 0) for (let k = 0; k < 6; k++) v += J[o + 6 + k] * x[bb + k];
        d[s] = K[r * 6 + s] * v;
      }
      b.N = d[0]; b.V = Math.hypot(d[1], d[2]); b.T = Math.abs(d[3]); b.M = Math.hypot(d[4], d[5]);
      evaluateBond(b);
      if (b.util > maxUtil) maxUtil = b.util;
      memU[A[r]] = Math.max(memU[A[r]], b.util);
      if (Bn[r] >= 0) memU[Bn[r]] = Math.max(memU[Bn[r]], b.util);
    }
    // a member is as loaded as its worst joint (colours whole panels)
    for (let i = 0; i < n; i++) for (const id of mems[i].chunks) st.chunks[id].util = memU[i];
    return maxUtil;
  }
}

function supportRow(sp, cell) {
  return { a: sp.id, b: GROUND, nx: sp.nx, ny: sp.ny, nz: sp.nz, cx: sp.x, cy: sp.y, cz: sp.z, L: cell, integrity: 1, limits: { stiffness: 200 } };
}

export function evaluateBond(b) {
  const L = b.L, A = L * L, lim = b.limits;
  const k = Math.max(b.integrity, 0);
  const pull = Math.max(b.N, 0), press = Math.max(-b.N, 0);
  const FLOOR = 60;
  const mCap = lim.tension * k * L * L * L / 4;
  let ub, mode;
  if (pull > 0) {
    const ua = pull / Math.max(lim.tension * k * A, FLOOR), um = b.M / Math.max(mCap, FLOOR * L);
    ub = ua + um; mode = ua > um ? 'tension' : 'bending';
  } else {
    ub = b.M / Math.max(mCap + press * L / 2, FLOOR * L); mode = 'bending';
  }
  const uc = press / Math.max(lim.compression * k * A, FLOOR);
  const tau = b.V / A + b.T / (0.208 * L * L * L);
  const us = tau / Math.max(lim.shear * k + 0.6 * press / A, FLOOR / A);
  let util = ub;
  if (uc > util) { util = uc; mode = 'compression'; }
  if (us > util) { util = us; mode = b.T / (0.208 * L * L * L) > b.V / A ? 'torsion' : 'shear'; }
  b.util = util;
  b.mode = mode;
  return util;
}

// orthonormal frame [n, t1, t2] into out[0..8]
function basis(nx, ny, nz, out) {
  const l = Math.hypot(nx, ny, nz) || 1;
  nx /= l; ny /= l; nz /= l;
  let ax = 1, ay = 0, az = 0;
  if (Math.abs(nx) > 0.6) { ax = 0; ay = 1; }
  // t1 = normalize(a - (a.n) n)
  const dn = ax * nx + ay * ny + az * nz;
  let tx = ax - dn * nx, ty = ay - dn * ny, tz = az - dn * nz;
  const tl = Math.hypot(tx, ty, tz);
  tx /= tl; ty /= tl; tz /= tl;
  out[0] = nx; out[1] = ny; out[2] = nz;
  out[3] = tx; out[4] = ty; out[5] = tz;
  out[6] = ny * tz - nz * ty; out[7] = nz * tx - nx * tz; out[8] = nx * ty - ny * tx;
}

function cross(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }

// Block incomplete Cholesky with 6x6 blocks. Returns z = M^-1 r.
function blockIC0(n, m, hi, lo, Kd, Ko) {
  const lowerStart = new Int32Array(n + 1), upperStart = new Int32Array(n + 1);
  for (let r = 0; r < m; r++) if (hi[r] >= 0) { lowerStart[hi[r] + 1]++; upperStart[lo[r] + 1]++; }
  for (let i = 0; i < n; i++) { lowerStart[i + 1] += lowerStart[i]; upperStart[i + 1] += upperStart[i]; }
  const lower = new Int32Array(lowerStart[n]), upper = new Int32Array(upperStart[n]);
  const lf = lowerStart.slice(0, n), uf = upperStart.slice(0, n);
  for (let r = 0; r < m; r++) if (hi[r] >= 0) { lower[lf[hi[r]]++] = r; upper[uf[lo[r]]++] = r; }

  const Ld = new Float64Array(n * BB), Lo = new Float64Array(m * BB);
  const D = new Float64Array(BB);
  for (let shift = 0; ; shift = shift ? shift * 4 : 1e-3) {
    let ok = true;
    for (let i = 0; i < n && ok; i++) {
      for (let t = 0; t < BB; t++) D[t] = Kd[i * BB + t];
      for (let q = lowerStart[i]; q < lowerStart[i + 1]; q++) {
        const o = lower[q] * BB;
        for (let a = 0; a < 6; a++) for (let b = 0; b <= a; b++) {
          let s = 0;
          for (let k = 0; k < 6; k++) s += Lo[o + a * 6 + k] * Lo[o + b * 6 + k];
          D[a * 6 + b] -= s;
        }
      }
      for (let a = 0; a < 6; a++) D[a * 7] *= 1 + shift;
      ok = cholB(D, Ld, i * BB);
      if (!ok) break;
      const l = i * BB;
      for (let q = upperStart[i]; q < upperStart[i + 1]; q++) {
        const o = upper[q] * BB;
        // row a of L[hi][i]: solve Ld * y = Ko[row a]
        for (let a = 0; a < 6; a++) {
          for (let c = 0; c < 6; c++) {
            let s = Ko[o + a * 6 + c];
            for (let k = 0; k < c; k++) s -= Ld[l + c * 6 + k] * Lo[o + a * 6 + k];
            Lo[o + a * 6 + c] = s / Ld[l + c * 7];
          }
        }
      }
    }
    if (ok) break;
    if (shift > 10) throw new Error('IC0 failed');
  }

  const y = new Float64Array(n * 6);
  return (r, z) => {
    for (let i = 0; i < n; i++) {
      const vi = i * 6;
      let t0 = r[vi], t1 = r[vi + 1], t2 = r[vi + 2], t3 = r[vi + 3], t4 = r[vi + 4], t5 = r[vi + 5];
      for (let q = lowerStart[i]; q < lowerStart[i + 1]; q++) {
        const rr = lower[q], o = rr * BB, k = lo[rr] * 6;
        const y0 = y[k], y1 = y[k + 1], y2 = y[k + 2], y3 = y[k + 3], y4 = y[k + 4], y5 = y[k + 5];
        t0 -= Lo[o] * y0 + Lo[o + 1] * y1 + Lo[o + 2] * y2 + Lo[o + 3] * y3 + Lo[o + 4] * y4 + Lo[o + 5] * y5;
        t1 -= Lo[o + 6] * y0 + Lo[o + 7] * y1 + Lo[o + 8] * y2 + Lo[o + 9] * y3 + Lo[o + 10] * y4 + Lo[o + 11] * y5;
        t2 -= Lo[o + 12] * y0 + Lo[o + 13] * y1 + Lo[o + 14] * y2 + Lo[o + 15] * y3 + Lo[o + 16] * y4 + Lo[o + 17] * y5;
        t3 -= Lo[o + 18] * y0 + Lo[o + 19] * y1 + Lo[o + 20] * y2 + Lo[o + 21] * y3 + Lo[o + 22] * y4 + Lo[o + 23] * y5;
        t4 -= Lo[o + 24] * y0 + Lo[o + 25] * y1 + Lo[o + 26] * y2 + Lo[o + 27] * y3 + Lo[o + 28] * y4 + Lo[o + 29] * y5;
        t5 -= Lo[o + 30] * y0 + Lo[o + 31] * y1 + Lo[o + 32] * y2 + Lo[o + 33] * y3 + Lo[o + 34] * y4 + Lo[o + 35] * y5;
      }
      const l = i * BB;
      const y0 = t0 / Ld[l];
      const y1 = (t1 - Ld[l + 6] * y0) / Ld[l + 7];
      const y2 = (t2 - Ld[l + 12] * y0 - Ld[l + 13] * y1) / Ld[l + 14];
      const y3 = (t3 - Ld[l + 18] * y0 - Ld[l + 19] * y1 - Ld[l + 20] * y2) / Ld[l + 21];
      const y4 = (t4 - Ld[l + 24] * y0 - Ld[l + 25] * y1 - Ld[l + 26] * y2 - Ld[l + 27] * y3) / Ld[l + 28];
      const y5 = (t5 - Ld[l + 30] * y0 - Ld[l + 31] * y1 - Ld[l + 32] * y2 - Ld[l + 33] * y3 - Ld[l + 34] * y4) / Ld[l + 35];
      y[vi] = y0; y[vi + 1] = y1; y[vi + 2] = y2; y[vi + 3] = y3; y[vi + 4] = y4; y[vi + 5] = y5;
    }
    for (let i = n - 1; i >= 0; i--) {
      const vi = i * 6;
      let t0 = y[vi], t1 = y[vi + 1], t2 = y[vi + 2], t3 = y[vi + 3], t4 = y[vi + 4], t5 = y[vi + 5];
      for (let q = upperStart[i]; q < upperStart[i + 1]; q++) {
        const rr = upper[q], o = rr * BB, k = hi[rr] * 6;
        for (let b = 0, ob = o; b < 6; b++, ob += 6) {
          const zb = z[k + b];
          t0 -= Lo[ob] * zb; t1 -= Lo[ob + 1] * zb; t2 -= Lo[ob + 2] * zb; t3 -= Lo[ob + 3] * zb; t4 -= Lo[ob + 4] * zb; t5 -= Lo[ob + 5] * zb;
        }
      }
      const l = i * BB;
      const z5 = t5 / Ld[l + 35];
      const z4 = (t4 - Ld[l + 34] * z5) / Ld[l + 28];
      const z3 = (t3 - Ld[l + 27] * z4 - Ld[l + 33] * z5) / Ld[l + 21];
      const z2 = (t2 - Ld[l + 20] * z3 - Ld[l + 26] * z4 - Ld[l + 32] * z5) / Ld[l + 14];
      const z1 = (t1 - Ld[l + 13] * z2 - Ld[l + 19] * z3 - Ld[l + 25] * z4 - Ld[l + 31] * z5) / Ld[l + 7];
      const z0 = (t0 - Ld[l + 6] * z1 - Ld[l + 12] * z2 - Ld[l + 18] * z3 - Ld[l + 24] * z4 - Ld[l + 30] * z5) / Ld[l];
      z[vi] = z0; z[vi + 1] = z1; z[vi + 2] = z2; z[vi + 3] = z3; z[vi + 4] = z4; z[vi + 5] = z5;
    }
  };
}

// Dense Cholesky of a symmetric 6x6 (lower triangle used) into out[o..]
function cholB(D, out, o) {
  for (let i = 0; i < 36; i++) out[o + i] = 0;
  for (let j = 0; j < 6; j++) {
    let s = D[j * 6 + j];
    for (let k = 0; k < j; k++) s -= out[o + j * 6 + k] * out[o + j * 6 + k];
    if (!(s > 0)) return false;
    const d = Math.sqrt(s);
    out[o + j * 6 + j] = d;
    for (let i = j + 1; i < 6; i++) {
      let t = D[i * 6 + j];
      for (let k = 0; k < j; k++) t -= out[o + i * 6 + k] * out[o + j * 6 + k];
      out[o + i * 6 + j] = t / d;
    }
  }
  return true;
}
