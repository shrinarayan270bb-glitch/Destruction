// Stress solver: Rigid-Body-Spring Model (RBSM).
//
// Every chunk is rigid with 3 DOF (ux, uy, rotation). Every bond is three
// springs at its contact centroid: normal, shear and rotational. Solving
// K q = f (gravity + debris loads) with preconditioned conjugate gradient
// gives each bond its axial force N, shear V and bending moment M. Those are
// checked against the bond's limits:
//
//   tension fibre      N/L + 6|M|/L^2  > tension      (pulled / bent open)
//   compression fibre -N/L + 6|M|/L^2  > compression  (crushed)
//   shear              |V|/L           > shear
//
// Cantilevers, columns and counterweights all fall out of this naturally,
// because moments accumulate through the rotational springs.

import { GROUND } from './graph.js';
import { GRAVITY } from './materials.js';

const SHEAR_RATIO = 0.4;

export class StressSolver {
  constructor(structure) {
    this.st = structure;
    this.local = new Int32Array(structure.chunks.length).fill(-1);
    this.maxIter = 4000;
    this.tol = 1e-5;
    this.lastIters = 0;
    this.lastMs = 0;
  }

  // Solve one anchored island. Writes N/V/M/util into bonds and q into chunks.
  // Returns the max utilisation found.
  solveIsland(isl) {
    const t0 = now();
    const st = this.st;
    const ids = isl.chunks.filter(id => !st.chunks[id].destroyed);
    const n = ids.length;
    const local = this.local;
    ids.forEach((id, i) => { local[id] = i; });

    // ---- gather bonds and their Jacobians -------------------------------
    // each bond row: a, b (local, -1 = ground), then for the 3 springs:
    // k, Ja[3], Jb[3]
    const bondIds = [];
    for (const id of ids) {
      for (const bi of st.chunks[id].bonds) {
        const b = st.bonds[bi];
        if (b.broken) continue;
        if (b.a !== id) continue; // visit each bond once (from its 'a' side)
        if (b.b !== GROUND && local[b.b] < 0) continue;
        bondIds.push(bi);
      }
    }
    const m = bondIds.length;
    const A = new Int32Array(m), B = new Int32Array(m);
    const K = new Float64Array(m * 3);
    const J = new Float64Array(m * 3 * 6);
    for (let r = 0; r < m; r++) {
      const b = st.bonds[bondIds[r]];
      const ca = st.chunks[b.a];
      A[r] = local[b.a];
      B[r] = b.b === GROUND ? -1 : local[b.b];
      const cb = b.b === GROUND ? null : st.chunks[b.b];
      const nx = b.nx, ny = b.ny, tx = -ny, ty = nx;
      const rax = b.cx - ca.x, ray = b.cy - ca.y;
      const rbx = cb ? b.cx - cb.x : 0, rby = cb ? b.cy - cb.y : 0;
      const E = b.limits.stiffness * Math.max(b.integrity, 0.05);
      const L = b.L;
      K[r * 3 + 0] = E * L;
      K[r * 3 + 1] = E * L * SHEAR_RATIO;
      K[r * 3 + 2] = E * L * L * L / 12;
      const o = r * 18;
      // normal: delta_n = n.(ub-ua) + thb*cross(rb,n) - tha*cross(ra,n)
      J.set([-nx, -ny, -(rax * ny - ray * nx), nx, ny, (rbx * ny - rby * nx)], o);
      // shear
      J.set([-tx, -ty, -(rax * ty - ray * tx), tx, ty, (rbx * ty - rby * tx)], o + 6);
      // rotation
      J.set([0, 0, -1, 0, 0, 1], o + 12);
    }

    // ---- load vector -----------------------------------------------------
    const N3 = n * 3;
    const f = new Float64Array(N3);
    const x = new Float64Array(N3);
    for (let i = 0; i < n; i++) {
      const c = st.chunks[ids[i]];
      f[i * 3 + 0] = c.ext[0];
      f[i * 3 + 1] = -c.mass * GRAVITY + c.ext[1];
      f[i * 3 + 2] = c.ext[2];
      c.extSolved[0] = c.ext[0]; c.extSolved[1] = c.ext[1]; c.extSolved[2] = c.ext[2];
      x[i * 3] = c.qx; x[i * 3 + 1] = c.qy; x[i * 3 + 2] = c.qt;   // warm start
    }

    const mul = (v, out) => {
      out.fill(0);
      for (let r = 0; r < m; r++) {
        const a = A[r] * 3, bb = B[r] * 3;
        for (let s = 0; s < 3; s++) {
          const o = r * 18 + s * 6;
          let d = J[o] * v[a] + J[o + 1] * v[a + 1] + J[o + 2] * v[a + 2];
          if (bb >= 0) d += J[o + 3] * v[bb] + J[o + 4] * v[bb + 1] + J[o + 5] * v[bb + 2];
          const k = K[r * 3 + s] * d;
          out[a] += k * J[o]; out[a + 1] += k * J[o + 1]; out[a + 2] += k * J[o + 2];
          if (bb >= 0) { out[bb] += k * J[o + 3]; out[bb + 1] += k * J[o + 4]; out[bb + 2] += k * J[o + 5]; }
        }
      }
    };

    const precond = blockIC0(n, m, A, B, K, J);

    // ---- preconditioned conjugate gradient ------------------------------
    const r_ = new Float64Array(N3), z = new Float64Array(N3), p = new Float64Array(N3), Ap = new Float64Array(N3);
    mul(x, Ap);
    let fn = 0;
    for (let i = 0; i < N3; i++) { r_[i] = f[i] - Ap[i]; fn += f[i] * f[i]; }
    fn = Math.sqrt(fn) || 1;
    let rz = 0;
    precond(r_, z);
    for (let i = 0; i < N3; i++) { p[i] = z[i]; rz += r_[i] * z[i]; }
    let it = 0;
    for (; it < this.maxIter; it++) {
      let rn = 0;
      for (let i = 0; i < N3; i++) rn += r_[i] * r_[i];
      if (Math.sqrt(rn) < this.tol * fn) break;
      mul(p, Ap);
      let pAp = 0;
      for (let i = 0; i < N3; i++) pAp += p[i] * Ap[i];
      if (pAp <= 0) break;
      const alpha = rz / pAp;
      for (let i = 0; i < N3; i++) { x[i] += alpha * p[i]; r_[i] -= alpha * Ap[i]; }
      precond(r_, z);
      let rz2 = 0;
      for (let i = 0; i < N3; i++) rz2 += r_[i] * z[i];
      const beta = rz2 / rz;
      rz = rz2;
      for (let i = 0; i < N3; i++) p[i] = z[i] + beta * p[i];
    }
    this.lastIters = it;

    // ---- write back -------------------------------------------------------
    for (let i = 0; i < n; i++) {
      const c = st.chunks[ids[i]];
      c.qx = x[i * 3]; c.qy = x[i * 3 + 1]; c.qt = x[i * 3 + 2];
      c.util = 0;
    }
    let maxUtil = 0;
    for (let r = 0; r < m; r++) {
      const b = st.bonds[bondIds[r]];
      const a = A[r] * 3, bb = B[r] * 3;
      const d = [0, 0, 0];
      for (let s = 0; s < 3; s++) {
        const o = r * 18 + s * 6;
        let v = J[o] * x[a] + J[o + 1] * x[a + 1] + J[o + 2] * x[a + 2];
        if (bb >= 0) v += J[o + 3] * x[bb] + J[o + 4] * x[bb + 1] + J[o + 5] * x[bb + 2];
        d[s] = K[r * 3 + s] * v;
      }
      b.N = d[0]; b.V = d[1]; b.M = d[2];
      evaluateBond(b);
      if (b.util > maxUtil) maxUtil = b.util;
      st.chunks[b.a].util = Math.max(st.chunks[b.a].util, b.util);
      if (b.b !== GROUND) st.chunks[b.b].util = Math.max(st.chunks[b.b].util, b.util);
    }

    for (const id of ids) local[id] = -1;
    this.lastMs = now() - t0;
    return maxUtil;
  }
}

// Utilisation = load / capacity for the governing failure mode (1.0 = breaks).
// Plastic section of length L:
//   pulled  (N > 0):  N / (t L)  +  |M| / (t L^2 / 4)
//   pressed (N < 0):  |M| / (t L^2 / 4  +  |N| L / 2)   compression adds
//                     bending capacity, so a resting block holds until its
//                     load resultant leaves the contact (the no-tension rule)
//   crushed:          |N| / (c L)
//   sheared:          |V| / (s L + 0.6 |N|)            friction under compression
export function evaluateBond(b) {
  const L = b.L, lim = b.limits;
  const k = Math.max(b.integrity, 0);
  const M = Math.abs(b.M), V = Math.abs(b.V);
  const pull = Math.max(b.N, 0), press = Math.max(-b.N, 0);
  const FLOOR = 60; // N, keeps zero-tension contacts from tripping on solver noise
  const tCap = Math.max(lim.tension * k * L, FLOOR);
  const mCap = lim.tension * k * L * L / 4;
  let ub, mode;
  if (pull > 0) {
    const ua = pull / tCap, um = M / Math.max(mCap, FLOOR * L);
    ub = ua + um; mode = ua > um ? 'tension' : 'bending';
  } else {
    ub = M / Math.max(mCap + press * L / 2, FLOOR * L); mode = 'bending';
  }
  const uc = press / Math.max(lim.compression * k * L, FLOOR);
  const us = V / Math.max(lim.shear * k * L + 0.6 * press, FLOOR);
  let util = ub;
  if (uc > util) { util = uc; mode = 'compression'; }
  if (us > util) { util = us; mode = 'shear'; }
  b.util = util;
  b.mode = mode;
  return util;
}

// Block incomplete Cholesky, zero fill. The support graph is a 4-connected
// grid, which has no triangles, so IC(0) never needs a "shared neighbour"
// correction for off-diagonal blocks: L_ji = A_ji * L_ii^-T exactly.
// Returns z = M^-1 r as a function.
function blockIC0(n, m, A, B, K, J) {
  const Ad = new Float64Array(n * 9);       // diagonal blocks
  const Ao = new Float64Array(m * 9);       // off-diagonal block A[hi][lo] per bond
  const hiOf = new Int32Array(m), loOf = new Int32Array(m);
  for (let r = 0; r < m; r++) {
    const a = A[r], b = B[r];
    const hi = b < 0 ? -1 : Math.max(a, b), lo = b < 0 ? -1 : Math.min(a, b);
    hiOf[r] = hi; loOf[r] = lo;
    for (let s = 0; s < 3; s++) {
      const o = r * 18 + s * 6, k = K[r * 3 + s];
      for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) {
        Ad[a * 9 + i * 3 + j] += k * J[o + i] * J[o + j];
        if (b >= 0) {
          Ad[b * 9 + i * 3 + j] += k * J[o + 3 + i] * J[o + 3 + j];
          // A[hi][lo](i,j) = k * J_hi[i] * J_lo[j]
          const oh = hi === a ? 0 : 3, ol = lo === a ? 0 : 3;
          Ao[r * 9 + i * 3 + j] += k * J[o + oh + i] * J[o + ol + j];
        }
      }
    }
  }
  // adjacency: bonds where node is hi (lower) / lo (upper)
  const lowerStart = new Int32Array(n + 1), upperStart = new Int32Array(n + 1);
  for (let r = 0; r < m; r++) if (hiOf[r] >= 0) { lowerStart[hiOf[r] + 1]++; upperStart[loOf[r] + 1]++; }
  for (let i = 0; i < n; i++) { lowerStart[i + 1] += lowerStart[i]; upperStart[i + 1] += upperStart[i]; }
  const lower = new Int32Array(lowerStart[n]), upper = new Int32Array(upperStart[n]);
  const lf = lowerStart.slice(0, n), uf = upperStart.slice(0, n);
  for (let r = 0; r < m; r++) if (hiOf[r] >= 0) { lower[lf[hiOf[r]]++] = r; upper[uf[loOf[r]]++] = r; }

  const Ld = new Float64Array(n * 9);       // lower-triangular Cholesky of each pivot
  const Lo = new Float64Array(m * 9);       // L[hi][lo]
  const D = new Float64Array(9);
  for (let shift = 0; ; shift = shift ? shift * 4 : 1e-3) {
    let ok = true;
    for (let i = 0; i < n && ok; i++) {
      for (let t = 0; t < 9; t++) D[t] = Ad[i * 9 + t];
      for (let q = lowerStart[i]; q < lowerStart[i + 1]; q++) {
        const o = lower[q] * 9;
        for (let x = 0; x < 3; x++) for (let y = 0; y < 3; y++)
          D[x * 3 + y] -= Lo[o + x * 3] * Lo[o + y * 3] + Lo[o + x * 3 + 1] * Lo[o + y * 3 + 1] + Lo[o + x * 3 + 2] * Lo[o + y * 3 + 2];
      }
      for (let x = 0; x < 3; x++) D[x * 4] *= 1 + shift;
      ok = chol3(D, Ld, i * 9);
      if (!ok) break;
      // L[j][i] = A[j][i] * Ld_i^-T  for neighbours j > i
      for (let q = upperStart[i]; q < upperStart[i + 1]; q++) {
        const o = upper[q] * 9;
        for (let x = 0; x < 3; x++) {
          // solve row: X[x,:] * Ld^T = A[x,:]  -> forward substitution with Ld
          const l = i * 9;
          const y0 = Ao[o + x * 3] / Ld[l];
          const y1 = (Ao[o + x * 3 + 1] - Ld[l + 3] * y0) / Ld[l + 4];
          const y2 = (Ao[o + x * 3 + 2] - Ld[l + 6] * y0 - Ld[l + 7] * y1) / Ld[l + 8];
          Lo[o + x * 3] = y0; Lo[o + x * 3 + 1] = y1; Lo[o + x * 3 + 2] = y2;
        }
      }
    }
    if (ok) break;
    if (shift > 10) throw new Error('IC0 failed');
  }

  const y = new Float64Array(n * 3);
  return (r, z) => {
    // forward: L y = r
    for (let i = 0; i < n; i++) {
      let b0 = r[i * 3], b1 = r[i * 3 + 1], b2 = r[i * 3 + 2];
      for (let q = lowerStart[i]; q < lowerStart[i + 1]; q++) {
        const rr = lower[q], o = rr * 9, k = loOf[rr] * 3;
        b0 -= Lo[o] * y[k] + Lo[o + 1] * y[k + 1] + Lo[o + 2] * y[k + 2];
        b1 -= Lo[o + 3] * y[k] + Lo[o + 4] * y[k + 1] + Lo[o + 5] * y[k + 2];
        b2 -= Lo[o + 6] * y[k] + Lo[o + 7] * y[k + 1] + Lo[o + 8] * y[k + 2];
      }
      const l = i * 9;
      const y0 = b0 / Ld[l];
      const y1 = (b1 - Ld[l + 3] * y0) / Ld[l + 4];
      const y2 = (b2 - Ld[l + 6] * y0 - Ld[l + 7] * y1) / Ld[l + 8];
      y[i * 3] = y0; y[i * 3 + 1] = y1; y[i * 3 + 2] = y2;
    }
    // backward: L^T z = y
    for (let i = n - 1; i >= 0; i--) {
      let b0 = y[i * 3], b1 = y[i * 3 + 1], b2 = y[i * 3 + 2];
      for (let q = upperStart[i]; q < upperStart[i + 1]; q++) {
        const rr = upper[q], o = rr * 9, k = hiOf[rr] * 3;
        // subtract Lo^T * z_hi
        b0 -= Lo[o] * z[k] + Lo[o + 3] * z[k + 1] + Lo[o + 6] * z[k + 2];
        b1 -= Lo[o + 1] * z[k] + Lo[o + 4] * z[k + 1] + Lo[o + 7] * z[k + 2];
        b2 -= Lo[o + 2] * z[k] + Lo[o + 5] * z[k + 1] + Lo[o + 8] * z[k + 2];
      }
      const l = i * 9;
      const z2 = b2 / Ld[l + 8];
      const z1 = (b1 - Ld[l + 7] * z2) / Ld[l + 4];
      const z0 = (b0 - Ld[l + 3] * z1 - Ld[l + 6] * z2) / Ld[l];
      z[i * 3] = z0; z[i * 3 + 1] = z1; z[i * 3 + 2] = z2;
    }
  };
}

// Cholesky of a symmetric 3x3 (row-major) into lower-triangular out[o..o+8]
function chol3(D, out, o) {
  const l00 = D[0];
  if (!(l00 > 0)) return false;
  const a = Math.sqrt(l00);
  const b = D[3] / a, c = D[6] / a;
  const d2 = D[4] - b * b;
  if (!(d2 > 0)) return false;
  const d = Math.sqrt(d2);
  const e = (D[7] - c * b) / d;
  const f2 = D[8] - c * c - e * e;
  if (!(f2 > 0)) return false;
  out[o] = a; out[o + 1] = 0; out[o + 2] = 0;
  out[o + 3] = b; out[o + 4] = d; out[o + 5] = 0;
  out[o + 6] = c; out[o + 7] = e; out[o + 8] = Math.sqrt(f2);
  return true;
}

function invert3(m, o) {
  const a = m[o], b = m[o + 1], c = m[o + 2], d = m[o + 3], e = m[o + 4], f = m[o + 5], g = m[o + 6], h = m[o + 7], i = m[o + 8];
  const A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
  const det = a * A + b * B + c * C;
  if (Math.abs(det) < 1e-300) { m[o] = a ? 1 / a : 0; m[o + 4] = e ? 1 / e : 0; m[o + 8] = i ? 1 / i : 0; m[o + 1] = m[o + 2] = m[o + 3] = m[o + 5] = m[o + 6] = m[o + 7] = 0; return; }
  const id = 1 / det;
  m[o] = A * id; m[o + 1] = -(b * i - c * h) * id; m[o + 2] = (b * f - c * e) * id;
  m[o + 3] = B * id; m[o + 4] = (a * i - c * g) * id; m[o + 5] = -(a * f - c * d) * id;
  m[o + 6] = C * id; m[o + 7] = -(a * h - b * g) * id; m[o + 8] = (a * e - b * d) * id;
}

function now() { return (typeof performance !== 'undefined' ? performance : Date).now(); }
