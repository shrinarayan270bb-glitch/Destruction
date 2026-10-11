// Canvas renderer. Reads simulation state, draws nothing it does not own.
import { GROUND } from './graph.js';
import { MATERIALS } from './materials.js';

export class Renderer {
  constructor(canvas, sim) {
    this.cv = canvas;
    this.ctx = canvas.getContext('2d');
    this.sim = sim;
    this.cam = { x: 20, y: 10, scale: 20 };
    this.view = { stress: true, graph: false, com: true, critical: false, deflect: false };
    this.hover = null;          // world point
    this.hoverChunk = null;
    this.tool = 'blast';
    this.blastRadius = 2.5;
    this.colors = {};
    this.critical = null; this.criticalVersion = -1;
    this.refreshColors();
  }

  refreshColors() {
    const cs = getComputedStyle(document.documentElement);
    for (const k of ['sky-top', 'sky-bottom', 'ground', 'ground-line', 'ink', 'muted', 'accent', 'ok', 'warn', 'crit', 'grid'])
      this.colors[k] = cs.getPropertyValue('--' + k).trim();
  }

  resize() {
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const r = this.cv.getBoundingClientRect();
    this.w = r.width; this.h = r.height;
    this.cv.width = Math.round(r.width * dpr);
    this.cv.height = Math.round(r.height * dpr);
    this.dpr = dpr;
  }

  fit() {
    let x0 = Infinity, x1 = -Infinity, y1 = 0;
    for (const c of this.sim.st.chunks) {
      if (c.destroyed) continue;
      x0 = Math.min(x0, c.x); x1 = Math.max(x1, c.x); y1 = Math.max(y1, c.y);
    }
    if (!isFinite(x0)) { x0 = 0; x1 = 20; y1 = 10; }
    // keep the ground near the bottom edge so the scene is not lost in sky
    const pad = 3, floor = Math.min(70, this.h * 0.14);
    const wW = x1 - x0 + pad * 2, wH = y1 + pad;
    this.cam.scale = Math.min(this.w / wW, (this.h - floor - 12) / wH);
    this.cam.x = (x0 + x1) / 2;
    this.cam.y = (this.h / 2 - floor) / this.cam.scale;
  }

  toScreen(x, y) { return [(x - this.cam.x) * this.cam.scale + this.w / 2, this.h / 2 - (y - this.cam.y) * this.cam.scale]; }
  toWorld(sx, sy) { return { x: (sx - this.w / 2) / this.cam.scale + this.cam.x, y: (this.h / 2 - sy) / this.cam.scale + this.cam.y }; }

  zoomAt(sx, sy, f) {
    const p = this.toWorld(sx, sy);
    this.cam.scale = Math.max(3, Math.min(160, this.cam.scale * f));
    const q = this.toWorld(sx, sy);
    this.cam.x += p.x - q.x; this.cam.y += p.y - q.y;
  }

  draw() {
    const { ctx, sim, cam } = this;
    const st = sim.st;
    const C = this.colors;
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);

    // sky
    const g = ctx.createLinearGradient(0, 0, 0, this.h);
    g.addColorStop(0, C['sky-top']); g.addColorStop(1, C['sky-bottom']);
    ctx.fillStyle = g; ctx.fillRect(0, 0, this.w, this.h);

    // metre grid (every 5 m) for scale
    ctx.strokeStyle = C.grid; ctx.lineWidth = 1;
    ctx.beginPath();
    const tl = this.toWorld(0, 0), br = this.toWorld(this.w, this.h);
    for (let x = Math.floor(tl.x / 5) * 5; x < br.x; x += 5) { const [sx] = this.toScreen(x, 0); ctx.moveTo(sx, 0); ctx.lineTo(sx, this.h); }
    for (let y = 5; y < tl.y; y += 5) { const [, sy] = this.toScreen(0, y); ctx.moveTo(0, sy); ctx.lineTo(this.w, sy); }
    ctx.stroke();
    ctx.fillStyle = C.muted; ctx.font = '11px "IBM Plex Mono", ui-monospace, monospace';
    for (let y = 5; y < tl.y; y += 5) { const [, sy] = this.toScreen(0, y); ctx.fillText(y + ' m', 6, sy - 3); }

    // ground
    const [, gy] = this.toScreen(0, 0);
    ctx.fillStyle = C.ground; ctx.fillRect(0, gy, this.w, this.h - gy);
    ctx.fillStyle = C['ground-line']; ctx.fillRect(0, gy, this.w, 2);

    const s = st.cell;
    const px = s * cam.scale;

    // deflection exaggeration: biggest standing displacement drawn as 0.35 m
    let dScale = 0;
    if (this.view.deflect) {
      let mx = 0;
      for (const c of st.chunks) if (!c.destroyed && st.islands.get(c.island)?.anchored) mx = Math.max(mx, Math.hypot(c.qx, c.qy));
      dScale = mx > 0 ? 0.35 / mx : 0;
    }

    if (this.view.critical && this.criticalVersion !== sim.topologyVersion) {
      this.critical = st.articulationPoints();
      this.criticalVersion = sim.topologyVersion;
    }

    // chunks
    const pos = new Array(st.chunks.length);
    for (const c of st.chunks) {
      if (c.destroyed) continue;
      const isl = st.islands.get(c.island);
      if (!isl) continue;
      let wx, wy, a = 0;
      if (isl.body) {
        const p = isl.body.getWorldPoint(c.local);
        wx = p.x; wy = p.y; a = isl.body.getAngle();
      } else {
        wx = c.x + c.qx * dScale; wy = c.y + c.qy * dScale; a = c.qt * dScale;
      }
      pos[c.id] = [wx, wy, a];
      const [sx, sy] = this.toScreen(wx, wy);
      if (sx < -px || sx > this.w + px || sy < -px || sy > this.h + px) continue;
      const m = MATERIALS[c.mat];
      let fill = m.color;
      if (this.view.stress && !isl.body) fill = stressColor(c.util, C);
      ctx.save();
      ctx.translate(sx, sy);
      if (a) ctx.rotate(-a);
      ctx.fillStyle = fill;
      ctx.fillRect(-px / 2, -px / 2, px, px);
      const dmg = 1 - c.health / m.hp;
      if (dmg > 0.02) { ctx.fillStyle = `rgba(20,16,12,${Math.min(dmg, 1) * 0.55})`; ctx.fillRect(-px / 2, -px / 2, px, px); }
      if (isl.body && this.view.stress) { ctx.fillStyle = 'rgba(255,255,255,0.08)'; ctx.fillRect(-px / 2, -px / 2, px, px); }
      if (px > 5) { ctx.strokeStyle = 'rgba(0,0,0,0.22)'; ctx.lineWidth = 1; ctx.strokeRect(-px / 2 + 0.5, -px / 2 + 0.5, px - 1, px - 1); }
      if (this.view.critical && this.critical?.has(c.id)) {
        ctx.strokeStyle = C.crit; ctx.lineWidth = Math.max(1.5, px * 0.12);
        ctx.strokeRect(-px / 2 + 1, -px / 2 + 1, px - 2, px - 2);
        ctx.beginPath(); ctx.moveTo(-px / 2, px / 2); ctx.lineTo(px / 2, -px / 2); ctx.stroke();
      }
      ctx.restore();
    }

    // cracks: damaged or broken bonds between chunks that are still drawn
    ctx.lineCap = 'round';
    for (const b of st.bonds) {
      if (b.integrity >= 0.98 && !b.broken) continue;
      if (b.b === GROUND) continue;
      const pa = pos[b.a], pb = pos[b.b];
      if (!pa || !pb) continue;
      const sameIsland = st.chunks[b.a].island === st.chunks[b.b].island;
      if (b.broken && !sameIsland) continue;
      const mx = (pa[0] + pb[0]) / 2, my = (pa[1] + pb[1]) / 2;
      const ex = -(pb[1] - pa[1]) / 2, ey = (pb[0] - pa[0]) / 2;   // along the shared edge
      const [x0, y0] = this.toScreen(mx - ex, my - ey), [x1, y1] = this.toScreen(mx + ex, my + ey);
      const [xm, ym] = this.toScreen(mx + (pb[0] - pa[0]) * 0.12, my + (pb[1] - pa[1]) * 0.12);
      ctx.strokeStyle = b.broken ? 'rgba(15,12,10,0.9)' : `rgba(15,12,10,${0.25 + (1 - b.integrity) * 0.6})`;
      ctx.lineWidth = b.broken ? 2 : 1.2;
      ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(xm, ym); ctx.lineTo(x1, y1); ctx.stroke();
    }

    // weights
    for (const w of sim.weights) {
      const p = w.getPosition();
      const [sx, sy] = this.toScreen(p.x, p.y);
      ctx.save(); ctx.translate(sx, sy); ctx.rotate(-w.getAngle());
      ctx.fillStyle = '#2b2f33'; ctx.fillRect(-cam.scale / 2, -cam.scale / 2, cam.scale, cam.scale);
      ctx.strokeStyle = C.accent; ctx.lineWidth = 2; ctx.strokeRect(-cam.scale / 2 + 1, -cam.scale / 2 + 1, cam.scale - 2, cam.scale - 2);
      if (cam.scale > 18) { ctx.fillStyle = '#f5f2ea'; ctx.font = `600 ${Math.round(cam.scale * 0.32)}px "IBM Plex Mono", monospace`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText((w.getMass() / 1000).toFixed(1) + 't', 0, 0); }
      ctx.restore();
    }
    ctx.textAlign = 'left'; ctx.textBaseline = 'alphabetic';

    // support graph
    if (this.view.graph) {
      ctx.lineWidth = Math.max(1, Math.min(2.5, cam.scale * 0.06));
      for (const b of st.bonds) {
        if (b.broken) continue;
        const pa = pos[b.a];
        if (!pa) continue;
        const dyn = !!st.islands.get(st.chunks[b.a].island)?.body;
        const [x0, y0] = this.toScreen(pa[0], pa[1]);
        let x1, y1;
        if (b.b === GROUND) { [x1, y1] = this.toScreen(pa[0], 0); }
        else { const pb = pos[b.b]; if (!pb) continue; [x1, y1] = this.toScreen(pb[0], pb[1]); }
        ctx.strokeStyle = dyn ? 'rgba(120,120,120,0.5)' : stressColor(b.util, C);
        ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke();
        if (b.b === GROUND) {
          ctx.fillStyle = C.ink;
          ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x1 - 4, y1 + 6); ctx.lineTo(x1 + 4, y1 + 6); ctx.closePath(); ctx.fill();
        }
      }
      ctx.fillStyle = C.ink;
      const r = Math.max(1.2, Math.min(3, cam.scale * 0.07));
      for (const c of st.chunks) {
        const p = pos[c.id]; if (!p) continue;
        const [sx, sy] = this.toScreen(p[0], p[1]);
        ctx.beginPath(); ctx.arc(sx, sy, r, 0, Math.PI * 2); ctx.fill();
      }
    }

    // centre of mass + support span
    if (this.view.com) {
      for (const isl of st.islands.values()) {
        if (isl.chunks.length < (isl.anchored ? 4 : 10)) continue;
        const sm = sim.islandSummary(isl);
        const [cx, cy] = this.toScreen(sm.x, sm.y);
        if (isl.anchored && sm.span) {
          const inside = sm.margin > 0;
          const col = !inside ? C.crit : sm.margin < 0.4 ? C.warn : C.ok;
          const [l] = this.toScreen(sm.span.lo, 0), [rr] = this.toScreen(sm.span.hi, 0);
          ctx.fillStyle = col; ctx.fillRect(l, gy + 3, rr - l, 5);
          ctx.setLineDash([5, 4]); ctx.strokeStyle = col; ctx.lineWidth = 1.5;
          ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx, gy + 8); ctx.stroke(); ctx.setLineDash([]);
        }
        drawComMarker(ctx, cx, cy, isl.anchored ? 8 : 6, C);
        if (isl.anchored && cam.scale > 12) {
          ctx.fillStyle = C.ink; ctx.font = '600 11px "IBM Plex Mono", ui-monospace, monospace';
          const t = `${(sm.mass / 1000).toFixed(1)} t` + (sm.span ? `  ·  margin ${sm.margin.toFixed(2)} m` : '');
          ctx.fillText(t, cx + 12, cy - 10);
        }
      }
    }

    // particles
    for (const p of sim.particles) {
      const [sx, sy] = this.toScreen(p.x, p.y);
      const sz = Math.max(1.5, p.s * cam.scale);
      ctx.globalAlpha = Math.max(0, Math.min(1, p.life));
      ctx.fillStyle = p.color; ctx.fillRect(sx - sz / 2, sy - sz / 2, sz, sz);
    }
    ctx.globalAlpha = 1;

    // hover highlight + tool cursor
    if (this.hoverChunk && !this.hoverChunk.destroyed && pos[this.hoverChunk.id]) {
      const p = pos[this.hoverChunk.id];
      const [sx, sy] = this.toScreen(p[0], p[1]);
      ctx.save(); ctx.translate(sx, sy); ctx.rotate(-p[2]);
      ctx.strokeStyle = C.accent; ctx.lineWidth = 2.5; ctx.strokeRect(-px / 2 - 1, -px / 2 - 1, px + 2, px + 2);
      ctx.restore();
    }
    if (this.hover) {
      const [sx, sy] = this.toScreen(this.hover.x, this.hover.y);
      ctx.strokeStyle = C.accent; ctx.lineWidth = 1.5;
      if (this.tool === 'blast') {
        ctx.setLineDash([6, 5]);
        ctx.beginPath(); ctx.arc(sx, sy, this.blastRadius * cam.scale, 0, Math.PI * 2); ctx.stroke();
        ctx.setLineDash([]);
      } else if (this.tool === 'weight') {
        ctx.setLineDash([4, 4]); ctx.strokeRect(sx - cam.scale / 2, sy - cam.scale / 2, cam.scale, cam.scale); ctx.setLineDash([]);
      }
      ctx.beginPath(); ctx.moveTo(sx - 7, sy); ctx.lineTo(sx + 7, sy); ctx.moveTo(sx, sy - 7); ctx.lineTo(sx, sy + 7); ctx.stroke();
    }
  }
}

// green → amber → red by utilisation (1.0 = failure)
export function stressColor(u, C) {
  if (!(u > 0)) return C.ok;
  if (u < 0.5) return mix(C.ok, C.warn, u / 0.5);
  return mix(C.warn, C.crit, Math.min(1, (u - 0.5) / 0.5));
}

const hexCache = new Map();
function hex(h) {
  let v = hexCache.get(h);
  if (!v) { const n = parseInt(h.replace('#', ''), 16); v = [(n >> 16) & 255, (n >> 8) & 255, n & 255]; hexCache.set(h, v); }
  return v;
}
function mix(a, b, t) {
  const A = hex(a), B = hex(b);
  return `rgb(${Math.round(A[0] + (B[0] - A[0]) * t)},${Math.round(A[1] + (B[1] - A[1]) * t)},${Math.round(A[2] + (B[2] - A[2]) * t)})`;
}

// crash-test-dummy style quartered disc
function drawComMarker(ctx, x, y, r, C) {
  ctx.fillStyle = '#f7f3e8';
  ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2); ctx.fill();
  ctx.fillStyle = '#16181a';
  ctx.beginPath(); ctx.moveTo(x, y); ctx.arc(x, y, r, -Math.PI / 2, 0); ctx.closePath(); ctx.fill();
  ctx.beginPath(); ctx.moveTo(x, y); ctx.arc(x, y, r, Math.PI / 2, Math.PI); ctx.closePath(); ctx.fill();
  ctx.strokeStyle = '#16181a'; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2); ctx.stroke();
}
