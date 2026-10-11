// three.js renderer for the 3D sandbox. Reads simulation state only.
import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.min.js';
import { GROUND } from './graph.js';
import { MATERIALS } from './materials.js';

const MAX_WEIGHTS = 24, MAX_DUST = 700;

export class Renderer3D {
  constructor(canvas, overlay, sim) {
    this.canvas = canvas;
    this.overlay = overlay;
    this.octx = overlay.getContext('2d');
    this.sim = sim;
    this.view = { stress: true, graph: false, com: true, critical: false, deflect: false };
    this.tool = 'blast';
    this.blastRadius = 2.5;
    this.hover = null;          // {point, chunk}
    this.colors = {};
    this.critical = null; this.criticalVersion = -1;

    const r = this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    r.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    r.shadowMap.enabled = true;
    r.shadowMap.type = THREE.PCFSoftShadowMap;

    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(45, 1, 0.5, 900);
    this.orbit = { target: new THREE.Vector3(20, 6, 4), radius: 60, theta: -0.65, phi: 1.05 };

    this.hemi = new THREE.HemisphereLight(0xffffff, 0x666055, 1.6);
    this.scene.add(this.hemi);
    this.sun = new THREE.DirectionalLight(0xffffff, 2.2);
    this.sun.castShadow = true;
    this.sun.shadow.mapSize.set(2048, 2048);
    this.sun.shadow.bias = -0.0004;
    this.scene.add(this.sun, this.sun.target);

    this.groundMat = new THREE.MeshStandardMaterial({ color: 0x6e655a, roughness: 1 });
    const ground = new THREE.Mesh(new THREE.PlaneGeometry(600, 600), this.groundMat);
    ground.rotation.x = -Math.PI / 2;
    ground.receiveShadow = true;
    this.scene.add(ground);
    this.grid = new THREE.GridHelper(600, 120, 0x000000, 0x000000);
    this.grid.material.transparent = true; this.grid.material.opacity = 0.12;
    this.grid.position.y = 0.01;
    this.scene.add(this.grid);

    // dynamic overlays
    this.lines = makeLines(16000, true);           // support graph
    this.scene.add(this.lines);
    this.guides = makeLines(4000, true);           // support polygons + plumb lines
    this.scene.add(this.guides);
    this.hoverBox = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(1.04, 1.04, 1.04)), new THREE.LineBasicMaterial({ color: 0xf0a51c }));
    this.hoverBox.visible = false;
    this.scene.add(this.hoverBox);
    this.blastBall = new THREE.Mesh(new THREE.SphereGeometry(1, 24, 16), new THREE.MeshBasicMaterial({ color: 0xf0a51c, wireframe: true, transparent: true, opacity: 0.35 }));
    this.blastBall.visible = false;
    this.scene.add(this.blastBall);

    this.weightMesh = new THREE.InstancedMesh(new THREE.BoxGeometry(0.98, 0.98, 0.98), new THREE.MeshStandardMaterial({ color: 0x2b2f33, roughness: 0.6, metalness: 0.3 }), MAX_WEIGHTS);
    this.weightMesh.castShadow = true;
    this.scene.add(this.weightMesh);
    this.dust = new THREE.InstancedMesh(new THREE.BoxGeometry(1, 1, 1), new THREE.MeshLambertMaterial({ color: 0xffffff }), MAX_DUST);
    this.scene.add(this.dust);

    this.tmpM = new THREE.Matrix4(); this.tmpQ = new THREE.Quaternion(); this.tmpP = new THREE.Vector3(); this.tmpS = new THREE.Vector3(1, 1, 1);
    this.tmpC = new THREE.Color();
    this.raycaster = new THREE.Raycaster();
    this.groundPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
    this.refreshColors();
    this.setSim(sim);
  }

  setSim(sim) {
    this.sim = sim;
    if (this.voxels) { this.scene.remove(this.voxels); this.voxels.geometry.dispose(); }
    const n = sim.st.chunks.length;
    this.voxels = new THREE.InstancedMesh(new THREE.BoxGeometry(0.97, 0.97, 0.97), new THREE.MeshStandardMaterial({ roughness: 0.85, metalness: 0.04 }), n);
    this.voxels.castShadow = true;
    this.voxels.receiveShadow = true;
    for (let i = 0; i < n; i++) this.voxels.setColorAt(i, this.tmpC.set(0xffffff));
    this.scene.add(this.voxels);
    this.criticalVersion = -1;
    this.matColor = {};
    for (const [k, m] of Object.entries(MATERIALS)) this.matColor[k] = new THREE.Color(m.color);
    // shadow camera covers the scene
    let x0 = 1e9, x1 = -1e9, z0 = 1e9, z1 = -1e9, y1 = 0;
    for (const c of sim.st.chunks) { x0 = Math.min(x0, c.x); x1 = Math.max(x1, c.x); z0 = Math.min(z0, c.z); z1 = Math.max(z1, c.z); y1 = Math.max(y1, c.y); }
    this.bounds = { x0, x1, z0, z1, y1 };
    const cx = (x0 + x1) / 2, cz = (z0 + z1) / 2, span = Math.max(x1 - x0, z1 - z0, y1) / 2 + 30;
    this.sun.position.set(cx - 40, 80, cz + 55);
    this.sun.target.position.set(cx, 0, cz);
    const sc = this.sun.shadow.camera;
    sc.left = -span; sc.right = span; sc.top = span; sc.bottom = -span; sc.near = 1; sc.far = 260;
    sc.updateProjectionMatrix();
  }

  refreshColors() {
    const cs = getComputedStyle(document.documentElement);
    for (const k of ['sky-top', 'sky-bottom', 'ground', 'ink', 'muted', 'accent', 'ok', 'warn', 'crit'])
      this.colors[k] = cs.getPropertyValue('--' + k).trim();
    this.scene.background = new THREE.Color(this.colors['sky-bottom']);
    this.scene.fog = new THREE.Fog(new THREE.Color(this.colors['sky-bottom']), 140, 420);
    this.groundMat.color.set(this.colors.ground);
    this.okC = new THREE.Color(this.colors.ok); this.warnC = new THREE.Color(this.colors.warn); this.critC = new THREE.Color(this.colors.crit);
    const dark = document.documentElement.dataset.theme === 'dark' || (!document.documentElement.dataset.theme && matchMedia('(prefers-color-scheme: dark)').matches);
    this.grid.material.color.set(dark ? 0xffffff : 0x000000);
    this.hemi.intensity = dark ? 1.0 : 1.6;
    this.sun.intensity = dark ? 1.5 : 2.2;
  }

  resize() {
    const r = this.canvas.getBoundingClientRect();
    this.w = r.width; this.h = r.height;
    this.renderer.setSize(r.width, r.height, false);
    this.camera.aspect = r.width / Math.max(1, r.height);
    this.camera.updateProjectionMatrix();
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    this.overlay.width = Math.round(r.width * dpr); this.overlay.height = Math.round(r.height * dpr);
    this.dpr = dpr;
  }

  fit() {
    const b = this.bounds;
    this.orbit.target.set((b.x0 + b.x1) / 2, b.y1 * 0.35, (b.z0 + b.z1) / 2);
    const size = Math.max(b.x1 - b.x0, b.y1, b.z1 - b.z0) + 6;
    const fov = this.camera.fov * Math.PI / 180;
    const aspect = Math.min(1, this.camera.aspect || 1);
    this.orbit.radius = size / (2 * Math.tan(fov / 2) * aspect) * 1.05;
    this.orbit.theta = -0.65; this.orbit.phi = 1.08;
  }

  rotate(dx, dy) { this.orbit.theta -= dx * 0.006; this.orbit.phi = Math.max(0.08, Math.min(1.52, this.orbit.phi - dy * 0.006)); }
  pan(dx, dy) {
    const o = this.orbit, k = o.radius * 0.0016;
    const right = new THREE.Vector3(Math.cos(o.theta), 0, -Math.sin(o.theta));
    const fwd = new THREE.Vector3(Math.sin(o.theta), 0, Math.cos(o.theta));
    o.target.addScaledVector(right, -dx * k).addScaledVector(fwd, -dy * k);
  }
  zoom(f) { this.orbit.radius = Math.max(6, Math.min(400, this.orbit.radius * f)); }

  _placeCamera() {
    const o = this.orbit;
    const sp = Math.sin(o.phi);
    this.camera.position.set(o.target.x + o.radius * sp * Math.sin(o.theta), o.target.y + o.radius * Math.cos(o.phi), o.target.z + o.radius * sp * Math.cos(o.theta));
    this.camera.lookAt(o.target);
    this.camera.updateMatrixWorld();
  }

  // ray pick: {chunk, point} on a voxel, or {point} on the ground
  pick(sx, sy) {
    this._placeCamera();
    const ndc = new THREE.Vector2(sx / this.w * 2 - 1, -(sy / this.h) * 2 + 1);
    this.raycaster.setFromCamera(ndc, this.camera);
    this.voxels.computeBoundingSphere();
    const hit = this.raycaster.intersectObject(this.voxels, false)[0];
    if (hit && hit.instanceId !== undefined) {
      const c = this.sim.st.chunks[hit.instanceId];
      if (c && !c.destroyed) return { chunk: c, point: hit.point.clone(), normal: hit.face ? hit.face.normal.clone() : null };
    }
    const p = new THREE.Vector3();
    if (this.raycaster.ray.intersectPlane(this.groundPlane, p)) return { point: p };
    return null;
  }

  draw() {
    const sim = this.sim, st = sim.st;
    this._placeCamera();

    // deflection exaggeration: largest standing displacement shown as 0.4 m
    let dScale = 0;
    if (this.view.deflect) {
      let mx = 0;
      for (const c of st.chunks) if (!c.destroyed && st.islands.get(c.island)?.anchored) mx = Math.max(mx, Math.hypot(c.q[0], c.q[1], c.q[2]));
      dScale = mx > 0 ? 0.4 / mx : 0;
    }
    if (this.view.critical && this.criticalVersion !== sim.topologyVersion) {
      this.critical = st.articulationPoints();
      this.criticalVersion = sim.topologyVersion;
    }

    const M = this.tmpM, Q = this.tmpQ, P = this.tmpP, S = this.tmpS, C = this.tmpC;
    const pos = this.pos || (this.pos = new Float32Array(st.chunks.length * 3));
    if (pos.length < st.chunks.length * 3) this.pos = new Float32Array(st.chunks.length * 3);
    const zero = new THREE.Vector3(0, 0, 0);
    for (const c of st.chunks) {
      const isl = st.islands.get(c.island);
      if (c.destroyed || !isl) { M.compose(P.set(0, -999, 0), Q.identity(), zero); this.voxels.setMatrixAt(c.id, M); continue; }
      if (isl.body) {
        const b = isl.body, w = sim.physics.toWorld(b, c.x, c.y, c.z);
        P.set(w[0], w[1], w[2]); Q.set(b.q[1], b.q[2], b.q[3], b.q[0]);
      } else if (dScale) {
        P.set(c.x + c.q[0] * dScale, c.y + c.q[1] * dScale, c.z + c.q[2] * dScale);
        const rx = c.q[3] * dScale, ry = c.q[4] * dScale, rz = c.q[5] * dScale, ang = Math.hypot(rx, ry, rz);
        if (ang > 1e-9) Q.setFromAxisAngle(new THREE.Vector3(rx / ang, ry / ang, rz / ang), ang); else Q.identity();
      } else { P.set(c.x, c.y, c.z); Q.identity(); }
      pos[c.id * 3] = P.x; pos[c.id * 3 + 1] = P.y; pos[c.id * 3 + 2] = P.z;
      M.compose(P, Q, S);
      this.voxels.setMatrixAt(c.id, M);
      // colour: material, or stress ramp for standing chunks, darkened by damage
      if (this.view.critical && !isl.body && this.critical?.has(c.id)) C.copy(this.critC);
      else if (this.view.stress && !isl.body) this._ramp(c.util, C);
      else { C.copy(this.matColor[c.mat]); if (this.view.stress) C.lerp(this.matColor.slab, 0.25); }
      let dmg = 1 - c.health / MATERIALS[c.mat].hp;
      let worst = 1;
      for (const bi of c.bonds) { const b = st.bonds[bi]; if (!b.broken && b.integrity < worst) worst = b.integrity; }
      dmg = Math.max(dmg, (1 - worst) * 0.8);
      if (dmg > 0.02) C.multiplyScalar(1 - Math.min(dmg, 1) * 0.55);
      this.voxels.setColorAt(c.id, C);
    }
    this.voxels.instanceMatrix.needsUpdate = true;
    this.voxels.instanceColor.needsUpdate = true;

    // weights
    for (let i = 0; i < MAX_WEIGHTS; i++) {
      const w = sim.weights[i];
      if (w && !w.removed) { P.set(...w.pos); Q.set(w.q[1], w.q[2], w.q[3], w.q[0]); M.compose(P, Q, S); }
      else M.compose(P.set(0, -999, 0), Q.identity(), zero);
      this.weightMesh.setMatrixAt(i, M);
    }
    this.weightMesh.instanceMatrix.needsUpdate = true;

    // dust
    const ps = sim.particles;
    for (let i = 0; i < MAX_DUST; i++) {
      const p = ps[i];
      if (p) { const s = p.s * Math.min(1, p.life); M.compose(P.set(p.x, p.y, p.z), Q.identity(), new THREE.Vector3(s, s, s)); this.dust.setColorAt(i, C.set(p.color)); }
      else M.compose(P.set(0, -999, 0), Q.identity(), zero);
      this.dust.setMatrixAt(i, M);
    }
    this.dust.instanceMatrix.needsUpdate = true;
    if (this.dust.instanceColor) this.dust.instanceColor.needsUpdate = true;

    this._drawGraph(pos);
    this._drawGuides();

    // tool cursor
    const h = this.hover;
    this.hoverBox.visible = false;
    this.blastBall.visible = false;
    if (h) {
      if (h.chunk && !h.chunk.destroyed) {
        const c = h.chunk, isl = st.islands.get(c.island);
        if (isl) {
          this.hoverBox.visible = true;
          this.hoverBox.position.set(pos[c.id * 3], pos[c.id * 3 + 1], pos[c.id * 3 + 2]);
          if (isl.body) this.hoverBox.quaternion.set(isl.body.q[1], isl.body.q[2], isl.body.q[3], isl.body.q[0]); else this.hoverBox.quaternion.identity();
        }
      }
      if (this.tool === 'blast' && h.point) {
        this.blastBall.visible = true;
        this.blastBall.position.copy(h.point);
        this.blastBall.scale.setScalar(this.blastRadius);
      }
    }

    this.renderer.render(this.scene, this.camera);
    this._drawOverlay();
  }

  _ramp(u, out) {
    if (!(u > 0)) return out.copy(this.okC);
    if (u < 0.5) return out.copy(this.okC).lerp(this.warnC, u / 0.5);
    return out.copy(this.warnC).lerp(this.critC, Math.min(1, (u - 0.5) / 0.5));
  }

  _drawGraph(pos) {
    const L = this.lines;
    L.visible = this.view.graph;
    if (!L.visible) return;
    const st = this.sim.st;
    const P = L.geometry.attributes.position.array, Cc = L.geometry.attributes.color.array;
    const C = this.tmpC;
    let k = 0;
    const max = P.length / 6;
    for (const b of st.bonds) {
      if (b.broken || k >= max) continue;
      const a = st.chunks[b.a];
      if (a.destroyed) continue;
      const isl = st.islands.get(a.island);
      if (!isl) continue;
      const ax = pos[a.id * 3], ay = pos[a.id * 3 + 1], az = pos[a.id * 3 + 2];
      let bx, by, bz;
      if (b.b === GROUND) { bx = ax; by = 0; bz = az; }
      else { const o = b.b; bx = pos[o * 3]; by = pos[o * 3 + 1]; bz = pos[o * 3 + 2]; }
      P.set([ax, ay, az, bx, by, bz], k * 6);
      if (isl.body) C.setRGB(0.5, 0.5, 0.5); else this._ramp(b.util, C);
      Cc.set([C.r, C.g, C.b, C.r, C.g, C.b], k * 6);
      k++;
    }
    L.geometry.setDrawRange(0, k * 2);
    L.geometry.attributes.position.needsUpdate = true;
    L.geometry.attributes.color.needsUpdate = true;
  }

  _drawGuides() {
    const G = this.guides;
    G.visible = this.view.com;
    if (!G.visible) return;
    const st = this.sim.st;
    const P = G.geometry.attributes.position.array, Cc = G.geometry.attributes.color.array;
    const C = this.tmpC;
    let k = 0;
    const max = P.length / 6;
    this.comList = [];
    for (const isl of st.islands.values()) {
      if (isl.chunks.length < (isl.anchored ? 4 : 12)) continue;
      const sm = this.sim.islandSummary(isl);
      this.comList.push({ isl, sm });
      if (!isl.anchored || !sm.poly) continue;
      C.copy(sm.margin <= 0 ? this.critC : sm.margin < 0.5 ? this.warnC : this.okC);
      const poly = sm.poly;
      for (let i = 0; i < poly.length && k < max - 2; i++) {
        const [ax, az] = poly[i], [bx, bz] = poly[(i + 1) % poly.length];
        P.set([ax, 0.05, az, bx, 0.05, bz], k * 6); Cc.set([C.r, C.g, C.b, C.r, C.g, C.b], k * 6); k++;
      }
      // plumb line from the centre of mass to the ground, and a cross where it lands
      P.set([sm.x, sm.y, sm.z, sm.x, 0.05, sm.z], k * 6); Cc.set([C.r, C.g, C.b, C.r, C.g, C.b], k * 6); k++;
      P.set([sm.x - 0.6, 0.05, sm.z, sm.x + 0.6, 0.05, sm.z], k * 6); Cc.set([C.r, C.g, C.b, C.r, C.g, C.b], k * 6); k++;
      P.set([sm.x, 0.05, sm.z - 0.6, sm.x, 0.05, sm.z + 0.6], k * 6); Cc.set([C.r, C.g, C.b, C.r, C.g, C.b], k * 6); k++;
    }
    G.geometry.setDrawRange(0, k * 2);
    G.geometry.attributes.position.needsUpdate = true;
    G.geometry.attributes.color.needsUpdate = true;
  }

  _drawOverlay() {
    const ctx = this.octx;
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    ctx.clearRect(0, 0, this.w, this.h);
    if (!this.view.com || !this.comList) return;
    const v = new THREE.Vector3();
    ctx.font = '600 11px "IBM Plex Mono", ui-monospace, monospace';
    for (const { isl, sm } of this.comList) {
      v.set(sm.x, sm.y, sm.z).project(this.camera);
      if (v.z > 1) continue;
      const x = (v.x + 1) / 2 * this.w, y = (1 - v.y) / 2 * this.h;
      const r = isl.anchored ? 8 : 6;
      ctx.fillStyle = '#f7f3e8';
      ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2); ctx.fill();
      ctx.fillStyle = '#16181a';
      ctx.beginPath(); ctx.moveTo(x, y); ctx.arc(x, y, r, -Math.PI / 2, 0); ctx.closePath(); ctx.fill();
      ctx.beginPath(); ctx.moveTo(x, y); ctx.arc(x, y, r, Math.PI / 2, Math.PI); ctx.closePath(); ctx.fill();
      ctx.strokeStyle = '#16181a'; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2); ctx.stroke();
      if (isl.anchored && this.w >= 600) {
        const t = `${(sm.mass / 1000).toFixed(1)} t` + (sm.margin !== null ? `  ·  margin ${sm.margin.toFixed(2)} m` : '');
        const tw = ctx.measureText(t).width;
        ctx.fillStyle = 'rgba(20,22,24,0.72)';
        ctx.fillRect(x + 11, y - 22, tw + 10, 18);
        ctx.fillStyle = '#f2efe6';
        ctx.fillText(t, x + 16, y - 9);
      }
    }
  }
}

function makeLines(maxSegments, colors) {
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(maxSegments * 6), 3));
  if (colors) g.setAttribute('color', new THREE.BufferAttribute(new Float32Array(maxSegments * 6), 3));
  g.setDrawRange(0, 0);
  const l = new THREE.LineSegments(g, new THREE.LineBasicMaterial({ vertexColors: true, depthTest: true }));
  l.frustumCulled = false;
  return l;
}
