// UI wiring: input, overlays, inspector, scenarios.
import { Simulation, fmtMass } from './sim.js';
import { Renderer, stressColor } from './render.js';
import { PRESETS } from './structures.js';
import { MATERIALS, FOOTINGS } from './materials.js';
import { GROUND } from './graph.js';

const $ = (id) => document.getElementById(id);
const planck = window.planck;
if (!planck) {
  $('hint').textContent = 'The physics library did not load. Check your connection and reload the page.';
  throw new Error('planck.js missing');
}

const SCENARIOS = [
  { tag: 'house-base', title: 'Undermine the house', note: 'Cut through every ground-floor wall. The whole house above drops as one body and only cracks where it lands.', act: 'remove' },
  { tag: 'house-wall', title: 'Breach the house end wall', note: 'The part that hung off that wall breaks away; the rest stands.', act: 'blast' },
  { tag: 'office-storey', title: 'Cut the office at the 3rd floor', note: 'Everything above falls as one block onto the floors below, which crush where it hits.', act: 'remove' },
  { tag: 'office-corner', title: 'Blast an upper-floor corner', note: 'The corner the blast left unsupported leaves as one piece. The building stays up.', act: 'blast' },
  { tag: 'office-column', title: 'Remove a ground-floor column', note: 'The bay above it can no longer span. It drops as one frame through every floor.', act: 'remove' },
  { tag: 'crane-counterweight', title: 'Drop the crane counterweight', note: 'The centre of mass slides toward the jib and leaves the ballast base. The crane tips.', act: 'remove' },
  { tag: 'crane-cable', title: 'Cut the hook cable', note: 'The load falls away. The centre of mass moves back over the base and the crane holds.', act: 'cut' },
  { tag: 'crane-leg', title: 'Cut the jib-side mast leg', note: 'One leg cannot resist the bending moment on its own.', act: 'remove' },
  { tag: 'tower-toe', title: 'Undermine the leaning tower', note: 'The base shrinks until the centre of mass sits outside it.', act: 'remove' },
];

let initialPreset = 'site';
try { const saved = localStorage.getItem('ds-preset'); if (saved && PRESETS[saved]) initialPreset = saved; } catch (e) { /* storage blocked */ }

const sim = new Simulation(planck, initialPreset);
const canvas = $('view');
const ren = new Renderer(canvas, sim);
let selected = null;

// ---- scene + controls -------------------------------------------------------

for (const [key, p] of Object.entries(PRESETS)) {
  const o = document.createElement('option');
  o.value = key; o.textContent = p.label;
  $('preset').appendChild(o);
}
$('preset').value = initialPreset;
$('preset').addEventListener('change', () => loadPreset($('preset').value));
$('reset').addEventListener('click', () => loadPreset(sim.preset));
$('fit').addEventListener('click', () => ren.fit());
$('pause').addEventListener('click', togglePause);
$('slow').addEventListener('click', () => {
  const on = sim.settings.timeScale === 1;
  sim.settings.timeScale = on ? 0.25 : 1;
  $('slow').setAttribute('aria-pressed', String(on));
});

function loadPreset(name) {
  sim.reset(name);
  selected = null;
  ren.hoverChunk = null;
  ren.criticalVersion = -1;
  try { localStorage.setItem('ds-preset', name); } catch (e) { /* storage blocked */ }
  buildScenarios();
  ren.fit();
}

function togglePause() {
  sim.settings.paused = !sim.settings.paused;
  $('pause').setAttribute('aria-pressed', String(sim.settings.paused));
  $('pause').textContent = sim.settings.paused ? 'Resume' : 'Pause';
}

function setTool(t) {
  ren.tool = t;
  for (const b of document.querySelectorAll('.seg button')) b.setAttribute('aria-pressed', String(b.dataset.tool === t));
  if (t !== 'inspect') selected = null;
}
for (const b of document.querySelectorAll('.seg button')) b.addEventListener('click', () => setTool(b.dataset.tool));

function setOverlay(key, on) {
  ren.view[key] = on;
  $('ov-' + key).setAttribute('aria-pressed', String(on));
}
for (const b of document.querySelectorAll('.chip')) b.addEventListener('click', () => setOverlay(b.dataset.ov, !ren.view[b.dataset.ov]));

const bindSlider = (id, fmt, apply) => {
  const el = $(id), out = $(id + '-out');
  const upd = () => { out.textContent = fmt(+el.value); apply(+el.value); };
  el.addEventListener('input', upd); upd();
};
let power = 160, weightT = 3;
bindSlider('radius', v => v.toFixed(1) + ' m', v => { ren.blastRadius = v; });
bindSlider('power', v => v + ' HP', v => { power = v; });
bindSlider('wmass', v => v.toFixed(1) + ' t', v => { weightT = v; });
bindSlider('fragility', v => v.toFixed(1) + '×', v => { sim.settings.fragility = v; });

function buildScenarios() {
  const box = $('scenarios');
  box.textContent = '';
  const tags = new Set(sim.st.chunks.map(c => c.tag).filter(Boolean));
  for (const sc of SCENARIOS) {
    if (!tags.has(sc.tag)) continue;
    const b = document.createElement('button');
    b.id = 'scen-' + sc.tag;
    const t = document.createElement('b'); t.textContent = sc.title;
    const n = document.createElement('span'); n.textContent = sc.note;
    b.append(t, n);
    b.addEventListener('click', () => runScenario(sc));
    box.appendChild(b);
  }
}

function runScenario(sc) {
  const chunks = sim.st.chunks.filter(c => c.tag === sc.tag && !c.destroyed);
  if (!chunks.length) { toast('Already gone. Rebuild the scene to try again.'); return; }
  if (sc.act === 'blast') {
    let x = 0, y = 0;
    for (const c of chunks) { const p = sim.chunkPos(c); x += p.x; y += p.y; }
    sim.explode(x / chunks.length, y / chunks.length, sc.tag === 'office-corner' ? 3 : 2.4, sc.tag === 'office-corner' ? 400 : 320);
  } else if (sc.act === 'cut') {
    const mid = chunks[Math.floor(chunks.length / 2)];
    sim.removeChunks([mid]);
  } else {
    sim.removeChunks(chunks);
  }
  if (sim.settings.paused) togglePause();
  toast(sc.title);
}

// ---- pointer input -----------------------------------------------------------

const pointers = new Map();
let pinch = null;

canvas.addEventListener('contextmenu', e => e.preventDefault());
canvas.addEventListener('pointerdown', (e) => {
  canvas.setPointerCapture(e.pointerId);
  pointers.set(e.pointerId, { x: e.offsetX, y: e.offsetY, sx: e.offsetX, sy: e.offsetY, moved: false, button: e.button });
  if (pointers.size === 2) {
    const [a, b] = [...pointers.values()];
    pinch = { d: Math.hypot(a.x - b.x, a.y - b.y), mx: (a.x + b.x) / 2, my: (a.y + b.y) / 2 };
    for (const p of pointers.values()) p.moved = true;
  }
});
canvas.addEventListener('pointermove', (e) => {
  const p = pointers.get(e.pointerId);
  ren.hover = ren.toWorld(e.offsetX, e.offsetY);
  if (!p) return;
  const dx = e.offsetX - p.x, dy = e.offsetY - p.y;
  p.x = e.offsetX; p.y = e.offsetY;
  if (pointers.size === 2 && pinch) {
    const [a, b] = [...pointers.values()];
    const d = Math.hypot(a.x - b.x, a.y - b.y), mx = (a.x + b.x) / 2, my = (a.y + b.y) / 2;
    ren.cam.x -= (mx - pinch.mx) / ren.cam.scale; ren.cam.y += (my - pinch.my) / ren.cam.scale;
    if (pinch.d > 0) ren.zoomAt(mx, my, d / pinch.d);
    pinch = { d, mx, my };
    return;
  }
  if (!p.moved && Math.hypot(p.x - p.sx, p.y - p.sy) > 6) p.moved = true;
  if (p.moved) { ren.cam.x -= dx / ren.cam.scale; ren.cam.y += dy / ren.cam.scale; }
});
const release = (e) => {
  const p = pointers.get(e.pointerId);
  pointers.delete(e.pointerId);
  if (pointers.size < 2) pinch = null;
  if (!p || p.moved || p.button !== 0 || e.type === 'pointercancel') return;
  act(ren.toWorld(e.offsetX, e.offsetY));
};
canvas.addEventListener('pointerup', release);
canvas.addEventListener('pointercancel', release);
canvas.addEventListener('pointerleave', (e) => { if (e.pointerType === 'mouse') ren.hover = null; });
canvas.addEventListener('wheel', (e) => { e.preventDefault(); ren.zoomAt(e.offsetX, e.offsetY, Math.exp(-e.deltaY * 0.0015)); }, { passive: false });

function act(w) {
  switch (ren.tool) {
    case 'blast': sim.explode(w.x, w.y, ren.blastRadius, power); break;
    case 'drill': if (!sim.drill(w.x, w.y)) toast('Nothing to drill there'); break;
    case 'weight': sim.dropWeight(w.x, Math.max(w.y, 0.6), weightT * 1000); break;
    case 'inspect': selected = sim.pick(w.x, w.y); break;
  }
}

window.addEventListener('keydown', (e) => {
  if (e.target.closest('input, select, textarea')) return;
  const k = e.key.toLowerCase();
  const tools = { 1: 'blast', 2: 'drill', 3: 'weight', 4: 'inspect' };
  if (tools[k]) setTool(tools[k]);
  else if (k === ' ') { e.preventDefault(); togglePause(); }
  else if (k === 'r') loadPreset(sim.preset);
  else if (k === 'f') ren.fit();
  else if (k === 's') setOverlay('stress', !ren.view.stress);
  else if (k === 'g') setOverlay('graph', !ren.view.graph);
  else if (k === 'c') setOverlay('com', !ren.view.com);
  else if (k === 'a') setOverlay('critical', !ren.view.critical);
  else if (k === 'd') setOverlay('deflect', !ren.view.deflect);
});

let toastTimer = 0;
function toast(text) {
  const el = $('toast');
  el.textContent = text; el.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.classList.remove('show'), 2200);
}

// ---- theme -------------------------------------------------------------------

const mq = window.matchMedia('(prefers-color-scheme: dark)');
mq.addEventListener?.('change', () => ren.refreshColors());
new MutationObserver(() => ren.refreshColors()).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme', 'class', 'style'] });

// ---- read-outs ---------------------------------------------------------------

const DIRS = { '1,0': 'Right', '-1,0': 'Left', '0,1': 'Above', '0,-1': 'Below' };

function renderInspector() {
  const c = selected && !selected.destroyed ? selected : ren.hoverChunk;
  $('insp-mode').textContent = selected && !selected.destroyed ? 'pinned' : 'hover a chunk';
  const box = $('inspect');
  if (!c || c.destroyed) {
    if (box.dataset.state !== 'empty') {
      box.innerHTML = '<div class="empty">Point at any chunk to see its mass, health and the load on each of its bonds. Pick Inspect to pin one.</div>';
      box.dataset.state = 'empty';
    }
    return;
  }
  box.dataset.state = 'chunk';
  const m = MATERIALS[c.mat];
  const isl = sim.st.islands.get(c.island);
  const sm = isl ? sim.islandSummary(isl) : null;
  const rows = [];
  for (const bi of c.bonds) {
    const b = sim.st.bonds[bi];
    if (b.broken) continue;
    let label;
    if (b.b === GROUND) label = 'Ground';
    else { const s = b.a === c.id ? 1 : -1; label = DIRS[`${b.nx * s},${b.ny * s}`] || 'Bond'; }
    const u = isl && isl.anchored ? b.util : 0;
    const col = stressColor(u, ren.colors);
    rows.push(`<div class="bondrow"><span>${label}</span><span class="meter"><i style="width:${Math.min(100, u * 100).toFixed(0)}%;background:${col}"></i></span><span>${(u * 100).toFixed(0)}%</span></div>`);
  }
  const state = !isl ? '—' : isl.anchored ? 'Standing' : 'Falling';
  const margin = sm && sm.span ? `${sm.margin.toFixed(2)} m ${sm.margin > 0 ? 'inside' : 'outside'}` : isl && !isl.anchored ? 'no support' : '—';
  const worst = c.bonds.map(bi => sim.st.bonds[bi]).filter(b => !b.broken).sort((a, b) => b.util - a.util)[0];
  box.innerHTML = `
    <dl class="kv">
      <dt>Material</dt><dd>${m.label}</dd>
      <dt>Mass</dt><dd>${c.mass.toFixed(0)} kg</dd>
      <dt>Health</dt><dd>${Math.max(0, c.health).toFixed(0)} / ${m.hp}</dd>
      ${c.footing && c.groundBond >= 0 ? `<dt>Footing</dt><dd>${FOOTINGS[c.footing].label}</dd>` : ''}
      <dt>State</dt><dd>${state}</dd>
      <dt>Island mass</dt><dd>${sm ? fmtMass(sm.mass) : '—'}</dd>
      <dt>Island COM</dt><dd>${sm ? `${sm.x.toFixed(2)}, ${sm.y.toFixed(2)} m` : '—'}</dd>
      <dt>COM vs base</dt><dd>${margin}</dd>
      ${worst && isl?.anchored ? `<dt>Worst bond</dt><dd>${worst.mode}, N ${(worst.N / 1000).toFixed(1)} kN, M ${(worst.M / 1000).toFixed(1)} kN·m</dd>` : ''}
    </dl>
    <div class="bonds">${rows.join('') || '<div class="empty">No intact bonds</div>'}</div>`;
}

let lastLogTop = null;
function renderLog() {
  if (sim.events[0] === lastLogTop) return;
  lastLogTop = sim.events[0];
  const ol = $('log');
  ol.textContent = '';
  for (const ev of sim.events.slice(0, 18)) {
    const li = document.createElement('li');
    li.dataset.kind = ev.kind;
    const t = document.createElement('time'); t.textContent = 't+' + ev.t.toFixed(1) + 's';
    const s = document.createElement('span'); s.textContent = ev.text;
    li.append(t, s);
    ol.appendChild(li);
  }
}

let solveAvg = 0;
function renderStats(fps) {
  const n = sim.counts();
  $('s-chunks').textContent = n.alive;
  $('s-bonds').textContent = n.bonds;
  $('s-anch').textContent = n.anchored;
  $('s-fall').textContent = n.falling;
  $('s-solve').textContent = solveAvg.toFixed(1) + ' ms';
  $('s-queue').textContent = sim.stats.queue;
  $('fps').textContent = fps.toFixed(0) + ' fps';
}

// ---- loop --------------------------------------------------------------------

const ro = new ResizeObserver(() => { ren.resize(); });
ro.observe(canvas);
ren.resize();
loadPreset(initialPreset);

let last = performance.now(), uiTimer = 0, frames = 0, fps = 60, hoverTimer = 0;
function frame(t) {
  const dt = Math.min(0.05, (t - last) / 1000);
  last = t;
  sim.update(dt);
  solveAvg += (sim.stats.solveMs - solveAvg) * 0.1;
  hoverTimer += dt;
  if (hoverTimer > 0.05) { hoverTimer = 0; ren.hoverChunk = ren.hover ? sim.pick(ren.hover.x, ren.hover.y) : null; }
  ren.draw();
  frames++; uiTimer += dt;
  if (uiTimer > 0.25) { fps = frames / uiTimer; frames = 0; uiTimer = 0; renderStats(fps); renderInspector(); renderLog(); }
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
