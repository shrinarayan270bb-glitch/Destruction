// Preset structures, authored on a grid of 0.5 m cells.
// x grows right, y grows up, row 0 sits on the ground.

import { Structure, Builder } from './graph.js';

export function house(b, ox = 0) {
  b.at(ox).foot('cast');
  const W = 22;
  // ground floor: brick outer walls (2 thick) and a brick bearing wall
  b.rect(0, 0, 2, 6, 'brick').rect(W - 2, 0, 2, 6, 'brick');
  b.tag('house-wall').rect(0, 0, 2, 6, 'brick').tag(null);
  b.rect(11, 0, 1, 6, 'brick');
  b.rect(0, 6, W, 1, 'concrete');                        // first floor slab
  // upper floor
  b.rect(0, 7, 2, 5, 'brick').rect(W - 2, 7, 2, 5, 'brick');
  b.rect(11, 7, 1, 5, 'brick');
  b.rect(5, 7, 1, 4, 'drywall');                         // partition, free under the attic floor
  b.rect(0, 12, W, 1, 'wood');                           // attic floor
  // pitched timber roof: stepped rafters meeting at a ridge
  for (let k = 0; k < 6; k++) {
    const y = 13 + k;
    b.rect(2 * k, y, 3, 1, 'wood');
    b.rect(W - 3 - 2 * k, y, 3, 1, 'wood');
  }
  b.rect(10, 19, 2, 1, 'wood');
  b.rect(11, 13, 1, 6, 'wood');                          // king post
  // chimney, close to the right-hand wall
  b.rect(17, 13, 2, 6, 'brick');
  return { x0: ox, x1: ox + W, h: 20 };
}

export function office(b, ox = 0, storeys = 6) {
  b.at(ox).foot('cast');
  const W = 29;
  const cols = [0, 9, 19, 28];
  for (let k = 0; k < storeys; k++) {
    const base = k * 7;
    for (const c of cols) b.tag(k === 0 && c === 9 ? 'office-column' : null).rect(c, base, 1, 6, 'steel');
    b.tag(null);
    b.rect(0, base + 6, W, 1, 'concrete');
    // non-structural partitions standing on the slab below (gap at the top)
    if (k > 0 && k % 2 === 1) b.rect(4, base, 1, 5, 'drywall');
    if (k > 0 && k % 2 === 0) b.rect(23, base, 1, 5, 'drywall');
  }
  const top = storeys * 7;
  // rooftop water tank on steel legs, legs sit over the columns
  b.rect(9, top, 1, 2, 'steel').rect(19, top, 1, 2, 'steel');
  b.rect(9, top + 2, 11, 2, 'crate');
  return { x0: ox, x1: ox + W, h: top + 5 };
}

// Tower crane. `c` is the left leg of a 4-wide ladder mast. The ballast
// block only rests on the ground, so the crane stands on its centre of mass:
// lose the counterweight and the COM leaves the base.
export function crane(b, ox = 0, o = {}) {
  const mastH = o.mastH ?? 36;
  const jib = o.jib ?? 28;
  const cj = o.counterJib ?? 10;
  const hookAt = o.hookAt ?? 21;   // cells out along the jib
  const drop = o.drop ?? 22;       // cable length in cells
  const crateW = o.crateW ?? 4, crateH = o.crateH ?? 3;
  const c = cj + 3;
  b.at(ox);
  // concrete ballast base, resting
  const bw = o.ballast ?? 4;   // half-width in cells, centred on the mast
  b.foot('resting').rect(c + 2 - bw, 0, 2 * bw, 3, 'ballast');
  b.foot('cast');
  // ladder mast: two legs + a rung every 3 rows
  const m0 = 3, m1 = 3 + mastH;
  b.rect(c, m0, 1, mastH, 'lattice').rect(c + 3, m0, 1, mastH, 'lattice');
  b.tag('crane-leg').rect(c + 3, m0, 1, 2, 'lattice').tag(null);
  for (let y = m0 + 2; y < m1; y += 3) b.rect(c + 1, y, 2, 1, 'lattice');
  // slewing unit + operator cab
  b.rect(c - 1, m1, 6, 3, 'steel');
  b.rect(c + 4, m1 - 2, 2, 2, 'steel');
  // tower head
  b.rect(c + 1, m1 + 3, 2, 5, 'lattice');
  // jib and counter-jib: 3-row trusses (two chords + alternating posts)
  const truss = (x0, len) => {
    b.rect(x0, m1, len, 1, 'lattice').rect(x0, m1 + 2, len, 1, 'lattice');
    for (let x = x0; x < x0 + len; x++) if ((x - x0) % 2 === 0) b.set(x, m1 + 1, 'lattice');
  };
  truss(c + 5, jib);
  truss(c - 1 - cj, cj);
  // counterweight hanging under the counter-jib tail
  b.tag('crane-counterweight').rect(c - 1 - cj, m1 - 5, 5, 5, 'concrete').tag(null);
  // hook: cable + container
  if (drop > 0) {
    const hx = c + 5 + hookAt;
    b.tag('crane-cable').rect(hx, m1 - drop, 1, drop, 'cable').tag(null);
    b.rect(hx - Math.floor(crateW / 2), m1 - drop - crateH, crateW, crateH, 'crate');
  }
  return { x0: ox + c - 1 - cj, x1: ox + c + 5 + jib, h: m1 + 8 };
}

// Stone tower that leans one cell every 6 rows. It only rests on the ground
// (zero-tension footing), so it stands purely on centre of mass.
export function leaningTower(b, ox = 0) {
  b.at(ox).foot('resting');
  const lean = [0, 0, 1, 2, 3, 3];
  for (let k = 0; k < 6; k++) b.rect(lean[k], k * 6, 6, 6, 'stone');
  b.tag('tower-toe').rect(3, 0, 3, 2, 'stone').tag(null);
  b.foot('cast');
  return { x0: ox, x1: ox + 11, h: 36 };
}

export const PRESETS = {
  site: {
    label: 'Construction site',
    build(b) {
      leaningTower(b, 0);
      office(b, 14, 6);
      crane(b, 36, { mastH: 46, jib: 30, hookAt: 24, drop: 12 });
      house(b, 88);
    },
  },
  house: { label: 'House', build(b) { house(b, 0); } },
  office: { label: 'Office block', build(b) { office(b, 0, 6); } },
  crane: { label: 'Tower crane', build(b) { crane(b, 0); } },
  tower: { label: 'Leaning tower', build(b) { leaningTower(b, 0); } },
};

export function buildPreset(name) {
  const st = new Structure(0.5, 0.5);
  const b = new Builder(st);
  PRESETS[name].build(b);
  return st.finalize();
}
