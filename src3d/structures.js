// 3D presets on a 1 m voxel grid. x right, y up, z depth; row y=0 sits on the ground.
import { Structure, Builder } from './graph.js';

export function house(b, ox = 0, oz = 0) {
  b.at(ox, oz).foot('cast');
  const W = 10, D = 8;
  const walls = (y0, h, mat) => {
    b.box(0, y0, 0, W, h, 1, mat).box(0, y0, D - 1, W, h, 1, mat);
    b.box(0, y0, 1, 1, h, D - 2, mat).box(W - 1, y0, 1, 1, h, D - 2, mat);
  };
  // ground floor: brick shell + bearing wall down the middle
  walls(0, 3, 'brick');
  b.box(5, 0, 1, 1, 3, D - 2, 'brick');
  b.clear(4, 0, 0, 2, 2, 1);                                   // front door
  b.clear(5, 0, 3, 1, 2, 1);                                   // inner door
  for (const x of [1, 7]) b.clear(x, 1, 0, 2, 1, 1).clear(x, 1, D - 1, 2, 1, 1);   // windows
  b.clear(0, 1, 3, 1, 1, 2).clear(W - 1, 1, 3, 1, 1, 2);
  b.tag('house-base').box(0, 0, 1, 1, 1, D - 2, 'brick').box(W - 1, 0, 1, 1, 1, D - 2, 'brick')
    .box(1, 0, 0, 3, 1, 1, 'brick').box(6, 0, 0, 3, 1, 1, 'brick').box(1, 0, D - 1, 8, 1, 1, 'brick')
    .box(5, 0, 1, 1, 1, 2, 'brick').box(5, 0, 4, 1, 1, 3, 'brick')
    .set(0, 0, 0, 'brick').set(W - 1, 0, 0, 'brick').set(0, 0, D - 1, 'brick').set(W - 1, 0, D - 1, 'brick').tag(null);
  b.tag('house-wall').box(0, 1, 1, 1, 2, D - 2, 'brick').tag(null);
  b.clear(0, 1, 3, 1, 1, 2);
  b.box(0, 3, 0, W, 1, D, 'slab');                             // first floor
  // upper floor
  walls(4, 3, 'brick');
  b.box(5, 4, 1, 1, 3, D - 2, 'brick');
  b.clear(5, 4, 3, 1, 2, 1);
  for (const x of [1, 7]) b.clear(x, 5, 0, 2, 1, 1).clear(x, 5, D - 1, 2, 1, 1);
  b.box(0, 7, 0, W, 1, D, 'wood');                             // attic floor
  // gabled roof, ridge along z, gable ends in brick
  for (let k = 0; k < 5; k++) {
    const y = 8 + k;
    b.box(k, y, 0, 1, 1, D, 'wood').box(W - 1 - k, y, 0, 1, 1, D, 'wood');
    if (W - 2 - 2 * k > 0) b.box(k + 1, y, 0, W - 2 - 2 * k, 1, 1, 'brick').box(k + 1, y, D - 1, W - 2 - 2 * k, 1, 1, 'brick');
  }
  b.box(7, 8, 5, 1, 6, 1, 'brick');                            // chimney
  return { x0: ox, x1: ox + W, z0: oz, z1: oz + D, h: 14 };
}

export function office(b, ox = 0, oz = 0, storeys = 5) {
  b.at(ox, oz).foot('cast');
  const W = 13, D = 9;
  const cols = [];
  for (const x of [0, 4, 8, 12]) for (const z of [0, 4, 8]) cols.push([x, z]);
  for (let k = 0; k < storeys; k++) {
    const base = k * 4;
    for (const [x, z] of cols) {
      b.tag(k === 0 && x === 4 ? 'office-column' : null).box(x, base, z, 1, 3, 1, 'steel');
      if (k === 2) b.tag('office-storey').set(x, base + 1, z, 'steel');
    }
    b.tag(null);
    if (k === 2) for (const z of [0, 4, 8]) b.tag('office-edge').box(0, base, z, 1, 3, 1, 'steel').tag(null);
    b.box(0, base + 3, 0, W, 1, D, 'slab');
    // non-structural partitions standing on the slab (gap under the next slab)
    if (k > 0) b.box(k % 2 ? 2 : 10, base, 1, 1, 2, 3, 'drywall');
  }
  const top = storeys * 4;
  // rooftop water tank on legs over one bay
  for (const [x, z] of [[8, 4], [12, 4], [8, 8], [12, 8]]) b.box(x, top, z, 1, 1, 1, 'steel');
  b.box(8, top + 1, 4, 5, 2, 5, 'crate');
  return { x0: ox, x1: ox + W, z0: oz, z1: oz + D, h: top + 3 };
}

// Tower crane: 4-leg lattice mast on a resting ballast block, jib along +x.
export function crane(b, ox = 0, oz = 0, o = {}) {
  const mastH = o.mastH ?? 24, jib = o.jib ?? 18, cj = o.counterJib ?? 7;
  const hookAt = o.hookAt ?? 12, drop = o.drop ?? 12, bw = o.ballast ?? 1;
  const c = cj + 2;            // mast occupies x c..c+2, z 0..2
  b.at(ox, oz);
  b.foot('resting').box(c + 1 - bw, 0, 1 - bw, 2 * bw + 1, 2, 2 * bw + 1, 'ballast');   // just under the mast: it stands on its COM
  b.foot('cast');
  const m0 = 2, m1 = 2 + mastH;
  for (const [x, z] of [[c, 0], [c + 2, 0], [c, 2], [c + 2, 2]]) b.box(x, m0, z, 1, mastH, 1, 'lattice');
  for (let y = m0 + 2; y < m1; y += 3) {
    b.box(c + 1, y, 0, 1, 1, 1, 'lattice').box(c + 1, y, 2, 1, 1, 1, 'lattice');
    b.box(c, y, 1, 1, 1, 1, 'lattice').box(c + 2, y, 1, 1, 1, 1, 'lattice');
  }
  b.tag('crane-leg').box(c + 2, m0, 0, 1, 2, 1, 'lattice').box(c + 2, m0, 2, 1, 2, 1, 'lattice').tag(null);
  b.box(c, m1, 0, 3, 3, 3, 'steel');                         // slewing unit, as tall as the jib
  b.box(c + 1, m1 + 3, 1, 1, 4, 1, 'lattice');               // tower head
  // jib and counter-jib: two chords with posts, one voxel wide
  const truss = (x0, len) => {
    b.box(x0, m1, 1, len, 1, 1, 'lattice').box(x0, m1 + 2, 1, len, 1, 1, 'lattice');
    for (let x = x0; x < x0 + len; x += 2) b.set(x, m1 + 1, 1, 'lattice');
  };
  truss(c + 3, jib);
  truss(c - cj, cj);
  b.tag('crane-counterweight').box(c - cj, m1 - 3, 0, 3, 3, 3, 'counterweight').tag(null);
  if (drop > 0) {
    const hx = c + 3 + hookAt;
    b.tag('crane-cable').box(hx, m1 - drop, 1, 1, drop, 1, 'cable').tag(null);
    b.box(hx - 1, m1 - drop - 3, 0, 3, 3, 3, 'crate');
  }
  return { x0: ox + c - cj, x1: ox + c + 3 + jib, z0: oz - bw, z1: oz + 3 + bw, h: m1 + 6 };
}

export function leaningTower(b, ox = 0, oz = 0) {
  b.at(ox, oz).foot('resting');
  const lean = [0, 0, 0, 1, 1, 1, 1, 2];
  for (let k = 0; k < lean.length; k++) b.box(lean[k], k * 3, 0, 3, 3, 3, 'stone');
  b.foot('cast');
  b.tag('tower-toe').box(2, 0, 0, 1, 2, 3, 'stone').tag(null);
  return { x0: ox, x1: ox + 5, z0: oz, z1: oz + 3, h: 24 };
}

export const PRESETS = {
  site: {
    label: 'Construction site',
    build(b) {
      leaningTower(b, 0, 3);
      office(b, 8, 0);
      crane(b, 24, 3);
      house(b, 50, 0);
    },
  },
  house: { label: 'House', build(b) { house(b); } },
  office: { label: 'Office block', build(b) { office(b); } },
  crane: { label: 'Tower crane', build(b) { crane(b); } },
  tower: { label: 'Leaning tower', build(b) { leaningTower(b); } },
};

export function buildPreset(name) {
  const st = new Structure(1);
  PRESETS[name].build(new Builder(st));
  return st.finalize();
}
