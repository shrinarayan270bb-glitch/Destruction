// Material table. Capacities are per metre of bond contact length (2D section),
// so a bond of length L can carry `tension * L` newtons of pure pull.
// Densities are "effective": an I-beam or lattice cell is mostly air.
//
//   density      kg/m^3
//   tension      N/m   (pull apart, also limits bending via the extreme fibre)
//   compression  N/m   (crushing)
//   shear        N/m   (sliding)
//   stiffness    relative Young's modulus, only affects load sharing
//   hp           explosion damage a chunk absorbs before it is destroyed
//   toughness    impact impulse (N*s) that fully breaks a bond
export const MATERIALS = {
  concrete: { label: 'Reinforced concrete', density: 2400, tension: 420e3, compression: 20e6, shear: 160e3, stiffness: 30, hp: 120, toughness: 9000, color: '#9a9a94' },
  brick:    { label: 'Brick',               density: 1900, tension: 150e3, compression: 8e6,  shear: 70e3,  stiffness: 15, hp: 70,  toughness: 4000, color: '#b0623f' },
  wood:     { label: 'Timber',              density: 600,  tension: 120e3, compression: 2e6,  shear: 45e3,  stiffness: 10, hp: 50,  toughness: 3000, color: '#c49a5c' },
  steel:    { label: 'Steel I-beam',        density: 3000, tension: 2.5e6, compression: 6e6,  shear: 1.5e6, stiffness: 200, hp: 260, toughness: 30000, color: '#4f6d8a' },
  lattice:  { label: 'Steel lattice',       density: 1000, tension: 3e6,   compression: 4e6,  shear: 1.5e6, stiffness: 120, hp: 140, toughness: 14000, color: '#e0a526' },
  cable:    { label: 'Steel cable',         density: 250,  tension: 3e6,   compression: 5e3,  shear: 5e3,   stiffness: 150, hp: 60,  toughness: 9000, color: '#3b3f45' },
  drywall:  { label: 'Drywall',             density: 700,  tension: 8e3,   compression: 400e3, shear: 10e3, stiffness: 3,  hp: 20,  toughness: 800,  color: '#d8d4c8' },
  stone:    { label: 'Stone masonry',       density: 2200, tension: 400e3, compression: 12e6, shear: 300e3, stiffness: 25, hp: 110, toughness: 5000, color: '#8c8270' },
  ballast:  { label: 'Precast ballast',     density: 2400, tension: 2e6,   compression: 20e6, shear: 1.5e6, stiffness: 40, hp: 220, toughness: 20000, color: '#77736a' },
  crate:    { label: 'Steel container',     density: 1600, tension: 2e6,   compression: 4e6,  shear: 1e6,   stiffness: 100, hp: 200, toughness: 20000, color: '#2f7a6a' },
};

// How a structure is fixed to the ground. A footing is a bond to the static
// world, so it has its own limits. tension = 0 means "just resting":
// it can push down but never pull, which gives the classic middle-third rule.
export const FOOTINGS = {
  cast:    { label: 'Cast-in foundation', tension: 600e3, compression: 50e6, shear: 400e3, stiffness: 60 },
  bolted:  { label: 'Anchor bolts',       tension: 900e3, compression: 50e6, shear: 800e3, stiffness: 200 },
  resting: { label: 'Resting on ground',  tension: 0,     compression: 50e6, shear: 300e3, stiffness: 40 },
};

export const GRAVITY = 9.81;
