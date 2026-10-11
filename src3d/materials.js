// 3D material table. A chunk is a 1 m voxel; its density is "effective":
// a voxel of slab is a 25 cm slab plus air, a voxel of wall is a 25 cm wall.
// Capacities are stresses on a bond face (N per m^2 of the 1 m^2 face):
//
//   density      kg/m^3 of the voxel
//   tension      N/m^2   pull apart; bending capacity follows as t*L^3/4
//   compression  N/m^2   crushing
//   shear        N/m^2   sliding and twisting
//   stiffness    relative Young's modulus, only affects load sharing
//   hp           explosion damage a chunk absorbs before it is destroyed
//   toughness    controls the energy needed to break a bond on impact
export const MATERIALS = {
  slab:     { label: 'Concrete slab',     density: 500,  tension: 90e3,  compression: 6e6,  shear: 120e3, stiffness: 30,  hp: 120, toughness: 9000,  color: '#a3a39c' },
  concrete: { label: 'Concrete wall',     density: 800,  tension: 90e3,  compression: 8e6,  shear: 150e3, stiffness: 30,  hp: 140, toughness: 9000,  color: '#8f8f88' },
  brick:    { label: 'Brick',             density: 450,  tension: 40e3,  compression: 4e6,  shear: 60e3,  stiffness: 15,  hp: 70,  toughness: 4000,  color: '#b0623f' },
  wood:     { label: 'Timber',            density: 150,  tension: 25e3,  compression: 1e6,  shear: 15e3,  stiffness: 10,  hp: 50,  toughness: 3000,  color: '#c49a5c' },
  steel:    { label: 'Steel column',      density: 600,  tension: 2e6,   compression: 6e6,  shear: 1e6,   stiffness: 200, hp: 260, toughness: 30000, color: '#4f6d8a' },
  lattice:  { label: 'Steel lattice',     density: 250,  tension: 2e6,   compression: 3e6,  shear: 1e6,   stiffness: 120, hp: 140, toughness: 14000, color: '#e0a526' },
  cable:    { label: 'Steel cable',       density: 60,   tension: 1.5e6, compression: 2e3,  shear: 2e3,   stiffness: 150, hp: 60,  toughness: 9000,  color: '#3b3f45' },
  drywall:  { label: 'Drywall',           density: 100,  tension: 5e3,   compression: 300e3, shear: 6e3,  stiffness: 3,   hp: 20,  toughness: 800,   color: '#d8d4c8' },
  stone:    { label: 'Stone masonry',     density: 1800, tension: 400e3, compression: 10e6, shear: 200e3, stiffness: 25,  hp: 110, toughness: 5000,  color: '#8c8270' },
  crate:    { label: 'Steel container',   density: 400,  tension: 600e3, compression: 3e6,  shear: 400e3, stiffness: 100, hp: 200, toughness: 20000, color: '#2f7a6a' },
  counterweight: { label: 'Counterweight block', density: 800, tension: 1e6, compression: 20e6, shear: 1e6, stiffness: 40, hp: 220, toughness: 20000, color: '#6c6a63' },
  ballast:  { label: 'Precast ballast',   density: 2400, tension: 1e6,   compression: 20e6, shear: 1e6,   stiffness: 40,  hp: 220, toughness: 20000, color: '#77736a' },
};

// Footings: bonds to the static ground. tension = 0 means "just resting".
export const FOOTINGS = {
  cast:    { label: 'Cast-in foundation', tension: 300e3, compression: 50e6, shear: 300e3, stiffness: 60 },
  bolted:  { label: 'Anchor bolts',       tension: 600e3, compression: 50e6, shear: 600e3, stiffness: 200 },
  resting: { label: 'Resting on ground',  tension: 0,     compression: 50e6, shear: 200e3, stiffness: 40 },
};

export const GRAVITY = 9.81;
