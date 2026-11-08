// Display-only omission in pinned body4.STL (SHA256 424b5ab4705ae612704c02c026b00ebdde8657404a1ef13a56ac001c6c8c9c5e).
// Faces [27712, 34130) are the absent front upright, foot and top mount; not the lift column or crossbeam.
export function removeAbsentBodyBracket(geometry) {
  const count = geometry.getAttribute('position').count;
  if (geometry.index !== null || count !== 219762)
    throw new Error('Expected the pinned body4 non-indexed mesh');
  const kept = [];
  for (let vertex = 0; vertex < count; vertex++)
    if (vertex < 83136 || vertex >= 102390) kept.push(vertex);
  geometry.setIndex(kept);
}
