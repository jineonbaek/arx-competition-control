import test from 'node:test';
import assert from 'node:assert/strict';
import {existsSync, readFileSync} from 'node:fs';
import {STLLoader} from '../public/vendor/lift2-preview/STLLoader.js';

const moduleURL = new URL('../public/js/robot-preview-geometry.js', import.meta.url);
async function api() {
  assert.ok(existsSync(moduleURL), 'Missing selective body accessory removal');
  return import(moduleURL);
}
function officialBody() {
  const bytes = readFileSync(new URL('../public/models/lift2/meshes/body4.STL', import.meta.url));
  return new STLLoader().parse(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength));
}
test('removes only the absent front upright and its top mount from the pinned body mesh', async () => {
  const {removeAbsentBodyBracket} = await api();
  const geometry = officialBody();
  const positions = geometry.getAttribute('position').array.slice();
  assert.equal(geometry.getAttribute('position').count, 219762);
  removeAbsentBodyBracket(geometry);
  assert.equal(geometry.index.count, 200508);
  assert.deepEqual(geometry.getAttribute('position').array, positions, 'Original mesh vertices must be retained');
  const kept = new Set(geometry.index.array);
  for (let vertex = 0; vertex < 219762; vertex++)
    assert.equal(kept.has(vertex), vertex < 83136 || vertex >= 102390, 'Unexpected change outside accessory faces');
});
test('rejects a different mesh instead of removing arbitrary geometry', async () => {
  const {removeAbsentBodyBracket} = await api();
  const geometry = officialBody();
  geometry.getAttribute('position').count = 3;
  assert.throws(() => removeAbsentBodyBracket(geometry), /pinned body4/);
});
