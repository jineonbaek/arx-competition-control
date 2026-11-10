export function newEditor(side, feedback) {
  const editor = { side, dirty: { joints: false, gripper: false, tcp: false },
    joints: null, gripper: null, tcp: null };
  return followFeedback(editor, feedback);
}
export function followFeedback(editor, feedback) {
  if (!feedback?.fresh || !Array.isArray(feedback.base_wrist)) {
    editor.tcp=null;
    editor.dirty.tcp=false;
  }
  if (!feedback?.fresh) return editor;
  if (!editor.dirty.joints && Array.isArray(feedback.joints)) editor.joints = feedback.joints.map(v => v * 180 / Math.PI);
  if (!editor.dirty.gripper && Number.isFinite(feedback.gripper)) editor.gripper = feedback.gripper;
  if (!editor.dirty.tcp && Array.isArray(feedback.base_wrist)) editor.tcp = feedback.base_wrist.slice();
  return editor;
}
export function checkedNumber(value, min = -Infinity, max = Infinity) {
  if (typeof value === 'boolean' || value === null || String(value).trim() === '') return { valid: false, message: '请输入有效数字' };
  const parsed = Number(value);
  if (!Number.isFinite(parsed)) return { valid: false, message: '请输入有效数字' };
  if (parsed < min || parsed > max) return { valid: false, message: '目标超出配置范围' };
  return { valid: true, value: parsed, message: '' };
}
export function validBounds(pair) {
  return Array.isArray(pair) && pair.length === 2 && pair.every(Number.isFinite) && pair[0] < pair[1];
}

export const CHASSIS_KEYS = Object.freeze({
  KeyW: { axis: 'x', sign: 1, label: '前进' },
  KeyS: { axis: 'x', sign: -1, label: '后退' },
  KeyA: { axis: 'y', sign: 1, label: '左移' },
  KeyD: { axis: 'y', sign: -1, label: '右移' },
  KeyQ: { axis: 'omega', sign: 1, label: '左转' },
  KeyE: { axis: 'omega', sign: -1, label: '右转' },
});
export function chassisKeyIntent(keys) {
  if (keys.has('Space')) return { kind: 'brake' };
  const codes = Object.keys(CHASSIS_KEYS).filter(code => keys.has(code));
  if (!codes.length) return { kind: 'idle' };
  const axes = { x: 0, y: 0, omega: 0 };
  for (const code of codes) {
    const { axis, sign } = CHASSIS_KEYS[code];
    if (axes[axis] !== 0) return { kind: 'conflict' };
    axes[axis] = sign;
  }
  return { kind: 'move', codes, ...axes };
}
