export function newEditor(side, feedback) {
  const editor = { side, dirty: { joints: false, gripper: false, tcp: false },
    joints: null, gripper: null, tcp: null };
  return followFeedback(editor, feedback);
}
export function followFeedback(editor, feedback) {
  if (!feedback?.fresh) return editor;
  if (!editor.dirty.joints && Array.isArray(feedback.joints)) editor.joints = feedback.joints.map(v => v * 180 / Math.PI);
  if (!editor.dirty.gripper && Number.isFinite(feedback.gripper)) editor.gripper = feedback.gripper;
  if (!editor.dirty.tcp && Array.isArray(feedback.tcp)) editor.tcp = feedback.tcp.map((v, i) => i < 3 ? v * 1000 : v * 180 / Math.PI);
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
  KeyW: { direction: 'forward', label: '前进' },
  KeyS: { direction: 'backward', label: '后退' },
  KeyA: { direction: 'left', label: '左移' },
  KeyD: { direction: 'right', label: '右移' },
  KeyQ: { direction: 'yaw-left', label: '左转' },
  KeyE: { direction: 'yaw-right', label: '右转' },
});
export function chassisKeyIntent(keys) {
  if (keys.has('Space')) return { kind: 'brake' };
  const codes = Object.keys(CHASSIS_KEYS).filter(code => keys.has(code));
  if (!codes.length) return { kind: 'idle' };
  if (codes.length > 1) return { kind: 'conflict' };
  return { kind: 'move', code: codes[0], ...CHASSIS_KEYS[codes[0]] };
}
