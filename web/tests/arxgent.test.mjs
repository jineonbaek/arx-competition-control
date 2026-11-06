import test from 'node:test';
import assert from 'node:assert/strict';
import { requestId, sendRequest, messageElement, activityText } from '../public/js/arxgent.js';

test('request identities work on HTTP LAN without crypto.randomUUID and retries keep identity', () => {
  const id = requestId({getRandomValues: bytes => bytes.fill(19)});
  assert.match(id, /^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/);
  const first = sendRequest('conversation', 'hello', null);
  assert.deepEqual(sendRequest('conversation', 'hello', first), first);
  assert.notEqual(sendRequest('conversation', 'new text', first).request_id, first.request_id);
});

test('model and user markup is always text, never HTML', () => {
  const document = {createElement: tag => ({tag, children: [], append(...children) {this.children.push(...children);},
    set innerHTML(value) {throw new Error('HTML injection');}})};
  const payload = '<img src=x onerror=alert(1)> **reply**';
  const element = messageElement(document, {role: 'assistant', content: payload});
  assert.equal(element.children[1].textContent, payload);
  assert.equal(element.children[0].textContent, 'arxgent');
});

test('activity distinguishes camera reads from text-only thinking and tool failures', () => {
  assert.equal(activityText({busy: true, job: {status: 'running', phase: 'thinking'}, activity: []}), 'arxgent 正在思考…');
  assert.match(activityText({busy: true, job: {status: 'running', phase: 'tool'}, activity: [{tool: 'observe_scene', status: 'running'}]}), /查看相机/);
  assert.match(activityText({busy: false, job: {status: 'error', error: '网络不可用'}, activity: []}), /网络不可用/);
});
