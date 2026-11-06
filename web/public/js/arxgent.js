const toolLabels = {get_system_status: '读取机器人状态', observe_scene: '查看相机', read_observation: '读取已有观测'};

export function requestId(crypto = globalThis.crypto) {
  const bytes = crypto.getRandomValues(new Uint8Array(16));
  bytes[6] = (bytes[6] & 15) | 64;
  bytes[8] = (bytes[8] & 63) | 128;
  const hex = Array.from(bytes, value => value.toString(16).padStart(2, '0')).join('');
  return [hex.slice(0, 8), hex.slice(8, 12), hex.slice(12, 16), hex.slice(16, 20), hex.slice(20)].join('-');
}

export function sendRequest(conversation, message, previous) {
  if (previous?.conversation_id === conversation && previous.message === message) return previous;
  return {action: 'send', conversation_id: conversation, message, request_id: requestId()};
}

export function messageElement(document, message) {
  const row = document.createElement('article');
  row.className = 'arxgent-message' + (message.role === 'user' ? ' user' : '');
  const label = document.createElement('span');
  label.className = 'arxgent-message-label';
  label.textContent = message.role === 'user' ? '你' : 'arxgent';
  const content = document.createElement('p');
  content.className = 'arxgent-message-content';
  content.textContent = message.content;
  row.append(label, content);
  return row;
}

export function activityText(state) {
  if (!state.available && state.unavailable_reason) return state.unavailable_reason;
  if (state.job?.status === 'cancelling') return '正在取消，等待当前云端请求结束…';
  if (state.job?.status === 'error') return state.job.error || '本轮未完成，请开始新对话后重试。';
  if (state.job?.status === 'cancelled') return '已取消本轮回复。可开始新对话。';
  const activity = state.activity || [];
  if (state.busy && state.job?.phase === 'tool') {
    const last = activity.at(-1);
    return '正在' + (toolLabels[last?.tool] || '读取信息') + '…';
  }
  if (state.busy) return activity.length ? '已读取信息，arxgent 正在整理回答…' : 'arxgent 正在思考…';
  if (activity.length) return '本轮已调用：' + [...new Set(activity.map(item => toolLabels[item.tool] || '读取信息'))].join(' · ');
  return '可以聊天，也可以让我查看状态和相机';
}

export function mountArxgent(document = globalThis.document) {
  const panel = document.getElementById('arxgent-panel');
  if (!panel) return;
  const $ = id => document.getElementById('arxgent-' + id);
  let state = null, pendingRequest = null, requesting = false, generation = 0;
  let timer, messageKey = '', requestError = '';
  const suggestions = [...panel.querySelectorAll('[data-arxgent-prompt]')];

  function render() {
    if (!state) return;
    const key = JSON.stringify(state.messages);
    if (key !== messageKey) {
      const nearBottom = $('messages').scrollHeight - $('messages').scrollTop - $('messages').clientHeight < 45;
      if (state.messages.length) {
        $('messages').replaceChildren(...state.messages.map(message => messageElement(document, message)));
      } else {
        const empty = document.createElement('p');
        empty.className = 'arxgent-empty';
        empty.textContent = '我是 arxgent。可以查看机器人状态和三路相机，和你一起判断现场情况。';
        $('messages').replaceChildren(empty);
      }
      if (nearBottom) $('messages').scrollTop = $('messages').scrollHeight;
      messageKey = key;
    }
    $('status').classList.toggle('online', state.available);
    $('status').lastChild.textContent = state.available ? (state.busy ? '正在回复' : '已连接') : '暂不可用';
    $('activity').textContent = activityText(state);
    $('activity').dataset.busy = String(state.busy);
    $('messages').setAttribute('aria-busy', String(state.busy));
    $('input').disabled = !state.available;
    $('send').disabled = requesting || state.busy || !state.available || !$('input').value.trim();
    $('new').disabled = requesting || state.busy;
    $('cancel').hidden = !state.busy;
    $('cancel').disabled = requesting || state.job?.status === 'cancelling';
    for (const button of suggestions) button.disabled = requesting || state.busy || !state.available;
    $('error').hidden = !requestError;
    $('error').textContent = requestError;
  }

  function accept(next) {
    state = next;
    if (pendingRequest && next.job?.id === pendingRequest.request_id) {
      if ($('input').value.trim() === pendingRequest.message) $('input').value = '';
      pendingRequest = null;
    }
    render();
  }

  async function request(options = {}) {
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 10000);
    try {
      const reply = await fetch('/api/arxgent', {credentials: 'same-origin', cache: 'no-store', ...options, signal: controller.signal});
      const data = await reply.json();
      if (!reply.ok || !data.ok) throw new Error(data.message || '对话暂不可用');
      return data;
    } finally {
      clearTimeout(timeout);
    }
  }

  function schedule() {
    clearTimeout(timer);
    timer = setTimeout(poll, state?.busy ? 1000 : 8000);
  }

  async function poll() {
    if (requesting) {schedule(); return;}
    const started = generation;
    try {
      const next = await request();
      if (started === generation) {requestError = ''; accept(next);}
    } catch (error) {
      if (started === generation) {
        requestError = '对话连接暂不可用，正在重连。';
        if (!state) {
          $('status').lastChild.textContent = '连接中断';
          $('error').hidden = false;
          $('error').textContent = requestError;
        } else render();
      }
    } finally {
      schedule();
    }
  }

  async function post(data) {
    if (requesting) return;
    requesting = true;
    generation++;
    clearTimeout(timer);
    requestError = '';
    render();
    try {
      accept(await request({method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(data)}));
    } catch (error) {
      requestError = error.name === 'AbortError' ? '请求超时，正在核对处理状态；请勿重复发送。' : error.message;
    } finally {
      requesting = false;
      render();
      schedule();
    }
  }

  $('form').addEventListener('submit', event => {
    event.preventDefault();
    const text = $('input').value.trim();
    if (!state?.available || state.busy || requesting || !text) return;
    pendingRequest = sendRequest(state.conversation_id, text, pendingRequest);
    post(pendingRequest);
  });
  $('input').addEventListener('input', render);
  $('input').addEventListener('keydown', event => {
    if (event.key === 'Enter' && !event.shiftKey && !event.isComposing && event.keyCode !== 229) {
      event.preventDefault();
      $('form').requestSubmit();
    }
  });
  $('cancel').addEventListener('click', () => {
    if (state?.busy) post({action: 'cancel', conversation_id: state.conversation_id, request_id: state.job.id});
  });
  $('new').addEventListener('click', async () => {
    if (!state || state.busy) return;
    pendingRequest = null;
    await post({action: 'new', conversation_id: state.conversation_id});
    $('input').focus();
  });
  for (const button of suggestions) button.addEventListener('click', () => {
    $('input').value = button.dataset.arxgentPrompt;
    $('input').focus();
    $('form').requestSubmit();
  });
  poll();
}

if (typeof document !== 'undefined') mountArxgent();
