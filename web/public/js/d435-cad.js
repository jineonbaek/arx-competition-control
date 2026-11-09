const byId = id => document.getElementById(id);
const button = byId('read');
const frame = byId('frame');
const targetFrames = 8;

button.onclick = async () => {
  button.disabled = true;
  frame.hidden = true;
  byId('rows').replaceChildren();
  byId('status').textContent = '正在读取 D435 彩色帧…';
  byId('center').textContent = '';
  byId('spread').textContent = '';
  const stamps = new Set();
  const centers = [];
  const deadline = performance.now() + 10000;
  try {
    while (stamps.size < targetFrames && performance.now() < deadline) {
      const response = await fetch('/api/d435-cad', {cache: 'no-store', signal: AbortSignal.timeout(4000)});
      const data = await response.json();
      if (!response.ok) throw Error(data.message || '快照读取失败');
      if (!('board' in data)) throw Error('当前网页后端尚未加载整板识别版本。');
      if (!stamps.has(data.stamp)) {
        stamps.add(data.stamp);
        frame.src = data.image;
        frame.hidden = false;
        byId('status').textContent = '已读取 ' + stamps.size + '/' + targetFrames + ' 帧 · 当前检出 ' +
          data.detected + ' 个标签，' + data.tags.length + ' 个通过检查。';
        if (data.board) centers.push(data.board.base_mm);
        byId('center').textContent = data.board
          ? '整板中心 · base_link [X / Y / Z] ' + data.board.base_mm.join(' / ') +
            ' mm（' + data.board.pairs + ' 对对称标签）'
          : '当前帧未定位整板中心。请让更多成对的标签进入画面。';
        let spread = '整板定位 ' + centers.length + '/' + stamps.size + ' 帧';
        if (centers.length >= 2) {
          const ranges = [0, 1, 2].map(axis => {
            const values = centers.map(point => point[axis]);
            return (Math.max(...values) - Math.min(...values)).toFixed(1);
          });
          spread += ' · 峰峰值 [X / Y / Z] ' + ranges.join(' / ') + ' mm';
        }
        byId('spread').textContent = spread;
        byId('rows').replaceChildren(...data.tags.map(tag => {
          const row = document.createElement('p');
          row.textContent = 'ID ' + tag.id + ' · base_link [mm] ' + tag.base_mm.join(' / ') +
            ' · body4 [mm] ' + (tag.body4_mm?.join(' / ') || '升降反馈未配对') + ' · RMS ' + tag.rms_px + ' px';
          return row;
        }));
      }
      if (stamps.size < targetFrames) await new Promise(resolve => setTimeout(resolve, 300));
    }
    if (stamps.size < targetFrames) throw Error('读取超时，仅收到 ' + stamps.size + ' 张不同快照。');
  } catch (err) {
    byId('status').textContent = err.message;
  } finally {
    button.disabled = false;
  }
};
button.click();
