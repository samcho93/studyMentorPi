// lesson-runner.js — run ```python run blocks inside the lesson page.
// A docked panel (right side on desktop, bottom sheet on phones) runs the code in the same
// Pyodide worker as the Playground (tools/playground-worker.js) and shows output, the 2D
// replay of the virtual MentorPi, cv2.imshow images and mentorpi_sim.plot charts.
import { View, themeColors, drawWorld, drawRobot, drawScan, drawPath, drawMovers, odomToWorld } from './world2d.js';

const LIB_BASE = new URL('../python/', location.href).href;
const WORKER_URL = new URL('../tools/playground-worker.js' + (new URL(import.meta.url).search || ''), location.href).href;
const esc = (s) => s.replace(/[&<>]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c]));

// ------------------------------------------------------------------ panel DOM
const panel = document.createElement('aside');
panel.className = 'runner';
panel.setAttribute('aria-label', '코드 실행 결과');
panel.innerHTML = `
<div class="rn-head">
  <strong data-r="title">실행 결과</strong><span class="rn-status" data-r="status">준비 전</span>
  <button class="rn-x" type="button" data-r="close" aria-label="닫기">&times;</button>
</div>
<div class="rn-tabs" role="tablist">
  <button type="button" data-pane="out" aria-selected="true">출력</button>
  <button type="button" data-pane="w3d" aria-selected="false">3D 시뮬레이터</button>
  <button type="button" data-pane="world" aria-selected="false">2D 보기</button>
  <button type="button" data-pane="imgs" aria-selected="false">이미지<span data-r="nimg"></span></button>
  <button type="button" data-pane="chart" aria-selected="false">그래프<span data-r="nplot"></span></button>
  <button type="button" data-pane="code" aria-selected="false">코드 수정</button>
  <button type="button" data-pane="live" aria-selected="false" hidden data-r="liveTab">시뮬레이터</button>
</div>
<div class="rn-body">
  <div class="rn-pane on" data-p="out"><pre class="rn-out" data-r="out"></pre></div>
  <div class="rn-pane" data-p="world">
    <canvas class="rn-cv" data-r="world"></canvas>
    <div class="rn-play">
      <button type="button" class="rn-btn" data-r="play">&#9654;</button>
      <span class="rn-t" data-r="t">— s</span>
      <input type="range" data-r="seek" min="0" max="0" value="0" aria-label="재생 위치">
      <select data-r="speed" aria-label="재생 속도"><option value="1">1×</option><option value="2" selected>2×</option><option value="4">4×</option></select>
    </div>
    <div class="rn-sum" data-r="sum"></div>
  </div>
  <div class="rn-pane" data-p="w3d"><div class="rn-frame" data-r="w3d"><p class="rn-empty" style="padding:14px">가상 로봇을 쓰는 코드를 실행하면 MentorPi URDF 모델로 3D 재생합니다.</p></div></div>
  <div class="rn-pane" data-p="live"><div class="rn-frame" data-r="live"></div></div>
  <div class="rn-pane" data-p="imgs"><div class="rn-imgs" data-r="imgs"></div></div>
  <div class="rn-pane" data-p="chart"><canvas class="rn-cv" data-r="chart"></canvas></div>
  <div class="rn-pane" data-p="code"><textarea class="rn-code" data-r="code" spellcheck="false" aria-label="실행할 코드"></textarea></div>
</div>
<div class="rn-foot">
  <button type="button" class="rn-btn primary" data-r="run">&#9654; 다시 실행</button>
  <button type="button" class="rn-btn" data-r="stop" disabled>&#9632; 정지</button>
  <a class="rn-link" data-r="pg" href="../tools/playground.html" target="_blank" rel="noopener">Playground에서 열기 ↗</a>
</div>`;
document.body.appendChild(panel);
const R = (k) => panel.querySelector(`[data-r="${k}"]`);

// ------------------------------------------------------------------ resizable panel
// drag the left edge (desktop) or the top edge (phones); double-click resets; size is remembered
const SIZE_KEY = 'studymentorpi.runner.size.v1';
const handle = document.createElement('div');
handle.className = 'rn-resize';
handle.title = '드래그해서 크기 조절 (더블클릭: 기본 크기)';
handle.setAttribute('role', 'separator');
panel.appendChild(handle);
const rootStyle = document.documentElement.style;
const isSheet = () => window.matchMedia('(max-width: 1000px)').matches;
function applySize(sz) {
  if (sz && sz.w) rootStyle.setProperty('--rn-w', sz.w + 'px'); else rootStyle.removeProperty('--rn-w');
  if (sz && sz.h) rootStyle.setProperty('--rn-h', sz.h + 'px'); else rootStyle.removeProperty('--rn-h');
  window.dispatchEvent(new Event('resize'));
}
let size = {};
try { size = JSON.parse(localStorage.getItem(SIZE_KEY) || '{}') || {}; } catch (e) { size = {}; }
applySize(size);
handle.addEventListener('pointerdown', (e) => {
  e.preventDefault();
  handle.setPointerCapture(e.pointerId);
  panel.classList.add('resizing'); document.body.classList.add('rn-dragging');
  const move = (ev) => {
    if (isSheet()) size.h = Math.round(Math.max(160, Math.min(window.innerHeight - 60, window.innerHeight - ev.clientY)));
    else size.w = Math.round(Math.max(320, Math.min(window.innerWidth - 300, window.innerWidth - ev.clientX)));
    applySize(size);
  };
  const up = () => {
    handle.removeEventListener('pointermove', move); handle.removeEventListener('pointerup', up); handle.removeEventListener('pointercancel', up);
    panel.classList.remove('resizing'); document.body.classList.remove('rn-dragging');
    try { localStorage.setItem(SIZE_KEY, JSON.stringify(size)); } catch (err) { /* ignore */ }
  };
  handle.addEventListener('pointermove', move); handle.addEventListener('pointerup', up); handle.addEventListener('pointercancel', up);
});
handle.addEventListener('dblclick', () => {
  size = {}; applySize(size);
  try { localStorage.removeItem(SIZE_KEY); } catch (err) { /* ignore */ }
});

function showPane(name) {
  panel.querySelectorAll('.rn-tabs button').forEach((b) => b.setAttribute('aria-selected', String(b.dataset.pane === name)));
  panel.querySelectorAll('.rn-pane').forEach((p) => p.classList.toggle('on', p.dataset.p === name));
  if (name === 'world') { view.resize(); render(); }
  if (name === 'chart') drawChart();
  if (name === 'w3d') ensure3D();
  panel.classList.toggle('wide', name === 'live' || name === 'w3d');
}
panel.querySelectorAll('.rn-tabs button').forEach((b) => b.addEventListener('click', () => showPane(b.dataset.pane)));
function open() { panel.classList.add('open'); document.body.classList.add('runner-open'); }
function close() { panel.classList.remove('open'); document.body.classList.remove('runner-open'); document.querySelectorAll('.code-block.running').forEach((c) => c.classList.remove('running')); }
R('close').addEventListener('click', close);
document.addEventListener('keydown', (e) => { if (e.key === 'Escape' && panel.classList.contains('open')) close(); });

const out = (text, cls = '') => {
  const s = document.createElement('span');
  if (cls) s.className = 'rn-' + cls;
  s.textContent = text.endsWith('\n') ? text : text + '\n';
  R('out').appendChild(s);
  R('out').scrollTop = R('out').scrollHeight;
};
const status = (t, cls = '') => { R('status').textContent = t; R('status').className = 'rn-status' + (cls ? ' ' + cls : ''); };

// ------------------------------------------------------------------ worker
let worker = null, ready = false, running = false, runId = 0, pending = null, t0 = 0;
function startWorker() {
  ready = false;
  worker = new Worker(WORKER_URL);
  worker.onmessage = (ev) => {
    const m = ev.data;
    if (m.type === 'status') status(m.text);
    else if (m.type === 'ready') {
      ready = true; status('준비됨', 'ok');
      out(`✔ Python ${m.python} (Pyodide ${m.pyodide}) 준비 완료`, 'ok');
      if (pending) { const c = pending; pending = null; run(c); }
    } else if (m.type === 'fatal') { status('Pyodide 로드 실패', 'bad'); out('Pyodide를 불러오지 못했습니다: ' + m.error + ' (인터넷 연결 필요)', 'err'); }
    else if (m.type === 'stdout') out(m.text);
    else if (m.type === 'stderr') out(m.text, 'err');
    else if (m.type === 'done' && m.runId === runId) finish(m);
  };
  worker.onerror = (e) => out('worker 오류: ' + (e.message || e), 'err');
  worker.postMessage({ type: 'init', base: LIB_BASE });
  out('파이썬(Pyodide)을 불러오는 중… 처음 한 번은 10~30초 걸립니다.', 'info');
}
function run(code) {
  open();
  R('code').value = code;
  R('pg').href = '../tools/playground.html#code=' + b64url(code);
  if (!worker) startWorker();
  if (!ready) { pending = code; status('불러오는 중…'); return; }
  if (running) return;
  running = true; runId++; t0 = performance.now(); playing = false;
  R('run').disabled = true; R('stop').disabled = false;
  status('실행 중…');
  out('▶ 실행 ' + new Date().toLocaleTimeString(), 'info');
  showPane('out');
  worker.postMessage({ type: 'run', code, runId });
}
function finish(m) {
  running = false; R('run').disabled = false; R('stop').disabled = true;
  const sec = ((performance.now() - t0) / 1000).toFixed(1);
  status(m.status === 'ok' ? `완료 (${sec} s)` : '오류', m.status === 'ok' ? 'ok' : 'bad');
  const res = m.result;
  if (!res) return;
  result = res;
  showImages(res.images || []);
  R('nplot').textContent = res.plots && res.plots.length ? ' ' + res.plots.length : '';
  R('seek').max = Math.max(0, res.frames.length - 1);
  if (res.summary.used_sim && res.frames.length > 1) {
    const s = res.summary;
    R('sum').innerHTML = `시뮬 <b>${s.t.toFixed(1)} s</b> · ${res.chassis} · ${esc(res.world.title)} · 이동 <b>${s.distance.toFixed(2)} m</b> · 충돌 <b>${s.collisions}</b>`;
    out(`■ 시뮬레이션 ${s.t.toFixed(1)} s 기록 → 2D 재생 탭`, 'ok');
    showPane(panel.dataset.pref3d === '0' ? 'world' : 'w3d');   // 3D first, 2D only if chosen frame = 0; play(true);
    send3D(res);
  } else {
    if (m.status === 'ok') out('■ 완료', 'ok');
    if (res.images && res.images.length) showPane('imgs');
    else if (res.plots && res.plots.length) showPane('chart');
  }
}
R('run').addEventListener('click', () => run(R('code').value));
R('stop').addEventListener('click', () => {
  if (!running) return;
  worker.terminate(); worker = null; running = false;
  R('stop').disabled = true; R('run').disabled = false;
  out('■ 정지 — 다음 실행 때 Python을 다시 불러옵니다', 'err'); status('정지됨');
});
R('code').addEventListener('input', () => { R('pg').href = '../tools/playground.html#code=' + b64url(R('code').value); });
R('code').addEventListener('keydown', (e) => {
  if ((e.ctrlKey || e.metaKey) && e.key === 'Enter') { e.preventDefault(); run(R('code').value); }
  if (e.key === 'Tab') { e.preventDefault(); const t = e.target, s = t.selectionStart; t.setRangeText('    ', s, t.selectionEnd, 'end'); }
});
function b64url(str) { const b = new TextEncoder().encode(str); let s = ''; for (const c of b) s += String.fromCharCode(c); return btoa(s).replace(/\+/g, '-').replace(/\//g, '_').replace(/=+$/, ''); }

// ------------------------------------------------------------------ 2D replay
const view = new View(R('world'));
let C = themeColors(), result = null, frame = 0, playing = false, lastTs = 0, playT = 0;
function render() {
  if (!result || !result.frames.length) return;
  view.fit(result.world.bounds);
  drawWorld(view, result.world, C);
  const fr = result.frames, i = Math.min(frame, fr.length - 1), f = fr[i], upto = fr.slice(0, i + 1);
  drawPath(view, upto.map((q) => q.gt), C.accent, 2);
  drawPath(view, upto.map((q) => odomToWorld(result.start, q.od)), C.warn, 1.4, [6, 4]);
  drawMovers(view, f.movers, C);
  if (f.scan && f.scan.length) drawScan(view, f.gt, f.scan, C, { rays: true });
  drawRobot(view, f.gt, result.chassis, result.geom, C, { steer: f.steer });
  R('t').textContent = `${f.t.toFixed(1)} / ${fr[fr.length - 1].t.toFixed(1)} s`;
  R('seek').value = i;
}
function tick(ts) {
  if (!playing || !result) return;
  const dt = lastTs ? (ts - lastTs) / 1000 : 0; lastTs = ts;
  playT += dt * Number(R('speed').value);
  const fr = result.frames;
  while (frame < fr.length - 1 && fr[frame + 1].t - fr[0].t <= playT) frame++;
  render();
  if (frame >= fr.length - 1) { playing = false; R('play').innerHTML = '&#8634;'; return; }
  requestAnimationFrame(tick);
}
function play(fromStart) {
  if (!result || !result.frames.length) return;
  if (fromStart || frame >= result.frames.length - 1) frame = 0;
  playT = result.frames[frame].t - result.frames[0].t; playing = true; lastTs = 0;
  R('play').innerHTML = '&#10074;&#10074;';
  requestAnimationFrame(tick);
}
R('play').addEventListener('click', () => { if (playing) { playing = false; R('play').innerHTML = '&#9654;'; } else play(); });
R('seek').addEventListener('input', (e) => { playing = false; R('play').innerHTML = '&#9654;'; frame = Number(e.target.value); render(); });
new ResizeObserver(() => { view.resize(); render(); drawChart(); }).observe(panel);
new MutationObserver(() => { C = themeColors(); render(); drawChart(); }).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });

function showImages(images) {
  R('nimg').textContent = images.length ? ' ' + images.length : '';
  const box = R('imgs');
  box.innerHTML = images.length ? '' : '<p class="rn-empty">cv2.imshow("이름", img)로 띄운 이미지가 여기에 나옵니다.</p>';
  for (const im of images) {
    const fig = document.createElement('figure');
    fig.innerHTML = `<img alt="${esc(im.name)}" src="data:image/png;base64,${im.png}"><figcaption>${esc(im.name)} · ${im.shape.join('×')}</figcaption>`;
    box.appendChild(fig);
  }
}
const PAL = ['#4f46e5', '#e0823d', '#1f9d55', '#d64541', '#7b4fd1', '#0891b2'];
function drawChart() {
  const cv = R('chart');
  if (!cv.offsetParent) return;
  const r = cv.getBoundingClientRect(), d = Math.min(window.devicePixelRatio || 1, 2);
  cv.width = Math.max(50, r.width * d); cv.height = Math.max(50, r.height * d);
  const ctx = cv.getContext('2d');
  ctx.fillStyle = C.bg; ctx.fillRect(0, 0, cv.width, cv.height);
  const plots = (result && result.plots) || [];
  ctx.font = `${12 * d}px sans-serif`; ctx.fillStyle = C.faint;
  if (!plots.length) { ctx.fillText('mentorpi_sim.plot(xs, ys, "이름")으로 그래프를 그립니다', 12 * d, 24 * d); return; }
  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
  for (const p of plots) p.x.forEach((x, i) => { const y = p.y[i]; if (x == null || y == null) return; x0 = Math.min(x0, x); x1 = Math.max(x1, x); y0 = Math.min(y0, y); y1 = Math.max(y1, y); });
  if (x1 === x0) x1 = x0 + 1; if (y1 === y0) { y1 += 1; y0 -= 1; }
  const L = 52 * d, Rm = 12 * d, T = 12 * d, B = 28 * d;
  const X = (x) => L + (x - x0) / (x1 - x0) * (cv.width - L - Rm), Y = (y) => cv.height - B - (y - y0) / (y1 - y0) * (cv.height - T - B);
  ctx.strokeStyle = C.grid;
  for (let k = 0; k <= 4; k++) {
    const y = y0 + (y1 - y0) * k / 4, x = x0 + (x1 - x0) * k / 4;
    ctx.beginPath(); ctx.moveTo(L, Y(y)); ctx.lineTo(cv.width - Rm, Y(y)); ctx.stroke();
    ctx.fillText(y.toPrecision(3), 4 * d, Y(y) + 4 * d); ctx.fillText(x.toPrecision(3), X(x) - 10 * d, cv.height - 8 * d);
  }
  plots.forEach((p, k) => {
    ctx.strokeStyle = ctx.fillStyle = PAL[k % PAL.length]; ctx.lineWidth = 2 * d; ctx.beginPath();
    let pen = false;
    p.x.forEach((x, i) => { const y = p.y[i]; if (x == null || y == null) { pen = false; return; } pen ? ctx.lineTo(X(x), Y(y)) : ctx.moveTo(X(x), Y(y)); pen = true; });
    ctx.stroke(); ctx.fillText('— ' + p.label, L + 6 * d, T + 14 * d * (k + 1));
  });
}

// ------------------------------------------------------------------ 3D replay (sim3d?embed=replay)
let f3d = null, f3dReady = false, pending3d = null;
function ensure3D() {
  if (f3d) return;
  f3d = document.createElement('iframe');
  f3d.title = 'MentorPi 3D 재생';
  f3d.src = new URL('../sim3d/index.html?embed=replay', location.href).href;
  R('w3d').innerHTML = '';
  R('w3d').appendChild(f3d);
}
function send3D(res) {
  pending3d = res;
  if (!f3d) ensure3D();
  if (f3dReady) { f3d.contentWindow.postMessage({ type: 'mp-replay', result: res }, '*'); pending3d = null; }
}
window.addEventListener('message', (ev) => {
  if (f3d && ev.source === f3d.contentWindow && ev.data && ev.data.type === 'mp3d-ready') {
    f3dReady = true;
    if (pending3d) send3D(pending3d);
  }
});
panel.querySelector('[data-pane="w3d"]').addEventListener('click', () => { panel.dataset.pref3d = '1'; });
panel.querySelector('[data-pane="world"]').addEventListener('click', () => { panel.dataset.pref3d = '0'; });

// ------------------------------------------------------------------ live simulators inside the panel
function openLive(href, label) {
  const u = new URL(href, location.href);
  u.searchParams.set('embed', '1');
  const box = R('live');
  if (!box.firstChild || box.firstChild.dataset.src !== u.href) {
    box.innerHTML = '';
    const fr = document.createElement('iframe');
    fr.title = label; fr.src = u.href; fr.dataset.src = u.href;
    fr.allow = 'camera';
    box.appendChild(fr);
  }
  R('liveTab').hidden = false;
  R('liveTab').textContent = label;
  R('title').textContent = label;
  open();
  showPane('live');
}
document.addEventListener('click', (e) => {
  const a = e.target.closest('a[href]');
  if (!a || e.ctrlKey || e.metaKey || e.shiftKey || e.button !== 0) return;
  if (!a.closest('.lesson')) return;                     // only links inside the lesson article
  const href = a.getAttribute('href');
  let label = null;
  if (/sim3d\/(index\.html)?(\?|#|$)/.test(href)) label = '3D 시뮬레이터';
  else if (/(^|\/)sim\/(index\.html)?(\?|#|$)/.test(href)) label = '2D 시뮬레이터';
  else if (/tools\/(kinematics-lab|color-lab|urdf-viewer)\.html/.test(href)) label = a.textContent.trim().replace(/\s+/g, ' ').slice(0, 20) || '도구';
  if (!label) return;
  e.preventDefault();
  openLive(href, label);
});

// ------------------------------------------------------------------ wire code blocks
document.querySelectorAll('.code-block.code-run').forEach((blk) => {
  const btn = blk.querySelector('.run-inline');
  if (!btn) return;
  btn.addEventListener('click', () => {
    document.querySelectorAll('.code-block.running').forEach((c) => c.classList.remove('running'));
    blk.classList.add('running');
    R('out').textContent = '';
    R('title').textContent = '실행 결과';
    run(blk.querySelector('code').textContent);
  });
});
