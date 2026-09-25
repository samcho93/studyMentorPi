// playground.js — ROS 2 Playground: CodeMirror editor + Pyodide worker + 2D replay of the virtual MentorPi.
import { createEditor } from './playground-editor.js';
import { EXAMPLES, DEFAULT_CODE } from './playground-examples.js';
import { View, themeColors, drawWorld, drawRobot, drawScan, drawPath, drawMovers, odomToWorld } from '../assets/js/world2d.js';

const $ = (id) => document.getElementById(id);
const LS_CODE = 'studymentorpi.playground.code.v1';
const LIB_BASE = new URL('../python/', location.href).href;
const lsGet = (k) => { try { return localStorage.getItem(k); } catch (e) { return null; } };
const lsSet = (k, v) => { try { localStorage.setItem(k, v); } catch (e) { /* private mode */ } };

// ------------------------------------------------------------------ output
const outEl = $('out');
function out(text, cls = '') {
  const near = outEl.scrollHeight - outEl.scrollTop - outEl.clientHeight < 40;
  const line = document.createElement('div');
  if (cls) line.className = 'ln-' + cls;
  line.textContent = text;
  outEl.appendChild(line);
  while (outEl.childElementCount > 4000) outEl.firstElementChild.remove();
  if (near) outEl.scrollTop = outEl.scrollHeight;
}
function setStatus(text, cls = '') { const s = $('pyStatus'); s.textContent = text; s.className = 'pg-status' + (cls ? ' ' + cls : ''); }

// ------------------------------------------------------------------ code sources
const b64url = (str) => { const b = new TextEncoder().encode(str); let s = ''; for (const c of b) s += String.fromCharCode(c); return btoa(s).replace(/\+/g, '-').replace(/\//g, '_').replace(/=+$/, ''); };
const unb64url = (s) => { let b = s.replace(/-/g, '+').replace(/_/g, '/'); while (b.length % 4) b += '='; return new TextDecoder().decode(Uint8Array.from(atob(b), (c) => c.charCodeAt(0))); };
let fromLesson = false;
function initialCode() {
  const m = location.hash.match(/(?:^#|&)code=([^&]+)/);
  if (m) {
    try {
      const code = unb64url(decodeURIComponent(m[1]));
      history.replaceState(null, '', location.pathname + location.search);
      fromLesson = true;
      return code;
    } catch (e) { out('링크의 코드를 해석하지 못했습니다: ' + e.message, 'err'); }
  }
  const ex = new URLSearchParams(location.search).get('example');
  const found = EXAMPLES.find((e) => e.id === ex);
  if (found) return found.code;
  const saved = lsGet(LS_CODE);
  return saved && saved.trim() ? saved : DEFAULT_CODE;
}

// ------------------------------------------------------------------ tabs
function showPane(id) {
  document.querySelectorAll('.pg-tabs button').forEach((b) => b.setAttribute('aria-selected', String(b.dataset.pane === id)));
  document.querySelectorAll('.pg-pane').forEach((p) => p.classList.toggle('on', p.id === id));
  if (id === 'paneWorld') { view.resize(); render(); }
  if (id === 'paneChart') drawChart();
  if (id === 'pane3d') ensure3D();
}
// 3D replay of the last run in the URDF simulator (sim3d?embed=replay)
let f3dReady = false, pending3d = null;
function ensure3D() { const f = $('f3d'); if (!f.src) f.src = f.dataset.src; }
function send3D(res) {
  pending3d = res; ensure3D();
  if (f3dReady) { $('f3d').contentWindow.postMessage({ type: 'mp-replay', result: res }, '*'); pending3d = null; }
}
window.addEventListener('message', (ev) => {
  if (ev.source === $('f3d').contentWindow && ev.data && ev.data.type === 'mp3d-ready') { f3dReady = true; if (pending3d) send3D(pending3d); }
});
document.querySelectorAll('.pg-tabs button').forEach((b) => b.addEventListener('click', () => showPane(b.dataset.pane)));

// ------------------------------------------------------------------ 2D replay
const view = new View($('world'));
let C = themeColors();
let result = null;          // last run export
let frameIdx = 0, playing = false, lastTs = 0, playT = 0;
const defaultWorld = { bounds: [-3, -2, 3, 2], walls: [], boxes: [], cylinders: [] };

function render() {
  const w = result ? result.world : defaultWorld;
  view.fit(w.bounds);
  drawWorld(view, w, C);
  if (!result || !result.frames.length) {
    const ctx = view.ctx;
    ctx.fillStyle = C.faint; ctx.font = `${14 * view.dpr}px sans-serif`; ctx.textAlign = 'center';
    ctx.fillText('코드를 실행하면 가상 MentorPi의 움직임이 여기서 재생됩니다', view.canvas.width / 2, view.canvas.height / 2);
    ctx.textAlign = 'start';
    return;
  }
  const fr = result.frames;
  const i = Math.min(frameIdx, fr.length - 1);
  const f = fr[i];
  const upto = fr.slice(0, i + 1);
  drawPath(view, upto.map((q) => q.gt), C.accent, 2.2);
  if ($('chkOdom').checked) drawPath(view, upto.map((q) => odomToWorld(result.start, q.od)), C.warn, 1.6, [6, 4]);
  drawMovers(view, f.movers, C);
  if (f.scan && f.scan.length) drawScan(view, f.gt, f.scan, C, { rays: $('chkRays').checked });
  drawRobot(view, f.gt, result.chassis, result.geom, C, { steer: f.steer });
  // HUD
  const ctx = view.ctx, d = view.dpr;
  ctx.font = `${12 * d}px "JetBrains Mono", monospace`;
  ctx.fillStyle = C.text;
  const cmd = f.cmd.map((v) => v.toFixed(2)).join(', ');
  const txt = `t=${f.t.toFixed(1)}s  cmd(vx,vy,wz)=(${cmd})  pose=(${f.gt[0].toFixed(2)}, ${f.gt[1].toFixed(2)}, ${(f.gt[2] * 180 / Math.PI).toFixed(0)}°)`;
  ctx.fillText(txt, 10 * d, view.canvas.height - 10 * d);
}

function setFrame(i) {
  if (!result) return;
  frameIdx = Math.max(0, Math.min(i, result.frames.length - 1));
  $('seek').value = frameIdx;
  const f = result.frames[frameIdx];
  $('playT').textContent = `${f.t.toFixed(1)} / ${result.frames[result.frames.length - 1].t.toFixed(1)} s`;
  render();
}
function tick(ts) {
  if (!playing || !result) return;
  const dt = lastTs ? (ts - lastTs) / 1000 : 0;
  lastTs = ts;
  playT += dt * Number($('speed').value);
  const fr = result.frames;
  let i = frameIdx;
  while (i < fr.length - 1 && fr[i + 1].t - fr[0].t <= playT) i++;
  setFrame(i);
  if (i >= fr.length - 1) { playing = false; $('btnPlay').innerHTML = '&#8634;'; return; }
  requestAnimationFrame(tick);
}
function play(fromStart = false) {
  if (!result || !result.frames.length) return;
  if (fromStart || frameIdx >= result.frames.length - 1) { frameIdx = 0; }
  playT = result.frames[frameIdx].t - result.frames[0].t;
  playing = true; lastTs = 0;
  $('btnPlay').innerHTML = '&#10074;&#10074;';
  requestAnimationFrame(tick);
}
$('btnPlay').addEventListener('click', () => {
  if (playing) { playing = false; $('btnPlay').innerHTML = '&#9654;'; } else play();
});
$('seek').addEventListener('input', (e) => { playing = false; $('btnPlay').innerHTML = '&#9654;'; setFrame(Number(e.target.value)); });
$('chkRays').addEventListener('change', render);
$('chkOdom').addEventListener('change', render);
new ResizeObserver(() => { view.resize(); render(); drawChart(); }).observe($('paneWorld'));
new MutationObserver(() => { C = themeColors(); render(); drawChart(); }).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });

// ------------------------------------------------------------------ images + chart
function showImages(images) {
  const box = $('imgs');
  $('imgCount').textContent = images.length ? String(images.length) : '';
  if (!images.length) {
    box.innerHTML = '<div class="pg-empty">코드에서 <code>cv2.imshow("이름", img)</code>를 부르면 여기에 표시됩니다.</div>';
    return;
  }
  box.innerHTML = '';
  for (const im of images) {
    const fig = document.createElement('figure');
    const img = document.createElement('img');
    img.src = 'data:image/png;base64,' + im.png;
    img.alt = im.name;
    const cap = document.createElement('figcaption');
    cap.textContent = `${im.name}  ${im.shape.join('×')}`;
    fig.append(img, cap);
    box.appendChild(fig);
  }
}
const PALETTE = ['#4f46e5', '#e0823d', '#1f9d55', '#d64541', '#7b4fd1', '#0891b2'];
function drawChart() {
  const cv = $('chart');
  if (!cv.offsetParent) return;
  const r = cv.getBoundingClientRect(), d = Math.min(window.devicePixelRatio || 1, 2);
  cv.width = Math.max(50, r.width * d); cv.height = Math.max(50, r.height * d);
  const ctx = cv.getContext('2d');
  ctx.fillStyle = C.bg; ctx.fillRect(0, 0, cv.width, cv.height);
  const plots = (result && result.plots) || [];
  ctx.font = `${12 * d}px sans-serif`; ctx.fillStyle = C.faint;
  if (!plots.length) { ctx.textAlign = 'center'; ctx.fillText('mentorpi_sim.plot(xs, ys, "이름")으로 그래프를 그릴 수 있습니다', cv.width / 2, cv.height / 2); ctx.textAlign = 'start'; return; }
  let xmin = Infinity, xmax = -Infinity, ymin = Infinity, ymax = -Infinity;
  for (const p of plots) p.x.forEach((x, i) => { const y = p.y[i]; if (x == null || y == null) return; xmin = Math.min(xmin, x); xmax = Math.max(xmax, x); ymin = Math.min(ymin, y); ymax = Math.max(ymax, y); });
  if (xmax === xmin) xmax = xmin + 1;
  if (ymax === ymin) { ymax += 1; ymin -= 1; }
  const pad = (ymax - ymin) * 0.08; ymin -= pad; ymax += pad;
  const L = 56 * d, R = 16 * d, T = 16 * d, B = 34 * d;
  const X = (x) => L + (x - xmin) / (xmax - xmin) * (cv.width - L - R);
  const Y = (y) => cv.height - B - (y - ymin) / (ymax - ymin) * (cv.height - T - B);
  ctx.strokeStyle = C.grid; ctx.lineWidth = 1;
  for (let k = 0; k <= 4; k++) {
    const y = ymin + (ymax - ymin) * k / 4, x = xmin + (xmax - xmin) * k / 4;
    ctx.beginPath(); ctx.moveTo(L, Y(y)); ctx.lineTo(cv.width - R, Y(y)); ctx.stroke();
    ctx.fillText(y.toPrecision(3), 6 * d, Y(y) + 4 * d);
    ctx.fillText(x.toPrecision(3), X(x) - 10 * d, cv.height - 12 * d);
  }
  plots.forEach((p, k) => {
    ctx.strokeStyle = PALETTE[k % PALETTE.length]; ctx.lineWidth = 2 * d;
    ctx.beginPath();
    let pen = false;
    p.x.forEach((x, i) => { const y = p.y[i]; if (x == null || y == null) { pen = false; return; } pen ? ctx.lineTo(X(x), Y(y)) : ctx.moveTo(X(x), Y(y)); pen = true; });
    ctx.stroke();
    ctx.fillStyle = PALETTE[k % PALETTE.length];
    ctx.fillText('— ' + p.label, L + 8 * d, T + 14 * d * (k + 1));
  });
}

function summarize(res) {
  const s = res.summary;
  if (!s.used_sim) { $('summary').innerHTML = '<span class="hint">가상 로봇을 쓰지 않은 코드입니다 (출력·이미지·그래프 탭을 확인하세요).</span>'; return; }
  const deg = (v) => v.toFixed(1) + '°';
  $('summary').innerHTML =
    `시뮬 <code>${s.t.toFixed(1)} s</code> · 섀시 <code>${res.chassis}</code> · 월드 <code>${res.world.title}</code> · 이동 거리 <code>${s.distance.toFixed(2)} m</code> · ` +
    `충돌 <code style="color:${s.collisions ? 'var(--danger)' : 'inherit'}">${s.collisions}</code><br>` +
    `실제 자세 <code>(${s.gt[0]}, ${s.gt[1]}, ${deg(s.gt[2])})</code> · /odom <code>(${s.odom[0]}, ${s.odom[1]}, ${deg(s.odom[2])})</code> <span class="hint">(odom 원점 = 시작 자세)</span>`;
}

// ------------------------------------------------------------------ worker
let worker = null, ready = false, running = false, runId = 0;
let slowTimer = null, loadT0 = 0;
function startWorker() {
  ready = false;
  loadT0 = performance.now();
  $('btnRun').disabled = false;
  out('파이썬(Pyodide)을 불러오는 중입니다 — 처음에는 10~30초 걸립니다. 실행을 먼저 눌러 두면 준비되는 대로 실행합니다.', 'info');
  clearTimeout(slowTimer);
  slowTimer = setTimeout(() => {
    if (!ready) out('아직 준비 중입니다. 인터넷(cdn.jsdelivr.net) 접속이 느리거나 방화벽에서 막혀 있을 수 있습니다.', 'err');
  }, 40000);
  worker = new Worker('playground-worker.js');
  worker.onmessage = (ev) => {
    const m = ev.data;
    if (m.type === 'status') setStatus(m.text);
    else if (m.type === 'ready') {
      ready = true;
      $('pyInfo').textContent = `Pyodide ${m.pyodide} · Python ${m.python} · mentorpi_sim ${m.lib}`;
      setStatus('준비됨 — Ctrl+Enter로 실행', 'ok');
      clearTimeout(slowTimer);
      $('btnRun').disabled = false;
      $('btnRun').innerHTML = '&#9654; 실행';
      out('✔ 파이썬 준비 완료 (' + ((performance.now() - loadT0) / 1000).toFixed(1) + ' s)', 'ok');
      if (pendingRun) { pendingRun = false; run(); }
    } else if (m.type === 'fatal') {
      clearTimeout(slowTimer);
      pendingRun = false;
      $('btnRun').innerHTML = '&#9654; 실행';
      setStatus('Pyodide 로드 실패', 'bad');
      out('Pyodide를 불러오지 못했습니다: ' + m.error + '\n(인터넷 연결이 필요합니다 — jsdelivr CDN)', 'err');
    } else if (m.type === 'stdout') out(m.text);
    else if (m.type === 'stderr') out(m.text, 'err');
    else if (m.type === 'done' && m.runId === runId) finish(m);
  };
  worker.onerror = (e) => { out('worker 오류: ' + (e.message || e), 'err'); };
  worker.postMessage({ type: 'init', base: LIB_BASE });
}
let t0 = 0;
let pendingRun = false;
function run() {
  if (running) return;
  if (!ready) {
    if (!pendingRun) {
      pendingRun = true;
      $('btnRun').innerHTML = '&#8987; 준비되면 실행';
      out('아직 불러오는 중입니다 — 준비가 끝나면 자동으로 실행합니다.', 'info');
    }
    return;
  }
  const src = code();
  lsSet(LS_CODE, src);
  running = true;
  runId++;
  t0 = performance.now();
  playing = false;
  $('btnRun').disabled = true;
  $('btnStop').disabled = false;
  setStatus('실행 중…');
  out('▶ 실행 (' + new Date().toLocaleTimeString() + ')', 'info');
  worker.postMessage({ type: 'run', code: src, runId });
}
function finish(m) {
  running = false;
  $('btnRun').disabled = false;
  $('btnStop').disabled = true;
  const sec = ((performance.now() - t0) / 1000).toFixed(2);
  setStatus(m.status === 'ok' ? '완료 (' + sec + ' s)' : '오류', m.status === 'ok' ? 'ok' : 'bad');
  const res = m.result;
  if (!res) return;
  result = res;
  showImages(res.images || []);
  $('plotCount').textContent = res.plots && res.plots.length ? String(res.plots.length) : '';
  summarize(res);
  $('seek').max = Math.max(0, res.frames.length - 1);
  if (res.summary.used_sim && res.frames.length > 1) {
    out(`■ ${m.status === 'ok' ? '완료' : '오류 전까지'} — 시뮬레이션 ${res.summary.t.toFixed(1)} s 기록 → 2D 재생`, m.status === 'ok' ? 'ok' : 'info');
    showPane('paneWorld');
    setFrame(0);
    play(true);
    send3D(res);
  } else {
    if (m.status === 'ok') out('■ 완료', 'ok');
    render();
    if (res.images && res.images.length) showPane('paneImgs');
    else if (res.plots && res.plots.length) showPane('paneChart');
  }
}
function stop() {
  if (!running) return;
  worker.terminate();
  running = false;
  out('■ 정지 — Python을 다시 시작합니다 (몇 초 걸립니다)', 'err');
  $('btnStop').disabled = true;
  startWorker();
}

// ------------------------------------------------------------------ idle state (after loading an example)
const SUMMARY0 = $('summary').innerHTML;
function resetView() {
  playing = false;
  result = null;
  frameIdx = 0;
  $('btnPlay').innerHTML = '&#9654;';
  $('seek').max = 0; $('seek').value = 0;
  $('playT').textContent = '— / — s';
  showImages([]);
  $('plotCount').textContent = '';
  $('summary').innerHTML = SUMMARY0;
  pending3d = null;
  const f = $('f3d');
  if (f3dReady && f.contentWindow) f.contentWindow.postMessage({ type: 'mp-reset' }, '*');
  showPane('paneWorld');
  view.resize(); render(); drawChart();
  setStatus(ready ? '준비됨 — Ctrl+Enter로 실행' : '준비 중…', ready ? 'ok' : '');
}

// ------------------------------------------------------------------ UI
let editor = null;
const doc0 = initialCode();
const code = () => (editor ? editor.getValue() : doc0);
$('btnRun').addEventListener('click', run);
$('btnStop').addEventListener('click', stop);
$('btnClear').addEventListener('click', () => { outEl.textContent = ''; });
$('btnShare').addEventListener('click', async () => {
  const url = location.origin + location.pathname + '#code=' + b64url(code());
  try { await navigator.clipboard.writeText(url); out('링크를 복사했습니다.', 'info'); } catch (e) { prompt('링크를 복사하세요', url); }
});
$('btnDownload').addEventListener('click', () => {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([code()], { type: 'text/x-python' }));
  a.download = 'main.py';
  a.click();
});
document.addEventListener('keydown', (e) => {
  if ((e.ctrlKey || e.metaKey) && e.key === 'Enter') { e.preventDefault(); run(); }
});
view.resize(); render();
startWorker();
if (fromLesson) run();

editor = await createEditor($('editor'), { doc: doc0, onRun: run, onChange: (v) => lsSet(LS_CODE, v) });
if (editor.loadError) out('코드 편집기(CodeMirror)를 불러오지 못해 기본 입력창을 씁니다.', 'info');

const sel = $('examples');
const groups = [...new Set(EXAMPLES.map((e) => e.group || '예제'))];
sel.innerHTML += groups.map((g) => `<optgroup label="${g}">` + EXAMPLES.filter((e) => (e.group || '예제') === g)
  .map((e) => `<option value="${e.id}">${e.title}</option>`).join('') + '</optgroup>').join('');
// the dropdown shows the example that is loaded in the editor
const norm = (t) => t.replace(/\r\n/g, '\n').trim();
const exFor = (code) => EXAMPLES.find((e) => norm(e.code) === norm(code));
sel.value = (exFor(editor.getValue()) || { id: '' }).id;
sel.addEventListener('change', () => {
  const ex = EXAMPLES.find((e) => e.id === sel.value);
  if (!ex) return;
  if (running) stop();
  editor.setValue(ex.code);
  resetView();
  out(`예제 불러옴: ${ex.title}`, 'info');
  sel.value = ex.id;
});
