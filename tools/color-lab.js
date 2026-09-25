// color-lab.js — HSV/LAB thresholding with OpenCV 8-bit conventions (cv2.cvtColor + cv2.inRange).
const $ = (id) => document.getElementById(id);
const src = $('src'), dst = $('dst');
const sctx = src.getContext('2d', { willReadFrequently: true }), dctx = dst.getContext('2d');
let space = 'hsv', rgba = null, W = 0, H = 0, conv = null, video = null, stream = null;

const DEF = {
  hsv: { names: ['H', 'S', 'V'], max: [179, 255, 255] },
  lab: { names: ['L', 'a', 'b'], max: [255, 255, 255] },
};
const PRESETS = {
  hsv: {
    '빨강(아래)': [0, 120, 70, 10, 255, 255], '빨강(위)': [170, 120, 70, 179, 255, 255], '초록': [40, 80, 60, 85, 255, 255],
    '파랑': [100, 120, 60, 130, 255, 255], '노랑': [20, 120, 120, 35, 255, 255], '검정(선)': [0, 0, 0, 179, 255, 60], '흰색': [0, 0, 200, 179, 40, 255],
  },
  lab: {
    // typical lab_config.yaml style ranges (OpenCV 8-bit LAB)
    '빨강': [0, 150, 130, 255, 255, 255], '초록': [0, 0, 0, 255, 115, 255], '파랑': [0, 0, 0, 255, 255, 110],
    '검정': [0, 0, 0, 60, 255, 255], '노랑': [100, 100, 160, 255, 150, 255],
  },
};
let range = PRESETS.hsv['빨강(아래)'].slice();

// ------------------------------------------------------------------ colour conversion (OpenCV formulas)
function rgb2hsv(r, g, b) {
  const mx = Math.max(r, g, b), mn = Math.min(r, g, b), d = mx - mn;
  let h = 0;
  if (d > 0) {
    if (mx === r) h = 60 * (g - b) / d; else if (mx === g) h = 120 + 60 * (b - r) / d; else h = 240 + 60 * (r - g) / d;
    if (h < 0) h += 360;
  }
  const s = mx === 0 ? 0 : d / mx;
  return [Math.round(h / 2) % 180, Math.round(s * 255), mx];
}
const f_lab = (t) => (t > 0.008856 ? Math.cbrt(t) : 7.787 * t + 16 / 116);
const lin = (c) => { c /= 255; return c <= 0.04045 ? c / 12.92 : ((c + 0.055) / 1.055) ** 2.4; };
function rgb2lab(r, g, b) {
  const R = lin(r), G = lin(g), B = lin(b);
  const X = (0.412453 * R + 0.357580 * G + 0.180423 * B) / 0.950456;
  const Y = 0.212671 * R + 0.715160 * G + 0.072169 * B;
  const Z = (0.019334 * R + 0.119193 * G + 0.950227 * B) / 1.088754;
  const L = Y > 0.008856 ? 116 * Math.cbrt(Y) - 16 : 903.3 * Y;
  const a = 500 * (f_lab(X) - f_lab(Y)), bb = 200 * (f_lab(Y) - f_lab(Z));
  return [Math.round(L * 255 / 100), Math.round(a + 128), Math.round(bb + 128)];
}
function convert() {
  if (!rgba) return;
  conv = new Uint8Array(W * H * 3);
  const fn = space === 'hsv' ? rgb2hsv : rgb2lab;
  for (let i = 0, j = 0; i < rgba.length; i += 4, j += 3) {
    const c = fn(rgba[i], rgba[i + 1], rgba[i + 2]);
    conv[j] = c[0]; conv[j + 1] = c[1]; conv[j + 2] = c[2];
  }
}

// ------------------------------------------------------------------ image input
function setImage(img) {
  const maxW = 640;
  const s = Math.min(1, maxW / (img.videoWidth || img.naturalWidth || img.width));
  W = Math.round((img.videoWidth || img.naturalWidth || img.width) * s);
  H = Math.round((img.videoHeight || img.naturalHeight || img.height) * s);
  src.width = dst.width = W; src.height = dst.height = H;
  sctx.drawImage(img, 0, 0, W, H);
  rgba = sctx.getImageData(0, 0, W, H).data;
  convert(); apply();
}
function loadSample(name) {
  stopCam();
  const img = new Image();
  img.onload = () => setImage(img);
  img.src = '../python/samples/' + name;
  $('srcCap').textContent = `원본 ${name} — 클릭하면 그 점의 색을 기준으로 범위를 잡습니다`;
}
$('selImg').addEventListener('change', () => loadSample($('selImg').value));
$('file').addEventListener('change', (e) => {
  const f = e.target.files[0]; if (!f) return;
  stopCam();
  const img = new Image();
  img.onload = () => { setImage(img); URL.revokeObjectURL(img.src); };
  img.src = URL.createObjectURL(f);
  $('srcCap').textContent = '원본 (' + f.name + ')';
});
function stopCam() { if (stream) { stream.getTracks().forEach((t) => t.stop()); stream = null; } video = null; $('btnCam').textContent = '웹캠'; }
$('btnCam').addEventListener('click', async () => {
  if (stream) { stopCam(); return; }
  try {
    stream = await navigator.mediaDevices.getUserMedia({ video: { width: 640, height: 480 } });
    video = document.createElement('video'); video.srcObject = stream; video.muted = true; video.playsInline = true;
    await video.play();
    $('btnCam').textContent = '웹캠 끄기';
    const tick = () => { if (!video) return; setImage(video); requestAnimationFrame(tick); };
    tick();
  } catch (e) { alert('웹캠을 열 수 없습니다: ' + e.message); }
});

// ------------------------------------------------------------------ threshold + morphology
function erodeDilate(mask, k, erode) {
  if (k <= 1) return mask;
  const r = (k - 1) >> 1, out = new Uint8Array(mask.length), tmp = new Uint8Array(mask.length);
  for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {               // horizontal pass
    let v = erode ? 1 : 0;
    for (let d = -r; d <= r; d++) { const xx = Math.min(W - 1, Math.max(0, x + d)), m = mask[y * W + xx]; if (erode ? !m : m) { v = erode ? 0 : 1; break; } }
    tmp[y * W + x] = v;
  }
  for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {               // vertical pass
    let v = erode ? 1 : 0;
    for (let d = -r; d <= r; d++) { const yy = Math.min(H - 1, Math.max(0, y + d)), m = tmp[yy * W + x]; if (erode ? !m : m) { v = erode ? 0 : 1; break; } }
    out[y * W + x] = v;
  }
  return out;
}
function apply() {
  if (!conv) return;
  let mask = new Uint8Array(W * H);
  const [a0, b0, c0, a1, b1, c1] = range;
  for (let i = 0, j = 0; i < mask.length; i++, j += 3) {
    mask[i] = conv[j] >= a0 && conv[j] <= a1 && conv[j + 1] >= b0 && conv[j + 1] <= b1 && conv[j + 2] >= c0 && conv[j + 2] <= c1 ? 1 : 0;
  }
  const k = Number($('kern').value);
  mask = erodeDilate(erodeDilate(mask, k, true), k, false);           // MORPH_OPEN
  // largest blob bbox + centroid (4-connected labelling)
  const lab = new Int32Array(W * H); let best = null, n = 0, count = 0;
  for (let i = 0; i < mask.length; i++) {
    if (!mask[i] || lab[i]) continue;
    n++; let area = 0, sx = 0, sy = 0, x0 = W, y0 = H, x1 = 0, y1 = 0;
    const st = [i]; lab[i] = n;
    while (st.length) {
      const p = st.pop(), x = p % W, y = (p - x) / W;
      area++; sx += x; sy += y; if (x < x0) x0 = x; if (x > x1) x1 = x; if (y < y0) y0 = y; if (y > y1) y1 = y;
      for (const q of [p - 1, p + 1, p - W, p + W]) {
        if (q < 0 || q >= mask.length || lab[q] || !mask[q]) continue;
        if ((q === p - 1 && x === 0) || (q === p + 1 && x === W - 1)) continue;
        lab[q] = n; st.push(q);
      }
    }
    if (area > 30) count++;
    if (!best || area > best.area) best = { area, cx: sx / area, cy: sy / area, x0, y0, x1, y1 };
  }
  // draw
  const out = dctx.createImageData(W, H), d = out.data, mode = $('view').value;
  let on = 0;
  for (let i = 0; i < mask.length; i++) {
    const o = i * 4, m = mask[i]; on += m;
    if (mode === 'mask') { const v = m ? 255 : 0; d[o] = d[o + 1] = d[o + 2] = v; }
    else if (mode === 'cut') { d[o] = m ? rgba[o] : 0; d[o + 1] = m ? rgba[o + 1] : 0; d[o + 2] = m ? rgba[o + 2] : 0; }
    else { const g = (rgba[o] + rgba[o + 1] + rgba[o + 2]) / 3 * 0.45; d[o] = m ? rgba[o] : g; d[o + 1] = m ? rgba[o + 1] : g; d[o + 2] = m ? rgba[o + 2] : g; }
    d[o + 3] = 255;
  }
  dctx.putImageData(out, 0, 0);
  if (best && best.area > 30) {
    dctx.strokeStyle = '#22c55e'; dctx.lineWidth = 2; dctx.strokeRect(best.x0, best.y0, best.x1 - best.x0 + 1, best.y1 - best.y0 + 1);
    dctx.fillStyle = '#ef4444'; dctx.beginPath(); dctx.arc(best.cx, best.cy, 5, 0, 7); dctx.fill();
  }
  const pct = (100 * on / mask.length).toFixed(1);
  $('stats').innerHTML = `<div><span>마스크 픽셀</span><b>${on} (${pct}%)</b></div><div><span>덩어리(>30px)</span><b>${count}</b></div>` +
    (best && best.area > 30 ? `<div><span>최대 덩어리 중심</span><b>(${best.cx.toFixed(0)}, ${best.cy.toFixed(0)})</b></div><div><span>면적 · 외접 사각형</span><b>${best.area} · ${best.x1 - best.x0 + 1}×${best.y1 - best.y0 + 1}</b></div>` : '');
  $('dstCap').textContent = `결과 — ${W}×${H}, 초록 = 가장 큰 덩어리, 빨강 점 = 중심 (moments)`;
  const cvt = space === 'hsv' ? 'COLOR_BGR2HSV' : 'COLOR_BGR2LAB';
  $('code').textContent =
    `img = cv2.imread("${$('selImg').value}")\n` +
    `conv = cv2.cvtColor(img, cv2.${cvt})\n` +
    `lower = np.array([${range[0]}, ${range[1]}, ${range[2]}])\nupper = np.array([${range[3]}, ${range[4]}, ${range[5]}])\n` +
    `mask = cv2.inRange(conv, lower, upper)\n` +
    `mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((${k}, ${k}), np.uint8))`;
}

// ------------------------------------------------------------------ controls
function buildRanges() {
  const def = DEF[space];
  $('ranges').innerHTML = def.names.map((n, i) =>
    `<div class="rng"><b>${n}</b><input type="range" data-i="${i}" min="0" max="${def.max[i]}" value="${range[i]}" aria-label="${n} 최소"><input type="range" data-i="${i + 3}" min="0" max="${def.max[i]}" value="${range[i + 3]}" aria-label="${n} 최대">` +
    `<span></span><output id="lo${i}">${range[i]}</output><output id="hi${i}">${range[i + 3]}</output></div>`).join('');
  $('ranges').querySelectorAll('input').forEach((el) => el.addEventListener('input', () => {
    range[Number(el.dataset.i)] = Number(el.value); syncOut(); apply();
  }));
  $('presets').innerHTML = Object.keys(PRESETS[space]).map((k) => `<button type="button">${k}</button>`).join('');
}
function syncOut() {
  for (let i = 0; i < 3; i++) { $('lo' + i).textContent = range[i]; $('hi' + i).textContent = range[i + 3]; }
  $('ranges').querySelectorAll('input').forEach((el) => { el.value = range[Number(el.dataset.i)]; });
}
$('presets').addEventListener('click', (e) => {
  const b = e.target.closest('button'); if (!b) return;
  range = PRESETS[space][b.textContent].slice(); syncOut(); apply();
});
$('spaceRow').addEventListener('click', (e) => {
  const b = e.target.closest('button'); if (!b) return;
  space = b.dataset.s;
  $('spaceRow').querySelectorAll('button').forEach((x) => x.classList.toggle('on', x === b));
  range = Object.values(PRESETS[space])[0].slice();
  buildRanges(); convert(); apply();
});
$('kern').addEventListener('input', () => { $('oK').textContent = $('kern').value; apply(); });
$('view').addEventListener('change', apply);
src.addEventListener('click', (e) => {
  if (!conv) return;
  const r = src.getBoundingClientRect();
  // canvas uses object-fit: contain — map click into image pixels
  const s = Math.min(r.width / W, r.height / H), ox = (r.width - W * s) / 2, oy = (r.height - H * s) / 2;
  const x = Math.floor((e.clientX - r.left - ox) / s), y = Math.floor((e.clientY - r.top - oy) / s);
  if (x < 0 || y < 0 || x >= W || y >= H) return;
  const j = (y * W + x) * 3, c = [conv[j], conv[j + 1], conv[j + 2]];
  const tol = space === 'hsv' ? [10, 70, 70] : [60, 20, 20];
  const mx = DEF[space].max;
  range = [0, 1, 2].map((i) => Math.max(0, c[i] - tol[i])).concat([0, 1, 2].map((i) => Math.min(mx[i], c[i] + tol[i])));
  syncOut(); apply();
  const o = (y * W + x) * 4;
  $('srcCap').innerHTML = `<span class="swatch" style="background:rgb(${rgba[o]},${rgba[o + 1]},${rgba[o + 2]})"></span>(${x}, ${y}) ${DEF[space].names.join('')} = (${c.join(', ')}) → ±(${tol.join(', ')}) 범위 설정`;
});

buildRanges();
loadSample('colors.png');
