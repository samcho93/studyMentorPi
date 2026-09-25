/* site.js — 공통 UI: 모바일 내비, 코드 복사, 목차 하이라이트 */
(function () {
  'use strict';

  // 오래된 학습 진행률 데이터 정리 (완료 표시·진행률 기능은 제거됨)
  try { localStorage.removeItem('studymentorpi.progress.v1'); } catch (e) { /* ignore */ }

  // ------------------------------------------------------------ 모바일 내비
  var toggle = document.getElementById('navToggle');
  var sidebar = document.getElementById('sidebar');
  var scrim = document.getElementById('scrim');

  function closeNav() {
    if (sidebar) sidebar.classList.remove('open');
    if (scrim) scrim.classList.remove('open');
  }

  if (toggle && sidebar) {
    toggle.addEventListener('click', function () {
      var open = sidebar.classList.toggle('open');
      if (scrim) scrim.classList.toggle('open', open);
    });
  }
  if (scrim) scrim.addEventListener('click', closeNav);
  document.addEventListener('keydown', function (e) {
    if (e.key === 'Escape') closeNav();
  });

  // ------------------------------------------------------------- 코드 복사
  document.querySelectorAll('.copy-btn').forEach(function (btn) {
    btn.addEventListener('click', function () {
      var block = btn.closest('.code-block');
      var code = block && block.querySelector('code');
      if (!code) return;
      var text = code.textContent;

      var done = function () {
        btn.textContent = '복사됨';
        btn.classList.add('copied');
        setTimeout(function () {
          btn.textContent = '복사';
          btn.classList.remove('copied');
        }, 1400);
      };

      if (navigator.clipboard && navigator.clipboard.writeText) {
        navigator.clipboard.writeText(text).then(done, fallback);
      } else {
        fallback();
      }

      function fallback() {
        var ta = document.createElement('textarea');
        ta.value = text;
        ta.style.position = 'fixed';
        ta.style.opacity = '0';
        document.body.appendChild(ta);
        ta.select();
        try { document.execCommand('copy'); done(); } catch (e) { /* noop */ }
        document.body.removeChild(ta);
      }
    });
  });

  // -------------------------------------------------------- 목차 하이라이트
  var tocLinks = Array.prototype.slice.call(
    document.querySelectorAll('.lesson-toc a'));

  if (tocLinks.length && 'IntersectionObserver' in window) {
    var map = {};
    tocLinks.forEach(function (a) {
      var id = decodeURIComponent(a.getAttribute('href').slice(1));
      map[id] = a;
    });

    var visible = new Set();
    var observer = new IntersectionObserver(function (entries) {
      entries.forEach(function (entry) {
        if (entry.isIntersecting) visible.add(entry.target.id);
        else visible.delete(entry.target.id);
      });

      // 문서 순서상 가장 위에 있는 보이는 헤딩을 현재로 표시
      var current = null;
      Object.keys(map).forEach(function (id) {
        if (current === null && visible.has(id)) current = id;
      });
      tocLinks.forEach(function (a) { a.classList.remove('current'); });
      if (current && map[current]) map[current].classList.add('current');
    }, { rootMargin: '-8% 0px -70% 0px', threshold: 0 });

    Object.keys(map).forEach(function (id) {
      var el = document.getElementById(id);
      if (el) observer.observe(el);
    });
  }

  // ---------------------------------------------------- 키보드 단축키 (← →)
  document.addEventListener('keydown', function (e) {
    if (e.target.matches('input, textarea, select')) return;
    if (e.metaKey || e.ctrlKey || e.altKey) return;
    if (e.key === 'ArrowLeft') {
      var prev = document.querySelector('a.pager-prev');
      if (prev) location.href = prev.getAttribute('href');
    } else if (e.key === 'ArrowRight') {
      var next = document.querySelector('a.pager-next');
      if (next) location.href = next.getAttribute('href');
    }
  });

  // 활성 사이드바 항목을 화면 안으로
  var active = document.querySelector('.sidebar-nav a.active');
  if (active && sidebar) {
    var top = active.offsetTop - sidebar.clientHeight / 2;
    if (top > 0) sidebar.scrollTop = top;
  }
})();

/* ---- studyMentorPi: Playground open button — pass code via URL hash (base64url) ---- */
(function () {
  'use strict';
  function b64url(str) {
    var bytes = new TextEncoder().encode(str);
    var bin = '';
    for (var i = 0; i < bytes.length; i++) bin += String.fromCharCode(bytes[i]);
    return btoa(bin).replace(/\+/g, '-').replace(/\//g, '_').replace(/=+$/, '');
  }
  document.querySelectorAll('[data-playground]').forEach(function (btn) {
    btn.addEventListener('click', function () {
      var code = btn.closest('.code-block').querySelector('code').textContent;
      var base = btn.getAttribute('data-playground').replace(/playground$/, '');
      location.href = base + 'tools/playground.html#code=' + b64url(code);
    });
  });
})();

/* ---- studyMentorPi: lightweight Python syntax colouring (ML Basic look) ---- */
(function () {
  'use strict';
  var KW = /^(False|None|True|and|as|assert|async|await|break|class|continue|def|del|elif|else|except|finally|for|from|global|if|import|in|is|lambda|nonlocal|not|or|pass|raise|return|try|while|with|yield)$/;
  var BI = /^(print|range|len|round|min|max|sum|abs|sorted|enumerate|zip|list|dict|set|tuple|int|float|str|next|isinstance|open|super|any|all)$/;
  var TOKEN = /("""[\s\S]*?"""|'''[\s\S]*?'''|"(?:\.|[^"\\n])*"|'(?:\.|[^'\\n])*'|#[^\n]*|\b\d+(?:\.\d+)?(?:e[-+]?\d+)?\b|\b[A-Za-z_]\w*\b)/g;
  function esc(s) { return s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;'); }
  function paint(code) {
    var src = code.textContent, out = '', last = 0, m;
    TOKEN.lastIndex = 0;
    while ((m = TOKEN.exec(src))) {
      var t = m[0], cls = null;
      if (t[0] === '#') cls = 'tk-c';
      else if (t[0] === '"' || t[0] === "'") cls = 'tk-s';
      else if (/^\d/.test(t)) cls = 'tk-n';
      else if (KW.test(t)) cls = 'tk-k';
      else if (BI.test(t) || src[TOKEN.lastIndex] === '(') cls = 'tk-f';
      out += esc(src.slice(last, m.index)) + (cls ? '<span class="' + cls + '">' + esc(t) + '</span>' : esc(t));
      last = TOKEN.lastIndex;
    }
    code.innerHTML = out + esc(src.slice(last));
  }
  document.querySelectorAll('.code-block code.lang-python').forEach(paint);
})();
