#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
studyMentorPi static site builder
  content/curriculum.json + content/{base,ros,mobile,ai}/*.md  ->  index.html, lessons/*.html

Standard library only (the tiny markdown renderer below is self-contained).
Usage:
  python build.py            # full build
  python build.py --serve    # build, then preview at http://localhost:8000
  python build.py --check    # build and fail (exit 1) on missing chapters / sections
"""
from __future__ import annotations

import html
import json
import re
import sys
from pathlib import Path


def _utf8_stdout() -> None:
    """Windows 콘솔(cp949)에서도 한글이 깨지지 않게 합니다."""
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            try:
                stream.reconfigure(encoding="utf-8", errors="replace")
            except (ValueError, OSError):
                pass


_utf8_stdout()

ROOT = Path(__file__).resolve().parent
CONTENT = ROOT / "content"
LESSONS = ROOT / "lessons"
FIGURES = CONTENT / "figures"

TOOL_LINKS = {
    "sim": ("sim/index.html", "2D 시뮬레이터"),
    "sim3d": ("sim3d/index.html", "3D 시뮬레이터"),
    "playground": ("tools/playground.html", "ROS 2 Playground"),
    "lab": ("tools/kinematics-lab.html", "섀시 기구학 실험실"),
    "color": ("tools/color-lab.html", "HSV 색 실험실"),
    "urdf": ("tools/urdf-viewer.html", "URDF 뷰어"),
}

REQUIRED_SECTIONS = ["학습 목표", "자주 나는 오류와 해결", "참고자료"]


# ---------------------------------------------------------------- markdown

def _b64url(text: str) -> str:
    import base64
    return base64.urlsafe_b64encode(text.encode("utf-8")).decode("ascii").rstrip("=")


class MarkdownRenderer:
    """강의에 필요한 범위만 지원하는 소형 마크다운 렌더러."""

    CALLOUT_LABEL = {
        "tip": "TIP",
        "info": "참고",
        "warn": "주의",
        "warning": "주의",
        "danger": "위험",
        "safety": "안전 확인 (실물 실습 전 필수)",
        "design": "설계 포인트",
        "check": "체크포인트",
        "task": "실습",
        "mission": "팀 미션",
    }

    def __init__(self, depth: int = 1):
        self.rel = "../" * depth
        self.toc: list = []

    # ---- inline --------------------------------------------------------
    def inline(self, text: str) -> str:
        out = []
        for part in re.split(r"(`[^`]+`)", text):
            if len(part) > 1 and part.startswith("`") and part.endswith("`"):
                out.append("<code>" + html.escape(part[1:-1]) + "</code>")
                continue
            s = html.escape(part, quote=False)
            s = re.sub(r"!\[([^\]]*)\]\(([^)]+)\)", self._img, s)
            s = re.sub(r"\[([^\]]+)\]\(([^)\s]+)\)", self._link, s)
            s = re.sub(r"\*\*([^*]+)\*\*", r"<strong>\1</strong>", s)
            s = re.sub(r"(?<![\w*])\*([^*\n]+)\*(?![\w*])", r"<em>\1</em>", s)
            s = re.sub(r"~~([^~]+)~~", r"<del>\1</del>", s)
            s = re.sub(r"\$([^$\n]+)\$", r'<span class="math">\1</span>', s)
            out.append(s)
        return "".join(out)

    def _img(self, m):
        return '<img src="%s" alt="%s" loading="lazy">' % (self._href(m.group(2)), m.group(1))

    def _link(self, m):
        url = m.group(2)
        ext = ' target="_blank" rel="noopener"' if "://" in url else ""
        return '<a href="%s"%s>%s</a>' % (self._href(url), ext, m.group(1))

    def _href(self, url: str) -> str:
        if url.startswith("~/"):
            return self.rel + url[2:]
        return url

    def _sub(self):
        sub = MarkdownRenderer(depth=0)
        sub.rel = self.rel
        return sub

    # ---- blocks --------------------------------------------------------
    def render(self, md: str) -> str:
        lines = md.replace("\r\n", "\n").split("\n")
        out = []
        i, n = 0, len(lines)

        while i < n:
            line = lines[i]
            stripped = line.strip()

            if not stripped:
                i += 1
                continue

            if stripped.startswith("```"):
                info = stripped[3:].strip() or "text"
                i += 1
                buf = []
                while i < n and not lines[i].strip().startswith("```"):
                    buf.append(lines[i])
                    i += 1
                i += 1
                out.append(self._code_block(info, "\n".join(buf)))
                continue

            if stripped.startswith("@fig["):
                m = re.match(r"@fig\[([\w-]+)\]\s*(.*)", stripped)
                if m:
                    out.append(self._figure(m.group(1), m.group(2).strip()))
                    i += 1
                    continue

            if stripped.startswith("@btn["):
                m = re.match(r"@btn\[([^\]]+)\]\s*(.*)", stripped)
                if m:
                    out.append('<p class="cta-row"><a class="ghost-btn cta" href="%s">%s</a></p>'
                               % (html.escape(self._href(m.group(1))),
                                  html.escape(m.group(2).strip() or "열기")))
                    i += 1
                    continue

            if stripped.startswith(":::"):
                m = re.match(r":::\s*([\w-]+)\s*(.*)", stripped)
                kind = (m.group(1) if m else "info").lower()
                title = (m.group(2).strip() if m else "")
                i += 1
                buf = []
                depth = 0
                while i < n:
                    s = lines[i].strip()
                    if s == ":::" and depth == 0:
                        break
                    if re.match(r":::\s*[\w-]+", s):
                        depth += 1
                    elif s == ":::":
                        depth -= 1
                    buf.append(lines[i])
                    i += 1
                i += 1
                label = title or self.CALLOUT_LABEL.get(kind, "참고")
                css = "warn" if kind == "warning" else kind
                out.append(
                    '<div class="callout callout-%s"><div class="callout-title">%s</div>'
                    '<div class="callout-body">%s</div></div>'
                    % (css, html.escape(label), self._sub().render("\n".join(buf))))
                continue

            m = re.match(r"^(#{1,4})\s+(.*)$", stripped)
            if m:
                level = len(m.group(1))
                text = m.group(2).strip()
                slug = self._slug(text)
                if level in (2, 3):
                    self.toc.append((level, slug, text))
                out.append('<h%d id="%s">%s<a class="anchor" href="#%s">#</a></h%d>'
                           % (level, slug, self.inline(text), slug, level))
                i += 1
                continue

            if re.match(r"^(---|\*\*\*)$", stripped):
                out.append("<hr>")
                i += 1
                continue

            if ("|" in stripped and i + 1 < n
                    and re.match(r"^\s*\|?[\s:|-]+\|[\s:|-]*$", lines[i + 1])):
                head = self._row(stripped)
                i += 2
                body = []
                while i < n and "|" in lines[i] and lines[i].strip():
                    body.append(self._row(lines[i]))
                    i += 1
                thead = "".join("<th>%s</th>" % self.inline(c) for c in head)
                rows = "".join("<tr>%s</tr>" % "".join("<td>%s</td>" % self.inline(c) for c in r)
                               for r in body)
                out.append('<div class="table-wrap"><table><thead><tr>%s</tr></thead>'
                           "<tbody>%s</tbody></table></div>" % (thead, rows))
                continue

            if stripped.startswith(">"):
                buf = []
                while i < n and lines[i].strip().startswith(">"):
                    buf.append(re.sub(r"^\s*>\s?", "", lines[i]))
                    i += 1
                out.append("<blockquote>%s</blockquote>" % self._sub().render("\n".join(buf)))
                continue

            if re.match(r"^\s*([-*+]|\d+\.)\s+", line):
                block, i = self._collect_list(lines, i)
                out.append(block)
                continue

            if stripped.startswith("<"):
                buf = []
                while i < n and lines[i].strip():
                    buf.append(lines[i])
                    i += 1
                out.append("\n".join(buf))
                continue

            buf = []
            while (i < n and lines[i].strip() and not re.match(
                    r"^\s*(#{1,4}\s|```|:::|@fig\[|@btn\[|>|[-*+]\s|\d+\.\s|---$)", lines[i])):
                buf.append(lines[i].strip())
                i += 1
            out.append("<p>%s</p>" % self.inline(" ".join(buf)))

        return "\n".join(out)

    # ---- helpers -------------------------------------------------------
    def _row(self, line: str) -> list:
        cells = re.split(r"(?<!\\)\|", line.strip().strip("|"))
        return [c.strip().replace("\\|", "|") for c in cells]

    def _figure(self, name: str, caption: str) -> str:
        path = FIGURES / (name + ".svg")
        if not path.exists():
            return ('<div class="callout callout-warn"><div class="callout-title">그림 없음</div>'
                    '<div class="callout-body"><p>content/figures/%s.svg</p></div></div>'
                    % html.escape(name))
        svg = path.read_text(encoding="utf-8").strip()
        cap = ('<figcaption>%s</figcaption>' % self.inline(caption)) if caption else ""
        return '<figure class="fig" id="fig-%s">%s%s</figure>' % (html.escape(name), svg, cap)

    def _code_block(self, info: str, code: str) -> str:
        parts = info.split()
        lang = parts[0]
        flags = set(parts[1:])
        badges, buttons, cls = "", "", ""
        if "robot" in flags:
            badges = '<span class="code-badge badge-robot">실물 전용</span>'
            cls = " code-robot"
        if "run" in flags:
            badges = '<span class="code-badge badge-run">브라우저 실행</span>'
            buttons = ('<button class="run-btn run-inline" type="button" title="오른쪽 패널에서 바로 실행">&#9654; 실행</button>'
                       '<a class="pg-btn" href="%stools/playground.html#code=%s" target="_blank" rel="noopener" '
                       'title="Playground에서 열기 (새 탭)">Playground ↗</a>'
                       % (self.rel, _b64url(code)))
            cls = " code-run"
        return ('<div class="code-block%s">'
                '<div class="code-head"><span class="code-lang">%s</span>%s'
                '<span class="code-actions">%s<button class="copy-btn" type="button">복사</button></span></div>'
                '<pre><code class="lang-%s">%s</code></pre></div>'
                % (cls, html.escape(lang), badges, buttons, html.escape(lang), html.escape(code)))

    def _collect_list(self, lines: list, i: int):
        n = len(lines)
        items = []
        while i < n:
            m = re.match(r"^(\s*)([-*+]|\d+\.)\s+(.*)$", lines[i])
            if not m:
                if lines[i].strip() and items and lines[i].startswith("   "):
                    indent, text, ordered = items[-1]
                    items[-1] = (indent, text + " " + lines[i].strip(), ordered)
                    i += 1
                    continue
                break
            items.append((len(m.group(1)), m.group(3), m.group(2)[0].isdigit()))
            i += 1

        def item_html(text: str) -> str:
            m = re.match(r"^\[( |x|X)\]\s+(.*)$", text)
            if m:
                checked = " checked" if m.group(1).lower() == "x" else ""
                return ('<label class="check-item"><input type="checkbox"%s> %s</label>'
                        % (checked, self.inline(m.group(2))))
            return self.inline(text)

        def build(pos, level):
            tag = "ol" if items[pos][2] else "ul"
            buf = ["<%s>" % tag]
            while pos < len(items) and items[pos][0] >= level:
                indent, text, _ = items[pos]
                if indent > level:
                    sub, pos = build(pos, indent)
                    buf.append(sub)
                    continue
                buf.append("<li>" + item_html(text))
                if pos + 1 < len(items) and items[pos + 1][0] > level:
                    sub, pos = build(pos + 1, items[pos + 1][0])
                    buf.append(sub)
                else:
                    pos += 1
                buf.append("</li>")
            buf.append("</%s>" % tag)
            return "".join(buf), pos

        block = build(0, items[0][0])[0] if items else ""
        return block, i

    @staticmethod
    def _slug(text: str) -> str:
        s = re.sub(r"[`*_\[\]()#]", "", text).strip().lower()
        s = re.sub(r"[^0-9a-z가-힣]+", "-", s).strip("-")
        return s or "section"


# ---------------------------------------------------------------- front matter

def split_front_matter(text: str):
    text = text.replace("\r\n", "\n")
    if not text.startswith("---\n"):
        return {}, text
    end = text.find("\n---", 4)
    if end == -1:
        return {}, text
    meta: dict = {}
    for line in text[4:end].split("\n"):
        line = re.sub(r"\s+#.*$", "", line)
        if ":" in line:
            k, v = line.split(":", 1)
            v = v.strip().strip('"')
            if v.startswith("[") and v.endswith("]"):
                meta[k.strip()] = [x.strip().strip('"') for x in v[1:-1].split(",") if x.strip()]
            else:
                meta[k.strip()] = v
    return meta, text[end + 4:].lstrip("\n")


# ---------------------------------------------------------------- templates

def sidebar_html(cur: dict, current, rel: str) -> str:
    out = ['<nav class="sidebar-nav">']
    out.append('<div class="brand-row">'
               '<a class="brand" href="%sindex.html"><span class="brand-mark">&#9673;</span>'
               '<span class="brand-text">studyMentorPi<small>이동로봇 강의</small></span></a>'
               '<button class="theme-icon" type="button" data-theme-toggle data-theme-icon '
               'aria-label="테마 전환"></button>'
               "</div>" % rel)
    out.append('<div class="side-tools">'
               '<a class="side-tool" href="%ssim3d/index.html">3D 시뮬레이터</a>'
               '<a class="side-tool" href="%ssim/index.html">2D 시뮬레이터</a>'
               '<a class="side-tool" href="%stools/playground.html">ROS 2 Playground</a>'
               '<a class="side-tool" href="%stools/kinematics-lab.html">기구학 실험실</a>'
               '<a class="side-tool" href="%stools/color-lab.html">HSV 색 실험실</a>'
               '<a class="side-tool" href="%stools/urdf-viewer.html">URDF 뷰어</a>'
               "</div>" % (rel, rel, rel, rel, rel, rel))
    for track in cur["tracks"]:
        out.append('<div class="nav-track" data-track="%s">' % track["id"])
        out.append('<div class="nav-track-title track-%s">%s</div>'
                   % (track["id"], html.escape(track["title"])))
        for part in track["parts"]:
            out.append('<div class="nav-part"><span class="nav-part-no">%s</span>%s</div>'
                       % (html.escape(part["no"]), html.escape(part["title"])))
            out.append("<ul>")
            for cid in part["chapters"]:
                ch = cur["index"][cid]
                active = ' class="active"' if cid == current else ""
                out.append('<li><a href="%slessons/%s.html"%s data-slug="%s">'
                           '<span class="nav-no">%s</span>%s</a></li>'
                           % (rel, cid, active, cid, cid.upper(), html.escape(ch["title"])))
            out.append("</ul>")
        out.append("</div>")
    out.append("</nav>")
    return "\n".join(out)


PAGE = """<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{title}</title>
<meta name="description" content="{desc}">
<link rel="icon" href="data:image/svg+xml,<svg xmlns=%22http://www.w3.org/2000/svg%22 viewBox=%220 0 100 100%22><rect x=%2215%22 y=%2225%22 width=%2270%22 height=%2250%22 rx=%2212%22 fill=%22%2322c55e%22/><circle cx=%2250%22 cy=%2250%22 r=%2212%22 fill=%22%23052e16%22/></svg>">
<link rel="stylesheet" href="{rel}assets/css/main.css?v={ver}">
<link rel="stylesheet" href="{rel}assets/css/mentorpi.css?v={ver}">
<link rel="stylesheet" href="{rel}assets/css/ml-theme.css?v={ver}">
<link rel="stylesheet" crossorigin="anonymous" href="https://cdn.jsdelivr.net/gh/orioncactus/pretendard@v1.3.9/dist/web/variable/pretendardvariable.min.css">
<link rel="stylesheet" crossorigin="anonymous" href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;600;700&display=swap">
<script src="{rel}assets/js/theme.js?v={ver}"></script>
</head>
<body class="{bodyclass}">
<button class="nav-toggle" id="navToggle" aria-label="목차 열기">&#9776;</button>
<aside class="sidebar" id="sidebar">
{sidebar}
</aside>
<div class="sidebar-scrim" id="scrim"></div>
<main class="main">
{content}
</main>
<script src="{rel}assets/js/site.js?v={ver}"></script>
{extra}
</body>
</html>
"""


_VER = None


def asset_version() -> str:
    """Short hash of CSS/JS so browsers refetch them after every change (cache busting)."""
    global _VER
    if _VER is None:
        import hashlib
        h = hashlib.sha256()
        for d in ("assets", "tools", "sim", "sim3d"):
            for p in sorted((ROOT / d).rglob("*")):
                if p.suffix in (".css", ".js"):
                    h.update(p.read_bytes().replace(b"\r\n", b"\n"))
        _VER = h.hexdigest()[:8]
    return _VER


def stamp_tool_pages() -> None:
    """Add ?v=<asset hash> to local .js/.css references in the hand-written tool pages
    (sim/, sim3d/, tools/) so browsers and GitHub Pages never serve stale scripts."""
    pat = re.compile(r'((?:src|href)=")((?:\.\./|\./)?[\w./-]+\.(?:js|css))(?:\?v=[0-9a-f]+)?(")')
    for page in [ROOT / "sim" / "index.html", ROOT / "sim3d" / "index.html", *sorted((ROOT / "tools").glob("*.html"))]:
        if not page.exists():
            continue
        text = page.read_text(encoding="utf-8")
        new = pat.sub(lambda m: "%s%s?v=%s%s" % (m.group(1), m.group(2), asset_version(), m.group(3)), text)
        if new != text:
            write_lf(page, new)


def lesson_page(cur: dict, cid: str, meta: dict, body_md: str) -> str:
    info = cur["index"][cid]
    r = MarkdownRenderer(depth=1)
    content_html = r.render(body_md)
    order = cur["order"]
    pos = order.index(cid)
    prev_id = order[pos - 1] if pos > 0 else None
    next_id = order[pos + 1] if pos < len(order) - 1 else None

    toc = "".join('<a class="toc-l%d" href="#%s">%s</a>' % (lvl, sid, html.escape(txt))
                  for lvl, sid, txt in r.toc)

    def pager(cls, label, other):
        if not other:
            return '<span class="%s disabled"></span>' % cls
        return ('<a class="%s" href="%s.html"><span>%s · %s</span>%s</a>'
                % (cls, other, label, other.upper(), html.escape(cur["index"][other]["title"])))

    requires = meta.get("requires") or info.get("requires") or []
    if isinstance(requires, str):
        requires = [requires]
    req_html = ""
    if requires:
        req_html = '<div class="requires">선수 챕터: %s</div>' % " ".join(
            '<a href="%s.html">%s</a>' % (x, x.upper()) for x in requires if x in cur["index"])

    tools = []
    for src in (info.get("tools") or [], meta.get("tools") or []):   # curriculum first, then chapter extras
        for t in ([src] if isinstance(src, str) else src):
            if t not in tools:
                tools.append(t)
    tool_btns = "".join('<a class="ghost-btn" href="../%s">%s</a>' % TOOL_LINKS[t]
                        for t in tools if t in TOOL_LINKS)

    level = meta.get("level", info.get("level", ""))
    duration = meta.get("duration", info.get("duration", 60))

    header = """
<article class="lesson" data-slug="{cid}" data-track="{track}">
  <div class="lesson-head">
    <div class="crumbs"><span class="track-pill track-{track}">{track_title}</span> {part} &middot; 약 {minutes}분 &middot; {level}</div>
    <h1><span class="lesson-no">{no}</span>{title}</h1>
    <p class="lede">{summary}</p>
    {req}
    <div class="lesson-actions">
      {tools}
    </div>
  </div>
  <div class="lesson-grid">
    <div class="lesson-body">
{content}
      <nav class="pager">{prev}{next}</nav>
    </div>
    <aside class="lesson-toc"><div class="toc-title">이 장의 내용</div><div class="toc-links">{toc}</div></aside>
  </div>
</article>
""".format(cid=cid, track=info["track"], track_title=html.escape(info["track_title"]),
           part=html.escape(info["part"]), minutes=duration, level=html.escape(level),
           no=cid.upper(), title=html.escape(info["title"]),
           summary=html.escape(info.get("summary", "")), req=req_html, tools=tool_btns,
           content=content_html, prev=pager("pager-prev", "이전", prev_id),
           next=pager("pager-next", "다음", next_id), toc=toc)

    return PAGE.format(
        ver=asset_version(),
        title="%s. %s · studyMentorPi" % (cid.upper(), info["title"]),
        desc=html.escape(info.get("summary", "")),
        rel="../", bodyclass="lesson-page track-page-%s" % info["track"],
        sidebar=sidebar_html(cur, cid, "../"),
        content=header,
        extra='<script type="module" src="../assets/js/lesson-runner.js?v=%s"></script>' % asset_version())


def index_page(cur: dict) -> str:
    total = len(cur["order"])
    sections = []
    for track in cur["tracks"]:
        parts_html = []
        for part in track["parts"]:
            items = []
            for cid in part["chapters"]:
                ch = cur["index"][cid]
                tools = "".join('<span class="tag">%s</span>' % html.escape(TOOL_LINKS[t][1])
                                for t in ch.get("tools", []) if t in TOOL_LINKS)
                items.append(
                    '<a class="lesson-card" href="lessons/%s.html" data-slug="%s">'
                    '<div class="lc-no">%s</div>'
                    '<div class="lc-main"><div class="lc-title">%s</div>'
                    '<div class="lc-sum">%s</div>'
                    '<div class="lc-meta"><span class="lc-min">%s분 · %s</span>%s</div></div>'
                    '</a>'
                    % (cid, cid, cid.upper(), html.escape(ch["title"]),
                       html.escape(ch.get("summary", "")), ch.get("duration", 60),
                       html.escape(ch.get("level", "")), tools))
            parts_html.append(
                '<section class="part"><header class="part-head">'
                '<div class="part-no">%s</div><div><h3>%s</h3></div></header>'
                '<div class="part-list">%s</div></section>'
                % (html.escape(part["no"]), html.escape(part["title"]), "".join(items)))
        sections.append(
            '<div class="track-block" data-track="%s"><h2 class="track-h track-%s">%s</h2>'
            '<p class="track-desc">%s</p>%s</div>'
            % (track["id"], track["id"], html.escape(track["title"]),
               html.escape(track["desc"]), "".join(parts_html)))

    hero = """
<header class="hero">
  <div class="hero-inner">
    <div class="hero-tag">MentorPi &middot; Raspberry Pi 5 &middot; ROS 2 Humble &middot; LiDAR &middot; SLAM &middot; Nav2 &middot; Vision AI</div>
    <h1>studyMentorPi</h1>
    <p class="hero-lede">
      Hiwonder <b>MentorPi</b>(메카넘 / 애커만 섀시, 라즈베리파이 5, 2D 라이다, 3D 깊이 카메라)로 배우는
      <b>이동로봇 인터랙티브 강의</b>입니다. 리눅스·파이썬·OpenCV·ROS 2 기초에서 출발해
      <b>섀시 기구학 &rarr; 오도메트리 &rarr; 라이다 &rarr; SLAM &rarr; 내비게이션</b>을 차례로 쌓고,
      색 인식·MediaPipe·YOLOv5·자율주행·군집 제어까지 이어진 뒤, <b>순찰 경비·QR 배송·미니 시티 자율주행·사람 추종 카트</b>(중급) 와 <b>프런티어 탐사·ICP+EKF 융합·강화학습 주행·시맨틱 자연어 내비</b>(고급) 8개 팀 프로젝트로 통합합니다. 모든 핵심 알고리즘은 브라우저의
      실제 URDF로 움직이는 <b>3D 시뮬레이터</b>, <b>2D 시뮬레이터</b>와 <b>ROS 2 Playground</b>(rclpy 호환 Python)에서 먼저 돌려 보고 실물로 옮깁니다. 총 {total}개 챕터.
    </p>
    <div class="hero-cta">
      <a class="btn primary" href="lessons/b01.html">B01부터 시작</a>
      <a class="btn" href="sim3d/index.html">3D 시뮬레이터 (URDF)</a>
      <a class="btn" href="sim/index.html">2D 시뮬레이터</a>
      <a class="btn" href="tools/playground.html">ROS 2 Playground</a>
      <a class="btn" href="tools/kinematics-lab.html">섀시 기구학 실험실</a>
      <a class="btn" href="tools/color-lab.html">HSV 색 실험실</a>
    </div>
  </div>
  <div class="hero-art">
    <div class="hero-flow">
      <div class="flow-step"><b>1</b><span>기초 &middot; 하드웨어<small>Linux · Python · RPi5 · 확장보드 · Docker</small></span></div>
      <div class="flow-arrow">&darr;</div>
      <div class="flow-step"><b>2</b><span>ROS 2 &middot; OpenCV<small>Node · Topic · Service · TF2 · URDF · 영상처리</small></span></div>
      <div class="flow-arrow">&darr;</div>
      <div class="flow-step"><b>3</b><span>이동로봇 핵심<small>메카넘·애커만 · IMU/Odom · LiDAR · SLAM · Nav2</small></span></div>
      <div class="flow-arrow">&darr;</div>
      <div class="flow-step"><b>4</b><span>비전 AI &middot; 자율주행<small>색 추적 · MediaPipe · YOLOv5 · 차선 · 군집</small></span></div>
      <div class="flow-arrow">&darr;</div>
      <div class="flow-step hot"><b>5</b><span>팀 프로젝트<small>중급 4 · 고급 4 (탐사 · EKF · 강화학습 · 시맨틱)</small></span></div>
    </div>
    <div class="hero-art-cap">같은 rclpy 코드 — 브라우저 시뮬 &rarr; MentorPi 실물</div>
  </div>
</header>

<section class="facts">
  <div class="fact"><div class="fact-n">2</div><div class="fact-l">종 섀시 (메카넘 · 애커만)</div></div>
  <div class="fact"><div class="fact-n">360&deg;</div><div class="fact-l">2D 라이다 + 3D 깊이 카메라</div></div>
  <div class="fact"><div class="fact-n">18</div><div class="fact-l">개 원본 강좌 → 4개 트랙 + 프로젝트 8개</div></div>
  <div class="fact"><div class="fact-n">0</div><div class="fact-l">설치 없이 시작 (브라우저 시뮬)</div></div>
</section>

<section class="curriculum">
  <h2>커리큘럼</h2>
  {sections}
</section>

<footer class="site-foot">
  <p>폴리텍 AI응용소프트웨어과 강의 자료. Hiwonder MentorPi 공식 교재(영문)를 바탕으로 한국어로 재구성·보강했습니다.
  명령어·경로는 공식 이미지(ROS 2 Humble, Docker) 기준이며 펌웨어/이미지 버전에 따라 다를 수 있습니다. 실물 주행 실습은 넓고 안전한 공간에서 진행하세요.</p>
  <p class="foot-links">
    <a href="https://github.com/samcho93/studyMentorPi" target="_blank" rel="noopener">이 사이트 저장소</a>
    <a href="https://www.hiwonder.com/" target="_blank" rel="noopener">Hiwonder</a>
    <a href="https://docs.ros.org/en/humble/" target="_blank" rel="noopener">ROS 2 Humble 문서</a>
    <a href="https://docs.nav2.org/" target="_blank" rel="noopener">Nav2 문서</a>
    <a href="https://pyodide.org" target="_blank" rel="noopener">Pyodide</a>
  </p>
</footer>
""".format(total=total, sections="".join(sections))

    return PAGE.format(
        ver=asset_version(),
        title="studyMentorPi · MentorPi 이동로봇 인터랙티브 강의",
        desc="Hiwonder MentorPi(라즈베리파이 5 + ROS 2)로 배우는 이동로봇: 섀시 기구학·라이다·SLAM·내비게이션·비전·자율주행 한국어 인터랙티브 강의.",
        rel="", bodyclass="home",
        sidebar=sidebar_html(cur, None, ""),
        content=hero, extra="")


# ---------------------------------------------------------------- build

def write_lf(path: Path, text: str) -> None:
    """Always write LF line endings (Path.write_text would emit CRLF on Windows)."""
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)


def load_curriculum() -> dict:
    data = json.loads((CONTENT / "curriculum.json").read_text(encoding="utf-8"))
    index, order = {}, []
    for track in data["tracks"]:
        for part in track["parts"]:
            for cid in part["chapters"]:
                info = data["chapters"][cid]
                info["track"] = track["id"]
                info["track_title"] = track["title"]
                info["part"] = "%s %s" % (part["no"], part["title"])
                index[cid] = info
                order.append(cid)
    data["index"] = index
    data["order"] = order
    return data


def refresh_manifest() -> None:
    """python/manifest.json lists the mentorpi_sim files the Playground mounts into Pyodide."""
    if not (ROOT / "python" / "make_manifest.py").exists():
        return
    import importlib.util
    spec = importlib.util.spec_from_file_location("make_manifest", ROOT / "python" / "make_manifest.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.main()
    mod.write_examples()


def main() -> int:
    refresh_manifest()
    cur = load_curriculum()
    LESSONS.mkdir(exist_ok=True)

    built, missing, problems = 0, [], []
    for cid in cur["order"]:
        track = cur["index"][cid]["track"]
        src = CONTENT / track / (cid + ".md")
        if not src.exists():
            missing.append(cid)
            continue
        meta, body = split_front_matter(src.read_text(encoding="utf-8"))
        for sec in REQUIRED_SECTIONS:
            if not re.search(r"^##\s+%s" % re.escape(sec), body, re.M):
                problems.append("%s: '## %s' 섹션 없음" % (cid, sec))
        if re.search(r"^##\s+실습 \(실물\)", body, re.M) and ":::safety" not in body:
            problems.append("%s: 실물 실습에 :::safety 블록 없음" % cid)
        write_lf(LESSONS / (cid + ".html"), lesson_page(cur, cid, meta, body))
        built += 1

    write_lf(ROOT / "index.html", index_page(cur))
    stamp_tool_pages()
    print("빌드 완료: 챕터 %d개 + index.html" % built)
    if missing:
        print("원고 없음: %s" % ", ".join(missing))
    for p in problems:
        print("경고:", p)

    if "--check" in sys.argv and (missing or problems):
        return 1

    if "--serve" in sys.argv:
        import functools
        import http.server
        import socketserver

        class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
            extensions_map = {**http.server.SimpleHTTPRequestHandler.extensions_map,
                              ".wasm": "application/wasm", ".mjs": "text/javascript"}

            def end_headers(self):
                self.send_header("Cache-Control", "no-store, must-revalidate")
                super().end_headers()

            def log_message(self, fmt, *args):
                pass

        handler = functools.partial(NoCacheHandler, directory=str(ROOT))
        with socketserver.TCPServer(("", 8000), handler) as httpd:
            print("미리보기: http://localhost:8000  (Ctrl+C 로 종료)")
            httpd.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
