/* playground-worker.js — Pyodide + mentorpi_sim (rclpy shim) in a Web Worker.
 *
 * The student's script runs synchronously here, so `while True:` cannot freeze the page.
 * Simulated time advances only inside rclpy.spin / spin_once / time.sleep; when the script
 * ends the recorded frames (+ cv2.imshow images, plots) are posted to the page for replay.
 * "정지" terminates this worker (the page starts a fresh one).
 *
 * main -> worker : {type:'init', base}  {type:'run', code, runId}
 * worker -> main : status | ready | fatal | stdout | stderr | done {runId, status, result}
 */
'use strict';

const PYODIDE_VERSION = '0.29.5';
const PYODIDE_BASE = 'https://cdn.jsdelivr.net/pyodide/v' + PYODIDE_VERSION + '/full/';
const MOUNT = '/home/pyodide/lib';
const WORK = '/home/pyodide';

let pyodide = null;
let runner = null;
const post = (m) => self.postMessage(m);

const RUNNER = `
import json, sys, traceback, os
import mentorpi_sim

FILENAME = "main.py"

def run(source):
    os.chdir(${JSON.stringify(WORK)})
    mentorpi_sim.begin_run()
    if "cv2" in source:
        try:
            import cv2
            mentorpi_sim.install_cv2_hooks(cv2)
        except Exception as e:
            sys.stderr.write("cv2 로드 실패: %s\\n" % e)
    status = "ok"
    try:
        code = compile(source, FILENAME, "exec")
        exec(code, {"__name__": "__main__", "__file__": FILENAME})
    except SystemExit as e:
        status = "ok" if e.code in (None, 0) else "error"
    except mentorpi_sim.SimTimeUp:
        status = "ok"
    except BaseException as exc:
        tb = [f for f in traceback.extract_tb(exc.__traceback__) if f.filename == FILENAME]
        lines = ["Traceback (most recent call last):\\n"] + traceback.format_list(tb)
        lines += traceback.format_exception_only(type(exc), exc)
        sys.stderr.write("".join(lines))
        status = "error"
    sys.stdout.flush()
    return json.dumps({"status": status, "result": json.loads(mentorpi_sim.export_json())})
`;

async function init(base) {
  post({ type: 'status', text: 'Pyodide ' + PYODIDE_VERSION + ' 내려받는 중… (처음 한 번 약 10 MB)' });
  importScripts(PYODIDE_BASE + 'pyodide.js');
  pyodide = await self.loadPyodide({ indexURL: PYODIDE_BASE });
  pyodide.setStdout({ batched: (s) => post({ type: 'stdout', text: s }) });
  pyodide.setStderr({ batched: (s) => post({ type: 'stderr', text: s }) });
  pyodide.setStdin({ stdin: () => undefined });

  post({ type: 'status', text: 'numpy 불러오는 중…' });
  await pyodide.loadPackage('numpy', { messageCallback: () => {} });

  post({ type: 'status', text: 'mentorpi_sim (rclpy shim) 마운트 중…' });
  const manifest = await (await fetch(base + 'manifest.json', { cache: 'no-cache' })).json();
  const texts = await Promise.all(manifest.files.map(async (rel) => {
    const res = await fetch(base + rel + '?v=' + manifest.version);
    if (!res.ok) throw new Error(rel + ' → HTTP ' + res.status);
    return [rel, await res.text()];
  }));
  for (const [rel, text] of texts) {
    const path = MOUNT + '/' + rel;
    pyodide.FS.mkdirTree(path.slice(0, path.lastIndexOf('/')));
    pyodide.FS.writeFile(path, text);
  }
  const bins = await Promise.all((manifest.samples || []).map(async (rel) => {
    const res = await fetch(base + rel + '?v=' + manifest.version);
    if (!res.ok) throw new Error(rel + ' → HTTP ' + res.status);
    return [rel, new Uint8Array(await res.arrayBuffer())];
  }));
  for (const [rel, data] of bins) pyodide.FS.writeFile(WORK + '/' + rel.split('/').pop(), data);

  pyodide.runPython('import sys\nsys.path.insert(0, ' + JSON.stringify(MOUNT) + ')\n');
  pyodide.runPython(RUNNER);
  runner = pyodide.globals.get('run');
  post({
    type: 'ready', pyodide: PYODIDE_VERSION, lib: manifest.version,
    python: pyodide.runPython('import sys; sys.version.split()[0]'),
  });
}

async function run(msg) {
  let res;
  try {
    if (/\bimport\s+cv2\b|\bfrom\s+cv2\b/.test(msg.code)) post({ type: 'status', text: 'OpenCV 불러오는 중… (처음 한 번 수십 MB)' });
    try { await pyodide.loadPackagesFromImports(msg.code, { messageCallback: () => {} }); } catch (e) { /* surfaces below */ }
    post({ type: 'status', text: '실행 중…' });
    res = JSON.parse(runner(msg.code));
  } catch (e) {
    post({ type: 'stderr', text: String(e && e.message ? e.message : e) });
    res = { status: 'error', result: null };
  }
  post({ type: 'done', runId: msg.runId, status: res.status, result: res.result });
}

self.onmessage = (ev) => {
  const m = ev.data || {};
  if (m.type === 'init') init(m.base).catch((e) => post({ type: 'fatal', error: String(e && e.message ? e.message : e) }));
  else if (m.type === 'run') run(m);
};
