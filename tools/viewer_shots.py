"""Screenshots of the trial viewer in headless Chromium (for the README), via the viewer's renderAt() hook.

    pip install playwright && python -m playwright install chromium
    python tools/viewer_shots.py viewer/data/eindhoven_centre.json docs/img/viewer \\
        --at 127:follow --at 127:oblique --at 60:top

Serves the project folder on a local port while it runs.
"""
import argparse
import functools
import http.server
import json
import threading
from pathlib import Path

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parent.parent


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("json")
    ap.add_argument("out_prefix")
    ap.add_argument("--at", action="append", default=[], help="seconds:follow|top|oblique|overview|close")
    ap.add_argument("--size", default="1400x860")
    args = ap.parse_args()
    handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=str(ROOT))
    httpd = http.server.ThreadingHTTPServer(("127.0.0.1", 0), handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    port = httpd.server_address[1]
    w, h = (int(v) for v in args.size.split("x"))
    data = json.loads(Path(args.json).read_text())
    ego = next((tr for tr in data.get("tracks", {}).values() if tr["kind"] == "participant"), None)
    with sync_playwright() as p:
        browser = p.chromium.launch(args=["--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader"])
        page = browser.new_page(viewport={"width": w, "height": h})
        rel = Path(args.json).resolve().relative_to(ROOT).as_posix()
        page.goto(f"http://127.0.0.1:{port}/viewer/?file=/{rel}")
        page.wait_for_function("window.viewer && document.getElementById('sub').textContent.includes('lanes')",
                               timeout=120000)
        for spec in args.at:
            sec, mode = spec.split(":")
            sec = float(sec)
            view = None
            if mode in ("oblique", "overview", "close") and ego:
                s = min(ego["s"], key=lambda q: abs(q[0] - data.get("t0", 0) - sec))
                if mode == "oblique":
                    view = {"pos": [s[1] - 140, 150, s[2] + 160], "target": [s[1], 0, s[2]]}
                elif mode == "close":
                    view = {"pos": [s[1] + 28, 22, s[2] + 26], "target": [s[1] - 4, 0, s[2] - 3]}
                else:
                    view = {"pos": [s[1] - 500, 700, s[2] + 650], "target": [s[1] + 100, 0, s[2] - 80]}
            elif mode == "overview":
                view = None
            cam = "orbit" if mode in ("oblique", "overview", "close") else mode
            page.evaluate("([t, cam, view]) => window.viewer.renderAt(t, cam, view)", [sec, cam, view])
            page.wait_for_timeout(500)
            page.evaluate("([t, cam, view]) => window.viewer.renderAt(t, cam, view)", [sec, cam, view])
            out = f"{args.out_prefix}_{mode}_{int(sec)}.png"
            Path(out).parent.mkdir(parents=True, exist_ok=True)
            page.screenshot(path=out)
            print("wrote", out)
        browser.close()
    httpd.shutdown()


if __name__ == "__main__":
    main()
