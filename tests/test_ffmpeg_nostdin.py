"""Every ffmpeg the code runs has -nostdin.

Under `timeout` (which puts its command in a process group of its own) in a tmux pane, an ffmpeg without -nostdin
touches the terminal and is stopped by SIGTTOU: `make_videos.py render` hung right after the clip QA until its
timeout (2026-11-24, four CARLA 1.0 twin clips, 35 min each). An ffmpeg that reads its input from a pipe
("-i", "-") is exempt: its stdin is the pipe, not the terminal.
"""
import ast
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))


def _is_ffmpeg(node):
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return Path(node.value).name in ("ffmpeg", "ffmpeg.exe")
    if isinstance(node, ast.Name):
        return node.id.lower() == "ffmpeg"
    if isinstance(node, ast.Attribute):
        return node.attr.lower() == "ffmpeg"
    return False


def _ffmpeg_lists():
    for f in sorted(list((ROOT / "coupledcarla").rglob("*.py")) + list((ROOT / "tools").rglob("*.py"))):
        try:
            tree = ast.parse(f.read_text(), str(f))
        except SyntaxError:
            continue
        for node in ast.walk(tree):
            if isinstance(node, ast.List) and node.elts and _is_ffmpeg(node.elts[0]):
                yield f, node


def test_every_ffmpeg_command_has_nostdin():
    missing = []
    for f, node in _ffmpeg_lists():
        words = [e.value for e in node.elts if isinstance(e, ast.Constant) and isinstance(e.value, str)]
        reads_pipe = any(a == "-i" and b in ("-", "pipe:0", "pipe:") for a, b in zip(words, words[1:]))
        if "-nostdin" not in words and not reads_pipe:
            missing.append(f"{f.relative_to(ROOT)}:{node.lineno}")
    assert not missing, "ffmpeg without -nostdin (hangs under timeout in a terminal): " + ", ".join(missing)


def test_the_scan_finds_the_video_commands():
    found = {f.name for f, _ in _ffmpeg_lists()}
    assert "make_videos.py" in found        # the scan would otherwise pass by finding nothing


def test_make_videos_commands_start_with_nostdin(tmp_path):
    import make_videos as mv
    cmds = [mv.encode_cmd(tmp_path / "%06d.jpg", tmp_path / "a.mp4", 25, 23, audio=tmp_path / "a.wav", duration=2),
            mv.poster_cmd(tmp_path / "000001.jpg", tmp_path / "p.jpg"),
            mv.preview_cmd(tmp_path / "%06d.jpg", tmp_path / "p.webp", 25, 0, 3),
            mv.remux_cmd(tmp_path / "a.mp4", tmp_path / "a.wav", tmp_path / "b.mp4")]
    for c in cmds:
        assert c[1] == "-nostdin", c[:3]
