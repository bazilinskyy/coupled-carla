"""The PowerShell tools: a function must not call an external command of its own name.

PowerShell resolves names case-insensitively and functions before applications, so `function Wsl` calling
`& wsl ...` calls itself until the call depth overflows (tools/multigpu_win5090.ps1, 2026-11-16); call `wsl.exe`.
"""
import re
from pathlib import Path

TOOLS = Path(__file__).resolve().parent.parent / "tools"


def test_no_function_calls_a_command_of_its_own_name():
    bad = []
    for ps1 in sorted(TOOLS.glob("*.ps1")):
        text = ps1.read_text(encoding="utf-8", errors="replace")
        names = {m.lower() for m in re.findall(r"^\s*function\s+([\w-]+)", text, re.M | re.I)}
        for n, line in enumerate(text.splitlines(), 1):
            for m in re.finditer(r"&\s+([\w.-]+)", line):
                cmd = m.group(1).lower()
                if cmd in names and not cmd.endswith(".exe"):
                    bad.append(f"{ps1.name}:{n}: & {m.group(1)}")
    assert not bad, "functions shadow these commands: " + "; ".join(bad)
