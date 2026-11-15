"""Listen-proxy check of a recorded soundscape: a spectrogram and the A-weighted level over time as a PNG, and the
level statistics (L_Aeq, L10, L50, L90, L_Amax) printed.

    python tools/sound_check.py logs/<experiment>/<time>_trial0/audio.wav [--out check.png] [--reference-db 94]

The trial's audio.json supplies the calibration (reference_db) and its events.csv, when there, marks the events
of the participant's own vehicle on the level curve.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from coupledcarla import soundrec  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("wav")
    ap.add_argument("--out")
    ap.add_argument("--reference-db", type=float)
    ap.add_argument("--title")
    ap.add_argument("--max-hz", type=float, default=12000.0)
    ap.add_argument("--dpi", type=int, default=80)
    args = ap.parse_args()
    wav = Path(args.wav)
    meta = {}
    if (wav.parent / "audio.json").exists():
        meta = json.loads((wav.parent / "audio.json").read_text())
    ref = args.reference_db or float(meta.get("reference_db", 94.0))
    x, rate = soundrec.read_wav(wav)
    t, lv = soundrec.dba_timeline(x, ref, 0.125, rate)
    leq = soundrec.level_dba(x, ref, rate)
    stats = {"L_Aeq": round(leq, 1), "L_Amax": round(float(lv.max()), 1),
             "L10": round(float(np.percentile(lv, 90)), 1), "L50": round(float(np.percentile(lv, 50)), 1),
             "L90": round(float(np.percentile(lv, 10)), 1), "seconds": round(len(x) / rate, 1),
             "peak": round(float(np.abs(x).max()), 3)}
    print(json.dumps(stats))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(11, 6.2), sharex=True, gridspec_kw={"height_ratios": [2.2, 1]})
    mono = x.mean(axis=1)
    a1.specgram(mono, NFFT=2048, Fs=rate, noverlap=1536, cmap="magma", vmin=-150, vmax=-40)
    a1.set_yscale("symlog", linthresh=500)
    a1.set_ylim(30, args.max_hz)
    a1.set_ylabel("Hz")
    lst = meta.get("listener") or {}
    a1.set_title(args.title or f"{lst.get('kind', '')} {lst.get('role', '')}: {wav.parent.name}  "
                 f"(L_Aeq {stats['L_Aeq']} dB, L90 {stats['L90']}, L10 {stats['L10']})", fontsize=10)
    a2.plot(t, lv, lw=0.8, color="#2a6fb0")
    a2.axhline(leq, color="#c0392b", lw=0.8, ls="--", label=f"L_Aeq {leq:.1f} dB")
    a2.set_ylabel("dB(A), fast")
    a2.set_xlabel("simulation time (s)")
    a2.grid(alpha=0.3)
    a2.legend(loc="lower right", fontsize=8)
    fig.tight_layout()
    out = Path(args.out) if args.out else wav.with_name("audio_check.png")
    fig.savefig(out, dpi=args.dpi, **({"pil_kwargs": {"quality": 82}} if out.suffix.lower() in (".jpg", ".jpeg")
                                      else {}))
    print(out)


if __name__ == "__main__":
    main()
