"""Figures for the README's ship section, from coupledcarla/shipmodel.py (no CARLA): each as a PNG and as an
interactive HTML with Plotly embedded (htmlpreview renders only embedded Plotly).

    python -m tools.ship_figures [--out docs/img]

ships_turning      KVLCC2 (full scale) turning with 35 deg rudder in still water, current, wind and waves
ships_validation   the 7 m KVLCC2 model against Yasukawa & Yoshimura (2015), Tables 4 and 5 (calculated)
ships_waves        KVLCC2's added resistance in regular waves (IMO MEPC.1/Circ.850/Rev.3) by wave direction

Needs plotly and kaleido (requirements-tools.txt).
"""
import argparse
import math
import os

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from coupledcarla import shipmodel as sm

SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]       # categorical slots 1-4, in this order
INK, INK2, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"


def _style(fig, title, height=520):
    fig.update_layout(title=dict(text=title, x=0.01, font=dict(size=17, color=INK)), template="plotly_white",
                      paper_bgcolor=SURFACE, plot_bgcolor=SURFACE, height=height, width=900,
                      font=dict(family="Helvetica, Arial, sans-serif", size=13, color=INK2),
                      legend=dict(orientation="h", y=-0.16, x=0.0), margin=dict(l=70, r=30, t=70, b=90),
                      hovermode="closest")
    fig.update_xaxes(gridcolor=GRID, zerolinecolor=GRID, linecolor=INK2)
    fig.update_yaxes(gridcolor=GRID, zerolinecolor=GRID, linecolor=INK2)
    return fig


def _save(fig, out, name):
    fig.write_html(os.path.join(out, name + ".html"), include_plotlyjs=True, full_html=True)
    fig.write_image(os.path.join(out, name + ".png"), scale=2)
    print("wrote", os.path.join(out, name) + ".{png,html}")


def turning(out):
    h = sm.KVLCC2
    cases = [("Still water", None),
             ("Current 1 m/s to the east", sm.Environment(current_mps=1.0, current_to_deg=90.0)),
             ("Wind 19 m/s from the north (Beaufort 8)", sm.Environment(wind_mps=19.0, wind_from_deg=0.0)),
             ("Waves Hs 5.5 m, Tp 9.1 s from the north", sm.Environment(wave_hs=5.5, wave_tp=9.1,
                                                                         wave_from_deg=0.0))]
    fig = go.Figure()
    for (name, env), colour in zip(cases, SERIES):
        ship = sm.Ship.at_speed(h, env=env)
        ship.rudder_cmd = math.radians(35)
        xs, ys, ts, kn = [0.0], [0.0], [0.0], [ship.speed / sm.KNOT]
        while ship.t < 1800:
            ship.step(1.0)
            if int(ship.t) % 10 == 0:
                xs.append(ship.x / h.lpp), ys.append(ship.y / h.lpp), ts.append(ship.t), kn.append(ship.speed / sm.KNOT)
        fig.add_trace(go.Scatter(x=ys, y=xs, mode="lines", name=name, line=dict(color=colour, width=2),
                                 customdata=np.c_[ts, kn],
                                 hovertemplate=name + "<br>t %{customdata[0]:.0f} s, %{customdata[1]:.1f} kn"
                                               "<br>east %{x:.2f} L, north %{y:.2f} L<extra></extra>"))
    fig.add_trace(go.Scatter(x=[0], y=[0], mode="markers", name="Rudder order (15.5 kn, heading north)",
                             marker=dict(color=INK, size=9, symbol="triangle-up"), hoverinfo="name"))
    _style(fig, "KVLCC2 (320 m) turning with 35° rudder for 30 min: the environment moves the circles", 620)
    fig.update_xaxes(title="East (ship lengths)")
    fig.update_yaxes(title="North (ship lengths)", scaleanchor="x", scaleratio=1)
    _save(fig, out, "ships_turning")


def validation(out):
    l7 = sm.KVLCC2_L7
    t_s, t_p = sm.turning_circle(l7, 35), sm.turning_circle(l7, -35)
    z = {r: sm.zigzag(l7, r) for r in (10, 20, -10, -20)}
    turn_labels = ["Advance<br>35°", "Tactical<br>diameter 35°", "Advance<br>−35°", "Tactical<br>diameter −35°"]
    turn_paper = [3.31, 3.36, 3.26, 3.26]
    turn_model = [t_s["advance"], t_s["tactical_diameter"], t_p["advance"], t_p["tactical_diameter"]]
    zz_labels = ["10/10<br>1st", "10/10<br>2nd", "20/20<br>1st", "−10/−10<br>1st", "−10/−10<br>2nd", "−20/−20<br>1st"]
    zz_paper = [5.2, 15.8, 10.9, 7.6, 10.2, 14.5]
    zz_model = [z[10]["osa1"], z[10]["osa2"], z[20]["osa1"], z[-10]["osa1"], z[-10]["osa2"], z[-20]["osa1"]]
    fig = make_subplots(rows=1, cols=2, column_widths=[0.42, 0.58], horizontal_spacing=0.1,
                        subplot_titles=("Turning (ship lengths)", "Zig-zag overshoot angles (°)"))
    for col, labels, paper, model, unit in [(1, turn_labels, turn_paper, turn_model, " L"),
                                            (2, zz_labels, zz_paper, zz_model, "°")]:
        for name, vals, colour in [("Paper's calculation (Tables 4, 5)", paper, SERIES[0]),
                                   ("This model (shipmodel.py)", model, SERIES[1])]:
            fig.add_trace(go.Bar(x=labels, y=vals, name=name, marker=dict(color=colour, line=dict(width=0)),
                                 showlegend=(col == 1), legendgroup=name,
                                 hovertemplate="%{x}: " + name + ": %{y:.2f}" + unit + "<extra></extra>"),
                          row=1, col=col)
    _style(fig, "The 7 m KVLCC2 model against Yasukawa & Yoshimura (2015)")
    fig.update_layout(barmode="group", bargap=0.3, bargroupgap=0.08)
    fig.update_annotations(font=dict(size=14, color=INK))
    fig.update_xaxes(tickangle=0)
    _save(fig, out, "ships_validation")


def waves(out):
    h = sm.KVLCC2
    u = 15.5 * sm.KNOT
    lam = np.linspace(0.25, 2.5, 181)
    w = np.sqrt(2 * math.pi * sm.G / (lam * h.lpp))
    fig = go.Figure()
    for (beta, name), colour in zip([(180, "Head seas"), (150, "30° off the bow"), (120, "60° off the bow"),
                                     (90, "Beam seas")], SERIES):
        sig = sm.added_resistance_rao(h, w, math.radians(beta), u) / (h.rho * sm.G * h.beam ** 2 / h.lpp)
        fig.add_trace(go.Scatter(x=lam, y=sig, mode="lines", name=name, line=dict(color=colour, width=2),
                                 hovertemplate=name + "<br>λ/L %{x:.2f}: σ %{y:.2f}<extra></extra>"))
    _style(fig, "KVLCC2 at 15.5 kn: added resistance in regular waves (IMO MEPC.1/Circ.850/Rev.3)")
    fig.update_xaxes(title="Wave length / ship length")
    fig.update_yaxes(title="R<sub>AW</sub> / (ρ g A² B² / L)", rangemode="tozero")
    _save(fig, out, "ships_waves")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", default=os.path.join("docs", "img"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    turning(a.out)
    validation(a.out)
    waves(a.out)


if __name__ == "__main__":
    main()
