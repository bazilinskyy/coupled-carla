"""Figures for the README's ship section, from coupledcarla/shipmodel.py (no CARLA): each as a PNG and as an
interactive HTML with Plotly embedded (htmlpreview renders only embedded Plotly).

    python -m tools.ship_figures [--out docs/img]

ships_turning      KVLCC2 (full scale) turning with 35 deg rudder in still water, current, wind and waves
ships_validation   the 7 m KVLCC2 model against Yasukawa & Yoshimura (2015), Tables 4 and 5 (calculated)
ships_waves        KVLCC2's added resistance in regular waves (IMO MEPC.1/Circ.850/Rev.3) by wave direction
ships_look         the KVLCC2 mesh of tools/ship_mesh.py (what the `mesh` look shows in CARLA)
ships_forces       the forces on KVLCC2 in a turn, in wind and waves, drawn where they act

Needs plotly and kaleido (requirements-tools.txt).
"""
import argparse
import math
import os

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from coupledcarla import shipmodel as sm
from tools import ship_mesh

SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"]    # categorical slots 1-5, in this order
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


def look(out):
    m, f, info = ship_mesh.build_ship(sm.KVLCC2)
    traces = []
    for mat, (v, n, uv, idx) in sorted(m.merged().items()):
        r, g, b = (int(255 * c) for c in ship_mesh.COLOURS[mat])
        t = idx.reshape(-1, 3)
        traces.append(go.Mesh3d(x=v[:, 0], y=v[:, 1], z=v[:, 2], i=t[:, 0], j=t[:, 1], k=t[:, 2],
                                color=f"rgb({r},{g},{b})", flatshading=True, name=mat, hoverinfo="name",
                                lighting=dict(ambient=0.5, diffuse=0.8, specular=0.05, roughness=0.9)))
    # the waterline as a thin plane, so the part under water reads as such
    xs = [f.x_ap - 25, f.x_fp + 25]
    ys = [-f.b - 25, f.b + 25]
    traces.append(go.Mesh3d(x=[xs[0], xs[1], xs[1], xs[0]], y=[ys[0], ys[0], ys[1], ys[1]], z=[0, 0, 0, 0],
                            i=[0, 0], j=[1, 2], k=[2, 3], color="rgb(70,120,160)", opacity=0.35, name="Waterline",
                            hoverinfo="name"))
    fig = go.Figure(traces)
    _style(fig, "", 560)
    fig.update_layout(title=dict(text=f"The ship's look in CARLA: a generic VLCC of KVLCC2's size "
                                      f"({info['loa']:.0f} m overall, 58 m beam), made by tools/ship_mesh.py"),
                      scene=dict(aspectmode="data", xaxis_visible=False, yaxis_visible=False, zaxis_visible=False,
                                 camera=dict(eye=dict(x=-0.95, y=1.75, z=0.55)), bgcolor=SURFACE),
                      showlegend=False, margin=dict(l=0, r=0, t=60, b=0))
    _save(fig, out, "ships_look")


def forces(out):
    """KVLCC2 holding a straight course at full revolutions against wind (Beaufort 8, 19 m/s) and waves (Hs 4 m)
    from 60 deg off the starboard bow, once steady (40 min): every force as an arrow from where it acts (top view,
    bow up). A side force acts at its centre of effort, its yaw moment / side force."""
    h = sm.KVLCC2
    env = sm.Environment(wind_mps=19.0, wind_from_deg=60.0, wave_hs=4.0, wave_tp=8.5, wave_from_deg=60.0)
    ship = sm.Ship.at_speed(h, env=env)
    ship.autopilot = sm.Autopilot(heading=0.0)
    for _ in range(2400):
        ship.step(1.0)
    p = ship.force_breakdown()
    m, f, info = ship_mesh.build_ship(h)
    xs, ys = f.waterline()
    L = h.lpp
    scale = 25.0 / 1e6                                   # 1 MN -> 25 m

    def at(k):
        fx, fy, fn = p[k]
        if k == "propeller":
            return f.x_ap + 0.03 * L                     # where the propeller is (x'_P = -0.69 is an inflow value)
        if abs(fy) < 1e3:
            return 0.0
        return max(f.x_ap, min(f.x_fp, fn / fy))         # centre of effort of the side force
    labels = {"hull": "Hull (the water's reaction)", "propeller": "Propeller thrust", "rudder": "Rudder (4° port)",
              "wind": "Wind, Beaufort 8 (Blendermann)", "waves": "Waves, Hs 4 m (mean drift)"}
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=np.r_[ys, -ys[::-1], ys[:1]], y=np.r_[xs, xs[::-1], xs[:1]], mode="lines",
                             fill="toself", fillcolor="rgba(82,81,78,0.10)", line=dict(color=INK2, width=1.5),
                             name="KVLCC2 at the waterline", hoverinfo="skip"))
    for k, colour in zip(["hull", "propeller", "rudder", "wind", "waves"], SERIES):
        fx, fy, fn = p[k]
        x0 = at(k)
        mag = math.hypot(fx, fy)
        fig.add_annotation(x=fy * scale, y=x0 + fx * scale, ax=0.0, ay=x0, xref="x", yref="y", axref="x",
                           ayref="y", showarrow=True, arrowhead=3, arrowsize=1.0, arrowwidth=3, arrowcolor=colour,
                           text="")
        fig.add_trace(go.Scatter(x=[0.0], y=[x0], mode="markers", marker=dict(color=colour, size=11,
                                 line=dict(color=SURFACE, width=2)), name=f"{labels[k]}: {mag / 1e3:,.0f} kN",
                                 hovertemplate=f"{labels[k]}<br>along the ship X {fx / 1e3:,.0f} kN<br>sideways Y "
                                               f"{fy / 1e3:,.0f} kN<br>yaw N {fn / 1e6:,.1f} MN·m<extra></extra>"))
    rel = math.radians(env.wind_from_deg) - ship.psi
    for name, y in (("wind and waves from here", 0.36 * L),):
        d = 70.0
        sx, sy = 100.0 + d * math.sin(rel), y + d * math.cos(rel) - 40
        fig.add_annotation(x=100.0, y=y - 40, ax=sx, ay=sy, xref="x", yref="y", axref="x", ayref="y",
                           showarrow=True, arrowhead=2, arrowwidth=1.5, arrowcolor=INK2, text="")
        fig.add_annotation(x=sx, y=sy, text=name, showarrow=False, yshift=12, font=dict(color=INK2, size=12))
    drift = math.degrees(math.atan2(-ship.vm, ship.u))
    fig.add_annotation(x=-150, y=-185, xanchor="left", showarrow=False, align="left", font=dict(color=INK2, size=12),
                       text=f"Steady at {ship.speed / sm.KNOT:.1f} kn (15.5 in calm water), drifting {drift:.1f}° "
                            f"to port; the forces add up to zero.")
    _style(fig, "Forces on KVLCC2 holding her course in wind and waves (arrows: 1 MN = 25 m)", 760)
    fig.update_xaxes(title="Starboard (m)", scaleanchor="y", scaleratio=1, range=[-160, 160])
    fig.update_yaxes(title="Forward (m)", range=[-200, 200])
    fig.update_layout(legend=dict(orientation="v", x=1.02, y=1.0), margin=dict(r=300), width=1000)
    _save(fig, out, "ships_forces")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", default=os.path.join("docs", "img"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    turning(a.out)
    validation(a.out)
    waves(a.out)
    look(a.out)
    forces(a.out)


if __name__ == "__main__":
    main()
