# ==============================================================================
# HOW TO READ THIS FILE
# Lines that start with "#" are notes for people. The computer skips them.
# Every other line is Python: instructions the computer follows from top to bottom.
# Text between three quote marks (""" ... """) is a "docstring": a description
# written for programmers. The computer keeps it but does not run it.
# 
# WHAT THIS FILE IS FOR
# It draws the project's pictures and some of its animations. It uses
# "matplotlib", a toolbox for drawing charts. A few words you will meet:
#   figure  - the whole page (fig);
#   axes    - one drawing area on the page, with its own x and y scale (ax);
#   plot    - draw lines; scatter - draw dots; text - write words;
#   patch   - a filled shape (a polygon, a rectangle ...);
#   legend  - the little key that says what each colour means;
#   dpi     - "dots per inch": how many pixels make one inch of picture;
#   zorder  - which things are drawn on top (higher numbers on top);
#   alpha   - see-through-ness: 1 is solid, 0 is invisible;
#   lw, ls  - line width, and line style ("-" solid, "--" dashed, ":" dotted);
#   colours are written like "#2a78d6": two hexadecimal digits each for red,
#   green and blue.
# ==============================================================================
"""Figures: the planned routes (PNG), an optional animation (GIF), and the comparison of the two Dubins methods."""
# "import" borrows a toolbox. matplotlib draws charts and pictures.
import matplotlib

# "Agg" tells matplotlib to draw into files only, without opening a window.
matplotlib.use("Agg")
# pyplot (nicknamed plt): matplotlib's drawing commands.
import matplotlib.pyplot as plt
# numpy (nicknamed np) works with whole lists ("arrays") of numbers at once.
import numpy as np
# Line2D: a line, used here to make sample lines for legends.
from matplotlib.lines import Line2D
# MaxNLocator: chooses tidy numbers for the marks on an axis.
from matplotlib.ticker import MaxNLocator
# Patch: a sample colour box for legends; PathPatch: a shape drawn from a path.
from matplotlib.patches import Patch, PathPatch
# A filled polygon shape for drawing (renamed PolygonPatch, so it is not
# confused with shapely's Polygon on the next lines).
from matplotlib.patches import Polygon as PolygonPatch
# Path: an outline made of straight and curved pieces.
from matplotlib.path import Path
# shapely's Polygon: a shape for geometry (here, to grow obstacles by r_safe).
from shapely.geometry import Polygon

# Eight distinguishable colours; more sinks reuse them with another line style.
COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
# Line styles: solid, dashed, dotted. After 8 robots the colours repeat with the next style.
STYLES = ["-", "--", ":"]
# INK: almost black, for text. MUTED: grey, for less important things.
INK, MUTED = "#14171c", "#6b6f76"


# "def" starts a recipe (a "function"). This one gives robot number k its
# colour and line style. The "_" at the start means "only used inside this file".
def _style(k):
    # "%" is the remainder after dividing: robot 9 gets colour 9 % 8 = 1.
    # "//" divides and drops the remainder: robots 8-15 get style 1 (dashed).
    return COLORS[k % len(COLORS)], STYLES[(k // len(COLORS)) % len(STYLES)]


# Draws the map on the drawing area ax: obstacles, their safety band, the base.
# out is everything planner.run handed back.
def _map(ax, out):
    # Short names for the map and the settings.
    scen, p = out["scenario"], out["params"]
    # For every obstacle o:
    for o in scen["obstacles"]:
        # Draw it as a filled grey shape (lw=0: no outline).
        ax.add_patch(PolygonPatch(o, closed=True, color="#9a9ea6", alpha=0.8, lw=0))
        # The outline of the obstacle grown by r_safe (2 m): its x and y values.
        x, y = Polygon(o).buffer(p["r_safe"]).exterior.xy
        # Draw that outline as a thin dashed grey line.
        ax.plot(x, y, color=MUTED, lw=0.6, ls="--")
    # The base station as a black triangle ("^"), on top of everything (zorder=10).
    ax.plot(*scen["base"], marker="^", ms=11, color=INK, zorder=10)
    # One metre across = one metre up, so shapes are not squashed.
    ax.set_aspect("equal")
    # A faint grid.
    ax.grid(color="#e4e3df", lw=0.5)
    # Put the grid behind everything else.
    ax.set_axisbelow(True)


# Draws the sensors, each in the colour of the robot that visits it.
def _sensors(ax, out, marker="o", size=14):
    """The static sensors, each in the colour of the sink that serves it."""
    # The map.
    scen = out["scenario"]
    # For every robot k and its tour ("enumerate" counts 0, 1, 2 ...):
    for k, tour in enumerate(out["result"]["tours"]):
        # Dots at its sensors (xy[tour order]; .T swaps rows and columns, giving
        # the x list and the y list), in its colour.
        ax.scatter(*scen["xy"][tour["order"]].T, s=size, marker=marker, color=_style(k)[0], zorder=5)


# Draws the picture of all the planned routes and saves it to the file "path".
def routes(out, path):
    """Save a PNG with obstacles, the r_safe boundary, every sink's trajectory and its sensors."""
    # Short names for the map, the result and the checks.
    scen, res, c = out["scenario"], out["result"], out["checks"]
    # A new page, 9 by 8 inches, with one drawing area.
    fig, ax = plt.subplots(figsize=(9, 8))
    # The obstacles and the base.
    _map(ax, out)
    # For every robot k:
    for k, tour in enumerate(res["tours"]):
        # Its colour and line style.
        color, ls = _style(k)
        # Its frames (its position every 0.5 s).
        f = res["frames"][k]
        # Draw its drive as a line. f[:, 0] is every x, f[:, 1] every y. "MS 3"
        # means mobile sink 3, the label in the legend.
        ax.plot(f[:, 0], f[:, 1], color=color, ls=ls, lw=1.4, label=f"MS {k}")
        # Its sensor furthest from the base (np.argmax finds the biggest distance) ...
        far =scen["xy"][tour["order"]][np.argmax(np.hypot(*(scen["xy"][tour["order"]] - scen["base"]).T))]
        # ... where its number is written, 4 points up and to the right.
        ax.annotate(str(k), far, xytext=(4, 4), textcoords="offset points", fontsize=8, color=INK)
    # The sensors, in their robots' colours.
    _sensors(ax, out)
    # The title: the scores and how many conflicts remain.
    ax.set_title(f"K = {c['K']} sinks, T_max = {c['T_max']:.1f} s, L_total = {c['L_total']:.0f} m, "
                 f"remaining conflicts = {c['conflict_pairs']}", fontsize=10, color=INK)
    # The legend, just outside the drawing area on the right.
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=7, frameon=False)
    # Arrange everything neatly on the page.
    fig.tight_layout()
    # Save the picture, 150 pixels per inch.
    fig.savefig(path, dpi=150)
    # Close the page, to free the memory.
    plt.close(fig)


# Makes a GIF film of all the robots moving, 12 pictures per second (fps,
# "frames per second").
def animation(out, path, fps=12):
    """Save a GIF of all sinks moving (one image per frame of dt seconds)."""
    # matplotlib's film-making tools (nicknamed anim).
    import matplotlib.animation as anim
    # Short names for the result and the settings.
    res, p = out["result"], out["params"]
    # Every robot's frames.
    frames = res["frames"]
    # The film needs as many pictures as the longest robot has frames.
    n = max(len(f) for f in frames.values())
    # A 7 by 7 inch page with one drawing area.
    fig, ax = plt.subplots(figsize=(7, 7))
    # The obstacles and the base.
    _map(ax, out)
    # For every robot k and its frames f:
    for k, f in frames.items():
        # Its whole route as a thin, faint line.
        ax.plot(f[:, 0], f[:, 1], color=_style(k)[0], lw=0.6, alpha=0.4)
    # The sensors as small squares ("s").
    _sensors(ax, out, marker="s", size=18)
    # A legend made by hand from three sample markers: sensor, robot, base.
    ax.legend(handles=[plt.Line2D([], [], ls="", marker="s", ms=5, color=MUTED, label="sensor (colour of its sink)"),
                       plt.Line2D([], [], ls="", marker="o", ms=7, color=MUTED, mec=INK, mew=0.5, label="mobile sink"),
                       plt.Line2D([], [], ls="", marker="^", ms=8, color=INK, label="base station")],
              loc="upper right", fontsize=7, framealpha=0.9)
    # One dot per robot, empty for now; the film moves them. [0] takes the one
    # line that plot hands back.
    dots = {k: ax.plot([], [], marker="o", ms=7, color=_style(k)[0], mec=INK, mew=0.5, zorder=8)[0] for k in frames}
    # A text in the top-left corner for the clock (transAxes: positions from
    # 0 to 1 across the drawing area).
    label = ax.text(0.02, 0.97, "", transform=ax.transAxes, fontsize=9, va="top")

    # Draws picture number i of the film.
    def update(i):
        # For every robot:
        for k, f in frames.items():
            # Its position at frame i (or its last one, if it has already finished).
            x, y = f[min(i, len(f) - 1)]
            # Move its dot there.
            dots[k].set_data([x], [y])
        # Show the time: frame number times 0.5 s.
        label.set_text(f"t = {i * p['dt']:.1f} s")
        # Hand back the things that changed, so only they are redrawn.
        return list(dots.values()) + [label]

    # The film: call update for pictures 0 to n - 1 (blit=True: redraw only what changed).
    ani = anim.FuncAnimation(fig, update, frames=n, blit=True)
    # Save it as a GIF, using the "pillow" picture toolbox.
    ani.save(path, writer="pillow", fps=fps)
    # Close the page.
    plt.close(fig)


# ---------------------------------------------------------------------------
# Comparison of the two Dubins methods (compare_dubins.py --figures)
# ---------------------------------------------------------------------------

SHORTEST, FEASIBLE = COLORS[0], COLORS[1]     # each method keeps its colour in every figure
# More colours: the page, the grid lines, a neutral grey, and quiet text.
SURFACE, GRID, BASELINE, QUIET = "#fcfcfb", "#e1e0d9", "#c3c2b7", "#52514e"
# The colour of each kind of leg (see compare_dubins.py): grey, orange, dark grey.
KIND_COLOR = {"shortest": BASELINE, "longer": FEASIBLE, "missing": QUIET}
# The words for each kind of leg, for legends.
KIND_LABEL = {"shortest": "Rule's type is the shortest", "longer": "Rule's type exists but is longer", "missing": "Rule's type does not exist (fallback)"}
# The colour of text written on each kind's bar (white on dark bars).
KIND_INK = {"shortest": INK, "longer": "#ffffff", "missing": "#ffffff"}
DASHES = (0, (3.5, 2.5))                      # feasible paths are dashed too, so they do not rely on colour alone


# Starts a comparison picture: the page, its drawing areas, a title and a
# subtitle. size is (width, height) in inches; left, right, bottom and top
# say where the drawing areas go (as fractions of the page); **grid takes
# the number of rows and columns.
def _start(size, title, subtitle, left=0.1, right=0.97, bottom=0.12, top=0.8, **grid):
    """A figure with its title and subtitle. The layout is fixed here, because _bar measures pixels."""
    # The page and its drawing areas.
    fig, axes = plt.subplots(figsize=size, dpi=150, facecolor=SURFACE, **grid)
    # Where the drawing areas sit on the page.
    fig.subplots_adjust(left=left, right=right, bottom=bottom, top=top)
    # The title, 0.14 inches from the top (positions run from 0 to 1, so inches
    # are divided by the page height).
    fig.text(0.02, 1 - 0.14 / size[1], title, fontsize=12, fontweight="bold", color=INK, va="top")
    # The subtitle, below it, in quieter grey.
    fig.text(0.02, 1 - 0.40 / size[1], subtitle, fontsize=9, color=QUIET, va="top")
    # Hand back the page and the drawing areas.
    return fig, axes


# Makes a drawing area plain and quiet, so the data stands out.
def _quiet_axes(ax, grid_axis=None):
    """Recessive axes: no box, hairline grid, muted ticks."""
    # The page colour as background.
    ax.set_facecolor(SURFACE)
    # The four border lines ("spines") ...
    for side in ax.spines:
        # ... are hidden.
        ax.spines[side].set_visible(False)
    # No tick marks, small grey numbers.
    ax.tick_params(length=0, labelsize=8.5, colors=MUTED)
    # Grid behind the data.
    ax.set_axisbelow(True)
    # If asked for, grid lines along one direction ("x" or "y") ...
    if grid_axis:
        # ... drawn thin and light.
        ax.grid(axis=grid_axis, color=GRID, lw=0.7)


# Draws one bar from (x0, y0) to (x1, y1), with its end rounded (on the
# "right" or at the "top") and its base square.
def _bar(ax, x0, x1, y0, y1, color, round_end=None):
    """A bar given in data coordinates, rounded by about 4 px at round_end ("right" or "top") and square at its baseline."""
    # A converter from screen pixels to data values.
    to_data = ax.transData.inverted()
    # How much 6 pixels are in data values, across and up.
    (a0, b0), (a1, b1) = to_data.transform([(0, 0), (6, 6)])
    # The rounding's size: 6 pixels, but at most half the bar.
    rx, ry = min(abs(a1 - a0), abs(x1 - x0) / 2), min(abs(b1 - b0), abs(y1 - y0) / 2)
    # Rounded on the right:
    if round_end == "right":
        # The outline's points, going round the bar.
        verts = [(x0, y0), (x1 - rx, y0), (x1, y0), (x1, y0 + ry), (x1, y1 - ry), (x1, y1), (x1 - rx, y1), (x0, y1), (x0, y0)]
        # What each point is: start, straight line to it, or a curve's control point
        # (CURVE3 makes the rounded corners).
        codes = [Path.MOVETO, Path.LINETO, Path.CURVE3, Path.CURVE3, Path.LINETO, Path.CURVE3, Path.CURVE3, Path.LINETO, Path.CLOSEPOLY]
    # Rounded at the top:
    elif round_end == "top":
        # The outline's points.
        verts = [(x0, y0), (x1, y0), (x1, y1 - ry), (x1, y1), (x1 - rx, y1), (x0 + rx, y1), (x0, y1), (x0, y1 - ry), (x0, y0)]
        # What each point is.
        codes = [Path.MOVETO, Path.LINETO, Path.LINETO, Path.CURVE3, Path.CURVE3, Path.LINETO, Path.CURVE3, Path.CURVE3, Path.CLOSEPOLY]
    # Not rounded at all:
    else:
        # A plain rectangle's four corners (and back to the start).
        verts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
        # Straight lines all round, then close the shape.
        codes = [Path.MOVETO, Path.LINETO, Path.LINETO, Path.LINETO, Path.CLOSEPOLY]
    # Draw the filled shape.
    ax.add_patch(PathPatch(Path(verts, codes), facecolor=color, lw=0, zorder=3))


# The legend of the two methods, at height y on the page.
def _method_legend(fig, y):
    # Blue solid line: shortest. Orange dashed line: the rule (feasible).
    fig.legend(handles=[Line2D([], [], color=SHORTEST, lw=2, label="Shortest (best of six)"),
                        Line2D([], [], color=FEASIBLE, lw=2, ls=DASHES, label="Feasible (rule of Section 0.13)")],
               loc="upper left", bbox_to_anchor=(0.012, y), ncol=2, frameon=False, fontsize=9, labelcolor=INK, handlelength=2.6)


# The legend of the three kinds of leg.
def _kind_legend(fig, y):
    # One colour box per kind, side by side.
    fig.legend(handles=[Patch(color=KIND_COLOR[k], label=KIND_LABEL[k]) for k in KIND_COLOR],
               loc="upper left", bbox_to_anchor=(0.012, y), ncol=3, frameon=False, fontsize=8.5, labelcolor=INK, handlelength=1.2, columnspacing=1.4)


# Picture: the four example legs, each with both paths.
def compare_example_legs(legs, R, path):
    """Four legs, all at one scale: both paths between the same two poses."""
    # The page (9 by 7.6 inches) with 2 x 2 drawing areas, a title and a subtitle.
    fig, axes = _start((9, 7.6), "The rule often finds the shortest path, but not always",
                       f"Four example legs with R = {R:.0f} m, all at the same scale. Arrows show the start and end headings.",
                       left=0.03, right=0.97, bottom=0.03, top=0.825, nrows=2, ncols=2)
    # Gaps between the drawing areas.
    fig.subplots_adjust(hspace=0.3, wspace=0.08)
    # The methods' legend under the subtitle.
    _method_legend(fig, 1 - 0.62 / 7.6)
    # The size of the biggest leg (np.ptp: largest minus smallest value), plus
    # 8 %, so all four use the same scale.
    span = 1.08 * max(max(np.ptp(leg[m][a]) for m in ("shortest", "feasible") for a in "xy") for leg in legs)
    # For each drawing area and its leg (axes.flat lists the 2 x 2 areas one by one):
    for ax, leg in zip(axes.flat, legs):
        # Page-coloured background.
        ax.set_facecolor(SURFACE)
        # No axes, numbers or border.
        ax.axis("off")
        # Equal scales, so circles look round.
        ax.set_aspect("equal")
        # All x values and all y values of both paths together.
        xs, ys = np.concatenate([leg["shortest"]["x"], leg["feasible"]["x"]]), np.concatenate([leg["shortest"]["y"], leg["feasible"]["y"]])
        # The middle of the leg.
        cx, cy = (xs.min() + xs.max()) / 2, (ys.min() + ys.max()) / 2
        # Show "span" metres across, centred on the leg ...
        ax.set_xlim(cx - span / 2, cx + span / 2)
        # ... and the same up and down.
        ax.set_ylim(cy - span / 2, cy + span / 2)
        # The shortest path: blue, solid.
        ax.plot(leg["shortest"]["x"], leg["shortest"]["y"], color=SHORTEST, lw=2, solid_capstyle="round", zorder=3)
        # The rule's path: orange, dashed, on top.
        ax.plot(leg["feasible"]["x"], leg["feasible"]["y"], color=FEASIBLE, lw=2, ls=DASHES, zorder=4)
        # For the start pose and the end pose:
        for q, name in ((leg["q0"], "start"), (leg["q1"], "end")):
            # The tip of an arrow 0.9 R long, pointing along the heading q[2].
            tip = (q[0] + 0.9 * R * np.cos(q[2]), q[1] + 0.9 * R * np.sin(q[2]))
            # Draw that arrow from the point to the tip.
            ax.annotate("", xy=tip, xytext=q[:2], arrowprops=dict(arrowstyle="-|>", color=INK, lw=1.1, mutation_scale=11), zorder=5)
            # A black dot on the point.
            ax.plot(*q[:2], marker="o", ms=5, color=INK, mec=SURFACE, mew=1.2, zorder=6)
            spots = [(q[0] + dx, q[1] + dy) for dx in (-3.5, 3.5) for dy in (-2.2, 2.2)]     # the clearest spot beside the point
            # Of four spots around the point, the one furthest from both paths.
            label = max(spots, key=lambda c: np.min(np.hypot(xs - c[0], ys - c[1])))
            # Write "start" or "end" there.
            ax.text(*label, name, ha="center", va="center", fontsize=8, color=MUTED, zorder=6)
        # Short names for the two paths.
        a, b = leg["shortest"], leg["feasible"]
        # How much longer the rule's path is, e.g. " (+12 %)", or nothing if it is not longer.
        extra = f"  (+{100 * (b['length'] / a['length'] - 1):.0f} %)" if b["length"] > a["length"] + 1e-6 else ""
        # The leg's title, in bold, top left.
        ax.set_title(f"{leg['title']}\n", loc="left", fontsize=9.5, fontweight="bold", color=INK, pad=4)
        # Under it: both shapes and lengths.
        ax.text(0, 1.012, f"Shortest {a['turnType']} {a['length']:.1f} m   |   Feasible {b['turnType']} {b['length']:.1f} m{extra}",
                transform=ax.transAxes, fontsize=8.5, color=QUIET, va="bottom")
        # Bottom-left corner of the drawing area, a little inside.
        x0, y0 = cx - span / 2 + 0.02 * span, cy - span / 2 + 0.03 * span
        # A scale bar 10 m long ...
        ax.plot([x0, x0 + 10], [y0, y0], color=MUTED, lw=1.2)
        # ... labelled "10 m".
        ax.text(x0 + 5, y0 + 0.012 * span, "10 m", ha="center", va="bottom", fontsize=7.5, color=MUTED)
    # Save the picture.
    fig.savefig(path, facecolor=SURFACE)
    # Close the page.
    plt.close(fig)


# Picture: how much longer the rule's plan is, map by map, as bars.
def compare_extra_length(rows, path):
    """Extra total length of the feasible plan on each map, largest first; maps where the number of sinks changed stand out."""
    # The "shortest" rows, looked up by map number.
    short = {r["seed"]: r for r in rows if r["method"] == "shortest"}
    # For each map: extra length in %, map number, robots before and after;
    # sorted from the smallest extra to the largest.
    items = sorted(((100 * (r["L"] / short[r["seed"]]["L"] - 1), r["seed"], short[r["seed"]]["K"], r["K"]) for r in rows if r["method"] == "pdf"))
    # The extra length over all maps together, in %.
    total = 100 * (sum(r["L"] for r in rows if r["method"] == "pdf") / sum(r["L"] for r in short.values()) - 1)
    # Page height: room for the titles plus 0.34 inches per bar.
    height = 1.45 + 0.34 * len(items)
    # The page, its title and subtitle.
    fig, ax = _start((7.5, height), f"Feasible plans are longer on every map: +{total:.1f} % in total",
                     "Extra total path length compared with the shortest-path plan, for each method's own routes",
                     left=0.1, right=0.97, bottom=0.42 / height, top=1 - 1.0 / height)
    # Plain drawing area with vertical grid lines.
    _quiet_axes(ax, "x")
    # Legend: orange where the number of robots changed, grey where it did not.
    fig.legend(handles=[Patch(color=FEASIBLE, label="Number of sinks changed"), Patch(color=BASELINE, label="Same number of sinks")],
               loc="upper left", bbox_to_anchor=(0.012, 1 - 0.62 / height), ncol=2, frameon=False, fontsize=8.5, labelcolor=INK, handlelength=1.2)
    # Room on the right for the labels.
    ax.set_xlim(0, max(i[0] for i in items) * 1.42)
    # One row per map.
    ax.set_ylim(-0.6, len(items) - 0.4)
    # "Seed 3" etc. written beside each row.
    ax.set_yticks(range(len(items)), [f"Seed {i[1]}" for i in items], color=INK)
    # Numbers on the x axis written as "5 %".
    ax.xaxis.set_major_formatter(lambda v, _: f"{v:.0f} %")
    # For each map, at row y:
    for y, (extra, _, k0, k1) in enumerate(items):
        # Its bar, orange if the number of robots changed, rounded on the right.
        _bar(ax, 0, extra, y - 0.31, y + 0.31, FEASIBLE if k0 != k1 else BASELINE, "right")
        # If the number of robots changed, a note like "15 → 16 sinks".
        note = f"   {k0} \u2192 {k1} sinks" if k0 != k1 else ""
        # The value (and note) just after the bar's end.
        ax.text(extra, y, f"  +{extra:.1f} %{note}", va="center", fontsize=8.5, color=INK, zorder=4)
    # A thin line at 0.
    ax.axvline(0, color=BASELINE, lw=0.8, zorder=4)
    # Save the picture.
    fig.savefig(path, facecolor=SURFACE)
    # Close the page.
    plt.close(fig)


# Picture: one robot's visiting order, driven with each method, on its map.
def compare_same_tour(out, tour, path):
    """One tour of the default plan, on its map, with both methods using the same visiting order."""
    # The tour built each way.
    a, b = tour["shortest"], tour["feasible"]
    # Page size in inches.
    size = (7.5, 7.6)
    # The page, its title, and a subtitle with both lengths.
    fig, ax = _start(size, "The same visiting order, drawn with both methods",
                     f"Seed {tour['seed']}, sink {tour['sink']}, the tour that changes most: shortest {a['L']:.0f} m, feasible {b['L']:.0f} m "
                     f"(+{100 * (b['L'] / a['L'] - 1):.0f} %)", left=0.07, right=0.97, bottom=0.05, top=0.87)
    # The methods' legend.
    _method_legend(fig, 1 - 0.62 / size[1])
    # Obstacles and base.
    _map(ax, out)
    # Page-coloured background.
    ax.set_facecolor(SURFACE)
    # The four border lines ...
    for side in ax.spines:
        # ... hidden.
        ax.spines[side].set_visible(False)
    # No tick marks, small grey numbers.
    ax.tick_params(length=0, labelsize=8, colors=MUTED)
    # The shortest-method path: blue, solid.
    ax.plot(a["path"][:, 0], a["path"][:, 1], color=SHORTEST, lw=2, solid_joinstyle="round", zorder=4)
    # The rule's path: orange, dashed.
    ax.plot(b["path"][:, 0], b["path"][:, 1], color=FEASIBLE, lw=2, ls=DASHES, zorder=5)
    # The robot's sensors as black dots with a light rim.
    ax.scatter(*out["scenario"]["xy"][a["order"]].T, s=36, color=INK, edgecolors=SURFACE, linewidths=1.2, zorder=6)
    # All points of both paths.
    both = np.vstack([a["path"], b["path"]])
    # Their smallest and largest x and y.
    low, high = both.min(axis=0), both.max(axis=0)
    # The middle, and half the size of a square holding them, plus 6 m.
    centre, half = (low + high) / 2, (high - low).max() / 2 + 6
    # Zoom to that square: across ...
    ax.set_xlim(centre[0] - half, centre[0] + half)
    # ... and up and down.
    ax.set_ylim(centre[1] - half, centre[1] + half)
    # Save the picture.
    fig.savefig(path, facecolor=SURFACE)
    # Close the page.
    plt.close(fig)


# Draws one bar made of three coloured pieces end to end (a "stacked" bar),
# one per kind of leg, at height y.
def _stack(ax, shares, y, thickness):
    """One horizontal stacked bar of the three kinds of leg, with 2 px gaps; a share is printed inside only if it fits."""
    # How much 3 pixels are in data values across ...
    a0, a1 = ax.transData.inverted().transform([(0, 0), (3, 3)])[:, 0]
    # ... the gap left between pieces.
    gap = abs(a1 - a0)
    # Where the next piece starts.
    pos = 0.0
    # The last kind that has any legs (its piece gets the rounded end).
    last = [k for k in KIND_COLOR if shares[k] > 0][-1]
    # For each kind:
    for k in KIND_COLOR:
        # Its share in %.
        share = 100 * shares[k]
        # None of this kind?
        if share <= 0:
            # Then skip it.
            continue
        # Where its piece ends.
        end = pos + share
        # Leave a gap before the next piece (not after the last).
        tip = end - (0 if k == last else gap)
        # But never let a piece shrink to nothing.
        tip = max(tip, pos + gap / 2)
        # Draw the piece.
        _bar(ax, pos, tip, y - thickness / 2, y + thickness / 2, KIND_COLOR[k], "right" if k == last else None)
        # Only if the piece is wide enough (9 % or more) ...
        if share >= 9:
            # ... write its share in the middle of it.
            ax.text((pos + tip) / 2, y, f"{share:.1f} %", ha="center", va="center", fontsize=8.5, color=KIND_INK[k], zorder=4)
        # The next piece starts here.
        pos = end


# Picture: the kinds of leg in each method's plan, as two stacked bars.
def compare_leg_types(rows, path):
    """How the rule's own turn type does on the legs of each final plan."""
    # The totals for each method, filled in below.
    totals = {}
    # For each method:
    for method in ("shortest", "pdf"):
        # The leg counts of all its maps.
        legs = [r["legs"] for r in rows if r["method"] == method]
        # All its legs together.
        n = sum(sum(x.values()) for x in legs)
        # The share of each kind, and the number of legs.
        totals[method] = ({k: sum(x[k] for x in legs) / n for k in KIND_COLOR}, n)
    # Page size in inches.
    size = (7.5, 3.2)
    # The page, title and subtitle.
    fig, ax = _start(size, "The heading search steers feasible plans to poses that suit the rule",
                     f"Legs where the rule's type exists but is longer fall from {100 * totals['shortest'][0]['longer']:.1f} % to {100 * totals['pdf'][0]['longer']:.1f} %",
                     left=0.2, right=0.92, bottom=0.17, top=0.66)
    # Plain drawing area.
    _quiet_axes(ax, "x")
    # The kinds' legend.
    _kind_legend(fig, 1 - 0.62 / size[1])
    # From 0 to 100 %.
    ax.set_xlim(0, 100)
    # Room for two bars.
    ax.set_ylim(-0.6, 1.6)
    # The names of the two bars, with their numbers of legs.
    ax.set_yticks([1, 0], [f"Shortest plan\n({totals['shortest'][1]} legs)", f"Feasible plan\n({totals['pdf'][1]} legs)"], color=INK)
    # Numbers on the x axis written as "50 %".
    ax.xaxis.set_major_formatter(lambda v, _: f"{v:.0f} %")
    # For each method, at row y:
    for y, method in ((1, "shortest"), (0, "pdf")):
        # Its stacked bar.
        _stack(ax, totals[method][0], y, 0.36)
        # The share of missing types, written just after the bar.
        ax.text(100.8, y, f"{100 * totals[method][0]['missing']:.1f} %", va="center", fontsize=8.5, color=INK, clip_on=False)
    # Save the picture.
    fig.savefig(path, facecolor=SURFACE)
    # Close the page.
    plt.close(fig)


# Picture: random legs by distance; two panels side by side.
def compare_by_distance(bands, R, path):
    """Random poses by distance: what the rule's type does, and the extra length it costs. Two measures, so two panels."""
    # Page size in inches.
    size = (9, 3.7)
    # The page with two drawing areas, a title and a subtitle.
    fig, (left, right) = _start(size, "The rule's cost falls mainly on short legs",
                                f"Random poses, R = {R:.0f} m. Not the planner's cost: the heading search avoids many of these poses.",
                                left=0.1, right=0.97, bottom=0.11, top=0.63, ncols=2)
    # The gap between the two panels.
    fig.subplots_adjust(wspace=0.14)
    # The kinds' legend.
    _kind_legend(fig, 1 - 0.62 / size[1])
    rows = list(range(len(bands)))[::-1]                      # the shortest legs at the top
    # The largest mean extra length, in %.
    top = max(b["extra"] for b in bands) * 100
    # For each panel, with its title and the end of its scale:
    for ax, title, limit in ((left, "Share of legs, by distance between the points", 100), (right, "Mean extra length of the feasible path", top * 1.2)):
        # Plain drawing area.
        _quiet_axes(ax, "x")
        # Its title, top left.
        ax.set_title(title, loc="left", fontsize=9.5, fontweight="bold", color=INK, pad=6)
        # Its scale.
        ax.set_xlim(0, limit)
        # One row per distance band.
        ax.set_ylim(-0.6, len(bands) - 0.4)
        # Numbers written as "40 %".
        ax.xaxis.set_major_formatter(lambda v, _: f"{v:.0f} %")
        # A thin line at 0.
        ax.axvline(0, color=BASELINE, lw=0.8, zorder=4)
    # The distance bands ("0-10 m" ...) beside the left panel's rows.
    left.set_yticks(rows, [b["label"] for b in bands], color=INK)
    # No labels on the right panel's rows (the left one has them).
    right.set_yticks([])
    # For each band, at row y:
    for y, b in zip(rows, bands):
        # Left: its stacked bar of the three kinds.
        _stack(left, b, y, 0.56)
        # Its mean extra length in %.
        extra = 100 * b["extra"]
        # Right: a bar that long ...
        _bar(right, 0, extra, y - 0.28, y + 0.28, FEASIBLE, "right")
        # ... with its value after it.
        right.text(extra, y, f"  +{extra:.1f} %", va="center", fontsize=8.5, color=INK, zorder=4)
    # Save the picture.
    fig.savefig(path, facecolor=SURFACE)
    # Close the page.
    plt.close(fig)


# Saves a film as a GIF: calls update(0), update(1), ... to draw each
# picture on the page fig, then writes them all into one file.
def save_gif(fig, update, n_frames, path, fps):
    """Save update(0), ..., update(n_frames - 1) as a looping GIF and close the figure. All frames share
    one palette, taken from eight frames spread over the animation, so that a colour used by only a few
    pixels in some frame (a green dot, say) keeps its colour there. Frames are drawn one at a time, and
    equal neighbours are merged by Pillow into one longer frame."""
    # PIL ("Pillow") reads and writes picture files.
    from PIL import Image

    # Draws picture i and hands it back as an image.
    def render(i):
        # Let the caller draw picture i on the page.
        update(i)
        # Paint the page into memory.
        fig.canvas.draw()
        # Copy its pixels (red, green, blue; [..., :3] drops see-through-ness)
        # into an image.
        return Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[..., :3].copy())

    # Eight pictures spread evenly over the film.
    sample = [np.asarray(render(i)) for i in np.unique(np.linspace(0, n_frames - 1, 8).astype(int))]
    # A GIF may use only 256 colours. Choose 255 that suit those eight pictures
    # (stacked into one tall image) as the film's shared palette.
    palette = Image.fromarray(np.concatenate(sample, axis=0)).quantize(colors=255, method=Image.Quantize.MEDIANCUT)
    # A tiny recipe: turn a picture into the shared palette's colours, without
    # speckles (no "dither").
    quantize = lambda im: im.quantize(palette=palette, dither=Image.Dither.NONE)
    # The first picture.
    first = quantize(render(0))
    # Save it with all the others after it: each shown 1000 / fps milliseconds,
    # looping for ever (loop=0).
    first.save(path, save_all=True, append_images=(quantize(render(i)) for i in range(1, n_frames)),
               duration=round(1000 / fps), loop=0)
    # Close the page.
    plt.close(fig)


# ---------------------------------------------------------------------------
# How the genetic algorithm improves its tours (ga_evolution.py)
# ---------------------------------------------------------------------------

BEST, POP = COLORS[0], COLORS[1]              # blue: the best tour of a generation; orange: the whole population


# Numbers that sum up one GA run (gens: its generations, from ga_tour.optimize).
def ga_summary(gens, R):
    """Numbers for titles and the run table, from one GA run's history (ga_tour.optimize)."""
    # The best fitness of each generation (each is sorted best first).
    best = np.array([g["fitness"][0] for g in gens])
    # The generations in which the best got better.
    improved = [i for i in range(1, len(best)) if best[i] < best[i - 1] - 1e-9]
    # Hand back a dictionary: best, mean and worst fitness per generation; the
    # best tour's length and peaks; the mean peaks; how many different tours;
    # the gain in % from first to last; the improving generations; and the
    # penalty per peak, pi * R.
    return {"best": best, "mean": np.array([g["fitness"].mean() for g in gens]),
            "worst": np.array([g["fitness"][-1] for g in gens]),
            "length": np.array([g["length"][0] for g in gens]), "peaks": np.array([g["peaks"][0] for g in gens]),
            "mean_peaks": np.array([g["peaks"].mean() for g in gens]),
            "distinct": np.array([len({tuple(t) for t in g["orders"]}) for g in gens]),
            "gain": 100 * (1 - best[-1] / best[0]), "improved": improved, "penalty": np.pi * R}


# x and y values for a staircase line: each generation is one step wide.
def _steps(values):
    """x and y for a steps-post line in which every generation, the last one included, has a step of width 1."""
    # x: 0, 1, ..., n; y: the values, with the last one repeated to finish the last step.
    return np.arange(len(values) + 1), np.append(values, values[-1])


# Sets up the x axis as generations 0 to n_gens.
def _generation_axis(ax, n_gens):
    # The range.
    ax.set_xlim(0, n_gens)
    # The label.
    ax.set_xlabel("Generation", fontsize=9, color=MUTED)


# Picture: how the GA's fitness changes, generation by generation.
def ga_convergence(gens, R, run_label, path):
    """The objective (fitness) of the best tour, the population mean and the population's range, per generation."""
    # The run's summary numbers.
    s = ga_summary(gens, R)
    # In how many generations the best got better.
    n_imp = len(s["improved"])
    # The title: how much it improved, or that it never did.
    title = (f"The best tour improves by {s['gain']:.1f} % over {len(gens) - 1} generations" if s["gain"] > 0
             else "The first population already holds the best tour this run finds")
    # Page size in inches.
    size = (9, 4.6)
    # The page, the title, and a subtitle explaining the chart.
    fig, ax = _start(size, title,
                     f"{run_label}.\nLower is better. The GA keeps the better half of each generation, so the best never "
                     f"gets worse; it improved in {n_imp} of {len(gens) - 1}.",
                     left=0.1, right=0.97, bottom=0.13, top=0.74)
    # Plain drawing area with horizontal grid lines.
    _quiet_axes(ax, "y")
    # Staircase values of the worst fitness.
    x, worst = _steps(s["worst"])
    # A faint orange band from the best to the worst tour of each generation.
    ax.fill_between(x, _steps(s["best"])[1], worst, step="post", color=POP, alpha=0.10, lw=0,
                    label="Population: best to worst")
    # The population's average: orange, dashed, as a staircase.
    ax.plot(*_steps(s["mean"]), color=POP, lw=2, ls=DASHES, drawstyle="steps-post", label="Population mean")
    # The best tour: blue, solid, as a staircase.
    ax.plot(*_steps(s["best"]), color=BEST, lw=2, drawstyle="steps-post", label="Best tour")
    # A blue dot wherever the best got better.
    ax.plot(s["improved"], s["best"][s["improved"]], ls="", marker="o", ms=5.5, color=BEST, mec=SURFACE, mew=1.5,
            zorder=5, label="Generation where the best improved")
    # Generations along the bottom.
    _generation_axis(ax, len(gens))
    # The label of the up-down axis.
    ax.set_ylabel("Fitness = length + m(π)·π·R  [m]", fontsize=9, color=MUTED)
    # Write the first best value at the start ...
    ax.annotate(f"{s['best'][0]:.0f} m", (0, s["best"][0]), xytext=(2, 5), textcoords="offset points",
                ha="left", va="bottom", fontsize=8.5, color=INK)
    # ... and the last one at the end.
    ax.annotate(f"{s['best'][-1]:.0f} m", (len(gens), s["best"][-1]), xytext=(0, -6), textcoords="offset points",
                ha="right", va="top", fontsize=8.5, color=INK)
    # The legend, in one row above the chart.
    ax.legend(loc="upper left", bbox_to_anchor=(0.0, 1.13), ncol=4, frameon=False, fontsize=8.5, labelcolor=INK,
              handlelength=2.4, columnspacing=1.4)
    # Save the picture.
    fig.savefig(path, facecolor=SURFACE)
    # Close the page.
    plt.close(fig)


# Picture: the best tour's fitness split into its two parts, and its peaks.
def ga_fitness_split(gens, R, run_label, path):
    """The best tour's fitness split into straight-line length and peak penalty, and its number of peaks m(pi)."""
    # The run's summary numbers.
    s = ga_summary(gens, R)
    # Peaks of the best tour at the start and at the end.
    m0, m1 = int(s["peaks"][0]), int(s["peaks"][-1])
    # How many metres shorter it got.
    saved = s["length"][0] - s["length"][-1]
    # More peaks at the end than at the start?
    if m1 > m0:
        # Then the title says so.
        title = f"The GA trades radial unimodality for length: {m0} peak to {m1}, {saved:.0f} m shorter"
    # Otherwise, if it got shorter:
    elif saved > 0:
        # Say how much shorter ("peak" or "peaks").
        title = f"The best tour gets {saved:.0f} m shorter and keeps {m1} peak" + ("s" if m1 != 1 else "")
    # Otherwise nothing changed:
    else:
        # Say so.
        title = "The best tour does not change in this run"
    # Page size in inches.
    size = (9, 6.2)
    # The page with two drawing areas, one above the other, sharing the x axis;
    # the top one is taller (height ratio 2 to 1.15).
    fig, (top, bottom) = _start(size, title,
                                f"{run_label}.\nEach peak (the tour turns back towards the base) costs π·R = "
                                f"{np.pi * R:.1f} m. Generation 0 has m(π) = 1 (Section 0.8.2).",
                                left=0.1, right=0.97, bottom=0.09, top=0.8, nrows=2, sharex=True,
                                gridspec_kw={"height_ratios": [2, 1.15], "hspace": 0.32})
    # Both drawing areas ...
    for ax in (top, bottom):
        # ... made plain, with horizontal grid lines.
        _quiet_axes(ax, "y")
    # The top chart's title.
    top.set_title("Fitness of the best tour = straight-line length + peak penalty", loc="left", fontsize=9.5,
                  fontweight="bold", color=INK, pad=6)
    # Staircase values of the best tour's straight-line length.
    x, length = _steps(s["length"])
    # Shade between the length and the fitness: that gap is the peak penalty.
    top.fill_between(x, length, _steps(s["best"])[1], step="post", color=BEST, alpha=0.18, lw=0,
                     label="Peak penalty m(π)·π·R")
    # The fitness: solid.
    top.plot(*_steps(s["best"]), color=BEST, lw=2, drawstyle="steps-post", label="Fitness")
    # The length: dashed.
    top.plot(x, length, color=BEST, lw=2, ls=DASHES, drawstyle="steps-post", label="Straight-line length")
    # A little room above and below.
    top.margins(y=0.08)
    # The label of the up-down axis.
    top.set_ylabel("Metres", fontsize=9, color=MUTED)
    # The top chart's legend.
    top.legend(loc="upper right", ncol=3, frameon=False, fontsize=8.5, labelcolor=INK, handlelength=2.4)
    # The bottom chart's title.
    bottom.set_title("Number of peaks m(π)", loc="left", fontsize=9.5, fontweight="bold", color=INK, pad=6)
    # The population's average number of peaks: orange, dashed.
    bottom.plot(*_steps(s["mean_peaks"]), color=POP, lw=2, ls=DASHES, drawstyle="steps-post", label="Population mean")
    # The best tour's peaks: blue, solid.
    bottom.plot(*_steps(s["peaks"]), color=BEST, lw=2, drawstyle="steps-post", label="Best tour")
    # From 0 to a little above the highest value.
    bottom.set_ylim(0, max(s["mean_peaks"].max(), s["peaks"].max()) * 1.15 + 0.5)
    # Only whole numbers on its up-down axis.
    bottom.yaxis.set_major_locator(MaxNLocator(integer=True))
    # The bottom chart's legend.
    bottom.legend(loc="upper left", ncol=2, frameon=False, fontsize=8.5, labelcolor=INK, handlelength=2.4)
    # Generations along the bottom.
    _generation_axis(bottom, len(gens))
    # Save the picture.
    fig.savefig(path, facecolor=SURFACE)
    # Close the page.
    plt.close(fig)


# Film: the whole population of tours on the map, generation by generation.
# hold: how many extra times the last picture is shown, so it stays on screen.
def ga_animation(out, run, run_label, path, fps=8, hold=16, dpi=80):
    """GIF: every tour of the population (thin grey) and the best tour (blue) on the map, generation by
    generation, with the fitness and the number of peaks drawn alongside. Tours are drawn as the GA scores
    them: straight lines between the sensors, through obstacles if need be."""
    # Short names for the map and the settings.
    scen, p = out["scenario"], out["params"]
    # The run's generations, and its sensors.
    gens, ids = run["generations"], np.asarray(run["ids"])
    # The run's summary numbers.
    s = ga_summary(gens, p["R"])
    # Sensor positions and the base.
    xy, base = scen["xy"], scen["base"]
    # How many generations (pictures).
    n = len(gens)
    # An 11 by 6.6 inch page.
    fig = plt.figure(figsize=(11, 6.6), dpi=dpi, facecolor=SURFACE)
    # The title.
    fig.text(0.015, 0.975, "How the genetic algorithm improves its tours", fontsize=12, fontweight="bold", color=INK, va="top")
    # The subtitle.
    fig.text(0.015, 0.935, f"{run_label}. Tours are drawn as the GA scores them: straight lines between sensors.",
             fontsize=9, color=QUIET, va="top")
    # The map's drawing area: left edge, bottom edge, width, height, as
    # fractions of the page.
    ax = fig.add_axes([0.015, 0.1, 0.5, 0.74])
    # Obstacles and base.
    _map(ax, out)
    # Page-coloured background.
    ax.set_facecolor(SURFACE)
    # No tick marks, small grey numbers.
    ax.tick_params(length=0, labelsize=7, colors=MUTED)
    # The border lines ...
    for side in ax.spines:
        # ... hidden.
        ax.spines[side].set_visible(False)
    # The sensors that are NOT in this run.
    others = np.setdiff1d(np.arange(len(xy)), ids)
    # If there are any ...
    if len(others):
        # ... draw them as small light-grey dots.
        ax.scatter(*xy[others].T, s=8, color="#c9c8c1", zorder=4)
    # This run's sensors: black dots.
    ax.scatter(*xy[ids].T, s=14, color=INK, zorder=6)
    # All sensors and the base.
    pts = np.vstack([xy, base[None, :]])
    # The smallest and largest x and y, with 4 m to spare.
    lo, hi = pts.min(0) - 4, pts.max(0) + 4
    # Show that range across ...
    ax.set_xlim(lo[0], hi[0])
    # ... and up and down.
    ax.set_ylim(lo[1], hi[1])
    # One faint grey line for every tour except the best, empty for now.
    pop_lines = [ax.plot([], [], color=QUIET, lw=0.7, alpha=0.08, zorder=2)[0] for _ in gens[0]["orders"][1:]]
    # The best tour's blue line, empty for now. (The "," after best_line takes
    # the one line out of the list that plot hands back.)
    best_line, = ax.plot([], [], color=BEST, lw=1.8, zorder=7)
    # A legend made by hand, under the map.
    ax.legend(handles=[Line2D([], [], color=BEST, lw=1.8, label="Best tour of the generation"),
                       Line2D([], [], color=QUIET, lw=0.9, alpha=0.5, label="The other tours of the population"),
                       Line2D([], [], ls="", marker="o", ms=4, color=INK, label="Sensors of this GA run"),
                       Line2D([], [], ls="", marker="^", ms=7, color=INK, label="Base station")],
              loc="upper left", bbox_to_anchor=(0.0, -0.005), ncol=2, frameon=False, fontsize=8, labelcolor=INK)

    # A drawing area for the fitness chart, top right ...
    top = fig.add_axes([0.6, 0.5, 0.37, 0.3])
    # ... and one for the peaks chart, bottom right.
    bottom = fig.add_axes([0.6, 0.12, 0.37, 0.25])
    # Both charts:
    for a in (top, bottom):
        # plain, with horizontal grid lines ...
        _quiet_axes(a, "y")
        # ... and generations 0 to n across.
        a.set_xlim(0, n)
    # Whole numbers only for peaks.
    bottom.yaxis.set_major_locator(MaxNLocator(integer=True))
    # The top chart's title.
    top.set_title("Fitness [m]", loc="left", fontsize=9.5, fontweight="bold", color=INK, pad=6)
    # The range of fitness values ...
    span = s["mean"].max() - s["best"].min()
    # ... shown with 8 % to spare.
    top.set_ylim(s["best"].min() - 0.08 * span, s["mean"].max() + 0.08 * span)
    # The bottom chart's title.
    bottom.set_title("Number of peaks m(π)", loc="left", fontsize=9.5, fontweight="bold", color=INK, pad=6)
    # From 0 to a little above the highest value.
    bottom.set_ylim(0, max(s["mean_peaks"].max(), s["peaks"].max()) * 1.15 + 0.5)
    # The label along the bottom.
    bottom.set_xlabel("Generation", fontsize=9, color=MUTED)
    # Empty lines and a dot, filled in by the film: the average fitness ...
    mean_f, = top.plot([], [], color=POP, lw=2, ls=DASHES, drawstyle="steps-post", label="Population mean")
    # ... the best fitness ...
    best_f, = top.plot([], [], color=BEST, lw=2, drawstyle="steps-post", label="Best tour")
    # ... a dot on the current generation ...
    dot_f, = top.plot([], [], ls="", marker="o", ms=6, color=BEST, mec=SURFACE, mew=1.5, zorder=5)
    # ... the average peaks ...
    mean_m, = bottom.plot([], [], color=POP, lw=2, ls=DASHES, drawstyle="steps-post")
    # ... and the best tour's peaks.
    best_m, = bottom.plot([], [], color=BEST, lw=2, drawstyle="steps-post")
    # The top chart's legend.
    top.legend(loc="upper right", frameon=False, fontsize=8, labelcolor=INK, handlelength=2.4)
    # A line of text that the film fills in.
    label = fig.text(0.015, 0.89, "", fontsize=10, fontweight="bold", color=INK, va="top")

    # The x list and y list of a tour: base, sensors in order, base.
    def tour_xy(order):
        # Stack the points, then swap rows and columns (.T).
        return np.vstack([base, xy[order], base]).T

    # Draws picture number "frame" of the film.
    def update(frame):
        # The generation to show (the last one again during the hold).
        i = min(frame, n - 1)
        # That generation.
        gen = gens[i]
        # Each grey line takes one of its tours (all but the best) ...
        for line, order in zip(pop_lines, gen["orders"][1:]):
            # ... and draws it.
            line.set_data(*tour_xy(order))
        # The blue line draws the best tour.
        best_line.set_data(*tour_xy(gen["orders"][0]))
        # The charts grow up to generation i: average fitness ...
        mean_f.set_data(*_steps(s["mean"][:i + 1]))
        # ... best fitness ...
        best_f.set_data(*_steps(s["best"][:i + 1]))
        # ... the dot ...
        dot_f.set_data([i], [s["best"][i]])
        # ... average peaks ...
        mean_m.set_data(*_steps(s["mean_peaks"][:i + 1]))
        # ... and best peaks.
        best_m.set_data(*_steps(s["peaks"][:i + 1]))
        # The text: generation, best fitness split into its parts, and how many
        # different tours there are.
        label.set_text(f"Generation {i} of {n - 1}:  best fitness {s['best'][i]:.0f} m = {s['length'][i]:.0f} m"
                       f" + {int(s['peaks'][i])} × {np.pi * p['R']:.1f} m,  {s['distinct'][i]} different tours")
        # Hand back everything that changed.
        return pop_lines + [best_line, mean_f, best_f, dot_f, mean_m, best_m, label]

    # Make and save the film: one picture per generation, plus the hold.
    save_gif(fig, update, n + hold, path, fps)


# Film: every tour as a dot (its length across, its peaks up), generation
# by generation.
def ga_population_animation(gens, R, run_label, path, fps=8, hold=16, dpi=80):
    """GIF: every tour of the population as a dot, straight-line length across and number of peaks m(pi)
    up, one frame per generation. Grey diagonals are lines of equal fitness L + m(pi)*pi*R: one more
    peak is worth pi*R metres of length, so the cloud must move left (or down) to improve."""
    # The run's summary numbers.
    s = ga_summary(gens, R)
    # How many generations.
    n = len(gens)
    # The penalty per peak, pi * R.
    pen = np.pi * R
    # Every length of every tour of every generation ...
    L_all = np.concatenate([g["length"] for g in gens])
    # ... and every number of peaks.
    m_all = np.concatenate([g["peaks"] for g in gens])
    # The smallest and largest length ...
    x0, x1 = L_all.min(), L_all.max()
    # ... with 4 % to spare on each side.
    x0, x1 = x0 - 0.04 * (x1 - x0), x1 + 0.04 * (x1 - x0)
    # The top of the peaks axis.
    y1 = m_all.max() + 1
    # A 9 by 6 inch page.
    fig = plt.figure(figsize=(9, 6), dpi=dpi, facecolor=SURFACE)
    # The title.
    fig.text(0.02, 0.975, "The population moves towards lower fitness", fontsize=12, fontweight="bold", color=INK, va="top")
    # The subtitle, explaining the chart.
    fig.text(0.02, 0.93, f"{run_label}. Each dot is one of the {len(gens[0]['orders'])} tours, spread a little up and "
             f"down within each m(π).\nGrey diagonals join tours of equal fitness L + m(π)·π·R: one more peak is "
             f"worth π·R = {pen:.1f} m of length.", fontsize=9, color=QUIET, va="top")
    # The chart's drawing area.
    ax = fig.add_axes([0.09, 0.1, 0.88, 0.66])
    # Plain, with no grid.
    _quiet_axes(ax, None)
    # Lengths across.
    ax.set_xlim(x0, x1)
    # Peaks up.
    ax.set_ylim(-0.5, y1)
    # Whole numbers only.
    ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    # The label along the bottom.
    ax.set_xlabel("Straight-line length of the tour [m]", fontsize=9, color=MUTED)
    # The label up the side.
    ax.set_ylabel("Number of peaks m(π)", fontsize=9, color=MUTED)
    # lines of equal fitness F: m = (F - L) / (pi R)
    F0, F1 = x0, x1 + y1 * pen
    # A tidy spacing between the grey lines: a power of ten (1, 10, 100 ...)
    # near one sixth of the range ...
    step = 10 ** np.floor(np.log10((F1 - F0) / 6))
    # ... doubled if that would give more than 12 lines.
    step *= 2 if (F1 - F0) / step > 12 else 1
    # For each fitness value F on that spacing:
    for F in np.arange(np.ceil(F0 / step) * step, F1, step):
        # A grey line through all (length, peaks) with fitness F: length + peaks * pen = F.
        ax.plot([F + 0.5 * pen, F - y1 * pen], [-0.5, y1], color=GRID, lw=0.8, zorder=1)
    # The line of equal fitness through the best tour (filled in by the film).
    best_iso, = ax.plot([], [], color=BEST, lw=1.2, ls=DASHES, zorder=2, label="Equal fitness to the best tour")
    # Its label.
    iso_label = ax.text(0, 0, "", fontsize=8, color=INK, ha="left", va="center")
    # The path the best tour has taken so far.
    trail, = ax.plot([], [], color=BEST, lw=1.2, alpha=0.6, zorder=3, label="Path of the best tour so far")
    # Small up-and-down shifts (-0.21 to +0.21), so dots with the same number
    # of peaks do not hide each other.
    jitter = (np.arange(len(gens[0]["orders"])) % 7 - 3) * 0.07
    # The population's dots (orange), filled in by the film.
    dots = ax.scatter([], [], s=22, color=POP, alpha=0.75, lw=0, zorder=4, label="A tour of the population")
    # The best tour's bigger blue dot.
    best = ax.scatter([], [], s=70, color=BEST, edgecolor=SURFACE, lw=1.8, zorder=5, label="Best tour")
    # The legend.
    ax.legend(loc="upper right", frameon=False, fontsize=8, labelcolor=INK)
    # A line of text that the film fills in.
    label = fig.text(0.09, 0.8, "", fontsize=10, fontweight="bold", color=INK, va="bottom")

    # Draws picture number "frame".
    def update(frame):
        # The generation to show.
        i = min(frame, n - 1)
        # That generation.
        g = gens[i]
        # Move the orange dots to (length, peaks + shift) of every tour but the best.
        dots.set_offsets(np.column_stack([g["length"][1:], g["peaks"][1:] + jitter[1:]]))
        # Move the blue dot to the best tour.
        best.set_offsets([[g["length"][0], g["peaks"][0]]])
        # The best fitness.
        F = s["best"][i]
        # The line of equal fitness through it.
        best_iso.set_data([F + 0.5 * pen, F - y1 * pen], [-0.5, y1])
        # Its label, near the top of that line ...
        iso_label.set_position((F - (y1 - 0.7) * pen + 0.012 * (x1 - x0), y1 - 0.7))
        # ... saying the fitness.
        iso_label.set_text(f"fitness {F:.0f} m")
        # The trail of the best tour up to now.
        trail.set_data(s["length"][:i + 1], s["peaks"][:i + 1])
        # The text line.
        label.set_text(f"Generation {i} of {n - 1}:  best fitness {F:.0f} m = {s['length'][i]:.0f} m + "
                       f"{int(s['peaks'][i])} × {pen:.1f} m,  {s['distinct'][i]} different tours")
        # Hand back everything that changed.
        return [dots, best, best_iso, iso_label, trail, label]

    # Make and save the film.
    save_gif(fig, update, n + hold, path, fps)


# Film: distance from the base along the best tour, generation by generation.
def ga_radial_animation(gens, r, R, run_label, path, fps=8, hold=16, dpi=80):
    """GIF: distance from the base (geodesic, eq. 9) along the tour, sensor by sensor, one frame per
    generation. Generation 0 is radially unimodal (one hill, Section 0.8.2); every extra hill is a peak
    that costs pi*R in the fitness."""
    # The run's summary numbers.
    s = ga_summary(gens, R)
    # How many generations.
    n = len(gens)
    # How many sensors a tour has.
    size = len(gens[0]["orders"][0])
    # Positions 0, 1, ..., size + 1 (the base at both ends).
    x = np.arange(size + 2)
    # A tiny recipe: the distances from the base along a tour, 0 at both ends.
    profile = lambda order: np.concatenate([[0.0], r[order], [0.0]])
    # The largest distance anywhere, plus 12 %: the top of the chart.
    top = max(r[o].max() for g in gens for o in g["orders"]) * 1.12
    # A 9 by 5.4 inch page.
    fig = plt.figure(figsize=(9, 5.4), dpi=dpi, facecolor=SURFACE)
    # The title.
    fig.text(0.02, 0.975, "Radial unimodality, generation by generation", fontsize=12, fontweight="bold", color=INK, va="top")
    # The subtitle.
    fig.text(0.02, 0.925, f"{run_label}. Distance from the base of each sensor, in the order the tour visits them.\n"
             f"Generation 0 has one hill (Section 0.8.2); each extra hill is a peak that costs π·R = "
             f"{np.pi * R:.1f} m in the fitness.", fontsize=9, color=QUIET, va="top")
    # The chart's drawing area.
    ax = fig.add_axes([0.09, 0.12, 0.88, 0.62])
    # Plain, with horizontal grid lines.
    _quiet_axes(ax, "y")
    # Positions in the tour across ...
    ax.set_xlim(0, size + 1)
    # ... distances up.
    ax.set_ylim(0, top)
    # Whole numbers only across.
    ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    # The label along the bottom.
    ax.set_xlabel("Position in the tour (0 and the last are the base)", fontsize=9, color=MUTED)
    # The label up the side.
    ax.set_ylabel("Distance from the base r [m]", fontsize=9, color=MUTED)
    # A faint line for every other tour, empty for now.
    pop_lines = [ax.plot([], [], color=QUIET, lw=0.7, alpha=0.1, zorder=2)[0] for _ in gens[0]["orders"][1:]]
    # The best tour's line, with a dot at each sensor.
    best_line, = ax.plot([], [], color=BEST, lw=2, marker="o", ms=4, zorder=4, label="Best tour")
    # Little "v" marks above the peaks.
    peaks_mark = ax.scatter([], [], s=60, marker="v", color=INK, zorder=5, label="Peak (radial maximum)")
    # The legend, in one row above the chart.
    ax.legend(handles=[best_line, peaks_mark, Line2D([], [], color=QUIET, lw=0.9, alpha=0.5, label="Other tours of the population")],
              loc="lower left", bbox_to_anchor=(0.0, 1.0), ncol=3, frameon=False, fontsize=8, labelcolor=INK)
    # A line of text that the film fills in.
    label = fig.text(0.09, 0.81, "", fontsize=10, fontweight="bold", color=INK, va="bottom")

    # Draws picture number "frame".
    def update(frame):
        # The generation to show.
        i = min(frame, n - 1)
        # That generation.
        g = gens[i]
        # Each faint line ...
        for line, order in zip(pop_lines, g["orders"][1:]):
            # ... shows one other tour's distances.
            line.set_data(x, profile(order))
        # The best tour's distances.
        d = profile(g["orders"][0])
        # Show them.
        best_line.set_data(x, d)
        at = np.flatnonzero((d[1:-1] > d[:-2]) & (d[1:-1] > d[2:])) + 1      # the radial maxima counted by ga_tour.peaks
        # Put a mark just above each peak (or none at all, as an empty 0 x 2 array).
        peaks_mark.set_offsets(np.column_stack([x[at], d[at] + 0.03 * top]) if len(at) else np.zeros((0, 2)))
        # The text: generation, number of peaks, fitness.
        label.set_text(f"Generation {i} of {n - 1}:  the best tour has m(π) = {int(s['peaks'][i])} "
                       f"peak{'s' if s['peaks'][i] != 1 else ''},  fitness {s['best'][i]:.0f} m")
        # Hand back everything that changed.
        return pop_lines + [best_line, peaks_mark, label]

    # Make and save the film.
    save_gif(fig, update, n + hold, path, fps)

# Finds where a dense path passes each stop: the position in the list of
# path points, for each stop in turn.
def stop_indices(path, stops, start=0):
    """Indices in the polyline `path` where it passes the stops, in order (base, sensors, base)."""
    # idx: the answers; i: where to start looking.
    idx, i = [], start
    # For each stop, in order:
    for stop in stops:
        # The distance from every later path point to it.
        d = np.hypot(*(path[i:] - stop).T)
        # The points that are exactly on it (within 0.000001 m).
        hit = np.flatnonzero(d < 1e-6)
        # The first such point; if there is none, the nearest point.
        i = i + (int(hit[0]) if len(hit) else int(np.argmin(d)))
        # Note it, and look for the next stop from there on.
        idx.append(i)
    # Hand back the positions.
    return idx


# Film on the map: how the best tour changes from one peak (generation 0)
# to its final shape. tours: optional drivable (Dubins) tours to draw instead
# of straight lines.
def ga_unimodal_animation(out, run, r, run_label, path, fps=6, dpi=80, tours=None):
    """GIF on the map: how the best tour of one GA run changes from radially unimodal (generation 0)
    to its final shape. Each leg is an arrow, blue when it moves away from the base (the distance r of
    eq. 9 grows) and orange when it comes back (r shrinks). A ringed sensor is a peak: there the tour
    turns back towards the base. Generation 0 and the last generation are traced leg by leg; in between,
    every generation that improved the best tour is shown. Tours are straight lines, as the GA scores them.

    tours: optional {generation: trajectory.build result} for every generation drawn (0, the improving
    ones and the last). Then each leg is drawn along the Dubins path the sink would drive around the
    obstacles, and the tiles and captions add its length and mission time."""
    # Circle: a round shape, for the distance rings.
    from matplotlib.patches import Circle
    # Short names for the map and the settings.
    scen, p = out["scenario"], out["params"]
    # Sensor positions and the base.
    xy, base = scen["xy"], scen["base"]
    # The run's generations and its sensors.
    gens, ids = run["generations"], np.asarray(run["ids"])
    # The run's summary numbers.
    s = ga_summary(gens, p["R"])
    # The penalty per peak.
    pen = np.pi * p["R"]
    # Blue for legs going out, orange for legs coming back.
    OUT, IN = COLORS[0], COLORS[1]
    # The last generation's number.
    last = len(gens) - 1

    # Everything about one tour's shape.
    def shape(order):
        # Its stops: base, sensors, base.
        stops = np.vstack([base, xy[order], base])
        # Their distances from the base.
        d = np.concatenate([[0.0], r[order], [0.0]])
        up = d[1:] > d[:-1]                                                # per leg: moving away from the base?
        peaks = np.flatnonzero((d[1:-1] > d[:-2]) & (d[1:-1] > d[2:])) + 1  # as counted by ga_tour.peaks
        # Hand back the stops, distances, out-or-back of each leg, and the peaks.
        return stops, d, up, peaks

    # The best tour of the first and of the last generation.
    first, final = gens[0]["orders"][0], gens[last]["orders"][0]
    # A tour with n sensors has n + 1 legs.
    n_legs = len(first) + 1
    # Generation 0 and every generation that improved.
    steps = [0] + list(s["improved"])
    # With drivable tours: a tiny recipe giving generation g's Dubins length.
    dub = (lambda g: tours[g]["L"]) if tours else None
    # How many improving steps made the Dubins path LONGER.
    ups = sum(dub(b) > dub(a) + 1e-9 for a, b in zip(steps[:-1], steps[1:])) if tours else 0
    # The caption for generation 0.
    unimodal = (f"Generation 0: radially unimodal. The tour only goes out (blue) until its farthest sensor, "
                f"turns back once (1 peak), then only comes back (orange).")
    # The film's script, one entry per picture: (kind, generation, legs drawn,
    # caption). It starts with 4 pictures of just the sensors.
    frames = [("intro", 0, 0, "The sensors of this group, in grey rings of distance from the base.")] * 4
    # Then generation 0, drawn one leg at a time:
    for k in range(1, n_legs + 1):
        # Is leg k going out?
        up = shape(first)[2][k - 1]
        # Two pictures for leg k, with a caption.
        frames += [("trace", 0, k, f"Generation 0, leg {k} of {n_legs}: "
                    + ("going out, away from the base" if up else "coming back towards the base"))] * 2
    # Then 10 pictures of the whole generation-0 tour.
    frames += [("show", 0, n_legs, unimodal)] * 10
    # Then each improving generation g (prev: the one before it):
    for prev, g in zip(steps[:-1], steps[1:]):
        # Its number of peaks.
        m = int(s["peaks"][g])
        # Its caption.
        text = (f"Generation {g}: the GA found a better tour with {m} peak{'s' if m != 1 else ''}, "
                f"{s['length'][0] - s['length'][g]:.0f} m shorter in straight lines than generation 0.")
        # With drivable tours ...
        if tours:
            # ... how its Dubins length changed ...
            change = dub(g) - dub(prev)
            # ... added to the caption.
            text += (f"\nIts Dubins path is {dub(g):.0f} m, {abs(change):.0f} m "
                     f"{'longer' if change > 0 else 'shorter'} than the previous best tour's.")
        # Five pictures of it.
        frames += [("show", g, n_legs, text)] * 5
    # Then the last generation, one leg at a time:
    for k in range(1, n_legs + 1):
        # Is leg k going out?
        up = shape(final)[2][k - 1]
        # Two pictures for leg k.
        frames += [("trace", last, k, f"Generation {last}, leg {k} of {n_legs}: "
                    + ("going out, away from the base" if up else "coming back towards the base"))] * 2
    # The last tour's number of peaks.
    m1 = int(s["peaks"][last])
    # The closing caption, with drivable tours:
    if tours:
        # peaks, Dubins lengths and times before and after.
        compare = (f"Generation 0 (grey, dashed) against generation {last}: {int(s['peaks'][0])} peak to {m1}; Dubins path "
                   f"{dub(0):.0f} \u2192 {dub(last):.0f} m, T {tours[0]['T']:.0f} \u2192 {tours[last]['T']:.0f} s.\n"
                   f"The GA scores straight lines, so the Dubins path got longer in {ups} of its {len(steps) - 1} "
                   f"improving steps.")
    # Without them:
    else:
        # peaks, and how much shorter the straight lines got.
        compare = (f"Generation 0 (grey, dashed) against generation {last}: {int(s['peaks'][0])} peak to {m1}. "
                   f"Each peak costs {pen:.1f} m, but the tour is {s['length'][0] - s['length'][last]:.0f} m shorter.")
    # End with 20 pictures comparing the first and last tours.
    frames += [("compare", last, n_legs, compare)] * 20

    # An 11 by 6.8 inch page.
    fig = plt.figure(figsize=(11, 6.8), dpi=dpi, facecolor=SURFACE)
    # The title.
    fig.text(0.015, 0.975, "From a radially unimodal tour to the GA's best tour" + (", as Dubins paths" if tours else ""),
             fontsize=12, fontweight="bold", color=INK, va="top")
    # The subtitle (it depends on whether drivable tours are shown).
    fig.text(0.015, 0.935, f"{run_label}. " + ("Each tour is the Dubins path the sink would drive around the obstacles "
                                                "(trajectory.build); the GA itself scores straight lines." if tours else
                                                "Tours are drawn as the GA scores them: straight lines between sensors."),
             fontsize=9, color=QUIET, va="top")
    # The caption line, filled in by the film.
    caption = fig.text(0.015, 0.85, "", fontsize=10, fontweight="bold", color=INK, va="bottom")
    # The map's drawing area.
    ax = fig.add_axes([0.045, 0.03, 0.5, 0.79])
    # A small chart of distance from the base, bottom right.
    prof = fig.add_axes([0.62, 0.1, 0.35, 0.2])
    # The legend: out, back, peak, distance rings.
    fig.legend(handles=[Line2D([], [], color=OUT, lw=2.2, label="Moving away from the base (r grows)"),
                        Line2D([], [], color=IN, lw=2.2, label="Coming back towards the base (r shrinks)"),
                        Line2D([], [], ls="", marker="o", ms=11, mfc="none", mec=INK, mew=1.8,
                               label="Peak: the tour turns back towards the base"),
                        Line2D([], [], color=GRID, lw=1.2, label="Straight-line distance from the base, every 20 m")],
               loc="upper left", bbox_to_anchor=(0.6, 0.83), frameon=False, fontsize=8.5, labelcolor=INK)
    # The "tiles": big numbers shown on the right.
    tiles = []
    # Their names: four with drivable tours, three without. (The "\" at the
    # end of the next line continues it on the line after.)
    names = ("Peaks m(π)", "Straight lines", "Dubins path", "Mission time T") if tours else \
        ("Peaks m(π)", "Straight-line length", "Fitness")
    # The spacing between tiles.
    gap = 0.09 if tours else 0.12
    # For each tile:
    for i, name in enumerate(names):
        # its name in small grey ...
        fig.text(0.62 + gap * i, 0.55, name, fontsize=8, color=QUIET, va="bottom")
        # ... and below it an empty big number, filled in by the film.
        tiles.append(fig.text(0.62 + gap * i, 0.54, "", fontsize=13, fontweight="bold", color=INK, va="top"))
    # A note explaining the fitness.
    fig.text(0.62, 0.43, f"Fitness = straight-line length + m(π) × π·R, with π·R = {pen:.1f} m.\n"
             "r is measured around the obstacles (eq. 9).", fontsize=8.5, color=QUIET, va="top")
    # The sensors not in this run.
    others = np.setdiff1d(np.arange(len(xy)), ids)
    # Every point that must fit on the map (including the drivable paths).
    pts = np.vstack([xy, base[None, :]] + ([np.asarray(t["path"]) for t in tours.values()] if tours else []))
    # Their smallest and largest x and y, with 4 m to spare.
    lo, hi = pts.min(0) - 4, pts.max(0) + 4
    # The distance from the base to the furthest corner of the map view.
    reach = np.hypot(*(np.vstack([lo, hi, [lo[0], hi[1]], [hi[0], lo[1]]]) - base).T).max()
    # The top of the small chart.
    top = max(r[ids].max(), 1.0) * 1.15

    # A memory of already-cut Dubins paths, so each is cut only once.
    legs_cache = {}

    # Cuts generation g's drivable path into one piece per leg.
    def dubins_legs(g, stops):
        """The Dubins path of generation g's best tour, cut at the sensors into one piece per leg."""
        # Not cut yet?
        if g not in legs_cache:
            # Its dense path points.
            dense = np.asarray(tours[g]["path"])
            # Where the path passes each stop.
            idx = stop_indices(dense, stops)
            # The pieces between consecutive stops.
            legs_cache[g] = [dense[a:b + 1] for a, b in zip(idx[:-1], idx[1:])]
        # Hand back the pieces.
        return legs_cache[g]

    # Draws a curved piece with an arrowhead at its end.
    def arrow_along(seg, color):
        """A line along seg with an arrowhead at its end."""
        # The piece itself.
        ax.plot(seg[:, 0], seg[:, 1], color=color, lw=2.0, zorder=5, solid_capstyle="round")
        # Distances measured backwards from its end.
        back = np.cumsum(np.hypot(*np.diff(seg[::-1], axis=0).T))
        # The point about 2.5 m before the end.
        j = len(seg) - 2 - int(np.searchsorted(back, 2.5))
        # An arrowhead from there to the end.
        ax.annotate("", xy=tuple(seg[-1]), xytext=tuple(seg[max(j, 0)]), zorder=5,
                    arrowprops=dict(arrowstyle="-|>", color=color, lw=2.0, mutation_scale=11, shrinkA=0, shrinkB=0))

    # Draws the first k legs of a tour. faint: as a grey dashed ghost.
    def draw_tour(order, k, faint=False, g=None):
        # The tour's shape.
        stops, d, up, peaks = shape(order)
        # A drivable ghost:
        if tours and faint:
            # its dense path ...
            dense = np.asarray(tours[g]["path"])
            # ... drawn grey and dashed.
            ax.plot(dense[:, 0], dense[:, 1], color=QUIET, lw=1.2, ls=DASHES, alpha=0.6, zorder=3)
        # For each of the first k legs (none if the ghost was just drawn):
        for i in range(k if not (tours and faint) else 0):
            # Its start and end stops.
            a, b = stops[i], stops[i + 1]
            # With drivable tours:
            if tours:
                # the leg's curved piece ...
                seg = dubins_legs(g, stops)[i]
                # ... if it has any length ...
                if len(seg) >= 2 and np.hypot(*(seg[-1] - seg[0])) + np.hypot(*np.diff(seg, axis=0).T).sum() > 1e-9:
                    # ... drawn as an arrow, blue going out, orange coming back.
                    arrow_along(seg, OUT if up[i] else IN)
            # A straight-line ghost:
            elif faint:
                # a grey dashed line.
                ax.plot([a[0], b[0]], [a[1], b[1]], color=QUIET, lw=1.2, ls=DASHES, alpha=0.6, zorder=3)
            # Otherwise:
            else:
                # a straight arrow, blue or orange.
                ax.annotate("", xy=tuple(b), xytext=tuple(a), zorder=5,
                            arrowprops=dict(arrowstyle="-|>", color=OUT if up[i] else IN, lw=2.0, mutation_scale=11,
                                            shrinkA=3, shrinkB=4))
        # For a real (not ghost) tour:
        if not faint:
            # Number each sensor reached so far ...
            for i in range(1, min(k + 1, len(stops) - 1)):
                # ... just beside it.
                ax.text(stops[i][0] + 1.6, stops[i][1] + 1.6, str(i), fontsize=7, color=INK, zorder=7)
            # And ring each peak reached so far.
            for i in peaks[peaks <= k]:
                # A ring is a big hollow circle.
                ax.scatter(*stops[i], s=200, facecolor="none", edgecolor=INK, lw=1.8, zorder=6)
        # Hand back the distances, out-or-back, and peaks.
        return d, up, peaks

    # Draws picture number "frame", following the script.
    def update(frame):
        # This picture's entry in the script.
        kind, g, k, text = frames[min(frame, len(frames) - 1)]
        # Clear the map ("clear axes") ...
        ax.cla()
        # ... and draw it again: obstacles and base.
        _map(ax, out)
        # Page-coloured background.
        ax.set_facecolor(SURFACE)
        # No tick marks, small grey numbers.
        ax.tick_params(length=0, labelsize=7, colors=MUTED)
        # The border lines ...
        for side in ax.spines:
            # ... hidden.
            ax.spines[side].set_visible(False)
        # Rings every 20 m round the base:
        for rad in np.arange(20, reach, 20):
            # a light circle ...
            ax.add_patch(Circle(tuple(base), rad, fill=False, color=GRID, lw=1.0, zorder=1))
            # ... labelled with its distance, up and to the right (0.707 = cos 45°).
            ax.text(base[0] + rad * 0.707, base[1] + rad * 0.707, f"{rad:.0f} m", fontsize=6.5, color=MUTED, zorder=1)
        # The map's range across ...
        ax.set_xlim(lo[0], hi[0])
        # ... and up and down.
        ax.set_ylim(lo[1], hi[1])
        # Other sensors, if any ...
        if len(others):
            # ... as light-grey dots.
            ax.scatter(*xy[others].T, s=7, color="#d6d5cf", zorder=2)
        # This run's sensors in black.
        ax.scatter(*xy[ids].T, s=16, color=INK, zorder=4)
        # The best tour of generation g.
        order = gens[g]["orders"][0]
        # In the comparison pictures ...
        if kind == "compare":
            # ... draw generation 0's tour as a ghost.
            draw_tour(first, n_legs, faint=True, g=0)
        # Draw the tour's first k legs (just its shape in the intro).
        d, up, peaks = draw_tour(order, k, g=g) if kind != "intro" else shape(order)[1:]
        # The caption.
        caption.set_text(text)
        # The peaks reached so far.
        shown = peaks[peaks <= k] if kind != "intro" else []
        # Is the whole tour drawn?
        whole = kind not in ("intro", "trace") or k == n_legs
        # Tile 1: the peaks so far (a dash in the intro).
        tiles[0].set_text(f"{len(shown)}" if kind == "trace" else (f"{int(s['peaks'][g])}" if kind != "intro" else "—"))
        # Tile 2: the straight-line length, once the whole tour is drawn.
        tiles[1].set_text(f"{s['length'][g]:.0f} m" if whole else "—")
        # With drivable tours:
        if tours:
            # Tile 3: the Dubins length ...
            tiles[2].set_text(f"{tours[g]['L']:.0f} m" if whole else "—")
            # Tile 4: the mission time.
            tiles[3].set_text(f"{tours[g]['T']:.0f} s" if whole else "—")
        # Without:
        else:
            # Tile 3: the fitness.
            tiles[2].set_text(f"{s['best'][g]:.0f} m" if whole else "—")
        # the same tour as distance from the base, leg by leg, in the same colours
        prof.cla()
        # The small chart, cleared above: plain, with horizontal grid lines.
        _quiet_axes(prof, "y")
        # Visit numbers across ...
        prof.set_xlim(0, n_legs)
        # ... distances up.
        prof.set_ylim(0, top)
        # Its title.
        prof.set_title("The same tour: distance r from the base, sensor by sensor", loc="left", fontsize=8.5,
                       color=INK, pad=4)
        # Its label.
        prof.set_xlabel("Visit number (0 and the last are the base)", fontsize=8, color=MUTED)
        # Whole numbers only.
        prof.xaxis.set_major_locator(MaxNLocator(integer=True))
        # Unless it is the intro:
        if kind != "intro":
            # for each leg drawn so far ...
            for i in range(k):
                # ... a line from one visit's distance to the next, blue or orange;
                prof.plot([i, i + 1], [d[i], d[i + 1]], color=OUT if up[i] else IN, lw=2, zorder=3)
            # and for each peak reached ...
            for i in shown:
                # ... a ring.
                prof.scatter([i], [d[i]], s=70, facecolor="none", edgecolor=INK, lw=1.5, zorder=4)
        # Nothing to hand back: every picture is drawn afresh.
        return []

    # Make and save the film, one picture per script entry.
    save_gif(fig, update, len(frames), path, fps)
