"""One visual vocabulary for figures, animation frames, and rover geometry."""

import matplotlib
import numpy as np
from matplotlib.patches import Circle, Polygon

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

NAVY = "#081222"
PANEL = "#0d1c30"
TEXT = "#d9e9ff"
MUTED = "#8aa9c9"
CYAN = "#37d5ee"
AMBER = "#e6ac62"
REFERENCE = "#a9bfdb"
COLORS = {"open_loop": AMBER, "feedback": CYAN}
LABELS = {"open_loop": "Open loop", "feedback": "Feedback"}
STYLES = {"open_loop": "--", "feedback": "-"}
# Same purely illustrative geometry everywhere; the kinematics do not use these dimensions.
BODY_LENGTH_M = 0.48
BODY_WIDTH_M = 0.30
WHEEL_LENGTH_M = 0.22
WHEEL_WIDTH_M = 0.07
TRACK_M = 0.38


def apply_theme():
    plt.rcParams.update(
        {
            "figure.facecolor": NAVY,
            "axes.facecolor": PANEL,
            "savefig.facecolor": NAVY,
            "text.color": TEXT,
            "axes.labelcolor": TEXT,
            "axes.edgecolor": "#39516a",
            "xtick.color": MUTED,
            "ytick.color": MUTED,
            "grid.color": "#52708d",
            "grid.alpha": 0.20,
            "grid.linewidth": 0.6,
            "axes.grid": True,
            "font.family": "DejaVu Sans",
            "font.size": 11,
            "axes.titlesize": 13,
            "axes.titleweight": "bold",
            "axes.labelsize": 11,
            "lines.linewidth": 2,
            "legend.frameon": False,
            "legend.fontsize": 9,
            "figure.dpi": 100,
            "svg.fonttype": "none",
            "axes.spines.top": False,
            "axes.spines.right": False,
        }
    )


def heading(fig, title, subtitle):
    fig.suptitle(title, x=0.055, y=0.985, ha="left", fontsize=19, fontweight="bold")
    fig.text(0.055, 0.925, subtitle, color=MUTED, fontsize=10, ha="left")


def spatial(ax, bounds):
    ax.set(xlim=bounds[:2], ylim=bounds[2:], xlabel="World x [m]", ylabel="World y [m]")
    ax.set_aspect("equal", adjustable="box")


def rover(ax, state, color=CYAN, ghost=False, zorder=8):
    """Oriented body, two drive wheels, rear caster and a forward-pointing nose.

    Local x is forward. This is a kinematic icon, not a mechanical CAD model.
    Returns artists so an animation can update without accumulating patches.
    """
    x, y, _, theta = state
    rotation = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    origin = np.array([x, y])
    artists = []

    def polygon(points, fill=True, facecolor=None, linewidth=1.3):
        patch = Polygon(
            np.asarray(points) @ rotation.T + origin,
            closed=True,
            facecolor=(facecolor or color) if fill else "none",
            edgecolor=color,
            alpha=0.60 if ghost else 1,
            linewidth=linewidth,
            linestyle=":" if ghost else "-",
            zorder=zorder,
        )
        ax.add_patch(patch)
        artists.append(patch)

    half_length, half_width = BODY_LENGTH_M / 2, BODY_WIDTH_M / 2
    polygon(
        [
            [-half_length, -half_width],
            [half_length - 0.06, -half_width],
            [half_length, 0],
            [half_length - 0.06, half_width],
            [-half_length, half_width],
        ],
        fill=not ghost,
        facecolor=PANEL,
        linewidth=1.7,
    )
    for side in (-1, 1):
        cy = side * TRACK_M / 2
        wl, ww = WHEEL_LENGTH_M / 2, WHEEL_WIDTH_M / 2
        polygon([[-wl, cy - ww], [wl, cy - ww], [wl, cy + ww], [-wl, cy + ww]], fill=not ghost)
    polygon([[0.15, 0], [-0.02, 0.065], [-0.02, -0.065]], fill=not ghost)
    caster = Circle(
        np.array([-0.16, 0]) @ rotation.T + origin,
        0.026,
        facecolor="none",
        edgecolor=color,
        alpha=0.60 if ghost else 0.8,
        zorder=zorder,
    )
    ax.add_patch(caster)
    artists.append(caster)
    return artists
