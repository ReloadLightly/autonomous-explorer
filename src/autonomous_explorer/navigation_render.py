"""Milestone 02 gallery. Reads saved planning/execution evidence; never runs a planner."""

from pathlib import Path

import numpy as np
from matplotlib.patches import Circle, Rectangle

from .evidence import load_log, read_json, sha256, write_json
from .model import wrap_angle
from .navigation_evidence import verify_navigation
from .render import animation_frames, mark_disturbance, save_figure
from .theme import (
    AMBER,
    COLORS,
    CYAN,
    LABELS,
    MUTED,
    PANEL,
    REFERENCE,
    STYLES,
    TEXT,
    apply_theme,
    heading,
    plt,
    rover,
    spatial,
)

VIOLATION = "#ef8583"


def draw_world(ax, config, inflation=0.0, bounds=None):
    """Render given rectangular obstacles, not a sensed or reconstructed map."""
    world = config["world"]
    xmin, xmax, ymin, ymax = world["bounds_m"]
    if inflation:
        for a, b, c, d in world["obstacles_xyxy_m"]:
            ax.add_patch(
                Rectangle(
                    (a - inflation, b - inflation),
                    c - a + 2 * inflation,
                    d - b + 2 * inflation,
                    facecolor=AMBER,
                    edgecolor=AMBER,
                    alpha=0.13,
                    lw=1,
                    zorder=1,
                )
            )
        ax.add_patch(
            Rectangle(
                (xmin + inflation, ymin + inflation),
                xmax - xmin - 2 * inflation,
                ymax - ymin - 2 * inflation,
                facecolor="none",
                edgecolor=AMBER,
                linestyle=":",
                alpha=0.7,
                lw=1,
                zorder=2,
            )
        )
    for a, b, c, d in world["obstacles_xyxy_m"]:
        ax.add_patch(
            Rectangle((a, b), c - a, d - b, facecolor="#283d52", edgecolor=MUTED, lw=1, zorder=2)
        )
    ax.add_patch(
        Rectangle(
            (xmin, ymin),
            xmax - xmin,
            ymax - ymin,
            facecolor="none",
            edgecolor=MUTED,
            lw=1.3,
            zorder=3,
        )
    )
    start, goal = config["start_xy_m"], config["goal_xy_m"]
    ax.scatter(*start, s=36, marker="o", facecolors="none", edgecolors=TEXT, zorder=7)
    ax.scatter(*goal, s=55, marker="X", color=TEXT, zorder=7)
    spatial(ax, world["bounds_m"] if bounds is None else bounds)


def full_bounds(config, logs):
    b = config["world"]["bounds_m"]
    pts = np.concatenate([log["states"][:, :2] for log in logs.values()])
    return [
        min(b[0], pts[:, 0].min()) - 0.6,
        max(b[1], pts[:, 0].max()) + 0.6,
        min(b[2], pts[:, 1].min()) - 0.6,
        max(b[3], pts[:, 1].max()) + 0.6,
    ]


def plot_reference(ax, plan):
    state = plan["reference_states"]
    ax.plot(
        state[:, 0], state[:, 1], ":", color=REFERENCE, lw=1.7, label="Timed reference", zorder=4
    )


def execution_paths(ax, logs, k=None):
    for name, log in logs.items():
        state = log["states"]
        i = len(state) - 1 if k is None else min(k, len(state) - 1)
        ax.plot(
            state[: i + 1, 0],
            state[: i + 1, 1],
            STYLES[name],
            color=COLORS[name],
            lw=2,
            label=LABELS[name],
            zorder=5,
        )
        rover(ax, state[i], COLORS[name])


def static_navigation(root, output, config, summary, plan, logs, audits, outcomes):
    radius = summary["rover_enclosing_radius_m"]
    bias = next(s for s in config["scenarios"] if s["name"] == "yaw_bias")
    t, reference = plan["times"], plan["reference_states"]
    bounds = full_bounds(config, logs)
    # 01: actual known workspace and conservative configuration-space construction.
    fig, axs = plt.subplots(1, 2, figsize=(12, 6.5))
    fig.subplots_adjust(top=0.81, bottom=0.14, left=0.07, right=0.97, wspace=0.22)
    heading(
        fig,
        "01  /  Give the rover room",
        "Known static geometry • left: full rover envelope • right: point search in inflated obstacles",
    )
    draw_world(axs[0], config)
    start = config["start_xy_m"]
    rover(axs[0], reference[0])
    axs[0].add_patch(
        Circle(start, radius, facecolor="none", edgecolor=CYAN, linestyle="--", lw=1.5)
    )
    axs[0].annotate(
        f"Enclosing radius {radius:.3f} m",
        start,
        xytext=(15, 22),
        textcoords="offset points",
        fontsize=9,
        color=CYAN,
    )
    axs[0].set_title("Workspace: body + wheels")
    draw_world(axs[1], config, radius + config["search_margin_m"])
    axs[1].set_title("Search space: footprint + reserve")
    axs[1].text(
        0.03,
        0.95,
        f"Obstacle offset = {radius + config['search_margin_m']:.3f} m",
        transform=axs[1].transAxes,
        va="top",
        fontsize=10,
        color=AMBER,
        bbox={"facecolor": PANEL, "edgecolor": "none", "alpha": 0.9},
    )
    fig.text(
        0.07,
        0.04,
        "The disk and box offsets are conservative. Obstacles are supplied, not perceived or mapped.",
        color=MUTED,
        fontsize=10,
    )
    save_figure(fig, output, "01_configuration_space", True)

    # 02: both algorithms search the exact same graph; expansions are saved in order.
    fig, axs = plt.subplots(1, 2, figsize=(12, 6.5))
    fig.subplots_adjust(top=0.81, bottom=0.12, left=0.07, right=0.97, wspace=0.22)
    heading(
        fig,
        "02  /  A heuristic changes the search",
        "Identical grid, footprint, edge checks and costs • A* uses Euclidean distance-to-go; Dijkstra uses zero",
    )
    for ax, algorithm, color in zip(axs, ["astar", "dijkstra"], [CYAN, AMBER], strict=True):
        s = load_log(root / (algorithm + ".npz"))
        draw_world(ax, config, radius + config["search_margin_m"])
        ax.scatter(s["expanded"][:, 0], s["expanded"][:, 1], s=7, c=color, alpha=0.35, zorder=3)
        ax.plot(s["path"][:, 0], s["path"][:, 1], "-", color=color, lw=2, zorder=5)
        ax.set_title(
            f"{'A*' if algorithm == 'astar' else 'Dijkstra'} • {summary[algorithm]['expanded_nodes']} expansions"
        )
        ax.text(
            0.03,
            0.95,
            f"Graph cost {summary[algorithm]['graph_cost_m']:.3f} m",
            transform=ax.transAxes,
            va="top",
            fontsize=10,
        )
    save_figure(fig, output, "02_astar_vs_dijkstra", True)

    # 03: distinguish a geometric graph path from a physically trackable reference.
    fig, axs = plt.subplots(1, 3, figsize=(15, 5.8))
    fig.subplots_adjust(top=0.79, bottom=0.13, left=0.055, right=0.98, wspace=0.24)
    heading(
        fig,
        "03  /  A path is not yet a trajectory",
        "Checked grid edges → checked line-of-sight shortcuts → certified rounded path + timing",
    )
    grid = load_log(root / "astar.npz")["path"]
    for ax, p, title, color in zip(
        axs,
        [grid, plan["shortcut"], reference[:, :2]],
        [
            "83 grid nodes" if len(grid) == 83 else f"{len(grid)} grid nodes",
            f"{len(plan['shortcut'])} shortcut waypoints",
            "C1 timed reference",
        ],
        [AMBER, REFERENCE, CYAN],
        strict=True,
    ):
        draw_world(ax, config)
        ax.plot(p[:, 0], p[:, 1], "-", color=color, lw=2)
        if len(p) < 100:
            ax.scatter(p[:, 0], p[:, 1], s=9, color=color)
        ax.set_title(title, fontsize=12)
    save_figure(fig, output, "03_path_to_trajectory", True)

    # 04: derivative/command evidence, including continuous extrema from certificates.
    certificate = read_json(root / "reference_certificate.json")
    fig, axs = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    fig.subplots_adjust(top=0.83, bottom=0.09, left=0.12, right=0.97, hspace=0.28)
    heading(
        fig,
        "04  /  Timing makes the reference executable",
        f"Analytical segment bounds • duration {t[-1]:.2f} s • speed stays positive • acceleration may jump at joins",
    )
    nominal = load_log(root / "nominal__open_loop.npz")
    axs[0].plot(t, reference[:, 2], color=CYAN, label="Reference speed")
    axs[0].axhline(config["cruise_speed_mps"], color=MUTED, ls=":", label="Speed cap")
    axs[0].set_ylabel("Speed [m/s]")
    for j, ax in enumerate(axs[1:]):
        ax.stairs(nominal["requested"][:, j], nominal["times"], color=CYAN, lw=1.4)
        limit = (
            config["reference_max_acceleration_mps2"]
            if j == 0
            else config["reference_max_yaw_rate_radps"]
        )
        for sign in [-1, 1]:
            ax.axhline(sign * limit, color=AMBER, ls=":", lw=1)
        ax.set_ylabel(["a [m/s²]", "ω [rad/s]"][j])
    for ax in axs:
        for time in plan["segment_boundaries"][1:-1]:
            ax.axvline(time, color=MUTED, alpha=0.18, lw=0.6)
    axs[0].legend(loc="lower center", bbox_to_anchor=(0.5, 1.01), ncol=2, fontsize=9)
    axs[-1].set_xlabel("Simulation time [s]")
    fig.text(
        0.12,
        0.025,
        f"Continuous minimum speed: {certificate['bounds']['min_speed_mps']:.4f} m/s • bounds include between-sample extrema",
        color=MUTED,
        fontsize=9,
    )
    save_figure(fig, output, "04_reference_feasibility")

    # 05: unsafe executions remain visible, including leaving the map.
    fig, ax = plt.subplots(figsize=(9, 8))
    fig.subplots_adjust(top=0.84, bottom=0.10, left=0.12, right=0.96)
    heading(
        fig,
        "05  /  Safe reference, different executions",
        "Shared yaw-bias case • open-loop failures retained • the controller itself has no obstacle feedback",
    )
    draw_world(ax, config, bounds=bounds)
    plot_reference(ax, plan)
    execution_paths(ax, logs)
    unsafe = outcomes["yaw_bias__open_loop"]["first_disk_overlap_time_s"]
    if unsafe is not None:
        k = int(np.argmin(abs(t - unsafe)))
        point = logs["open_loop"]["states"][k, :2]
        ax.scatter(*point, marker="x", s=65, color=VIOLATION, zorder=10)
        ax.annotate(
            f"First envelope overlap\nt = {unsafe:.2f} s",
            point,
            xytext=(-170, -65),
            textcoords="offset points",
            color=VIOLATION,
            fontsize=9,
            bbox={"facecolor": PANEL, "edgecolor": "none", "alpha": 0.95},
            arrowprops={"arrowstyle": "-", "color": VIOLATION},
        )
    ax.legend(loc="upper left", fontsize=9)
    save_figure(fig, output, "05_navigation_execution", True)

    # 06: snapshots use the same saved arrays and fixed world coordinates.
    fig, axs = plt.subplots(1, 4, figsize=(16, 6))
    fig.subplots_adjust(top=0.79, bottom=0.16, left=0.055, right=0.98, wspace=0.24)
    heading(
        fig,
        "06  /  Planning does not end at execution",
        "Same recorded comparison • before, during, after the disturbance, and near the finite-horizon goal",
    )
    snapshots = [
        max(0, bias["start_s"] - 2),
        (bias["start_s"] + bias["end_s"]) / 2,
        bias["end_s"] + 2,
        t[-1] - 2,
    ]
    for ax, time in zip(axs, snapshots, strict=True):
        k = int(np.argmin(abs(t - time)))
        draw_world(ax, config, bounds=bounds)
        plot_reference(ax, plan)
        execution_paths(ax, logs, k)
        rover(ax, reference[k], REFERENCE, ghost=True)
        ax.set_title(f"t = {t[k]:.2f} s", fontsize=11)
        if ax != axs[0]:
            ax.set_ylabel("")
    fig.text(
        0.055,
        0.05,
        "Cyan solid: feedback • amber dashed: open loop • dotted: reference • ghost: desired pose",
        color=MUTED,
        fontsize=10,
    )
    save_figure(fig, output, "06_navigation_snapshots")

    # 07: a separate feedback zoom prevents its smaller errors disappearing on the common scale.
    fig, axs = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    fig.subplots_adjust(top=0.83, bottom=0.09, left=0.12, right=0.97, hspace=0.26)
    heading(
        fig,
        "07  /  Tracking the planned reference",
        "Yaw-bias case • common error scale, explicit feedback zoom, and wrapped heading error",
    )
    for name, log in logs.items():
        error = np.linalg.norm(log["states"][:, :2] - log["reference_states"][:, :2], axis=1)
        axs[0].plot(log["times"], error, STYLES[name], color=COLORS[name], label=LABELS[name])
        if name == "feedback":
            axs[1].plot(log["times"], error * 100, color=CYAN)
        axs[2].plot(
            log["times"],
            wrap_angle(log["states"][:, 3] - log["reference_states"][:, 3]),
            STYLES[name],
            color=COLORS[name],
        )
    for ax, label in zip(
        axs, ["Position error [m]", "Feedback only [cm]", "Heading error [rad]"], strict=True
    ):
        mark_disturbance(ax, bias)
        ax.set_ylabel(label)
    axs[0].legend(loc="upper left")
    axs[-1].set_xlabel("Simulation time [s]")
    save_figure(fig, output, "07_navigation_error")

    # 08: clearance is measured independently of tracking accuracy.
    fig, axs = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    fig.subplots_adjust(top=0.82, bottom=0.11, left=0.12, right=0.97, hspace=0.27)
    heading(
        fig,
        "08  /  Did the rover stay clear?",
        "Disk-envelope clearance • lower row zooms feedback • negative values are retained violations",
    )
    for name, audit in audits.items():
        axs[0].plot(
            t, audit["sampled_clearance_m"], STYLES[name], color=COLORS[name], label=LABELS[name]
        )
    fb = audits["feedback"]
    axs[1].plot(t, fb["sampled_clearance_m"], color=CYAN, label="Feedback sampled clearance")
    axs[1].plot(
        t[:-1],
        fb["interval_lower_bound_m"],
        ":",
        color=REFERENCE,
        lw=1.4,
        label="Interval travel bound",
    )
    for j, ax in enumerate(axs):
        if j == 0:
            ax.axhline(0, color=VIOLATION, ls="--", lw=1, label="Envelope contact")
        mark_disturbance(ax, bias)
        ax.set_ylabel("Clearance [m]")
        ax.legend(loc="lower left", fontsize=8, ncol=2)
    axs[1].legend(loc="upper left", fontsize=8, ncol=2)
    axs[1].set_ylim(bottom=min(0.35, float(fb["interval_lower_bound_m"].min()) - 0.05))
    axs[-1].set_xlabel("Simulation time [s]")
    save_figure(fig, output, "08_clearance_audit", True)

    # 09: same actuator limits, requested/applied separately retained.
    fig, axs = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    fig.subplots_adjust(top=0.82, bottom=0.11, left=0.12, right=0.97, hspace=0.28)
    heading(
        fig,
        "09  /  Limits apply to both controllers",
        "Applied commands: held lines • requested samples: hollow markers • dashed horizontal lines: actuator limits",
    )
    for name, log in logs.items():
        for j, ax in enumerate(axs):
            ax.stairs(
                log["applied"][:, j],
                log["times"],
                color=COLORS[name],
                ls=STYLES[name],
                label=LABELS[name],
                lw=1.4,
            )
            idx = np.arange(0, len(log["requested"]), 90)
            ax.scatter(
                log["times"][idx],
                log["requested"][idx, j],
                facecolors="none",
                edgecolors=COLORS[name],
                s=22,
                marker="o" if name == "feedback" else "s",
                zorder=4,
            )
    for j, ax in enumerate(axs):
        if config["actuator_limits"]:
            for sign in [-1, 1]:
                ax.axhline(sign * config["actuator_limits"][j], color=MUTED, ls="--", lw=0.9)
        ax.set_ylabel(["Acceleration [m/s²]", "Yaw rate [rad/s]"][j])
        mark_disturbance(ax, bias)
    axs[0].legend(ncol=2)
    axs[-1].set_xlabel("Simulation time [s]")
    save_figure(fig, output, "09_navigation_controls")

    # 10: failure cases are separate from the successful main world and are never executed.
    fig, axs = plt.subplots(1, 2, figsize=(12, 6.5))
    fig.subplots_adjust(top=0.81, bottom=0.16, left=0.07, right=0.97, wspace=0.25)
    heading(
        fig,
        "10  /  Sometimes the right answer is no path",
        "Failures retained • point-robot success is not a valid finite-rover plan • no motion is executed here",
    )
    for ax, case in zip(axs, summary["failure_cases"], strict=True):
        c = case["configuration"]
        draw_world(ax, c, radius + config["reference_margin_m"])
        s = load_log(root / case["point"]["log"])
        if len(s["path"]):
            ax.plot(
                s["path"][:, 0], s["path"][:, 1], "--", color=AMBER, lw=2, label="Point path only"
            )
            ax.legend(loc="lower right", fontsize=9)
        ax.set_title(case["name"].replace("_", " ").title())
        ax.text(
            0.03,
            0.97,
            "Finite rover: no grid path",
            color=AMBER,
            transform=ax.transAxes,
            va="top",
            fontsize=10,
        )
    fig.text(
        0.07,
        0.05,
        "Failure refers to this grid and conservative envelope, not a completeness proof for every physical rover motion.",
        color=MUTED,
        fontsize=9,
    )
    save_figure(fig, output, "10_retained_planning_failures", True)

    # 11: resolution changes the graph, and thus both work and the resulting graph cost.
    fig, axs = plt.subplots(1, 2, figsize=(12, 6))
    fig.subplots_adjust(top=0.80, bottom=0.17, left=0.08, right=0.97, wspace=0.30)
    heading(
        fig,
        "11  /  A finer grid costs more search",
        "Recorded resolution study • optimality is on each graph, not over every continuous rover trajectory",
    )
    study = summary["resolution_study"]
    resolution = [x["resolution_m"] for x in study]
    axs[0].plot(resolution, [x["graph_cost_m"] for x in study], "o-", color=CYAN)
    axs[0].set(ylabel="A* graph-path length [m]", xlabel="Grid spacing [m]")
    axs[1].plot(resolution, [x["expanded_nodes"] for x in study], "s--", color=AMBER)
    axs[1].set(ylabel="Expanded nodes", xlabel="Grid spacing [m]", yscale="log")
    for ax in axs:
        ax.set_xticks(resolution, [f"{r:g}" for r in resolution])
    save_figure(fig, output, "11_grid_resolution")


def navigation_animation(output, config, summary, plan, logs, audits):
    times, reference = plan["times"], plan["reference_states"]
    bias = next(s for s in config["scenarios"] if s["name"] == "yaw_bias")
    vis = config["visualization"]
    bounds = full_bounds(config, logs)
    fig, axs = plt.subplots(1, 2, figsize=(10.8, 6.6))
    fig.subplots_adjust(top=0.80, bottom=0.19, left=0.065, right=0.98, wspace=0.20)
    heading(
        fig,
        "AUTONOMOUS EXPLORER  /  02",
        "Obstacle-Aware Navigation • one validated reference, one shared disturbance, two different executions",
    )
    trails, bodies, labels = {}, [], {}
    for ax, name in zip(axs, ["open_loop", "feedback"], strict=True):
        draw_world(ax, config, bounds=bounds)
        plot_reference(ax, plan)
        (trails[name],) = ax.plot([], [], STYLES[name], color=COLORS[name], lw=2, zorder=5)
        ax.set_title(LABELS[name], color=COLORS[name])
        labels[name] = ax.text(
            0.02,
            0.97,
            "",
            transform=ax.transAxes,
            va="top",
            fontsize=9,
            bbox={"facecolor": PANEL, "edgecolor": "none", "alpha": 0.9},
            zorder=15,
        )
    clock = fig.text(0.065, 0.10, "", fontsize=11, family="DejaVu Sans Mono")
    signal = fig.text(0.55, 0.10, "", fontsize=10, color=AMBER)
    fig.text(
        0.065,
        0.045,
        "Known obstacles • dotted reference + desired ghost • trails reveal executed history",
        color=MUTED,
        fontsize=9,
    )

    def update(k):
        for artist in bodies:
            artist.remove()
        bodies.clear()
        for ax, name in zip(axs, ["open_loop", "feedback"], strict=True):
            state = logs[name]["states"]
            trails[name].set_data(state[: k + 1, 0], state[: k + 1, 1])
            bodies.extend(rover(ax, reference[k], REFERENCE, ghost=True))
            bodies.extend(rover(ax, state[k], COLORS[name]))
            clearance = audits[name]["sampled_clearance_m"]
            failed = np.any(clearance[: k + 1] <= 0)
            labels[name].set_text(
                f"Clearance {clearance[k]:+.2f} m"
                + ("\nENVELOPE VIOLATION RECORDED" if failed else "")
            )
            labels[name].set_color(VIOLATION if failed else TEXT)
        clock.set_text(f"t = {times[k]:05.2f} / {times[-1]:05.2f} s")
        signal.set_text(
            f"YAW BIAS {bias['yaw_rate_radps']:+g} rad/s"
            if bias["start_s"] <= times[k] < bias["end_s"]
            else "Yaw bias inactive"
        )

    update.times = times
    indices = np.rint(np.linspace(0, len(times) - 1, round(vis["duration_s"] * vis["fps"]))).astype(
        int
    )
    review = [
        bias["start_s"] - 2,
        (bias["start_s"] + bias["end_s"]) / 2,
        min(times[-1], bias["end_s"] + 8),
    ]
    animation_frames(
        fig,
        update,
        indices,
        output / "navigation_hero.gif",
        vis["fps"],
        review_times=review,
        palette_accents=[VIOLATION],
    )


def search_animation(root, output, config, summary):
    fig, axs = plt.subplots(1, 2, figsize=(10.8, 6))
    fig.subplots_adjust(top=0.80, bottom=0.16, left=0.065, right=0.98, wspace=0.20)
    heading(
        fig,
        "Where should the rover go?",
        "Recorded A* and Dijkstra expansion order • shared expansion-count clock, not wall-clock runtime",
    )
    artists, data, labels = {}, {}, {}
    radius = summary["rover_enclosing_radius_m"]
    for ax, name, color in zip(axs, ["astar", "dijkstra"], [CYAN, AMBER], strict=True):
        draw_world(ax, config, radius + config["search_margin_m"])
        data[name] = load_log(root / (name + ".npz"))
        scatter = ax.scatter([], [], s=8, color=color, alpha=0.4, zorder=4)
        (line,) = ax.plot([], [], color=color, lw=2, zorder=5)
        artists[name] = scatter, line
        ax.set_title("A*" if name == "astar" else "Dijkstra", color=color)
        labels[name] = ax.text(0.02, 0.97, "", va="top", transform=ax.transAxes, fontsize=10)
    max_count = max(len(s["expanded"]) for s in data.values())
    progress = fig.text(0.065, 0.045, "", color=MUTED, fontsize=10)

    def update(k):
        for name in data:
            count = min(k, len(data[name]["expanded"]))
            artists[name][0].set_offsets(data[name]["expanded"][:count])
            done = count == len(data[name]["expanded"])
            p = data[name]["path"] if done else np.empty((0, 2))
            artists[name][1].set_data(p[:, 0], p[:, 1])
            labels[name].set_text(f"Expanded: {count}" + (" • path found" if done else ""))
        progress.set_text(
            f"Shared expansion budget: {k} nodes • identical graph-path cost {summary['astar']['graph_cost_m']:.3f} m"
        )

    update.times = np.arange(max_count + 1)
    indices = np.rint(np.linspace(0, max_count, 120)).astype(int)
    animation_frames(
        fig,
        update,
        indices,
        output / "search_replay.gif",
        fps=12,
        review_times=[max_count * 0.3, max_count * 0.7, max_count],
    )


def render_navigation(results, media):
    root, output = Path(results), Path(media)
    manifest = verify_navigation(root)
    output.mkdir(parents=True, exist_ok=True)
    config, summary, plan = (
        read_json(root / "config.json"),
        read_json(root / "planning_summary.json"),
        load_log(root / "plan.npz"),
    )
    case = config["visualization"]["scenario"]
    if case != "yaw_bias":
        raise ValueError("This curated gallery requires visualization.scenario='yaw_bias'")
    logs = {name: load_log(root / f"{case}__{name}.npz") for name in ["open_loop", "feedback"]}
    audits = {name: load_log(root / f"{case}__{name}__clearance.npz") for name in logs}
    if any(len(log["times"]) != len(plan["times"]) for log in logs.values()):
        raise ValueError("Choose a full-horizon comparison for navigation playback")
    apply_theme()
    static_navigation(
        root, output, config, summary, plan, logs, audits, read_json(root / "metrics.json")
    )
    navigation_animation(output, config, summary, plan, logs, audits)
    search_animation(root, output, config, summary)
    write_json(
        output / "provenance.json",
        {
            "milestone": 2,
            "results_manifest_sha256": sha256(root / "manifest.json"),
            "input_files_sha256": manifest["files_sha256"],
            "render_source_sha256": {
                name: sha256(Path(__file__).parent / name)
                for name in ["navigation_render.py", "render.py", "theme.py"]
            },
            "outputs_sha256": {
                p.name: sha256(p)
                for p in sorted(output.iterdir())
                if p.suffix in {".png", ".svg", ".gif"}
            },
            "animation_semantics": {
                "navigation": "nearest logged states; time in simulation seconds",
                "search": "shared node-expansion budget; not wall-clock runtime",
            },
        },
    )
    print(
        f"Rendered 11 navigation figures, 2 GIFs and 6 review frames from saved evidence into {output}"
    )
