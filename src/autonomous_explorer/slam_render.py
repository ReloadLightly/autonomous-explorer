"""Saved-data landmark SLAM gallery. Ground-truth crosses are evaluation overlays only."""

from pathlib import Path

import numpy as np

from .evidence import load_log, read_json, sha256, write_json
from .localization_render import covariance_ellipse, shaded_intervals
from .navigation_render import VIOLATION, draw_world
from .render import animation_frames, save_figure
from .slam_evidence import MODES, verify_slam
from .theme import AMBER, CYAN, MUTED, REFERENCE, apply_theme, heading, plt, rover

COLORS = {"known_map": REFERENCE, "first_sighting": AMBER, "slam": CYAN}
LABELS = {
    "known_map": "Supplied-map EKF",
    "first_sighting": "First sightings only",
    "slam": "Joint EKF SLAM",
}


def case_logs(root, case, seed):
    return {mode: load_log(root / f"{case}__{seed}__{mode}.npz") for mode in MODES}


def spatial_bounds(config, logs):
    bounds = config["navigation"]["world"]["bounds_m"]
    points = np.concatenate(
        [log[key][:, :2] for log in logs.values() for key in ["states", "estimates"]]
    )
    return [
        min(bounds[0], points[:, 0].min()) - 0.3,
        max(bounds[1], points[:, 0].max()) + 0.3,
        min(bounds[2], points[:, 1].min()) - 0.3,
        max(bounds[3], points[:, 1].max()) + 0.3,
    ]


def true_landmarks(ax, config):
    points = np.asarray(config["truth_landmarks_xy_m"])
    ax.scatter(
        points[:, 0],
        points[:, 1],
        marker="x",
        s=35,
        color=REFERENCE,
        linewidths=1,
        label="Truth (evaluation only)",
        zorder=7,
    )
    for j, point in enumerate(points):
        ax.annotate(
            f"L{j}", point, xytext=(4, 5), textcoords="offset points", color=MUTED, fontsize=7
        )


def draw_map(ax, log, k, color):
    """Map means and joint covariance come from the same recorded posterior snapshot."""
    slot = max(0, int(np.searchsorted(log["joint_indices"], k, side="right") - 1))
    state, covariance = log["joint_states"][slot], log["joint_covariances"][slot]
    points = state[4:].reshape(-1, 2)
    good = np.all(np.isfinite(points), axis=1)
    artists = [
        ax.scatter(
            points[good, 0],
            points[good, 1],
            s=28,
            marker="o",
            facecolors="none",
            edgecolors=color,
            label="Estimated map",
            zorder=8,
        )
    ]
    for j in np.flatnonzero(good):
        block = covariance[4 + 2 * j : 6 + 2 * j, 4 + 2 * j : 6 + 2 * j]
        ellipse = covariance_ellipse(ax, points[j], block)
        ellipse.set_facecolor(color)
        ellipse.set_edgecolor(color)
        ellipse.set_alpha(0.17)
        artists.append(ellipse)
    return artists


def plot_background(ax, config, log, bounds=None):
    draw_world(ax, config["navigation"], bounds=bounds)
    ax.plot(
        log["reference_states"][:, 0],
        log["reference_states"][:, 1],
        ":",
        color=MUTED,
        lw=1,
        label="Planned reference",
    )
    true_landmarks(ax, config)


def map_error_history(log, config):
    points = log["landmark_estimates"]
    truth = np.asarray(config["truth_landmarks_xy_m"])
    squared = np.sum((points - truth) ** 2, axis=2)
    count = np.sum(np.isfinite(squared), axis=1)
    return np.sqrt(
        np.divide(
            np.nansum(squared, axis=1), count, out=np.full(len(count), np.nan), where=count > 0
        )
    )


def static_slam(root, output, config, outcomes):
    seed = config["visualization"]["seed"]
    nominal = case_logs(root, "nominal", seed)
    bounds = spatial_bounds(config, nominal)
    fig, axs = plt.subplots(1, 2, figsize=(12, 6.5))
    fig.subplots_adjust(top=0.80, bottom=0.16, left=0.065, right=0.97, wspace=0.18)
    heading(
        fig,
        "01  /  Build the map while estimating motion",
        "Landmark coordinates withheld from both filters • crosses: evaluation truth • shaded obstacles: supplied routing map",
    )
    for ax, mode in zip(axs, ["first_sighting", "slam"], strict=True):
        log = nominal[mode]
        plot_background(ax, config, log, bounds)
        ax.plot(
            log["states"][:, 0],
            log["states"][:, 1],
            color=COLORS[mode],
            lw=1.8,
            label="Actual motion",
        )
        ax.plot(
            log["estimates"][:, 0],
            log["estimates"][:, 1],
            "--",
            color=COLORS[mode],
            lw=1,
            label="Pose estimate",
        )
        draw_map(ax, log, len(log["times"]) - 1, COLORS[mode])
        rover(ax, log["states"][-1], COLORS[mode])
        ax.set_title(LABELS[mode], color=COLORS[mode])
        ax.legend(loc="center", fontsize=7)
    fig.text(
        0.065,
        0.04,
        "Ellipses: approximate 95% map marginals. No truth alignment or surveyed landmarks enter SLAM.",
        color=MUTED,
        fontsize=9,
    )
    save_figure(fig, output, "01_mapping", True)

    log = nominal["slam"]
    fig, axs = plt.subplots(1, 4, figsize=(14, 5.8))
    fig.subplots_adjust(top=0.77, bottom=0.17, left=0.05, right=0.98, wspace=0.32)
    heading(
        fig,
        "02  /  Discover, correlate, revisit",
        "The map begins empty • each landmark appears only after a recorded sensor observation • same spatial scale in every panel",
    )
    for ax, fraction in zip(axs, [0.08, 0.40, 0.75, 1.0], strict=True):
        k = int(np.argmin(abs(log["times"] - fraction * log["times"][-1])))
        plot_background(ax, config, log, bounds)
        ax.plot(log["states"][: k + 1, 0], log["states"][: k + 1, 1], color=CYAN, lw=1.5)
        draw_map(ax, log, k, CYAN)
        rover(ax, log["states"][k], CYAN)
        mapped = np.count_nonzero(np.all(np.isfinite(log["landmark_estimates"][k]), axis=1))
        ax.set_title(f"t = {log['times'][k]:.1f} s\n{mapped} landmarks", fontsize=11)
        ax.set_xlabel("x [m]")
    save_figure(fig, output, "02_map_growth")

    scenarios = {s["name"]: s for s in config["scenarios"]}
    fig, axs = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
    fig.subplots_adjust(top=0.82, bottom=0.10, left=0.08, right=0.97, hspace=0.28, wspace=0.22)
    heading(
        fig,
        "03  /  Separate localization from tracking",
        "Upper: pose estimate versus actual position • lower: actual position versus reference • every partial history retained",
    )
    for column, case in enumerate(["nominal", "landmark_outage"]):
        for mode, saved in case_logs(root, case, seed).items():
            for row, target in [(0, "estimates"), (1, "reference_states")]:
                error = np.linalg.norm(saved["states"][:, :2] - saved[target][:, :2], axis=1)
                axs[row, column].plot(
                    saved["times"], error, color=COLORS[mode], lw=1.3, label=LABELS[mode]
                )
        for row in range(2):
            shaded_intervals(axs[row, column], config, scenarios[case])
            axs[row, column].set_ylabel("Position error [m]")
        axs[0, column].set_title(case.replace("_", " ").capitalize())
        axs[1, column].set_xlabel("Simulation time [s]")
    axs[0, 0].legend(fontsize=8)
    save_figure(fig, output, "03_errors")

    fig, axs = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
    fig.subplots_adjust(top=0.82, bottom=0.14, left=0.08, right=0.97, hspace=0.25, wspace=0.22)
    heading(
        fig,
        "04  /  What do revisits add on the same recorded motion?",
        "Cyan: normal SLAM replay • amber: revisits denied after a 5 s observation gap • identical observations and applied controls",
    )
    for column, case in enumerate(["nominal", "landmark_outage"]):
        source = load_log(root / f"{case}__{seed}__slam.npz")
        counter = load_log(root / f"{case}__{seed}__no_revisit.npz")
        for saved, color, label in [
            (source, CYAN, "SLAM"),
            ({**source, **counter}, AMBER, "Revisits denied"),
        ]:
            error = np.linalg.norm(saved["estimates"][:, :2] - source["states"][:, :2], axis=1)
            axs[0, column].plot(source["times"], error, color=color, lw=1.4, label=label)
            axs[1, column].plot(
                source["times"], map_error_history(saved, config), color=color, lw=1.4
            )
        for time in source["times"][np.any(source["landmark_events"] == 3, axis=1)]:
            for row in range(2):
                axs[row, column].axvline(time, color=REFERENCE, lw=0.8, ls=":")
        axs[0, column].set_title(case.replace("_", " ").capitalize())
        axs[0, column].set_ylabel("Localization error [m]")
        axs[1, column].set(ylabel="Discovered-map RMS [m]", xlabel="Simulation time [s]")
    axs[0, 0].legend(fontsize=8)
    fig.text(
        0.08,
        0.04,
        "Dotted lines: accepted revisit packets. Absolute map error need not improve for every seed; the initial pose prior anchors the frame.",
        color=MUTED,
        fontsize=9,
    )
    save_figure(fig, output, "04_revisits", True)

    covariance = nominal["slam"]["joint_covariances"][-1]
    sigma = np.sqrt(np.diag(covariance))
    correlation = covariance / np.outer(sigma, sigma)
    labels = ["x", "y", "v", "θ"] + [
        f"L{j}{axis}" for j in range(len(config["truth_landmarks_xy_m"])) for axis in ["x", "y"]
    ]
    fig, axs = plt.subplots(1, 2, figsize=(12, 6.5), gridspec_kw={"width_ratios": [1.2, 1]})
    fig.subplots_adjust(top=0.80, bottom=0.18, left=0.07, right=0.94, wspace=0.35)
    heading(
        fig,
        "05  /  One correlated belief over rover and map",
        "Final SLAM covariance in canonical feature-ID order • correlations propagate information across landmarks",
    )
    matrix = axs[0].imshow(correlation, vmin=-1, vmax=1, cmap="RdBu_r", interpolation="nearest")
    axs[0].set(
        xticks=range(len(labels)), yticks=range(len(labels)), xticklabels=labels, yticklabels=labels
    )
    axs[0].tick_params(axis="x", rotation=90, labelsize=8)
    axs[0].tick_params(axis="y", labelsize=8)
    axs[0].grid(False)
    fig.colorbar(matrix, ax=axs[0], fraction=0.046, pad=0.03, label="Correlation")
    for mode in ["first_sighting", "slam"]:
        saved = nominal[mode]
        trace = np.trace(saved["covariances"][:, :2, :2], axis1=1, axis2=2)
        axs[1].plot(saved["times"], trace, color=COLORS[mode], label=LABELS[mode])
    axs[1].set(xlabel="Simulation time [s]", ylabel="Position covariance trace [m²]", yscale="log")
    axs[1].legend(fontsize=8)
    fig.text(
        0.07,
        0.035,
        "A finite initial rover prior fixes the coordinate frame. Relative measurements cannot supply an independent absolute position or heading.",
        color=MUTED,
        fontsize=9,
    )
    save_figure(fig, output, "05_joint_uncertainty")

    logs = case_logs(root, "landmark_outage", seed)
    fig, axs = plt.subplots(1, 2, figsize=(12, 6.5))
    fig.subplots_adjust(top=0.80, bottom=0.18, left=0.07, right=0.97, wspace=0.22)
    heading(
        fig,
        "06  /  Map recovery does not erase unsafe execution",
        "Landmark outage plus unmodeled yaw bias • actual paths and interval clearance • negative values retained",
    )
    draw_world(axs[0], config["navigation"], bounds=spatial_bounds(config, logs))
    for mode, saved in logs.items():
        axs[0].plot(
            saved["states"][:, 0], saved["states"][:, 1], color=COLORS[mode], label=LABELS[mode]
        )
        axs[1].plot(
            saved["times"][:-1], saved["interval_clearance_bound_m"], color=COLORS[mode], lw=1.3
        )
        metric = outcomes[f"landmark_outage__{seed}__{mode}"]
        first = metric["first_disk_overlap_time_s"]
        if first is not None:
            k = int(np.searchsorted(saved["times"], first))
            axs[0].scatter(*saved["states"][k, :2], color=VIOLATION, marker="x", s=65, zorder=9)
    shaded_intervals(axs[1], config, scenarios["landmark_outage"])
    axs[1].axhline(0, color=VIOLATION, ls="--", lw=1)
    axs[1].set(xlabel="Simulation time [s]", ylabel="Interval clearance lower bound [m]")
    axs[0].legend(fontsize=8)
    fig.text(
        0.07,
        0.045,
        "The full multi-seed outcomes retain envelope overlaps and singular-speed exits. No contact response, stop, or replanning is implemented.",
        color=MUTED,
        fontsize=9,
    )
    save_figure(fig, output, "06_outage_clearance", True)


def slam_animation(output, config, logs, outcomes):
    vis = config["visualization"]
    modes = ["first_sighting", "slam"]
    times = max((logs[m]["times"] for m in modes), key=len)
    fig, axs = plt.subplots(1, 2, figsize=(11.5, 6.8))
    fig.subplots_adjust(top=0.80, bottom=0.29, left=0.06, right=0.97, wspace=0.18)
    heading(
        fig,
        "AUTONOMOUS EXPLORER  /  Build a landmark map",
        "Coordinates withheld • solid: actual motion • dashed: estimated motion • crosses: evaluation truth",
    )
    bounds = spatial_bounds(config, {m: logs[m] for m in modes})
    dynamic, trails, estimates, labels = [], {}, {}, {}
    for ax, mode in zip(axs, modes, strict=True):
        log = logs[mode]
        plot_background(ax, config, log, bounds)
        (trails[mode],) = ax.plot([], [], color=COLORS[mode], lw=2)
        (estimates[mode],) = ax.plot([], [], "--", color=COLORS[mode], lw=1)
        labels[mode] = fig.text(
            0.065 + modes.index(mode) * 0.48,
            0.16,
            "",
            va="top",
            fontsize=10,
            family="DejaVu Sans Mono",
        )
        ax.set_title(LABELS[mode], color=COLORS[mode])
    clock = fig.text(0.065, 0.06, "", fontsize=11, family="DejaVu Sans Mono")
    fig.text(
        0.065,
        0.025,
        "Map circles/ellipses: latest recorded joint posterior. Shaded obstacles are supplied for routing; SLAM builds the point map.",
        color=MUTED,
        fontsize=8,
    )

    def update(k):
        for artist in dynamic:
            artist.remove()
        dynamic.clear()
        for ax, mode in zip(axs, modes, strict=True):
            log = logs[mode]
            i = min(k, len(log["times"]) - 1)
            trails[mode].set_data(log["states"][: i + 1, 0], log["states"][: i + 1, 1])
            estimates[mode].set_data(log["estimates"][: i + 1, 0], log["estimates"][: i + 1, 1])
            dynamic.extend(draw_map(ax, log, i, COLORS[mode]))
            dynamic.extend(rover(ax, log["estimates"][i], COLORS[mode], ghost=True))
            unsafe = log["sampled_clearance_m"][i] <= 0
            dynamic.extend(rover(ax, log["states"][i], VIOLATION if unsafe else COLORS[mode]))
            mapped = np.count_nonzero(np.all(np.isfinite(log["landmark_estimates"][i]), axis=1))
            error = np.linalg.norm(log["states"][i, :2] - log["estimates"][i, :2])
            labels[mode].set_text(
                f"mapped {mapped} / {len(config['truth_landmarks_xy_m'])}    pose error {error:.3f} m"
            )
        revisits = int(
            np.count_nonzero(np.any(logs["slam"]["landmark_events"][: k + 1] == 3, axis=1))
        )
        clock.set_text(f"t = {times[k]:5.2f} s / {times[-1]:.2f} s    revisit packets: {revisits}")

    update.times = times
    indices = np.rint(np.linspace(0, len(times) - 1, round(vis["fps"] * vis["duration_s"]))).astype(
        int
    )
    animation_frames(
        fig,
        update,
        indices,
        output / "slam_hero.gif",
        vis["fps"],
        review_times=[times[-1] * 0.1, times[-1] * 0.6, times[-1]],
        palette_accents=[VIOLATION],
    )


def render_slam(results, media):
    root, output = Path(results), Path(media)
    manifest = verify_slam(root)
    config, outcomes = read_json(root / "config.json"), read_json(root / "metrics.json")
    if not {"nominal", "landmark_outage"} <= {s["name"] for s in config["scenarios"]}:
        raise ValueError("The curated SLAM gallery requires nominal and landmark_outage cases")
    vis = config["visualization"]
    if vis["seed"] not in config["seeds"] or vis["scenario"] not in {
        s["name"] for s in config["scenarios"]
    }:
        raise ValueError("Visualization must select a recorded scenario and seed")
    output.mkdir(parents=True, exist_ok=True)
    apply_theme()
    static_slam(root, output, config, outcomes)
    slam_animation(output, config, case_logs(root, vis["scenario"], vis["seed"]), outcomes)
    write_json(
        output / "provenance.json",
        {
            "milestone": 4,
            "results_manifest_sha256": sha256(root / "manifest.json"),
            "input_files_sha256": manifest["files_sha256"],
            "render_source_sha256": {
                name: sha256(Path(__file__).parent / name)
                for name in [
                    "slam_render.py",
                    "localization_render.py",
                    "navigation_render.py",
                    "render.py",
                    "theme.py",
                ]
            },
            "outputs_sha256": {
                p.name: sha256(p)
                for p in sorted(output.iterdir())
                if p.suffix in {".png", ".svg", ".gif"}
            },
            "animation_semantics": "shared logged-state clock; map uses latest recorded joint posterior; final state held on early exit",
        },
    )
    print(f"Rendered 6 SLAM figures, 1 GIF and 3 review frames into {output}")
