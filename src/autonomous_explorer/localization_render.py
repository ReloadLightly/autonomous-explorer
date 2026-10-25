"""Milestone 03 figures and replay; all empirical values come from saved logs."""

from pathlib import Path

import numpy as np
from matplotlib.patches import Ellipse

from .evidence import load_log, read_json, sha256, write_json
from .localization_evidence import MODES, verify_localization
from .model import wrap_angle
from .navigation_render import VIOLATION, draw_world
from .render import animation_frames, save_figure
from .theme import AMBER, CYAN, MUTED, REFERENCE, TEXT, apply_theme, heading, plt, rover

COLORS = {"oracle": REFERENCE, "dead_reckoning": AMBER, "ekf": CYAN}
LABELS = {
    "oracle": "Exact-state feedback",
    "dead_reckoning": "Dead reckoning",
    "ekf": "EKF feedback",
}


def shaded_intervals(ax, config, scenario):
    start, end = config["yaw_bias_interval_s"]
    if scenario["yaw_bias_radps"]:
        ax.axvspan(start, end, color=AMBER, alpha=0.15, label="Yaw bias")
    if scenario["landmark_outage_s"] is not None:
        ax.axvspan(
            *scenario["landmark_outage_s"], color=VIOLATION, alpha=0.10, label="Landmark outage"
        )


def bounds_for(config, logs):
    bounds = config["navigation"]["world"]["bounds_m"]
    points = np.concatenate(
        [log[key][:, :2] for log in logs.values() for key in ["states", "estimates"]]
    )
    return [
        min(bounds[0], points[:, 0].min()) - 0.4,
        max(bounds[1], points[:, 0].max()) + 0.4,
        min(bounds[2], points[:, 1].min()) - 0.4,
        max(bounds[3], points[:, 1].max()) + 0.4,
    ]


def landmarks(ax, config):
    xy = np.array(config["sensors"]["landmarks_xy_m"])
    ax.scatter(
        xy[:, 0],
        xy[:, 1],
        marker="D",
        s=35,
        facecolors="none",
        edgecolors=CYAN,
        label="Known landmarks",
        zorder=7,
    )
    for i, point in enumerate(xy):
        ax.annotate(
            f"L{i}", point, xytext=(5, 5), textcoords="offset points", fontsize=8, color=CYAN
        )


def covariance_ellipse(ax, state, covariance):
    values, vectors = np.linalg.eigh(covariance[:2, :2])
    scale = np.sqrt(np.maximum(values, 0) * 5.991464547107979)
    angle = np.degrees(np.arctan2(vectors[1, 1], vectors[0, 1]))
    artist = Ellipse(
        state[:2],
        width=2 * scale[1],
        height=2 * scale[0],
        angle=angle,
        edgecolor=CYAN,
        facecolor=CYAN,
        alpha=0.2,
        zorder=6,
    )
    ax.add_patch(artist)
    return artist


def case_logs(root, case, seed):
    return {mode: load_log(root / f"{case}__{seed}__{mode}.npz") for mode in MODES}


def static_localization(root, output, config, outcomes):
    seed = config["visualization"]["seed"]
    scenarios = {s["name"]: s for s in config["scenarios"]}
    # Observation geometry and actual measurement schedule.
    log = load_log(root / f"yaw_bias__{seed}__ekf.npz")
    t = log["times"]
    seen = np.flatnonzero(np.any(log["landmark_visible"], axis=1) & (t < 15))
    k = int(seen[-1]) if len(seen) else 0
    fig, axs = plt.subplots(1, 2, figsize=(12, 6))
    fig.subplots_adjust(top=0.80, bottom=0.16, left=0.07, right=0.97, wspace=0.25)
    heading(
        fig,
        "01  /  What the rover can measure",
        "Synthetic identified landmarks • 360° field of view • finite range and opaque obstacles",
    )
    draw_world(axs[0], config["navigation"])
    landmarks(axs[0], config)
    axs[0].plot(log["states"][: k + 1, 0], log["states"][: k + 1, 1], color=CYAN, lw=1.6)
    xy = np.array(config["sensors"]["landmarks_xy_m"])
    for j in np.flatnonzero(log["landmark_visible"][k]):
        axs[0].plot(
            [log["states"][k, 0], xy[j, 0]],
            [log["states"][k, 1], xy[j, 1]],
            ":",
            color=CYAN,
            alpha=0.7,
        )
    rover(axs[0], log["states"][k])
    axs[0].set_title(f"Recorded observations at t = {t[k]:.2f} s")
    # Plot only sensor opportunities, so scheduled gaps are not mistaken for occlusion.
    opportunities = np.arange(0, len(t), config["sensors"]["landmark_every_steps"])
    for j in range(len(xy)):
        good = opportunities[log["landmark_visible"][opportunities, j]]
        axs[1].scatter(t[good], np.full(len(good), j), s=9, marker="|", color=CYAN)
    axs[1].set(xlabel="Simulation time [s]", ylabel="Observed landmark ID", yticks=range(len(xy)))
    rate = 1 / (config["navigation"]["dt_s"] * config["sensors"]["landmark_every_steps"])
    axs[1].set_title(f"{rate:g} Hz observations after range/occlusion tests")
    axs[0].legend(loc="lower right", fontsize=8)
    save_figure(fig, output, "01_sensing", True)

    # Separate well-observed and outage cases; retain every unsafe trajectory.
    fig, axs = plt.subplots(1, 2, figsize=(12.5, 6.6))
    fig.subplots_adjust(top=0.79, bottom=0.17, left=0.065, right=0.97, wspace=0.20)
    heading(
        fig,
        "02  /  Feedback can only act on the state it receives",
        f"Seed {seed} • identical plant/noise streams • solid: actual motion • dotted cyan: EKF estimate",
    )
    for ax, case in zip(axs, ["yaw_bias", "landmark_outage"], strict=True):
        logs = case_logs(root, case, seed)
        draw_world(ax, config["navigation"], bounds=bounds_for(config, logs))
        ax.plot(
            logs["ekf"]["reference_states"][:, 0],
            logs["ekf"]["reference_states"][:, 1],
            ":",
            color=MUTED,
            lw=1,
            label="Reference",
        )
        for mode, saved in logs.items():
            ax.plot(
                saved["states"][:, 0], saved["states"][:, 1], color=COLORS[mode], label=LABELS[mode]
            )
            metric = outcomes[f"{case}__{seed}__{mode}"]
            violation = metric["first_disk_overlap_time_s"]
            if violation is not None:
                i = int(np.searchsorted(saved["times"], violation))
                ax.scatter(*saved["states"][i, :2], marker="x", color=VIOLATION, s=65, zorder=9)
        estimate = logs["ekf"]["estimates"]
        ax.plot(estimate[:, 0], estimate[:, 1], ":", color=CYAN, lw=1.1, label="EKF estimate")
        outage = scenarios[case]["landmark_outage_s"]
        ax.set_title(
            "Landmarks available"
            if outage is None
            else f"Landmarks absent on [{outage[0]:g}, {outage[1]:g}) s"
        )
        ax.legend(loc="lower right", fontsize=8)
    fig.text(
        0.065,
        0.045,
        "Red ×: first rover-envelope overlap. Collisions are audited after execution; no contact response or replanning.",
        color=VIOLATION,
        fontsize=10,
    )
    save_figure(fig, output, "02_execution", True)

    fig, axs = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
    fig.subplots_adjust(top=0.83, bottom=0.10, left=0.075, right=0.97, hspace=0.27, wspace=0.22)
    heading(
        fig,
        "03  /  Localization error becomes tracking error",
        "Upper: position estimate versus truth • lower: actual position versus reference • logarithmic metre scale",
    )
    for column, case in enumerate(["yaw_bias", "landmark_outage"]):
        for mode, saved in case_logs(root, case, seed).items():
            for row, target in [(0, "estimates"), (1, "reference_states")]:
                error = np.linalg.norm(saved["states"][:, :2] - saved[target][:, :2], axis=1)
                axs[row, column].plot(
                    saved["times"],
                    np.maximum(error, 1e-6),
                    color=COLORS[mode],
                    lw=1.3,
                    label=LABELS[mode],
                )
        for row in range(2):
            shaded_intervals(axs[row, column], config, scenarios[case])
            axs[row, column].set(yscale="log", ylabel="Position error [m]", ylim=(1e-4, 5))
        axs[0, column].set_title(case.replace("_", " ").capitalize())
        axs[1, column].set_xlabel("Simulation time [s]")
    axs[0, 0].legend(fontsize=8, loc="lower right")
    save_figure(fig, output, "03_errors")

    fig, axs = plt.subplots(1, 2, figsize=(12, 5.8), sharey=True)
    fig.subplots_adjust(top=0.80, bottom=0.17, left=0.075, right=0.97, wspace=0.18)
    heading(
        fig,
        "04  /  A small covariance does not certify clearance",
        "Truth-based interval clearance bounds • enclosing rover disk • negative values retain unsafe execution",
    )
    for ax, case in zip(axs, ["yaw_bias", "landmark_outage"], strict=True):
        for mode, saved in case_logs(root, case, seed).items():
            ax.plot(
                saved["times"][:-1],
                saved["interval_clearance_bound_m"],
                color=COLORS[mode],
                lw=1.4,
                label=LABELS[mode],
            )
        ax.axhline(0, color=VIOLATION, ls="--", lw=1)
        shaded_intervals(ax, config, scenarios[case])
        ax.set(title=case.replace("_", " ").capitalize(), xlabel="Simulation time [s]")
    axs[0].set_ylabel("Interval clearance lower bound [m]")
    axs[0].legend(fontsize=8, loc="lower left")
    fig.text(
        0.075,
        0.045,
        "Conditioned on numerical endpoints; this audit omits a formal global RK4 error bound.",
        color=MUTED,
        fontsize=9,
    )
    save_figure(fig, output, "04_clearance", True)

    saved = load_log(root / f"landmark_outage__{seed}__ekf.npz")
    t = saved["times"]
    error = saved["estimates"] - saved["states"]
    error[:, 3] = wrap_angle(error[:, 3])
    fig, axs = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
    fig.subplots_adjust(top=0.83, bottom=0.11, left=0.075, right=0.97, wspace=0.24, hspace=0.30)
    heading(
        fig,
        "05  /  Watch confidence fail during a sensor outage",
        f"EKF, seed {seed} • declared covariance is a model belief • fixed bias violates its zero-mean assumption",
    )
    for ax, index in zip(axs[0], [0, 1], strict=True):
        sigma = np.sqrt(saved["covariances"][:, index, index])
        ax.fill_between(
            t, -1.96 * sigma, 1.96 * sigma, color=CYAN, alpha=0.2, label="Marginal ±1.96σ"
        )
        ax.plot(t, error[:, index], color=CYAN, lw=1.4, label="Estimate − truth")
        ax.set(title=f"{'x' if index == 0 else 'y'} position error", ylabel="Position error [m]")
    nees = np.einsum(
        "ni,ni->n", error, np.linalg.solve(saved["covariances"], error[..., None])[..., 0]
    )
    axs[1, 0].plot(t, nees, color=CYAN, lw=1.2)
    axs[1, 0].axhline(4, color=REFERENCE, ls=":", lw=1, label="4D nominal mean (matched model)")
    axs[1, 0].set(
        title="Normalized estimation error squared", ylabel="NEES [dimensionless]", yscale="log"
    )
    opportunity = np.arange(0, len(t), config["sensors"]["landmark_every_steps"])
    axs[1, 1].step(
        t[opportunity], saved["landmark_visible"][opportunity].sum(axis=1), where="post", color=CYAN
    )
    axs[1, 1].set(
        title="Visible landmarks at sensor opportunities", ylabel="Number of observations"
    )
    for ax in axs.flat:
        shaded_intervals(ax, config, scenarios["landmark_outage"])
    axs[0, 0].legend(fontsize=8, loc="lower left")
    axs[1, 0].legend(fontsize=8, loc="upper left")
    for ax in axs[1]:
        ax.set_xlabel("Simulation time [s]")
    save_figure(fig, output, "05_uncertainty")

    fig, axs = plt.subplots(1, 2, figsize=(12, 5.8))
    fig.subplots_adjust(top=0.80, bottom=0.19, left=0.075, right=0.97, wspace=0.24)
    heading(
        fig,
        f"06  /  {len(config['seeds'])} seeds, with every outcome retained",
        "Dots: individual runs • bars: mean • whiskers: min–max • a small reproducibility study",
    )
    cases = [s["name"] for s in config["scenarios"]]
    for ax, metric, title in zip(
        axs,
        ["rms_localization_m", "rms_position_m"],
        ["Localization RMS (oracle has a shadow EKF)", "Actual tracking RMS"],
        strict=True,
    ):
        for i, mode in enumerate(MODES):
            x = np.arange(len(cases)) + (i - 1) * 0.24
            values = np.array(
                [
                    [outcomes[f"{case}__{s}__{mode}"][metric] for s in config["seeds"]]
                    for case in cases
                ]
            )
            means = values.mean(axis=1)
            ax.bar(x, means, width=0.21, color=COLORS[mode], alpha=0.65, label=LABELS[mode])
            ax.errorbar(
                x,
                means,
                yerr=[means - values.min(axis=1), values.max(axis=1) - means],
                fmt="none",
                ecolor=TEXT,
                capsize=3,
                lw=1,
            )
            for column in range(values.shape[1]):
                ax.scatter(
                    x,
                    values[:, column],
                    color=COLORS[mode],
                    edgecolors=TEXT,
                    lw=0.4,
                    s=20,
                    zorder=5,
                )
        ax.set(
            title=title,
            ylabel="RMS position error [m]",
            xticks=np.arange(len(cases)),
            xticklabels=[c.replace("_", "\n") for c in cases],
        )
    axs[0].legend(fontsize=8, loc="upper left")
    save_figure(fig, output, "06_seed_comparison")


def localization_animation(output, config, logs, outcomes):
    vis = config["visualization"]
    scenario = next(s for s in config["scenarios"] if s["name"] == vis["scenario"])
    times = max((log["times"] for log in logs.values()), key=len)
    fig, axs = plt.subplots(1, 3, figsize=(13.5, 6.5))
    fig.subplots_adjust(top=0.79, bottom=0.32, left=0.05, right=0.98, wspace=0.23)
    heading(
        fig,
        "AUTONOMOUS EXPLORER  /  Sensing and localization",
        "Same plant/noise streams • solid: truth • dotted: estimate • ghost: estimated pose • red: envelope overlap",
    )
    bounds = bounds_for(config, logs)
    dynamic, trails, estimates, labels = [], {}, {}, {}
    for ax, mode in zip(axs, MODES, strict=True):
        log = logs[mode]
        draw_world(ax, config["navigation"], bounds=bounds)
        ax.plot(
            log["reference_states"][:, 0], log["reference_states"][:, 1], ":", color=MUTED, lw=1
        )
        (trails[mode],) = ax.plot([], [], color=COLORS[mode], lw=2)
        (estimates[mode],) = ax.plot([], [], ":", color=COLORS[mode], lw=1.2)
        labels[mode] = fig.text(
            0.055 + MODES.index(mode) * 0.33,
            0.19,
            "",
            va="top",
            fontsize=9,
            family="DejaVu Sans Mono",
        )
        ax.set_title(LABELS[mode], color=COLORS[mode], fontsize=12)
    clock = fig.text(0.05, 0.055, "", fontsize=11, family="DejaVu Sans Mono")
    fig.text(
        0.05,
        0.018,
        "EKF ellipse: approximate 95% position belief. Collisions are post hoc observations; no runtime safety intervention.",
        color=MUTED,
        fontsize=9,
    )

    def update(k):
        for artist in dynamic:
            artist.remove()
        dynamic.clear()
        for ax, mode in zip(axs, MODES, strict=True):
            log = logs[mode]
            i = min(k, len(log["times"]) - 1)
            actual, estimate = log["states"][i], log["estimates"][i]
            trails[mode].set_data(log["states"][: i + 1, 0], log["states"][: i + 1, 1])
            if mode != "oracle":
                estimates[mode].set_data(log["estimates"][: i + 1, 0], log["estimates"][: i + 1, 1])
                dynamic.extend(rover(ax, estimate, COLORS[mode], ghost=True))
            if mode == "ekf":
                dynamic.append(covariance_ellipse(ax, estimate, log["covariances"][i]))
            clearance = log["sampled_clearance_m"][i]
            dynamic.extend(rover(ax, actual, VIOLATION if clearance <= 0 else COLORS[mode]))
            tracking = np.linalg.norm(actual[:2] - log["reference_states"][i, :2])
            first = outcomes[f"{vis['scenario']}__{vis['seed']}__{mode}"][
                "first_disk_overlap_time_s"
            ]
            violation = first is not None and log["times"][i] >= first
            labels[mode].set_text(
                f"tracking {tracking:.3f} m\nclearance {clearance:+.3f} m"
                + ("\nENVELOPE VIOLATED" if violation else "")
            )
            labels[mode].set_color(VIOLATION if violation else TEXT)
        outage = scenario["landmark_outage_s"]
        offline = outage is not None and outage[0] <= times[k] < outage[1]
        clock.set_text(
            f"t = {times[k]:5.2f} s / {times[-1]:.2f} s    "
            + ("LANDMARK OUTAGE" if offline else "landmark sensor enabled")
        )
        clock.set_color(VIOLATION if offline else TEXT)

    update.times = times
    indices = np.rint(np.linspace(0, len(times) - 1, round(vis["fps"] * vis["duration_s"]))).astype(
        int
    )
    animation_frames(
        fig,
        update,
        indices,
        output / "localization_hero.gif",
        vis["fps"],
        review_times=[15.0, 27.0, 45.0],
        palette_accents=[VIOLATION],
    )


def render_localization(results, media):
    root, output = Path(results), Path(media)
    manifest = verify_localization(root)
    config, outcomes = read_json(root / "config.json"), read_json(root / "metrics.json")
    names = {s["name"] for s in config["scenarios"]}
    if not {"yaw_bias", "landmark_outage"} <= names:
        raise ValueError(
            "Curated localization gallery requires yaw_bias and landmark_outage scenarios"
        )
    if config["visualization"]["seed"] not in config["seeds"]:
        raise ValueError("Visualization seed must be an experiment seed")
    output.mkdir(parents=True, exist_ok=True)
    apply_theme()
    static_localization(root, output, config, outcomes)
    vis = config["visualization"]
    logs = case_logs(root, vis["scenario"], vis["seed"])
    localization_animation(output, config, logs, outcomes)
    write_json(
        output / "provenance.json",
        {
            "milestone": 3,
            "results_manifest_sha256": sha256(root / "manifest.json"),
            "input_files_sha256": manifest["files_sha256"],
            "render_source_sha256": {
                name: sha256(Path(__file__).parent / name)
                for name in [
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
            "animation_semantics": "nearest logged states; shared simulation clock; last state held on early exit",
        },
    )
    print(
        f"Rendered 6 localization figures, 1 GIF and 3 review frames from saved evidence into {output}"
    )
