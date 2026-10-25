"""Saved-evidence mission gallery: goal choices, health intervention, and honest arrival audits."""

from pathlib import Path

import numpy as np

from .evidence import load_log, read_json, sha256, write_json
from .exploration_evidence import MODES, verify_exploration
from .mission import STATES
from .navigation_render import VIOLATION, draw_world
from .render import animation_frames, save_figure
from .theme import AMBER, CYAN, MUTED, REFERENCE, TEXT, apply_theme, heading, plt, rover

COLORS = {"unguarded": AMBER, "guarded": CYAN}
LABELS = {"unguarded": "No health supervision", "guarded": "Health-supervised mission"}
STATE_NAMES = {value: key for key, value in STATES.items()}
STATE_COLORS = [CYAN, REFERENCE, AMBER, "#b194e9", "#89d6a0", VIOLATION]


def logs_for(root, case, seed):
    return {mode: load_log(root / f"{case}__{seed}__{mode}.npz") for mode in MODES}


def plot_bounds(config, logs):
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


def outage(ax, scenario):
    if scenario["landmark_outage_s"] is not None:
        ax.axvspan(*scenario["landmark_outage_s"], color=VIOLATION, alpha=0.10)


def map_background(ax, config, bounds):
    draw_world(ax, config["navigation"], bounds=bounds)
    points = np.asarray(config["policy"]["viewpoints_xy_m"])
    ax.scatter(
        points[:, 0],
        points[:, 1],
        s=22,
        marker="s",
        facecolors="none",
        edgecolors=MUTED,
        zorder=7,
        label="Candidate viewpoints",
    )
    for j, point in enumerate(points):
        ax.annotate(
            str(j), point, xytext=(4, 5), textcoords="offset points", color=MUTED, fontsize=8
        )


def static_exploration(root, output, config, outcomes):
    seed = config["visualization"]["seed"]
    logs = logs_for(root, "temporary_outage", seed)
    bounds = plot_bounds(config, logs)
    fig, axs = plt.subplots(1, 2, figsize=(12, 6.5))
    fig.subplots_adjust(top=0.80, bottom=0.17, left=0.07, right=0.97, wspace=0.20)
    heading(
        fig,
        "01  /  Choose goals, monitor health, return home",
        "Temporary landmark outage • solid: actual motion • dashed: estimated motion • squares: supplied candidate viewpoints",
    )
    for ax, mode in zip(axs, MODES, strict=True):
        log = logs[mode]
        map_background(ax, config, bounds)
        cells = log["coverage_cells"][log["coverage"][-1]]
        ax.scatter(cells[:, 0], cells[:, 1], marker="s", s=65, color=COLORS[mode], alpha=0.10)
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
            lw=1.0,
            label="Estimated motion",
        )
        rover(ax, log["states"][-1], COLORS[mode])
        ax.set_title(LABELS[mode], color=COLORS[mode], fontsize=12)
        ax.legend(fontsize=8, loc="center")
    fig.text(
        0.07,
        0.04,
        "Tinted cells: modelled observation coverage from estimated poses. The obstacle map is supplied; occupancy is not learned.",
        color=MUTED,
        fontsize=9,
    )
    save_figure(fig, output, "01_online_missions", True)

    cases = [s["name"] for s in config["scenarios"]]
    fig, axs = plt.subplots(len(cases), 1, figsize=(12, 8), sharex=True)
    fig.subplots_adjust(top=0.82, bottom=0.15, left=0.13, right=0.97, hspace=0.45)
    heading(
        fig,
        "02  /  Decisions as a state history",
        "Recorded supervisor modes • short histories end at their terminal decision • no extrapolated mission outcomes",
    )
    for ax, case in zip(np.atleast_1d(axs), cases, strict=True):
        for row, mode in enumerate(MODES):
            log = load_log(root / f"{case}__{seed}__{mode}.npz")
            states = log["mission_states"]
            splits = np.r_[0, np.flatnonzero(np.diff(states)) + 1, len(states)]
            for start, end in zip(splits[:-1], splits[1:], strict=True):
                left = log["times"][start]
                right = log["times"][end] if end < len(states) else log["times"][-1] + 0.4
                ax.broken_barh(
                    [(left, right - left)], (row - 0.3, 0.6), facecolors=STATE_COLORS[states[start]]
                )
        ax.set(
            yticks=[0, 1],
            yticklabels=["unguarded", "guarded"],
            ylim=(-0.6, 1.6),
            title=case.replace("_", " ").capitalize(),
        )
    np.atleast_1d(axs)[-1].set_xlabel("Simulation time [s]")
    for j, name in enumerate(STATES):
        fig.text(0.13 + j * 0.14, 0.045, name, color=STATE_COLORS[j], fontsize=10)
    save_figure(fig, output, "02_supervisor_states")

    metadata = read_json(root / f"nominal__{seed}__guarded.json")
    fig, axs = plt.subplots(1, 2, figsize=(12, 5.8))
    fig.subplots_adjust(top=0.79, bottom=0.17, left=0.075, right=0.97, wspace=0.23)
    heading(
        fig,
        "03  /  Why this viewpoint next?",
        "Utility = unseen model cells + weighted estimated-map uncertainty − weighted checked-path length",
    )
    for ax, decision in zip(axs, metadata["decisions"][:2], strict=True):
        candidates = decision["candidates"]
        ids = [c["id"] for c in candidates]
        values = [c["utility"] for c in candidates]
        ax.bar(
            np.arange(len(ids)),
            values,
            color=[CYAN if j == decision["selected"] else REFERENCE for j in ids],
            alpha=0.8,
        )
        ax.set(
            xticks=np.arange(len(ids)),
            xticklabels=ids,
            xlabel="Candidate viewpoint ID",
            ylabel="Heuristic utility",
            title=f"t = {decision['time_s']:.2f} s • coverage {decision['coverage_fraction']:.0%}",
        )
        ax.axhline(0, color=MUTED, lw=0.8)
    fig.text(
        0.075,
        0.04,
        "Every candidate score, selected goal, and checked route is saved. This greedy heuristic has no global optimality claim.",
        color=MUTED,
        fontsize=9,
    )
    save_figure(fig, output, "03_goal_utilities", True)

    fig, axs = plt.subplots(1, len(cases), figsize=(14, 5.7), sharey=True)
    fig.subplots_adjust(top=0.79, bottom=0.16, left=0.07, right=0.98, wspace=0.23)
    heading(
        fig,
        "04  /  Audit actual clearance separately from mission status",
        "Truth-based enclosing-disk interval bounds • positive means clear within the numerical audit • unsafe histories remain visible",
    )
    for ax, scenario in zip(np.atleast_1d(axs), config["scenarios"], strict=True):
        for mode, log in logs_for(root, scenario["name"], seed).items():
            ax.plot(
                log["times"][:-1],
                log["interval_clearance_bound_m"],
                color=COLORS[mode],
                lw=1.4,
                label=LABELS[mode],
            )
        outage(ax, scenario)
        ax.axhline(0, color=VIOLATION, ls="--", lw=1)
        ax.set(title=scenario["name"].replace("_", " ").capitalize(), xlabel="Simulation time [s]")
    np.atleast_1d(axs)[0].set_ylabel("Interval clearance bound [m]")
    np.atleast_1d(axs)[0].legend(fontsize=7, loc="lower left")
    save_figure(fig, output, "04_clearance", True)

    scenario = next(s for s in config["scenarios"] if s["name"] == "temporary_outage")
    fig, axs = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    fig.subplots_adjust(top=0.82, bottom=0.11, left=0.08, right=0.97, hspace=0.25)
    heading(
        fig,
        "05  /  Brake, wait for evidence, replan",
        "Temporary outage • supervisor reads estimate/covariance and accepted-observation age • actual speed is evaluation only",
    )
    for mode, log in logs.items():
        axs[0].plot(log["times"], log["landmark_age_s"], color=COLORS[mode], label=LABELS[mode])
        axs[1].plot(log["times"], log["states"][:, 2], color=COLORS[mode], lw=1.2)
    axs[0].axhline(
        config["policy"]["max_landmark_age_s"], color=VIOLATION, ls=":", label="Stale-age threshold"
    )
    axs[0].set_ylabel("Age of accepted landmark data [s]")
    axs[0].legend(fontsize=9)
    axs[1].set(xlabel="Simulation time [s]", ylabel="Actual stored speed [m/s]")
    for ax in axs:
        outage(ax, scenario)
    save_figure(fig, output, "05_localization_health")

    fig, axs = plt.subplots(1, 2, figsize=(12, 5.8))
    fig.subplots_adjust(top=0.79, bottom=0.20, left=0.075, right=0.97, wspace=0.25)
    heading(
        fig,
        "06  /  Reporting completion does not prove arrival",
        "Dots: each seed • model coverage and truth-pose home error answer different questions • all aborted runs retained",
    )
    for offset, mode in enumerate(MODES):
        x = np.arange(len(cases)) + (offset - 0.5) * 0.18
        for seed_value in config["seeds"]:
            values = [outcomes[f"{case}__{seed_value}__{mode}"] for case in cases]
            axs[0].scatter(
                x,
                [m["estimated_coverage_fraction"] for m in values],
                color=COLORS[mode],
                edgecolors=TEXT,
                lw=0.4,
                s=35,
                label=LABELS[mode] if seed_value == config["seeds"][0] else None,
            )
            for j, metric in enumerate(values):
                axs[1].scatter(
                    x[j],
                    metric["actual_home_error_m"],
                    marker="o" if metric["mission_status"] == "complete" else "x",
                    color=COLORS[mode],
                    s=40,
                )
    for ax in axs:
        ax.set(xticks=np.arange(len(cases)), xticklabels=[c.replace("_", "\n") for c in cases])
    axs[0].set(ylabel="Modelled coverage fraction", ylim=(0, 1.05))
    axs[0].legend(fontsize=8)
    axs[1].axhline(
        config["evaluation_home_tolerance_m"],
        color=REFERENCE,
        ls=":",
        label="Home audit distance tolerance",
    )
    axs[1].set_ylabel("Actual final distance from home [m]")
    axs[1].legend(fontsize=8)
    fig.text(
        0.075,
        0.035,
        "○ reports complete; × aborted. An aborted mission does not claim home arrival. Home audit also checks actual final speed.",
        color=MUTED,
        fontsize=9,
    )
    save_figure(fig, output, "06_completion_audit")


def exploration_animation(output, config, logs, metadata):
    vis = config["visualization"]
    times = max((log["times"] for log in logs.values()), key=len)
    fig, axs = plt.subplots(1, 2, figsize=(11.5, 6.8))
    fig.subplots_adjust(top=0.80, bottom=0.29, left=0.06, right=0.97, wspace=0.18)
    heading(
        fig,
        "AUTONOMOUS EXPLORER  /  Decide, explore, recover",
        "Solid: actual motion • dotted: estimated motion • pale path: current checked plan • squares: candidate viewpoints",
    )
    bounds = plot_bounds(config, logs)
    dynamic, paths, labels, plans, estimates = [], {}, {}, {}, {}
    for ax, mode in zip(axs, MODES, strict=True):
        map_background(ax, config, bounds)
        (paths[mode],) = ax.plot([], [], color=COLORS[mode], lw=2)
        (estimates[mode],) = ax.plot([], [], ":", color=COLORS[mode], lw=1)
        (plans[mode],) = ax.plot([], [], "--", color=REFERENCE, lw=1, alpha=0.7)
        labels[mode] = fig.text(
            0.065 + MODES.index(mode) * 0.48,
            0.17,
            "",
            va="top",
            fontsize=9,
            family="DejaVu Sans Mono",
        )
        ax.set_title(LABELS[mode], color=COLORS[mode], fontsize=12)
    clock = fig.text(0.065, 0.055, "", fontsize=11, family="DejaVu Sans Mono")
    fig.text(
        0.065,
        0.022,
        "The obstacle map is supplied. Terminal panels hold the last saved state and are labelled log ended.",
        color=MUTED,
        fontsize=8,
    )

    def update(k):
        for artist in dynamic:
            artist.remove()
        dynamic.clear()
        for ax, mode in zip(axs, MODES, strict=True):
            log = logs[mode]
            i = min(k, len(log["times"]) - 1)
            time = log["times"][i]
            paths[mode].set_data(log["states"][: i + 1, 0], log["states"][: i + 1, 1])
            estimates[mode].set_data(log["estimates"][: i + 1, 0], log["estimates"][: i + 1, 1])
            plan_events = [
                e
                for e in metadata[mode]["events"]
                if e.get("type") == "plan" and e["time_s"] <= time
            ]
            plan = np.array(plan_events[-1]["path"]) if plan_events else np.empty((0, 2))
            state = STATE_NAMES[int(log["mission_states"][i])]
            if state in {"braking", "waiting", "aborted", "complete"}:
                plan = np.empty((0, 2))
            plans[mode].set_data(plan[:, 0], plan[:, 1])
            points = log["landmark_estimates"][i]
            good = np.all(np.isfinite(points), axis=1)
            dynamic.append(
                ax.scatter(
                    points[good, 0], points[good, 1], color=COLORS[mode], marker="o", s=15, zorder=7
                )
            )
            dynamic.extend(rover(ax, log["estimates"][i], COLORS[mode], ghost=True))
            dynamic.extend(
                rover(
                    ax,
                    log["states"][i],
                    VIOLATION if log["sampled_clearance_m"][i] <= 0 else COLORS[mode],
                )
            )
            ended = k >= len(log["times"]) - 1
            labels[mode].set_text(
                f"{state.upper()}"
                + (" (log ended)" if ended else "")
                + f"\nmodel coverage {log['coverage'][i].mean():.0%}   landmark age {log['landmark_age_s'][i]:.1f}s"
            )
        clock.set_text(f"t = {times[k]:5.2f} s / {times[-1]:.2f} s")

    update.times = times
    indices = np.rint(np.linspace(0, len(times) - 1, round(vis["fps"] * vis["duration_s"]))).astype(
        int
    )
    animation_frames(
        fig,
        update,
        indices,
        output / "exploration_hero.gif",
        vis["fps"],
        review_times=[12, 20, times[-1]],
        palette_accents=[VIOLATION],
    )


def render_exploration(results, media):
    root, output = Path(results), Path(media)
    manifest = verify_exploration(root)
    config, outcomes = read_json(root / "config.json"), read_json(root / "metrics.json")
    if not {"nominal", "temporary_outage", "persistent_outage"} <= {
        s["name"] for s in config["scenarios"]
    }:
        raise ValueError(
            "The curated mission gallery requires nominal, temporary_outage and persistent_outage"
        )
    vis = config["visualization"]
    if vis["seed"] not in config["seeds"] or vis["scenario"] not in {
        s["name"] for s in config["scenarios"]
    }:
        raise ValueError("Visualization must select a recorded seed and scenario")
    output.mkdir(parents=True, exist_ok=True)
    apply_theme()
    static_exploration(root, output, config, outcomes)
    metadata = {
        mode: read_json(root / f"{vis['scenario']}__{vis['seed']}__{mode}.json") for mode in MODES
    }
    exploration_animation(output, config, logs_for(root, vis["scenario"], vis["seed"]), metadata)
    write_json(
        output / "provenance.json",
        {
            "milestone": 5,
            "results_manifest_sha256": sha256(root / "manifest.json"),
            "input_files_sha256": manifest["files_sha256"],
            "render_source_sha256": {
                name: sha256(Path(__file__).parent / name)
                for name in [
                    "exploration_render.py",
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
            "animation_semantics": "shared saved-state clock; terminal shorter logs hold their final sample with annotation",
        },
    )
    print(f"Rendered 6 exploration figures, 1 GIF and 3 review frames into {output}")
