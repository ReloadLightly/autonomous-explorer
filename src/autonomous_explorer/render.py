"""Render ONLY saved logs. No controller, integrator, or simulation is invoked."""

from pathlib import Path

import numpy as np
from PIL import Image, ImageColor

from .evidence import load_log, read_json, sha256, verify_evidence, write_json
from .model import wrap_angle
from .theme import (
    AMBER,
    COLORS,
    CYAN,
    LABELS,
    MUTED,
    NAVY,
    REFERENCE,
    STYLES,
    TEXT,
    apply_theme,
    heading,
    plt,
    rover,
    spatial,
)


def bounds_for(logs):
    points = np.concatenate(
        [log["states"][:, :2] for log in logs.values()]
        + [next(iter(logs.values()))["reference_states"][:, :2]]
    )
    low, high = points.min(axis=0), points.max(axis=0)
    width = max(high - low) + 1.8
    center = (low + high) / 2
    return [
        center[0] - width / 2,
        center[0] + width / 2,
        center[1] - width / 2,
        center[1] + width / 2,
    ]


def mark_disturbance(ax, scenario):
    if scenario["kind"] == "yaw_bias":
        ax.axvspan(scenario["start_s"], scenario["end_s"], color=AMBER, alpha=0.12)
        for t in (scenario["start_s"], scenario["end_s"]):
            ax.axvline(t, color=AMBER, alpha=0.4, lw=0.8, ls=":")


def bias_active(scenario, time):
    return scenario["kind"] == "yaw_bias" and scenario["start_s"] <= time < scenario["end_s"]


def save_figure(fig, destination, name, svg=False):
    fig.savefig(destination / (name + ".png"), dpi=150)
    if svg:
        fig.savefig(destination / (name + ".svg"))
    plt.close(fig)


def path_background(ax, reference, bounds):
    ax.plot(reference[:, 0], reference[:, 1], ":", color=REFERENCE, lw=1.6, label="Reference")
    ax.scatter(*reference[0, :2], marker="o", s=28, facecolors="none", edgecolors=REFERENCE)
    ax.scatter(*reference[-1, :2], marker="X", s=38, color=REFERENCE)
    spatial(ax, bounds)


def draw_paths(ax, logs, end=None, bodies=True):
    for name, log in logs.items():
        states = log["states"]
        k = len(states) - 1 if end is None else min(end, len(states) - 1)
        ax.plot(
            states[: k + 1, 0],
            states[: k + 1, 1],
            STYLES[name],
            color=COLORS[name],
            label=LABELS[name],
            lw=2,
        )
        if bodies:
            rover(ax, states[k], COLORS[name])


def static_figures(root, destination, config, logs, all_logs, outcomes):
    ref_log = logs["feedback"]
    t, ref = ref_log["times"], ref_log["reference_states"]
    scenario = next(
        s for s in config["scenarios"] if s["name"] == config["visualization"]["scenario"]
    )
    bounds = bounds_for(logs)
    # 01 is explicitly conceptual, not a measured motion trace.
    fig, ax = plt.subplots(figsize=(10, 6))
    fig.subplots_adjust(top=0.82, bottom=0.13, left=0.10, right=0.88)
    heading(
        fig,
        "01  /  A rover in the world",
        "Conceptual state diagram • same oriented body used throughout the gallery",
    )
    pose = [1.3, 1.0, 0.5, 0.55]
    rover(ax, pose)
    ax.annotate("", (2.3, 0), (0, 0), arrowprops={"arrowstyle": "->", "color": TEXT})
    ax.annotate("", (0, 2.1), (0, 0), arrowprops={"arrowstyle": "->", "color": TEXT})
    ax.text(2.3, -0.13, "World x", ha="right")
    ax.text(0.04, 2.08, "World y")
    ax.plot([pose[0], pose[0]], [0, pose[1]], ":", color=MUTED)
    ax.plot([0, pose[0]], [pose[1], pose[1]], ":", color=MUTED)
    ax.text(1.3, -0.13, "x", ha="center")
    ax.text(-0.1, 1, "y", va="center")
    forward = np.array([np.cos(pose[3]), np.sin(pose[3])])
    lateral = np.array([-forward[1], forward[0]])
    origin = np.array(pose[:2])
    ax.annotate(
        "",
        origin + 1.0 * forward,
        origin,
        color=CYAN,
        arrowprops={"arrowstyle": "->", "color": CYAN},
        fontsize=11,
    )
    ax.annotate(
        "",
        origin + 0.72 * lateral,
        origin,
        color=MUTED,
        arrowprops={"arrowstyle": "->", "color": MUTED},
        fontsize=10,
    )
    ax.text(*(origin + 1.05 * forward), "v, a\nbody x", color=CYAN, fontsize=11)
    ax.text(*(origin + 0.80 * lateral), "body y", color=MUTED, fontsize=10, ha="right")
    angles = np.linspace(0, pose[3], 30)
    ax.plot(pose[0] + 0.55 * np.cos(angles), pose[1] + 0.55 * np.sin(angles), color=AMBER)
    ax.text(1.92, 1.13, "θ", color=AMBER, fontsize=16)
    ax.text(0.85, 0.45, "(x, y) • axle centre\nω = dθ/dt [rad/s]", fontsize=11)
    ax.text(0.2, 2.38, "state = (x, y, v, θ)     control = (a, ω)", fontsize=13)
    ax.set(xlim=(-0.3, 2.8), ylim=(-0.3, 2.65))
    ax.set_aspect("equal")
    ax.axis("off")
    save_figure(fig, destination, "01_robot_geometry", True)

    # 02 measured dynamics rollouts, not hand-drawn paths.
    primitive = load_log(root / "primitives.npz")
    names = ["straight", "turn", "circle", "composed"]
    titles = [
        "Straight • ω = 0",
        "Turning • ω = 0.35 rad/s",
        "Circle • ω = 0.50 rad/s",
        "Composed • acceleration + turns",
    ]
    pts = np.concatenate([primitive[n + "_states"][:, :2] for n in names])
    low, high = pts.min(axis=0) - 0.55, pts.max(axis=0) + 0.55
    b = [low[0], high[0], low[1], high[1]]
    fig, axs = plt.subplots(2, 2, figsize=(11, 9))
    fig.subplots_adjust(top=0.85, hspace=0.38, wspace=0.25, bottom=0.08)
    heading(
        fig,
        "02  /  Commands become motion",
        "Saved RK4 rollouts • v(0) = 0.5 m/s • identical world scale in every panel",
    )
    for ax, name, title in zip(axs.flat, names, titles, strict=False):
        s = primitive[name + "_states"]
        ax.plot(s[:, 0], s[:, 1], color=CYAN)
        for k in [0, len(s) // 2, len(s) - 1]:
            rover(ax, s[k], ghost=k != len(s) - 1)
        spatial(ax, b)
        ax.set_title(title, fontsize=11)
    save_figure(fig, destination, "02_motion_primitives")

    # 03 saved error study against an analytical solution.
    study = load_log(root / "integration.npz")
    fig, ax = plt.subplots(figsize=(9, 5.5))
    fig.subplots_adjust(top=0.82, bottom=0.15, left=0.13, right=0.95)
    heading(
        fig,
        "03  /  How much integration error?",
        "Constant turn: v = 0.8 m/s, ω = 0.7 rad/s • final position error at t = 4 s",
    )
    for method, color, marker in [("euler", AMBER, "s"), ("rk4", CYAN, "o")]:
        errors = study[method + "_error"]
        slope = np.polyfit(np.log(study["dt"]), np.log(errors), 1)[0]
        ax.loglog(
            study["dt"],
            errors,
            marker + "-",
            color=color,
            label=f"{method.upper()} • fitted order {slope:.2f}",
        )
    ax.set(xlabel="Integration timestep Δt [s]", ylabel="Final position error [m]")
    ax.set_xticks(study["dt"], [f"{dt:g}" for dt in study["dt"]])
    ax.minorticks_off()
    ax.legend(loc="upper left")
    save_figure(fig, destination, "03_integration_accuracy", True)

    # 04 reference path and derivatives reconstructed from logged reference arrays.
    fig = plt.figure(figsize=(12, 6))
    grid = fig.add_gridspec(
        2, 2, width_ratios=[1.2, 1], top=0.82, bottom=0.13, hspace=0.55, wspace=0.35
    )
    ax = fig.add_subplot(grid[:, 0])
    path_background(ax, ref, bounds)
    rover(ax, ref[0], REFERENCE, ghost=True)
    rover(ax, ref[-1], REFERENCE, ghost=True)
    ax.annotate(
        f"Start: θ = {ref[0, 3]:.2f} rad",
        ref[0, :2],
        xytext=(22, 12),
        textcoords="offset points",
        fontsize=10,
    )
    ax.annotate(
        f"End: θ = {ref[-1, 3]:.2f} rad",
        ref[-1, :2],
        xytext=(-145, -12),
        textcoords="offset points",
        fontsize=10,
    )
    for k in np.rint(np.array([0.16, 0.40, 0.64, 0.84]) * (len(t) - 1)).astype(int):
        ax.scatter(*ref[k, :2], s=18, color=CYAN)
        ax.annotate(
            f"{t[k]:g} s", ref[k, :2], xytext=(5, -14), textcoords="offset points", fontsize=9
        )
    speed_ax = fig.add_subplot(grid[0, 1])
    speed_ax.plot(t, ref[:, 2], color=CYAN)
    speed_ax.set(xlabel="Time [s]", ylabel="Reference speed [m/s]")
    v, a = ref_log["reference_velocity"], ref_log["reference_acceleration"]
    curvature = (v[:, 0] * a[:, 1] - v[:, 1] * a[:, 0]) / ref[:, 2] ** 3
    curve_ax = fig.add_subplot(grid[1, 1])
    curve_ax.plot(t, curvature, color=AMBER)
    curve_ax.set(xlabel="Time [s]", ylabel="Signed curvature [1/m]")
    heading(
        fig,
        "04  /  A cubic path with endpoint velocities",
        "Flat outputs x(t), y(t) determine the full reference • feasible interpolation, no optimality claim",
    )
    save_figure(fig, destination, "04_reference_construction", True)

    # 05 entire recorded comparison with interval boundaries positioned in the world.
    fig, ax = plt.subplots(figsize=(9, 8))
    fig.subplots_adjust(top=0.83, left=0.12, right=0.93, bottom=0.10)
    heading(
        fig,
        "05  /  One reference, two executions",
        "Yaw-bias case • starts ○ and target × • oriented bodies show actual final headings",
    )
    path_background(ax, ref, bounds)
    draw_paths(ax, logs)
    boundaries = (
        [(scenario["start_s"], "bias begins"), (scenario["end_s"], "bias ends")]
        if scenario["kind"] == "yaw_bias"
        else []
    )
    for time, event in boundaries:
        label = f"{event} • {time:g} s"
        k = int(np.argmin(abs(t - time)))
        ax.scatter(*ref[k, :2], s=50, marker="D", edgecolors=AMBER, facecolors="none", zorder=9)
        ax.annotate(
            label,
            ref[k, :2],
            xytext=(-90, 17),
            textcoords="offset points",
            fontsize=10,
            arrowprops={"arrowstyle": "-", "color": AMBER},
            color=AMBER,
        )
    ax.legend(loc="lower right")
    save_figure(fig, destination, "05_trajectory_comparison", True)

    # 06 four snapshots of the same records, fixed bounds, genuine history only.
    fig, axs = plt.subplots(1, 4, figsize=(16, 5.5))
    fig.subplots_adjust(top=0.80, bottom=0.18, left=0.055, right=0.98, wspace=0.25)
    heading(
        fig,
        "06  /  The disturbance has a history",
        "Same recorded comparison at four times • solid/dashed trails stop at the snapshot time",
    )
    for ax, time in zip(axs, config["visualization"]["snapshot_times_s"], strict=False):
        k = int(np.argmin(abs(t - time)))
        path_background(ax, ref, bounds)
        draw_paths(ax, logs, k)
        rover(ax, ref[k], REFERENCE, ghost=True)
        ax.set_title(
            f"t = {t[k]:g} s" + (" • bias active" if bias_active(scenario, t[k]) else ""),
            fontsize=11,
        )
        if ax != axs[0]:
            ax.set_ylabel("")
    fig.text(
        0.055,
        0.07,
        "Cyan solid: feedback     Amber dashed: open loop     Pale dotted: reference + current ghost",
        color=MUTED,
        fontsize=11,
    )
    save_figure(fig, destination, "06_temporal_progression")

    # 07 norm and signed position components; shaded time spans align across rows.
    fig, axs = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    fig.subplots_adjust(top=0.84, left=0.10, right=0.97, bottom=0.09, hspace=0.22)
    heading(
        fig,
        "07  /  Position error over time",
        "Yaw-bias case • shaded interval is the delivered disturbance • error = actual − reference",
    )
    for name, log in logs.items():
        e = log["states"][:, :2] - log["reference_states"][:, :2]
        for ax, values in zip(axs, [np.linalg.norm(e, axis=1), e[:, 0], e[:, 1]], strict=False):
            ax.plot(log["times"], values, STYLES[name], color=COLORS[name], label=LABELS[name])
    for ax, label in zip(axs, ["Position norm [m]", "x error [m]", "y error [m]"], strict=False):
        mark_disturbance(ax, scenario)
        ax.set_ylabel(label)
    axs[0].legend(loc="upper left")
    axs[-1].set_xlabel("Simulation time [s]")
    save_figure(fig, destination, "07_position_error", True)

    # 08 headings unwrapped for display, wrapped for the actual error metric.
    fig, axs = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    fig.subplots_adjust(top=0.84, left=0.10, right=0.97, bottom=0.09, hspace=0.22)
    heading(
        fig,
        "08  /  Tracking speed and orientation",
        "Heading traces are continuous (unwrapped) • heading error is wrapped to [−π, π)",
    )
    axs[0].plot(t, ref[:, 2], ":", color=REFERENCE, label="Reference")
    axs[1].plot(t, np.unwrap(ref[:, 3]), ":", color=REFERENCE)
    for name, log in logs.items():
        state, desired = log["states"], log["reference_states"]
        values = [state[:, 2], np.unwrap(state[:, 3]), wrap_angle(state[:, 3] - desired[:, 3])]
        for ax, y in zip(axs, values, strict=False):
            ax.plot(log["times"], y, STYLES[name], color=COLORS[name], label=LABELS[name])
    for ax, label in zip(
        axs, ["Speed [m/s]", "Heading [rad]", "Heading error [rad]"], strict=False
    ):
        mark_disturbance(ax, scenario)
        ax.set_ylabel(label)
    axs[0].legend(loc="lower center", bbox_to_anchor=(0.5, 1.02), ncol=3)
    axs[-1].set_xlabel("Simulation time [s]")
    save_figure(fig, destination, "08_state_tracking")

    # 09 commanded versus applied; no hidden actuator model.
    fig, axs = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True)
    fig.subplots_adjust(top=0.81, left=0.10, right=0.97, bottom=0.11, hspace=0.30)
    heading(
        fig,
        "09  /  The correction costs control effort",
        (
            "No actuator limits • requested = applied; markers verify applied samples"
            if config["actuator_limits"] is None
            else "Symmetric actuator limits • requested: lines • applied: markers • limits: dotted"
        ),
    )
    for name, log in logs.items():
        for j, ax in enumerate(axs):
            ax.stairs(
                log["requested"][:, j],
                log["times"],
                color=COLORS[name],
                ls=STYLES[name],
                label=LABELS[name] + " requested",
                lw=1.6,
            )
            indices = np.arange(0, len(log["applied"]), 25)
            ax.scatter(
                log["times"][indices],
                log["applied"][indices, j],
                facecolors="none",
                edgecolors=COLORS[name],
                s=23,
                marker="o" if name == "feedback" else "s",
                label=LABELS[name] + " applied (samples)",
                zorder=4,
            )
            if config["actuator_limits"] is not None:
                for sign in [-1, 1]:
                    ax.axhline(sign * config["actuator_limits"][j], color=MUTED, ls=":")
    for ax, label in zip(axs, ["Acceleration [m/s²]", "Yaw rate [rad/s]"], strict=False):
        mark_disturbance(ax, scenario)
        ax.set_ylabel(label)
    axs[0].legend(ncol=2, fontsize=8, loc="upper center")
    axs[-1].set_xlabel("Simulation time [s]")
    save_figure(fig, destination, "09_control_activity")

    # 10 actual delivered arrays, including the seeded realization.
    fig, axs = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True)
    fig.subplots_adjust(top=0.81, left=0.11, right=0.97, bottom=0.11, hspace=0.35)
    heading(
        fig,
        "10  /  What the simulator actually received",
        f"Same arrays delivered to both controllers • piecewise constant on each {config['dt_s']:g} s interval",
    )
    for ax, j, label in zip(
        axs, [0, 1], ["Speed offset [m/s]", "Yaw-rate offset [rad/s]"], strict=False
    ):
        for case, color, style in [("seeded", MUTED, "-"), ("yaw_bias", AMBER, "--")]:
            log = all_logs[case]["feedback"]
            ax.stairs(
                log["disturbance"][:, j],
                log["times"],
                color=color,
                lw=0.8 if case == "seeded" else 2.2,
                ls=style,
                label=(
                    f"Seed {next(s['seed'] for s in config['scenarios'] if s['name'] == case)} (Gaussian)"
                    if case == "seeded"
                    else "Bounded yaw bias"
                ),
            )
        mark_disturbance(ax, scenario)
        ax.set_ylabel(label)
    axs[0].legend(loc="lower left", ncol=2)
    axs[-1].set_xlabel("Simulation time [s]")
    save_figure(fig, destination, "10_disturbance_profile")

    # 11 scenario breadth: all declared outcomes, no selected animation alone.
    fig, axs = plt.subplots(1, 2, figsize=(12, 5.5))
    fig.subplots_adjust(top=0.80, bottom=0.20, left=0.09, right=0.97, wspace=0.30)
    heading(
        fig,
        "11  /  Three declared comparisons",
        "Sample RMS and finite-horizon endpoint errors • log scale retains nominal discretization error",
    )
    cases = list(all_logs)
    for ax, key, title in zip(
        axs,
        ["rms_position_m", "final_position_m"],
        ["RMS position error [m]", "Final position error [m]"],
        strict=False,
    ):
        x = np.arange(len(cases))
        for name, offset in [("open_loop", -0.15), ("feedback", 0.15)]:
            values = [outcomes[f"{case}__{name}"][key] for case in cases]
            # Failed runs have no final error: show a text marker rather than inventing a value.
            for i, value in enumerate(values):
                if value is None:
                    ax.text(i + offset, 0.01, "failed", color=COLORS[name], rotation=90)
            values = [np.nan if v is None else v for v in values]
            ax.scatter(
                x + offset,
                values,
                s=65,
                marker="s" if name == "open_loop" else "o",
                color=COLORS[name],
                label=LABELS[name],
            )
        ax.set(
            xticks=x, xticklabels=[s.replace("_", " ") for s in cases], ylabel=title, yscale="log"
        )
        ax.legend(loc="lower right")
    save_figure(fig, destination, "11_scenario_outcomes", True)


def animation_frames(fig, update, indices, output, fps):
    frames = []
    # Fixed palette avoids per-frame colour flicker and keeps GIFs compact.
    palette = None
    for index in indices:
        update(int(index))
        fig.canvas.draw()
        rgb = Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy())
        if palette is None:
            palette = rgb.quantize(colors=112, method=Image.Quantize.MEDIANCUT)
            # At t=0 one body can obscure the other. Reserve every theme accent
            # explicitly so later amber/cyan traces cannot collapse to gray.
            colors = palette.getpalette()[: 112 * 3]
            background = np.array(ImageColor.getrgb(NAVY))
            for accent in [AMBER, CYAN, REFERENCE, MUTED, TEXT]:
                foreground = np.array(ImageColor.getrgb(accent))
                for alpha in [0.25, 0.5, 0.75, 1.0]:
                    colors.extend(
                        np.rint(alpha * foreground + (1 - alpha) * background).astype(int).tolist()
                    )
            palette.putpalette(colors + [0] * (768 - len(colors)))
        frames.append(rgb.quantize(palette=palette, dither=Image.Dither.NONE))
    # GIF stores centiseconds. Distribute 80/90 ms intervals at 12 fps rather
    # than truncating every 83 ms frame to 80 ms and accelerating playback.
    centisecond_edges = np.rint(np.arange(len(frames) + 1) * 100 / fps).astype(int)
    durations = (np.diff(centisecond_edges) * 10).tolist()
    durations[-1] += 800
    frames[0].save(
        output,
        save_all=True,
        append_images=frames[1:],
        duration=durations,
        loop=0,
        optimize=True,
        disposal=1,
    )
    # Retain full-size representative frames for an inspectable animation review.
    for label, index in [("before", 6.0), ("during", 10.0), ("after", 20.0)]:
        # update is also used directly so these are exact recorded timestamps.
        actual_index = int(np.argmin(abs(update.times - index)))
        update(actual_index)
        fig.savefig(output.parent / (output.stem + f"_{label}.png"), dpi=100)
    plt.close(fig)


def animations(destination, config, logs):
    t = logs["feedback"]["times"]
    ref = logs["feedback"]["reference_states"]
    b = bounds_for(logs)
    vis = config["visualization"]
    scenario = next(s for s in config["scenarios"] if s["name"] == vis["scenario"])
    fps = vis["fps"]
    indices = np.rint(np.linspace(0, len(t) - 1, round(fps * vis["duration_s"]))).astype(int)
    bias_description = (
        f"shared yaw bias: {scenario['yaw_rate_radps']:+g} rad/s, "
        f"{scenario['start_s']:g} ≤ t < {scenario['end_s']:g} s"
        if scenario["kind"] == "yaw_bias"
        else f"scenario: {scenario['name']}"
    )

    fig, axs = plt.subplots(1, 2, figsize=(10.4, 6.0))
    fig.subplots_adjust(top=0.80, bottom=0.17, left=0.07, right=0.97, wspace=0.22)
    heading(
        fig,
        "AUTONOMOUS EXPLORER",
        "01 / Trajectory Tracking Foundations     •     " + bias_description,
    )
    trails, dynamic = {}, []
    for ax, name in zip(axs, ["open_loop", "feedback"], strict=False):
        path_background(ax, ref, b)
        (trails[name],) = ax.plot([], [], STYLES[name], color=COLORS[name], lw=2.3)
        ax.set_title(LABELS[name], color=COLORS[name])
    clock = fig.text(0.07, 0.075, "", fontsize=12, family="DejaVu Sans Mono")
    signal = fig.text(0.52, 0.075, "", fontsize=11, color=AMBER)
    fig.text(
        0.07,
        0.025,
        f"Dotted: reference • ghost: desired pose • trails: executed history • playback ≈{t[-1] / vis['duration_s']:g}×",
        color=MUTED,
        fontsize=9,
    )

    def update_hero(k):
        for artist in dynamic:
            artist.remove()
        dynamic.clear()
        for ax, name in zip(axs, ["open_loop", "feedback"], strict=False):
            state = logs[name]["states"]
            i = min(k, len(state) - 1)
            trails[name].set_data(state[: i + 1, 0], state[: i + 1, 1])
            dynamic.extend(rover(ax, ref[k], REFERENCE, ghost=True))
            dynamic.extend(rover(ax, state[i], COLORS[name]))
        clock.set_text(f"SIMULATION  {t[k]:05.2f} / {t[-1]:05.2f} s")
        active = bias_active(scenario, t[k])
        signal.set_text(
            f"YAW BIAS ACTIVE  {scenario['yaw_rate_radps']:+g} rad/s"
            if active
            else "Yaw bias inactive"
        )
        signal.set_color(AMBER if active else MUTED)

    update_hero.times = t
    animation_frames(fig, update_hero, indices, destination / "hero.gif", fps)

    fig = plt.figure(figsize=(11.0, 6.0))
    grid = fig.add_gridspec(
        3,
        2,
        width_ratios=[1.05, 1.35],
        top=0.80,
        bottom=0.13,
        left=0.07,
        right=0.97,
        hspace=0.48,
        wspace=0.30,
    )
    world = fig.add_subplot(grid[:, 0])
    path_background(world, ref, b)
    axes = [fig.add_subplot(grid[i, 1]) for i in range(3)]
    heading(
        fig,
        "Replay the cause and the correction",
        "Yaw-bias case • synchronized pose, position error, acceleration and angular-rate histories",
    )
    trace_lines, world_lines, cursors, dynamic = {}, {}, [], []
    for name, log in logs.items():
        (world_lines[name],) = world.plot(
            [], [], STYLES[name], color=COLORS[name], label=LABELS[name]
        )
        series = [
            np.linalg.norm(log["states"][:, :2] - log["reference_states"][:, :2], axis=1),
            log["applied"][:, 0],
            log["applied"][:, 1],
        ]
        for j, (ax, values) in enumerate(zip(axes, series, strict=False)):
            (trace_lines[name, j],) = ax.plot(
                [],
                [],
                STYLES[name],
                color=COLORS[name],
                lw=1.5,
                drawstyle="steps-post" if j else "default",
            )
            # Full-record ranges stay fixed; traces reveal only the recorded past.
            lo, hi = np.min(values), np.max(values)
            trace_lines[name, j]._values = values
            ax.update_datalim([[0, lo], [t[-1], hi]])
    for ax, label in zip(axes, ["Position error [m]", "a [m/s²]", "ω [rad/s]"], strict=False):
        ax.autoscale_view()
        ax.set_xlim(0, t[-1])
        ax.set_ylabel(label, fontsize=10)
        ax.tick_params(labelsize=9)
        mark_disturbance(ax, scenario)
        cursors.append(ax.axvline(0, color=TEXT, lw=1.1))
    axes[-1].set_xlabel("Simulation time [s]")
    world.legend(loc="lower right", fontsize=8)
    clock = fig.text(0.07, 0.04, "", color=MUTED, fontsize=11, family="DejaVu Sans Mono")

    def update_replay(k):
        for artist in dynamic:
            artist.remove()
        dynamic.clear()
        for name, log in logs.items():
            i = min(k, len(log["states"]) - 1)
            states = log["states"]
            world_lines[name].set_data(states[: i + 1, 0], states[: i + 1, 1])
            dynamic.extend(rover(world, states[i], COLORS[name]))
            for j in range(3):
                line = trace_lines[name, j]
                count = min(k + 1 if j == 0 else k, len(line._values))
                line.set_data(log["times"][:count], line._values[:count])
        dynamic.extend(rover(world, ref[k], REFERENCE, ghost=True))
        for cursor in cursors:
            cursor.set_xdata([t[k], t[k]])
        clock.set_text(
            f"t = {t[k]:05.2f} s   |   "
            + ("BIAS ACTIVE" if bias_active(scenario, t[k]) else "bias inactive")
        )

    update_replay.times = t
    animation_frames(fig, update_replay, indices, destination / "diagnostic_replay.gif", fps)


def render_gallery(results, output):
    root, destination = Path(results), Path(output)
    manifest = verify_evidence(root)
    destination.mkdir(parents=True, exist_ok=True)
    config, outcomes = read_json(root / "config.json"), read_json(root / "metrics.json")
    all_logs = {}
    for entry in manifest["runs"]:
        all_logs.setdefault(entry["scenario"], {})[entry["controller"]] = load_log(
            root / entry["log"]
        )
    logs = all_logs[config["visualization"]["scenario"]]
    if any(len(log["times"]) != len(logs["feedback"]["times"]) for log in logs.values()):
        raise ValueError(
            "Selected animation requires equally complete logs; select a completed case"
        )
    apply_theme()
    static_figures(root, destination, config, logs, all_logs, outcomes)
    animations(destination, config, logs)
    write_json(
        destination / "provenance.json",
        {
            "results_manifest_sha256": sha256(root / "manifest.json"),
            "metrics_sha256": sha256(root / "metrics.json"),
            "input_files_sha256": manifest["files_sha256"],
            "render_source_sha256": {
                p.name: sha256(p)
                for p in Path(__file__).parent.glob("*.py")
                if p.name in ["render.py", "theme.py"]
            },
            "animation_source": vis_source(config),
            "outputs_sha256": {
                p.name: sha256(p)
                for p in sorted(destination.iterdir())
                if p.suffix in [".gif", ".png", ".svg"]
            },
        },
    )
    print(f"Rendered 11 figures, 2 GIFs, and 6 review frames from saved logs into {destination}")


def vis_source(config):
    return {
        "scenario": config["visualization"]["scenario"],
        "sample_selection": "nearest logged state",
        "fps": config["visualization"]["fps"],
        "simulation_duration_s": config["duration_s"],
        "playback_duration_s": config["visualization"]["duration_s"],
        "final_hold_ms": 800,
    }
