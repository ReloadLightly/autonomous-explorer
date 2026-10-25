"""Reproduce, simulate, render, or verify tracking, navigation, localization, and SLAM."""

import argparse
import json

from .evidence import run_experiments, verify_evidence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = ["reproduce", "simulate", "render", "verify"]
    parser.add_argument(
        "command",
        choices=commands
        + [prefix + name for prefix in ["plan-", "localize-", "slam-"] for name in commands],
    )
    parser.add_argument("--config")
    parser.add_argument("--results")
    parser.add_argument("--media")
    args = parser.parse_args()
    if args.command.startswith("slam-"):
        from .slam_evidence import run_slam, verify_slam

        config = args.config or "configs/slam.json"
        results = args.results or "results/slam"
        media = args.media or "media/slam"
        command = args.command.removeprefix("slam-")
        if command in {"simulate", "reproduce"}:
            print(json.dumps(run_slam(config, results), indent=2))
        if command in {"render", "reproduce"}:
            from .slam_render import render_slam

            render_slam(results, media)
        if command == "verify":
            manifest = verify_slam(results)
            print(
                f"Verified SLAM: {len(manifest['runs'])} executions and {len(manifest['replays'])} counterfactual replays."
            )
        return
    if args.command.startswith("localize-"):
        from .localization_evidence import run_localization, verify_localization

        config = args.config or "configs/localization.json"
        results = args.results or "results/localization"
        media = args.media or "media/localization"
        command = args.command.removeprefix("localize-")
        if command in {"simulate", "reproduce"}:
            print(json.dumps(run_localization(config, results), indent=2))
        if command in {"render", "reproduce"}:
            from .localization_render import render_localization

            render_localization(results, media)
        if command == "verify":
            manifest = verify_localization(results)
            print(
                f"Verified localization: {len(manifest['runs'])} runs, estimator replays, and clearance."
            )
        return
    planning = args.command.startswith("plan-")
    args.config = args.config or (
        "configs/navigation.json" if planning else "configs/milestone.json"
    )
    args.results = args.results or ("results/navigation" if planning else "results/milestone")
    args.media = args.media or ("media/navigation" if planning else "media")
    if planning:
        from .navigation_evidence import run_navigation, verify_navigation

        command = args.command.removeprefix("plan-")
        if command in {"simulate", "reproduce"}:
            print(json.dumps(run_navigation(args.config, args.results), indent=2))
        if command in {"render", "reproduce"}:
            from .navigation_render import render_navigation

            render_navigation(args.results, args.media)
        if command == "verify":
            manifest = verify_navigation(args.results)
            print(
                f"Verified navigation planning, {len(manifest['runs'])} executions, and clearance audits."
            )
        return
    if args.command in ("simulate", "reproduce"):
        print(json.dumps(run_experiments(args.config, args.results), indent=2))
    if args.command in ("render", "reproduce"):
        from .render import render_gallery

        render_gallery(args.results, args.media)
    if args.command == "verify":
        manifest = verify_evidence(args.results)
        print(f"Verified {len(manifest['runs'])} runs, checksums, and metrics.")


if __name__ == "__main__":
    main()
