"""Reproduce, simulate, render, or verify either milestone (plan-* for navigation)."""

import argparse
import json

from .evidence import run_experiments, verify_evidence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = ["reproduce", "simulate", "render", "verify"]
    parser.add_argument("command", choices=commands + ["plan-" + name for name in commands])
    parser.add_argument("--config")
    parser.add_argument("--results")
    parser.add_argument("--media")
    args = parser.parse_args()
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
