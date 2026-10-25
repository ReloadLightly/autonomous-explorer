"""Three commands: reproduce everything, simulate only, or render saved logs."""

import argparse
import json

from .evidence import run_experiments, verify_evidence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["reproduce", "simulate", "render", "verify"])
    parser.add_argument("--config", default="configs/milestone.json")
    parser.add_argument("--results", default="results/milestone")
    parser.add_argument("--media", default="media")
    args = parser.parse_args()
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
