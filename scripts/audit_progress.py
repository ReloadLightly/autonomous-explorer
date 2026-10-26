"""Audit milestone documentation and saved assets without rerunning experiments.

Run from any directory with Python 3.10+. Extend MILESTONES when publishing a new
milestone, and preserve the earlier entries and gallery sections.
"""

import hashlib
import json
import re
import subprocess
import sys
from collections import Counter
from pathlib import Path
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]
# Results directory, media directory, expected numbered figures, expected GIFs.
MILESTONES = (
    ("milestone", "media", 11, 2),
    ("navigation", "media/navigation", 11, 2),
    ("localization", "media/localization", 6, 1),
    ("slam", "media/slam", 6, 1),
    ("exploration", "media/exploration", 6, 1),
)
MEDIA_SUFFIXES = {".png", ".svg", ".gif"}


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def markdown_body(path):
    # Links written as examples inside fenced code are not rendered links.
    return re.sub(r"^```[^\n]*\n.*?^```[^\n]*$", "", path.read_text(), flags=re.M | re.S)


def links(path):
    return re.findall(r"!?\[[^\]]*\]\(([^\s)]+)\)", markdown_body(path))


def anchors(path):
    counts = Counter()
    result = set()
    for title in re.findall(r"^#{1,6}\s+(.+?)\s*#*\s*$", markdown_body(path), re.M):
        slug = re.sub(r"[^\w\- ]", "", title.lower()).replace(" ", "-")
        number = counts[slug]
        counts[slug] += 1
        result.add(slug if number == 0 else f"{slug}-{number}")
    return result


def main():
    tracked = set(
        subprocess.check_output(["git", "-C", str(ROOT), "ls-files", "-z"]).decode().split("\0")
    ) - {""}
    errors = []

    def check(condition, message):
        if not condition:
            errors.append(message)

    def tracked_file(path):
        relative = path.relative_to(ROOT).as_posix()
        check(relative in tracked, f"Untracked evidence/documentation: {relative}")
        check(path.is_file(), f"Missing file: {relative}")
        return path.is_file()

    def check_hash(path, digest):
        if tracked_file(path):
            check(sha256(path) == digest, f"Checksum mismatch: {path.relative_to(ROOT)}")

    documents = [ROOT / name for name in sorted(tracked) if name.endswith(".md")]
    link_count = 0
    for document in documents:
        for destination in links(document):
            url = urlsplit(destination)
            if url.scheme or url.netloc:
                continue
            link_count += 1
            target = (document.parent / unquote(url.path)).resolve() if url.path else document
            if not target.is_relative_to(ROOT):
                check(
                    False, f"Link leaves repository: {document.relative_to(ROOT)} -> {destination}"
                )
                continue
            relative = target.relative_to(ROOT).as_posix()
            if target.is_dir():
                check(
                    any(name.startswith(relative + "/") for name in tracked),
                    f"Linked directory has no tracked content: {relative}",
                )
            elif tracked_file(target) and url.fragment and target.suffix == ".md":
                check(
                    unquote(url.fragment) in anchors(target),
                    f"Broken anchor: {document.relative_to(ROOT)} -> {destination}",
                )

    readme_links = set(links(ROOT / "README.md"))
    gallery_links = set(links(ROOT / "GALLERY.md"))
    registered_media = set()
    registered_results = set()
    summaries = []
    for number, (name, media_name, expected_figures, expected_gifs) in enumerate(MILESTONES, 1):
        results = ROOT / "results" / name
        media = ROOT / media_name
        manifest_path = results / "manifest.json"
        provenance_path = media / "provenance.json"
        if not (tracked_file(manifest_path) and tracked_file(provenance_path)):
            continue
        registered_results.add(manifest_path.relative_to(ROOT).as_posix())
        manifest = json.loads(manifest_path.read_text())
        provenance = json.loads(provenance_path.read_text())
        for filename, digest in manifest["files_sha256"].items():
            check_hash(results / filename, digest)
        check(
            provenance["results_manifest_sha256"] == sha256(manifest_path),
            f"Gallery uses a different results manifest: {media_name}",
        )
        check(
            provenance["input_files_sha256"] == manifest["files_sha256"],
            f"Gallery inputs differ from saved evidence: {media_name}",
        )
        if "metrics_sha256" in provenance:
            check_hash(results / "metrics.json", provenance["metrics_sha256"])
        outputs = provenance["outputs_sha256"]
        on_disk = {path.name for path in media.iterdir() if path.suffix in MEDIA_SUFFIXES}
        check(set(outputs) == on_disk, f"Media/provenance inventory differs: {media_name}")
        figures = [n for n in outputs if re.fullmatch(r"\d\d_.+\.png", n)]
        gifs = [n for n in outputs if n.endswith(".gif")]
        check(len(figures) == expected_figures, f"Wrong figure count: {media_name}")
        check(len(gifs) == expected_gifs, f"Wrong animation count: {media_name}")
        for filename, digest in outputs.items():
            path = media / filename
            relative = path.relative_to(ROOT).as_posix()
            registered_media.add(relative)
            check_hash(path, digest)
            check(relative in gallery_links, f"Asset missing from GALLERY.md: {relative}")
            if filename in figures or filename in gifs:
                check(
                    relative in readme_links, f"Primary visual missing from README.md: {relative}"
                )
        for gif in gifs:
            for phase in ("before", "during", "after"):
                frame = f"{Path(gif).stem}_{phase}.png"
                check(frame in outputs, f"Animation review frame missing: {media_name}/{frame}")
        summaries.append(
            {
                "milestone": number,
                "name": name,
                "figures": len(figures),
                "animations": len(gifs),
                "svg_exports": sum(n.endswith(".svg") for n in outputs),
                "review_frames": sum(
                    n.endswith(("_before.png", "_during.png", "_after.png")) for n in outputs
                ),
                "media_assets": len(outputs),
                "saved_files": len(manifest["files_sha256"]),
                "executions": len(manifest["runs"]),
                "estimator_replays": len(manifest.get("replays", [])),
            }
        )
    tracked_media = {
        n for n in tracked if n.startswith("media/") and Path(n).suffix in MEDIA_SUFFIXES
    }
    check(tracked_media == registered_media, "Tracked media exists outside registered provenance")
    tracked_manifests = {n for n in tracked if re.fullmatch(r"results/[^/]+/manifest.json", n)}
    check(tracked_manifests == registered_results, "Result milestone missing from audit registry")
    if errors:
        for message in errors:
            print(message, file=sys.stderr)
        raise SystemExit(f"Progress audit failed: {len(errors)} issue(s)")
    print(
        json.dumps(
            {
                "status": "passed",
                "local_markdown_links": link_count,
                "milestones": summaries,
                "media_assets": len(registered_media),
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
