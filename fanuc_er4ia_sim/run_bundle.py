"""Zip a finished pipeline run for HTTP download by remote dashboard viewers."""

from __future__ import annotations

import os
import tempfile
import threading
import zipfile
from pathlib import Path

BUNDLE_NAME = "run_bundle.zip"
FINALIZED_MARKER = "run_finalized.json"
REVIEW_VIDEOS = ("sort_table.mp4", "sort_side.mp4")
# Raw frame sequences are kept for encode_video.py but dwarf review artifacts.
SKIP_TOP_LEVEL_DIRS = frozenset({"frames", "side_frames"})


def _review_videos_ready(run_dir: Path) -> bool:
    run_dir = Path(run_dir)
    report = run_dir / "execution_report.json"
    if not report.is_file():
        return False
    report_mtime = report.stat().st_mtime
    for name in REVIEW_VIDEOS:
        path = run_dir / name
        if not path.is_file() or path.stat().st_size == 0:
            return False
        if path.stat().st_mtime < report_mtime:
            return False
    return True


def bundle_ready(run_dir: Path) -> bool:
    """True once review encodes finished and the pipeline wrote ``run_finalized.json``."""
    run_dir = Path(run_dir)
    if not _review_videos_ready(run_dir):
        return False
    return (run_dir / FINALIZED_MARKER).is_file()


def mark_run_finalized(run_dir: Path) -> Path:
    """Record that review videos were encoded; enables remote ``/bundle.zip`` download."""
    from run_artifacts import atomic_write_json, utc_now_iso

    run_dir = Path(run_dir).resolve()
    if not _review_videos_ready(run_dir):
        raise FileNotFoundError(
            "Review videos are not ready; encode sort_table.mp4 and sort_side.mp4 first"
        )
    marker = run_dir / FINALIZED_MARKER
    atomic_write_json(
        marker,
        {
            "review_videos": list(REVIEW_VIDEOS),
            "finalized_at": utc_now_iso(),
        },
    )
    return marker


def _iter_bundle_files(run_dir: Path):
    run_dir = Path(run_dir).resolve()
    for path in sorted(run_dir.rglob("*")):
        if not path.is_file():
            continue
        relative = path.relative_to(run_dir)
        if relative.parts and relative.parts[0] in SKIP_TOP_LEVEL_DIRS:
            continue
        if relative.name == BUNDLE_NAME:
            continue
        if relative.name.startswith(".run_bundle."):
            continue
        yield path, relative


def _needs_rebuild(run_dir: Path, bundle_path: Path) -> bool:
    if not bundle_path.is_file():
        return True
    bundle_mtime = bundle_path.stat().st_mtime
    for path, _ in _iter_bundle_files(run_dir):
        if path.stat().st_mtime > bundle_mtime:
            return True
    return False


def build_bundle(run_dir: Path) -> Path:
    """Write ``run_bundle.zip`` beside run artifacts; replace atomically."""
    run_dir = Path(run_dir).resolve()
    if not run_dir.is_dir():
        raise FileNotFoundError(f"Run directory not found: {run_dir}")
    if not bundle_ready(run_dir):
        raise FileNotFoundError(
            "Run bundle not ready (need execution_report.json, review MP4s, and "
            f"{FINALIZED_MARKER} from the pipeline)"
        )
    bundle_path = run_dir / BUNDLE_NAME
    fd, tmp_name = tempfile.mkstemp(
        dir=run_dir, prefix=".run_bundle.", suffix=".zip"
    )
    os.close(fd)
    tmp_path = Path(tmp_name)
    try:
        with zipfile.ZipFile(tmp_path, "w", zipfile.ZIP_DEFLATED) as archive:
            for source, relative in _iter_bundle_files(run_dir):
                archive.write(source, relative.as_posix())
        os.replace(tmp_path, bundle_path)
    except Exception:
        try:
            tmp_path.unlink(missing_ok=True)
        except OSError:
            pass
        raise
    return bundle_path


_bundle_locks: dict[Path, threading.Lock] = {}


def _lock_for(run_dir: Path) -> threading.Lock:
    key = Path(run_dir).resolve()
    lock = _bundle_locks.get(key)
    if lock is None:
        lock = threading.Lock()
        _bundle_locks[key] = lock
    return lock


def ensure_bundle(run_dir: Path) -> Path:
    """Return an up-to-date zip path, building or refreshing when artifacts change."""
    run_dir = Path(run_dir).resolve()
    bundle_path = run_dir / BUNDLE_NAME
    with _lock_for(run_dir):
        if _needs_rebuild(run_dir, bundle_path):
            return build_bundle(run_dir)
    return bundle_path


def prepare_run_download(run_dir: Path) -> Path:
    """Mark finalized, build ``run_bundle.zip``, and return the archive path."""
    run_dir = Path(run_dir).resolve()
    mark_run_finalized(run_dir)
    archive = ensure_bundle(run_dir)
    print(
        f"[run-bundle] download ready: {archive} ({archive.stat().st_size / 1e6:.1f} MB)",
        flush=True,
    )
    return archive


def _main(argv=None):
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "--mark-finalized",
        type=Path,
        metavar="RUN_DIR",
        help="write run_finalized.json after review MP4s exist (for legacy runs)",
    )
    group.add_argument(
        "--prepare-download",
        type=Path,
        metavar="RUN_DIR",
        help="mark finalized and build run_bundle.zip for remote download",
    )
    args = parser.parse_args(argv)
    if args.mark_finalized is not None:
        path = mark_run_finalized(args.mark_finalized)
        print(f"[run-bundle] marked {path.parent} finalized ({path.name})", flush=True)
    else:
        prepare_run_download(args.prepare_download)
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
