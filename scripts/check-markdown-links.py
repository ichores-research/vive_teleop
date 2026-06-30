#!/usr/bin/env python3
"""Fail when a repository-local Markdown or HTML link target is missing."""

from __future__ import annotations

import re
from pathlib import Path
from urllib.parse import unquote, urlsplit


REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
MARKDOWN_LINK = re.compile(r"!?\[[^\]]*\]\(([^)]+)\)")
HTML_LINK = re.compile(r"(?:href|src)=[\"']([^\"']+)[\"']")
SKIPPED_DIRECTORIES = {".git", "Library", "Temp", "Obj", "Build", "Builds"}


def _local_path(source: Path, raw_target: str) -> Path | None:
    target = raw_target.strip()
    if target.startswith("<") and target.endswith(">"):
        target = target[1:-1]

    parsed = urlsplit(target)
    if parsed.scheme or parsed.netloc or not parsed.path:
        return None

    decoded_path = unquote(parsed.path)
    if decoded_path.startswith("/"):
        return REPOSITORY_ROOT / decoded_path.lstrip("/")
    return source.parent / decoded_path


def main() -> int:
    failures: list[str] = []
    for source in REPOSITORY_ROOT.rglob("*.md"):
        if any(part in SKIPPED_DIRECTORIES for part in source.parts):
            continue

        text = source.read_text(encoding="utf-8")
        targets = MARKDOWN_LINK.findall(text) + HTML_LINK.findall(text)
        for target in targets:
            local_path = _local_path(source, target)
            if local_path is not None and not local_path.exists():
                failures.append(
                    f"{source.relative_to(REPOSITORY_ROOT)}: "
                    f"missing link target {target!r}"
                )

    if failures:
        print("\n".join(failures))
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
