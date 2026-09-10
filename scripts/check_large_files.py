"""Reject staged blobs larger than 10 MiB.

This check inspects the staged Git object, not just the working-tree file, so
it also catches a large file that was staged before the hook ran.
"""

from __future__ import annotations

import argparse
import subprocess
import sys


LIMIT_BYTES = 10 * 1024 * 1024


def staged_paths() -> list[str]:
    result = subprocess.run(
        ["git", "diff", "--cached", "--name-only", "-z", "--diff-filter=ACMR"],
        check=True,
        stdout=subprocess.PIPE,
    )
    return [path for path in result.stdout.decode("utf-8", errors="surrogateescape").split("\0") if path]


def staged_size(path: str) -> int:
    result = subprocess.run(
        ["git", "cat-file", "-s", f":{path}"],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    )
    return int(result.stdout.strip())


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--staged", action="store_true", help="check staged blobs")
    args = parser.parse_args()
    if not args.staged:
        parser.error("use --staged")
    oversized = []
    for path in staged_paths():
        try:
            size = staged_size(path)
        except subprocess.CalledProcessError:
            continue
        if size > LIMIT_BYTES:
            oversized.append((path, size))
    if oversized:
        print("Commit rifiutato: file staged oltre il limite di 10 MiB:")
        for path, size in oversized:
            print(f"  {path} ({size / (1024 * 1024):.1f} MiB)")
        print("Rimuovi il file dallo staging oppure usa un artefatto esterno/LFS.")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
