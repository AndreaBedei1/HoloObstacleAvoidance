#!/usr/bin/env python3
"""Count ROBOVIS characters reproducibly from a submission PDF.

The official regular-paper rule is 10,000--50,000 characters excluding
white space and including references, tables, graphs, and appendices:
https://robovis.scitevents.org/Guidelines.aspx

This audit counts every non-whitespace Unicode character extractable from
every PDF page after NFKC normalisation. It therefore includes the title,
running heads, page numbers, body text, captions, tables, vector-graphic
labels, references, and appendices. It excludes whitespace, PDF metadata,
comments, drawing operators, and pixels in raster graphics. Any text baked
into an image must be audited separately because a PDF text extractor cannot
count it. The script prints a SHA-256 digest so that the result is tied to an
exact file.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import unicodedata

from pypdf import PdfReader


OFFICIAL_MINIMUM = 10_000
OFFICIAL_MAXIMUM = 50_000
INTERNAL_TARGET_MAXIMUM = 49_000
RULE_URL = "https://robovis.scitevents.org/Guidelines.aspx"


def audit(pdf_path: Path) -> dict[str, object]:
    reader = PdfReader(str(pdf_path))
    page_counts: list[int] = []
    for page in reader.pages:
        text = unicodedata.normalize("NFKC", page.extract_text() or "")
        page_counts.append(sum(not char.isspace() for char in text))

    total = sum(page_counts)
    digest = hashlib.sha256(pdf_path.read_bytes()).hexdigest()
    return {
        "file": pdf_path.name,
        "sha256": digest,
        "pages": len(reader.pages),
        "page_non_whitespace_characters": page_counts,
        "non_whitespace_characters": total,
        "official_minimum": OFFICIAL_MINIMUM,
        "official_maximum": OFFICIAL_MAXIMUM,
        "internal_target_maximum": INTERNAL_TARGET_MAXIMUM,
        "within_official_range": OFFICIAL_MINIMUM <= total <= OFFICIAL_MAXIMUM,
        "within_internal_target": total <= INTERNAL_TARGET_MAXIMUM,
        "rule_url": RULE_URL,
        "method": "PDF text extraction; Unicode NFKC; count characters where str.isspace() is false",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pdf", type=Path, help="submission PDF to audit")
    parser.add_argument("--json", type=Path, help="also write the audit as JSON")
    args = parser.parse_args()

    if not args.pdf.is_file():
        parser.error(f"PDF not found: {args.pdf}")

    result = audit(args.pdf)
    print(f"ROBOVIS rule: {RULE_URL}")
    print(f"PDF: {result['file']}")
    print(f"SHA-256: {result['sha256']}")
    print(f"Pages: {result['pages']}")
    print(f"Non-whitespace characters: {result['non_whitespace_characters']:,}")
    print(f"Official range: {OFFICIAL_MINIMUM:,}--{OFFICIAL_MAXIMUM:,}")
    print(f"Internal target maximum: {INTERNAL_TARGET_MAXIMUM:,}")
    print("Page counts: " + ", ".join(str(value) for value in result["page_non_whitespace_characters"]))
    print("Method: " + str(result["method"]))

    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(f"JSON: {args.json.resolve()}")

    if int(result["non_whitespace_characters"]) > OFFICIAL_MAXIMUM:
        print("ERROR: the official maximum is exceeded.", file=sys.stderr)
        return 2
    if int(result["non_whitespace_characters"]) < OFFICIAL_MINIMUM:
        print("ERROR: the official minimum is not reached.", file=sys.stderr)
        return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
