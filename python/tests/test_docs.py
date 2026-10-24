"""The documentation's relative links and code references resolve."""

import re

from conftest import ROOT

DOCS = [ROOT / "README.md", ROOT / "CHANGELOG.md", ROOT / "firmware" / "README.md", ROOT / "data" / "PROVENANCE.md",
        *sorted((ROOT / "docs").glob("*.md"))]


def test_relative_links_resolve():
    bad = []
    for doc in DOCS:
        for target in re.findall(r"\]\(([^)#\s]+)(?:#[^)]*)?\)", doc.read_text()):
            if target.startswith(("http://", "https://", "mailto:")):
                continue
            if not (doc.parent / target).exists():
                bad.append(f"{doc.relative_to(ROOT)} -> {target}")
    assert not bad, bad


def test_backticked_repository_paths_exist():
    """`firmware/src/ks_negotiate.c`-style references in the docs must exist."""
    pat = re.compile(r"`((?:firmware|python|scripts|data|config|docs|figures)/[\w./-]+?)(?::[\w:]+)?`")
    bad = []
    for doc in DOCS:
        for path in pat.findall(doc.read_text()):
            if any(ch in path for ch in "*<>") or "/build" in path or path.startswith("data/raw"):
                continue                                  # globs, build outputs, the excluded raw-data dir
            if not (ROOT / path.rstrip("/")).exists():
                bad.append(f"{doc.relative_to(ROOT)} -> {path}")
    assert not bad, bad
