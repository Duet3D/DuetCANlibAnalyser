#!/usr/bin/env python3
"""
Regenerate the decode spec from CANlib and run the test suite.

This is the single command to run whenever CANlib changes. It:
  1. (optionally) updates the pinned CANlib checkout to its latest commit
  2. regenerates duet_can_spec.json from the headers
  3. runs the decoder unit tests

Usage:
  python regenerate.py            # regenerate from the current CANlib checkout
  python regenerate.py --update   # git-pull CANlib first, then regenerate
"""
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def run(cmd, **kw):
    print(f"$ {' '.join(str(c) for c in cmd)}")
    return subprocess.run(cmd, cwd=ROOT, check=True, **kw)


def main():
    update = "--update" in sys.argv[1:]
    if update:
        canlib = ROOT / "CANlib"
        if (canlib / ".git").exists() or (ROOT / ".gitmodules").exists():
            run(["git", "submodule", "update", "--remote", "--", "CANlib"])
        else:
            print("CANlib is not a git checkout; skipping --update.")

    run([sys.executable, "generator/generate_spec.py"])
    run([sys.executable, "tests/test_decoder.py"])
    print("\nRegeneration complete. Review/commit duet_can_spec.json.")


if __name__ == "__main__":
    main()
