"""Run all four single-obstacle model extensions; forward CLI options."""
import sys
from vocbf.__main__ import main
if __name__ == "__main__":
    raise SystemExit(main(["demo", *sys.argv[1:]]))
