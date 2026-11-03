"""Model extension; see ../../docs/MODEL_SCOPE.md."""
from vocbf.__main__ import main
if __name__ == "__main__":
    raise SystemExit(main(["run", "--model", "dynamic_unicycle", "--output", "runs/dynamic_unicycle"]))
