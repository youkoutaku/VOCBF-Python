"""Model extension; see ../../docs/MODEL_SCOPE.md."""
from vocbf.__main__ import main
if __name__ == "__main__":
    raise SystemExit(main(["run", "--model", "double_integrator", "--output", "runs/double_integrator"]))
