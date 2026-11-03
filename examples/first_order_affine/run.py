"""Model extension; see ../../docs/MODEL_SCOPE.md."""
from vocbf.__main__ import main
if __name__ == "__main__":
    raise SystemExit(main(["run", "--model", "single_integrator", "--output", "runs/single_integrator"]))
