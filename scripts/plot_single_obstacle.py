"""Regenerate the single-obstacle figures from saved data without rerunning control."""
import argparse
import json
from pathlib import Path
import numpy as np
from vocbf.single_obstacle import render


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, default=root / 'evidence/single_obstacle')
    parser.add_argument('--no-animation', action='store_true')
    args = parser.parse_args()
    aggregate = json.loads((args.data / 'summary.json').read_text())
    records = []
    for entry in aggregate['results']:
        folder = args.data / entry['model']
        summary = json.loads((folder / 'summary.json').read_text())
        with np.load(folder / 'trajectory.npz', allow_pickle=False) as archive:
            data = dict(archive)
        records.append((summary, data, []))
    render(args.data, records, not args.no_animation)


if __name__ == '__main__':
    main()
