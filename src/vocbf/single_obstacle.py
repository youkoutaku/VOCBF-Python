"""Straight-line destination demo with explicitly disclosed waypoint guidance.

This is a model-extension fixture, not an original-paper experiment. The
controller equations are unchanged. First-order plants retain the existing
acceleration-filter / continuous-velocity-ramp dynamic extension.
"""
from pathlib import Path
import datetime
import hashlib
import importlib.metadata
import json
import platform
import sys

import numpy as np

from .controller import VOCBF
from .models import KINDS, Model
from .simulation import jsonable

GOAL = np.array([4.0, 0.0])
OBSTACLES = np.array([[2.0, 0.15]])
RADII = np.array([0.45])
WAYPOINTS = np.array([[1.2, -0.7], [2.8, -0.7], [4.0, 0.0]])
TITLES = dict(zip(KINDS, ['Single integrator', 'Double integrator',
                         'Kinematic unicycle', 'Dynamic unicycle']))


def initial_state(model):
    """All plants start with body center (0, 0), heading 0 and zero speed."""
    state = np.zeros(model.nx)
    if 'unicycle' in model.kind:
        state[0] = -model.offset
    return state


def simulate(kind, steps=1200, dt=0.05, guidance='waypoints'):
    if kind not in KINDS or guidance not in ('waypoints', 'direct'):
        raise ValueError('Unknown model or guidance')
    if steps <= 0 or not np.isfinite(dt) or dt <= 0:
        raise ValueError('steps and dt must be positive and finite')
    model = Model(kind)
    controller = VOCBF(model, OBSTACLES, RADII)
    state = initial_state(model)
    states = [state.copy()]
    extended = [controller.extended(state).copy()]
    controls, events, targets, hold_centers = [], [], [], []
    minimum = float(np.min(np.linalg.norm(model.center(state) - OBSTACLES, axis=1) - RADII))
    dwell, waypoint_index, outcome = 0.0, 0, 'timeout'
    for k in range(steps):
        if guidance == 'waypoints':
            if waypoint_index < 2 and np.linalg.norm(model.center(state) - WAYPOINTS[waypoint_index]) < 0.35:
                waypoint_index += 1
            target = WAYPOINTS[waypoint_index]
        else:
            target = GOAL
        control, info = controller.action(state, target, dt, k * dt)
        info.update(reference_target=target.tolist(), waypoint_index=waypoint_index if guidance == 'waypoints' else None)
        events.append(info)
        if control is None:
            outcome = info.get('status', 'no_control')
            break
        if not np.isfinite(control).all():
            outcome = 'nonfinite_control'
            break
        samples = controller.advance(state, control, dt)
        centers = np.array([model.center(z) for z in samples])
        clearance = np.linalg.norm(centers[:, None, :] - OBSTACLES[None, :, :], axis=2) - RADII
        minimum = min(minimum, float(clearance.min()))
        state = samples[-1]
        states.append(state.copy())
        extended.append(controller.extended(state).copy())
        controls.append(control.copy())
        targets.append(target.copy())
        hold_centers.append(centers)
        if controller.lifted:
            info['plant_command_end'] = controller.virtual_velocity.tolist()
        if minimum < -1e-6:
            outcome = 'sampled_collision'
            break
        dwell = dwell + dt if controller.settled(state, control, GOAL) else 0.0
        if dwell >= 0.5 - 1e-9:
            outcome = 'goal_complete'
            break
    data = dict(time=np.arange(len(states)) * dt, state=np.asarray(states),
                augmented_state=np.asarray(extended),
                control=np.asarray(controls).reshape(-1, 2),
                reference_target=np.asarray(targets).reshape(-1, 2),
                hold_centers=np.asarray(hold_centers).reshape(-1, 21, 2),
                center=np.array([model.center(z) for z in states]))
    configured_parameters = dict(controller.cfg)
    effective_parameters = dict(configured_parameters)
    if controller.model.kind == 'double_integrator':
        # The Cartesian adaptation selects two CLFs inside VOCBF.action.
        effective_parameters['clf_weights'] = [100., 1.]
    summary = dict(method='vocbf', model=kind, scenario='single_obstacle',
        model_extension=True, initial=states[0].tolist(), goal=GOAL.tolist(),
        obstacles=OBSTACLES.tolist(), inflated_radii=RADII.tolist(), offset=model.offset,
        guidance=guidance, waypoints=WAYPOINTS.tolist() if guidance == 'waypoints' else [],
        waypoint_switch_distance=0.35 if guidance == 'waypoints' else None,
        controller_parameters=dict(configured=configured_parameters, effective=effective_parameters,
            scope='Effective is the configuration passed to the QP solver. The Cartesian adaptation overrides the four configured CLF weights with its two CLF weights; other constructor settings are retained.'),
        model_bounds={key: getattr(model, key) for key in ('vmax', 'wmax', 'amax', 'alphamax')},
        dt=dt, requested_steps=steps, executed_steps=len(controls),
        duration_s=len(controls) * dt, outcome=outcome,
        final_goal_error=float(np.linalg.norm(model.center(state) - GOAL)),
        minimum_sampled_clearance=minimum, collision_sampled=minimum < -1e-6,
        settled_dwell_s=dwell, solver_fallbacks=0, dynamic_extension=controller.lifted,
        input_semantics='held filter acceleration; continuously ramped plant velocity' if controller.lifted else 'held physical acceleration',
        goal_criterion='body-center error < 0.15 m and speed < 0.08 m/s (Cartesian), or |v| and |omega| < 0.05 (unicycle), continuously at hold endpoints for 0.5 s',
        verification='21 independent DOP853 samples per hold; no continuous-time safety proof',
        python=sys.version.split()[0], platform=platform.platform())
    return jsonable(summary), data, jsonable(events)


def _write_json(path, value):
    path.write_text(json.dumps(jsonable(value), indent=2, allow_nan=False) + '\n')


def _axes(ax, summary, data):
    import matplotlib.pyplot as plt
    ax.plot([0, 4], [0, 0], '--', color='#8995a5', lw=1, label='Straight destination line')
    if summary['guidance'] == 'waypoints':
        guide = np.vstack([[0., 0.], WAYPOINTS])
        ax.plot(guide[:, 0], guide[:, 1], ':', color='#9d81b6', lw=1, label='Waypoint guide')
        ax.scatter(WAYPOINTS[:2, 0], WAYPOINTS[:2, 1], marker='+', color='#9d81b6', s=45)
    for obstacle, radius in zip(OBSTACLES, RADII):
        ax.add_patch(plt.Circle(obstacle, radius, facecolor='#c9cfd7', edgecolor='#677184', lw=1.2))
    ax.scatter(0, 0, color='#24364b', s=30, label='Start', zorder=5)
    ax.scatter(*GOAL, marker='*', color='#db9e2c', s=110, label='Goal', zorder=5)
    ax.set(xlim=(-.35, 4.45), ylim=(-1.35, .9), xlabel='x [m]', ylabel='y [m]', aspect='equal')
    ax.grid(alpha=.16)
    ax.set_title(TITLES[summary['model']] + (' (dynamic extension)' if summary['dynamic_extension'] else ''), fontsize=10)
    ax.text(.02, .96, f"{summary['outcome'].replace('_', ' ')} | {summary['duration_s']:.2f} s\n"
            f"error {summary['final_goal_error']:.3f} m | clearance {summary['minimum_sampled_clearance']:.3f} m",
            transform=ax.transAxes, va='top', fontsize=8, color='#34525b')


def render(output, records, animate=True):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    output = Path(output)
    count = len(records)
    fig, axes = plt.subplots(2 if count > 2 else 1, 2 if count > 1 else 1,
                             figsize=(11, 6.7) if count > 2 else (11 if count > 1 else 6, 4),
                             squeeze=False, layout='constrained')
    for ax, (summary, data, _) in zip(axes.flat, records):
        _axes(ax, summary, data)
        ax.plot(data['center'][:, 0], data['center'][:, 1], color='#167f79', lw=2, label='Body center')
        ax.scatter(*data['center'][-1], marker='s', color='#167f79', s=22, zorder=4)
        individual, iax = plt.subplots(figsize=(6, 3.8), layout='constrained')
        _axes(iax, summary, data)
        iax.plot(data['center'][:, 0], data['center'][:, 1], color='#167f79', lw=2, label='Body center')
        iax.legend(fontsize=7, loc='lower right')
        individual.savefig(output / summary['model'] / 'trajectory.png', dpi=160)
        plt.close(individual)
    axes.flat[0].legend(fontsize=7, loc='lower right')
    guidance_label = 'Disclosed waypoint guidance' if records[0][0]['guidance'] == 'waypoints' else 'Direct-goal diagnostic'
    fig.suptitle('VOCBF | Single obstacle, straight-line destination\n'
                 + guidance_label + '; sampled numerical trajectories', fontsize=13)
    fig.savefig(output / 'trajectories.png', dpi=170)
    plt.close(fig)
    if not animate:
        return
    fig, axes = plt.subplots(2 if count > 2 else 1, 2 if count > 1 else 1,
                             figsize=(10, 6) if count > 2 else (10 if count > 1 else 6, 4),
                             squeeze=False, layout='constrained')
    artists = []
    for ax, (summary, data, _) in zip(axes.flat, records):
        _axes(ax, summary, data)
        line, = ax.plot([], [], color='#167f79', lw=2)
        point, = ax.plot([], [], 'o', color='#167f79', ms=6)
        artists.append((line, point))
    fig.suptitle('VOCBF | ' + guidance_label + ' (approximately 2x playback)', fontsize=12)
    end_time = max(data['time'][-1] for _, data, _ in records)
    frame_times = np.linspace(0., end_time, max(2, int(np.ceil(end_time * 6)) + 1))
    def update(t):
        for (line, point), (_, data, _) in zip(artists, records):
            index = min(int(np.searchsorted(data['time'], t, side='right')), len(data['time']))
            center = data['center'][:max(1, index)]
            line.set_data(center[:, 0], center[:, 1])
            point.set_data(center[-1:, 0], center[-1:, 1])
        return [item for pair in artists for item in pair]
    movie = FuncAnimation(fig, update, frames=frame_times, blit=False)
    movie.save(output / 'animation.gif', writer=PillowWriter(fps=12), dpi=95)
    plt.close(fig)


def run(output, model='all', steps=1200, dt=0.05, guidance='waypoints', animate=True):
    out = Path(output)
    if out.exists() and any(out.iterdir()):
        raise FileExistsError('Refusing to overwrite ' + str(out))
    kinds = KINDS if model == 'all' else (model,)
    records = [simulate(kind, steps, dt, guidance) for kind in kinds]
    out.mkdir(parents=True, exist_ok=True)
    for summary, data, events in records:
        folder = out / summary['model']
        folder.mkdir()
        np.savez_compressed(folder / 'trajectory.npz', **data)
        _write_json(folder / 'summary.json', summary)
        _write_json(folder / 'events.json', events)
    render(out, records, animate)
    source_root = Path(__file__).resolve().parent
    hashes = {str(path.relative_to(source_root)): hashlib.sha256(path.read_bytes()).hexdigest()
              for path in sorted(source_root.rglob('*.py'))}
    aggregate = dict(created_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        scenario='single_obstacle', guidance=guidance,
        passed=all(s['outcome'] == 'goal_complete' and not s['collision_sampled'] for s, _, _ in records),
        results=[s for s, _, _ in records], source_sha256=hashes,
        dependency_versions={name: importlib.metadata.version(name) for name in ('numpy', 'scipy', 'casadi', 'clarabel', 'matplotlib', 'pillow')},
        scope='Illustrative model extensions; guidance is recorded explicitly; no original-figure reproduction or comparative ranking')
    _write_json(out / 'summary.json', aggregate)
    return aggregate
