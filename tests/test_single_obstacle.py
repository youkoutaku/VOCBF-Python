"""Closed-loop regression for the public single-obstacle fixture."""
import unittest
from unittest.mock import patch
import numpy as np
from vocbf.models import KINDS, Model
from vocbf.controller import default_config
from vocbf.vocbf_intervals import solve_source_union
from vocbf.single_obstacle import GOAL, OBSTACLES, RADII, initial_state, simulate


class SingleObstacleDemo(unittest.TestCase):
    def test_straight_reference_is_blocked_and_initial_states_match(self):
        self.assertTrue(0 < OBSTACLES[0, 0] < GOAL[0])
        self.assertLess(abs(OBSTACLES[0, 1]), RADII[0])
        for kind in KINDS:
            model = Model(kind)
            state = initial_state(model)
            np.testing.assert_allclose(model.center(state), [0., 0.], atol=1e-14)
            if 'unicycle' in kind:
                self.assertEqual(state[2], 0.)

    def test_all_guided_models_settle_and_records_replay(self):
        for kind in KINDS:
            with self.subTest(model=kind):
                summary, data, events = simulate(kind)
                self.assertEqual(summary['outcome'], 'goal_complete')
                self.assertFalse(summary['collision_sampled'])
                self.assertGreater(summary['minimum_sampled_clearance'], 0.)
                self.assertLess(summary['final_goal_error'], .15)
                self.assertGreaterEqual(summary['settled_dwell_s'], .5 - 1e-9)
                count = summary['executed_steps']
                self.assertEqual(len(events), count)
                self.assertEqual(len(data['state']), count + 1)
                np.testing.assert_allclose(np.diff(data['time']), summary['dt'])
                replay_kind = {'single_integrator': 'double_integrator',
                               'kinematic_unicycle': 'dynamic_unicycle'}.get(kind, kind)
                replay = Model(replay_kind)
                for step in (0, count // 2, count - 1):
                    expected = replay.integrate(data['augmented_state'][step], data['control'][step], summary['dt'])[-1]
                    np.testing.assert_allclose(expected, data['augmented_state'][step + 1], atol=1e-10)
                minimum = (np.linalg.norm(data['hold_centers'] - OBSTACLES[0], axis=-1) - RADII[0]).min()
                self.assertAlmostEqual(minimum, summary['minimum_sampled_clearance'], places=10)
                self.assertTrue(np.isfinite(data['control']).all())

    def test_reported_parameters_match_actual_solver_configuration(self):
        for kind in KINDS:
            with self.subTest(model=kind):
                with patch('vocbf.controller.solve_source_union', wraps=solve_source_union) as solve:
                    summary, _, _ = simulate(kind, steps=1)
                solve.assert_called_once()
                actual_configuration = solve.call_args.args[5]
                reported = summary['controller_parameters']
                self.assertEqual(reported['configured'], default_config())
                self.assertEqual(reported['effective'], actual_configuration)
                self.assertEqual(len(reported['effective']['clf_weights']),
                                 len(solve.call_args.args[4]))

    def test_direct_goal_failure_is_not_labeled_success(self):
        summary, data, _ = simulate('double_integrator', guidance='direct')
        self.assertEqual(summary['guidance'], 'direct')
        self.assertEqual(summary['outcome'], 'sampled_collision')
        self.assertTrue(summary['collision_sampled'])
        self.assertLess(summary['minimum_sampled_clearance'], 0.)
        self.assertGreater(np.linalg.norm(data['center'][-1] - GOAL), .15)

    def test_invalid_hold_is_rejected(self):
        for dt in (0., -1., float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                simulate('single_integrator', dt=dt)


if __name__ == '__main__':
    unittest.main()
