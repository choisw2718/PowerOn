import math
import random
import unittest

from simulation.benchmark.speed_learning import SPEED_SCALES, SpeedLearner, speed_state


class SpeedLearningTest(unittest.TestCase):
    def test_speed_scales_respect_controller_cap_authority(self):
        self.assertEqual(SPEED_SCALES, (0.55, 0.75, 1.0))
        self.assertTrue(all(0.0 < scale <= 1.0 for scale in SPEED_SCALES))
        self.assertEqual(SPEED_SCALES[-1], 1.0)

        learner = SpeedLearner(7)
        state = speed_state(0.30, 0.42, 10.0, 0.80, 0.16)
        for epsilon in (0.0, 0.5, 1.0):
            for _ in range(30):
                action = learner.choose(state, epsilon=epsilon)
                self.assertIn(action, range(len(SPEED_SCALES)))
                for controller_cap in (0.0, 0.20, 0.42, 0.80):
                    self.assertLessEqual(controller_cap * SPEED_SCALES[action], controller_cap)

    def test_speed_state_discretization_is_hashable_and_transparent(self):
        baseline = speed_state(0.25, 0.30, 8.0, 0.90, 0.10)
        self.assertIsInstance(baseline, tuple)
        self.assertEqual(len(baseline), 5)
        self.assertTrue(all(isinstance(b, int) for b in baseline))
        hash(baseline)

        # Symmetric steering magnitude maps to the same turn-severity bin.
        self.assertEqual(
            speed_state(0.25, 0.30, -25.0, 0.90, 0.10),
            speed_state(0.25, 0.30, 25.0, 0.90, 0.10),
        )
        # Distinct regimes map to distinct bins along each input axis.
        self.assertNotEqual(speed_state(0.01, 0.30, 8.0, 0.90, 0.10), baseline)
        self.assertNotEqual(speed_state(0.25, 0.42, 8.0, 0.90, 0.10), baseline)
        self.assertNotEqual(speed_state(0.25, 0.30, 26.0, 0.90, 0.10), baseline)
        self.assertNotEqual(speed_state(0.25, 0.30, 8.0, 0.25, 0.10), baseline)
        self.assertNotEqual(speed_state(0.25, 0.30, 8.0, 0.90, 0.30), baseline)

        # Infinite and non-finite edge inputs still yield valid hashable tuples.
        open_road = speed_state(0.40, 0.42, 0.0, math.inf, 0.05)
        missing = speed_state(math.nan, math.nan, math.nan, math.nan, math.nan)
        self.assertEqual(len(open_road), 5)
        self.assertEqual(len(missing), 5)
        self.assertEqual(len({baseline, open_road, missing}), 3)

    def test_always_default_one_on_unseen_states_and_ties(self):
        learner = SpeedLearner(11, alpha=0.5, gamma=0.9)
        default_idx = SPEED_SCALES.index(1.0)

        unseen_states = [
            speed_state(0.0, 0.0, 0.0, 0.2, 0.05),
            speed_state(0.2, 0.3, 15.0, 0.8, 0.16),
            speed_state(0.42, 0.42, -28.0, 2.5, 0.30),
        ]
        for state in unseen_states:
            self.assertEqual(learner.choose(state, epsilon=0.0), default_idx)
            self.assertEqual(SPEED_SCALES[learner.choose(state)], 1.0)
            self.assertEqual(learner.q_values(state), (0.0, 0.0, 0.0))

        # Exploring without updating must not mark the state as learned.
        probe = unseen_states[0]
        for _ in range(20):
            learner.choose(probe, epsilon=1.0)
        self.assertEqual(learner.choose(probe, epsilon=0.0), default_idx)

        # Zero-reward update preserves an all-zero tie -> 1.0.
        tie_all = unseen_states[1]
        learner.update(tie_all, 0, 0.0, unseen_states[2], terminated=True)
        self.assertEqual(learner.choose(tie_all, epsilon=0.0), default_idx)

        # Tie between action 0 and action 2 -> picks 1.0 (action 2).
        tie_with_full = ("tie-with-full",)
        learner.update(tie_with_full, 0, 2.0, unseen_states[2], terminated=True)
        learner.update(tie_with_full, 2, 2.0, unseen_states[2], terminated=True)
        self.assertEqual(learner.choose(tie_with_full, epsilon=0.0), default_idx)

        # Tie when all actions share the same negative Q-value -> picks 1.0 (action 2).
        tie_negative = ("tie-negative",)
        for action in range(len(SPEED_SCALES)):
            learner.update(tie_negative, action, -1.0, unseen_states[2], terminated=True)
        self.assertEqual(learner.choose(tie_negative, epsilon=0.0), default_idx)

        # Tie between 0 and 1 when 2 is strictly worse stays greedy and prefers 1 (0.75).
        tie_sub_full = ("tie-sub-full",)
        learner.update(tie_sub_full, 0, 2.0, unseen_states[2], terminated=True)
        learner.update(tie_sub_full, 1, 2.0, unseen_states[2], terminated=True)
        learner.update(tie_sub_full, 2, -2.0, unseen_states[2], terminated=True)
        self.assertEqual(learner.choose(tie_sub_full, epsilon=0.0), 1)

    def test_exploration_and_exact_replay(self):
        state = speed_state(0.22, 0.42, 18.0, 0.60, 0.16)
        explorer = SpeedLearner(123)
        seen_actions = {explorer.choose(state, epsilon=1.0) for _ in range(60)}
        self.assertEqual(seen_actions, {0, 1, 2})

        # Exact replay must be isolated from global random state and greedy evaluations.
        learner_a = SpeedLearner(99, alpha=0.25, gamma=0.9)
        learner_b = SpeedLearner(99, alpha=0.25, gamma=0.9)
        next_state = speed_state(0.18, 0.30, 12.0, 0.75, 0.16)

        actions_a = []
        actions_b = []
        for step in range(40):
            random.seed(step * 1000 + 1)
            act_a = learner_a.choose(state, epsilon=0.35)
            # Interleaving greedy eval must not perturb learner_a's exploration RNG.
            self.assertIn(learner_a.choose(state, epsilon=0.0), {0, 1, 2})
            act_b = learner_b.choose(state, epsilon=0.35)
            actions_a.append(act_a)
            actions_b.append(act_b)
            reward = -1.0 if act_a == 2 else 0.5
            learner_a.update(state, act_a, reward, next_state, terminated=False)
            learner_b.update(state, act_b, reward, next_state, terminated=False)

        self.assertEqual(actions_a, actions_b)
        self.assertEqual(learner_a.q_values(state), learner_b.q_values(state))
        self.assertGreater(len(set(actions_a)), 1)

    def test_q_update_terminal_and_truncation(self):
        s_curr = speed_state(0.35, 0.42, 24.0, 0.50, 0.20)
        s_next = speed_state(0.20, 0.30, 15.0, 0.90, 0.10)

        # Seed next_state with a strong positive greedy value Q(s_next, 1) = 10.0.
        trunc_learner = SpeedLearner(5, alpha=0.2, gamma=0.95)
        trunc_learner.update(s_next, 1, 10.0 / 0.2, None, terminated=True)
        self.assertAlmostEqual(max(trunc_learner.q_values(s_next)), 10.0)

        # Time truncation (terminated=False) must bootstrap from next_state.
        trunc_learner.update(s_curr, 0, 1.0, s_next, terminated=False)
        expected_trunc = 0.2 * (1.0 + 0.95 * 10.0)
        self.assertAlmostEqual(trunc_learner.q_values(s_curr)[0], expected_trunc)
        self.assertEqual(trunc_learner.choose(s_curr, epsilon=0.0), 0)

        # Real success/failure (terminated=True) must NOT bootstrap from next_state.
        term_learner = SpeedLearner(5, alpha=0.2, gamma=0.95)
        term_learner.update(s_next, 1, 10.0 / 0.2, None, terminated=True)
        term_learner.update(s_curr, 0, 1.0, s_next, terminated=True)
        expected_term = 0.2 * 1.0
        self.assertAlmostEqual(term_learner.q_values(s_curr)[0], expected_term)

        # Repeated update moves monotonically toward target.
        term_learner.update(s_curr, 0, 1.0, s_next, terminated=True)
        expected_second = expected_term + 0.2 * (1.0 - expected_term)
        self.assertAlmostEqual(term_learner.q_values(s_curr)[0], expected_second)

    def test_input_validation(self):
        with self.assertRaises(ValueError):
            SpeedLearner(1, alpha=-0.1)
        with self.assertRaises(ValueError):
            SpeedLearner(1, gamma=1.1)
        learner = SpeedLearner(1)
        state = speed_state(0.1, 0.2, 0.0, 1.0, 0.1)
        with self.assertRaises(ValueError):
            learner.choose(state, epsilon=-0.01)
        with self.assertRaises(ValueError):
            learner.choose(state, epsilon=1.5)
        with self.assertRaises(ValueError):
            learner.update(state, 3, 0.0, state)
        with self.assertRaises(ValueError):
            learner.update(state, True, 0.0, state)
        with self.assertRaises(ValueError):
            learner.update(state, 0, math.nan, state)


if __name__ == "__main__":
    unittest.main()
