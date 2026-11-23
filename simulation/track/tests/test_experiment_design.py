"""Tests for deterministic Latin Hypercube design and campaign trial summaries."""

import math
import unittest

from simulation.benchmark.design import latin_hypercube, summarize_trials


class LatinHypercubeTests(unittest.TestCase):
    def test_one_sample_per_stratum_and_bounds(self):
        ranges = {
            "max_speed": (0.15, 0.55),
            "wall_margin": (0.08, 0.28),
            "scan_ms": (120.0, 260.0),
        }
        for count in (1, 5, 20):
            design = latin_hypercube(ranges, count=count, seed=17)
            self.assertEqual(len(design), count)
            for row in design:
                self.assertEqual(set(row), set(ranges))
            for name, (low, high) in ranges.items():
                values = [row[name] for row in design]
                self.assertTrue(all(isinstance(val, float) for val in values))
                self.assertTrue(all(low <= val <= high for val in values))
                bins = [
                    min(count - 1, math.floor(((val - low) / (high - low)) * count))
                    for val in values
                ]
                self.assertEqual(sorted(bins), list(range(count)))

    def test_dimensions_are_independently_shuffled(self):
        ranges = {"alpha": (0.0, 1.0), "beta": (0.0, 1.0), "gamma": (0.0, 1.0)}
        design = latin_hypercube(ranges, count=24, seed=99)
        bins_by_factor = {
            name: [min(23, math.floor(row[name] * 24)) for row in design]
            for name in ranges
        }
        self.assertNotEqual(bins_by_factor["alpha"], bins_by_factor["beta"])
        self.assertNotEqual(bins_by_factor["alpha"], bins_by_factor["gamma"])
        self.assertNotEqual(bins_by_factor["beta"], bins_by_factor["gamma"])

    def test_constant_bounds_supported(self):
        ranges = {"fixed_int": (2, 2), "fixed_float": (0.25, 0.25), "varying": (1.0, 5.0)}
        design = latin_hypercube(ranges, count=6, seed=42)
        self.assertEqual(len(design), 6)
        self.assertTrue(all(row["fixed_int"] == 2.0 and isinstance(row["fixed_int"], float) for row in design))
        self.assertTrue(all(row["fixed_float"] == 0.25 for row in design))
        varying_bins = [min(5, math.floor(((row["varying"] - 1.0) / 4.0) * 6)) for row in design]
        self.assertEqual(sorted(varying_bins), list(range(6)))

    def test_replay_and_reordered_factor_mappings_are_stable(self):
        ordered = {"alpha": (0.1, 0.9), "beta": (10.0, 20.0), "gamma": (-5.0, 5.0)}
        reordered = {"gamma": (-5.0, 5.0), "alpha": (0.1, 0.9), "beta": (10.0, 20.0)}
        first = latin_hypercube(ordered, count=12, seed=2025)
        replay = latin_hypercube(ordered, count=12, seed=2025)
        permuted = latin_hypercube(reordered, count=12, seed=2025)
        different_seed = latin_hypercube(ordered, count=12, seed=2026)

        self.assertEqual(first, replay)
        self.assertEqual(first, permuted)
        self.assertEqual([list(row) for row in first], [list(row) for row in permuted])
        self.assertNotEqual(first, different_seed)

    def test_factor_streams_are_independent_across_added_factors(self):
        base = {"alpha": (0.1, 0.9), "beta": (10.0, 20.0)}
        extended = {"00_prefix": (100.0, 200.0), "alpha": (0.1, 0.9), "beta": (10.0, 20.0)}
        base_design = latin_hypercube(base, count=8, seed=7)
        extended_design = latin_hypercube(extended, count=8, seed=7)
        for base_row, ext_row in zip(base_design, extended_design):
            self.assertEqual(base_row["alpha"], ext_row["alpha"])
            self.assertEqual(base_row["beta"], ext_row["beta"])

    def test_validation_errors(self):
        with self.assertRaises(ValueError):
            latin_hypercube({}, count=4, seed=1)
        with self.assertRaises(TypeError):
            latin_hypercube([("a", (0.0, 1.0))], count=4, seed=1)  # type: ignore[arg-type]
        for bad_name in ("", "   "):
            with self.assertRaises(ValueError):
                latin_hypercube({bad_name: (0.0, 1.0)}, count=4, seed=1)
        with self.assertRaises(ValueError):
            latin_hypercube({1: (0.0, 1.0)}, count=4, seed=1)  # type: ignore[dict-item]
        for bad_bounds in ((1.0, 0.0), (math.nan, 1.0), (0.0, math.inf), (-math.inf, 0.0), (0.0,), "ab"):
            with self.assertRaises(ValueError):
                latin_hypercube({"x": bad_bounds}, count=4, seed=1)  # type: ignore[dict-item]
        with self.assertRaises(TypeError):
            latin_hypercube({"x": (True, 2.0)}, count=4, seed=1)  # type: ignore[dict-item]
        for bad_count in (0, -3):
            with self.assertRaises(ValueError):
                latin_hypercube({"x": (0.0, 1.0)}, count=bad_count, seed=1)
        for bad_type_count in (True, 2.5):
            with self.assertRaises(TypeError):
                latin_hypercube({"x": (0.0, 1.0)}, count=bad_type_count, seed=1)  # type: ignore[arg-type]
        for bad_seed in (False, 1.5):
            with self.assertRaises(TypeError):
                latin_hypercube({"x": (0.0, 1.0)}, count=4, seed=bad_seed)  # type: ignore[arg-type]


class SummarizeTrialsTests(unittest.TestCase):
    def test_counts_percentiles_completion_median_and_exploratory_labels(self):
        trials = [
            {"factors": {"speed": 0.20, "margin": 0.12}, "collision": False, "min_wall_m": 0.10, "seconds": 50.0, "completed": True},
            {"factors": {"speed": 0.30, "margin": 0.14}, "collision": False, "min_wall_m": 0.20, "seconds": 60.0, "completed": True},
            {"factors": {"speed": 0.40, "margin": 0.16}, "collision": False, "min_wall_m": 0.30, "seconds": 70.0, "completed": True},
            {"factors": {"speed": 0.50, "margin": 0.18}, "collision": True, "min_wall_m": None, "seconds": 5.0, "completed": False},
            {"factors": {"speed": 0.60, "margin": 0.20}, "collision": False, "min_wall_m": 0.40, "seconds": 120.0, "completed": False},
        ]
        summary = summarize_trials(trials)

        self.assertEqual(summary["runs"], 5)
        self.assertEqual(summary["completions"], 3)
        self.assertEqual(summary["collisions"], 1)
        self.assertEqual(summary["min_wall_m"]["valid_runs"], 4)
        self.assertAlmostEqual(summary["min_wall_m"]["p05"], 0.115)
        self.assertAlmostEqual(summary["min_wall_m"]["median"], 0.25)
        self.assertAlmostEqual(summary["completion_seconds_median"], 60.0)

        assoc = summary["exploratory_association"]
        self.assertEqual(assoc["label"], "exploratory_association")
        self.assertTrue(assoc["exploratory_only"])
        self.assertFalse(assoc["causal"])
        self.assertEqual(assoc["target"], "min_wall_m")
        self.assertEqual(assoc["method"], "spearman_rank")
        self.assertIn("exploratory", assoc["note"].lower())
        self.assertAlmostEqual(assoc["factors"]["speed"], 1.0)
        self.assertAlmostEqual(assoc["factors"]["margin"], 1.0)

    def test_spearman_ties_and_negative_association(self):
        trials = [
            {"factors": {"x": 1.0, "z": 100.0}, "collision": False, "min_wall_m": 10.0, "seconds": 10.0, "completed": True},
            {"factors": {"x": 2.0, "z": 50.0}, "collision": False, "min_wall_m": 20.0, "seconds": 11.0, "completed": True},
            {"factors": {"x": 2.0, "z": 10.0}, "collision": False, "min_wall_m": 30.0, "seconds": 12.0, "completed": True},
            {"factors": {"x": 4.0, "z": 1.0}, "collision": False, "min_wall_m": 40.0, "seconds": 13.0, "completed": True},
        ]
        assoc = summarize_trials(trials)["exploratory_association"]["factors"]
        self.assertAlmostEqual(assoc["x"], math.sqrt(0.9))
        self.assertAlmostEqual(assoc["z"], -1.0)

        both_tied = [
            {"factors": {"a": 1.0}, "collision": False, "min_wall_m": 10.0, "seconds": 1.0, "completed": True},
            {"factors": {"a": 1.0}, "collision": False, "min_wall_m": 20.0, "seconds": 1.0, "completed": True},
            {"factors": {"a": 2.0}, "collision": False, "min_wall_m": 10.0, "seconds": 1.0, "completed": True},
            {"factors": {"a": 2.0}, "collision": False, "min_wall_m": 20.0, "seconds": 1.0, "completed": True},
        ]
        self.assertAlmostEqual(
            summarize_trials(both_tied)["exploratory_association"]["factors"]["a"],
            0.0,
        )

    def test_missing_values_constant_and_insufficient_data(self):
        trials = [
            {"factors": {"const": 0.5, "sparse": 1.0, "varying": 1.0}, "collision": False, "min_wall_m": 0.2, "seconds": 10.0, "completed": True},
            {"factors": {"const": 0.5, "sparse": None, "varying": 2.0}, "collision": True, "min_wall_m": None, "seconds": 4.0, "completed": False},
            {"factors": {"const": 0.5, "varying": 3.0}, "collision": False, "min_wall_m": math.nan, "seconds": 12.0, "completed": False},
            {"factors": {"const": 0.5, "sparse": None, "varying": 4.0}, "collision": False, "min_wall_m": 0.4, "seconds": 14.0, "completed": False},
        ]
        summary = summarize_trials(trials)
        self.assertEqual(summary["runs"], 4)
        self.assertEqual(summary["completions"], 1)
        self.assertEqual(summary["collisions"], 1)
        self.assertEqual(summary["min_wall_m"]["valid_runs"], 2)
        self.assertAlmostEqual(summary["min_wall_m"]["p05"], 0.21)
        self.assertAlmostEqual(summary["min_wall_m"]["median"], 0.3)
        self.assertAlmostEqual(summary["completion_seconds_median"], 10.0)
        factors = summary["exploratory_association"]["factors"]
        self.assertIsNone(factors["const"])
        self.assertIsNone(factors["sparse"])
        self.assertAlmostEqual(factors["varying"], 1.0)

        constant_clearance = [
            {"factors": {"a": 1.0}, "collision": False, "min_wall_m": 0.2, "seconds": 8.0, "completed": True},
            {"factors": {"a": 2.0}, "collision": False, "min_wall_m": 0.2, "seconds": 9.0, "completed": True},
            {"factors": {"a": 3.0}, "collision": False, "min_wall_m": 0.2, "seconds": 10.0, "completed": True},
        ]
        self.assertIsNone(
            summarize_trials(constant_clearance)["exploratory_association"]["factors"]["a"]
        )

        single_valid = [
            {"factors": {"a": 1.0}, "collision": False, "min_wall_m": 0.15, "seconds": 9.0, "completed": False},
            {"factors": {"a": 2.0}, "collision": True, "min_wall_m": None, "seconds": 3.0, "completed": False},
        ]
        single_summary = summarize_trials(single_valid)
        self.assertIsNone(single_summary["completion_seconds_median"])
        self.assertIsNone(single_summary["exploratory_association"]["factors"]["a"])

    def test_empty_and_reordered_trials_replay(self):
        empty_summary = summarize_trials([])
        self.assertEqual(empty_summary["runs"], 0)
        self.assertEqual(empty_summary["completions"], 0)
        self.assertEqual(empty_summary["collisions"], 0)
        self.assertIsNone(empty_summary["min_wall_m"]["p05"])
        self.assertIsNone(empty_summary["min_wall_m"]["median"])
        self.assertIsNone(empty_summary["completion_seconds_median"])
        self.assertEqual(empty_summary["exploratory_association"]["factors"], {})

        trials = [
            {"factors": {"b": 3.0, "a": 1.0}, "collision": False, "min_wall_m": 0.12, "seconds": 25.0, "completed": True},
            {"factors": {"b": 1.0, "a": 2.0}, "collision": False, "min_wall_m": 0.18, "seconds": 20.0, "completed": True},
            {"factors": {"b": 2.0, "a": 3.0}, "collision": False, "min_wall_m": 0.30, "seconds": 30.0, "completed": True},
        ]
        reordered = [
            {"factors": {"a": 3.0, "b": 2.0}, "collision": False, "min_wall_m": 0.30, "seconds": 30.0, "completed": True},
            {"factors": {"a": 1.0, "b": 3.0}, "collision": False, "min_wall_m": 0.12, "seconds": 25.0, "completed": True},
            {"factors": {"a": 2.0, "b": 1.0}, "collision": False, "min_wall_m": 0.18, "seconds": 20.0, "completed": True},
        ]
        self.assertEqual(summarize_trials(iter(trials)), summarize_trials(reordered))


if __name__ == "__main__":
    unittest.main()
