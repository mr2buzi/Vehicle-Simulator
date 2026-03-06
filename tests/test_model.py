import unittest
from dataclasses import replace

from vehicle_sim.config import default_parameters
from vehicle_sim.model import EngineeringModel, calculate_metrics


class ModelTests(unittest.TestCase):
    def test_calculate_metrics_extracts_expected_values(self):
        data = {
            "t": [0.0, 1.0, 2.0, 3.0],
            "v": [0.0, 15.0, 27.0, 30.0],
            "a": [0.0, 4.0, 5.0, 1.0],
            "rpm": [1000.0, 3000.0, 5000.0, 5500.0],
            "gear": [1, 1, 2, 2],
            "slip": [0.0, 0.05, 0.2, 0.1],
        }
        metrics = calculate_metrics(data)
        self.assertEqual(metrics.zero_to_sixty_s, 2.0)
        self.assertGreater(metrics.top_speed_mph, 60.0)

    def test_model_generates_time_history_for_short_run(self):
        params = replace(default_parameters(), v_target=10.0)
        history = EngineeringModel(params).solve_run()
        self.assertGreater(len(history["t"]), 0)
        self.assertEqual(set(history.keys()), {"t", "v", "a", "rpm", "gear", "slip"})
        self.assertGreater(history["t"][-1], history["t"][0])


if __name__ == "__main__":
    unittest.main()
