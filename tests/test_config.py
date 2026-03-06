import unittest

from vehicle_sim.config import DEFAULT_ENTRY_VALUES, VehicleParameters, default_parameters


class VehicleParametersTests(unittest.TestCase):
    def test_default_parameters_are_valid(self):
        params = default_parameters()
        self.assertGreater(params.hp, 0)
        self.assertGreater(len(params.gears), 0)

    def test_string_gears_are_parsed_into_tuple(self):
        params = VehicleParameters.from_mapping(DEFAULT_ENTRY_VALUES | {"gears": "3.5, 2.1, 1.4"})
        self.assertEqual(params.gears, (3.5, 2.1, 1.4))

    def test_invalid_efficiency_is_rejected(self):
        with self.assertRaises(ValueError):
            VehicleParameters.from_mapping(DEFAULT_ENTRY_VALUES | {"eta": "1.2"})

    def test_empty_gear_list_is_rejected(self):
        with self.assertRaises(ValueError):
            VehicleParameters.from_mapping(DEFAULT_ENTRY_VALUES | {"gears": ""})


if __name__ == "__main__":
    unittest.main()
