import argparse

from vehicle_sim.config import DEFAULT_ENTRY_VALUES, VehicleParameters
from vehicle_sim.model import EngineeringModel, calculate_metrics
from vehicle_sim.report import generate_pdf_report


def build_argument_parser():
    parser = argparse.ArgumentParser(description="Run the vehicle simulator without the GUI.")
    parser.add_argument("--output", default=None, help="Optional output PDF path.")
    parser.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="Override a default parameter. Repeat for multiple overrides.",
    )
    return parser


def parse_overrides(items):
    values = dict(DEFAULT_ENTRY_VALUES)
    for item in items:
        if "=" not in item:
            raise ValueError(f"Invalid override '{item}'. Expected KEY=VALUE.")
        key, value = item.split("=", 1)
        key = key.strip()
        if key not in values:
            raise ValueError(f"Unknown parameter '{key}'.")
        values[key] = value.strip()
    return VehicleParameters.from_mapping(values)


def main():
    args = build_argument_parser().parse_args()
    params = parse_overrides(args.set)
    data = EngineeringModel(params).solve_run()
    metrics = calculate_metrics(data)
    report_path = generate_pdf_report(data, output_path=args.output)

    print("Simulation complete")
    if metrics.zero_to_sixty_s is not None:
        print(f"0-60 mph: {metrics.zero_to_sixty_s:.2f}s")
    else:
        print("0-60 mph: N/A")
    print(f"Top speed: {metrics.top_speed_mph:.1f} mph")
    print(f"Traction limited: {metrics.traction_limited_pct:.1f}%")
    print(f"Report: {report_path}")


if __name__ == "__main__":
    main()
