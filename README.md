# Vehicle Dynamics Simulation

## What the product is

This project is a desktop vehicle acceleration simulator built in Python. It models a rear-driven car through a simplified 3-degree-of-freedom longitudinal dynamics system and generates a PDF engineering report with performance metrics, plots, and governing equations.

The intended use is quick concept evaluation: change powertrain and chassis inputs, run the simulation, and inspect outputs such as 0-60 mph time, top speed, peak longitudinal acceleration, and wheel slip behavior.

## Architecture

The project currently lives in a single executable script, `sim.py`, with three functional layers:

1. `EngineeringModel`
   Handles the physics simulation.
   It builds the torque curve, computes drivetrain coupling, tire forces, longitudinal acceleration, and numerically integrates the state forward in time.

2. Reporting helpers
   Convert raw time-history output into summary metrics, plots, rendered equations, and a final PDF report.

3. Tkinter GUI
   Provides a simple parameter-entry interface and launches the full simulation/report pipeline.

The runtime flow is:

1. User enters parameters in the GUI.
2. Inputs are validated and normalized.
3. `EngineeringModel.solve_run()` simulates the run.
4. The report layer calculates metrics and builds `MultiBody_Dynamics_Analysis.pdf`.

## Tech stack

- Python
- Tkinter for the desktop UI
- NumPy for numerical operations
- SciPy for interpolation and ODE integration
- Matplotlib for plots and equation rendering
- ReportLab for PDF generation

## Key engineering challenges

- Stable low-speed tire slip behavior
  Slip ratio becomes numerically unstable near zero vehicle speed, so the model uses a small velocity floor to avoid singularities during launch.

- Drivetrain coupling during launch and shifts
  A rigid clutch model can destabilize the solver, so the implementation uses a bounded viscous coupling approximation.

- Load-sensitive traction estimation
  Rear tire normal load changes under acceleration. The model applies quasi-static load transfer before evaluating final tractive force.

- Keeping the reporting pipeline clean
  Plot and equation image assets are generated in a temporary directory so the workspace is not polluted by intermediate files after a run.

## Data model

The simulation input is a flat parameter dictionary with the following core fields:

- `hp`: engine power
- `torque`: peak engine torque
- `rpm_p`: RPM at peak power
- `rpm_t`: RPM at peak torque
- `max_rpm`: rev limit / shift ceiling
- `mass`: vehicle mass
- `gears`: comma-separated gearbox ratios
- `final_drive`: final drive ratio
- `tire_diam`: tire diameter
- `mu`: tire-road friction coefficient
- `CdA`: combined drag coefficient and frontal area term
- `Crr`: rolling resistance coefficient
- `wheelbase`: wheelbase
- `cg_height`: center-of-gravity height
- `eta`: drivetrain efficiency
- `v_target`: target speed to end the run

The dynamic state vector is:

- `v`: vehicle longitudinal speed
- `w_wheel`: wheel angular speed
- `w_eng`: engine angular speed

Simulation output is stored as time-history arrays:

- `t`: time
- `v`: speed
- `a`: longitudinal acceleration
- `rpm`: engine speed
- `gear`: active gear index for display
- `slip`: longitudinal tire slip ratio

## How to run

1. Install dependencies:

```bash
pip install numpy scipy matplotlib reportlab
```

2. Start the application:

```bash
python sim.py
```

3. Adjust the vehicle parameters in the GUI and click `RUN SIMULATION & GENERATE REPORT`.

4. The generated PDF report will be written to the project directory as `MultiBody_Dynamics_Analysis.pdf`.
