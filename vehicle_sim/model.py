import numpy as np
from scipy.integrate import solve_ivp
from scipy.interpolate import PchipInterpolator

from vehicle_sim.config import AIR_DENSITY, GRAVITY, SimulationMetrics, VehicleParameters


class EngineeringModel:
    def __init__(self, params: VehicleParameters):
        self.params = params
        self.ratios = params.gears
        self.torque_map = self._build_torque_map()

    def _build_torque_map(self):
        rpm_p = max(self.params.rpm_p, self.params.rpm_t + 500)
        peak_power_rad_s = rpm_p * 2 * np.pi / 60
        torque_at_peak_power = (self.params.hp * 745.7) / peak_power_rad_s

        raw_points = [
            (0.0, 0.0),
            (800.0, self.params.torque * 0.6),
            (self.params.rpm_t, self.params.torque),
            (rpm_p, torque_at_peak_power),
            (self.params.max_rpm, torque_at_peak_power * 0.6),
            (self.params.max_rpm + 1000.0, 0.0),
        ]
        raw_points.sort(key=lambda point: point[0])

        clean_rpms = []
        clean_torques = []
        for rpm, torque in raw_points:
            if clean_rpms and rpm <= clean_rpms[-1]:
                rpm = clean_rpms[-1] + 10.0
            clean_rpms.append(rpm)
            clean_torques.append(torque)
        return PchipInterpolator(clean_rpms, clean_torques, extrapolate=False)

    def get_engine_torque(self, engine_rad_s: float) -> float:
        rpm = max(engine_rad_s * 9.5493, 0.0)
        torque = self.torque_map(rpm)
        return max(0.0, float(torque)) if torque is not None else 0.0

    def get_tire_force_pure(self, wheel_rad_s: float, vehicle_speed: float, normal_load: float) -> float:
        tire_radius = self.params.tire_diam / 2.0
        speed_floor = 0.1
        slip = (wheel_rad_s * tire_radius - vehicle_speed) / max(abs(vehicle_speed), speed_floor)

        reference_load = (self.params.mass * GRAVITY) / 4.0
        load_sensitivity = 1 + 0.1 * (normal_load - reference_load) / reference_load
        effective_mu = self.params.mu / max(load_sensitivity, 0.1)

        stiffness_b, shape_c, curvature_e = 10.0, 1.9, 0.97
        slip_clamped = np.clip(slip, -1.0, 1.0)
        force_shape = np.sin(
            shape_c
            * np.arctan(
                stiffness_b * slip_clamped
                - curvature_e
                * (stiffness_b * slip_clamped - np.arctan(stiffness_b * slip_clamped))
            )
        )
        return normal_load * effective_mu * force_shape

    def derivatives(self, _time: float, state, gear_index: int, clutch_clamp: float):
        vehicle_speed, wheel_rad_s, engine_rad_s = state

        aero_force = 0.5 * AIR_DENSITY * self.params.CdA * vehicle_speed**2 * np.sign(vehicle_speed)
        rolling_force = (
            self.params.Crr * self.params.mass * GRAVITY * np.sign(vehicle_speed)
            if abs(vehicle_speed) > 1e-6
            else 0.0
        )

        total_ratio = self.ratios[gear_index] * self.params.final_drive
        tire_radius = self.params.tire_diam / 2.0
        transmission_input_speed = wheel_rad_s * total_ratio
        clutch_speed_delta = engine_rad_s - transmission_input_speed

        clutch_gain = 50.0
        clutch_limit = self.params.torque * 1.5
        clutch_torque = np.clip(clutch_speed_delta * clutch_gain * clutch_clamp, -clutch_limit, clutch_limit)

        static_rear_load = self.params.mass * GRAVITY * 0.6
        raw_tractive_force = self.get_tire_force_pure(wheel_rad_s, vehicle_speed, static_rear_load)
        estimated_accel = (raw_tractive_force - aero_force) / self.params.mass
        load_transfer = estimated_accel * self.params.mass * (self.params.cg_height / self.params.wheelbase)
        rear_load = np.clip(static_rear_load + load_transfer, 0.0, self.params.mass * GRAVITY)
        tractive_force = self.get_tire_force_pure(wheel_rad_s, vehicle_speed, rear_load)

        combustion_torque = self.get_engine_torque(engine_rad_s)
        engine_friction = engine_rad_s * 0.01
        engine_inertia = 0.25
        engine_accel = (combustion_torque - clutch_torque - engine_friction) / engine_inertia

        drive_torque_at_wheel = clutch_torque * total_ratio * self.params.eta
        road_reaction_torque = tractive_force * tire_radius
        wheel_inertia = 1.5
        wheel_accel = (drive_torque_at_wheel - road_reaction_torque) / wheel_inertia

        vehicle_accel = (tractive_force - aero_force - rolling_force) / self.params.mass
        return [vehicle_accel, wheel_accel, engine_accel]

    def solve_run(self):
        state = [0.0, 0.0, 1000.0 * 2 * np.pi / 60]
        current_time = 0.0
        time_step = 0.002
        target_speed_m_s = self.params.v_target / 2.237
        current_gear = 0
        shift_timer = 0.0
        history = {"t": [], "v": [], "a": [], "rpm": [], "gear": [], "slip": []}

        while current_time < 25.0 and state[0] < target_speed_m_s:
            if current_time < 0.5:
                clutch_clamp = current_time / 0.5
            elif shift_timer > 0:
                clutch_clamp = 0.0
            else:
                clutch_clamp = 1.0

            solution = solve_ivp(
                lambda t_s, y_s: self.derivatives(t_s, y_s, current_gear, clutch_clamp),
                (current_time, current_time + time_step),
                state,
                method="RK45",
            )
            next_state = solution.y[:, -1]
            derivs = self.derivatives(current_time, next_state, current_gear, clutch_clamp)

            engine_rpm = next_state[2] * 9.5493
            if shift_timer > 0:
                shift_timer -= time_step
                if shift_timer <= 0:
                    current_gear += 1
            elif engine_rpm > self.params.max_rpm * 0.98 and current_gear < len(self.ratios) - 1:
                shift_timer = 0.25

            tire_radius = self.params.tire_diam / 2.0
            safe_speed = max(next_state[0], 0.1)
            slip = (next_state[1] * tire_radius - next_state[0]) / safe_speed

            current_time += time_step
            state = next_state

            history["t"].append(current_time)
            history["v"].append(state[0])
            history["a"].append(derivs[0])
            history["rpm"].append(engine_rpm)
            history["gear"].append(current_gear + 1 if shift_timer <= 0 else 0)
            history["slip"].append(slip)

        return history


def calculate_metrics(data) -> SimulationMetrics:
    speed_mph = np.array(data["v"]) * 2.237
    time_s = np.array(data["t"])
    accel_g = np.array(data["a"]) / GRAVITY
    slip = np.array(data["slip"])

    zero_to_sixty = None
    reached_sixty = np.where(speed_mph >= 60.0)[0]
    if reached_sixty.size:
        zero_to_sixty = float(time_s[reached_sixty[0]])

    peak_g = float(np.max(accel_g[10:])) if len(accel_g) > 10 else 0.0
    traction_window = slip[10:]
    traction_limited_pct = (
        float(np.sum(traction_window > 0.15) / len(traction_window) * 100.0)
        if len(traction_window) > 0
        else 0.0
    )

    return SimulationMetrics(
        zero_to_sixty_s=zero_to_sixty,
        peak_longitudinal_g=peak_g,
        top_speed_mph=float(np.max(speed_mph)) if len(speed_mph) else 0.0,
        traction_limited_pct=traction_limited_pct,
    )
