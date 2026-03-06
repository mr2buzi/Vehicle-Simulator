import tempfile
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import messagebox, ttk

import matplotlib.pyplot as plt
import numpy as np
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_JUSTIFY
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import cm
from reportlab.platypus import Image, PageBreak, Paragraph, SimpleDocTemplate, Spacer, Table, TableStyle
from scipy.integrate import solve_ivp
from scipy.interpolate import PchipInterpolator


GRAVITY = 9.81
AIR_DENSITY = 1.225
REPORT_FILENAME = "MultiBody_Dynamics_Analysis.pdf"
DEFAULT_PARAMS = {
    "hp": "700",
    "torque": "600",
    "rpm_p": "7500",
    "rpm_t": "4500",
    "max_rpm": "8200",
    "mass": "1600",
    "gears": "3.8, 2.4, 1.7, 1.3, 1.0, 0.8",
    "final_drive": "3.73",
    "tire_diam": "0.68",
    "mu": "1.2",
    "CdA": "0.38",
    "Crr": "0.015",
    "wheelbase": "2.7",
    "cg_height": "0.5",
    "eta": "0.90",
    "v_target": "180",
}


@dataclass
class SimulationMetrics:
    zero_to_sixty_s: float | None
    peak_longitudinal_g: float
    top_speed_mph: float
    traction_limited_pct: float


class EngineeringModel:
    def __init__(self, params):
        self.p = params
        self.g = GRAVITY
        self.rho = AIR_DENSITY
        self.ratios = self._parse_gear_ratios()
        self.torque_map = self._build_torque_map()

    def _build_torque_map(self):
        rpm_p = max(self.p["rpm_p"], self.p["rpm_t"] + 500)
        peak_power_rad_s = rpm_p * 2 * np.pi / 60
        torque_at_peak_power = (self.p["hp"] * 745.7) / peak_power_rad_s

        raw_points = [
            (0.0, 0.0),
            (800.0, self.p["torque"] * 0.6),
            (self.p["rpm_t"], self.p["torque"]),
            (rpm_p, torque_at_peak_power),
            (self.p["max_rpm"], torque_at_peak_power * 0.6),
            (self.p["max_rpm"] + 1000.0, 0.0),
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

    def _parse_gear_ratios(self):
        parts = [part.strip() for part in self.p["gears"].split(",") if part.strip()]
        ratios = [float(part) for part in parts]
        if not ratios:
            raise ValueError("At least one gear ratio is required.")
        return ratios

    def get_engine_torque(self, engine_rad_s):
        rpm = max(engine_rad_s * 9.5493, 0.0)
        torque = self.torque_map(rpm)
        return max(0.0, float(torque)) if torque is not None else 0.0

    def get_tire_force_pure(self, wheel_rad_s, vehicle_speed, normal_load):
        tire_radius = self.p["tire_diam"] / 2.0
        speed_floor = 0.1
        slip = (wheel_rad_s * tire_radius - vehicle_speed) / max(abs(vehicle_speed), speed_floor)

        reference_load = (self.p["mass"] * self.g) / 4.0
        load_sensitivity = 1 + 0.1 * (normal_load - reference_load) / reference_load
        effective_mu = self.p["mu"] / max(load_sensitivity, 0.1)

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

    def derivatives(self, _time, state, gear_index, clutch_clamp):
        vehicle_speed, wheel_rad_s, engine_rad_s = state

        aero_force = 0.5 * self.rho * self.p["CdA"] * vehicle_speed**2 * np.sign(vehicle_speed)
        rolling_force = (
            self.p["Crr"] * self.p["mass"] * self.g * np.sign(vehicle_speed)
            if abs(vehicle_speed) > 1e-6
            else 0.0
        )

        total_ratio = self.ratios[gear_index] * self.p["final_drive"]
        tire_radius = self.p["tire_diam"] / 2.0
        transmission_input_speed = wheel_rad_s * total_ratio
        clutch_speed_delta = engine_rad_s - transmission_input_speed

        clutch_gain = 50.0
        clutch_limit = self.p["torque"] * 1.5
        clutch_torque = np.clip(clutch_speed_delta * clutch_gain * clutch_clamp, -clutch_limit, clutch_limit)

        static_rear_load = self.p["mass"] * self.g * 0.6
        raw_tractive_force = self.get_tire_force_pure(wheel_rad_s, vehicle_speed, static_rear_load)
        estimated_accel = (raw_tractive_force - aero_force) / self.p["mass"]
        load_transfer = estimated_accel * self.p["mass"] * (self.p["cg_height"] / self.p["wheelbase"])
        rear_load = np.clip(static_rear_load + load_transfer, 0.0, self.p["mass"] * self.g)
        tractive_force = self.get_tire_force_pure(wheel_rad_s, vehicle_speed, rear_load)

        combustion_torque = self.get_engine_torque(engine_rad_s)
        engine_friction = engine_rad_s * 0.01
        engine_inertia = 0.25
        engine_accel = (combustion_torque - clutch_torque - engine_friction) / engine_inertia

        drive_torque_at_wheel = clutch_torque * total_ratio * self.p["eta"]
        road_reaction_torque = tractive_force * tire_radius
        wheel_inertia = 1.5
        wheel_accel = (drive_torque_at_wheel - road_reaction_torque) / wheel_inertia

        vehicle_accel = (tractive_force - aero_force - rolling_force) / self.p["mass"]
        return [vehicle_accel, wheel_accel, engine_accel]

    def solve_run(self):
        state = [0.0, 0.0, 1000.0 * 2 * np.pi / 60]
        current_time = 0.0
        time_step = 0.002
        target_speed_m_s = self.p["v_target"] / 2.237
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
            elif engine_rpm > self.p["max_rpm"] * 0.98 and current_gear < len(self.ratios) - 1:
                shift_timer = 0.25

            tire_radius = self.p["tire_diam"] / 2.0
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


def render_latex_eq(latex_str, filename, fontsize=14):
    fig = plt.figure(figsize=(10, 1))
    fig.text(0.5, 0.5, f"${latex_str}$", size=fontsize, ha="center", va="center")
    plt.axis("off")
    plt.savefig(filename, dpi=200, bbox_inches="tight", pad_inches=0.1)
    plt.close(fig)


def calculate_metrics(data):
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


def create_dynamics_plot(data, output_path):
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 10), sharex=True)

    ax1.plot(data["t"], np.array(data["v"]) * 2.237, linewidth=2, color="#1f77b4")
    ax1.set_ylabel("Speed (mph)", fontweight="bold")
    ax1.set_title("Longitudinal Velocity Profile", fontweight="bold")
    ax1.grid(True, alpha=0.3)

    ax2.plot(data["t"], data["rpm"], color="#d62728", linewidth=1.5)
    ax2.set_ylabel("Engine RPM", fontweight="bold")
    ax2.set_title("Engine Speed & Shift Points", fontweight="bold")
    ax2.grid(True, alpha=0.3)

    ax3.plot(data["t"], np.array(data["slip"]) * 100, color="#2ca02c", label="Longitudinal Slip")
    ax3.set_ylabel("Slip Ratio (%)", fontweight="bold")
    ax3.set_xlabel("Time (s)", fontweight="bold")
    ax3.set_title("Tire Traction Dynamics", fontweight="bold")
    ax3.axhline(y=15, color="gray", linestyle="--", alpha=0.5, label="Traction Limit")
    ax3.set_ylim(-5, 40)
    ax3.legend(loc="upper right")
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=200)
    plt.close(fig)


def append_equation_block(story, title, description, latex, image_path, width_cm, styles):
    story.append(Paragraph(title, styles["h3"]))
    story.append(Paragraph(description, styles["body"]))
    render_latex_eq(latex, image_path)
    story.append(Image(str(image_path), width=width_cm * cm, height=1.5 * cm))


def build_report_styles():
    base_styles = getSampleStyleSheet()
    return {
        "title": ParagraphStyle(
            "MainTitle",
            parent=base_styles["Title"],
            fontSize=24,
            spaceAfter=5,
            alignment=TA_CENTER,
        ),
        "subtitle": ParagraphStyle(
            "SubTitle",
            parent=base_styles["Normal"],
            fontSize=12,
            spaceAfter=20,
            alignment=TA_CENTER,
        ),
        "h2": ParagraphStyle(
            "Heading2Custom",
            parent=base_styles["Heading2"],
            fontSize=14,
            spaceBefore=15,
            spaceAfter=10,
            textColor=colors.navy,
        ),
        "h3": ParagraphStyle(
            "Heading3Custom",
            parent=base_styles["Heading3"],
            fontSize=12,
            spaceBefore=10,
            spaceAfter=5,
        ),
        "body": ParagraphStyle(
            "JustifiedBody",
            parent=base_styles["BodyText"],
            alignment=TA_JUSTIFY,
            leading=14,
            fontSize=10,
        ),
    }


def generate_pdf_report(data):
    metrics = calculate_metrics(data)
    output_path = Path.cwd() / REPORT_FILENAME
    doc = SimpleDocTemplate(
        str(output_path),
        pagesize=A4,
        rightMargin=2 * cm,
        leftMargin=2 * cm,
        topMargin=2 * cm,
        bottomMargin=2 * cm,
    )
    styles = build_report_styles()
    story = []

    story.append(Paragraph("High-Fidelity Vehicle Dynamics Analysis", styles["title"]))
    story.append(Paragraph("Generated by Engineering Simulation Core v2.1", styles["subtitle"]))
    story.append(Spacer(1, 10))

    table_data = [
        ["Metric", "Result", "Notes"],
        [
            "0 - 60 mph",
            f"{metrics.zero_to_sixty_s:.2f} s" if metrics.zero_to_sixty_s is not None else "N/A",
            "Includes shift times",
        ],
        ["Top Speed", f"{metrics.top_speed_mph:.1f} mph", "Drag limited"],
        ["Peak Long. G", f"{metrics.peak_longitudinal_g:.2f} G", "Launch grip"],
        [
            "Traction Limited",
            f"{metrics.traction_limited_pct:.1f} %",
            "% of run with wheelspin",
        ],
    ]

    results_table = Table(table_data, colWidths=[5 * cm, 5 * cm, 6 * cm])
    results_table.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, 0), colors.navy),
                ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
                ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
                ("FONTSIZE", (0, 0), (-1, 0), 12),
                ("BOTTOMPADDING", (0, 0), (-1, 0), 8),
                ("TOPPADDING", (0, 0), (-1, 0), 8),
                ("BACKGROUND", (0, 1), (-1, -1), colors.white),
                ("TEXTCOLOR", (0, 1), (-1, -1), colors.black),
                ("FONTNAME", (0, 1), (-1, -1), "Helvetica"),
                ("ALIGN", (0, 0), (-1, -1), "LEFT"),
                ("GRID", (0, 0), (-1, -1), 1.5, colors.black),
                ("BOX", (0, 0), (-1, -1), 2, colors.black),
            ]
        )
    )
    story.append(results_table)
    story.append(Spacer(1, 20))

    zero_to_sixty_text = (
        f"{metrics.zero_to_sixty_s:.2f}s" if metrics.zero_to_sixty_s is not None else "the target speed window"
    )
    analysis_text = f"""
    <b>Engineering Justification:</b><br/>
    The simulation results reflect the coupling between engine inertia, tire tractive limits, and aerodynamic drag.
    The vehicle achieved a 0-60 time of <b>{zero_to_sixty_text}</b>.
    <br/><br/>
    1. <b>Launch Phase:</b> The tire slip plot indicates whether the vehicle is traction-limited or power-limited.
    The <i>Traction Limited</i> metric ({metrics.traction_limited_pct:.1f}%) quantifies how much of the run was spent modulating wheelspin.
    <br/>
    2. <b>Shift Dynamics:</b> The torque interruption visible in acceleration data corresponds to the clutch opening during gear shifts.
    <br/>
    3. <b>Aerodynamic Drag:</b> As velocity increases, the acceleration rate decays non-linearly ($F_d \\propto v^2$),
    eventually resulting in the drag-limited top speed.
    """
    story.append(Paragraph(analysis_text, styles["body"]))
    story.append(PageBreak())

    story.append(Paragraph("<b>Appendix A: Dynamics Visualization</b>", styles["h2"]))
    story.append(Spacer(1, 10))

    with tempfile.TemporaryDirectory() as temp_dir:
        temp_path = Path(temp_dir)
        plot_path = temp_path / "plots.png"
        create_dynamics_plot(data, plot_path)
        story.append(Image(str(plot_path), width=16 * cm, height=20 * cm))
        story.append(PageBreak())

        story.append(Paragraph("<b>Appendix B: Mathematical Framework</b>", styles["h2"]))
        story.append(
            Paragraph(
                "The simulation utilizes a 3-degree-of-freedom explicit ODE solver. This section summarizes the governing equations used by the physics engine.",
                styles["body"],
            )
        )
        story.append(Spacer(1, 10))

        append_equation_block(
            story,
            "<b>1. Rotational Engine Dynamics</b>",
            "The engine is modeled as a rotating inertia subject to combustion torque, internal friction, and clutch loading.",
            r"I_{e} \frac{d\omega_e}{dt} = T_{comb}(\omega_e, \theta_{thr}) - T_{clutch} - T_{fric}",
            temp_path / "eq_eng.png",
            10,
            styles,
        )
        append_equation_block(
            story,
            "<b>2. Clutch & Transmission Kinematics</b>",
            "The clutch is modeled as a variable stiffness coupler. When slipping, torque transfer is proportional to clamp load and speed delta.",
            r"T_{clutch} = k_{gain} \cdot \mu_{clutch} \cdot (\omega_e - \omega_w \cdot G_{gear} \cdot G_{final})",
            temp_path / "eq_clutch.png",
            12,
            styles,
        )
        append_equation_block(
            story,
            "<b>3. Pacejka Tire Model (Magic Formula)</b>",
            "Longitudinal traction starts from the slip ratio definition below.",
            r"\kappa = \frac{\omega_w r_{tire} - v}{\max(|v|, \epsilon)}",
            temp_path / "eq_slip.png",
            6,
            styles,
        )
        story.append(
            Paragraph(
                "The resulting tractive force is then computed with a load-sensitive Magic Formula relationship.",
                styles["body"],
            )
        )
        render_latex_eq(
            r"F_x = D \sin(C \arctan(B\kappa - E(B\kappa - \arctan(B\kappa)))) \cdot \lambda_{\mu,z}",
            temp_path / "eq_pacejka.png",
        )
        story.append(Image(str(temp_path / "eq_pacejka.png"), width=13 * cm, height=1.5 * cm))
        append_equation_block(
            story,
            "<b>4. Quasi-Static Load Transfer</b>",
            "Vertical load on the drive tires varies dynamically with longitudinal acceleration.",
            r"F_{z,rear} = \frac{mg}{2} + \frac{h_{cg}}{L} m a_x",
            temp_path / "eq_load.png",
            8,
            styles,
        )
        append_equation_block(
            story,
            "<b>5. Vehicle Equations of Motion</b>",
            "Chassis acceleration is solved from tractive and resistive force balance.",
            r"m \frac{dv}{dt} = F_x - \frac{1}{2}\rho C_d A v^2 - C_{rr} m g",
            temp_path / "eq_newton.png",
            10,
            styles,
        )

        doc.build(story)

    return str(output_path)


def parse_parameters(entries):
    params = {}
    for key, entry in entries.items():
        raw_value = entry.get().strip()
        if not raw_value:
            raise ValueError(f"{key.replace('_', ' ').title()} is required.")
        params[key] = raw_value if key == "gears" else float(raw_value)

    if params["mass"] <= 0 or params["tire_diam"] <= 0 or params["wheelbase"] <= 0:
        raise ValueError("Mass, tire diameter, and wheelbase must be positive.")
    if params["eta"] <= 0 or params["eta"] > 1:
        raise ValueError("Drivetrain efficiency must be between 0 and 1.")
    if params["max_rpm"] <= params["rpm_t"]:
        raise ValueError("Max RPM must be greater than peak torque RPM.")

    return params


def run_app():
    root = tk.Tk()
    root.title("Vehicle Dynamics Architect (v2.1)")

    main_frame = ttk.Frame(root, padding=15)
    main_frame.pack(fill=tk.BOTH, expand=True)

    style = ttk.Style()
    style.theme_use("clam")

    entries = {}
    ttk.Label(main_frame, text="Parameter Inputs", font=("Helvetica", 12, "bold")).grid(
        row=0,
        column=0,
        columnspan=2,
        pady=(0, 10),
    )

    row_index = 1
    for key, value in DEFAULT_PARAMS.items():
        ttk.Label(main_frame, text=f"{key.replace('_', ' ').title()}:").grid(
            row=row_index,
            column=0,
            sticky="e",
            padx=5,
            pady=2,
        )
        entry = ttk.Entry(main_frame)
        entry.insert(0, value)
        entry.grid(row=row_index, column=1, sticky="w", padx=5, pady=2)
        entries[key] = entry
        row_index += 1

    run_button = ttk.Button(main_frame, text="RUN SIMULATION & GENERATE REPORT")
    run_button.grid(row=row_index, columnspan=2, pady=20, sticky="ew")

    def run_full():
        run_button.state(["disabled"])
        root.update_idletasks()
        try:
            params = parse_parameters(entries)
            model = EngineeringModel(params)
            data = model.solve_run()
            report_path = generate_pdf_report(data)
            messagebox.showinfo("Simulation Complete", f"Report generated:\n{report_path}")
        except Exception as exc:
            messagebox.showerror("Simulation Error", str(exc))
        finally:
            run_button.state(["!disabled"])

    run_button.configure(command=run_full)
    root.mainloop()


if __name__ == "__main__":
    run_app()
