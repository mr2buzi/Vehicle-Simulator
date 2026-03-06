from dataclasses import dataclass
from typing import Mapping


GRAVITY = 9.81
AIR_DENSITY = 1.225
REPORT_FILENAME = "MultiBody_Dynamics_Analysis.pdf"
DEFAULT_ENTRY_VALUES = {
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


@dataclass(frozen=True)
class VehicleParameters:
    hp: float
    torque: float
    rpm_p: float
    rpm_t: float
    max_rpm: float
    mass: float
    gears: tuple[float, ...]
    final_drive: float
    tire_diam: float
    mu: float
    CdA: float
    Crr: float
    wheelbase: float
    cg_height: float
    eta: float
    v_target: float

    @classmethod
    def from_mapping(cls, values: Mapping[str, object]) -> "VehicleParameters":
        missing = [key for key in DEFAULT_ENTRY_VALUES if key not in values]
        if missing:
            raise ValueError(f"Missing required parameters: {', '.join(missing)}")

        gears_raw = values["gears"]
        if isinstance(gears_raw, str):
            gear_parts = [part.strip() for part in gears_raw.split(",") if part.strip()]
            gears = tuple(float(part) for part in gear_parts)
        else:
            gears = tuple(float(part) for part in gears_raw)

        if not gears:
            raise ValueError("At least one gear ratio is required.")
        if any(ratio <= 0 for ratio in gears):
            raise ValueError("All gear ratios must be positive.")

        params = cls(
            hp=float(values["hp"]),
            torque=float(values["torque"]),
            rpm_p=float(values["rpm_p"]),
            rpm_t=float(values["rpm_t"]),
            max_rpm=float(values["max_rpm"]),
            mass=float(values["mass"]),
            gears=gears,
            final_drive=float(values["final_drive"]),
            tire_diam=float(values["tire_diam"]),
            mu=float(values["mu"]),
            CdA=float(values["CdA"]),
            Crr=float(values["Crr"]),
            wheelbase=float(values["wheelbase"]),
            cg_height=float(values["cg_height"]),
            eta=float(values["eta"]),
            v_target=float(values["v_target"]),
        )
        params.validate()
        return params

    def validate(self) -> None:
        positive_fields = {
            "hp": self.hp,
            "torque": self.torque,
            "rpm_p": self.rpm_p,
            "rpm_t": self.rpm_t,
            "max_rpm": self.max_rpm,
            "mass": self.mass,
            "final_drive": self.final_drive,
            "tire_diam": self.tire_diam,
            "mu": self.mu,
            "wheelbase": self.wheelbase,
            "v_target": self.v_target,
        }
        for field_name, value in positive_fields.items():
            if value <= 0:
                raise ValueError(f"{field_name} must be positive.")

        if self.eta <= 0 or self.eta > 1:
            raise ValueError("eta must be between 0 and 1.")
        if self.max_rpm <= self.rpm_t:
            raise ValueError("max_rpm must be greater than rpm_t.")
        if self.CdA < 0 or self.Crr < 0 or self.cg_height < 0:
            raise ValueError("CdA, Crr, and cg_height cannot be negative.")

    def to_entry_values(self) -> dict[str, str]:
        return {
            "hp": str(self.hp),
            "torque": str(self.torque),
            "rpm_p": str(self.rpm_p),
            "rpm_t": str(self.rpm_t),
            "max_rpm": str(self.max_rpm),
            "mass": str(self.mass),
            "gears": ", ".join(str(ratio) for ratio in self.gears),
            "final_drive": str(self.final_drive),
            "tire_diam": str(self.tire_diam),
            "mu": str(self.mu),
            "CdA": str(self.CdA),
            "Crr": str(self.Crr),
            "wheelbase": str(self.wheelbase),
            "cg_height": str(self.cg_height),
            "eta": str(self.eta),
            "v_target": str(self.v_target),
        }


@dataclass(frozen=True)
class SimulationMetrics:
    zero_to_sixty_s: float | None
    peak_longitudinal_g: float
    top_speed_mph: float
    traction_limited_pct: float


def default_parameters() -> VehicleParameters:
    return VehicleParameters.from_mapping(DEFAULT_ENTRY_VALUES)
