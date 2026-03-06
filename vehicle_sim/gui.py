import tkinter as tk
from tkinter import messagebox, ttk

from vehicle_sim.config import DEFAULT_ENTRY_VALUES, VehicleParameters
from vehicle_sim.model import EngineeringModel, calculate_metrics
from vehicle_sim.report import generate_pdf_report


def run_app():
    root = tk.Tk()
    root.title("Vehicle Dynamics Architect")

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
    for key, value in DEFAULT_ENTRY_VALUES.items():
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

    status_var = tk.StringVar(value="Ready")
    ttk.Label(main_frame, textvariable=status_var).grid(row=row_index, column=0, columnspan=2, sticky="w", pady=(4, 0))
    row_index += 1

    run_button = ttk.Button(main_frame, text="RUN SIMULATION & GENERATE REPORT")
    run_button.grid(row=row_index, columnspan=2, pady=20, sticky="ew")

    def run_full():
        run_button.state(["disabled"])
        status_var.set("Running simulation...")
        root.update_idletasks()
        try:
            raw_values = {key: entry.get().strip() for key, entry in entries.items()}
            params = VehicleParameters.from_mapping(raw_values)
            data = EngineeringModel(params).solve_run()
            metrics = calculate_metrics(data)
            report_path = generate_pdf_report(data)
            status_var.set("Report generated")
            messagebox.showinfo(
                "Simulation Complete",
                (
                    f"0-60 mph: {metrics.zero_to_sixty_s:.2f}s\n"
                    if metrics.zero_to_sixty_s is not None
                    else "0-60 mph: N/A\n"
                )
                + f"Top speed: {metrics.top_speed_mph:.1f} mph\n"
                + f"Report: {report_path}"
            )
        except Exception as exc:
            status_var.set("Simulation failed")
            messagebox.showerror("Simulation Error", str(exc))
        finally:
            run_button.state(["!disabled"])

    run_button.configure(command=run_full)
    root.mainloop()
