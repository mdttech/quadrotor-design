# Quadrotor Design
## Design, Optimization, and Fabrication of a Quadrotor

**Conceptual design, optimization, and fabrication of a 300 g payload / 20 min endurance quadrotor**

A MATLAB sizing tool built on Blade Element Momentum Theory coupled to empirical component-weight regressions, taken from mission requirements through parametric optimization, hardware selection, CAD, fabrication, and bench validation.

Course project — AE630, Autonomous Unmanned Aerial Systems.

![Quadrotor hover test]()

<!-- TODO: replace with your own GIF. See media/README.md for how to make one. -->

---

## Mission

| Requirement | Value |
|---|---|
| Payload | 300 g |
| Hover endurance | 20 min |
| Conditions | Sea level, ρ = 1.225 kg/m³ |
| Rotors | Fixed pitch, constant chord |
| Twist distribution | θ(r) = θ₀ − 17r (deg) |
| Target disc loading | 40–90 N/m² |
| Motor efficiency | η_m = 0.80 |

---

## Approach

The design closes on gross takeoff weight through a fixed-point iteration. Every component mass depends on power, power depends on thrust, and thrust depends on the weight you are trying to find — so the loop is run to convergence for every candidate configuration in a parametric sweep.

```
        ┌─────────────────────────────────────────┐
        │  GTOW guess = 3 × payload = 900 g       │
        └────────────────────┬────────────────────┘
                             ▼
              T_hover = W/4  ·  T_design = 2·T_hover
                             ▼
              ┌──────────────────────────────┐
              │  BEMT solver                 │
              │  → C_T, C_P, Ω, P_hover      │
              └──────────────┬───────────────┘
                             ▼
              Battery energy = P_elec × endurance
                             ▼
        ┌─────────────────────────────────────────┐
        │  Empirical weight regressions           │
        │  rotor · motor · ESC · battery · frame  │
        └────────────────────┬────────────────────┘
                             ▼
                  GTOW_new = Σ components
                             ▼
              |ΔW| < 5 g ?  ──no──► relax & repeat
                             │yes
                             ▼
                       converged design
```

**Key modeling note.** With fixed-pitch blades and a linear lift-curve slope, C_T is a property of blade geometry alone and does not vary with rotor speed. This lets the BEMT solver run **once** per geometry at a reference RPM; hover conditions then follow analytically from T ∝ Ω² and P ∝ Ω³. That is what makes a full six-dimensional sweep tractable — thousands of candidate designs converge in seconds instead of hours.

**Weight model.** Component masses use the log–log multivariable regressions of Winslow, Hrishikeshavan & Chopra, *Design Methodology for Small-Scale Unmanned Quadrotors*, Journal of Aircraft (2018), [DOI: 10.2514/1.C034483](https://doi.org/10.2514/1.C034483). Their reported accuracy is ±10% per weight group and ±4% on GTOW, validated against quadrotors from 30–1000 g.

---

## Design space

Six parameters were swept, with the loop run to convergence at every point:

| Parameter | Range |
|---|---|
| Rotor radius | 0.09 – 0.25 m |
| Root pitch θ₀ | 18° – 26° |
| Aspect ratio (R/c) | 6 – 16 |
| Blades per rotor | 2, 3, 4 |
| Battery cells | 3S, 4S, 6S |
| Motor Kv | 1000 – 3000 |

Note that solidity is **not** independent: σ = N_b·c/πR and AR = R/c give σ = N_b/(π·AR). Only two of the three may be chosen freely.

### Candidate configurations

| | Config 1 | Config 2 | Config 3 |
|---|---|---|---|
| Radius (m) | 0.09 | 0.09 | 0.09 |
| θ₀ (deg) | 26 | 20 | 22 |
| Aspect ratio | 6 | 9 | 8 |
| Blades | 2 | 2 | 2 |
| Cells | 3S | 4S | 4S |
| **GTOW (kg)** | **0.816** | **1.257** | **0.998** |
| Battery (kg) | 0.164 | 0.399 | 0.253 |
| Frame (kg) | 0.225 | 0.338 | 0.274 |
| Hover RPM | 7 849 | 16 717 | 12 157 |
| Kv | 3000 | 1800 | 1300 |
| P_max/motor (W) | 47.3 | 117.9 | 72.3 |

**Config 1** is the mathematical optimum and physically unbuildable — a 12 g motor lacks the stator volume to drive a 7-inch propeller without thermal failure. **Config 2** trades into a punishing 16 700 RPM hover. **Config 3** is the realistic middle ground and became the basis for hardware selection.

<!-- TODO: add your sweep figures -->
![Parameter sensitivity](results/figures/parameter_sweep.png)

---

## As-built vehicle

Component availability drove the final build away from the theoretical optimum.

| Component | Selection | Qty | Mass |
|---|---|---|---|
| Motor | Emax ECOII-2807, 1300 Kv | 4 | 216 g |
| Propeller | Gemfan Flash 7042 (7.0 × 4.2) | 4 | 24 g |
| ESC | Holybro Tekko32 F4 4-in-1, 50 A | 1 | 13.8 g |
| Battery | Molicel INR21700-P45B 4S1P, 4500 mAh | 1 | 313 g |
| Frame | Custom, aluminium tube arms | 1 | 220 g |
| Avionics & wiring | Pixhawk, GPS, harness | — | 150 g |
| Payload | — | — | 300 g |
| **All-up weight** | | | **≈ 1237 g** |

Frame: symmetric X configuration, 3 mm sandwich plates with integrated arm clamps, 12 mm OD aluminium tube arms, gusseted motor mounts. Load path runs motor → mount → arm → clamp → base plate → centre structure.

<!-- TODO: add CAD renders -->
| Assembly | Bottom plate | Motor mount |
|---|---|---|
| ![](cad/renders/assembly_iso.png) | ![](cad/renders/bottom_plate.png) | ![](cad/renders/motor_mount.png) |

---

## Results

Bench thrust testing against the built vehicle:

| Quantity | Predicted (Config 3) | Measured |
|---|---|---|
| Hover efficiency | ≈ 9.8 g/W | 5.41 g/W |
| Hover RPM | 12 157 | 8 440 |
| Hover power (total) | ≈ 102 W | ≈ 228 W |
| Endurance | 20+ min | **≈ 15 min** |

**The vehicle did not meet the 20-minute requirement, and the gap is instructive.**

Three causes account for most of it:

1. **Blade geometry mismatch.** Config 3 models AR = 8 at R = 0.09 m, giving an 11.25 mm chord and σ = 0.080. The Gemfan 7042 actually fitted has roughly a 15–18 mm chord, σ ≈ 0.13. A narrower modeled blade needs higher RPM for the same thrust — which is exactly the 12 157 vs 8 440 RPM discrepancy.

2. **Optimistic aerodynamics.** The BEMT solver uses a constant profile drag coefficient C_d0 = 0.01 and a linear lift curve with no stall. At tip Reynolds numbers of 10⁴–10⁵ this understates profile power, and the chosen θ₀ sits in the high-loading regime where the reference paper explicitly flags BEMT correlation degrading.

3. **Weight growth from procurement.** The intended 41 g motors were unavailable; the 54 g substitutes added 52 g of propulsion mass, which cascades through frame and battery sizing. Final disc loading reached ≈ 120 N/m², well above the 40–90 N/m² band the mission specified — and high disc loading is directly a hover-efficiency penalty.

Net: the sizing model was roughly **1.8× optimistic on hover efficiency**. Closing that gap requires low-Reynolds sectional airfoil tables in place of constant coefficients, and a solidity input matched to the propeller actually procurable rather than a free sweep variable.

---

## Repository structure

```
.
├── src/
│   ├── bemt/                  BEMT solver, inflow, tip-loss
│   ├── sizing/                convergence loop, weight regressions, sweeps
│   └── utils/                 plotting and table helpers
├── results/
│   ├── figures/               sweep plots, sensitivity studies
│   ├── sweep_results.csv      full converged design table
│   └── final_design_spec.md   as-built specification
├── cad/
│   └── renders/               SolidWorks screenshots and renders
├── hardware/
│   ├── BOM.md                 bill of materials with sources
│   └── motor_test_data.csv    bench thrust stand measurements
├── media/
│   ├── images/                build and test photographs
│   └── videos/                flight and bench test footage
├── docs/
│   ├── project_report.pdf     full written report
│   └── assignment_brief.pdf   original problem statement
└── README.md
```

---

## Running the code

Requires MATLAB R2020b or newer. No toolboxes beyond base MATLAB.

```matlab
cd src/sizing
run_parameter_sweep        % full design space sweep → results/sweep_results.csv
plot_sensitivity           % regenerate all figures in results/figures/
size_single_design         % converge one configuration and print its spec table
```

To size a single point directly:

```matlab
design = struct('R', 0.09, 'theta0', 22, 'AR', 8, 'Nb', 2, ...
                'cells', 4, 'Kv', 1300, 'payload', 0.300, 'endurance', 20);
result = sizing_loop(design);
disp(result.GTOW)
```

---

## Media

<!-- TODO: fill in -->
| | |
|---|---|
| ![](media/images/build_01.jpg) | ![](media/images/thrust_stand.jpg) |
| Assembly | Thrust stand testing |

Flight and bench test footage is in [`media/videos/`](media/videos/).

---

## References

1. Winslow, J., Hrishikeshavan, V., and Chopra, I., "Design Methodology for Small-Scale Unmanned Quadrotors," *Journal of Aircraft*, 2018. [DOI: 10.2514/1.C034483](https://doi.org/10.2514/1.C034483)
2. Leishman, J. G., *Principles of Helicopter Aerodynamics*, 2nd ed., Cambridge University Press, 2006.
3. Bershadsky, D., Haviland, S., and Johnson, E. N., "Electric Multirotor Propulsion System Sizing for Performance Prediction and Design Optimization," AIAA SciTech 2016.

---

## Team

Group 3, AE630 — <!-- TODO: list names, or link GitHub profiles -->

## License

Code released under the MIT License. See [LICENSE](LICENSE).
CAD renders, photographs, and the written report are © the authors; reuse by permission.
