# Bill of Materials

As-built configuration (Config 5). Masses are measured unless marked *spec*.

## Propulsion

| Item | Part | Qty | Unit mass | Total | Notes |
|---|---|---|---|---|---|
| Motor | Emax ECOII-2807, 1300 Kv | 4 | 54 g | 216 g | Substitute for BrotherHobby Avenger 2806.5 (unavailable) |
| Propeller | Gemfan Flash 7042, 7.0 × 4.2, PC | 4 | 6 g | 24 g | Bi-blade |
| ESC | Holybro Tekko32 F4 4-in-1 | 1 | 13.8 g | 13.8 g | 50 A continuous / 60 A burst |

## Power

| Item | Part | Qty | Unit mass | Total | Notes |
|---|---|---|---|---|---|
| Battery | Molicel INR21700-P45B 4S1P | 1 | 313 g | 313 g | 14.4 V nominal, 4500 mAh, 64.8 Wh |

**Pack current limit:** 45 A continuous. Hover draw ≈ 16 A. Note that full-throttle
demand (4 × 47.9 A ≈ 191 A) exceeds pack capability — usable thrust ceiling is
pack-limited to roughly 900 g/motor, still ~3× hover.

## Avionics

<!-- TODO: fill in actual parts and measured masses -->

| Item | Part | Qty | Unit mass | Total |
|---|---|---|---|---|
| Flight controller | Pixhawk | 1 | | |
| GPS | | 1 | | |
| Receiver | | 1 | | |
| Wiring harness | | 1 | | |
| Power module | | 1 | | |

## Structure

| Item | Spec | Qty | Total mass |
|---|---|---|---|
| Top plate | 3 mm, X profile | 1 | |
| Bottom plate | 3 mm, X profile | 1 | |
| Arms | 12 mm OD aluminium tube, 113 mm | 4 | |
| Motor mounts | 36 mm ID / 44 mm OD, gusseted | 4 | |
| Landing gear | 9 mm aluminium rod | 4 | |
| M3 × 25 mm bolts + nuts | arm clamping | 16 | |
| M3 × 6 mm bolts | Pixhawk mounting | 4 | |
| M3 × 3 mm bolts | motors + ESC | 20 | |
| **Frame subtotal** | | | **220 g** |

## Mass summary

| Group | Mass |
|---|---|
| Propulsion | 253.8 g |
| Power | 313.0 g |
| Avionics & wiring | 150.0 g |
| Structure | 220.0 g |
| Payload | 300.0 g |
| **All-up weight** | **1236.8 g** |

## Bench test data

Emax ECOII-2807-1300Kv with Gemfan 7042.

> **Voltage caveat:** manufacturer data below is quoted at 25.2 V (6S).
> The vehicle runs 4S (14.4 V nominal). Re-measure at 4S before using these
> figures for endurance prediction — the operating point differs.

| Throttle | Current (A) | Thrust (g) | Power (W) | Efficiency (g/W) | RPM |
|---|---|---|---|---|---|
| Hover | 2.2 | 300 | 55.4 | 5.41 | 8 440 |
| Medium | 4.0 | 500 | 100.8 | 4.96 | 10 550 |
| High | 9.3 | 900 | 234.4 | 3.84 | 13 960 |
| Maximum | 47.9 | 2190 | 1207.1 | 1.81 | — |

## Derived performance

| Quantity | Value |
|---|---|
| Thrust per motor at hover | 309 g |
| Total hover power | ≈ 228 W |
| Usable battery energy (85% DoD) | 55.1 Wh |
| **Estimated hover endurance** | **≈ 14.5 min** |
| Disc loading | ≈ 122 N/m² |
| Thrust-to-weight (pack-limited) | ≈ 2.9 |
