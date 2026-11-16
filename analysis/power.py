"""Power budget by operating mode, endurance and solar recharge.

Corrects the 2023 budget (365.3 W / "35.3 A"), which (i) added currents drawn
from different voltage rails, (ii) used 0.2 A per Hall encoder (datasheet:
14-20 mA) and per ultrasonic sensor (15 mA), (iii) included a Kinect and a
YDLIDAR G4 that were not fitted (as-built: LDS-01 + Pi Camera v2) and
(iv) assumed all six motors at rated load continuously.

Outputs: paper/figures/fig_power_modes.pdf, analysis/out/power.json
"""
from __future__ import annotations

import json
import os

import numpy as np

import style
from style import SERIES, INK, INK2, MUTED, save, plt, COL_W, PAGE_W
from rover_core.params import DEFAULT, G_EARTH

M = DEFAULT.motor
ETA_BUCK = 0.85       # LM2596 buck converters (5 V / 6 V rails)
V_BATT = 12.0
BATT_WH = 3 * 12.0 * 15.0   # three 12 V, 15 Ah packs (as built)
DOD = 0.80
CRR = 0.05


def motor_power(torque_frac: float) -> float:
    """Electrical power of one SG775 gear-motor at a fraction of rated torque
    (linear current-torque model through no-load and rated points)."""
    i = M.no_load_current + torque_frac * (M.rated_current - M.no_load_current)
    return M.voltage * i


# typical electrical power per component [W] (datasheets / measurements)
C = {
    "rpi_idle": 2.7, "rpi_load": 6.4,                  # Raspberry Pi 4 (official/pidramble)
    "lidar": 2.0,                                       # LDS-01 5 V, <= 400 mA
    "camera": 1.25,                                     # Pi Camera v2 ~250 mA
    "hc12_idle": 0.08, "hc12_avg": 0.3,                 # 16 mA idle, 100 mA TX @20 dBm
    "unos": 2 * 0.25,                                   # payload + arm MCUs
    "mega": 1.0,                                        # Mega from 12 V (regulator incl.)
    "us": 6 * 0.075,                                    # HC-SR04 15 mA each
    "mq_heaters": 0.8 + 0.75,                           # MQ-2 + MQ-4 heaters
    "small_sensors": 0.05,                              # DHT11, BMP180, MPU6050, rain, soil
    "encoders": 6 * 12.0 * 0.020,                       # Hall encoders 20 mA @ 12 V
    "proximity": 0.12,                                  # PR08 inductive, ~10 mA @ 12 V
    "servo_hold": 6 * 0.3, "servo_move": 6 * 3.0,       # MG996R @ 6 V
}

CATS = ["Mobility motors", "Brain (Pi 4 + MCUs)", "LiDAR + camera", "Payload sensors",
        "Robotic arm", "Radio link", "Conversion losses"]


def mode(name, motors_W, rpi, lidar_cam, servo, hc12):
    five = {  # loads behind the buck converters
        "Brain (Pi 4 + MCUs)": rpi + C["unos"],
        "LiDAR + camera": lidar_cam,
        "Payload sensors": C["mq_heaters"] + C["small_sensors"] + C["us"],
        "Robotic arm": servo,
        "Radio link": hc12,
    }
    twelve = {"Mobility motors": motors_W, "Brain (Pi 4 + MCUs)": C["mega"],
              "Payload sensors": C["encoders"] + C["proximity"]}
    out = {k: 0.0 for k in CATS}
    for k, v in five.items():
        out[k] += v
    for k, v in twelve.items():
        out[k] += v
    out["Conversion losses"] = sum(five.values()) * (1 / ETA_BUCK - 1)
    return name, out


def main():
    flat_frac = (DEFAULT.mass * G_EARTH / 6 * CRR * DEFAULT.wheel_radius) / M.rated_torque
    modes = [
        mode("S0 standby", 0.0, C["rpi_idle"], 0.0, 0.0, C["hc12_idle"]),
        mode("S1 science stop", 0.0, C["rpi_load"], C["lidar"] + C["camera"],
             C["servo_hold"] + 0.3 * C["servo_move"], C["hc12_avg"]),
        mode("S2 flat traverse", 6 * motor_power(flat_frac), C["rpi_load"],
             C["lidar"] + C["camera"], 0.0, C["hc12_avg"]),
        mode("S3 rough / skid turns", 6 * motor_power(0.6), C["rpi_load"],
             C["lidar"] + C["camera"], 0.0, C["hc12_avg"]),
        mode("S4 worst case (rated)", 6 * motor_power(1.0), C["rpi_load"],
             C["lidar"] + C["camera"], C["servo_move"], C["hc12_avg"]),
    ]
    usable = BATT_WH * DOD
    res = {"battery_Wh": BATT_WH, "usable_Wh": usable, "flat_torque_fraction": round(flat_frac, 3),
           "team_budget_W": 365.33, "modes": {}}
    for name, d in modes:
        tot = sum(d.values())
        res["modes"][name] = {"total_W": round(tot, 1), "endurance_h": round(usable / tot, 2),
                              "battery_current_A": round(tot / V_BATT, 2),
                              **{k: round(v, 2) for k, v in d.items()}}
    # mission profile: 40 % flat traverse, 10 % rough, 50 % science
    prof = {"S2 flat traverse": 0.4, "S3 rough / skid turns": 0.1, "S1 science stop": 0.5}
    p_avg = sum(res["modes"][k]["total_W"] * w for k, w in prof.items())
    res["profile_avg_W"] = round(p_avg, 1)
    res["profile_endurance_h"] = round(usable / p_avg, 2)
    # solar (50 W panel, 30 deg tilt)
    for site, psh in (("Earth_BeniSuef", 6.0), ("Mars_equator_tau0.5", 3.0)):
        for ctrl, eta in (("PWM", 0.62), ("MPPT", 0.80)):
            e = 50.0 * psh * eta
            res[f"solar_{site}_{ctrl}_Wh_per_day"] = round(e, 1)
            res[f"solar_{site}_{ctrl}_recharge_days"] = round(usable / e, 2)
    res["team_claim_recharge_h"] = round(BATT_WH / 50.0, 1)
    res["stall_peak_current_A"] = 6 * M.stall_current

    # ------------------------------------------------------------ figure
    names = [n for n, _ in modes]
    fig, ax = plt.subplots(figsize=(PAGE_W, 1.95))
    y = np.arange(len(modes))[::-1]
    left = np.zeros(len(modes))
    gap = 0.0
    for ci, cat in enumerate(CATS):
        vals = np.array([d[cat] for _, d in modes])
        ax.barh(y, vals, left=left, height=0.62, color=SERIES[ci], edgecolor=style.SURFACE,
                linewidth=1.0, label=cat)
        left += vals
    for yi, (n, d) in zip(y, modes):
        tot = sum(d.values())
        ax.text(tot + 4, yi, f"{tot:.0f} W  ·  {usable / tot:.1f} h", va="center", fontsize=7, color=INK)
    ax.axvline(res["team_budget_W"], color=MUTED, lw=0.9, ls="--")
    ax.text(res["team_budget_W"] - 3, y[0] + 0.45, "2023 budget 365 W", ha="right", va="bottom",
            fontsize=6.5, color=MUTED)
    ax.set_yticks(y)
    ax.set_yticklabels(names)
    ax.set_xlabel("electrical power drawn from the 12 V battery bus [W]   (label: total · endurance on 432 Wh usable)")
    ax.set_xlim(0, 400)
    ax.grid(axis="y", visible=False)
    ax.legend(ncol=4, loc="lower right", bbox_to_anchor=(1.0, 1.0), columnspacing=1.2, handlelength=1.2)
    save(fig, "fig_power_modes")

    with open(os.path.join(style.OUT_DIR, "power.json"), "w") as f:
        json.dump(res, f, indent=1)
    print(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
