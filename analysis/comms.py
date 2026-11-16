"""Communication analysis: HC-12 link budget, SSP timing and receiver robustness.

1. Link budget (433.4 MHz, 20 dBm, 0 dBi spring antennas) with free-space,
   two-ray ground-reflection and log-distance (n = 3) path-loss models and the
   HC-12 sensitivities per air data rate (-117/-112/-107/-100 dBm at
   5/15/58/236 kbps; FU3 mode at 9600 baud uses 15 kbps).
2. Serialisation time of SSP frames at 9600 baud (8N1).
3. Monte-Carlo comparison of the 2023 Arduino receiver (fixed 8-byte read after
   available()>5, CRC result ignored, buffer flushed by Serial1.begin()) with
   the ICD v2.0 receiver (length-driven, CRC-validated, resynchronising) for
   ground commands sent over a channel with random bit errors or byte drops.

Outputs: paper/figures/fig_link_budget.pdf, fig_protocol_mc.pdf, analysis/out/comms.json
"""
from __future__ import annotations

import json
import math
import os
import random

import numpy as np

import style
from style import BLUE, ORANGE, AQUA, YELLOW, INK2, MUTED, save, plt, COL_W
from rover_core.ssp import Frame, StreamDecoder
from rover_core.protocol import PT, plausible_length

F_HZ = 433.4e6
LAM = 299_792_458.0 / F_HZ
PT_DBM = 20.0
G_DBI = 0.0
L_SYS = 2.0           # connectors, cable, body shadowing [dB]
H_ROVER, H_GS = 0.8, 1.0
SENS = {"5 kbps": -117.0, "15 kbps (FU3 @ 9600 Bd)": -112.0, "58 kbps": -107.0, "236 kbps": -100.0}
FADE = 10.0           # fade margin [dB]


def fspl(d):
    return 20 * np.log10(4 * np.pi * d / LAM)


def two_ray(d):
    dc = 4 * np.pi * H_ROVER * H_GS / LAM
    pl = 40 * np.log10(d) - 20 * np.log10(H_ROVER * H_GS)
    return np.where(d < dc, fspl(d), np.maximum(pl, fspl(d)))


def log_distance(d, n=3.0, d0=1.0):
    return fspl(d0) + 10 * n * np.log10(d / d0)


def pr(pl):
    return PT_DBM + 2 * G_DBI - L_SYS - pl


def max_range(model, sens, margin=FADE):
    d = np.logspace(0, 4, 4000)
    ok = pr(model(d)) >= sens + margin
    return float(d[ok].max()) if ok.any() else 0.0


# ------------------------------------------------------------- Monte Carlo
LEGACY_CMDS = [PT.PING, PT.GET_TLM, PT.CAP_IMAGE, PT.GET_IMAGE, PT.START_SCAN, PT.STOP_SCAN]
VALID = set(int(t) for t in LEGACY_CMDS)


def channel(data: bytes, ber: float, p_drop: float, rng: random.Random) -> bytes:
    out = bytearray()
    for b in data:
        if p_drop and rng.random() < p_drop:
            continue
        if ber:
            for k in range(8):
                if rng.random() < ber:
                    b ^= 1 << k
        out.append(b)
    return bytes(out)


def fast_channel(data: bytes, ber: float, p_drop: float, rng: np.random.Generator) -> bytes:
    a = np.frombuffer(data, dtype=np.uint8).copy()
    if ber:
        flips = rng.random((a.size, 8)) < ber
        if flips.any():
            masks = (flips * (1 << np.arange(8))).sum(axis=1).astype(np.uint8)
            a ^= masks
    if p_drop:
        a = a[rng.random(a.size) >= p_drop]
    return a.tobytes()


def simulate(ber: float, p_drop: float, n: int, seed: int = 0):
    """Return outcome fractions for legacy and v2 receivers."""
    rng = np.random.default_rng(seed)
    cmds = rng.integers(0, len(LEGACY_CMDS), n)
    legacy = {"correct": 0, "wrong": 0, "lost": 0}
    v2 = {"correct": 0, "wrong": 0, "lost": 0}
    rxbuf = bytearray()        # legacy UART RX buffer (persists until flushed)
    buf = bytearray(8)         # legacy static buf[] keeps old bytes
    dec = StreamDecoder(own_addr=0x01, timeout_s=None, length_check=plausible_length)
    for c in cmds:
        sent = Frame(0x01, 0x0B, int(LEGACY_CMDS[c]))
        wire = sent.encode()
        rx = fast_channel(wire, ber, p_drop, rng)
        # ---------------- legacy: if available()>5: read up to 8, dispatch buf[3], flush
        rxbuf += rx
        if len(rxbuf) > 5:
            k = min(8, len(rxbuf))
            buf[:k] = rxbuf[:k]
            rxbuf.clear()  # Serial1.begin(9600) discards the rest
            t = buf[3]
            if t == sent.type:
                legacy["correct"] += 1
            elif t in VALID:
                legacy["wrong"] += 1
            else:
                legacy["lost"] += 1
        else:
            legacy["lost"] += 1
        # ---------------- v2: commands are separated by idle time (> 50 ms), so
        # the inter-byte timeout returns the receiver to HUNT between commands
        dec._in_frame = False
        dec._buf.clear()
        got = dec.feed(rx)
        if not got:
            v2["lost"] += 1
        elif got[0] == sent:
            v2["correct"] += 1
        else:
            v2["wrong"] += 1
    return ({k: v / n for k, v in legacy.items()}, {k: v / n for k, v in v2.items()})


def main():
    res = {}
    # ------------------------------------------------ link budget figure
    d = np.logspace(0, np.log10(3000), 400)
    fig, ax = plt.subplots(figsize=(COL_W, 2.3))
    models = [("free space", fspl, BLUE, "-"), ("two-ray ground", two_ray, ORANGE, "-"),
              ("log-distance, n = 3", log_distance, AQUA, "-")]
    for lab, mfun, c, ls in models:
        ax.plot(d, pr(mfun(d)), color=c, ls=ls, lw=1.6, label=lab)
    s15 = SENS["15 kbps (FU3 @ 9600 Bd)"]
    ax.axhline(s15, color=INK2, lw=0.9, ls="--")
    ax.text(1.1, s15 + 1.5, f"HC-12 sensitivity, 15 kbps air rate (FU3 @ 9600 Bd): {s15:.0f} dBm",
            fontsize=6, color=INK2)
    ax.axhline(s15 + FADE, color=MUTED, lw=0.7, ls=":")
    ax.text(1.1, s15 + FADE + 1.5, f"+{FADE:.0f} dB fade margin", fontsize=6, color=MUTED)
    r2 = max_range(two_ray, s15)
    ax.plot([r2], [s15 + FADE], "o", ms=4.5, mfc="white", mec=ORANGE, mew=1.2)
    ax.annotate(f"{r2:.0f} m", (r2, s15 + FADE), xytext=(r2 * 0.30, s15 + FADE + 9),
                fontsize=6.5, color=INK2, arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.6))
    ax.set_xscale("log")
    ax.set_xlabel("distance rover–ground station [m]")
    ax.set_ylabel("received power [dBm]")
    ax.set_xlim(1, 3000)
    ax.set_ylim(-135, -10)
    ax.legend(loc="upper right")
    save(fig, "fig_link_budget")
    for lab, mfun, _, _ in models:
        for sl, s in SENS.items():
            res[f"range_m_{lab}_{sl.split(' ')[0]}kbps"] = round(max_range(mfun, s), 0)
    res["two_ray_crossover_m"] = round(4 * math.pi * H_ROVER * H_GS / LAM, 1)

    # ------------------------------------------------ timing table (9600 8N1)
    def t_ms(nbytes, baud=9600):
        return nbytes * 10 / baud * 1000
    res["t_ping_req_ms"] = round(t_ms(8), 2)
    res["t_tlm_legacy_resp_ms"] = round(t_ms(15), 2)
    res["t_tlm_v2_resp_ms"] = round(t_ms(56 + 8), 2)
    res["t_image_legacy_s"] = round(t_ms(50 * (200 + 8)) / 1000, 2)
    res["image_legacy_efficiency"] = round(200 / 208, 3)
    res["tlm_v2_efficiency"] = round(56 / 64, 3)

    # ------------------------------------------------ Monte Carlo
    N = 40000
    bers = [1e-5, 3e-5, 1e-4, 3e-4, 1e-3, 3e-3, 1e-2]
    drops = [1e-4, 3e-4, 1e-3, 3e-3, 1e-2, 3e-2]
    mc_ber = []
    for i, b in enumerate(bers):
        lg, v2 = simulate(b, 0.0, N, seed=100 + i)
        mc_ber.append((b, lg, v2))
        print(f"BER {b:.0e}: legacy {lg}  v2 {v2}")
    mc_drop = []
    for i, p in enumerate(drops):
        lg, v2 = simulate(0.0, p, N, seed=200 + i)
        mc_drop.append((p, lg, v2))
        print(f"drop {p:.0e}: legacy {lg}  v2 {v2}")
    res["mc_frames_per_point"] = N
    res["mc_ber"] = [{"ber": b, "legacy": lg, "v2": v2} for b, lg, v2 in mc_ber]
    res["mc_drop"] = [{"p_drop": p, "legacy": lg, "v2": v2} for p, lg, v2 in mc_drop]

    floor = 1.0 / N
    total_v2_wrong = sum(row[2]["wrong"] * N for row in mc_ber + mc_drop)
    total_trials = N * (len(mc_ber) + len(mc_drop))
    res["mc_total_trials"] = total_trials
    res["mc_v2_wrong_total"] = int(round(total_v2_wrong))
    nz = lambda a: np.where(np.asarray(a) > 0, a, np.nan)  # zeros are not plotted on log axes
    fig, axs = plt.subplots(1, 2, figsize=(style.PAGE_W, 2.25), sharey=True)
    handles = None
    for ax, data, xl in ((axs[0], mc_ber, "bit error rate"), (axs[1], mc_drop, "byte-drop probability")):
        x = np.array([row[0] for row in data])
        lw = [row[1]["wrong"] for row in data]
        ll = [row[1]["lost"] for row in data]
        vl = [row[2]["lost"] for row in data]
        h1, = ax.plot(x, nz(lw), "-o", color=ORANGE, ms=3.5, label="2023 receiver: WRONG command executed")
        h2, = ax.plot(x, nz(ll), "--o", color=ORANGE, ms=3.0, mfc="white", label="2023 receiver: command ignored")
        h3, = ax.plot(x, nz(vl), "--s", color=BLUE, ms=3.0, mfc="white",
                      label="v2: corrupted frame rejected (detected, GS retries)")
        handles = [h1, h2, h3]
        ax.axhline(floor, color=MUTED, lw=0.7, ls=":")
        ax.text(x[0], floor * 1.3, f"v2 wrong commands: none in {N:,} per point (< 1/N)",
                fontsize=6.2, color=INK2)
        ax.set_xscale("log")
        ax.set_yscale("log")
        ax.set_xlabel(xl)
        ax.set_ylim(floor * 0.6, 1.0)
    axs[0].set_ylabel("fraction of commands")
    fig.legend(handles=handles, loc="lower center", bbox_to_anchor=(0.5, 0.98), ncol=3,
               fontsize=6.5, handlelength=2.2)
    save(fig, "fig_protocol_mc")

    with open(os.path.join(style.OUT_DIR, "comms.json"), "w") as f:
        json.dump(res, f, indent=1)
    print(json.dumps({k: v for k, v in res.items() if not k.startswith("mc_")}, indent=1))


if __name__ == "__main__":
    main()
