"""Repeat the SLAM study over several random seeds (sensor noise, slip,
chi mismatch draw) and report mean +- sd. Output: analysis/out/slam_seeds.json"""
import json
import os

import numpy as np

import style
import sim_slam

if __name__ == "__main__":
    style.save = lambda fig, name: style.plt.close(fig)   # figures only from the main run
    sim_slam.save = style.save
    rows = []
    for seed in (1, 2, 3, 4, 5, 6, 7, 8):
        sim_slam.main(seed=seed)
        with open(os.path.join(style.OUT_DIR, "slam.json")) as f:
            rows.append(json.load(f))
    keys = ["ate_rmse_odometry_m", "ate_rmse_slam_m", "heading_rmse_odometry_deg", "heading_rmse_slam_deg",
            "final_err_odometry_m", "final_err_slam_m"]
    out = {k: [round(float(np.mean([r[k] for r in rows])), 3), round(float(np.std([r[k] for r in rows])), 3)]
           for k in keys}
    for m in ("map_odometry", "map_slam"):
        for k in ("accuracy_mean_m", "accuracy_p95_m", "completeness_10cm", "spurious_gt15cm"):
            v = [r[m][k] for r in rows]
            out[f"{m}.{k}"] = [round(float(np.mean(v)), 3), round(float(np.std(v)), 3)]
    out["collisions_total"] = int(sum(r["collisions"] for r in rows))
    out["seeds"] = len(rows)
    with open(os.path.join(style.OUT_DIR, "slam_seeds.json"), "w") as f:
        json.dump(out, f, indent=1)
    print(json.dumps(out, indent=1))
