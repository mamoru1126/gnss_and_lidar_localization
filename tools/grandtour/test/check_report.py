#!/usr/bin/env python3
"""合成ミッション（make_fake_missions.py）の事前確認の結果が、作ったときの真値に合うかを確かめる。

  python3 check_report.py <inspect_grandtour.py の --out>
"""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from make_fake_missions import T_BASE_BOXBASE as BOX  # noqa: E402

out = Path(sys.argv[1])
js = json.loads((out / "results.json").read_text(encoding="utf-8"))
m, ov = js["missions"], js["overlap"]
errors = []


def check(cond, msg):
    print(("ok   " if cond else "FAIL ") + msg)
    if not cond:
        errors.append(msg)


check(set(m) == {"ETH-1", "ETH-3", "SBB-1"}, f"3 missions read: {sorted(m)}")
check(sorted(sorted(g) for g in js["sites"]) == [["ETH-1", "ETH-3"], ["SBB-1"]], f"sites: {js['sites']}")
o = ov.get("ETH-1->ETH-3", {})
check(0.55 < o.get("frac", 0) < 0.8, f"ETH-1->ETH-3 overlap frac {o.get('frac')}（約 0.67）")
check(o.get("opposite", 0) > 40 and o.get("same", 1e9) < 10, f"ETH-1->ETH-3 mostly opposite: {o}")
check(not any("SBB-1" in k for k in ov), "SBB-1 has no overlap entries")
e1 = m["ETH-1"]
check(abs(e1["stationary_start"] - 3.0) < 0.3, f"ETH-1 stationary start {e1['stationary_start']:.2f} s（3 s）")
check(abs(e1["length"] - 120.0) < 3.0, f"ETH-1 path length {e1['length']:.1f} m（120 m）")
check(e1["gnss_gaps"] == 1 and abs(e1["gnss_gap_total"] - 3.1) < 0.3, f"ETH-1 GNSS gap {e1['gnss_gaps']} / {e1['gnss_gap_total']:.1f} s")
check(abs(e1["gnss_std_p50"] - 0.02) < 1e-6, f"ETH-1 GNSS std {e1['gnss_std_p50']}")
for c in ("ETH-1", "ETH-3", "SBB-1"):
    cl, ct = m[c]["conv_leg"], m[c]["conv_tc"]
    check(cl["semantics"] == "standard" and cl["twist_frame"] == "child", f"{c} leg odometry: {cl['semantics']}, twist {cl['twist_frame']}")
    check(ct["semantics"] == "standard" and ct["twist_frame"] == "child" and ct["decided_by"] == "twist",
          f"{c} tc odometry: {ct['semantics']}, twist {ct['twist_frame']}, by {ct['decided_by']}")
    check(abs(abs(ct["walk_dir_deg"]) - 170) < 5, f"{c} tc frame walks at about ±170° (box_base yaw 170°): {ct['walk_dir_deg']:.0f}")
    tc = m[c].get("tf_check", {})
    check(tc.get("diff_official_deg", 99) < 1.0 and tc.get("diff_inverse_deg", 0) > 5.0,
          f"{c} static tf rotation: official {tc.get('diff_official_deg')} / inverse {tc.get('diff_inverse_deg')}")
    if c != "SBB-1":  # 直線だけだと並進の水平は決まらない
        te = np.asarray(tc.get("t_est", [9, 9, 9]))
        check(np.allclose(te[:2], np.array(BOX)[:2, 3], atol=0.03) and tc["t_diff_official_xy"] < 0.5 * tc["t_diff_inverse_xy"],
              f"{c} static tf translation: est {np.round(te, 3)} (true {np.array(BOX)[:3, 3]}), "
              f"diff official {tc.get('t_diff_official_xy')} / inverse {tc.get('t_diff_inverse_xy')}")
    pf = m[c].get("prism_fit", {})
    check(pf.get("rms", 1) < 0.01, f"{c} prism fit rms {pf.get('rms')}")
    lv = pf.get("lever") or [9, 9, 9]
    # プリズムは base で (-0.20, 0.05, 0.60)。真値のフレーム（box_base）で表すと、おおよそ (0.30, -0.10, 0.35)
    want = np.linalg.inv(np.array(BOX))[:3] @ np.array([-0.20, 0.05, 0.60, 1.0])
    if c == "SBB-1":  # 直線だけなので、プリズムの横の位置は決まらない
        check(pf.get("yaw_span_deg", 99) < 45, f"{c} prism lever marked unobservable (yaw span {pf.get('yaw_span_deg')})")
    else:
        check(np.allclose(lv[:2], want[:2], atol=0.03), f"{c} prism lever {np.round(lv, 3)}（box_base で {np.round(want, 3)}）")
    li = m[c].get("lidar", {})
    check(abs(li.get("base_height", 0) - 0.55) < 0.05, f"{c} base height {li.get('base_height')}（0.55）")
    check("intensity" in li.get("fields", []) and "time" not in li.get("fields", []), f"{c} livox fields {li.get('fields')}")
    check(abs(m[c]["adis_imu"]["acc_norm"] - 9.81) < 0.1, f"{c} adis acc {m[c]['adis_imu']['acc_norm']:.3f}（m/s²）")
    check(abs(m[c]["livox_imu"]["acc_norm"] - 1.0) < 0.02, f"{c} livox imu acc {m[c]['livox_imu']['acc_norm']:.3f}（g）")
rep = (out / "report.md").read_text(encoding="utf-8")
check("## 2. ミッションどうしの経路の重なり" in rep and "site_ETH-1_ETH-3.png" in rep, "report.md has overlap section and figure")
check((out / "site_ETH-1_ETH-3.png").exists() and (out / "site_SBB-1.png").exists(), "site figures written")

if errors:
    print(f"\n{len(errors)} check(s) failed")
    sys.exit(1)
print("\nall checks passed")
