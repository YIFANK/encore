#!/usr/bin/env python3
"""Measure table_z by hand-guiding the RIGHT follower's fingertip to the table.

Puts the arm in gravity-compensated float (external_effort mode, zero effort),
you pull the closed fingertips down until they touch the table, press Enter,
and the script records the cartesian z — that IS table_z in Heron's world
frame (the right-follower base frame). Optionally writes it into the config.

SAFETY: keep one hand on the arm the whole time (imperfect gravity comp can
drift), estop.py running in another terminal, workspace clear.

    .venv/bin/python tools/measure_table.py configs/abaka.yaml
"""
from __future__ import annotations

import sys
import time

import numpy as np
import yaml


def main() -> int:
    cfg_path = sys.argv[1] if len(sys.argv) > 1 else "configs/abaka.yaml"
    cfg = yaml.safe_load(open(cfg_path))
    ip = cfg["arms"]["right"]["ip"]
    import trossen_arm as ta

    print(f"connecting to right follower @ {ip} …")
    drv = ta.TrossenArmDriver()
    drv.configure(ta.Model.wxai_v0, ta.StandardEndEffector.wxai_v0_follower, ip, False)
    input("\nHANDS ON THE ARM. Press Enter to enable gravity-comp float mode … ")
    try:
        drv.set_all_modes(ta.Mode.external_effort)
        drv.set_all_external_efforts(np.zeros(drv.get_num_joints()).tolist(), 0.0, False)
        print("floating. Guide the CLOSED fingertips down until they TOUCH the table.")
        print("Live z (Ctrl-C aborts):")
        z = None
        try:
            while True:
                pose = np.asarray(drv.get_cartesian_positions())
                print(f"\r  z = {pose[2]:+.4f} m   (Enter in this terminal when touching)", end="")
                time.sleep(0.15)
                import select

                if select.select([sys.stdin], [], [], 0)[0]:
                    sys.stdin.readline()
                    z = float(pose[2])
                    break
        except KeyboardInterrupt:
            print("\naborted; no measurement taken")
        if z is not None:
            print(f"\n\nmeasured table_z = {z:.4f} m (right-follower base frame)")
            if input("write into config? [y/N] ").strip().lower() == "y":
                text = open(cfg_path).read()
                import re

                text = re.sub(r"^table_z:.*$", f"table_z: {z:.4f}", text, count=1, flags=re.M)
                open(cfg_path, "w").write(text)
                print(f"updated {cfg_path}")
    finally:
        print("switching back to position hold, then idle …")
        try:
            drv.set_all_modes(ta.Mode.position)
            drv.set_all_positions(np.asarray(drv.get_all_positions()), 0.5, True)
        except Exception:
            pass
        drv.cleanup()
    print("done; arm idled (braked).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
