#!/usr/bin/env python3
"""A/B test for ERP42 idle rearward drift in Gazebo Classic / ROS 2 Humble.

Run only inside WSL Ubuntu-22.04 with the ROS installation sourced and with no
Gazebo server already running. Temporarily modifies *installed* Xacro and the
spawn launch file, preserving and restoring both byte-for-byte in finally.
Does not change source files or leave any Gazebo processes running.

Example:
  python3 compare_idle_drift_wsl.py baseline damping2 friction02
"""

import argparse
import csv
import datetime
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time


PKG = Path("/root/erp42_ws/install/erp42/share/erp42")
MACROS = PKG / "urdf/macros_erp42.xacro"
LAUNCH = PKG / "launch/erp42.launch.py"
OUT = Path("/root/erp42_ws/log/idle_drift_ab")
CASES = {
    "baseline": (0.0, 0.0, 0.32),
    "damping2": (2.0, 0.0, 0.32),
    "damping10": (10.0, 0.0, 0.32),
    "friction02": (0.0, 0.2, 0.32),
    "combo": (2.0, 0.2, 0.32),
    "spawn28": (0.0, 0.0, 0.28),
    "combo_spawn28": (2.0, 0.2, 0.28),
    "no_controllers": (0.0, 0.0, 0.32),
}


def check_no_server():
    r = subprocess.run(["pgrep", "-x", "gzserver"], capture_output=True)
    if r.returncode == 0:
        raise RuntimeError("An existing gzserver is running. Stop it first.")


def set_macros(original, damping, friction):
    text = original.decode("utf-8")
    count = 0
    for name in ("wheel_front_joint", "wheel_back_joint"):
        pattern = (
            r'(<xacro:macro name="' + name +
            r'"[^>]*>.*?<joint[^>]*>.*?<origin xyz="\$\{x\} \$\{y\} \$\{z\}"\s*/>)'
        )
        def insert(match):
            nonlocal count
            count += 1
            value = match.group(1)
            if damping == 0 and friction == 0:
                return value
            return value + f'\n      <dynamics damping="{damping:g}" friction="{friction:g}" />'
        text, changed = re.subn(pattern, insert, text, count=1, flags=re.DOTALL)
        if changed != 1:
            raise RuntimeError(f"Expected wheel joint macro not found: {name}")
    if count != 2:
        raise RuntimeError(f"Unexpected wheel joint edits: {count}")
    return text.encode("utf-8")


def set_launch(original, height, no_controllers=False):
    text = original.decode("utf-8")
    if height != 0.32:
        assert '"-z", "0.32"' in text
        text = text.replace('"-z", "0.32"', '"-z", "0.28"')
    if no_controllers:
        old = '"joint_state_broadcaster", "steering", drive,'
        assert old in text
        text = text.replace(old, '"joint_state_broadcaster",')
    return text.encode("utf-8")


def query_pose():
    result = subprocess.run(
        ["timeout", "4s", "gz", "model", "-m", "erp42", "-p"],
        capture_output=True, text=True, timeout=6,
    )
    if result.returncode != 0:
        return None
    try:
        parts = [float(x) for x in result.stdout.split()]
        return parts if len(parts) == 6 else None
    except ValueError:
        return None


def stop_sim(proc):
    if proc.poll() is not None:
        return
    for sig, delay in ((signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=delay)
            return
        except subprocess.TimeoutExpired:
            continue


def run_case(case, sample_seconds, settle_seconds):
    OUT.mkdir(parents=True, exist_ok=True)
    logfile = OUT / f"{case}.gazebo.log"
    with logfile.open("w", encoding="utf-8") as log:
        proc = subprocess.Popen(
            ["ros2", "launch", "erp42", "erp42.launch.py", "gui:=false", "localization:=false"],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
            env={**os.environ, "GAZEBO_MODEL_DATABASE_URI": ""},
        )
        try:
            started = time.monotonic()
            while time.monotonic() - started < 40:
                if proc.poll() is not None:
                    raise RuntimeError(f"launch ended early: {logfile}")
                pose = query_pose()
                if pose is not None:
                    break
                time.sleep(1)
            else:
                raise RuntimeError(f"Timed out waiting for ERP42 pose: {logfile}")

            time.sleep(settle_seconds)
            samples = []
            started = time.monotonic()
            while True:
                pose = query_pose()
                now = time.monotonic() - started
                if pose is not None:
                    samples.append((now, pose))
                    print(f"{case:16s} t={now:6.2f}s x={pose[0]:+.7f} "
                          f"z={pose[2]:.6f} pitch={pose[4]:+.7f}", flush=True)
                if now >= sample_seconds:
                    break
                time.sleep(min(2.0, sample_seconds - now))
            if len(samples) < 2:
                raise RuntimeError(f"Insufficient Gazebo pose samples: {case}")
            first, last = samples[0], samples[-1]
            dx = last[1][0] - first[1][0]
            duration = last[0] - first[0]
            result = {
                "case": case, "dx_m": round(dx, 8),
                "drift_m_per_s": round(dx / duration, 9),
                "duration_wall_s": round(duration, 2),
                "start_x": round(first[1][0], 7),
                "end_x": round(last[1][0], 7),
                "z_m": round(last[1][2], 6),
                "pitch_rad": round(last[1][4], 8),
                "samples": len(samples),
                "gazebo_log": str(logfile),
            }
            print("RESULT:", result, flush=True)
            return result
        finally:
            stop_sim(proc)
            time.sleep(2)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("cases", nargs="*", default=["baseline", "damping2", "friction02", "combo", "spawn28"])
    parser.add_argument("--measure", type=float, default=12)
    parser.add_argument("--settle", type=float, default=3)
    args = parser.parse_args()
    if args.measure < 5 or args.measure > 60:
        parser.error("Measure duration must be between 5 and 60 seconds")
    for name in args.cases:
        if name not in CASES:
            parser.error(f"Unknown case {name}; choices: {sorted(CASES)}")
    check_no_server()
    macro_original = MACROS.read_bytes()
    launch_original = LAUNCH.read_bytes()
    results = []
    try:
        for name in args.cases:
            damping, friction, height = CASES[name]
            print(f"CASE {name} damping={damping} friction={friction} spawn_z={height}", flush=True)
            MACROS.write_bytes(set_macros(macro_original, damping, friction))
            LAUNCH.write_bytes(set_launch(launch_original, height, no_controllers=name == "no_controllers"))
            results.append(run_case(name, args.measure, args.settle))
    finally:
        MACROS.write_bytes(macro_original)
        LAUNCH.write_bytes(launch_original)
        print("RESTORED original installed macros and launch byte-for-byte", flush=True)
    if results:
        path = OUT / "results.csv"
        with path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=list(results[0]))
            writer.writeheader()
            writer.writerows(results)
        print("RESULT_CSV=", path, flush=True)


if __name__ == "__main__":
    main()
