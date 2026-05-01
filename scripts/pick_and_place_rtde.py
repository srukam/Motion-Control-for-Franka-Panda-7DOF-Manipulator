#!/usr/bin/env python3
"""
Air pick-and-place via ur_rtde — talks directly to the UR5e at 192.168.1.15.

Bypasses ROS 2 driver, MoveIt, URCap, and the headless URScript path
entirely. ur_rtde sends fire-and-forget URScript snippets per command via
the secondary client, which works regardless of controller_stopper /
pendant program state. This is the same Phase-1 pick-and-place pattern as
ur5_arm_controller, just driven directly without the ROS stack.

Sequence:
    approach -> grasp(hold) -> lift -> translate -> lower
    -> release(hold) -> retreat -> home

Run from the project root:
    python3 scripts/pick_and_place_rtde.py

Requirements:
    pip install ur_rtde

The robot only needs to be reachable on the network and in Remote control
mode. No URCap, no headless launch, no controller_manager, nothing.
"""
import sys
import time

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

ROBOT_IP = "192.168.1.15"

# Conservative defaults
VEL_LIN = 0.10   # m/s
ACC_LIN = 0.40   # m/s^2

# Air pick-and-place offsets (meters), all relative to where the EEF is now
DX_APPROACH = 0.15
DZ_APPROACH = -0.05
DZ_LIFT = 0.10
DY_TRANSLATE = 0.15
DZ_LOWER = -0.08
DX_RETREAT = -0.15


def fmt(seq, fmt_str="+.3f"):
    return "[" + ", ".join(f"{v:{fmt_str}}" for v in seq) + "]"


def main():
    print(f"connecting to UR5e at {ROBOT_IP} ...")
    try:
        rtde_c = RTDEControl(ROBOT_IP)
        rtde_r = RTDEReceive(ROBOT_IP)
    except RuntimeError as e:
        print(f"connection failed: {e}")
        return 1
    print("connected.")

    try:
        q0 = rtde_r.getActualQ()
        p0 = rtde_r.getActualTCPPose()
        print(f"start joints : {fmt(q0)}")
        print(f"start TCP    : {fmt(p0)}  (x,y,z, rx,ry,rz)")

        home_tcp = list(p0)

        def move_l_offset(dx=0.0, dy=0.0, dz=0.0, label=""):
            cur = rtde_r.getActualTCPPose()
            target = [cur[0] + dx, cur[1] + dy, cur[2] + dz,
                      cur[3], cur[4], cur[5]]
            print(f"  -> moveL {label}: dx={dx:+.3f} dy={dy:+.3f} dz={dz:+.3f}  "
                  f"target={fmt(target[:3])}")
            return rtde_c.moveL(target, VEL_LIN, ACC_LIN)

        print("\n[1/8] APPROACH (forward + down)")
        move_l_offset(dx=DX_APPROACH, dz=DZ_APPROACH, label="approach")

        print("\n[2/8] GRASP (1 s hold — no gripper wired)")
        time.sleep(1.0)

        print("\n[3/8] LIFT")
        move_l_offset(dz=DZ_LIFT, label="lift")

        print("\n[4/8] TRANSLATE (sideways +y)")
        move_l_offset(dy=DY_TRANSLATE, label="translate")

        print("\n[5/8] LOWER")
        move_l_offset(dz=DZ_LOWER, label="lower")

        print("\n[6/8] RELEASE (1 s hold)")
        time.sleep(1.0)

        print("\n[7/8] RETREAT (back -x)")
        move_l_offset(dx=DX_RETREAT, label="retreat")

        print(f"\n[8/8] RETURN HOME via moveL to start TCP {fmt(home_tcp[:3])}")
        rtde_c.moveL(home_tcp, VEL_LIN, ACC_LIN)

        q_final = rtde_r.getActualQ()
        p_final = rtde_r.getActualTCPPose()
        print(f"\nfinal joints : {fmt(q_final)}")
        print(f"final TCP    : {fmt(p_final)}")
        print("\nair pick-and-place complete.")
        return 0

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt — stopping robot")
        try:
            rtde_c.stopL(2.0)
        except Exception:
            pass
        return 130
    except Exception as e:
        print(f"FAIL: {e}")
        try:
            rtde_c.stopL(2.0)
        except Exception:
            pass
        return 1
    finally:
        try:
            rtde_c.stopScript()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
