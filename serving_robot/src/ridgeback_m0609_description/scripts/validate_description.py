#!/usr/bin/env python3
"""Static validation for the Ridgeback/M0609 serving robot description."""

import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
XACRO_PATH = PACKAGE_ROOT / "urdf" / "ridgeback_m0609.urdf.xacro"

EXPECTED_MOVING_JOINTS = {
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint",
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
    "upper_tray_left_slide_joint",
    "upper_tray_right_slide_joint",
    "rg2_finger_joint",
    "rg2_left_inner_finger_joint",
    "rg2_left_inner_knuckle_joint",
    "rg2_left_outer_knuckle_joint",
    "rg2_right_inner_finger_joint",
    "rg2_right_inner_knuckle_joint",
}


def fail(message):
    print(f"[FAIL] {message}", file=sys.stderr)
    raise SystemExit(1)


def main():
    if not XACRO_PATH.is_file():
        fail(f"missing xacro: {XACRO_PATH}")

    result = subprocess.run(
        ["xacro", str(XACRO_PATH)],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode:
        fail(f"xacro expansion failed:\n{result.stderr}")

    robot = ET.fromstring(result.stdout)
    links = {link.attrib["name"]: link for link in robot.findall("link")}
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
    children = {joint.find("child").attrib["link"] for joint in joints.values()}
    roots = set(links) - children
    moving = {
        name
        for name, joint in joints.items()
        if joint.attrib.get("type") != "fixed"
    }

    if robot.attrib.get("name") != "ridgeback_m0609":
        fail("unexpected robot name")
    if roots != {"ridgeback_base_link"}:
        fail(f"description must be one tree; roots={sorted(roots)}")
    if moving != EXPECTED_MOVING_JOINTS:
        fail(
            "moving joint mismatch: "
            f"missing={sorted(EXPECTED_MOVING_JOINTS-moving)}, "
            f"extra={sorted(moving-EXPECTED_MOVING_JOINTS)}"
        )

    mount = joints.get("m0609_mount_joint")
    if mount is None:
        fail("M0609 mount joint is missing")
    if mount.find("parent").attrib["link"] != "arm_mount_link":
        fail("M0609 is not attached to arm_mount_link")
    if mount.find("child").attrib["link"] != "base_link":
        fail("M0609 base link attachment is invalid")

    shelf = links.get("serving_shelf_link")
    if shelf is None or len(shelf.findall("collision")) < 8:
        fail("serving shelf collision geometry is incomplete")

    magnet = links.get("serving_tray_magnet_link")
    magnet_joint = joints.get("serving_tray_magnet_joint")
    if magnet is None or magnet_joint is None:
        fail("fixed top-deck magnet is missing")
    if magnet_joint.attrib.get("type") != "fixed":
        fail("top-deck magnet joint must be fixed")
    if magnet.findall("collision"):
        fail("flush top-deck magnet must not add collision geometry")

    for side, expected_axis in (("left", "0 1 0"), ("right", "0 -1 0")):
        link_name = f"upper_tray_{side}_link"
        joint_name = f"upper_tray_{side}_slide_joint"
        if link_name not in links:
            fail(f"sliding tray link is missing: {link_name}")
        joint = joints.get(joint_name)
        if joint is None or joint.attrib.get("type") != "prismatic":
            fail(f"sliding tray joint is invalid: {joint_name}")
        if joint.find("axis").attrib.get("xyz") != expected_axis:
            fail(f"unexpected axis for {joint_name}")
        limit = joint.find("limit").attrib
        if float(limit["lower"]) != 0.0 or float(limit["upper"]) != 0.25:
            fail(f"unexpected travel for {joint_name}")

    mass = 0.0
    for link in links.values():
        inertial = link.find("inertial")
        if inertial is not None and inertial.find("mass") is not None:
            mass += float(inertial.find("mass").attrib["value"])

    print("[PASS] ridgeback_m0609 description")
    print(f"  links={len(links)} joints={len(joints)} moving={len(moving)}")
    print(f"  modeled total mass={mass:.1f} kg")
    print("  second deck=2 powered sliding trays, travel=0.25 m/side")
    print("  top deck=embedded fixed pizza-board magnet")
    print("  TF root: ridgeback_base_link")


if __name__ == "__main__":
    main()
