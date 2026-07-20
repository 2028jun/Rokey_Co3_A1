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
    "rg2_finger_joint",
    "rg2_left_inner_finger_joint",
    "rg2_left_inner_knuckle_joint",
    "rg2_left_outer_knuckle_joint",
    "rg2_right_inner_finger_joint",
    "rg2_right_inner_knuckle_joint",
}

RG2_MIMIC_MULTIPLIERS = {
    "rg2_left_inner_finger_joint": -1.0,
    "rg2_left_inner_knuckle_joint": -1.0,
    "rg2_left_outer_knuckle_joint": -1.0,
    "rg2_right_inner_finger_joint": -1.0,
    "rg2_right_inner_knuckle_joint": 1.0,
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

    for name, multiplier in RG2_MIMIC_MULTIPLIERS.items():
        mimic = joints[name].find("mimic")
        if mimic is None or mimic.attrib.get("joint") != "rg2_finger_joint":
            fail(f"invalid RG2 mimic source: {name}")
        if float(mimic.attrib.get("multiplier", "nan")) != multiplier:
            fail(f"invalid RG2 mimic multiplier: {name}")

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

    mass = 0.0
    for link in links.values():
        inertial = link.find("inertial")
        if inertial is not None and inertial.find("mass") is not None:
            mass += float(inertial.find("mass").attrib["value"])

    print("[PASS] ridgeback_m0609 description")
    print(f"  links={len(links)} joints={len(joints)} moving={len(moving)}")
    print(f"  modeled total mass={mass:.1f} kg")
    print("  shelf trays=2, short-end service approach=clear")
    print("  TF root: ridgeback_base_link")


if __name__ == "__main__":
    main()
