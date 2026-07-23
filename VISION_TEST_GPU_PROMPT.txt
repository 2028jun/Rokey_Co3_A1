# Vision-test GPU follow-up prompt: finish the two remaining visual failures

Target: Isaac Sim 5.1.0-rc.19, ROS 2 Humble, RTX 5080, branch
`vision-test`, `ROS_DOMAIN_ID=102`.

Read `AGENTS.md` and `GPU_RUN_LOG.txt` completely. Preserve existing work and
the untracked generated robot USD. Do not repeat Pass 6 geometry searches or
Pass 7's terminal/YAML diagnosis.

## Current state

Pass 7 moved the person out of the table to a standing customer position,
added coherent 0.85 uniform scaling, moved the intrusion target to the
customer-side edge, replaced spherical joint caps with tapered transitions,
and made annotated ROS output plus `rqt_image_view` the documented workflow.

The real GPU workflow now starts successfully and publishes
`/hand_detection/image`, but visual acceptance is not complete:

1. The untouched opposite arm remains in the source asset's spread pose.
2. The detached reaching glove stopped near the chair in the forced full-reach
   capture instead of entering the ROI. The endpoint code was subsequently
   changed to author exact reach rotations but has not been re-rendered.

## Required work

- Re-run a forced full-reach capture first. Confirm whether the exact endpoint
  fix lands the glove at `TABLE_HAND_TARGET=(-3.25,-3.33,0.98)`.
- If it does not, print/measure the rendered wrist world transform and correct
  the transform/scale composition. Do not tune YOLO or move the ROI.
- Put the opposite arm in a relaxed pose. Prefer extracting it into a static
  Xform rig or rebuilding the body mesh without that arm and adding a clean
  relaxed replacement. Do not hide the entire person or accept a T-pose.
- Inspect tapered shoulder/elbow transitions at rest, mid-reach, and full
  reach. Correct holes, detachment, or obvious cone silhouettes with
  overlapping transition mesh geometry.
- Verify the person stays outside table/chair geometry and the root remains
  fixed for at least four complete cycles.
- Run the four-terminal workflow in `hand_safety/README.md`, manually publish
  `table_arrived=true`, view `/hand_detection/image`, and confirm at least one
  annotated hand frame and ROI true pulse.

Capture front, side, overhead, fixed-camera, and shoulder/elbow close-up views.
Append a dated GPU log entry, rewrite this prompt to only remaining work, then
commit and push tested results to `vision-test`.
