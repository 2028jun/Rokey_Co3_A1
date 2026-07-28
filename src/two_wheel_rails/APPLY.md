# nav_robot5 autonomous path simplification patch

Copy these files into `nav_robot5/src/two_wheel_rails/`, rebuild, and run:

```bash
cd /home/rokey/cobot3_ws/nav_robot5
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select two_wheel_rails
source install/setup.bash
ros2 run two_wheel_rails autonomous_mission --table-id 2
```

The existing `rail_mission` entry point is preserved.
