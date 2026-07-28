# two_wheel_rails autonomous path simplification patch

Copy these files into `src/two_wheel_rails/`, rebuild, and run:

```bash
cd <워크스페이스 경로>
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select two_wheel_rails
source install/setup.bash
ros2 run two_wheel_rails autonomous_mission --table-id 2
```

The existing `rail_mission` entry point is preserved.
