# Branch Worktrees

This repo now uses separate git worktrees so each branch can keep its own:

- `ros2_ws/build`
- `ros2_ws/install`
- `ros2_ws/log`
- `.roslog`
- `.ros_motion_pids`
- `runtime_logs`

That avoids branch switching corrupting ROS artifacts or mixing binaries from different branches.

## Paths

- `ros_branch` workspace:
  - `/home/ws/ugv_rpi`
- `main` workspace:
  - `/home/ws/ugv_rpi/.worktrees/main`

## Rule

Always build and run from the worktree directory of the branch you want to test.

Do not switch branches in-place and reuse the same build output.

## Examples

Run `ros_branch`:

```bash
cd /home/ws/ugv_rpi
bash ./restart_ros_slam_stack.sh
```

Open `main`:

```bash
cd /home/ws/ugv_rpi/.worktrees/main
```

Build current worktree:

```bash
cd /path/to/the/worktree
bash ./safe_colcon_build.sh rasprover_bringup
```

## Notes

- The `main` worktree is currently the older pre-ROS baseline branch. It is isolated so it cannot overwrite the ROS branch build artifacts.
- The `safe_colcon_build.sh` helper is available in the ROS worktree. If you create more ROS worktrees later, use the same pattern there.
- Large local dependency sources under `ros2_ws/src/third_party/` are ignored by git and stay local to each worktree.
- Maps in `maps/` are local runtime data and are ignored by git.
- Do not run robot stacks from two worktrees at the same time. The Pi, serial ports, LiDAR, and ROS graph are still shared physical resources.
