# Rendering and headless operation

Verified 2026-10-01 in container `ur7e_lego` (Gazebo Harmonic gz-sim 8.11.0, ROS 2 Jazzy).

## Host
- Hybrid laptop: Intel Arrow Lake iGPU + NVIDIA RTX 5070 Laptop GPU (driver 595.91).
- X11 session on `DISPLAY=:1`, X auth at `/run/user/1001/gdm/Xauthority`.

## GPU rendering (default, `RENDER_MODE=gpu`)
Set in `docker/docker-compose.yaml`:
- GPU passthrough: `deploy.resources.reservations.devices` with `driver: nvidia`, `capabilities: [gpu]`
- `NVIDIA_DRIVER_CAPABILITIES=all`, `NVIDIA_VISIBLE_DEVICES=all`
- PRIME render offload (needed on the hybrid laptop):
  `__NV_PRIME_RENDER_OFFLOAD=1`, `__GLX_VENDOR_LIBRARY_NAME=nvidia`,
  `__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json`
- X11: mount `/tmp/.X11-unix` and the host Xauthority file; `QT_X11_NO_MITSHM=1`

Check: `glxinfo -B` inside the container reports
`OpenGL renderer string: NVIDIA GeForce RTX 5070 Laptop GPU/PCIe/SSE2`, OpenGL 4.6.
Gazebo GUI (ogre2) and RViz2 both render on the host display; RViz ~31 fps.

## Software rendering fallback (`RENDER_MODE=software`)
`docker/entrypoint.sh` unsets the NVIDIA GLX/EGL variables and sets
`LIBGL_ALWAYS_SOFTWARE=1`, `MESA_GL_VERSION_OVERRIDE=4.5` (Mesa llvmpipe;
`libgl1-mesa-dri` and `mesa-utils` are in the image). Not yet exercised with the
Gazebo GUI on this machine.

## Headless simulation
- `gazebo_gui:=false` runs `gz sim -s -r` (server only). Physics (DART) needs no GPU.
- Verified: brick drop test and full Phase 1 pick-and-place trials run headless.

## Overhead RGB-D camera (Phase 2)
- Gazebo `rgbd_camera` sensor on the D435 colour frame (`realsense2_description`
  geometry), 640x480, 69.4 deg HFOV, depth clip 0.1-3.0 m, update_rate 15 Hz,
  lens 0.95 m above the table top. One pixel is ~2 mm on the table.
- Measured on the RTX 5070 with GPU rendering, Gazebo GUI + RViz open, sim at
  RTF 1.00: colour 13.5 Hz, depth 8.9 Hz (`ros2 topic hz`, which under-reads
  large messages). Software-rendering rate not measured yet.
- ROS topics: `/camera/color/image_raw`, `/camera/color/camera_info`,
  `/camera/depth/image_raw` (32FC1, metres), `/camera/points`; all in
  `camera_color_optical_frame` (`<gz_frame_id>` is honoured, despite an sdformat
  warning during URDF conversion).
- **Gazebo's point cloud is mislabelled:** its data is in the x-forward body
  convention (`camera_color_frame`) but stamped with the optical frame, so it
  renders ~1 m off (verified by transforming points to world: table at z = 1.71 m
  instead of 0.75 m; re-interpreted in `camera_color_frame` it lands at 0.7508 m).
  It is therefore not bridged; `/camera/points` is built from depth + intrinsics by
  `depth_image_proc::PointCloudXyzrgbNode`, as with a real RealSense.
- The arm's "ready" pose blocks most of the table from the camera; the SRDF has an
  `observe` pose (shoulder turned 180 deg) that parks the arm off the table.

## Capturing images without a live window
- Gazebo GUI screenshot service saves a PNG of the 3D view regardless of window
  stacking. The request is a **directory**, not a filename:
  ```
  gz service -s /gui/screenshot --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
    --timeout 3000 --req 'data: "/home/ros/ws/media"'
  ```
- Move the GUI camera:
  ```
  gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean \
    --timeout 3000 --req 'pose: {position: {x: 1.3, y: -1.3, z: 1.6}, orientation: {x: -0.1625, y: 0.0638, z: 0.9166, w: 0.3598}}'
  ```
- Fully headless capture (no GUI) will need a camera sensor in the world; the
  `Sensors` system with `ogre2` is already loaded in `lego_phase1.sdf`.

## Gotchas
- `bash -lc` inside the container does not source ROS (login shells skip `.bashrc`);
  use `bash -c 'source ~/ws/install/setup.bash; ...'`.
- Ctrl-C on `ros2 launch` can leave the `gz sim` server running; use
  `src/lego_dualarm_bringup/scripts/kill_sim.sh`.
