# Sawyer Native ROS 1 ↔ ROS 2 Bridge Setup with RoboStack

This guide explains how to set up a fresh Ubuntu laptop to run a **native ROS 1 Noetic ↔ ROS 2 Humble bridge** using RoboStack/Pixi, without Docker and without installing ROS 1 system-wide.

The goal of this setup is to reach the point where:

- ROS 1 Noetic works inside RoboStack
- ROS 2 Humble works inside RoboStack
- `ros1_bridge` is built from source
- Standard ROS 1 ↔ ROS 2 message mappings are generated
- `sensor_msgs/JointState` is confirmed as a supported bridge type
- A local ROS 1 → ROS 2 bridge test can be performed

This document intentionally stops **before connecting to the physical Sawyer robot**.

---

## 1. Target Architecture

```text
ROS 1 application / Sawyer
        |
        | ROS 1
        v
RoboStack Noetic
        |
        v
ros1_bridge
        |
        | ROS 2 DDS
        v
RoboStack Humble
        |
        v
ROS 2 application
```

The important point is that Sawyer remains ROS 1-based. RoboStack provides the ROS 1 and ROS 2 userspace environments natively on the host, while `ros1_bridge` performs translation between the two middleware ecosystems.

---

## 2. Tested Platform

This procedure was developed for:

- Ubuntu 22.04
- x86_64 laptop
- ROS 1 Noetic through RoboStack
- ROS 2 Humble through RoboStack
- Pixi package/environment manager
- Python 3.12
- `ros1_bridge` built from source

---

## 3. Important Rules Before Starting

Do not source a system ROS installation while working with RoboStack.

A shell containing something such as:

```bash
source /opt/ros/humble/setup.bash
```

can contaminate the RoboStack environment.

Check the current shell:

```bash
env | grep -E 'ROS|AMENT|COLCON|PYTHONPATH'
```

On a clean terminal, ideally nothing ROS-related should appear.

If ROS is automatically sourced in `~/.bashrc`, inspect it with:

```bash
grep -n "ros/" ~/.bashrc
```

Comment out lines such as:

```bash
# source /opt/ros/humble/setup.bash
```

Then open a new terminal.

---

# 4. Install Pixi

Install Pixi:

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

Close the terminal and open a new one.

Verify:

```bash
pixi --version
```

---

# 5. Create the RoboStack Workspace

Create a workspace:

```bash
cd ~
mkdir -p sawyer_rs
cd sawyer_rs
```

Create:

```bash
nano pixi.toml
```

Use the following configuration:

```toml
[workspace]
name = "sawyer_bridge"
channels = ["https://prefix.dev/conda-forge"]
platforms = ["linux-64"]

[dependencies]
python = "3.12.*"

[environments]
noetic = { features = ["noetic"] }
humble = { features = ["humble"] }

[feature.noetic]
channels = ["https://prefix.dev/robostack-noetic"]

[feature.noetic.dependencies]
ros-noetic-desktop = "*"

[feature.humble]
channels = ["https://prefix.dev/robostack-humble"]

[feature.humble.dependencies]
ros-humble-desktop = "*"
colcon-common-extensions = "*"
cmake = "*"
ninja = "*"
pkg-config = "*"
git = "*"
rospkg = "*"
catkin_pkg = "*"
cxx-compiler = "*"
libstdcxx-ng = "*"
```

Install the environments:

```bash
pixi install
```

---

# 6. Verify ROS 1 Noetic

Enter Noetic:

```bash
cd ~/sawyer_rs
pixi shell -e noetic
```

Check:

```bash
python --version
```

Expected:

```text
Python 3.12.x
```

Then:

```bash
rosversion -d
```

Expected:

```text
noetic
```

Verify ROS 1 tools:

```bash
rostopic --help
```

Check where the package is loaded from:

```bash
python -c "import rosgraph_msgs; print(rosgraph_msgs.__file__)"
```

It should point inside:

```text
~/sawyer_rs/.pixi/envs/noetic/
```

Verify `genmsg`:

```bash
python -c "import genmsg; print(genmsg.__file__)"
```

Exit:

```bash
exit
```

---

# 7. Verify ROS 2 Humble

Enter Humble:

```bash
cd ~/sawyer_rs
pixi shell -e humble
```

Check:

```bash
python --version
```

Expected:

```text
Python 3.12.x
```

Verify Humble:

```bash
echo $ROS_DISTRO
```

Expected:

```text
humble
```

Check ROS 2:

```bash
ros2 topic list
```

Verify `rclpy`:

```bash
python -c "import rclpy; print(rclpy.__file__)"
```

It should point inside:

```text
~/sawyer_rs/.pixi/envs/humble/lib/python3.12/site-packages/
```

Verify the compiled Python extension:

```bash
find "$CONDA_PREFIX" -name "_rclpy_pybind11*.so"
```

Expected to contain something like:

```text
_rclpy_pybind11.cpython-312-x86_64-linux-gnu.so
```

Exit:

```bash
exit
```

---

# 8. Confirm That a Prebuilt ros1_bridge Package Is Not Available

Search RoboStack Humble:

```bash
cd ~/sawyer_rs

pixi search ros-humble-ros1-bridge \
  -c https://prefix.dev/robostack-humble
```

Also try:

```bash
pixi search ros1-bridge \
  -c https://prefix.dev/robostack-humble
```

If no package is found, build `ros1_bridge` from source.

---

# 9. Clone ros1_bridge

Create a build workspace:

```bash
cd ~/sawyer_rs
mkdir -p bridge_ws/src
cd bridge_ws/src
```

Clone the official repository:

```bash
git clone https://github.com/ros2/ros1_bridge.git
```

Check:

```bash
cd ros1_bridge
git branch --show-current
```

The repository currently uses:

```text
master
```

Return to the workspace:

```bash
cd ~/sawyer_rs/bridge_ws
```

---

# 10. Prepare the Mixed Build Environment

Start from a clean terminal.

Enter Humble:

```bash
cd ~/sawyer_rs
pixi shell -e humble
```

Define the ROS 1 prefix:

```bash
export ROS1_PREFIX="$HOME/saw_rs_t/.pixi/envs/noetic"
```

Expose ROS 1 executables:

```bash
export PATH="$ROS1_PREFIX/bin:$PATH"
```

Expose ROS 1 C/C++ packages through `pkg-config`:

```bash
export PKG_CONFIG_PATH="$ROS1_PREFIX/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
```

Expose ROS 1 package manifests:

```bash
export ROS_PACKAGE_PATH="$ROS1_PREFIX/share"
```

Expose ROS 1 Python packages:

```bash
export PYTHONPATH="$ROS1_PREFIX/lib/python3.12/site-packages:$PYTHONPATH"
```

Use the RoboStack/Conda runtime libraries:

```bash
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$ROS1_PREFIX/lib:${LD_LIBRARY_PATH:-}"
```

Use the RoboStack/Conda compiler:

```bash
export CC="$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-cc"
export CXX="$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-c++"
```

---

# 11. Verify the Mixed Environment

Check ROS 1 C++ discovery:

```bash
pkg-config --modversion roscpp
```

Check ROS 1 package discovery:

```bash
rospack find sensor_msgs
rospack find std_msgs
rospack find geometry_msgs
```

Expected paths should point under:

```text
~/sawyer_rs/.pixi/envs/noetic/share/
```

Check the ROS 1 message definition:

```bash
rosmsg show sensor_msgs/JointState
```

Check ROS 1 Python support:

```bash
python -c "import genmsg; print(genmsg.__file__)"
```

Check ROS 2 package discovery:

```bash
ros2 pkg prefix rclcpp
ros2 pkg prefix sensor_msgs
```

Check the ROS 2 message definition:

```bash
ros2 interface show sensor_msgs/msg/JointState
```

Verify `rclpy` still works:

```bash
python -c "import rclpy; print(rclpy.__file__)"
```

All of these commands should succeed before building the bridge.

---

# 12. Build ros1_bridge

Go to:

```bash
cd ~/sawyer_rs/bridge_ws
```

Clean any previous attempt:

```bash
rm -rf build install log
```

Build:

```bash
colcon build \
  --symlink-install \
  --packages-select ros1_bridge \
  --cmake-force-configure \
  --cmake-args \
    -DBUILD_TESTING=OFF \
    -DCMAKE_C_COMPILER="$CC" \
    -DCMAKE_CXX_COMPILER="$CXX"
```

A successful build should end with something similar to:

```text
Finished <<< ros1_bridge

Summary: 1 package finished
```

CMake deprecation warnings can normally be ignored if the package finishes successfully.

---

# 13. Load the Bridge

Source the new bridge overlay:

```bash
source ~/sawyer_rs/bridge_ws/install/local_setup.bash
```

Verify:

```bash
ros2 pkg list | grep ros1_bridge
```

Expected:

```text
ros1_bridge
```

---

# 14. Verify Message Conversion Pairs

Print supported mappings:

```bash
ros2 run ros1_bridge dynamic_bridge -- --print-pairs | head -30
```

A successful bridge should show mappings for packages such as:

```text
std_msgs
sensor_msgs
geometry_msgs
diagnostic_msgs
actionlib_msgs
```

Specifically verify `JointState`:

```bash
ros2 run ros1_bridge dynamic_bridge -- --print-pairs | grep JointState
```

Expected:

```text
'sensor_msgs/msg/JointState' (ROS 2) <=> 'sensor_msgs/JointState' (ROS 1)
```

At this point, the native ROS 1 ↔ ROS 2 bridge build is working.

---

# 15. Optional Local ROS 1 → ROS 2 Test

Before connecting Sawyer, perform a localhost test.

## Terminal 1 — ROS 1 Master

```bash
cd ~/sawyer_rs
pixi shell -e noetic
roscore
```

Leave it running.

## Terminal 2 — Bridge

```bash
cd ~/sawyer_rs
pixi shell -e humble
```

Configure the mixed environment:

```bash
export ROS1_PREFIX="$HOME/sawyer_rs/.pixi/envs/noetic"

export PATH="$ROS1_PREFIX/bin:$PATH"
export PKG_CONFIG_PATH="$ROS1_PREFIX/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
export ROS_PACKAGE_PATH="$ROS1_PREFIX/share"
export PYTHONPATH="$ROS1_PREFIX/lib/python3.12/site-packages:$PYTHONPATH"
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$ROS1_PREFIX/lib:${LD_LIBRARY_PATH:-}"

source ~/sawyer_rs/bridge_ws/install/local_setup.bash

export ROS_MASTER_URI=http://localhost:11311
```

Start the bridge:

```bash
ros2 run ros1_bridge dynamic_bridge -- --bridge-all-1to2-topics
```

Leave it running.

## Terminal 3 — ROS 1 Publisher

```bash
cd ~/sawyer_rs
pixi shell -e noetic
```

Publish:

```bash
rostopic pub -r 2 /bridge_test std_msgs/String "data: 'hello from ROS1'"
```

## Terminal 4 — ROS 2 Subscriber

```bash
cd ~/sawyer_rs
pixi shell -e humble
```

Run:

```bash
ros2 topic echo /bridge_test std_msgs/msg/String
```

Expected output:

```text
data: hello from ROS1
---
data: hello from ROS1
---
```

This proves:

```text
ROS 1 publisher
       |
       v
RoboStack Noetic
       |
       v
ros1_bridge
       |
       v
RoboStack Humble
       |
       v
ROS 2 subscriber
```

without Docker and without a custom TCP gateway.

---

# 16. Common Problems and Fixes

## Problem: Noetic imports packages from `/opt/ros/humble`

Example:

```text
ImportError ...
/opt/ros/humble/...
```

Cause:

A system ROS 2 installation was sourced before entering RoboStack.

Fix:

```bash
env | grep -E 'ROS|AMENT|COLCON|PYTHONPATH'
```

Remove or comment out:

```bash
source /opt/ros/humble/setup.bash
```

from `~/.bashrc`.

Then open a new terminal.

---

## Problem: rclpy looks for a CPython 3.12 binary while Python is 3.14

Example:

```text
_rclpy_pybind11.cpython-312-x86_64-linux-gnu.so
```

while:

```bash
python --version
```

shows Python 3.14.

Fix:

Pin Python in `pixi.toml`:

```toml
[dependencies]
python = "3.12.*"
```

Then:

```bash
pixi install
```

Verify both environments:

```bash
pixi run -e noetic python --version
pixi run -e humble python --version
```

Both should report Python 3.12.x.

---

## Problem: `ModuleNotFoundError: No module named 'genmsg'`

Cause:

The bridge-generation script cannot see the ROS 1 Python packages.

Fix:

```bash
export ROS1_PREFIX="$HOME/sawyer_rs/.pixi/envs/noetic"

export PYTHONPATH="$ROS1_PREFIX/lib/python3.12/site-packages:$PYTHONPATH"
```

Verify:

```bash
python -c "import genmsg; print(genmsg.__file__)"
```

---

## Problem: GLIBCXX or CXXABI linker errors

Examples:

```text
GLIBCXX_3.4.31
CXXABI_1.3.15
```

Cause:

System GCC/libstdc++ is being mixed with newer RoboStack/Conda libraries.

Fix:

Install/use the Conda compiler:

```toml
cxx-compiler = "*"
libstdcxx-ng = "*"
```

Then:

```bash
export CC="$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-cc"
export CXX="$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-c++"
```

Keep Conda runtime libraries first:

```bash
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$ROS1_PREFIX/lib:${LD_LIBRARY_PATH:-}"
```

---

## Problem: Bridge builds but shows no supported conversion pairs

Example:

```text
No message type conversion pairs supported.
No service type conversion pairs supported.
```

Cause:

ROS 1 message packages were not discoverable while the bridge was compiled.

Check:

```bash
rospack find sensor_msgs
```

If it fails, set:

```bash
export ROS_PACKAGE_PATH="$ROS1_PREFIX/share"
```

Then verify:

```bash
rospack find sensor_msgs
rospack find std_msgs
rosmsg show sensor_msgs/JointState
```

Delete the previous bridge build completely:

```bash
cd ~/sawyer_rs/bridge_ws
rm -rf build install log
```

Then rebuild.

---

## Problem: AMENT_PREFIX_PATH / CMAKE_PREFIX_PATH warnings after deleting the build

Example:

```text
The path '.../bridge_ws/install/ros1_bridge' ... doesn't exist
```

Cause:

The current shell previously sourced an older bridge overlay that has since been deleted.

This is usually harmless during a rebuild.

For a completely clean rebuild, open a fresh terminal and recreate the mixed environment before running `colcon build`.

---

# 17. Setup Completion Checklist

Before attempting to connect to Sawyer, verify all of the following:

```text
[✓] Pixi installed
[✓] RoboStack Noetic installed
[✓] RoboStack Humble installed
[✓] Both environments use Python 3.12
[✓] rostopic works
[✓] ros2 works
[✓] genmsg is discoverable
[✓] rclpy imports successfully
[✓] ROS 1 sensor_msgs is discoverable
[✓] ROS 2 sensor_msgs is discoverable
[✓] Conda C/C++ compiler is used
[✓] ros1_bridge builds successfully
[✓] ros1_bridge package is visible
[✓] Standard message conversion pairs exist
[✓] sensor_msgs/JointState mapping exists
[ ] Local ROS 1 → ROS 2 test completed
[ ] Sawyer networking configured
[ ] Sawyer ROS 1 topics verified
[ ] Sawyer /robot/joint_states bridged to ROS 2
```

---

# 18. Current Milestone

At the end of this document, the expected verified state is:

```text
RoboStack Noetic                   WORKING
RoboStack Humble                   WORKING
Python 3.12 compatibility          WORKING
Native ros1_bridge build           WORKING
Standard message mappings          WORKING
JointState ROS1 ↔ ROS2 mapping     WORKING
Docker                             NOT REQUIRED
Custom TCP gateway                 NOT REQUIRED FOR THIS TEST
Physical Sawyer connection         NOT YET TESTED
```

The next step is to connect the laptop to Sawyer's Ethernet network, point the ROS 1 side of the bridge to Sawyer's ROS master, verify `/robot/joint_states` in ROS 1, and then verify the same topic from ROS 2.