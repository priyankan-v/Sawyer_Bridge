# Build and validate the Conda package

Users install a binary; the following commands are for maintainers on
**Ubuntu 22.04 x86_64 with system Humble at `/opt/ros/humble`**.

## Build dependencies

Install missing builder prerequisites using Ubuntu packages:

```bash
sudo apt install build-essential cmake pkg-config libboost-system-dev \
  libboost-thread-dev libboost-filesystem-dev libboost-regex-dev \
  libboost-chrono-dev libboost-date-time-dev libboost-program-options-dev \
  libconsole-bridge-dev libtinyxml2-dev python3-dev python3-setuptools \
  python3-catkin-pkg python3-rospkg python3-empy python3-yaml python3-numpy \
  python3-colcon-common-extensions
```

Humble must provide `ament_cmake`, `ament_cmake_python`, `rclcpp`, `rclcpp_action`, `rclpy`, launch,
`rosidl_default_generators`, `std_msgs`, `sensor_msgs`, `geometry_msgs` and
`action_msgs`. The source manifest pins upstream ROS 1 archives by SHA256.
ROS 1 message generation, Python tools and the localhost test master stay in
the ignored build directory. The artifact contains only eight ROS 1 C++ libraries.

## Build the artifact

From the repository root:

```bash
/usr/bin/python3 packaging/build_package.py --prefix .runtime/full-prefix --jobs 3
conda create -p "$PWD/.runtime/conda-build-tools" --override-channels \
  -c conda-forge conda-build conda-index patchelf -y
/usr/bin/python3 packaging/relocate.py .runtime/full-prefix \
  --patchelf .runtime/conda-build-tools/bin/patchelf
.runtime/conda-build-tools/bin/conda-build packaging/recipe --override-channels \
  -c conda-forge --output-folder "$PWD/dist" --croot "$PWD/.runtime/conda-bld"
.runtime/conda-build-tools/bin/python -m conda_index dist
```

The builder forces CMake reconfiguration and explicitly selects the staged ROS 2
Intera packages, preventing stale install-prefix/ROS 1 interface caches. Use fresh
staging directories when changing the package layout. Increment
the recipe's version/build for a new artifact; Conda caches packages by filename.
The recipe points to `.runtime/full-prefix`. Build tools are separate from the
installed payload and are not user runtime dependencies.

## Install at a separate prefix

```bash
conda create -p "$PWD/.runtime/full-install" --offline \
  "$PWD/dist/linux-64/sawyer_bridge-0.5.0-humble_py310_0.conda" -y
source /opt/ros/humble/setup.bash
source "$PWD/.runtime/full-install/local_setup.bash"
ros2 run sawyer_bridge check
ros2 launch sawyer_bridge bridge.launch.py --show-args
/usr/bin/python3 tests/package_layout.py "$PWD/.runtime/full-install"
```

ELF paths are relative to the installed prefix, plus `/opt/ros/humble/lib`.
Conda installs neither Python nor a ROS distribution. External dependencies are
Ubuntu libc/libstdc++/libgcc, Boost 1.74, console_bridge, system Python 3.10/PyYAML
and the existing Humble libraries/Python dependencies (including NumPy for
generated message bindings). Conda's dependency solver cannot validate these
system prerequisites; the installed `check` command verifies imports and linkage.
This binary is not compatible with other OS, Python or ROS ABIs.

## Maintainer tests

With the installed overlay sourced:

```bash
/usr/bin/python3 tests/transport_test.py \
  --ros1-prefix "$PWD/.runtime/clean-build/ros1-install" \
  --report "$PWD/.runtime/full-transport.json"
/usr/bin/python3 -m unittest discover -s tests -p 'test_*.py' -v
/usr/bin/python3 tests/prepare_actionlib.py
/usr/bin/python3 tests/action_transport.py \
  --ros1-prefix "$PWD/.runtime/clean-build/ros1-install" \
  --report "$PWD/.runtime/full-actions.json"
```

The test runs the installed launch file with synthetic configuration, owns a localhost master and isolated DDS domain. It verifies JointState
and synthetic Image data from ROS 1, and JointCommand/HeadPanCommand data from
ROS 2. It publishes only to `/sawyer_pkg_test/*`, never robot command topics.
Tests and ROS 1 Python fixtures are not installed in the artifact.

Native serialization tests also compare empty arrays, boolean arrays, headers
and converted bytes against generated ROS 1 C++ interfaces:

```bash
export SAWYER_ROOT="$PWD"
export SAWYER_PACKAGE_PREFIX="$PWD/.runtime/full-install"
export SAWYER_ROS1_BUILD_PREFIX="$PWD/.runtime/clean-build/ros1-install"
export PKG_CONFIG_PATH="$SAWYER_ROS1_BUILD_PREFIX/lib/pkgconfig"
cmake -S tests/native -B .runtime/clean-native
cmake --build .runtime/clean-native
/usr/bin/python3 tests/native/link_test.py
export LD_LIBRARY_PATH="$SAWYER_PACKAGE_PREFIX/libexec/sawyer/ros1/lib:$LD_LIBRARY_PATH"
.runtime/clean-native/check_conversion
```

Optional **read-only** hardware validation, with the network variables from README:

```bash
/usr/bin/python3 tests/hardware_readonly.py
```

It opens only telemetry paths, uses DDS domain 198, and removes its temporary
ROS 1 parameter namespace. No physical motion or camera control is performed.

## Release status

The 0.5.0 release adds startup Intera graph discovery, bidirectional adapters
for `CalibrationCommand` and `MotionCommand`, and `CameraStrobe` (robot header
MD5 `7886e4ecc3bbbe7c1e9a3321acbb3a39`). It remains one bridge package/process;
ROS 1 Python and native actionlib are fixture-only dependencies, excluded from
the artifact. `packaging/test-sources.json` pins the official ROS 1 actionlib
archive used for tests by SHA256.

The public SDK and repository do not supply the robot's
`CameraFieldCalibrationCommand` goal definition. Discovery reports and skips this unsupported action; its complete matching
`.action` definition is required to add support.
Hardware action goals, calibration and service invocations are not run automatically.

Validated on **2026-10-13**:

| Check | Result |
|---|---|
| Native build, Conda recipe and fresh installation | PASS |
| Interface/type support | PASS; 47 messages, 3 services, 2 action types |
| Payload and linkage | PASS; one bridge package, 35 ELF files, eight private ROS 1 libraries |
| Message transport | PASS; JointState, JointCommand, HeadPanCommand, Image and CameraStrobe timestamp/int16 |
| Native actionlib/rclpy interoperability | PASS; both actions and all services in both directions |
| Action lifecycle | PASS; success, abort, rejection, feedback, cancellation, concurrent cancel-all and server loss |
| Configuration/discovery tests | PASS; four tests, including non-invoking service probes |
| Native serialization regression | PASS; headers, empty arrays, boolean arrays and exact ROS 1 bytes |
| Live graph discovery | 125 topic entries, 16 Intera service endpoints, 2 supported action servers; known camera-field action reported |
| Live read-only telemetry | PASS; 706 messages at 120.15 Hz, all seven arm positions/velocities, limits and robot state |

Artifact: `dist/linux-64/sawyer_bridge-0.5.0-humble_py310_0.conda`
(8,518,272 bytes). SHA256:

```text
5be8d8d96abfaeefc1e7f7c87a4b3b2a9895d1137bcce86deda9a73443f95b8b
```

Detailed local results: `.runtime/full-*`. No hardware action goals or service
invocations were performed; only read-only telemetry and header probes ran.
The unavailable camera-field action definition prevents complete coverage of
that firmware-specific interface. Conda's linkage audit still warns about
external system/Humble libraries; all installed ELF linkage checks pass.
Original Git branch references and existing local environments are preserved.

A public channel upload and validation on a separate clean PC remain release
steps. Local channel installation uses:

```bash
conda install -c "file://$PWD/dist" sawyer_bridge
```
