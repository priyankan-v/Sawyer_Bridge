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

Humble must provide `ament_cmake`, `rclcpp`, `rclpy`, launch,
`rosidl_default_generators`, `std_msgs`, `sensor_msgs`, `geometry_msgs` and
`action_msgs`. The source manifest pins upstream ROS 1 archives by SHA256.
ROS 1 message generation, Python tools and the localhost test master stay in
the ignored build directory. The artifact contains only eight ROS 1 C++ libraries.

## Build the artifact

From the repository root:

```bash
/usr/bin/python3 packaging/build_package.py --prefix .runtime/clean-prefix --jobs 3
conda create -p "$PWD/.runtime/conda-build-tools" --override-channels \
  -c conda-forge conda-build conda-index patchelf -y
/usr/bin/python3 packaging/relocate.py .runtime/clean-prefix \
  --patchelf .runtime/conda-build-tools/bin/patchelf
.runtime/conda-build-tools/bin/conda-build packaging/recipe --override-channels \
  -c conda-forge --output-folder "$PWD/dist" --croot "$PWD/.runtime/conda-bld"
.runtime/conda-build-tools/bin/python -m conda_index dist
```

Use fresh staging/build directories when changing the source layout. Increment
the recipe's version/build for a new artifact; Conda caches packages by filename.
The recipe points to `.runtime/clean-prefix`. Build tools are separate from the
installed payload and are not user runtime dependencies.

## Install at a separate prefix

```bash
conda create -p "$PWD/.runtime/clean-install" --offline \
  "$PWD/dist/linux-64/sawyer-ros2-bridge-0.3.0-humble_py310_0.conda" -y
source /opt/ros/humble/setup.bash
source "$PWD/.runtime/clean-install/local_setup.bash"
ros2 run sawyer_bridge check
ros2 launch sawyer_bridge bridge.launch.py --show-args
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
  --report "$PWD/.runtime/clean-transport.json"
/usr/bin/python3 -m unittest discover -s tests -p test_configuration.py -v
```

The test runs the installed launch file with synthetic configuration, owns a localhost master and isolated DDS domain. It verifies JointState
and synthetic Image data from ROS 1, and JointCommand/HeadPanCommand data from
ROS 2. It publishes only to `/sawyer_pkg_test/*`, never robot command topics.
Tests and ROS 1 Python fixtures are not installed in the artifact.

Native serialization tests also compare empty arrays, boolean arrays, headers
and converted bytes against generated ROS 1 C++ interfaces:

```bash
export SAWYER_ROOT="$PWD"
export SAWYER_PACKAGE_PREFIX="$PWD/.runtime/clean-install"
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

Validated on this machine on **2026-10-13**:

| Check | Result |
|---|---|
| Native build and Conda recipe tests | PASS |
| Fresh-prefix installation | PASS; one Conda package, no extra ROS/Python environment |
| Imports, library linkage and required mappings | PASS; all 35 ELF files resolve without build paths; mappings unchanged (49 Intera entries) |
| Installed launch and localhost transport | PASS; JointState, Image pixels, exact JointCommand/HeadPanCommand fields |
| Native serialization | PASS; headers, empty arrays and boolean arrays |
| XML-RPC configuration | PASS; two tests for defaults, opt-in and master rejection |
| Payload audit | PASS; eight private C++ ROS 1 libraries, no applications/ROS 1 Python/test master |
| Sawyer read-only telemetry | PASS; 710 messages at 119.95 Hz; all seven arm positions/velocities, limits and robot state |

Artifact: `dist/linux-64/sawyer-ros2-bridge-0.3.0-humble_py310_0.conda`
(8,370,220 bytes). SHA256:

```text
1f4b0693e3f6d4e094c5cd194d5f7aa9ce2ccce5eff32712fbb48bd5e9bf5546
```

Detailed logs/results are local build outputs under `.runtime/clean-*`.
Conda's linkage audit warns about external system/Humble libraries, which its
solver does not manage; actual installed linkage/import checks pass.
No physical actuation was performed. The `conda` and `main` branch references
are unchanged. Existing local environments, network settings and generated
workspace outputs are preserved; removed legacy sources/results were archived
locally under `.runtime/legacy-*` and are excluded from the package.
A public channel upload and validation on a separate clean PC remain release
steps. Local channel installation uses:

```bash
conda install -c "file://$PWD/dist" sawyer-ros2-bridge
```
