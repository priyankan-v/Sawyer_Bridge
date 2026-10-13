# Building the Sawyer Conda package

Target: **Ubuntu 22.04 x86_64**, external ROS 2 Humble at `/opt/ros/humble`,
Ubuntu's GCC/Boost 1.74 and `/usr/bin/python3` 3.10. This package intentionally
uses the system ROS ABI. It is not a RoboStack package, a generic Linux binary,
or a replacement ROS installation. No Docker or robot firmware changes are used.

## Builder prerequisites

Install Humble and its interface generators before building. The required Ubuntu
build dependencies are listed below; install them only if missing:

```bash
sudo apt install build-essential cmake pkg-config libboost-system-dev \
  libboost-thread-dev libboost-filesystem-dev libboost-regex-dev \
  libboost-chrono-dev libboost-date-time-dev libboost-program-options-dev \
  libconsole-bridge-dev libtinyxml2-dev python3-dev python3-setuptools \
  python3-catkin-pkg python3-rospkg python3-empy python3-yaml python3-numpy \
  python3-tk python3-colcon-common-extensions
```

Humble supplies `ament_cmake`, `rclcpp`, `rclpy`, launch, `rosidl_default_generators`,
`std_msgs`, `sensor_msgs`, `geometry_msgs` and `action_msgs`. No system Noetic is
installed. The builder downloads only the pinned ROS 1 source repositories in
`packaging/ros1-sources.json`, verifies SHA256 and selects individual packages.
No ROS 1 desktop, navigation, simulator, SDK application or ROS distribution
metapackage is built. Message generation is limited to C++ and Python.

## Native build

From the repository root on `conda-pkg`:

```bash
/usr/bin/python3 packaging/build_package.py --prefix .runtime/pkg-prefix --jobs 2
```

The builder clears inherited Conda/ROS paths, uses system compiler/Python, builds
private ROS 1 clients, builds both Intera interface packages and the application
against Humble, then rebuilds the existing bridge. Its package allowlist is
`builtin_interfaces`, `std_msgs`, `geometry_msgs`, `sensor_msgs`,
`intera_core_msgs`, and `intera_motion_msgs`. Interface definitions and joint
safety logic are preserved. Build/cache outputs stay under `.runtime/`.

## Create the Conda artifact

Package-building tools can live in an isolated **maintainer tools** prefix;
this is not a ROS environment and is not needed by users:

```bash
conda create -p "$PWD/.runtime/conda-build-tools" --override-channels \
  -c conda-forge conda-build conda-index -y
/usr/bin/python3 packaging/relocate.py .runtime/pkg-prefix \
  --patchelf .runtime/conda-build-tools/bin/patchelf
.runtime/conda-build-tools/bin/conda-build packaging/recipe --override-channels \
  -c conda-forge --output-folder "$PWD/dist" --croot "$PWD/.runtime/conda-bld"
```

The recipe packages the native staging prefix. It does not rebuild against
Conda's Python/Boost, install a ROS metapackage or install another Python runtime.
Native compilation and Conda archive creation are separate reproducible steps.
ELF library paths use `$ORIGIN` plus `/opt/ros/humble/lib`. Installed applications
use `/usr/bin/python3`; the ROS 1 Python modules remain private to helper
subprocesses. Only supported system libraries are external dependencies.

The staged tree is a package-build artifact. After source changes, rebuild it
before creating another Conda archive. Keep version/build numbers distinct for
published releases. Do not upload unvalidated artifacts.

## Install and validate at another prefix

```bash
conda create -p "$PWD/.runtime/package-install-test" --offline \
  "$PWD/dist/linux-64/sawyer-ros2-bridge-0.2.0-humble_py310_1.conda" -y
source /opt/ros/humble/setup.bash
source "$PWD/.runtime/package-install-test/local_setup.bash"
export SAWYER_STATE_DIR="$PWD/.runtime/package-test-state"
ros2 run sawyer_ros2_control sawyer_runtime --check
ros2 run sawyer_ros2_control verify_bridge
ros2 run sawyer_ros2_control move_joint --help
```

`verify_bridge` owns a localhost master, private DDS domain and synthetic fixture.
It checks ROS 1→ROS 2 JointState/Image and ROS 2→ROS 1 JointCommand/HeadPanCommand
fields, then cleans up its processes. It never publishes to physical robot topics.
The bundled pure-Python master is only used by this explicit self-test.

Developer regression tests reuse the staged or installed prefix:

```bash
export SAWYER_PACKAGE_PREFIX="$PWD/.runtime/package-install-test"
./scripts/env.sh ros2 python -m unittest discover -s tests -v
./scripts/env.sh ros1 python tests/camera_control_checks.py
/usr/bin/python3 tests/run_local.py
/usr/bin/python3 tests/run_camera_local.py
```

The source-only ROS 1 master/CLI helpers used by those legacy integration runners
are restricted to localhost and come from the pinned build cache. They are not
user installation requirements. GUI tests require a desktop display.

## External runtime dependencies

Conda does not manage the existing system Humble installation. Runtime needs
Ubuntu's `libstdc++6`, `libgcc-s1`, libc, Boost 1.74 libraries, `libconsole-bridge1.0`,
`libtinyxml2-9`, `libpython3.10`, and system Python dependencies listed in README.
Humble provides its own middleware, message and typesupport libraries. A release
must be tested on a clean Ubuntu 22.04 machine with these prerequisites before
being described as generally distributable. Other OS/Python/ROS ABIs require a
separate build; the installed runtime check rejects unsupported platforms.

## Channel publication

```bash
.runtime/conda-build-tools/bin/python -m conda_index dist
# Users can then install from a locally served channel:
conda install -c file:///absolute/path/to/dist sawyer-ros2-bridge
```

No public channel is configured or uploaded automatically. Configure channel
ownership and release credentials separately when publication is authorized.
See [MIGRATION.md](MIGRATION.md) for the actual validation results and remaining
hardware checks.
