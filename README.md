# Sawyer bridge for ROS 2 Humble

`sawyer-ros2-bridge` connects your existing ROS 2 Humble installation to Sawyer's
onboard ROS 1 graph. It provides the C++ bridge, Intera message interfaces and a
selective topic configuration. Write your own ROS 2 applications using these interfaces.

```text
Your ROS 2 nodes ↔ packaged C++ bridge ↔ Sawyer's onboard ROS 1
```

The package contains **no joint-motion application, camera viewer, SDK, robot
models or ROS distribution**. Only the ROS 1 C++ libraries needed by the bridge
are bundled privately. No Noetic installation or separate ROS environments are needed.

## Install

Requires **Ubuntu 22.04 x86_64**, Conda and system **ROS 2 Humble** installed under
`/opt/ros/humble`. System Python 3.10, Humble’s Python dependencies, PyYAML and Ubuntu Boost 1.74/console_bridge
runtime libraries must be available; see [PACKAGING.md](PACKAGING.md) for details.

Install the supplied artifact into your current Conda prefix:

```bash
conda install --offline /path/to/sawyer-ros2-bridge-0.3.0-humble_py310_0.conda
source /opt/ros/humble/setup.bash
source "$CONDA_PREFIX/local_setup.bash"
ros2 run sawyer_bridge check
```

No build is needed on the user's computer. The package has not been published to
a public channel. Source both setup files in each terminal that uses this package.
Python entry points use `/usr/bin/python3` to match system Humble.

## Connect and start — terminal 1

Connect the PC to Sawyer's network. Set the master URI and **your PC's reachable
IP address**. These values match the project's existing connection; change them
only if your network differs:

```bash
export ROS_MASTER_URI=http://169.254.121.3:11311
export ROS_IP=169.254.121.2
unset ROS_HOSTNAME
export ROS_DOMAIN_ID=42
ros2 launch sawyer_bridge bridge.launch.py
```

The launch command loads topic parameters through XML-RPC and starts the C++
`parameter_bridge`. It runs in the foreground. Sawyer supplies the ROS 1 master;
this package does not start one.

## Read joint states — terminal 2

```bash
source /opt/ros/humble/setup.bash
source "$CONDA_PREFIX/local_setup.bash"
export ROS_DOMAIN_ID=42
ros2 topic echo /robot/joint_states
# Alternatively, measure the receive rate:
ros2 topic hz /robot/joint_states
```

The default configuration forwards joint states, robot state, joint limits,
endpoint state, collision state and head state. Command topics are disabled.

## Connect your application

To forward commands from your own ROS 2 application, restart the bridge with:

```bash
ros2 launch sawyer_bridge bridge.launch.py commands:=true
# Add head_commands:=true if your application needs head-pan commands.
```

This enables transport only; the bridge sends no movement commands by itself.
Your application must enforce readiness, limits, command freshness and safe stopping.
Inspect the interface and subscription without publishing:

```bash
ros2 interface show intera_core_msgs/msg/JointCommand
ros2 topic info /robot/limb/right/joint_command --verbose
```

For other topics/services, copy the installed configuration, edit it and pass it
as `config:=/absolute/path/to/bridge.yaml`:

```bash
cp "$CONDA_PREFIX/share/sawyer_bridge/config/sawyer_bridge.yaml" ./bridge.yaml
ros2 launch sawyer_bridge bridge.launch.py config:="$PWD/bridge.yaml"
```

Services are disabled by default. Built-in core and Intera mappings are compiled
into the bridge; adding a YAML entry cannot add an unsupported message mapping.
ROS actions are not forwarded by ros1_bridge.

## Stop and troubleshoot

Stop your application safely first, then press **Ctrl+C** in the bridge terminal.
Stopping the bridge disconnects transport; it does not disable the robot.

- **Master connection failed:** check `ROS_MASTER_URI`, network and robot availability.
- **No joint states:** check `ROS_IP` is reachable from Sawyer, and matching `ROS_DOMAIN_ID` values.
- **Command subscriber absent:** start with `commands:=true` and confirm your application's QoS is compatible.
- **Missing library/type support:** source both setup files and run `ros2 run sawyer_bridge check`. Keep Conda ROS/Python libraries out of the system Humble library paths.

## Repository

| Path | Purpose |
|---|---|
| `src/ros1_bridge/` | C++ bridge and build-time conversion generator |
| `src/intera_*_msgs/` | ROS 2 Intera interfaces |
| `src/sawyer_bridge/` | Launch, configuration and package check |
| `interfaces/ros1/` | Matching ROS 1 Intera definitions, used during compilation |
| `packaging/` | Native builder, pinned sources, relocation and Conda recipe |
| `tests/` | Maintainer conversion/transport tests; excluded from the artifact |

Build and validation instructions: [PACKAGING.md](PACKAGING.md).
Previous implementations remain on their original branches.
