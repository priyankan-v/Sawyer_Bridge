# Sawyer bridge for ROS 2 Humble

`sawyer_bridge` connects your existing ROS 2 Humble installation to Sawyer's
onboard ROS 1 graph. It provides the C++ bridge, Intera message interfaces and a
startup discovery of Intera topics, services and actions. Write your own ROS 2 applications using these interfaces.

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

Install the supplied artifact into your current Conda prefix. If the previous
`sawyer-ros2-bridge` package is installed, remove it first with
`conda remove sawyer-ros2-bridge`:

```bash
conda install --offline /path/to/sawyer_bridge-0.5.0-humble_py310_0.conda
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

### What starts

1. A short configuration process reads `sawyer_bridge.yaml`, writes its topic and
   service/action lists to Sawyer's ROS 1 parameter server, then exits. By default,
   it discovers advertised Intera endpoints in both ROS graphs (allow about five
   seconds for DDS discovery). Service probes exchange headers without calling services.
2. The C++ `parameter_bridge` process starts one ROS 1 client node and one ROS 2
   client node, both named `sawyer_bridge` in their respective graphs.
3. It reads `/sawyer_bridge/topics`, `/sawyer_bridge/services_1_to_2` and
   `/sawyer_bridge/services_2_to_1` and `/sawyer_bridge/actions`. Each enabled topic creates a subscriber on
   the source side and a publisher on the destination side. Compiled converters
   translate incoming messages. Configured services create forwarding endpoints;
   discovered Intera service servers are exposed in the other ROS graph.
4. Action adapters forward goals, feedback, results and cancellation between
   ROS 1 and ROS 2. The ROS 1 callback thread and ROS 2 executor process them until Ctrl+C.

There is one ROS package for launching, configuring and running the transport:
**`sawyer_bridge`**. The launch command runs in the foreground. Sawyer already
runs the ROS 1 master. No master, motion program, camera viewer or robot-enabling
process is started on your PC.

The executable alone reads parameters already stored on the ROS 1 master; it
cannot load YAML by itself. Use the launch command above to load the list and
start transport together. Changes to the list take effect when the bridge restarts.

## Read joint states — terminal 2

```bash
source /opt/ros/humble/setup.bash
source "$CONDA_PREFIX/local_setup.bash"
export ROS_DOMAIN_ID=42
ros2 topic echo /robot/joint_states
# Alternatively, measure the receive rate:
ros2 topic hz /robot/joint_states
```

The default mode discovers all supported advertised Intera endpoints, including
command topics. Enabling transport publishes no commands or goals by itself.
For the previous telemetry-only workflow, use:

```bash
ros2 launch sawyer_bridge bridge.launch.py all_interfaces:=false
```

Discovery is a startup snapshot: start your ROS 2 servers first, and restart the
bridge after adding endpoints. It does not invent topic names for unused types.

## Connect your application

In telemetry mode, enable the arm command path explicitly with:

```bash
ros2 launch sawyer_bridge bridge.launch.py all_interfaces:=false commands:=true
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

With `all_interfaces:=false`, the YAML lists alone select the endpoints.
Service types use `intera_core_msgs/ServiceName` for `services_2_to_1` (a ROS 1
server), and `intera_core_msgs/srv/ServiceName` for `services_1_to_2` (a ROS 2 server).
Action entries specify `action`, `type`, `direction` and optional `server_timeout`
(default 5 seconds for lost server/goal acknowledgement; not a trajectory deadline).
An action endpoint uses one direction; use distinct names for native servers in
both graphs. Action examples are in the installed YAML.

Coverage: **47 regular Intera messages, 3 services and 2 actions**
(`CalibrationCommand`, `MotionCommand`), with both forwarding directions.
`CameraStrobe` matches the robot's published ROS 1 definition/MD5.
The robot also advertises **`CameraFieldCalibrationCommand`**, whose goal
schema is absent from the available SDK. Discovery reports and skips it; its
complete `.action` definition is required to support it. The bridge never guesses it.

Actions need protocol adapters because ROS 1 and ROS 2 action transports differ
([ROS 2 action design](https://design.ros2.org/articles/actions.html)). They run
inside the same C++ bridge process. A ROS 1 rejection after local ROS 2 acceptance
returns an aborted ROS 2 goal with the original result. ROS 2 goal rejection has
no result payload, so the ROS 1 result contains `ROS2_GOAL_REJECTED`. Server loss
aborts outstanding goals; it does not report success.

## Stop and troubleshoot

Cancel/finish active actions and stop your application safely first, then press **Ctrl+C** in the bridge terminal.
Stopping the bridge disconnects transport; it does not disable the robot.

- **Master connection failed:** check `ROS_MASTER_URI`, network and robot availability.
- **No joint states:** check `ROS_IP` is reachable from Sawyer, and matching `ROS_DOMAIN_ID` values.
- **Command subscriber absent:** use discovery mode, or telemetry mode with `commands:=true` and confirm your application's QoS is compatible.
- **Missing library/type support:** source both setup files and run `ros2 run sawyer_bridge check`. Keep Conda ROS/Python libraries out of the system Humble library paths.

## Repository

| Path | Purpose |
|---|---|
| `src/intera_*_msgs/` | ROS 2 Intera interfaces |
| `src/sawyer_bridge/` | C++ transport, launch, configuration, conversion generator and package check |
| `interfaces/ros1/` | Matching ROS 1 Intera definitions, used during compilation |
| `packaging/` | Native builder, pinned sources, relocation and Conda recipe |
| `tests/` | Maintainer conversion/transport tests; excluded from the artifact |

Build and validation instructions: [PACKAGING.md](PACKAGING.md).
Previous implementations remain on their original branches.

The C++ transport is derived from the upstream ROS 2 `ros1_bridge` project.
Its original C++ namespace and mapping format are retained internally; no
separate `ros1_bridge` ROS package is installed.
