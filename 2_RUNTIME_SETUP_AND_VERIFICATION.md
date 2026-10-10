# Stage 2 — Running and Verifying the Sawyer ROS 1 → ROS 2 Bridge
 
**Platform:** Ubuntu 22.04, ROS 1 Noetic + ROS 2 Humble via Pixi  
**Prerequisite:** `ros1_bridge` has already been built successfully.

## 1. Objective

After connecting Sawyer's Ethernet cable, start the existing `ros1_bridge` and verify that live Sawyer joint states arrive in ROS 2.

```text
Sawyer controller (ROS 1 master)
           |
           | Ethernet / ROS 1 TCPROS
           v
Workstation (ROS 1 network client)
           |
           v
ros1_bridge dynamic_bridge (Pixi runtime)
           |
           | ROS 2 DDS
           v
ROS 2 Humble subscriber
           |
           v
/robot/joint_states [sensor_msgs/msg/JointState]
```

This stage is **read-only**. Do **not** enable the robot, publish joint commands, or launch motion scripts to verify connectivity.

## 2. Known lab settings — verify before use

| Setting | Previously used value |
|---|---|
| Pixi project | `~/sawyer_rs` |
| Sawyer robot address | `169.254.121.3` |
| Workstation wired address | `169.254.121.2` |
| ROS 1 master | `http://169.254.121.3:11311` |
| Sawyer hostname (previous setup) | `021611CP00085.local` |
| ROS 1 source topic | `/robot/joint_states` |
| ROS 2 output topic | `/robot/joint_states` |
| ROS 1 message | `sensor_msgs/JointState` |
| ROS 2 message | `sensor_msgs/msg/JointState` |

**Do not blindly apply these addresses:** first inspect the actual wired interface. Commands below use the values above as examples. Replace them if your lab configuration differs.

## 3. Plug in Sawyer and verify its network

1. Power Sawyer normally and allow its controller/ROS system to finish starting.
2. Connect the Sawyer Ethernet cable to the workstation's intended wired port.
3. Open **Terminal A** on the Ubuntu workstation.

Find your interface:

```bash
ip -br link
ip -br addr
```

Identify the Ethernet device, e.g. `enp3s0` (not necessarily that exact name). Inspect its existing IPv4 address before changing anything:

```bash
ip -4 addr show dev enp3s0
```

If it already has the correct wired address, **do not add it again**. If the address is missing, and the verified lab configuration uses `169.254.121.2/16`, add it temporarily:

```bash
sudo ip link set enp3s0 up
sudo ip addr add 169.254.121.2/16 dev enp3s0
```

This command is temporary and normally does not persist after reboot. Do not flush all network addresses or disconnect the workstation's other networks.

Verify the route back to Sawyer:

```bash
ip route get 169.254.121.3
```

**Expected:** the route uses your wired interface, and ideally shows `src 169.254.121.2`.

Test the connection:

```bash
ping -c 3 169.254.121.3
nc -vz -w 3 169.254.121.3 11311
```

A successful TCP connection to port **11311** confirms the ROS 1 master TCP port is reachable. Some devices do not respond to ICMP/ping, so a failed ping alone does not prove failure.

### 3.1 Check Sawyer hostname resolution (important for ROS 1)

ROS 1 uses multiple peer-to-peer connections. Even if connecting to the master by IP works, Sawyer may return a **hostname** for its topic publishers. That hostname must be resolvable and reachable from this workstation.

Verify the hostname used by the robot in this lab:

```bash
getent hosts 021611CP00085.local
```

If it does not resolve, and you have independently confirmed that the hostname belongs to Sawyer at `169.254.121.3`, add an appropriate local name mapping:

```bash
sudo nano /etc/hosts
```

Add the line if missing:

```text
169.254.121.3  021611CP00085.local
```

Recheck:

```bash
getent hosts 021611CP00085.local
```

If another hostname appears in ROS 1 errors or topic publisher details, check that name too. Do not map unrelated names to Sawyer blindly.

## 4. Terminal B — Verify ROS 1 data with Pixi Noetic

Open a **new terminal**:

```bash
cd ~/sawyer_rs
pixi shell -e noetic
```

Set the ROS 1 master and the workstation's **actual** wired IP in this terminal:

```bash
export ROS_MASTER_URI=http://169.254.121.3:11311
export ROS_IP=169.254.121.2
unset ROS_HOSTNAME
```

Verify:

```bash
printf 'ROS_MASTER_URI=%s\nROS_IP=%s\n' "$ROS_MASTER_URI" "$ROS_IP"
```

Expected:

```text
ROS_MASTER_URI=http://169.254.121.3:11311
ROS_IP=169.254.121.2
```

Query Sawyer's master:

```bash
rostopic list
```

Confirm the topic exists:

```bash
rostopic type /robot/joint_states
```

Expected:

```text
sensor_msgs/JointState
```

Read a single live message:

```bash
rostopic echo -n 1 /robot/joint_states
```

Optionally measure the ROS 1 publish rate:

```bash
rostopic hz /robot/joint_states
```

Press `Ctrl+C` to stop frequency measurement. If `rostopic list` succeeds but `rostopic echo` hangs, inspect hostname resolution and peer-to-peer connectivity in §9.

**Do not run `roscore` locally** — the ROS 1 master should be the one on Sawyer.

## 5. Terminal C — Start ros1_bridge using Pixi Humble

Open a **new terminal**:

```bash
cd ~/sawyer_rs
pixi shell -e humble
```


```bash
export ROS1_PREFIX="$HOME/sawyer_rs/.pixi/envs/noetic"

export PATH="$ROS1_PREFIX/bin:$PATH"
export PKG_CONFIG_PATH="$ROS1_PREFIX/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
export ROS_PACKAGE_PATH="$ROS1_PREFIX/share"
export PYTHONPATH="$ROS1_PREFIX/lib/python3.12/site-packages:$PYTHONPATH"
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$ROS1_PREFIX/lib:${LD_LIBRARY_PATH:-}"

source ~/sawyer_rs/bridge_ws/install/local_setup.bash
```

Your successful bridge build is assumed to be accessible from this environment. Set ROS 1 network variables **again in this terminal**:

```bash
export ROS_MASTER_URI=http://169.254.121.3:11311
export ROS_IP=169.254.121.2
unset ROS_HOSTNAME
```

**Why again?** Values exported in Terminal B do not automatically appear in Terminal C, even though both use the same Pixi project.

Verify what the bridge will use:

```bash
echo "$ROS_MASTER_URI"
echo "$ROS_IP"
ros2 pkg executables ros1_bridge
```

Expected executable: `ros1_bridge dynamic_bridge`.

Check that standard JointState messages have a bridge mapping:

```bash
ros2 run ros1_bridge dynamic_bridge --print-pairs | grep -F 'sensor_msgs/JointState'
```

Expected: a mapping pairing ROS 1 `sensor_msgs/JointState` with ROS 2 `sensor_msgs/msg/JointState`. Exact formatting can vary.

Start the **ROS 1 → ROS 2** bridge:

```bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics
```

Leave Terminal C running. This option is convenient for inspection because supported ROS 1 topics can be exposed on ROS 2 even before an application subscriber has been started. It is a **read-only bridge direction**, which is preferable for this stage.

If this binary does not recognize the option, inspect available options:

```bash
ros2 run ros1_bridge dynamic_bridge --help
```

Alternative (on builds that support the default on-demand behavior):

```bash
ros2 run ros1_bridge dynamic_bridge
```

With an on-demand bridge, a topic might not appear immediately until a ROS 2 subscriber is active. In that case, request the topic with its explicit type (see §6).

**Runtime requirement:** a successful Pixi build is not, by itself, proof that this Humble shell has every runtime library required by `ros1_bridge`. If the executable reports missing `libroscpp`, message type support, or other ROS 1 runtime libraries, fix the bridge runtime environment before trying topic commands. Do not assume the bridge can run from an arbitrary Humble-only environment.

## 6. Terminal D — Verify live Sawyer data in ROS 2

Open another **new terminal**:

```bash
cd ~/sawyer_rs
pixi shell -e humble
```

Check the ROS 2 environment and bridge discovery:

```bash
ros2 topic list
ros2 topic type /robot/joint_states
```

Expected type:

```text
sensor_msgs/msg/JointState
```

Read joint data:

```bash
ros2 topic echo /robot/joint_states sensor_msgs/msg/JointState
```

Giving the message type explicitly also helps if a dynamic bridge creates topics on demand.

Typical Sawyer joint names include:

```text
head_pan
right_j0
right_j1
right_j2
right_j3
right_j4
right_j5
right_j6
torso_t0
```

The message should contain fields including `header`, `name`, `position`, `velocity`, and `effort`. Data should continue arriving as Sawyer publishes it, even if the physical joints are not moving.

In Terminal C, look for a log indicating a **1-to-2 bridge** was created for `/robot/joint_states`.

Check message frequency:

```bash
ros2 topic hz /robot/joint_states
```

Optional diagnostics:

```bash
ros2 topic info /robot/joint_states -v
ros2 topic echo --once /robot/joint_states sensor_msgs/msg/JointState
```

Use `Ctrl+C` to stop continuous commands.

### Success criteria

The stage succeeds when:

1. Sawyer's ROS master can be reached over Ethernet.
2. The Noetic terminal receives one or more `sensor_msgs/JointState` messages from Sawyer.
3. The bridge terminal stays running without master connection errors.
4. The Humble terminal receives `sensor_msgs/msg/JointState` messages at `/robot/joint_states`.
5. `ros2 topic hz` reports a continuing publish rate.

You have then verified **ROS 1 → ROS 2 state communication**, not yet ROS 2 motion control, command safety, or bridge latency.

## 7. Short daily startup sequence

**Assumes:** Ethernet interface/IP and Sawyer hostname have already been configured correctly. Replace the IPs if needed.

### Terminal A — network

```bash
ip -br addr
ip route get 169.254.121.3
nc -vz -w 3 169.254.121.3 11311
```

### Terminal B — optional ROS 1 confirmation

```bash
cd ~/sawyer_rs
pixi shell -e noetic
export ROS_MASTER_URI=http://169.254.121.3:11311
export ROS_IP=169.254.121.2
unset ROS_HOSTNAME
rostopic echo -n 1 /robot/joint_states
```

### Terminal C — bridge (leave running)

```bash
cd ~/sawyer_rs
pixi shell -e humble
export ROS_MASTER_URI=http://169.254.121.3:11311
export ROS_IP=169.254.121.2
unset ROS_HOSTNAME
ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics
```

### Terminal D — ROS 2 verification

```bash
cd ~/sawyer_rs
pixi shell -e humble
ros2 topic echo /robot/joint_states sensor_msgs/msg/JointState
```

When confirmed, use another Humble terminal for:

```bash
ros2 topic hz /robot/joint_states
```

## 8. Stop the setup safely

1. Stop `ros2 topic echo` / `ros2 topic hz` using `Ctrl+C`.
2. Stop the bridge in Terminal C using `Ctrl+C`.
3. Exit the Pixi shells if finished.
4. Do not remove existing persistent network configuration just to end a bridge test.

No robot command or enable/disable operation is required for this read-only test.

## 9. Troubleshooting

### 9.1 `Failed to contact master at [localhost:11311]`

**Cause:** The ROS 1 master address in the **bridge process** is unset or defaults to localhost.

In the **bridge terminal**, stop the process and run:

```bash
export ROS_MASTER_URI=http://169.254.121.3:11311
export ROS_IP=169.254.121.2
unset ROS_HOSTNAME
echo "$ROS_MASTER_URI"
ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics
```

Setting `ROS_MASTER_URI` in a different terminal does **not** fix the bridge's existing process. Also check whether a shell startup script overrides it.

### 9.2 `Connection refused` or timeout to `169.254.121.3:11311`

Check:

```bash
ip -br addr
ip route get 169.254.121.3
nc -vz -w 3 169.254.121.3 11311
```

Possible causes: wrong Ethernet port/interface, wrong IP, cable/link problem, Sawyer not fully booted, or ROS master not running at the configured address.

### 9.3 `rostopic list` works, but `rostopic echo` receives nothing

ROS 1 master registration is **not** the same as reaching a topic publisher. ROS 1 topic transfer uses connections between individual peers.

Check:

```bash
rostopic info /robot/joint_states
getent hosts 021611CP00085.local
ip route get 169.254.121.3
```

Verify any advertised Sawyer hostname resolves to the right wired address. Verify `ROS_IP` points to the workstation's wired IP and the robot can reach that IP. Inspect firewall / routing only if needed.

### 9.4 ROS 1 receives data but ROS 2 topic is missing

Check bridge mapping:

```bash
ros2 run ros1_bridge dynamic_bridge --print-pairs | grep -F 'sensor_msgs/JointState'
```

Check that Terminal C is still running and has the correct `ROS_MASTER_URI`. Use the explicit ROS 2 type:

```bash
ros2 topic echo /robot/joint_states sensor_msgs/msg/JointState
```

Check bridge logs for a `1to2` creation line. If a mapping is missing, the bridge build may not include the relevant message type.

### 9.5 `ros2` commands fail with a NumPy C-extension / Python ABI error

Example symptom: `cpython-312` NumPy extension in a Python 3.10 process.

This is a **Python/Pixi dependency mismatch**, not evidence that Sawyer Ethernet is failing. Check:

```bash
which python
python --version
python -c 'import numpy; print(numpy.__version__, numpy.__file__)'
which ros2
```

For ROS 2 Humble on Ubuntu 22.04, Python 3.10 is typically expected. Ensure Python and compiled NumPy extensions in the Pixi environment are compatible; also check `PYTHONPATH` / environment contamination from other environments. Do not reinstall random versions system-wide to solve a per-environment conflict.

### 9.6 Bridge executable exists, but cannot load ROS 1 libraries

A bridge that builds successfully may still require a joint runtime environment containing its ROS 1 and ROS 2 libraries. Inspect the error and your Pixi activation/setup. In general, use the same supported combined runtime arrangement that was used to build and successfully test this particular bridge; do not switch to an unrelated Humble-only runtime.

### 9.7 Standard joint states work, but Sawyer-specific messages do not

Check message mappings:

```bash
ros2 run ros1_bridge dynamic_bridge --print-pairs | grep -E 'JointCommand|EndpointState|RobotAssemblyState'
```

Custom `intera_core_msgs` need matching compatible ROS 1/ROS 2 definitions available **when building the bridge**; a missing custom mapping does not invalidate the standard `sensor_msgs/JointState` test.

### 9.8 Bridge appears to work but data arrives only intermittently

Measure both sides separately:

```bash
# Noetic terminal
rostopic hz /robot/joint_states
```

```bash
# Humble terminal
ros2 topic hz /robot/joint_states
```

Check host CPU load, DDS discovery, ROS peer hostname resolution, and unexpected competing nodes. Comparable frequencies are encouraging, but frequency alone does **not** establish end-to-end latency.

## 10. Test record (fill after each session)

| Item | Record |
|---|---|
| Date / operator | |
| Sawyer reachable on TCP 11311? | |
| Ethernet interface and workstation IPv4 | |
| Sawyer hostname resolves? | |
| `rostopic echo -n 1 /robot/joint_states` | PASS / FAIL |
| `JointState` bridge mapping | PASS / FAIL |
| Bridge started without master errors | PASS / FAIL |
| ROS 2 `/robot/joint_states` | PASS / FAIL |
| ROS 1 publish rate | Hz |
| ROS 2 received rate | Hz |
| Notes / error output | |

## 11. References

- [Official `ros1_bridge` README](https://github.com/ros2/ros1_bridge/blob/master/README.md)
- [Official `ros1_bridge` documentation](https://github.com/ros2/ros1_bridge/blob/master/doc/index.rst)
- [Intera SDK `intera.sh` (ROS master and client network configuration)](https://github.com/RethinkRobotics/intera_sdk/blob/master/intera.sh)

---

**Milestone:** Receive live `/robot/joint_states` from the physical Sawyer controller in ROS 2 Humble through the Pixi-built ROS 1 bridge.