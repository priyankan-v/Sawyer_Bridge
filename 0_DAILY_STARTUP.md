The reproducible Sawyer custom-interface startup is now documented in [README.md](README.md).
The environments and message paths are explained in [How our setup works](docs/HOW_THE_SETUP_WORKS.md).
For daily use, start the bridge with `pixi run bridge-start` in one terminal,
then enter `pixi run ros2-shell` in another and run ordinary `ros2` commands.
These tasks load the project network settings and appropriate environments.
The procedure below is retained for the original joint-state-only bridge installation.

After connecting Sawyer's Ethernet cable, the following is the basic startup procedure.

## Terminal 1
Check the link availability
```bash
ip -br addr
```
Look for `enp3s0` with the space `169.254.121.2/16 dev enp3s0`

### Clear the terminal and Start NOETIC
```bash
cd ~/sawyer_rs
pixi shell -e noetic
```
```bash
export ROS_MASTER_URI=http://169.254.121.3:11311
export ROS_IP=169.254.121.2
unset ROS_HOSTNAME
```
```bash
rosparam load config/bridge.yaml
rosparam get /topics
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

## Terminal 2 - Humble
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
```bash
export ROS_MASTER_URI=http://169.254.121.3:11311
export ROS_IP=169.254.121.2
unset ROS_HOSTNAME
```

Verify what the bridge will use:
```bash
echo "$ROS_MASTER_URI"
echo "$ROS_IP"
ros2 pkg executables ros1_bridge
```

```bash
ros2 run ros1_bridge parameter_bridge
```

## Terminal 3 - Humble
```bash
cd ~/sawyer_rs
pixi shell -e humble

ros2 topic echo /robot/joint_states --once
ros2 topic hz /robot/joint_states
```
