"""Own and clean up only our subprocess groups; isolate both ROS graphs."""
import json
import os
import signal
import socket
import subprocess
import time
import xmlrpc.client
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
os.chdir(ROOT)
assert os.environ['ROS_MASTER_URI'] == 'http://127.0.0.1:11421'
assert os.environ['ROS_DOMAIN_ID'] == '87' and os.environ['ROS_LOCALHOST_ONLY'] == '1'
socket.setdefaulttimeout(2)
owned, logs = [], []


def start(name, command):
    log = open(ROOT / 'docs/results' / (name+'.log'), 'w')
    logs.append(log)
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    owned.append(process)
    return process


def run(command, seconds=45):
    result = subprocess.run(command, timeout=seconds, text=True, capture_output=True)
    print(result.stdout, flush=True)
    print(result.stderr, flush=True)
    assert result.returncode == 0, (command, result.returncode)
    return result


try:
    # Refuse to attach fixtures to an existing master on our designated port.
    with socket.socket() as probe:
        probe.bind(('127.0.0.1',11421))
    start('local_master', ['scripts/env.sh','ros1','roscore','-p','11421'])
    deadline = time.monotonic()+15
    while True:
        try:
            assert xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getUri('/test')[0] == 1
            break
        except (OSError, AssertionError):
            if time.monotonic()>deadline:
                raise
            time.sleep(.2)
    start('local_ros1_fixture', ['scripts/env.sh','ros1','python','tests/ros1_fixture.py'])
    config = json.loads(json.dumps(__import__('yaml').safe_load((ROOT/'config/sawyer_bridge.yaml').read_text())))
    config['topics'] = [t for t in config['topics'] if t['enabled']]
    config['topics'] += [dict(topic='/sawyer_bridge_test/joint_command', type='intera_core_msgs/msg/JointCommand', direction='2to1',queue_size=100),
                         dict(topic='/sawyer_bridge_test/ack', type='std_msgs/msg/String', direction='1to2',queue_size=100),
                         dict(topic='/sawyer_bridge_test/disabled',type='std_msgs/msg/String',direction='2to1',queue_size=1,enabled=False)]
    config['services_2_to_1'] = [dict(service='/sawyer_bridge_test/'+short,type='intera_core_msgs/'+typ,package='')
                                for short,typ in [('fk','SolvePositionFK'),('ik','SolvePositionIK'),('io','IOComponentCommandSrv')]]
    cfg_path = ROOT/'.runtime/local_bridge.json'
    cfg_path.write_text(json.dumps(config))
    start('local_bridge', ['scripts/start_bridge.sh',str(cfg_path)])
    run(['scripts/env.sh','ros2','python','tests/ros2_transport.py','docs/results/local_transport.json'],60)
    run(['scripts/env.sh','ros2','python','-m','unittest','discover','-s','tests','-v'])
    run(['scripts/control.sh','one','--joint','right_j0','--position','0.01','--duration','2'])
    run(['scripts/env.sh','ros2','ros2','run','sawyer_ros2_control','move_joint','--joint','right_j2','--distance','.01','--time','2'])
    run(['scripts/env.sh','ros2','python','scripts/move_joint.py','--joint','head_pan','--position','.01','--time','2'])
    # No fixture or bridge ever advertises a physical JointCommand publisher.
    state = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getSystemState('/test')[2]
    assert '/robot/limb/right/joint_command' not in dict(state[0])
    assert '/sawyer_bridge_test/disabled' not in dict(state[0]) and '/sawyer_bridge_test/disabled' not in dict(state[1])
    print('PASS: dry-run application produced no physical-command publisher.', flush=True)
    # Restart our own bridge with head commands enabled only on the private graph.
    bridge=owned[-1]
    os.killpg(bridge.pid,signal.SIGINT)
    bridge.wait(timeout=5)
    config['topics'].append(dict(topic='/robot/head/command_head_pan',type='intera_core_msgs/msg/HeadPanCommand',direction='2to1',queue_size=1))
    cfg_path.write_text(json.dumps(config))
    log_path=ROOT/'.runtime/head_commands.jsonl'
    log_path.write_text('')
    start('local_head_bridge',['scripts/start_bridge.sh',str(cfg_path)])
    run(['scripts/env.sh','ros2','python','scripts/move_joint.py','--joint','head_pan','--distance','.01','--time','2',
         '--execute','--operator-approved','--report','docs/results/local_head_motion.json'])
    # Relative displacement now starts at .01; an absolute target moves back to zero.
    run(['scripts/env.sh','ros2','ros2','run','sawyer_ros2_control','move_joint','--joint','head_pan','--position','0','--time','2',
         '--execute','--operator-approved'])
    commands=[json.loads(line) for line in log_path.read_text().splitlines()]
    assert len(commands)>100
    assert all(c['pan_mode']==1 and abs(c['speed_ratio']-.05)<1e-6 and -1<=c['target']<=.9064 for c in commands)
    assert abs(commands[-1]['target'])<.002
    report_path=ROOT/'docs/results/local_head_motion.json'
    report=json.loads(report_path.read_text())
    report.update(status='PASS',scope='Simulated ROS 1 head on localhost master 11421 and DDS domain 87; no physical hardware',
                  relative_and_absolute_execution_verified=True,head_commands_observed=len(commands))
    report_path.write_text(json.dumps(report,indent=2)+'\n')
    state=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getSystemState('/test')[2]
    assert '/robot/limb/right/joint_command' not in dict(state[0])
    print('PASS: isolated head relative and absolute quintic execution; no arm commands.',flush=True)
finally:
    for process in reversed(owned):
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
    for process in reversed(owned):
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGKILL)
            process.wait()
    for log in logs:
        log.close()
