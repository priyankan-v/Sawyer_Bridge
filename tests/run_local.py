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


def read_arm_commands():
    return [json.loads(line) for line in (ROOT/'.runtime/arm_commands.jsonl').read_text().splitlines()]


try:
    # Refuse to attach fixtures to an existing master on our designated port.
    with socket.socket() as probe:
        probe.bind(('127.0.0.1',11421))
    missing_master_report = ROOT/'docs/results/local_unavailable_master.json'
    begin = time.monotonic()
    result = subprocess.run(['scripts/env.sh','ros1','python','scripts/preflight.py','--require-ready',
                             '--joint','right_j2','--output',str(missing_master_report)],
                            timeout=15,text=True,capture_output=True)
    missing_master = json.loads(missing_master_report.read_text())
    assert result.returncode == 1 and not missing_master['ready']
    assert 'ROS master unavailable' in missing_master['errors'][0]
    missing_master.update(test_status='PASS',scope='Unused localhost master port 11421; no physical hardware',
                          elapsed_seconds=time.monotonic()-begin)
    missing_master_report.write_text(json.dumps(missing_master,indent=2)+'\n')
    print('PASS: missing ROS master fails preflight before node registration.',flush=True)
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
    run(['scripts/env.sh','ros2','ros2','run','sawyer_ros2_control','move_joint','--joint','right_j2','--distance','.01','--time','2','--dry-run'])
    run(['scripts/env.sh','ros2','python','scripts/move_joint.py','--joint','head_pan','--position','.01','--time','2','--dry-run'])
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
         '--report','docs/results/local_head_motion.json'])
    # Relative displacement now starts at .01; an absolute target moves back to zero.
    run(['scripts/env.sh','ros2','ros2','run','sawyer_ros2_control','move_joint','--joint','head_pan','--position','0','--time','2'])
    commands=[json.loads(line) for line in log_path.read_text().splitlines()]
    assert len(commands)>100
    assert all(c['pan_mode']==1 and abs(c['speed_ratio']-1.)<1e-6 and -1<=c['target']<=.9064 for c in commands)
    assert abs(commands[-1]['target'])<.002
    report_path=ROOT/'docs/results/local_head_motion.json'
    report=json.loads(report_path.read_text())
    assert report['status'] == 'reached'
    report.update(test_status='PASS',scope='Simulated ROS 1 head on localhost master 11421 and DDS domain 87; no physical hardware',
                  relative_and_absolute_execution_verified=True,head_commands_observed=len(commands))
    report_path.write_text(json.dumps(report,indent=2)+'\n')
    state=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getSystemState('/test')[2]
    assert '/robot/limb/right/joint_command' not in dict(state[0])
    print('PASS: isolated head relative and absolute quintic execution; no arm commands.',flush=True)
    # Enable arm paths only after the head-only graph checks, still on the private master.
    bridge = owned[-1]
    os.killpg(bridge.pid, signal.SIGINT)
    bridge.wait(timeout=5)
    config['topics'] = [t for t in config['topics'] if t['topic'] != '/robot/head/command_head_pan']
    config['topics'] += [dict(t, enabled=True) for t in __import__('yaml').safe_load(
        (ROOT/'config/sawyer_bridge.yaml').read_text())['topics']
        if t['topic'] in ('/robot/limb/right/joint_command', '/robot/limb/right/joint_command_timeout',
                          '/robot/limb/right/set_speed_ratio')]
    cfg_path.write_text(json.dumps(config))
    arm_log = ROOT/'.runtime/arm_commands.jsonl'
    arm_log.write_text('')
    start('local_arm_bridge', ['scripts/start_bridge.sh', str(cfg_path)])
    selected = 'right_j2'
    base = ['scripts/env.sh','ros2','python','scripts/move_joint.py','--joint',selected,'--time','2']
    for destination, value, report_name in (('--distance','.05','local_arm_relative.json'),
                                             ('--position','0','local_arm_absolute.json')):
        arm_log.write_text('')
        run(base + [destination, value, '--report', 'docs/results/'+report_name])
        commands = read_arm_commands()
        assert len(commands) > 100
        for command in commands:
            assert command['mode'] == 1 and command['names'] == [selected]
            assert not command['velocity'] and not command['acceleration'] and not command['effort']
            assert command['stamp_ns'] > 0 and not command['collision']
            assert abs(command['speed_ratio']-1.) < 1e-6 and abs(command['command_timeout']-.2) < 1e-6
        goal = .05 if destination == '--distance' else 0.
        assert abs(commands[-1]['position'][0]-goal) < .002
        report_path = ROOT/'docs/results'/report_name
        report = json.loads(report_path.read_text())
        assert report['status'] == 'reached' and report['final_error_rad'] <= .002
        report.update(test_status='PASS',scope='Simulated ROS 1 arm on localhost master 11421 and DDS domain 87; no physical hardware',
                      command_fields_verified=True, only_selected_joint_commanded=True,
                      arm_commands_observed=len(commands))
        report_path.write_text(json.dumps(report,indent=2)+'\n')
    print('PASS: direct arm relative and absolute execution; only the selected joint is commanded.',flush=True)
    # A too-short duration must be rejected before either commands or motion.
    arm_log.write_text('')
    short = ['scripts/env.sh','ros2','python','scripts/move_joint.py','--joint',selected,'--time','.05']
    result = subprocess.run(short + ['--position','.1'], timeout=20, text=True, capture_output=True)
    assert result.returncode == 1 and 'minimum quintic time' in result.stdout and 'No commands sent' in result.stdout
    assert read_arm_commands() == []
    report = dict(status='PASS',scope='Isolated ROS 1 fixture; no physical hardware',
                  requested_time_seconds=.05,no_commands_published=True,message=result.stdout.strip())
    (ROOT/'docs/results/local_minimum_time_rejection.json').write_text(json.dumps(report,indent=2)+'\n')
    print('PASS: minimum-time rejection sends no commands.',flush=True)
    # A feasible reference still needs measured arrival; repeating absolute zero is a no-op.
    fast = ['scripts/env.sh','ros2','python','scripts/move_joint.py','--joint',selected,'--time','1.1']
    run(fast + ['--position','.1'])
    run(fast + ['--position','0','--report','docs/results/local_absolute_zero.json'])
    report_path = ROOT/'docs/results/local_absolute_zero.json'
    report = json.loads(report_path.read_text())
    assert report['status'] == 'reached' and report['target_rad'] == 0.
    assert report['final_error_rad'] <= report['tolerance_rad'] and report['settle_seconds'] < .05
    assert report['trajectory_seconds'] >= report['minimum_time_seconds']
    report.update(test_status='PASS',scope='Rate-limited simulated ROS 1 arm; no physical hardware')
    report_path.write_text(json.dumps(report,indent=2)+'\n')
    arm_log.write_text('')
    run(['scripts/env.sh','ros2','python','scripts/move_joint.py','--joint',selected,
         '--position','0','--time','5','--report','docs/results/local_repeated_absolute_zero.json'])
    report_path = ROOT/'docs/results/local_repeated_absolute_zero.json'
    report = json.loads(report_path.read_text())
    assert report['status'] == 'already_at_target' and report['samples'] == 0
    assert read_arm_commands() == []
    report.update(test_status='PASS',scope='Rate-limited simulated ROS 1 arm; no physical hardware',
                  no_commands_published=True)
    report_path.write_text(json.dumps(report,indent=2)+'\n')
    print('PASS: measured arrival at absolute zero and repeated zero produces no commands.',flush=True)
    # Reproduce the operator's small measured overshoot only on the private fixture.
    run(['scripts/env.sh','ros1','rostopic','pub','--once','/sawyer_bridge_test/feedback_offset',
         'std_msgs/Float64','data: -0.0016419921875'])
    for end, goal, duration, filename in (('-end', -1., '5', 'local_arm_lower_limit_overshoot.json'),
                                          ('+end', 1., '8', 'local_arm_limit_recovery.json')):
        arm_log.write_text('')
        run(['scripts/env.sh','ros2','python','scripts/move_joint.py','--position',end,
             '--time',duration,'--joint',selected,'--report','docs/results/'+filename])
        commands = read_arm_commands()
        assert commands and commands[-1]['position'] == [goal]
        assert all(-1. <= command['position'][0] <= 1. and command['names'] == [selected]
                   and command['mode'] == 1 for command in commands)
        report_path = ROOT/'docs/results'/filename
        report = json.loads(report_path.read_text())
        assert report['status'] == 'reached' and report['target_rad'] == goal
        assert report['final_error_rad'] <= report['tolerance_rad']
        if end == '-end':
            assert report['measured_rad'] < -1. and 0. < report['limit_overshoot_rad'] < .002
            arm_log.write_text('')
            run(['scripts/env.sh','ros2','python','scripts/move_joint.py','--position','-end',
                 '--time','5','--joint',selected,'--report','docs/results/local_repeated_limit_overshoot.json'])
            repeated = json.loads((ROOT/'docs/results/local_repeated_limit_overshoot.json').read_text())
            assert repeated['status'] == 'already_at_target' and not read_arm_commands()
        else:
            assert report['start_rad'] < -1. and report['reference_start_rad'] == -1.
        report.update(test_status='PASS', scope='Measured-limit overshoot on isolated ROS 1 fixture; no physical hardware',
                      end_option=end, only_selected_joint_commanded=True,
                      all_commands_within_limits=True)
        report_path.write_text(json.dumps(report, indent=2)+'\n')
    run(['scripts/env.sh','ros1','rostopic','pub','--once','/sawyer_bridge_test/feedback_offset',
         'std_msgs/Float64','data: 0.0'])
    run(['scripts/env.sh','ros2','python','scripts/move_joint.py','--position','0',
         '--time','5','--joint',selected])
    print('PASS: small measured limit overshoot completes, repeated -end sends nothing, +end recovers with all commands in limits.', flush=True)
    arm_log.write_text('')
    result = subprocess.run(base + ['--position','2'], timeout=20, text=True, capture_output=True)
    assert result.returncode == 1 and 'outside joint limits' in result.stdout
    assert read_arm_commands() == []
    print('PASS: out-of-limit target is rejected without publishing commands.',flush=True)
    # Raise simulated collision after 20 arm commands and require an abort without a hold.
    run(['scripts/env.sh','ros1','rostopic','pub','--once','/sawyer_bridge_test/inject_collision',
         'std_msgs/Bool','data: true'])
    arm_log.write_text('')
    collision_start = time.monotonic()
    # The separate full-arm guarded application still supports collision monitoring.
    result = subprocess.run(['scripts/control.sh','one','--joint',selected,'--position','.01','--duration','2',
                             '--execute','--operator-approved'], timeout=45, text=True, capture_output=True)
    print(result.stdout,flush=True)
    print(result.stderr,flush=True)
    assert result.returncode == 1 and 'Robot readiness or collision check failed' in result.stdout
    commands = read_arm_commands()
    assert 20 <= len(commands) < 60 and any(c['collision'] for c in commands)
    # No queued or safe-hold commands should arrive once the application exits.
    time.sleep(.3)
    assert len(read_arm_commands()) == len(commands)
    collision_report = dict(status='PASS',scope='Separate guarded full-arm application on isolated simulated arm; no physical hardware',
                            returncode=result.returncode,arm_commands_observed=len(commands),
                            no_commands_after_exit=True,collision_injected_after_commands=20,
                            elapsed_seconds=time.monotonic()-collision_start,
                            abort_reason='Robot readiness or collision check failed')
    (ROOT/'docs/results/local_arm_collision_abort.json').write_text(json.dumps(collision_report,indent=2)+'\n')
    print('PASS: separate guarded full-arm application still aborts on collision.',flush=True)
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
