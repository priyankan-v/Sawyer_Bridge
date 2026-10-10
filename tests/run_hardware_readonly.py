"""Read-only bridge integration; keep existing joint-state bridge process alive."""
import json
import os
import signal
import subprocess
from pathlib import Path
import yaml

ROOT=Path(__file__).resolve().parents[1]
os.chdir(ROOT)
processes=[]
logs=[]


def start(name,argv):
    log=open(ROOT/'docs/results'/f'{name}.log','w')
    logs.append(log)
    p=subprocess.Popen(argv,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    processes.append(p)
    return p


try:
    subprocess.run(['scripts/env.sh','ros1','python','scripts/preflight.py','--output','docs/results/hardware_preflight.json'],check=True,timeout=20)
    config=yaml.safe_load((ROOT/'config/sawyer_bridge.yaml').read_text())
    config['topics']=[t for t in config['topics'] if t['enabled'] or t['topic']=='/robot/head/head_state']
    for t in config['topics']:
        t['enabled']=True
        assert t['direction']=='1to2'
    config['topics'] += [dict(topic='/io/end_effector/config',type='intera_core_msgs/msg/IONodeConfiguration',direction='1to2',queue_size=1,qos=dict(history='keep_last',depth=1,durability='transient_local')),
                         dict(topic='/io/end_effector/right_gripper/state',type='intera_core_msgs/msg/IODeviceStatus',direction='1to2',queue_size=1)]
    config['services_2_to_1']=[dict(service='/ExternalTools/right/PositionKinematicsNode/'+suffix,type='intera_core_msgs/'+typ,package='')
                              for suffix,typ in [('FKService','SolvePositionFK'),('IKService','SolvePositionIK')]]
    cfg=ROOT/'.runtime/hardware_readonly.json'
    cfg.parent.mkdir(exist_ok=True)
    cfg.write_text(json.dumps(config))
    bridge=start('hardware_bridge',['scripts/start_bridge.sh',str(cfg)])
    sampler=start('hardware_ros1_sampler',['scripts/env.sh','ros1','python','scripts/measure_ros1.py','docs/results/hardware_ros1_samples.json','15'])
    subprocess.run(['scripts/env.sh','ros2','python','tests/hardware_readonly.py','docs/results/hardware_readonly.json','docs/results/hardware_ros2_samples.json'],check=True,timeout=40)
    sampler.wait(timeout=20)
    assert sampler.returncode==0
    # Dry run must refuse the currently observed out-of-limit configuration.
    dry=subprocess.run(['scripts/control.sh','one','--joint','right_j0','--position','-2.785','--duration','5'],text=True,capture_output=True,timeout=20)
    (ROOT/'docs/results/hardware_command_dry_run.log').write_text(dry.stdout+dry.stderr)
    print(dry.stdout+dry.stderr)
    result=json.loads((ROOT/'docs/results/hardware_readonly.json').read_text())
    result['command_dry_run']={'returncode':dry.returncode,'output':dry.stdout+dry.stderr}
    # Compare identical arm stamp/name identities within the common observation window.
    ros1=json.loads((ROOT/'docs/results/hardware_ros1_samples.json').read_text())
    ros2=json.loads((ROOT/'docs/results/hardware_ros2_samples.json').read_text())
    arm1=[r for r in ros1 if all(f'right_j{i}' in r['names'] for i in range(7))]
    arm2=[r for r in ros2 if all(f'right_j{i}' in r['names'] for i in range(7))]
    lo=max(min(r['stamp_ns'] for r in arm1),min(r['stamp_ns'] for r in arm2))
    hi=min(max(r['stamp_ns'] for r in arm1),max(r['stamp_ns'] for r in arm2))
    keys=lambda rows:{(r['stamp_ns'],tuple(r['names'])) for r in rows if lo<=r['stamp_ns']<=hi}
    k1,k2=keys(arm1),keys(arm2)
    result['common_window_sample_comparison']={'ros1_unique':len(k1),'ros2_unique':len(k2),'ros1_seen_missing_in_ros2':len(k1-k2),
                                               'ros2_seen_missing_in_ros1':len(k2-k1),
                                               'meaning':'Receiver-to-receiver timestamp/name correlation, not proof of controller-to-bridge loss'}
    result['ros1_arm_receive_hz']=(len(arm1)-1)/((arm1[-1]['received_ns']-arm1[0]['received_ns'])/1e9)
    (ROOT/'docs/results/hardware_readonly.json').write_text(json.dumps(result,indent=2)+'\n')
finally:
    for p in reversed(processes):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGKILL)
                p.wait()
    for log in logs:log.close()
