#!/usr/bin/env python3
"""Runnable checks; synthetic limits below are test fixtures, never hardware defaults."""
import copy
import importlib.machinery
import importlib.util
import tempfile
from pathlib import Path
import yaml
p=Path(__file__).resolve().parents[1]
loader=importlib.machinery.SourceFileLoader('jointctl',str(p/'tools/jointctl'))
spec=importlib.util.spec_from_loader(loader.name,loader)
cli=importlib.util.module_from_spec(spec);loader.exec_module(cli)

def main():
    root=p.parent
    driver=(root/'jc_driver/application/jc01/jc01.cpp').read_text()
    assert '#define M18_SLAVE_ID 0' in driver and '#define J03_NODE_ID 1' in driver
    assert '{3,  -1, -1, -1, -1, -1}' in driver and 'J09' not in driver
    assert 'J03_TX_PASSGE = 1,\n    J03_RX_PASSGE,\n    J03_CFG_PASSGE,' in driver
    assert 'SHM_TARGET_KEY_NUM (0x01)' in driver and 'SHM_FEEDBACK_KEY_NUM (0x02)' in driver
    assert '/tmp/j3-testbench/target_handle' in driver and '/tmp/j3-testbench/feedback_handle' in driver
    for script in ('start_driver.sh','send_command.sh','driver_session.exp'):
        data=(root/'jc_driver/tools'/script).read_bytes()
        assert b'\r' not in data and b'/home/nvidia/Documents' not in data
    session=(root/'jc_driver/tools/driver_session.exp').read_text()
    assert '[info script]' in session and 'motor_test]' in session and 'fallback' not in session
    assert 'app_motor_write' in driver and 'index != 3' in driver
    commands=(root/'jc_driver/application/app_comm.cpp').read_text()
    assert 'if (index != 3) goto out;' in commands and 'AppWrite' in commands
    assert 'registerCommand' not in (root/'jc_driver/drivers/m18/m18.cpp').read_text()
    manager=(p/'src/driver_manager.cpp').read_text()
    assert '/proc/self/exe' in manager and '/home/nvidia/Documents' not in manager
    entry=(root/'jc_driver/tests/motor_test/main.cpp').read_text()
    assert entry.index('mkdir("/tmp/j3-testbench"') < entry.index('initLogger(')
    cli.load_config(p/"profiles/experiment.yaml")
    c={
        'experiment':dict(direction='extend',mode='staircase',max_current_raw=1,extend_position_sign=1),
        'staircase':dict(levels=5,transition_s=1,settle_s=1,record_s=3),
        'linear':dict(rise_s=10,peak_hold_s=3,fall_s=10),
        'preload':dict(speed_mm_s=.1,contact_force_n=5,contact_confirm_s=.3,max_travel_mm=2,timeout_s=30),
        'unload':dict(current_slew_raw_s=.1,retract_distance_mm=1,retract_speed_mm_s=.1),
        'limits':dict(current_raw=2,force_n=100,position_min_mm=0,position_max_mm=50,
            velocity_mm_s=1,contact_displacement_mm=1,force_timeout_ms=100,
            experiment_timeout_s=300,heartbeat_timeout_s=1),
        'sensor':dict(device='/dev/ttyUSB0',baud=115200),
        'output':dict(directory='test-output'),
    }
    with tempfile.TemporaryDirectory() as directory:
        path=Path(directory)/'experiment.yaml'
        def read(value):
            path.write_text(yaml.safe_dump(value),encoding='utf-8');return cli.load_config(path)
        def rejects(value):
            try:read(value)
            except ValueError:return
            raise AssertionError('invalid config accepted')
        loaded=read(c);flat=cli.controller_config(loaded)
        assert loaded['experiment']['position_mode']=='relative' and flat['position_mode']=='relative'
        for mode in ('relative','absolute'):
            variant=copy.deepcopy(c);variant['experiment']['position_mode']=mode
            assert cli.controller_config(read(variant))['position_mode']==mode
        for mode in ('invalid','',None,1,True):
            bad=copy.deepcopy(c);bad['experiment']['position_mode']=mode;rejects(bad)
        assert flat['direction']==1 and flat['preload_speed']==.0001
        assert loaded['output']['directory']==str((path.parent/'test-output').resolve())
        reverse=copy.deepcopy(c);reverse['experiment']['direction']='retract'
        assert cli.controller_config(read(reverse))['direction']==-1
        bad=copy.deepcopy(c);bad['limits']['current_raw']=None;rejects(bad)
        for group,key,value in [('experiment','max_current_raw',float('nan')),
                                ('experiment','extend_position_sign',0),
                                ('staircase','levels',True),('staircase','levels',1.5),
                                ('limits','force_n',-1),('limits','current_raw',.5),
                                ('limits','position_max_mm',-1),('sensor','baud',123),
                                ('preload','contact_force_n',101)]:
            bad=copy.deepcopy(c);bad[group][key]=value;rejects(bad)
        bad=copy.deepcopy(c);bad['experiment']['surprise']=1;rejects(bad)
        bad=copy.deepcopy(c);del bad['limits']['force_n'];rejects(bad)
        path.write_text(yaml.safe_dump(c)+'\nexperiment: {}\n')
        try:cli.load_config(path)
        except ValueError:pass
        else:raise AssertionError('duplicate key accepted')
        path.write_text('!!python/object/apply:os.system ["must-not-run"]')
        try:cli.load_config(path)
        except (yaml.YAMLError,ValueError):pass
        else:raise AssertionError('unsafe YAML tag accepted')
    print('YAML checks passed')
if __name__=='__main__':main()
