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
    template=yaml.safe_load((p/'profiles/experiment.yaml').read_text())
    c=copy.deepcopy(template)
    c['experiment'].update(max_current_a=1,extend_position_sign=1)
    c['preload'].update(speed_mm_s=.1,contact_force_n=5,max_travel_mm=2,timeout_s=30)
    c['unload'].update(current_slew_a_s=.1,retract_distance_mm=1,retract_speed_mm_s=.1)
    c['limits'].update(current_a=2,force_n=100,position_min_mm=0,position_max_mm=50,
        velocity_mm_s=1,contact_displacement_mm=1,experiment_timeout_s=300)
    with tempfile.TemporaryDirectory() as directory:
        path=Path(directory)/'experiment.yaml'
        def read(value):
            path.write_text(yaml.safe_dump(value),encoding='utf-8');return cli.load_config(path)
        def rejects(value):
            try:read(value)
            except ValueError:return
            raise AssertionError('invalid config accepted')
        loaded=read(c);flat=cli.controller_config(loaded)
        assert flat['direction']==1 and flat['preload_speed']==.0001
        assert loaded['output']['directory']==str((path.parent/'../../logs').resolve())
        reverse=copy.deepcopy(c);reverse['experiment']['direction']='retract'
        assert cli.controller_config(read(reverse))['direction']==-1
        rejects(template)
        for group,key,value in [('experiment','max_current_a',float('nan')),
                                ('experiment','extend_position_sign',0),
                                ('staircase','levels',True),('staircase','levels',1.5),
                                ('limits','force_n',-1),('limits','current_a',.5),
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
