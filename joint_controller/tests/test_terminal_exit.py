#!/usr/bin/env python3
"""PTY + fake controller lifecycle checks; all sockets stay in a temporary directory."""
import os
import runpy
from unittest.mock import Mock
from pathlib import Path
import pty
import select
import signal
import subprocess
import sys
import tempfile
import time
import yaml

FAKE = r'''#!/usr/bin/env python3
import json, os, select, signal, socket, time
from pathlib import Path
root=Path(__file__).parent
path=str(root/'controller.sock')
stopped=False
def stop(*args):
    global stopped
    stopped=True
signal.signal(signal.SIGTERM,stop)
(root/'pid').write_text(str(os.getpid()))
if os.environ.get('STARTUP_WAIT'):
    while not stopped: time.sleep(.01)
else:
    server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
    server.bind(path);server.listen()
    urgent=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM);urgent.bind(path+'.urgent')
    while not stopped:
        for ready in select.select([server,urgent],[],[],.02)[0]:
            if ready is urgent: urgent.recv(8);continue
            client,_=server.accept()
            with client:
                data=b''
                while b'\n' not in data:
                    part=client.recv(8192)
                    if not part:break
                    data+=part
                request=json.loads(data)
                client.sendall(json.dumps(dict(ok=request['op']!='experiment-start',message='TEST LOAD REJECTED' if request['op']=='experiment-start' else 'accepted',state='IDLE',stage='IDLE',driver_disabled_confirmed=True)).encode())
                if request['op']=='shutdown':stopped=True
    server.close();urgent.close()
# Simulate draining the final recording before unlinking the sockets.
(root/'stopping').touch();time.sleep(.2)
(root/'saved').write_text('recording finalized')
for suffix in ('','.urgent'):
    try:os.unlink(path+suffix)
    except FileNotFoundError:pass
'''


def run_case(cli_source, profile, action, startup=False, repeat=False):
    with tempfile.TemporaryDirectory(prefix='j3-exit-test-') as temporary:
        root=Path(temporary)
        text=cli_source.read_text().replace("SOCKET = '/tmp/j3-testbench/controller.sock'",f'SOCKET = {str(root/"controller.sock")!r}')
        assert '/tmp/j3-testbench/controller.sock' not in text
        cli=root/'jointctl';cli.write_text(text)
        fake=root/'joint_controller';fake.write_text(FAKE);fake.chmod(0o755)
        config=yaml.safe_load(profile.read_text());config['output']['directory']=str(root/'logs')
        cfg=root/'experiment.yaml';cfg.write_text(yaml.safe_dump(config))
        master,slave=pty.openpty()
        env=os.environ.copy()
        if startup:env['STARTUP_WAIT']='1'
        process=subprocess.Popen([sys.executable,str(cli),'experiment','--config',str(cfg)],stdin=slave,stdout=slave,stderr=slave,env=env)
        os.close(slave)
        output=bytearray();child=None
        def wait_until(predicate):
            deadline=time.monotonic()+8
            while not predicate():
                assert time.monotonic()<deadline,output[-2500:].decode(errors='replace')
                if select.select([master],[],[],.02)[0]:
                    try:output.extend(os.read(master,65536))
                    except OSError:pass
        try:
            wait_until(lambda:(root/'pid').exists())
            child=int((root/'pid').read_text())
            if not startup:wait_until(lambda:'正式记录'.encode() in output)
            if action=='q':
                began=time.monotonic()
                wait_until(lambda:time.monotonic()-began>=1.2)
                assert 2<=output.count(b'\r\x1b[2K')<=3, 'status display must refresh at 2 Hz'
                os.write(master,b's')
                wait_until(lambda:b'TEST LOAD REJECTED' in output)
                mark=len(output);began=time.monotonic()
                wait_until(lambda:time.monotonic()-began>=1.2)
                assert b'TEST LOAD REJECTED' in output[mark:], 'status poll erased command rejection'
                os.write(master,b'q')
            else:os.kill(process.pid,action)
            if repeat:
                wait_until(lambda:(root/'stopping').exists())
                os.kill(process.pid,signal.SIGINT)
            wait_until(lambda:process.poll() is not None)
            assert process.wait()==(0 if action=='q' else 130),output.decode(errors='replace')
            assert (root/'saved').read_text()=='recording finalized'
            assert not (root/'controller.sock').exists()
            assert not (root/'controller.sock.urgent').exists()
            try:os.kill(child,0)
            except ProcessLookupError:pass
            else:raise AssertionError('orphan controller')
            child=None
            assert b'Traceback' not in output,output.decode(errors='replace')
        finally:
            if process.poll() is None:process.kill();process.wait()
            if child:
                try:os.kill(child,signal.SIGKILL)
                except ProcessLookupError:pass
            os.close(master)


def main():
    source=Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).resolve().parents[1]/'tools/jointctl'
    profile=Path(sys.argv[2]) if len(sys.argv)>2 else Path(__file__).resolve().parents[1]/'profiles/experiment.yaml'
    stop=runpy.run_path(str(source))['stop_controller']
    previous=signal.getsignal(signal.SIGINT)
    for result in (4,subprocess.TimeoutExpired('fake-controller',30)):
        child=Mock(pid=123,poll=Mock(return_value=None))
        if isinstance(result,Exception):child.wait.side_effect=result
        else:child.wait.return_value=result
        try:stop(child)
        except RuntimeError as error:
            assert ('PID 123' if isinstance(result,Exception) else 'code 4') in str(error)
        else:raise AssertionError('unconfirmed stop reported as successful')
        child.terminate.assert_called_once();child.kill.assert_not_called()
        assert signal.getsignal(signal.SIGINT)==previous
    for action in ('q',signal.SIGINT,signal.SIGTERM,signal.SIGHUP):
        run_case(source,profile,action)
    run_case(source,profile,signal.SIGINT,startup=True)
    run_case(source,profile,signal.SIGINT,repeat=True)
    print('terminal lifecycle: q, SIGINT, SIGTERM, SIGHUP, startup interruption and repeated Ctrl-C passed')

if __name__=='__main__':main()
