"""No DDS initialization or sending: real upper-body packet/CRC/serialization."""
import argparse,json,time
from pathlib import Path
import numpy as np
from precise_timing import PreciseSleeper
from waist_hold_candidate import fill_hold_command
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

def run(name,sleep,count):
    clock=time.perf_counter;crc=CRC();times=[];work=[];late=[];overruns=0
    expected=None
    for k in range(count):
        start=clock()
        if expected is not None:
            delay=start-expected;late.append(delay)
            if delay>.050:raise RuntimeError('50ms lateness deadline exceeded')
        packet=fill_hold_command(unitree_hg_msg_dds__LowCmd_(),[0.]*14,[0.]*3,.2,5)
        packet.crc=crc.Crc(packet)
        packet.serialize()
        times.append(clock());work.append(times[-1]-start)
        finished=clock()
        if finished>=start+args.period:
            overruns+=1;expected=finished+args.period
        else:expected=start+args.period
        sleep(max(0,expected-clock()))
    intervals=np.diff(times)*1000
    return dict(name=name,cycles=count,motor_commands_sent=0,dds_initialized=False,
        interval_ms_p50_p95_p99_max=np.percentile(intervals,[50,95,99,100]).tolist(),
        late_ms_p50_p95_p99_max=np.percentile(np.array(late)*1000,[50,95,99,100]).tolist(),
        effective_hz=(len(times)-1)/(times[-1]-times[0]),
        period_overruns=overruns,missed_50ms_deadlines=int(np.sum(np.array(late)>.05)),
        work_ms_p50_p99_max=np.percentile(np.array(work)*1000,[50,99,100]).tolist())

parser=argparse.ArgumentParser()
parser.add_argument('--read-only-dds',action='store_true')
parser.add_argument('--period',type=float,default=.004)
parser.add_argument('--cycles',type=int,default=2500)
args=parser.parse_args()
if args.period not in (.004,.020) or not 100<=args.cycles<=2500:
    raise ValueError('Only bounded 4ms/20ms benchmarks supported')
transport=None
if args.read_only_dds:
    import unitree_sdk2py.core.channel as channel
    from arm_runtime import UnitreeTransport
    channel.ChannelConfigHasInterface=channel.ChannelConfigHasInterface.replace(
        'name="$__IF_NAME__$"','address="$__IF_NAME__$"').replace(
        '/tmp/cdds.LOG',str(Path('timing_dds_load.log').resolve()).replace('\\','/'))
    class CountingTransport(UnitreeTransport):
        def __init__(self,*args):self.received_count=0;super().__init__(*args)
        def _receive(self,msg):self.received_count+=1;super()._receive(msg)
    transport=CountingTransport('<PRIVATE_IP>')
    time.sleep(1)
old=None if args.read_only_dds else run('time.sleep (high-resolution measuring clock)',time.sleep,200)
sleeper=PreciseSleeper()
try:new=run(sleeper.backend,sleeper,args.cycles)
finally:sleeper.close()
result={'old_wait':old,'corrected':new,'clock':str(time.get_clock_info('perf_counter')),
        'global_timer_settings_changed':False,'includes_dds_io':args.read_only_dds,
        'motor_commands_sent':0,'period_s':args.period,'dds_initialized':args.read_only_dds}
new.pop('dds_initialized',None)
if transport is not None:
    result['read_only_load']={'state_count':transport.received_count,
         'state_error':transport.state_error,'mode_error':transport.mode_error,
         'mode':transport.mode,'motor_publisher_created':transport.publisher is not None}
    transport.close()
filename='timing_benchmark_dds_20ms.json' if args.period==.020 else ('timing_benchmark_dds.json' if args.read_only_dds else 'timing_benchmark.json')
Path(filename).write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result),flush=True)
