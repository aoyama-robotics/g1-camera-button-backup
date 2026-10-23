import json, os, threading, time
import unitree_sdk2py.core.channel as channel
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

def watchdog():
    time.sleep(15)
    print(json.dumps({'result':'watchdog_timeout','motor_commands_sent':0}),flush=True)
    os._exit(2)
threading.Thread(target=watchdog,daemon=True).start()
box = {'count':0}
def cb(m):
    if 'first_tick' not in box: box['first_tick']=m.tick
    box['count'] += 1
    box['last'] = m
    box['time'] = time.monotonic()
print('READ_ONLY_SUBSCRIBER iface=<PRIVATE_IP> domain=0 topic=rt/lowstate duration=8s',flush=True)
try:
    channel.ChannelConfigHasInterface = channel.ChannelConfigHasInterface.replace('name="$__IF_NAME__$"', 'address="$__IF_NAME__$"').replace('/tmp/cdds.LOG', os.path.abspath('cdds_readonly.log').replace('\\','/'))
    ChannelFactoryInitialize(0, '<PRIVATE_IP>')
    def check_mode():
        try:
            client=MotionSwitcherClient()
            client.SetTimeout(.5)
            client.Init()
            initial=client.CheckMode()
            print(json.dumps({'recovery_before':initial,'motor_commands_sent':0}),flush=True)
            if initial[0]!=0 or not isinstance(initial[1],dict):
                raise RuntimeError('Cannot establish mode; no selection')
            if initial[1].get('name')=='':
                selected=client.SelectMode('ai')
                print(json.dumps({'select_ai_result':selected,'motor_commands_sent':0}),flush=True)
                if selected[0]!=0:raise RuntimeError('AI selection unsuccessful; no retry')
                time.sleep(1)
            elif initial[1].get('name')!='ai':
                raise RuntimeError('Unexpected controller; no selection')
            code, result=client.CheckMode()
            box['check_mode']={'code':code,'result':result}
            api_code, api_version=client.GetServerApiVersion()
            box['server_api_version']={'code':api_code,'value':api_version,'is_firmware_version':False}
            from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
            loco=LocoClient();loco.SetTimeout(.5);loco.Init()
            fsm_code,fsm_id=loco.GetFsmId()
            box['fsm']={'code':fsm_code,'id':fsm_id}
        except Exception as e:
            box['check_mode']={'error':str(e)}
    sub = ChannelSubscriber('rt/lowstate', LowState_)
    sub.Init(cb,10)
    threading.Thread(target=check_mode,daemon=True).start()
    time.sleep(8)
    result={'count':box['count'],'first_tick':box.get('first_tick'),'motor_commands_sent':0,'check_mode':box.get('check_mode',{'status':'no_result'}),'server_api_version':box.get('server_api_version'),'fsm':box.get('fsm')}
    if 'last' in box:
        m=box['last']
        result.update(age_s=time.monotonic()-box['time'],mode_machine=m.mode_machine,tick=m.tick,lowstate_version=list(m.version),
          joints=[{'index':i,'q':m.motor_state[i].q,'dq':m.motor_state[i].dq,'mode':m.motor_state[i].mode} for i in range(29)],waist=[{'index':i,'q':m.motor_state[i].q,'dq':m.motor_state[i].dq,'mode':m.motor_state[i].mode,'vol':m.motor_state[i].vol,'temperature':list(m.motor_state[i].temperature)} for i in (12,13,14)],
          arms=[{'index':i,'q':m.motor_state[i].q,'dq':m.motor_state[i].dq,'mode':m.motor_state[i].mode} for i in range(15,29)])
    print(json.dumps(result),flush=True)
except Exception as e:
    print(json.dumps({'error':type(e).__name__,'message':str(e),'motor_commands_sent':0}),flush=True)
    os._exit(1)
os._exit(0)


