"""One bounded read-only GetFsmId request, exact raw response plus parser."""
import json, os, threading, time
import unitree_sdk2py.core.channel as channel
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.g1.loco.g1_loco_api import ROBOT_API_ID_LOCO_GET_FSM_ID
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

def watchdog():
    time.sleep(8)
    print('READONLY_WATCHDOG_TIMEOUT',flush=True)
    os._exit(2)
threading.Thread(target=watchdog,daemon=True).start()
channel.ChannelConfigHasInterface=channel.ChannelConfigHasInterface.replace('name="$__IF_NAME__$"','address="$__IF_NAME__$"').replace('/tmp/cdds.LOG',os.path.abspath('fsm_raw_cdds.log').replace('\\','/'))
channel.ChannelFactoryInitialize(0,'<PRIVATE_IP>')
client=LocoClient();client.SetTimeout(.75);client.Init()
original=client._Call
raw=[]
def capture(api_id,parameter):
    code,data=original(api_id,parameter)
    raw.append({'service':'sport','api_id':api_id,'parameter':parameter,'status_code':code,'raw_data':data})
    return code,data
client._Call=capture
code,value=client.GetFsmId()
switcher=MotionSwitcherClient();switcher.SetTimeout(.75);switcher.Init()
mode_code,mode=switcher.CheckMode()
print(json.dumps({'reads':raw,'get_fsm_id_parsed':{'status_code':code,'value':value},'motion_switcher':{'status_code':mode_code,'result':mode},'motor_commands_sent':0}),flush=True)
os._exit(0)
