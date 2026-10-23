import json,os
from pathlib import Path
import unitree_sdk2py.core.channel as channel
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
channel.ChannelConfigHasInterface=channel.ChannelConfigHasInterface.replace('name="$__IF_NAME__$"','address="$__IF_NAME__$"').replace('/tmp/cdds.LOG',str(Path('arm_capabilities_cdds.log').resolve()).replace('\\','/'))
channel.ChannelFactoryInitialize(0,'<PRIVATE_IP>')
client=G1ArmActionClient();client.SetTimeout(.5);client.Init()
code,result=client.GetActionList()
print(json.dumps({'GetActionList_code':code,'result':result,'motor_commands_sent':0}),flush=True)
os._exit(0)
