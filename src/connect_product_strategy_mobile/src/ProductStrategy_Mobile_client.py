#!/usr/bin/env python
'''
ROS Service CLIENT

'''

import sys
import rospy
from connect_product_strategy_mobile.srv import TimdaMode, TimdaModeResponse

TIMDA_SERVER = 'mobile_state'
def pass_esp8266_info_to_server(data):
    rospy.wait_for_service(TIMDA_SERVER)
    
    try:        
        req = rospy.ServiceProxy(TIMDA_SERVER, TimdaMode)
        res = req(data)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    return res    

def usage():
    return "%s [move2place_cmd]:Home, Table1, Table2, Shelf"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        move2place_cmd = sys.argv[1]
    else:
        print(usage())
        sys.exit(1)

    print("Client Requesting move to %s"%(move2place_cmd))
    server_response = pass_esp8266_info_to_server(move2place_cmd)
    print('Request TIMDA mobile to move to %s,\nTIMDA mobile moved to request place? \t%s'%(move2place_cmd, server_response.nav_res))
    
