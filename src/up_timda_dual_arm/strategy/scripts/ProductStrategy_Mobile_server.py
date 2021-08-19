#!/usr/bin/env python
'''
ROS Service SERVER
'''

import rospy
from timda_dual_arm.srv import TimdaMode_info, TimdaMode_infoResponse

TIMDA_SERVER = 'mobile_state'
class TIMDA_MobileState():
    def __init__(self):
        #initiate ros server
        self.server = rospy.Service(TIMDA_SERVER, TimdaMode_info, self.stream_mobile_state)
        print('TIMDA_MobileState initiated. wait for client request')

    def stream_mobile_state(self, req):
        
        # Client REQUEST
        print("Client Request to move to Place {}". format(req.item_req))
                         
        # Server RESPONSE
        res = TimdaMode_infoResponse()
        if req.item_req == 'Home':
            print('Home Reached')
            res.nav_res = 'finish' # response 'finish' when TIMDA mobile arrived at the navigation place
        
        elif req.item_req == 'Table1':
            print('Table1 Reached')
            res.nav_res = 'finish'

        elif req.item_req == 'Table2':
            print('Table2 Reached')
            res.nav_res = 'finish'
            
        elif req.item_req == 'Shelf':
            print('Shelf Reached')
            res.nav_res = 'finish'
        
        else:
            res.nav_res = 'ERROR Unknown navigation place'

        return res

if __name__ == '__main__':
    
    rospy.init_node('timda_mobile_state')    
    ms = TIMDA_MobileState()
    rospy.spin()

    del ms

