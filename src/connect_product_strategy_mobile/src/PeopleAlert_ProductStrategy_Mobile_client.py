#!/usr/bin/env python
'''
ROS Service CLIENT

'''

import sys
import rospy
from connect_product_strategy_mobile.srv import TimdaMode, TimdaModeResponse
from connect_product_strategy_mobile.msg import depth_alert

class ArmMobileTimdaConnection:
    def __init__(self):
        self.TIMDA_SERVER = '/Timda_mobile'

        self.sub_alert_level = rospy.Subscriber('alert_level', depth_alert, self.timda_reaction)        

    def pass_esp8266_info_to_server(self, data):
        rospy.wait_for_service(self.TIMDA_SERVER)
        
        try:        
            req = rospy.ServiceProxy(self.TIMDA_SERVER, TimdaMode)
            res = req(data)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        return res  

    def timda_reaction(self, depth_alert):
        if(depth_alert == 1):
            # person_detected < 1 m
            # arm: STOP; mobile: move away
            print('depth_alert: [level 1] person_detected < 1 m' )

        elif(depth_alert == 2):
            # (person_detected < 1 m) & (time_last > 3 sec)
            # arm: STOP; mobile: X
            print('depth_alert: [level 2] (person_detected < 1 m) & (time_last > 3 sec)')
            Shelf_back_done = self.pass_esp8266_info_to_server('shelf_back')  #('Shelf_left', 'Shelf_right')            
            print('mobile arrive [Shelf_back] done ? {}'.format(Shelf_back_done))

        elif(depth_alert == 0):
            # (no people) or (person_detected > 1 m)
            # arm: X; mobile: X
            print('depth_alert: [level 0] person_detected ')        
    
def usage():
    return "%s [move2place_cmd]:initial, Table1, Table2, Shelf_back, Shelf_left, Shelf_right"%sys.argv[0]

if __name__ == "__main__":

    rospy.init_node("arm_connect_mobile")

    try:
        amtc = ArmMobileTimdaConnection()
    except rospy.ROSInterruptException:
        pass

    print("Arm requesting move to [initial]")
    initial_done = amtc.pass_esp8266_info_to_server('initial')
    print('mobile arrive [initial] done ? {}'.format(initial_done))

    print("Arm requesting move to [Shelf]")
    shelf_done = amtc.pass_esp8266_info_to_server('shelf')
    print('mobile arrive [Shelf] done ? {}'.format(shelf_done))

    amtc.timda_reaction(0)
    amtc.timda_reaction(1)
    amtc.timda_reaction(2)

    shelf_done = amtc.pass_esp8266_info_to_server('shelf')
    print('mobile arrive [Shelf] done ? {}'.format(shelf_done))

    print("Arm requesting move to [initial]")
    initial_done = amtc.pass_esp8266_info_to_server('initial')
    print('mobile arrive [initial] done ? {}'.format(initial_done))

    rospy.spin()   
    
