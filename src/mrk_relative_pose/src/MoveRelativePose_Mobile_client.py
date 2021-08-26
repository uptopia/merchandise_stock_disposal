#!/usr/bin/env python
'''
ROS Service CLIENT

'''

import sys
import rospy
from mrk_relative_pose.srv import aruco_relative_pose, aruco_relative_poseResponse

TIMDA_SERVER_relative_pose = '/Timda_mobile_relative_pose'
def pass_esp8266_info_to_server_relative_pose(x_length, y_length, theta):
    rospy.wait_for_service(TIMDA_SERVER_relative_pose)
    
    try:        
        req = rospy.ServiceProxy(TIMDA_SERVER_relative_pose, aruco_relative_pose)
        res = req(x_length, y_length, theta)
        return res
        
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s\n [move2relative place]:(x_length, y_length, theta)"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 4:
        x_length = float(sys.argv[1])
        y_length = float(sys.argv[2])
        theta = float(sys.argv[3])
        print(x_length, y_length, theta)
    else:
        print(usage())
        sys.exit(1)

    print("Client Requesting move to (x_length, y_length, theta) = %s, %s, %s"%(x_length, y_length, theta))
    server_response = pass_esp8266_info_to_server_relative_pose(x_length, y_length, theta)
    print('Request TIMDA mobile to move (x_length, y_length, theta) = {}, {}, {} relative to current pose,\
            \nTIMDA mobile moved to request place? \t{}'.format(x_length, y_length, theta, server_response.nav_done_res))
    
