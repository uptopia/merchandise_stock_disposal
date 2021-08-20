#!/usr/bin/env python

import sys
import rospy
from ArucoMarkerPosture.srv import aruco_info, aruco_infoResponse

def get_ar_marker(side):
    rospy.wait_for_service('get_ar_marker')
    
    try:        
        req = rospy.ServiceProxy('get_ar_marker', aruco_info)
        res = req(side)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    return res    

def usage():
    return "%s [dual_arm_cmd]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        dual_arm_cmd = sys.argv[1]
    else:
        print(usage())
        sys.exit(1)

    print("Client Requesting %s"%(dual_arm_cmd))
    server_response = get_ar_marker(dual_arm_cmd)
    print("%s Hand Camera information:\nArUco ID:%s\n"%(dual_arm_cmd, server_response.ids))
    # print("ArUco corners:{}\n".format(server_response.corners))
    # print("ArUco rvecs:{}\n".format(server_response.rvecs))
    # print("ArUco tvecs:{}\n".format(server_response.tvecs))