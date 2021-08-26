#!/usr/bin/env python
'''
ROS Service SERVER
'''

import rospy
from mrk_relative_pose.srv import aruco_relative_pose, aruco_relative_poseResponse

TIMDA_SERVER_relative_pose = '/Timda_mobile_relative_pose'
class TIMDA_MobileMoveRelativePoseState():
    def __init__(self):
        #initiate ros server
        self.server = rospy.Service(TIMDA_SERVER_relative_pose, aruco_relative_pose, self.stream_mobile_move_relative_state)
        print('TIMDA_MobileMoveRelativePoseState initiated. wait for client request')

    def stream_mobile_move_relative_state(self, req):
        
        # Client REQUEST
        print("Client Request to move to (x_length, y_length, theta) =\
                 {}, {}, {} relative to current pose".format(req.x_length, req.y_length, req.theta))
                         
        # Server RESPONSE
        res = aruco_relative_poseResponse()
        res.nav_done_res = self.mobile_move(req.x_length, req.y_length, req.theta)
        print('res.nav_done = ', res.nav_done_res)
        return res
    
    def mobile_move(self, x_length, y_length, theta):

        '''
        one_unit_equals_actual_length = 0.005 #(m)
        x_unit = x_length/one_unit_equals_actual_length
        y_unit = y_length/one_unit_equals_actual_length
        '''

        finish = 'finish'
        return finish

if __name__ == '__main__':
    
    rospy.init_node('timda_mobile_move_relative_pose_state')    
    ms = TIMDA_MobileMoveRelativePoseState()
    rospy.spin()

    del ms

