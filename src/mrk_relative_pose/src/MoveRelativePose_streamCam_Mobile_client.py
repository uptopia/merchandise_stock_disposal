#!/usr/bin/env python
'''
ROS Service CLIENT

'''
import cv2
import sys
import rospy
import numpy as np

from hand_eye.srv import eye2base, eye2baseResponse
from detect_aruco_pose_dual_cam.srv import aruco_info, aruco_infoResponse
from mrk_relative_pose.srv import aruco_relative_pose, aruco_relative_poseResponse



class MarkerRelativePose():
    def __init__(self):
        # Reference
        self.ref_id = 201
        # self.ref_x_length = 333.0
        # self.ref_y_length = 333.0
        # self.ref_theta    = 333.0

        self.ref_rvec = np.array([3.12173076, 0.01492018, 1.35288716])
        self.ref_tvec = np.array([-0.23255156, -0.09936203,  0.57744756])
        self.ref_cam_H_mrk = self.rvectvec2matrix(self.ref_rvec, self.ref_tvec)

        # self.ref_rvecInBase = []
        # self.ref_tvecInBase = []
        # self.ref_base_H_mrk = np.identity(4, dtype=float)

        # Current
        # self.curr_x_length = self.ref_x_length
        # self.curr_y_length = self.ref_y_length
        # self.curr_theta    = self.ref_theta
        self.curr_rvec = self.ref_rvec
        self.curr_tvec = self.ref_tvec
        self.curr_cam_H_mrk = np.identity(4, dtype=float)

        # self.curr_rvecInBase = self.ref_rvecInBase
        # self.curr_tvecInBase = self.ref_tvecInBase
        # self.curr_base_H_mrk = np.identity(4, dtype=float)

        # Relative Pose
        self.x_lengthInBase = 0.0
        self.y_lengthInBase = 0.0
        self.thetaInBase = 0.0

        self.TIMDA_SERVER_relative_pose = '/Timda_mobile_relative_pose'

    def rvectvec2matrix(self, rvec, tvec):        
        matrix = np.identity(4, dtype=float)

        rotation_matrix = cv2.Rodrigues(rvec)
        matrix[:3, :3] = rotation_matrix[0]
        for idx in range(len(tvec)):
            matrix[idx][3] = tvec[idx]
        # print("matrix =\n{}\n".format(matrix.round(7)))
        
        return matrix

    def get_ar_marker(self, side):
        rospy.wait_for_service('get_ar_marker')
        try:
            req = rospy.ServiceProxy('get_ar_marker', aruco_info)
            res = req(side)

            self.ids = np.array(res.ids)            
            self.corners = None
            self.rvecs = np.array(res.rvecs) 
            self.tvecs = np.array(res.tvecs)
            self.rvecs = self.rvecs.reshape(int(len(self.rvecs)/3), 3)
            self.tvecs = self.tvecs.reshape(int(len(self.tvecs)/3), 3)

            return res#, self.ids, self.corners, self.rvecs, self.tvecs
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def get_eye2hand_transformation(self, arm_side, ini_pose):  #TODO: ini_pose?????
        service_name = '/' + arm_side + '_arm/eye_trans2base' #/right_arm/eye_trans2base; /left_arm/eye_trans2base

        rospy.wait_for_service(service_name)        
        
        try:        
            req = rospy.ServiceProxy(service_name, eye2base)
            res_eye2hand = req(ini_pose)
        except rospy.ServiceException as e:
            print("eye_trans2base request failed: %s"%e)

        return res_eye2hand 

    def calculate_relative_pose(self, arm_side):
        res = self.get_ar_marker(arm_side)

        if(np.size(self.ids)==0):
            return [], [], [], [], []

        for rvec, tvec, id in zip(self.rvecs, self.tvecs, self.ids):
            if (id == self.ref_id):
                self.curr_rvec = rvec
                self.curr_tvec = tvec
                self.curr_cam_H_mrk = self.rvectvec2matrix(self.curr_rvec, self.curr_tvec)
                print('curr_rvec = ', rvec)
                print('curr_tvec = ', tvec)

                # markerInCam relative pose
                val_cam = self.ref_cam_H_mrk[0:3,3] - self.curr_cam_H_mrk[0:3,3]

                print('self.ref_cam_H_mrk:\n', self.ref_cam_H_mrk)             
                print('self.curr_cam_H_mrk:\n', self.curr_cam_H_mrk)                
                print('self.ref_cam_H_mrk(x,y,z):\n', self.ref_cam_H_mrk.reshape(4,4)[0:3,3])
                print('self.curr_cam_H_mrk(x,y,z):\n', self.curr_cam_H_mrk.reshape(4,4)[0:3,3])                
                print('val_cam', val_cam)

                # markerInBase relative pose
                self.curr_cam_H_mrk = self.curr_cam_H_mrk.reshape(-1)
                curr_base_H_mrk_res = self.get_eye2hand_transformation(arm_side, self.curr_cam_H_mrk)
                self.curr_base_H_mrk = curr_base_H_mrk_res.trans
                self.curr_base_H_mrk_pos = curr_base_H_mrk_res.pos

                self.ref_cam_H_mrk = self.ref_cam_H_mrk.reshape(-1)
                ref_base_H_mrk_res = self.get_eye2hand_transformation(arm_side, self.ref_cam_H_mrk)
                self.ref_base_H_mrk = ref_base_H_mrk_res.trans
                self.ref_base_H_mrk_pos = ref_base_H_mrk_res.pos

                val_base = np.array(self.ref_base_H_mrk_pos) - np.array(self.curr_base_H_mrk_pos)

                print('self.ref_base_H_mrk:\n', self.ref_base_H_mrk)
                print('self.curr_base_H_mrk:\n', self.curr_base_H_mrk)                
                print('self.ref_base_H_mrk(x,y,z):\n', self.ref_base_H_mrk_pos)
                print('self.curr_base_H_mrk(x,y,z):\n', self.curr_base_H_mrk_pos)
                print('val_base', val_base)

                # assign moving length
                self.x_lengthInBase = val_base[0]
                self.y_lengthInBase = val_base[1]
                # self.thetaInBase 

        return self.x_lengthInBase, self.y_lengthInBase
    
    # def pass_esp8266_info_to_server_relative_pose(self, x_length, y_length, theta):
    #     rospy.wait_for_service(self.TIMDA_SERVER_relative_pose)
        
    #     try:        
    #         req = rospy.ServiceProxy(TIMDA_SERVER_relative_pose, aruco_relative_pose)
    #         res = req(x_length, y_length, theta)
    #     except rospy.ServiceException as e:
    #         print("Service call failed: %s"%e)

    #     return res    

if __name__ == "__main__":
    mp = MarkerRelativePose()
    mp.calculate_relative_pose('right')
    # mp.calculate_relative_pose('left')