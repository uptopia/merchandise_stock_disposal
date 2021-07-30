#!/usr/bin/env python

import rospy
import numpy as np

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError

from aruco_detection.srv import aruco_info, aruco_infoResponse

class MarkerPosture():
    def __init__(self):        
        self.cam_left_topic = '/camera/color/image_raw'
        self.cam_info_left_topic = '/camera/color/camera_info'

        self.cam_right_topic = '/camera/color/image_raw'
        self.cam_info_right_topic = '/camera/color/camera_info'

        # Create vectors we'll be using for rotations and translations for postures
        self.ids = None
        self.corners = None
        self.rvecs = None 
        self.tvecs = None
            
        self.sub_camera_info_left = rospy.Subscriber(self.cam_info_left_topic, CameraInfo, self.get_camera_info)  
        self.sub_markers_left = rospy.Subscriber(self.cam_left_topic, Image, self.stream_img)          
        self.sub_camera_info_right = rospy.Subscriber(self.cam_info_right_topic, CameraInfo, self.get_camera_info)  
        self.sub_markers_right = rospy.Subscriber(self.cam_right_topic, Image, self.stream_img)

        #initiate ros server
        self.server = rospy.Service('get_ar_marker', aruco_info, self.stream_selected_camera)
        print('MarkerPosture initiated. wait for client request')

    def stream_selected_camera(self, req):
        #================#
        # Client REQUEST
        #================#
        print("Stream {} hand camera!". format(req.side_cmd))
        
        if req.side_cmd == 'left':            
            sub_camera_info_left = rospy.Subscriber(self.cam_info_left_topic, CameraInfo, self.get_camera_info)  
            sub_markers_left = rospy.Subscriber(self.cam_left_topic, Image, self.stream_img)          
            print('Streaming LEFT camera...')
        else:             
            sub_camera_info_right = rospy.Subscriber(self.cam_info_right_topic, CameraInfo, self.get_camera_info)  
            sub_markers_right = rospy.Subscriber(self.cam_right_topic, Image, self.stream_img)
            print('Streaming RIGHT camera...')
            
        #================#
        # Server RESPONSE
        #================#
        res = aruco_infoResponse()

        ids_list = []
        corners_list = []
        rvecs_list = []
        tvecs_list = []

        for idx, corner, rvec, tvec in zip(self.ids, self.corners, self.rvecs, self.tvecs):
            # print('ID: {};\nCorners:\n{}'.format(idx, corner))
            print('ID: {};'.format(idx))

            ids_list.append(int(idx))

            for angle in range(0, len(corner[0])):                
                corners_list.append(corner[0][angle][0]) # angle.pixel_x
                corners_list.append(corner[0][angle][1]) # angle.pixel_y
            
            rvecs_list.append(rvec[0][0])
            rvecs_list.append(rvec[0][1])
            rvecs_list.append(rvec[0][2])

            tvecs_list.append(tvec[0][0])
            tvecs_list.append(tvec[0][1])
            tvecs_list.append(tvec[0][2])

        res.ids = ids_list
        res.corners = corners_list
        res.rvecs = rvecs_list
        res.tvecs = tvecs_list

        # print("ids:", res.ids)
        # print("corners:", res.corners)
        # print("rvecs:", res.rvecs)
        # print("tvecs:", res.tvecs)

        return res

    def stream_img(self, data):        
        img_color, img_gray = self.data2cvimg(data)
        self.ids, self.corners, self.rvecs, self.tvecs = self.detect_aruco_markers(img_color, img_gray)

    def data2cvimg(self, data):        
        bridge = CvBridge()
        try:
            img_color = bridge.imgmsg_to_cv2(data, "bgr8")#data.encoding)  
            img_gray = cv2.cvtColor(img_color, cv2.COLOR_BGR2GRAY)          
        except CvBridgeError as e:
            print('CvBridgeError:', e)
            return
        return img_color, img_gray

    def get_camera_info(self, camera_info):
        self.height = camera_info.height
        self.width = camera_info.width
        self.frame_id = camera_info.header.frame_id        
        self.intrinsic_matrix = np.array(camera_info.K).reshape(3, 3)
        self.distortion_coeff = np.array(camera_info.D)        

    def detect_aruco_markers(self, img_color, img_gray):
        
        # Constant parameters used in Aruco methods
        markerLength = 0.019 #0.020
        axisLength = 0.010
        ARUCO_PARAMETERS = aruco.DetectorParameters_create()
        # ARUCO_PARAMETERS.adaptiveThreshConstant = 10
        ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL) #DICT_ARUCO_ORIGINAL, DICT_5X5_250      

        # Detect Aruco markers
        self.corners, self.ids, rejectedImgPoints = aruco.detectMarkers(img_gray, ARUCO_DICT, parameters = ARUCO_PARAMETERS)

        font = cv2.FONT_HERSHEY_SIMPLEX
        if self.ids is not None:           

            ### NOTICE: some Opencv version use 2 output "rvec, tvec", others use 3 output "rvec, tvec, _"!!!!
            self.rvecs, self.tvecs = aruco.estimatePoseSingleMarkers(self.corners, markerLength, self.intrinsic_matrix, self.distortion_coeff)
            # (rvec - tvec).any()  # get rid of that nasty numpy value array error

            # self.publish_aruco_corners_pose(ids, corners, rvecs, tvecs)
            
            # # Print corners and ids to the console
            # for i, corner in zip(ids, corners):
            #     print('ID: {};\nCorners:\n{}'.format(i, corner))          
       
            strg = ''
            for n in range(0, self.ids.size):
                strg += str(self.ids[n][0])+', '
                aruco.drawAxis(img_color, self.intrinsic_matrix, self.distortion_coeff, self.rvecs[n], self.tvecs[n], axisLength)
                # print(rvecs[n], tvecs[n])                

            # Outline all of the markers detected in our image
            aruco.drawDetectedMarkers(img_color, self.corners, self.ids) #, borderColor = (0, 0, 255))
            cv2.putText(img_color, "Id: " + strg, (10,30), font, 1, (0,0,255), 2, cv2.LINE_AA)

        else:
            # code to show 'No Ids' when no markers are found
            cv2.putText(img_color, "No Ids", (10,30), font, 1, (0,0,255), 2, cv2.LINE_AA)

        cv2.imshow('img_color', img_color)
        cv2.waitKey(1)

        return self.ids, self.corners, self.rvecs, self.tvecs    

if __name__ == '__main__':
    
    rospy.init_node('aruco')
    
    mp = MarkerPosture()

    rospy.spin()

    cv2.destroyAllWindows()
    del mp

