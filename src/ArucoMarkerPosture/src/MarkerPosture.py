#!/usr/bin/env python

import rospy, rospkg
import numpy as np

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

import ConfigParser

import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError

from ArucoMarkerPosture.srv import aruco_info, aruco_infoResponse


class MarkerPosture():
    def __init__(self):        
        self.cam_info_topic_left = '/cam_left/color/camera_info'
        self.cam_info_topic_right = '/cam_right/color/camera_info'
        self.cam_left_topic = '/cam_left/color/image_raw'       
        self.cam_right_topic = '/cam_right/color/image_raw'

        # Camera intrinsic parameters

        self.use_self_calib_intrinsic_param = False  #[True]: use calibrated data; [False]: use realsense data

        if(self.use_self_calib_intrinsic_param == True):
            # Read from calib data
            rospack = rospkg.RosPack()
            self.curr_path = rospack.get_path('ArucoMarkerPosture')            
            self.frame_id = 0
            self.width = 1920
            self.height = 1080
            self.camera_id_left = 3
            self.camera_id_right = 8
            self.camera_info_path_left = self.curr_path + '/config/camera_' + str(self.camera_id_left) + '_internal.ini'
            self.camera_info_path_right = self.curr_path + '/config/camera_' + str(self.camera_id_right) + '_internal.ini'
            print('====================')
            print('   MarkerPosture    ')
            print('====================')
            print('camera_info_path(left/right):\n{},\n{}'.format(self.camera_info_path_left, self.camera_info_path_right))
    
        else:
            # Use realsense data            
            self.camera_info_left = CameraInfo()
            self.camera_info_right = CameraInfo()
            self.sub_camera_info_left = rospy.Subscriber(self.cam_info_topic_left, CameraInfo, self.get_camera_info_left)        
            self.sub_camera_info_right = rospy.Subscriber(self.cam_info_topic_right, CameraInfo, self.get_camera_info_right)
            print('====================')
            print('   MarkerPosture    ')
            print('====================')
            print('camera_info_topic(left/right):\n{},\n{}'.format(self.cam_info_topic_left, self.cam_info_topic_right))
        
        self.cam_left_topic = '/cam_left/color/image_raw'       
        self.cam_right_topic = '/cam_right/color/image_raw'

        # Create vectors we'll be using for rotations and translations for postures
       
        self.img_color_left = []
        self.img_gray_left = []
        self.img_color_right = []
        self.img_gray_right = []

        self.ids = None
        self.corners = None
        self.rvecs = None 
        self.tvecs = None

        self.windowname_left = 'left cam view'
        self.windowname_right = 'right cam view'
        img_color = np.zeros((540, 960))


        # cv2.namedWindow('img_color', cv2.WINDOW_NORMAL)
        # cv2.imshow('img_color', img_color)
        # cv2.resizeWindow('img_color', 960, 540)
        # cv2.moveWindow('img_color', 100, 100)
        # cv2.waitKey()

        cv2.namedWindow(self.windowname_left, cv2.WINDOW_NORMAL)
        cv2.imshow(self.windowname_left, img_color)
        cv2.resizeWindow(self.windowname_left, 480, 270)
        cv2.moveWindow(self.windowname_left, 100, 100)       

        cv2.namedWindow(self.windowname_right, cv2.WINDOW_NORMAL)
        cv2.imshow(self.windowname_right, img_color)
        cv2.resizeWindow(self.windowname_right, 480, 270)
        cv2.moveWindow(self.windowname_right, 500, 100)
        cv2.waitKey()
        
        self.sub_markers_left = rospy.Subscriber(self.cam_left_topic, Image, self.stream_img_left)
        self.sub_markers_right = rospy.Subscriber(self.cam_right_topic, Image, self.stream_img_right)

        #initiate ros server
        self.server = rospy.Service('get_ar_marker', aruco_info, self.stream_selected_camera)
        print('MarkerPosture initiated. wait for client request')

    def stream_selected_camera(self, req):
        #================#
        # Client REQUEST
        #================#
        print("Stream {} hand camera!". format(req.side_cmd))
        
        if req.side_cmd == 'left':
            if(self.use_self_calib_intrinsic_param == True):
                self.intrinsic_matrix, self.distortion_coeff = self.get_camera_info_calib_data(self.camera_info_path_left, self.width, self.height)
            else:
                self.height, self.width, self.frame_id, self.intrinsic_matrix, self.distortion_coeff = self.get_camera_info(self.camera_info_left)          
            self.ids, self.corners, self.rvecs, self.tvecs = self.detect_aruco_markers(self.img_color_left, self.img_gray_left, self.windowname_left)            
            print('Streaming LEFT camera...')
            print('intrinsic_matrix, distortion_coeff: {},\n{}'.format(self.intrinsic_matrix, self.distortion_coeff))
        else:
            if(self.use_self_calib_intrinsic_param == True):
                self.intrinsic_matrix, self.distortion_coeff = self.get_camera_info_calib_data(self.camera_info_path_right, self.width, self.height)
            else:
                self.height, self.width, self.frame_id, self.intrinsic_matrix, self.distortion_coeff = self.get_camera_info(self.camera_info_right)
            self.ids, self.corners, self.rvecs, self.tvecs = self.detect_aruco_markers(self.img_color_right, self.img_gray_right, self.windowname_right)            
            print('Streaming RIGHT camera...')
            print('intrinsic_matrix, distortion_coeff: {},\n{}'.format(self.intrinsic_matrix, self.distortion_coeff))
            
        #================#
        # Server RESPONSE
        #================#
        res = aruco_infoResponse()

        ids_list = []
        corners_list = []
        rvecs_list = []
        tvecs_list = []
        
        if(self.ids is not None):
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

    def stream_img_left(self, data):        
        self.img_color_left, self.img_gray_left = self.data2cvimg(data)
    
    def stream_img_right(self, data):        
        self.img_color_right, self.img_gray_right = self.data2cvimg(data)

    def data2cvimg(self, data):        
        bridge = CvBridge()
        try:
            img_color = bridge.imgmsg_to_cv2(data, "bgr8")#data.encoding)  
            img_gray = cv2.cvtColor(img_color, cv2.COLOR_BGR2GRAY)          
        except CvBridgeError as e:
            print('CvBridgeError:', e)
            return
        return img_color, img_gray

    def get_camera_info_calib_data(self, camera_info_path, color_width, color_height):
        config = ConfigParser.ConfigParser()
        config.optionxform = str            
        config.read(camera_info_path)

        internal_name = 'Internal_' + str(color_width) + '_' + str(color_height)
        b00 = float(config.get(internal_name, "Key_1_1"))
        b01 = float(config.get(internal_name, "Key_1_2"))
        b02 = float(config.get(internal_name, "Key_1_3"))
        b10 = float(config.get(internal_name, "Key_2_1"))
        b11 = float(config.get(internal_name, "Key_2_2"))
        b12 = float(config.get(internal_name, "Key_2_3"))
        b20 = float(config.get(internal_name, "Key_3_1"))
        b21 = float(config.get(internal_name, "Key_3_2"))
        b22 = float(config.get(internal_name, "Key_3_3"))

        intrinsic_matrix = np.mat([[b00, b01, b02],
                                    [b10, b11, b12],
                                    [b20, b21, b22]])

        distcoeff_name = 'DistCoeffs_' + str(color_width) + '_' + str(color_height)
        k_1 = float(config.get(distcoeff_name, "K_1"))
        k_2 = float(config.get(distcoeff_name, "K_2"))
        k_3 = float(config.get(distcoeff_name, "K_3"))
        p_1 = float(config.get(distcoeff_name, "p_1"))
        p_2 = float(config.get(distcoeff_name, "p_2"))   

        distortion_coeff = np.array([k_1, k_2, p_1, p_2, k_3]) 
        # distortion_coeff = np.array([0.0, 0, 0, 0, 0])

        return intrinsic_matrix, distortion_coeff         

    def get_camera_info(self, camera_info):
        height = camera_info.height
        width = camera_info.width
        frame_id = camera_info.header.frame_id        
        intrinsic_matrix = np.array(camera_info.K).reshape(3, 3)
        distortion_coeff = np.array(camera_info.D)
        return height, width, frame_id, intrinsic_matrix, distortion_coeff
    
    def get_camera_info_left(self, camera_info):
        self.camera_info_left = camera_info
        # return self.camera_info_left

    def get_camera_info_right(self, camera_info):
        self.camera_info_right = camera_info
        # return self.camera_info_right   

    def detect_aruco_markers(self, img_color, img_gray, windowname):
        
        # Constant parameters used in Aruco methods
        markerLength = 0.018 #0.020
        axisLength = 0.010
        ARUCO_PARAMETERS = aruco.DetectorParameters_create()
        # ARUCO_PARAMETERS.adaptiveThreshConstant = 10
        ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_250) #DICT_ARUCO_ORIGINAL, DICT_5X5_250      

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
       
        # cv2.imshow('img_color', img_color)
        cv2.imshow(windowname, img_color)
        cv2.waitKey(1)
        print('!!!!!!!!!!!!!!!!!!!image updated')

        return self.ids, self.corners, self.rvecs, self.tvecs    

if __name__ == '__main__':
    
    rospy.init_node('aruco')
    
    mp = MarkerPosture()

    rospy.spin()

    cv2.destroyAllWindows()
    del mp

