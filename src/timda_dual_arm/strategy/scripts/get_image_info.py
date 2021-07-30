#!/usr/bin/env python3

import os
import rospy
import numpy as np
import cv2
from math import sin, cos, radians

from strategy.srv import aruco_info, aruco_infoResponse

class ObjInfo():    

    def __init__(self, id, name, letter, state):         
        self.id = id            # 10 ~ 200
        self.name = name        # 'plum_riceball', 'salmon_riceball', 'sandwich', 'burger', 'drink', 'lunch_box'
        self.letter = letter    # ABCD, EFGH, IJK, LMN, OPQ, RST
        self.state = state      # 'new', 'old', 'expired'
        
        #changable due to object pose                
        self.side_id = ''       # 'front', 'back', 'left_side', 'right_side', 'bottom'
        self.pose = None
        self.euler = None
        self.cam_H_mrk = None
        self.suc_ang = None

        # self['expired']
        # self['vector']
 
class GetObjInfo():
    def __init__(self):
        self.folder = '/home/upup/ws_dual_wrs_up/src/timda_dual_arm/strategy/scripts/'
        self.left_cam_H_flange = self.load_transformation(self.folder + 'left_cam2flange.txt')
        self.right_cam_H_flange = self.load_transformation(self.folder + 'right_cam2flange.txt')

        self.ids = None
        self.corners = None
        self.rvecs = None 
        self.tvecs = None

        '''
        Plum Riceball:      ABCD (10~, 20~, 30~, 40~)   (5 sides)
        Salmon Riceball:    EFGH (50~, 60~, 70~, 80~)   (5 sides)
        Sandwich:           IJK  (90~, 100~, 110~)      (5 sides)
        Hamburger:          LMN  (120~, 130~, 140~)     (2 sides)
        Drink:              OPQ  (150~, 160~, 170~)     (4 sides)??
        Lunchbox:           RST  (180~, 190~, 200~)     (2 side)
        '''
        self.merchandise_side_id_name = ['front', 'back', 'left_side', 'right_side', 'bottom', 'side_id5', 'side_id6', 'side_id7', 'side_id8', 'side_id9']

        self.merchandise_letters_expired = ['A', 'G', 'J', 'L', 'O', 'T']   #(6) ABCD, EFGH, IJK, LMN, OPQ, RST
        self.merchandise_letters_new = ['B', 'C', 'E', 'F', 'H', 'P', 'R']  #(8) ABCD, EFGH, IJK, LMN, OPQ, RST
        self.check_duplicate()

        self.merchandise_types = ['plum_riceball', 'salmon_riceball', 'sandwich', 'burger', 'drink', 'lunch_box']
        self.merchandise_quantity = [4, 4, 3, 3, 3, 3]
        self.merchandise_status = ['new', 'old', 'expired']        
        self.merchandise_list = []
        # self.merchandise_list = self.generate_merchandise_log()
        self.generate_merchandise_log()

    def load_transformation(self, path):

        cam_H_flange = np.identity(4, dtype=float)
        
        if not os.path.exists(path):
            print("File {} does not exist!!! Use 4*4 IDENTITY matrix instead.".format(path))
        else:
            print("Load cam2flange from file: {}".format(path))
            
            file = open(path, 'rb') #'rb', 'r'
            lines = file.readlines()        
            
            #assign value to 4*4 matrix
            num = 0
            for line in lines:
                line = line.strip()         #remove '\n'              
                values = line.split(', ')   #split values by ', '                        
                
                for idx in range(len(values)):
                    cam_H_flange[num][idx] = float(values[idx])
        
                num = num + 1            

            file.close()
        
        print('cam_H_flange =\n{}\n'.format(cam_H_flange))

        return cam_H_flange

    def check_duplicate(self):
        all_letters = self.merchandise_letters_expired + self.merchandise_letters_new
        
        unique_letter = []
        duplicate_letter = []        
        for letter in all_letters:
            if letter not in unique_letter:
                unique_letter.append(letter)
            else:
                duplicate_letter.append(letter)                
        
        if len(duplicate_letter)>0:
            print("ERRRROOOOOOOOOOOOOORRRRR! merchandise letters assigned wrong with duplications!!")            
            print('duplicate_letter = ', duplicate_letter)
            exit()

    def generate_merchandise_log(self):

        total_quantity = 0
        for num in self.merchandise_quantity:
            total_quantity = total_quantity + num

        id = 1
        for type_idx in range(len(self.merchandise_types)):

            for num in range(self.merchandise_quantity[type_idx]):
                aruco_id = id*10
                letter = chr(64 + id)
                if letter in self.merchandise_letters_expired:
                    status = self.merchandise_status[2]  #'expired'
                elif letter in self.merchandise_letters_new:
                    status = self.merchandise_status[0]  #'new'
                else:
                    status = self.merchandise_status[1]  #'old'

                self.merchandise_list.append(ObjInfo(aruco_id, self.merchandise_types[type_idx], letter, status))

                id +=1

        self.print_merchandise_log(self.merchandise_list)

    def print_merchandise_log(self, merchandise_list):
        print('\n*=====================================*')
        print('{:^10}  {:^20}  {:^8} {:^10} {:^12}'.format('aruco_id', 'merchandise name', 'letter', 'state', 'side_id')) #, pose, euler, cam_H_mrk, suc_ang:')
        for obj in merchandise_list:
            print('{:^10}  {:^20}  {:^8} {:^10} {:^12}'.format(obj.id, obj.name, obj.letter, obj.state, obj.side_id)) #, obj.pose, obj.euler, obj.cam_H_mrk, obj.suc_ang)
        print('*=====================================*\n')

    def get_merchandise_log(self):
        return self.merchandise_list

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

    def new_get_obj_info(self, side, base_H_flange_list):        

        res = self.get_ar_marker(side)

        if side == 'left':
            cam_H_flange = self.left_cam_H_flange
        else:
            cam_H_flange = self.right_cam_H_flange
        flange_H_cam = np.linalg.inv(cam_H_flange)          #TODO: need to check!!!!         
        base_H_flange = np.array(base_H_flange_list).reshape(4,4)        

        base_H_mrks, names, exps, side_ids = [], [], [], []     #obj_mat
        if(np.size(self.ids)==0):
            return [], [], [], [], []

        for rvec, tvec, id in zip(self.rvecs, self.tvecs, self.ids):            
            #get_obj_name: 'name', 'expiration status', 'side_id'
            name, exp, side_id = self.get_obj_name(id)
            if(name != 'ERROR'):
                names = np.append(names, name)
                exps = np.append(exps, exp)
                side_ids = np.append(side_ids, side_id)

                #transformation
                cam_H_mrk = self.rvectvec2matrix(rvec, tvec)
                base_H_mrk = self.calculate_mrk2base(cam_H_mrk, flange_H_cam, base_H_flange) #np.mat
                base_H_mrks = np.append(base_H_mrks, base_H_mrk) #np.mat

                print('id, id/10', id, int(id/10))
                self.merchandise_list[int(id/10)].id = id
                self.merchandise_list[int(id/10)].pose = base_H_mrk[0:3, 3]
                self.merchandise_list[int(id/10)].euler = base_H_mrk[0:3, 2]
                self.merchandise_list[int(id/10)].cam_H_mrk = cam_H_mrk

        base_H_mrks = base_H_mrks.reshape(int(len(base_H_mrks)/16), 4, 4)
        # self.filter_obj(self.ids, base_H_mrks, names, exps, side_ids)        
        # self.print_merchandise_log(self.merchandise_list)       

        return self.ids, base_H_mrks, names, exps, side_ids

    def filter_out_same_obj(self, ids, base_H_mrks, names, status):
        pass

    #make sure same object
    def filter_obj(self, ids, obj_mat, names, exps, side_ids):
        for i in range(len(ids)-1):
            if names[i] == names[i+1] and exps[i] == exps[i+1]:
                if np.linalg.norm(np.subtract(obj_mat[i, 0:3, 3], obj_mat[i+1, 0:3, 3])) < 0.05:
                    if side_ids[i] == 0:
                        ids[i+1] = -1
                    elif side_ids[i+1] == 0:
                        ids[i] = -1
                    elif side_ids[i] == 1:
                        ids[i] = -1
                    elif side_ids[i+1] == 1:
                        ids[i+1] = -1
                    else:
                        ids[i] = -1
                    if ids[i+1] == -1:
                        ids[i], ids[i+1] = ids[i+1], ids[i]
                        names[i], names[i+1] = names[i+1], names[i]
                        exps[i], exps[i+1] = exps[i+1], exps[i]
                        side_ids[i], side_ids[i+1] = side_ids[i+1], side_ids[i]
                        obj_mat[i], obj_mat[i+1] = obj_mat[i+1], obj_mat[i]
    
    def convert_rvec_matrix(self, rot_form1):
        
        # rvec <-> 3*3 matrix
        rot_form2 = cv2.Rodrigues(rot_form1) #rot_form2[0]: rvec, matrix; rot_form2[1]: jacobian
             
        return rot_form2[0]

    def rvectvec2matrix(self, rvec, tvec):
        
        matrix = np.identity(4, dtype=float)

        rotation_matrix = cv2.Rodrigues(rvec)
        matrix[:3, :3] = rotation_matrix[0]
        for idx in range(len(tvec)):
            matrix[idx][3] = tvec[idx]
        # print("matrix =\n{}\n".format(matrix.round(7)))
        
        return matrix

    def get_obj_name(self, id):        
        
        tot = len(self.merchandise_list)
        
        first_obj_id_min = self.merchandise_list[0].id 
        last_obj_id_min = self.merchandise_list[tot-1].id
        
        if((id >= last_obj_id_min + 10) or (id < first_obj_id_min + 10)): #outside predefined aruco id            
            # print('ERRRROR!!! id, name, exp_state, side_id: {}, {}, {}, {}'.format(id, 'ERROR', 'expired', 'ERROR'))
            return 'ERROR', 'expired', id
        
        for num in range(0, tot-1): #TODO:check
            print('num', num)
            obj_id_min = self.merchandise_list[num].id
            obj_id_max = obj_id_min + 10            
            
            if ((id >= obj_id_min) and (id < obj_id_max)):
                self.merchandise_list[num].side_id = self.merchandise_side_id_name[id % 10] #self.merchandise_side_id_name[int(id - obj_id_min)]
                # print('id, name, exp_state, side_id: {}, {}, {}, {}'.format( \
                #     id, self.merchandise_list[num].name, self.merchandise_list[num].state, self.merchandise_list[num].side_id))
                return self.merchandise_list[num].name, self.merchandise_list[num].state, self.merchandise_list[num].side_id


    def calculate_mrk2base(self, cam_H_mrk, flange_H_cam, base_H_flange):  #visiontoArm

          # base_H_flange*flange_H_cam*cam_H_mrk
        base_H_mrk = np.dot(base_H_flange, np.dot(flange_H_cam, cam_H_mrk)) 

        # # base_H_mrk = base_H_flange*flange_H_tool*tool_H_cam*cam_H_mrk (correct!!!)
        # base_H_mrk = np.dot(base_H_flange, np.dot(flange_H_tool, np.dot(tool_H_cam, cam_H_mrk)))

        return base_H_mrk


if __name__ == '__main__':

    mp = GetObjInfo()
    # obj_list = mp.generate_merchandise_log()
    mp.get_obj_name(32)
    mp.get_obj_name(68)
    mp.get_obj_name(77)
    mp.get_obj_name(128)
    mp.get_obj_name(205)
    mp.get_obj_name(300)
    # mp.generate_merchandise_log()
    # print(mp.merchandise_list[0])
    # print(mp.merchandise_list[0].name)
    # print(mp.merchandise_list[5])
    # print(mp.merchandise_list[5].state)
    
    # mp.rodrigues2matrix(mp.left_cam_H_flange[:3, :3])
    # mp.rodrigues2matrix(np.array([0.47443521, 1.0222658, 0.2739153]))
    # mp.rvec_tvec2matrix(np.array([0.47443521, 1.0222658, 0.2739153]), np.array([0.005, 0.010, 0.015]))
    # x=np.array([1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16])
    # x = x.reshape(4,4)
    # print(x)       
    
    # mp.set
    # mp.match_expired_letter_to_aruco_id(expired_letters)
    
    