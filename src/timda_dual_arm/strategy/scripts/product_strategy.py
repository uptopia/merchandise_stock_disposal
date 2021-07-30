#!/usr/bin/env python

import rospy
#import queue
import Queue as queue
# from Queue import Queue
import copy
import numpy as np
from enum import IntEnum
from math import radians, degrees, sin, cos, pi

from std_msgs.msg import Bool, Int32

from arm_control import DualArmTask
from arm_control import ArmTask, SuctionTask, Command, Status, RobotiqGripper

from obj_info import ObjInfo
from get_image_info import GetObjInfo

'''
Plum Riceball:      ABCD (10~, 20~, 30~, 40~)   (5 sides)
Salmon Riceball:    EFGH (50~, 60~, 70~, 80~)   (5 sides)
Sandwich:           IJK  (90~, 100~, 110~)      (5 sides)
Hamburger:          LMN  (120~, 130~, 140~)     (2 sides)
Drink:              OPQ  (150~, 160~, 170~)     (4 sides)??
Lunchbox:           RST  (180~, 190~, 200~)     (2 side)
'''
#camera_pose: pos, euler #, phi??
cam_pose = {'left' :[[[0.28,  0.2, 0.15],  [0.0, 65, 0.0]],     #shelf level 1
                    [[0.28,  0.2, -0.25],  [0.0, 65, 0.0]],     #shelf level 2
                    [[0.28,  0.2, -0.65],    [0.0, 65, 0.0]]],  #shelf level 3

          'right':[[[0.28, -0.2, 0.15],  [0.0, 65, 0.0]],       #shelf level 1
                    [[0.28, -0.2, -0.25],  [0.0, 65, 0.0]],     #shelf level 2
                    [[0.28, -0.2, -0.65],    [0.0, 65, 0.0]]],  #shelf level 3

          'left_indx' : 0,                                      #_index: current shelf level 
          'right_indx' : 0}

# place_pose = [[[-0.38,  0, -0.796],[0.0, 0.0, 0.0]],
#               [[-0.38,  0, -0.796],[0.0, 0.0, 0.0]],
#               [[-0.43,  0, -0.796],[0.0, 0.0, 0.0]],                             
#               [[-0.43,  0, -0.796],[0.0, 0.0, 0.0]],
#               [[-0.38,  0.02, -0.73],[0.0, 0.0, 0.0]],
#               [[-0.38,  -0.02, -0.73],[0.0, 0.0, 0.0]],
#               [[-0.43,  -0.02, -0.73],[0.0, 0.0, 0.0]],                             
#               [[-0.43,  0.02, -0.73],[0.0, 0.0, 0.0]],
#               [[-0.38,  0, -0.68],[0.0, 0.0, 0.0]],
#               [[-0.38,  0, -0.68],[0.0, 0.0, 0.0]],
#               [[-0.43,  0, -0.68],[0.0, 0.0, 0.0]],                             
#               [[-0.43,  0, -0.7],[0.0, 0.0, 0.0]],
#               [[-0.38,  0, -0.7],[0.0, 0.0, 0.0]],
#               [[-0.38,  0, -0.7],[0.0, 0.0, 0.0]],
#               [[-0.43,  0, -0.7],[0.0, 0.0, 0.0]],                             
#               [[-0.43,  0, -0.7],[0.0, 0.0, 0.0]]]

# obj_pose = [[[[0.465, -0.1, -0.18], [0, 90, 0]],
#             [[0.465,  0.1, -0.18], [0, 90, 0]]],
#             [[[0.545, -0.1, -0.43], [0, 90,  0]],
#             [[0.545,  0.1, -0.43], [0, 90,  0]]],
#             [[[0.6, -0.2, -0.883], [0, 90,  0]],
#             [[0.6,  0.2, -0.883], [0, 90, 0]]]]

class State(IntEnum):
    init            = 0 #(o)
    get_obj_inf     = 1 #(o)#take_pic + get_obj_info
    select_obj      = 2 #(?)
    move2obj        = 3
    check_pose      = 4
    pick            = 5
    place           = 6
    finish          = 7


class MerchandiseTask():
    def __init__(self, name_arm_ctrl, en_sim_arm_ctrl):
        # arm control
        self.name_arm_ctrl = name_arm_ctrl
        self.en_sim_arm_ctrl = en_sim_arm_ctrl
        self.dual_arm = DualArmTask(self.name_arm_ctrl, self.en_sim_arm_ctrl)

        # state machine
        self.state = State.init

        # object information
        self.camera = GetObjInfo()
        self.curr_merchandise_list = []
        
        # # self.place_pose_queue = queue.Queue()

        self.object_queue = queue.Queue()   #object detected from both camera
        
        self.left_tar_obj = queue.Queue()
        self.right_tar_obj = queue.Queue()

        # # self.left_retry_obj = queue.Queue()
        # # self.right_retry_obj = queue.Queue()

        self.target_obj_queue = {'left' : self.left_tar_obj, 'right' : self.right_tar_obj}
        self.target_obj = {'left': None, 'right': None}

        # # self.retry_obj_queue = {'left': self.left_retry_obj, 'right': self.right_retry_obj}
        
        # # self.obj_done = np.zeros((100), dtype=bool)
        # # self.obj_retry = np.zeros((100), dtype=bool)
        self.next_level = {'left': False, 'right': False}  #go_to_next_level?

    #     self.init()
            
    
    # def init(self):
    #     for pose in place_pose:
    #         self.place_pose_queue.put(pose)
        
    def get_obj_info(self, arm_side):
        fb = self.dual_arm.get_feedback(arm_side) # position(x,y,z), orientation(x,y,z,w), euler, orientation, phi
       
        ids, base_H_mrks, names, exps, side_ids = self.camera.new_get_obj_info(arm_side, fb.orientation)
        self.curr_merchandise_list = self.camera.get_merchandise_log()        
        self.camera.print_merchandise_log(self.curr_merchandise_list)
        
        if ids is None:
            return
        for id, base_H_mrk, name, exp, side_id in zip(ids, base_H_mrks, names, exps, side_ids):
            
            num = int(id/10)-1
            self.curr_merchandise_list[num]['id'] = id
            self.curr_merchandise_list[num]['side_id'] = side_id
            
            self.curr_merchandise_list[num]['pos'] = base_H_mrk[0:3, 3]
            self.curr_merchandise_list[num]['vector'] = base_H_mrk[0:3, 2]                  
            self.curr_merchandise_list[num]['sucang'], roll = self.dual_arm.suc2vector(base_H_mrk[0:3, 2], [0, 1.57, 0])
            self.curr_merchandise_list[num]['euler'] = [roll, 90, 0]      

            self.camera.print_merchandise_log(self.curr_merchandise_list)


            obj = ObjInfo()
            obj['id'] = id
            obj['side_id'] = side_id

            obj['name'] = name
            obj['expired'] = exp
            
            obj['pos'] = base_H_mrk[0:3, 3]            
            obj['vector'] = base_H_mrk[0:3, 2] #aruco_z_axis, rotation (coordinate_axis: aruco z axis)
            obj['sucang'], roll = self.dual_arm.suc2vector(base_H_mrk[0:3, 2], [0, 1.57, 0])#TODO:[0, 1.57, 0]) #suck ang(0-90), roll (7 axi)
            obj['euler']   = [roll, 90, 0]

            #Check Arm approach from [TOP] or [DOWN]: aruco_z_axis's [z value]
            if obj['vector'][2] >= -0.2:  #TODO aruco_z_axis's [z value]>=, >???
                #Arm approach from top => GOOD to go!!!                
                self.object_queue.put(obj)
                print('Good!!! object put in queue; name: {}, id: {}'.format(obj['name'], obj['id']))                
            else:                
                print('ERROR!!! Arm approach from downside => BAD, aruco_z_axis: {}'.format(obj['vector']))

    def check_pose(self, arm_side):
        self.target_obj[arm_side] = self.target_obj_queue[arm_side].get()

        fb = self.dual_arm.get_feedback(arm_side)        
        ids, base_H_mrks,  _, _, _ = self.camera.new_get_obj_info(arm_side, fb.orientation)

        if ids is None:
            return
        for id, base_H_mrk, in zip(ids, base_H_mrks):
            if id == self.target_obj[arm_side]['id']:
                self.target_obj[arm_side]['pos'] = base_H_mrk[0:3, 3] 
                if base_H_mrk[2, 2] > -0.1: #TODO: i dont know z value > -0.1????
                    self.target_obj[arm_side]['sucang'], roll = self.dual_arm.suc2vector(base_H_mrk[0:3, 2], [0, 1.57, 0])
                    self.target_obj[arm_side]['euler']   = [roll, 90, 0]
        pass

    def state_control(self, arm_state, arm_side):
 
        print('\nCURRENT: arm_state= {}, arm_side = {}'.format(arm_state, arm_side))
        if arm_state is None:
            arm_state = State.init

        elif arm_state == State.init:
            arm_state = State.get_obj_inf

        elif arm_state == State.get_obj_inf:
            arm_state = State.select_obj

        elif arm_state == State.select_obj:
            if self.object_queue.empty():
                print('camera at shelf level #{}'.format(cam_pose[arm_side+'_indx']))
                if cam_pose[arm_side+'_indx'] >= 3:                    
                    arm_state = State.finish
                else:                    
                    arm_state = State.get_obj_inf                    
            else:
                arm_state = State.move2obj                
        
        elif arm_state == State.move2obj:
            arm_state = State.check_pose

        elif arm_state == State.check_pose:
            arm_state = State.pick
        
        elif arm_state == State.pick:
            if arm_side == 'left':
                is_grip = self.dual_arm.left_arm.suction.is_grip
            else:
                is_grip = self.dual_arm.right_arm.suction.is_grip

            if is_grip:
                arm_state = State.place
            elif self.next_level[arm_side] == True:
                self.next_level[arm_side] = False
                if cam_pose[arm_side+'_indx'] >= 3:
                    arm_state = State.finish
                else:
                    arm_state = State.get_obj_inf   #move to next level shelf to take picture
            else:
                # if self.obj_retry[self.target_obj[arm_side]['id']] == False:
                #     self.retry_obj_queue[arm_side].put(self.target_obj[arm_side])
                # arm_state = State.move2obj
                arm_state = State.finish #tmp

        elif arm_state == State.place:
            if self.next_level[arm_side] == True:
                self.next_level[arm_side] = False
                if cam_pose[arm_side+'_indx'] >= 3:
                    arm_state = State.finish
                else:
                    arm_state = State.get_obj_inf   #move to next level shelf to take picture
            else:
                arm_state = State.move2obj

        elif arm_state == State.finish:
            arm_state = None
        
        print('SWITCHED to: arm_state= {}, arm_side = {}'.format(arm_state, arm_side))
        return arm_state

    def strategy(self, arm_state, arm_side):

        print("--->execute strategy: ", arm_state)
        cmd = Command()
        cmd_queue = queue.Queue()

        if arm_state == State.init:
            print(' ++++++++++ init ++++++++++ ', arm_side)
            cmd['cmd'] = 'jointMove'            
            cmd['jpos'] = [0, 0, -1.2, 0, 1.87, 0, -0.87, 0]#[0, 0, 0, 0, 0, 0, 0, 0]#[-50, 0, -1.2, 0, 1.87, 0, -0.87, 0] #[0, 0, 0, 0, 0, 0, 0, 0]
            cmd['state'] = State.init
            cmd['speed'] = 100
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(arm_side, True, cmd_queue)
            print('left_arm.status:', self.dual_arm.left_arm.status)
            print('right_arm.status:',self.dual_arm.right_arm.status)

        elif arm_state == State.get_obj_inf:
            print(' ++++++++++ get_obj_inf ++++++++++ ', arm_side)            
            print('camera_pose: \n\tarm_side = {}; \n\tpos, euler, phi = {}, {}, {}'.format( 
                arm_side+'_indx', 
                cam_pose[arm_side][cam_pose[arm_side+'_indx']][0],
                cam_pose[arm_side][cam_pose[arm_side+'_indx']][1],
                0))
            
            cmd['cmd'], cmd['mode'] = 'ikMove', 'p2p'
            cmd['pos'], cmd['euler'], cmd['phi'] = cam_pose[arm_side][cam_pose[arm_side+'_indx']][0], cam_pose[arm_side][cam_pose[arm_side+'_indx']][1], 0
            cmd_queue.put(copy.deepcopy(cmd))
            
            cmd['cmd'] = 'occupied'
            cmd['state'] = State.get_obj_inf
            cmd_queue.put(copy.deepcopy(cmd))
            arm_side = self.dual_arm.send_cmd(arm_side, False, cmd_queue)

            if arm_side != 'fail':
                cam_pose[arm_side+'_indx'] += 1         #TODO: 1 shelf level only take picture one time!!!!! NO!!!!
                print('arm_side move to take picture pose SUCCEED')
            else:
                print('arm_side move to take picture pose FAILED')
            print('left_arm.status:', self.dual_arm.left_arm.status)
            print('right_arm.status:',self.dual_arm.right_arm.status)

        elif arm_state == State.select_obj:
            print(' ++++++++++ select_obj ++++++++++ ', arm_side)
            self.get_obj_info(arm_side)
            # self.arrange_obj(side)

            cmd['cmd'] = None
            cmd['state'] = State.select_obj
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(arm_side, True, cmd_queue)
            print('left_arm.status:', self.dual_arm.left_arm.status)
            print('right_arm.status:',self.dual_arm.right_arm.status)

        elif arm_state == State.move2obj:
            print(' ++++++++++ move2obj ++++++++++ ', arm_side)
            obj = None
            if self.object_queue.empty():
                self.next_level[arm_side] = True
                print('object_queue.empty()= True, no merchandise, Move to next level of shelf')
                return
            else:
                #TODO:check object_queue if fail grasp, do what??? target_obj_queue, obj_done

                # for _ in range(self.object_queue.qsize()):
                obj = ObjInfo()
                obj['name'] = 'ERROR'
                while(obj['name']=='ERROR'):
                    obj = self.object_queue.get() #remove 'ERROR' (aruco ID detected, but it's not in the predefined ID)
                    
                print(obj['name'], obj['id'], obj['side_id'], obj['expired'])
     
                pos = copy.deepcopy(obj['pos']) #(x, y, z): base_H_mrks[0:3, 3]                
                # pos[1] += 0.032
                # pos[2] += 0.065
                cmd['suc_cmd'] = 'Off'
                cmd['cmd'], cmd['mode'] = 'ikMove', 'p2p'
                cmd['state'] = State.move2obj
                cmd['pos'], cmd['euler'], cmd['phi'] = pos, [0, 90, 0], 0 #[0.4, pos[1], pos[2]], [0, 90, 0], 0
                cmd_queue.put(copy.deepcopy(cmd))

                cmd['cmd'] = 'occupied'
                cmd_queue.put(copy.deepcopy(cmd))
                arm_side = self.dual_arm.send_cmd(arm_side, False, cmd_queue)

                if arm_side == 'fail': #TODO: failed because of what???? cannot reach???
                    self.object_queue.put(obj)  #bcus failed so put the obj back to queue
                    # self.obj_done[obj['id']] = False
                    print('move2obj FAILED!!!!!!! arm_side = {}, name = {}, id = {}'.format(arm_side, obj['name'], obj['id']))
                else:
                    self.target_obj_queue[arm_side].put(obj)
                    print('move2obj OK! arm_side = {}, name = {}, id = {}'.format(arm_side, obj['name'], obj['id']))
            print('left_arm.status:', self.dual_arm.left_arm.status)
            print('right_arm.status:', self.dual_arm.right_arm.status)

        elif arm_state == State.check_pose:
            print(' ++++++++++ check_pose ++++++++++ ', arm_side)
            self.check_pose(arm_side)

            cmd['cmd'], cmd['state'] = 'occupied', State.check_pose
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(arm_side, True, cmd_queue)
            print('left_arm.status:', self.dual_arm.left_arm.status)
            print('right_arm.status:', self.dual_arm.right_arm.status)

        elif arm_state == State.pick:
            print(' ++++++++++ pick ++++++++++ ', arm_side)
            obj = copy.deepcopy(self.target_obj[arm_side])
            if obj['vector'][2] > 0.7:
                obj['pos'][0] -= 0.02
               # obj['pos'][2] += 0.05

            cmd['state'] = State.pick
            cmd['cmd'], cmd['mode'] = 'fromtNoaTarget', 'line'          #????
            cmd['pos'], cmd['euler'], cmd['phi'] = obj['pos'], obj['euler'], 0
            cmd['suc_cmd'], cmd['noa'] = obj['sucang'], [0, 0, -0.03]
            cmd_queue.put(copy.deepcopy(cmd))

            cmd['cmd'], cmd['mode'], cmd['noa'] = 'grasping', 'line', [0, 0, 0.05]  #???/
            cmd['suc_cmd'], cmd['speed'] = 'On', 15
            if obj['vector'][2] < 0.2:
                cmd['speed'] = 30
            cmd_queue.put(copy.deepcopy(cmd))

            cmd['cmd'], cmd['mode'],  = 'relativePos', 'line'   #
            cmd['speed'], cmd['suc_cmd'] = 40, 'calibration'    #????
            cmd['pos'] = [0, 0, 0.03]
            cmd_queue.put(copy.deepcopy(cmd))

            cmd['cmd'], cmd['mode'] = 'ikMove', 'line'
            cmd['pos'], cmd['euler'], cmd['phi'] = [0.45, obj['pos'][1], obj['pos'][2]+0.08], obj['euler'], 0
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(arm_side, True, cmd_queue)
            print('left_arm.status:', self.dual_arm.left_arm.status)
            print('right_arm.status:', self.dual_arm.right_arm.status)

        elif arm_state == State.place:
            print(' ++++++++++ place ++++++++++ ', arm_side)
            cmd['state'] = State.place
            cmd['cmd'] = 'jointMove'
            cmd['jpos'] = [0, 0, -1.5, 0, 2.07, 0, -0.57, 0]
            cmd_queue.put(copy.deepcopy(cmd))

            pose = self.place_pose_queue.get()
            pos, euler = pose[0], pose[1]
            if arm_side == 'left':
                pos[1] += 0.12
            else:
                pos[1] -= 0.12
            cmd['cmd'], cmd['mode'] = 'fromtNoaTarget', 'line'
            cmd['pos'], cmd['euler'], cmd['phi'] = pos, euler, 0
            cmd['suc_cmd'], cmd['noa'] = 0, [0, 0, -0.2]
            cmd_queue.put(copy.deepcopy(cmd))

            cmd['cmd'], cmd['mode'], cmd['noa'] = 'noaMove', 'line', [0, 0, 0.2]
            cmd_queue.put(copy.deepcopy(cmd))

            cmd['cmd'], cmd['mode'], cmd['noa'] = 'noaMove', 'line', [0, 0, -0.2]
            cmd['suc_cmd'] = 'Off'
            cmd_queue.put(copy.deepcopy(cmd))

            cmd['cmd'] = 'jointMove'
            cmd['jpos'] = [0, 0, -1.8, 0, 2.57, 0, -0.87, 0]
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(arm_side, True, cmd_queue)
            print('left_arm.status:', self.dual_arm.left_arm.status)
            print('right_arm.status:', self.dual_arm.right_arm.status)

        elif arm_state == State.finish:
            print(' ++++++++++ finish ++++++++++ ', arm_side)
            cmd['suc_cmd'] = 'Off'
            cmd['cmd'] = 'jointMove'
            cmd['jpos'] = [0, 0, -1, 0, 1.57, 0, -0.57, 0]
            cmd['state'] = State.finish
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(arm_side, False, cmd_queue)
            print('left_arm.status:', self.dual_arm.left_arm.status)
            print('right_arm.status:', self.dual_arm.right_arm.status)

        return arm_side

    def process(self):
            #assign task to LEFT of RIGHT arm
            # (1) check left/right arm status
            # (2) state_control: switch arm state
            # (3) strategy: perform arm actions

            rate = rospy.Rate(10)
            rospy.on_shutdown(self.dual_arm.shutdown)
            while True:
                l_status = self.dual_arm.left_arm.status
                if l_status == Status.idle or l_status == Status.occupied:
                    l_state = self.state_control(self.dual_arm.left_arm.state, 'left')
                    self.strategy(l_state, 'left')
                rate.sleep()
                
                r_status = self.dual_arm.right_arm.status
                if r_status == Status.idle or r_status == Status.occupied:
                    r_state = self.state_control(self.dual_arm.right_arm.state, 'right')
                    self.strategy(r_state, 'right')
                rate.sleep()

                if l_state is None and r_state is None:
                    if l_status == Status.idle and r_status == Status.idle:
                        return

if __name__ == '__main__':
    rospy.init_node('merchandize')

    strategy = MerchandiseTask('dual_arm', True)
    rospy.on_shutdown(strategy.dual_arm.shutdown)
    strategy.process()
    strategy.dual_arm.shutdown()
    del strategy.dual_arm
