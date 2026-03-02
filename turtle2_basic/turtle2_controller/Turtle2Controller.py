# Copyright 2026 The RLinf Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import signal
from turtle2_basic.turtle2_controller.kinematics import kinematics
from turtle2_basic.turtle2_controller.controllers import HeadController,ChassisController,LiftController,ArmsController,MovingHeadController,MovingChassisController,MovingLiftController
from turtle2_basic.turtle2_controller.sensors import Camera
import time

class RobotController:
    def __init__(self):
        pass

    def head_control(self, cmd):
        pass
    def head_data(self):
        pass
    def lift_control(self, cmd):
        pass
    def lift_data(self):
        pass
    def chassis_control(self, cmd):
        pass
    def chassis_data(self):
        pass
    def get_infer_flag(self):
        infer_flag=rospy.get_param("/master_teach_mode",0)
        return (infer_flag==0)

 
class Turtle2Controller(RobotController):
    def __init__(self,init_node=False):
        if init_node:
            rospy.init_node('turtle2_controller')
        self.data_yaw = 0
        self.data_pitch = 0
        self.kinematics =  kinematics()
        self.head = HeadController()
        self.lift = LiftController()
        self.arms = ArmsController()
        self.chassis = ChassisController()
        self.cam = Camera()
        rospy.Rate(2).sleep()

        self.virtual_zero_tf_inv = None

    def head_control(self, cmd):
        self.head.send_control_pitch(cmd[0])
        self.head.send_control_yaw(cmd[1])
        
    def head_control_pitch(self, pitch : float):
        self.head.send_control_pitch(pitch)
    
    def head_control_yaw(self, yaw : float):
        self.head.send_control_yaw(yaw)

    def head_data(self):
        return self.head.get_data()

    def lift_control(self,cmd):
        self.lift.send_control(cmd)

    def lift_data(self):
        return self.lift.get_data()
    
    def arms_start(self):
        self.arms.start()

    def arms_stop(self):
        self.arms.stop()
    
    def arms_control(self,cmd_l,cmd_r):
        self.arms.send_control(cmd_l,cmd_r)

    def arms_control_pose_trj(self, is_async_: bool, pose_trj_l, pose_trj_r, pos_method='linear', quaternion_interpolation_method="slerp" ,infer_time=0.05, step_time=0.005, interpolation_step=200):
        self.arms.send_control_pose_trj(is_async_, pose_trj_l, pose_trj_r, pos_method, quaternion_interpolation_method, infer_time, step_time, interpolation_step)

    def arms_control_raw_trj(self,cmd_l, cmd_r,t_step=0.005):
        self.arms.send_control_raw_trj(cmd_l, cmd_r, t_step)

    def start_arm_actions_thread(self):
        self.arms.start_arm_actions_thread()

    def arms_data(self):
        return self.arms.get_data()
    
    def arms_cur_data(self):
        return self.arms.get_cur_data()
    
    def arms_joint_data(self):
        return self.arms.get_pos_data()

    def arms_zero(self):
        self.arms.send_zero()
    
    def arms_zero_after_oepen_gripper(self):
        arm_l,arm_r = self.arms_data()
        arm_l[6] = 4.0
        arm_r[6] = 4.0
        self.arms.send_control(arm_l,arm_r)
        rospy.Rate(0.5).sleep()
        arm_l = [0.0,0.0,0.0,0.0,0.0,0.0,4.0]
        arm_r = [0.0,0.0,0.0,0.0,0.0,0.0,4.0]
        self.arms.send_control(arm_l,arm_r)
        rospy.Rate(0.5).sleep()
        arm_l = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        arm_r = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self.arms.send_control(arm_l,arm_r)

    def chassis_control_vel(self,cmd):
        self.chassis.send_control_vel(cmd)

    def chassis_pose_data(self):
        return self.chassis.pose_data()
    
    def chassis_rel_pose_data(self):
        return self.chassis.rel_pose_data()
    
    def chassis_control_global_pose(self,cmd):
        self.chassis.send_control_global_pose(cmd)

    def chassis_set_virtual_zero(self,pose):
        self.chassis.set_virtual_zero(pose)

    def chassis_set_current_pose_as_virtual_zero(self):
        self.chassis.set_current_pose_as_virtual_zero()

    def chassis_stop(self):
        self.chassis.stop()

    def chassis_start(self):
        self.chassis.start()

    def chassis_control_relative_pose(self, cmd, is_arrived=False):
        return self.chassis.send_control_relative_pose(cmd, is_arrived)

    def chassis_control_delta_pose(self,cmd,is_arrived=False):
        self.chassis.send_control_delta_pose(cmd, is_arrived)

    def chassis_move(self,cmd,t):
        st = time.time()
        while time.time() - st < t:
            self.chassis.send_control_vel([i/t for i in cmd])
            time.sleep(0.02)
    
    def optimize_rel_pathxyY_control(self, path):
        self.chassis.optimize_pathxyY_control(path)

    def cam1_data(self):
        return self.cam.get_cam1_data()
    
    def cam2_data(self):
        return self.cam.get_cam2_data()

    def cam3_data(self):
        return self.cam.get_cam3_data()
    
    def cam1_compress(self):
        return self.cam.compress_image(self.cam.get_cam1_data())
    
    def cam2_compress(self):
        return self.cam.compress_image(self.cam.get_cam2_data())
    
    def cam3_compress(self):
        return self.cam.compress_image(self.cam.get_cam3_data())


class MovingController(RobotController):
    def __init__(self,init_node=False):
        if init_node:
            rospy.init_node('moving_controller')
        self.data_yaw = 0
        self.data_pitch = 0
        self.kinematics =  kinematics()
        self.head = MovingHeadController()
        self.lift = MovingLiftController()
        self.arms = ArmsController()
        self.chassis = MovingChassisController()
        self.cam = Camera()
        rospy.Rate(2).sleep()

        self.virtual_zero_tf_inv = None

    def head_control(self, cmd):
        self.head.send_control(cmd)

    def head_data(self):
        return self.head.get_data()

    def lift_control(self,cmd):
        self.lift.send_control(cmd)

    def lift_data(self):
        return self.lift.get_data()
    
    def arms_start(self):
        self.arms.start()

    def arms_stop(self):
        self.arms.stop()

    def arms_control(self,cmd_l,cmd_r):
        self.arms.send_control(cmd_l,cmd_r)

    def arms_control_pose_trj(self, is_async_: bool, pose_trj_l, pose_trj_r, \
                pos_method='linear', quaternion_interpolation_method="slerp" ,infer_time=0.05, step_time=0.005, interpolation_step=200):
        self.arms.send_control_pose_trj(is_async_, pose_trj_l, pose_trj_r, pos_method, quaternion_interpolation_method, infer_time, step_time, interpolation_step)

    def arms_control_pose_trj_sync(self, pose_trj_l, pose_trj_r, \
                pos_method='linear', quaternion_interpolation_method="slerp", t_step=0.005, interpolation_step=200):
        self.arms.send_control_pose_trj_sync(pose_trj_l, pose_trj_r, pos_method, quaternion_interpolation_method, t_step, interpolation_step)
    
    def arms_control_pose_trj_async(self, pose_trj_l, pose_trj_r, \
                pos_method='linear', quaternion_interpolation_method="slerp" ,infer_time=0.05, t_step=0.005, interpolation_step=200):
        self.arms.send_control_pose_trj_async(pose_trj_l, pose_trj_r, pos_method, quaternion_interpolation_method, infer_time, t_step, interpolation_step)

    def arms_control_raw_trj(self,cmd_l, cmd_r,t_step=0.005):
        self.arms.send_control_raw_trj(cmd_l, cmd_r, t_step)

    def start_arm_actions_thread(self):
        self.arms.start_arm_actions_thread()

    def arms_data(self):
        return self.arms.get_data()
    
    def arms_cur_data(self):
        return self.arms.get_cur_data()
    
    def arms_joint_data(self):
        return self.arms.get_pos_data()

    def arms_zero(self):
        self.arms.send_zero()
    
    def arms_zero_after_oepen_gripper(self):
        arm_l,arm_r = self.arms_data()
        arm_l[6] = 4.0
        arm_r[6] = 4.0
        self.arms.send_control(arm_l,arm_r)
        rospy.Rate(0.5).sleep()
        arm_l = [0.0,0.0,0.0,0.0,0.0,0.0,4.0]
        arm_r = [0.0,0.0,0.0,0.0,0.0,0.0,4.0]
        self.arms.send_control(arm_l,arm_r)
        rospy.Rate(0.5).sleep()
        arm_l = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        arm_r = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self.arms.send_control(arm_l,arm_r)

    def chassis_control_vel(self,cmd):
        self.chassis.send_control_vel(cmd)

    def chassis_pose_data(self):
        return self.chassis.pose_data()
    
    def chassis_rel_pose_data(self):
        return self.chassis.rel_pose_data()
    
    def chassis_control_global_pose(self,cmd):
        self.chassis.send_control_global_pose(cmd)

    def chassis_set_virtual_zero(self,pose):
        self.chassis.set_virtual_zero(pose)

    def chassis_set_current_pose_as_virtual_zero(self):
        self.chassis.set_current_pose_as_virtual_zero()

    def chassis_stop(self):
        self.chassis.stop()

    def chassis_start(self):
        self.chassis.start()

    def chassis_control_relative_pose(self,cmd,is_arrived=False):
        return self.chassis.send_control_relative_pose(cmd,is_arrived)

    def chassis_control_delta_pose(self,cmd):
        self.chassis.send_control_delta_pose(cmd)

    def chassis_move(self,cmd,t):
        st = time.time()
        while time.time() - st < t:
            self.chassis.send_control_vel([i/t for i in cmd])
            time.sleep(0.02)
    
    def optimize_rel_pathxyY_control(self, path):
        self.chassis.optimize_pathxyY_control(path)

    def cam1_data(self):
       return self.cam.get_cam1_data()
    
    def cam2_data(self):
        return self.cam.get_cam2_data()

    def cam3_data(self):
        return self.cam.get_cam3_data()
    
    def cam1_compress(self):
        return self.cam.compress_image(self.cam.get_cam1_data())
    
    def cam2_compress(self):
        return self.cam.compress_image(self.cam.get_cam2_data())
    
    def cam3_compress(self):
        return self.cam.compress_image(self.cam.get_cam3_data())

def handle_sigint(signum, frame):
    rospy.signal_shutdown("SIGINT received")
    exit(0)

def robot_controller_access(registry_node :bool = True):
    import threading
    if threading.current_thread() is threading.main_thread():
        signal.signal(signal.SIGINT, handle_sigint)
    import os
    robot_type = os.environ.get('ROBOT_TYPE', 'TURTLE2')
    if robot_type == 'TURTLE2':
        return Turtle2Controller(registry_node)
    elif robot_type == 'MOVING':
        return MovingController(registry_node)
    else:
        raise ValueError(f"Unsupported robot type: {robot_type}")

if __name__ == "__main__":
    turtle2 = Turtle2Controller(True)
    turtle2.chassis_set_current_pose_as_virtual_zero()
    turtle2.chassis_control_relative_pose([0.5, 0.0, 0.0],True)

