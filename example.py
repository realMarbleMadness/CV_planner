import sys
import rospy

import numpy as np
import cv2 as cv
import baxter_interface

from plans import BaxterInterfacePlanner
from moveit_commander.conversions import list_to_pose, pose_to_list

import Cali_Cam as cc
import im2env 
import pdb

from copy import deepcopy as copy
import quaternion
#from baxter_tools import tuck_arms

from config import Config

conf = Config()

class Baxter_IK(object):
    def __init__(self):
        self.planner = BaxterInterfacePlanner('left')

        # quaternions for the end effector. Comments are quaternions converted to roll, pitch, yaw (Euler angles)
        self.hook_up = [-0.50279713167, 0.439115525177, -0.503707035483, 0.548317264308] # [-0.11168128,  1.46367804,  4.73751126]
        self.hook_down = [0.502106700973, 0.516845017147, 0.532957371384, 0.44352737162] #[-0.0465213 ,  1.6733205 ,  1.49358146]
        self.hook_left = [0.703867943158, 0.0439191618412, 0.708933206651, -0.00739826497198] #[-0.07238247,  1.57982682,  0.0513615 ]
        self.hook_right = [-0.0163754827529, 0.717149782385, 0.0494403950619, 0.694970273124] #[0.09238971, 1.60429892, 3.09631993]

        #example: hookLeft = quaternion.as_euler_angles(quaternion.as_quat_array(np.array(<quaternion>)))

        # L shaped arm configuration
        # must append a quaternion to the end of it
        self.L_shape = [0.734563160617, 0.69072462064, 0.450152531088]


    def getQuarternion(self, euler_z):
        """get the list of quarternion based on euler z angle
        
        Arguments:
            euler_z {float} -- z angle. For example, hook_up is 3*pi/2, hook_down is pi/2, hook_left is 0, and hook_right is pi
        
        Returns:
            list of float -- the 1*4 python list of quarternions
        """
        print 'wrist angle (rads):',euler_z
        desiredQuat = quaternion.as_float_array(quaternion.from_euler_angles(0, np.pi/2, euler_z)).tolist()
        print 'desired quat: ', desiredQuat
        return desiredQuat


    def execute_trajectory(self, init_obs, final_obs): 
        '''
        init_obs -> initial obstacle positon, for now, just one, list
        final_obstacle -> final obstacle positon, for now, just one, list
        '''

        # TODO
        # constraint the angle coming from optimizer to -pi/2 to pi/2
        # the block orientation is 90 degrees out of phase, ex: if a block is at 0 dgrees, need claws at 90 degrees to grab it


        testquat = [2.819154132497404e-17, 0.8877101178652074, 0.46040280911364956, 5.435656772071729e-17]


        pose_start = []
        pose_action = []
        gripper_action = []
        for oi, of in zip(init_obs, final_obs):
            # trimming the last one for now

            # just trust it!
            oi[0] += 0.005
            of[0] += 0.005
            oi[2] += 0.075
            of[2] += 0.075
            
            final_angle = of[3]            
            oi = oi[:-1].tolist()
            of = of[:-1].tolist()

            # TODO: replace the hardcoded hook_up to use getQuarternion

            pose_start = [0.835162615786, 0.00508696410378, 0.409410184983] + self.hook_up
    
            pose_Li = [oi[0]-0.1] + oi[1:] + self.hook_up #intermediate pose
            pose_Lf = [of[0]-0.1] + of[1:] + self.hook_up #final pose
            
            pose_init = oi + self.hook_up
            pose_final = of + self.getQuarternion(final_angle)

            pose_fb = copy(pose_final)  # move 10 cm away from the final pose
            pose_fb[0] -= 0.1

            pose_action += [pose_start, pose_Li, pose_init, pose_Li, pose_Lf, pose_final, pose_final, pose_fb, pose_Lf]
            gripper_action += [100, 100, 0, 0, 0, 0, 100, 100, 100] 

        # if check_limits(pose_action):
        #     print "waypoint out of bounds"    
        # else:

        waypoint_list = zip(pose_action, gripper_action)
        self.planner.ik_joint_and_gripper_plan_execution(waypoint_list)
        return 0


def check_limits(pose_targets):
    """Check if Baxter goes out of bounds and if so, skip waypoint

    Parameters
    ----------
    pose_target : Pose
        current pose target to be about to added to the planning queue

    Returns
    -------
    bool
        Indicates whether target is valid and should be added to the planning queue

    """
    x_lower = conf.X_LOWER
    x_upper = conf.X_UPPER # do not go to top row of toy to not destroy toy and to allow go lower for bottom row
    y_lower = conf.Y_LOWER
    y_upper = conf.Y_UPPER
    z_lower = conf.Z_LOWER
    z_upper = conf.Z_UPPER
    for pose_target in pose_targets:
        if type(pose_target) is list:
            x = pose_target[0]
            y = pose_target[1]
            z = pose_target[2]
        else:
            x = pose_target.position.x
            y = pose_target.position.y
            z = pose_target.position.z

        if x < x_lower or x > x_upper:
            return True
        if y < y_lower or y > y_upper:
            return True
        if z < z_lower or z > z_upper:
            return True
    return False


def main():
    rospy.init_node("robot_autonomy")

    # assume this is camera input
    im = im2env.get_image(30)
    init_obs, end_obs = im2env.to_planner(im)

    print 'init: ', init_obs
    print 'ending: ', end_obs
    # pdb.set_trace()

    gerard_way = Baxter_IK()
    gerard_way.execute_trajectory(init_obs, end_obs)

    # pdb.set_trace()


if __name__ == '__main__':
    main()




#intermediate pos? 4/29 6:20 PM
    #      position: 
    # x: 0.647337088674
    # y: 0.498833860267
    # z: 0.301081012295
    # orientation: 
    # x: -0.454397760529
    # y: 0.480571374262
    # z: -0.539562793204
    # w: 0.521004627289


    #return it to the start position
# pose_action += [0.647337088674, 0.498833860267, 0.301081012295] + self.hook_up
# pose_action += [100]
#pose_action += [[0.969264823977, 0.527207560657, 0.496214967195] + self.hook_up]
#gripper_action += [0]

# print 'trajectory: ', pose_action
# print 'gripper action: ', gripper_action  
#pdb.set_trace()    


    [0.696196812113,0.187203255955, 0.689617017459, 0.0685077294556]
