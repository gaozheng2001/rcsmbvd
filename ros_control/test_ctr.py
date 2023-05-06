import numpy as np
import rospy
import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi

from rcsmbvd.srv import mocap_joints
from rcsmbvd.msg import *

from utils.process_joints import *

# import kinematics

class ExampleMoveItTrajectories(object):
    """ExampleMoveItTrajectories"""
    def __init__(self):

        # Initialize the node
        super(ExampleMoveItTrajectories, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('example_move_it_trajectories')

        try:
            self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
            if self.is_gripper_present:
                gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
                self.gripper_joint_name = gripper_joint_names[0]
            else:
                self.gripper_joint_name = ""
            self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

            # Create the MoveItInterface necessary objects
            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander("robot_description")
            self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
            self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                          moveit_msgs.msg.DisplayTrajectory,
                                                          queue_size=20)

            if self.is_gripper_present:
                gripper_group_name = "gripper"
                self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())


            rospy.wait_for_service('/mocap_joints')
            print("Service found")
            self.mocap = rospy.ServiceProxy('/mocap_joints', mocap_joints)

            rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
        except Exception as e:
            print (e)
            self.is_init_success = False
        else:
            self.is_init_success = True


    def reach_named_position(self, target):
        arm_group = self.arm_group

        # Going to one of those targets
        rospy.loginfo("Going to named target " + target)
        # Set the target
        arm_group.set_named_target(target)
        # Plan the trajectory
        (success_flag, trajectory_message, planning_time, error_code) = arm_group.plan()
        # Execute the trajectory and block while it's not finished
        return arm_group.execute(trajectory_message, wait=True)

    def reach_joint_angles(self, tolerance, joint_pose: list):
        arm_group = self.arm_group
        success = True

        # Get the current joint positions
        joint_positions = arm_group.get_current_joint_values()
        rospy.loginfo("Printing current joint positions before movement :")
        #for p in joint_positions: rospy.loginfo(p)
        joint_positions_bef = joint_positions.copy()
        # Set the goal joint tolerance
        self.arm_group.set_goal_joint_tolerance(tolerance)

        # Set the joint target configuration
        if self.degrees_of_freedom == 7:
            joint_positions[0] = pi/2
            joint_positions[1] = 0
            joint_positions[2] = pi/4
            joint_positions[3] = -pi/4
            joint_positions[4] = 0
            joint_positions[5] = pi/2
            joint_positions[6] = 0.2
        elif self.degrees_of_freedom == 6:
            joint_positions[0] = 0
            joint_positions[1] = 0
            joint_positions[2] = pi/2
            joint_positions[3] = pi/4
            joint_positions[4] = 0
            joint_positions[5] = pi/2
            if joint_pose != None:
                joint_positions = joint_pose
        print("Target joint positions :")
        print(joint_positions)
        # Plan and execute in one command

        try:
            arm_group.set_joint_value_target(joint_positions)
        except Exception as e:
            arm_group.set_joint_value_target(joint_positions_bef)
            print(type(e))
            print(e)
            success = False

        try:
            arm_group.go(wait=True)
        except Exception as e:
            print(type(e))
            print(e)
            success = False


        #success &= arm_group.go(wait=True)

        # Show joint positions after movement
        new_joint_positions = arm_group.get_current_joint_values()
        rospy.loginfo("Printing current joint positions after movement :")
        #for p in new_joint_positions: rospy.loginfo(p)
        print(new_joint_positions)
        return success

    def get_cartesian_pose(self):
        arm_group = self.arm_group

        # Get the current pose and display it
        pose = arm_group.get_current_pose()
        rospy.loginfo("Actual cartesian pose is : ")
        rospy.loginfo(pose.pose)

        return pose.pose

    def reach_cartesian_pose(self, pose, tolerance, constraints):
        arm_group = self.arm_group

        # Set the tolerance
        arm_group.set_goal_position_tolerance(tolerance)

        # Set the trajectory constraint if one is specified
        if constraints is not None:
            arm_group.set_path_constraints(constraints)

        # Get the current Cartesian Position
        arm_group.set_pose_target(pose)

        # Plan and execute
        rospy.loginfo("Planning and going to the Cartesian Pose")
        return arm_group.go(wait=True)

    def reach_gripper_position(self, relative_position):
        gripper_group = self.gripper_group

        # We only have to move this joint because all others are mimic!
        gripper_joint = self.robot.get_joint(self.gripper_joint_name)
        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_min_absolute_pos = gripper_joint.min_bound()
        try:
            val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
            return val
        except:
            return False 


def mocap(example):
    global success
    #import pdb; pdb.set_trace()
    mocap_joint = example.mocap()
    rospy.loginfo('get mocap joints')
    mocap_process(mocap_joint.mocap_data)
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


# def Scale(vect, terget_len):
#     '''
#     Scale a vector to a target length
#     '''
#     return vect / np.linalg.norm(vect) * terget_len


# def map_arm_2_to_plant_arm_8_4(arm_2, arm_4):
#     '''
#     Map the arm 2 to the plant arm 8 and 4
#     p8: the plant arm 8 (0, 0, 0)
#     p2: the arm 2 (x, y, z)
#     p4: the plant arm 4 (x, y, z)
#     '''
#     r = 10/1000
#     r_1_2 = 15/1000
#     p4 = arm_4.copy()
#     p2 = arm_2.copy()

#     '''
#     {x**2 + y**2 = r**2
#     {x * x4 + y * y4 = r**2
#     ==>
#     (x4**2 + y4**2) * x**2 + (- 2 * x4 * r**2) * x + r**4 - r**2 * y4**2 = 0
#     '''
#     A = p4[0]**2 + p4[1]**2
#     B = - 2 * p4[0] * r**2
#     C = r**4 - r**2*p4[1]**2

#     def intersection(p1, p2, p3):
#         '''
#         Scale point 3 to r_1_2 away from point 2 along the line 2-1
#         '''
#         arm4 = p1.copy()
#         arm4[-1] = 0.0
#         arm2 = p3.copy()
#         arm2[-1] = 0.0
#         r4 = arm4 - p2
#         r2 = arm2 - p2

#         r2_vertical = np.cross(r4, np.cross(r4, r2))
#         r2_vertical = Scale(r2_vertical, np.abs(np.cross(r4, r2)[-1]/np.linalg.norm(r4)))
#         x, y, _ = arm2 + r2_vertical
#         return x, y
#     x1 = (- B + np.sqrt(B**2 - 4 * A * C)) / (2 * A)
#     y1 = (r**2 - x1 * p4[0]) / p4[1]
#     x2 = (- B - np.sqrt(B**2 - 4 * A * C)) / (2 * A)
#     y2 = (r**2 - x2 * p4[0]) / p4[1]
#     if x1 * p4[1] - y1 * p4[0] > 0:
#         x_out, y_out = intersection(p4, (x1, y1, 0), p2)
#         robot_arm_2 = np.array([x1, y1, 0])
#     else:
#         x_out, y_out = intersection(p4, (x2, y2, 0), p2)
#         robot_arm_2 = np.array([x2, y2, 0])
#     return np.array([x_out, y_out, arm_2[-1]]), robot_arm_2
    

# def convert_angle_range(angle, range_min=-np.pi, range_max=np.pi):
#     '''
#     Convert the angle to the range of [range_min, range_max]
#     '''
#     if angle > pi:
#         angle = angle - 2 * pi
#     elif angle < -pi:
#         angle = angle + 2 * pi

#     range_min = range_min / 180 * pi
#     range_max = range_max / 180 * pi

#     # return (angle - pi) * (range_max - range_min) / (2 * pi) + (range_max + range_min) / 2

#     upper = angle > range_max
#     lower = angle < range_min
#     if range_max - range_min < 2 * np.pi:
#         if upper:
#             return range_max
#         elif lower:
#             return range_min
#         else:
#             return angle
#     else:
#         if upper:
#             return angle - 2 * np.pi
#         elif lower:
#             return angle + 2 * np.pi
#         else:
#             return angle


# def get_rob_arm_joint_angles(r_arm_joints):
#     #base frame: point 8(0, 0, 0)
#     r_arm_8, r_arm_1, r_arm_2, r_arm_4 = r_arm_joints - r_arm_joints[0]
#     #import pdb; pdb.set_trace()
#     arm_1_2 = r_arm_2 - r_arm_1
#     arm_2_4 = r_arm_4 - r_arm_2
#     arm_2_4_len = np.linalg.norm(arm_2_4)
#     arm_1_2[-1] = 0.0
#     r_arm_2[:2] = Scale(arm_1_2, 0.15)[:2]

#     r_arm_2, robot_arm_2 = map_arm_2_to_plant_arm_8_4(r_arm_2, r_arm_4)

#     #Map point 2 to the joint of robot arm 3
#     robot_arm_1 = np.array([0, 0, 128.3 + 115])/1000
#     robot_arm_2 = robot_arm_2 + robot_arm_1
#     robot_arm_3 = Scale(r_arm_2 - robot_arm_2, 0.28) + robot_arm_2
#     robot_arm_4 = Scale(r_arm_4 - r_arm_2, (140 + 105) / 1000) + robot_arm_3

#     rob_joint_1 = np.arctan2(robot_arm_2[1], robot_arm_2[0]) + pi/2
#     rob_joint_2 = - np.arccos((robot_arm_3[2] - (128.3 + 115)/1000)/0.28)
#     if (robot_arm_3[0]**2 + robot_arm_3[1]**2) < (robot_arm_4[0]**2 + robot_arm_4[1]**2):
#         rob_joint_3 = - np.arccos((robot_arm_3[2] - robot_arm_4[2])/((140 + 105) / 1000)) + pi - np.abs(rob_joint_2)
#     else:
#         rob_joint_3 = np.arccos((robot_arm_3[2] - robot_arm_4[2])/((140 + 105) / 1000)) + pi - np.abs(rob_joint_2)

#     #print(robot_arm_1, robot_arm_2, robot_arm_3, robot_arm_4)
#     raw_rob_arm = [robot_arm_1, robot_arm_2, robot_arm_3, robot_arm_4]

#     rob_joint_1 = convert_angle_range(rob_joint_1, -150.1, 150.1)
#     rob_joint_2 = convert_angle_range(rob_joint_2, -150.1, 150.1)
#     rob_joint_3 = convert_angle_range(rob_joint_3, -150.1, 150.1)
#     return rob_joint_1, rob_joint_2, rob_joint_3, raw_rob_arm


# def get_rob_hand_joint_angles(r_hand_joints, rob_joint_1_3):
#     '''
#     r_hand_joints: 21 * 3 numpy array
#     '''
#     r_4 = (140 + 105) / 1000
#     r_5 = 28.5 * 2 / 1000
#     r_6 = (130 +105) / 1000

#     rob_joint_1, rob_joint_2, rob_joint_3, raw_rob_arm = rob_joint_1_3
#     rob_joins_1_3 = np.array([rob_joint_1, rob_joint_2, rob_joint_3])

#     hand_root = r_hand_joints[0]
#     hand_center = np.sum(r_hand_joints[1:], axis=0) / 20
#     hand_vertical = hand_center - hand_root

#     #1: calculate gripper present
#     gripper_ver1 = r_hand_joints[4] - hand_center
#     gripper_ver2 = r_hand_joints[12] - hand_center
#     gripper_cos = np.dot(gripper_ver1, gripper_ver2) / (np.linalg.norm(gripper_ver1) * np.linalg.norm(gripper_ver2))
#     gripper_sin = np.linalg.norm(np.cross(gripper_ver1, gripper_ver2)) / (np.linalg.norm(gripper_ver1) * np.linalg.norm(gripper_ver2))
#     gripper = np.arctan2(gripper_sin, gripper_cos)
#     gripper = convert_angle_range(gripper, 0.0, 2 * pi)
#     gripper = 1.0 if gripper > pi/3 else 3 * gripper / pi

#     #2.1: get the position of the end of the arm in the base frame
#     hand_vertical = Scale(hand_vertical, np.sqrt(r_5**2 + r_6**2))

#     T_0_3 = kinematics.transformation_matrix(rob_joins_1_3, 0, 3)

#     robot_arm_4 = raw_rob_arm[-1]
#     rob_arm_0_6 = hand_vertical + robot_arm_4
#     px0, py0, pz0 = rob_arm_0_6
    
#     #2.2: use transformation matrix to get the position of the end of the arm in the joint 3 frame
#     rob_arm_3_6 = np.dot(np.matrix(T_0_3).I, np.array([px0, py0, pz0, 1]).reshape(4, 1))[:-1]
#     px, py, pz = rob_arm_3_6
#     #2.3: use transformation matrix to calculate joint angles 4, 5
#     '''
#     T_i_1_i = [[ci    , -si   , 0    , ai-1    ], 
#                [sici-1, cici-1, -si-1, -si-1*di],  
#                [sisi-a, cisi-a, ci-1 , ci-1*di ], 
#                [0     , 0     , 0    , 1       ]]
#     '''
#     rob_joint_5 = np.array(np.arccos(- (py + r_4) / r_6))[0][0]
#     temp_a = - np.sin(rob_joint_5) * r_6
#     temp_b = r_5
#     temp_c = - np.sin(rob_joint_5) * r_6
#     temp_d = - r_5
#     tamp_cos = (temp_a * px - temp_b * pz) / (temp_a * temp_c - temp_b * temp_d)
#     tamp_sin = (px - temp_a * tamp_cos) / temp_b
#     #rob_joint_4 = np.array(np.arctan2(tamp_sin, tamp_cos) - pi/2)[0][0]
#     rob_joint_4 = np.array(np.arctan2(tamp_sin, tamp_cos))[0][0]
    
#     #TODO: find out the way to calculate joint 6
#     #3: calculate joint 6
#     # By default, the direction from 
#     # the root of the middle finger to the fingertip of the thumb 
#     # is the positive direction of the y-axis of the joint 6 coordinate system
#     palm_facing = r_hand_joints[9] - r_hand_joints[4]
#     palm_facing_I = np.cross(hand_vertical, np.cross(hand_vertical, palm_facing))
#     palm_facing_I = palm_facing_I / np.linalg.norm(palm_facing_I)
#     palm_facing_I_0 = palm_facing_I + hand_vertical + robot_arm_4

#     rob_joins_1_5 = np.array([rob_joint_1, rob_joint_2, rob_joint_3, rob_joint_4, rob_joint_5])
#     T_0_5 = kinematics.transformation_matrix(rob_joins_1_5, 0, 5)
#     palm_facing_I_5 = np.dot(np.matrix(T_0_5).I, np.array([palm_facing_I_0[0], palm_facing_I_0[1], palm_facing_I_0[2], 1]).reshape(4, 1))[:-1]
    
#     #rob_joint_6 = np.array(np.arctan2(- palm_facing_I_5[0], - palm_facing_I_5[-1]) + pi / 2)[0][0]
    
#     rob_joint_6 = np.array(np.arctan2(- palm_facing_I_5[0], - palm_facing_I_5[-1]))[0][0]

#     rob_joint_4 = convert_angle_range(rob_joint_4, -148.98, 148.98)
#     rob_joint_5 = convert_angle_range(rob_joint_5, -144.97, 145.0)
#     rob_joint_6 = convert_angle_range(rob_joint_6, -148.98, 148.98)

#     return rob_joint_4, rob_joint_5, rob_joint_6, gripper


def mocap_process(joints):
    global success, example

    r_arm_joints = np.array(joints.body_joints).reshape(-1, 3)
    r_arm_joints = Convert_img_to_world(r_arm_joints)
    Scale_p2w = 0.556 / (r_arm_joints[1,2] - r_arm_joints[0,2])
    r_arm_joints = Scale_pixel_to_world(r_arm_joints, Scale_p2w)

    r_hand_joints = np.array(joints.rhand_joints).reshape(-1, 3)
    r_hand_joints = Convert_img_to_world(r_hand_joints)
    r_hand_joints = Scale_pixel_to_world(r_hand_joints, Scale_p2w)

    #TODO: mapping whole right arm to kinova Gen3 Lite
    '''
    Gen3 Lite:     | Mocap:
          6        |          
          -        |          1
      4 | 5        |       2  
      -            |       3  
      3            |       4  
     2             |          
      1            |          8
      -            |          
      0            |          
    for arm: joints 1, 2, 3
    for hand: joints 4, 5, 6

    tentative solution:
    Gen3 Lite:     | Mocap:
            point 0 <-> point 8
            point 3 <-> point 2
            point 4 <-> point 4
    '''
    rob_joint_1_3 = get_rob_arm_joint_angles(r_arm_joints)

    rob_joint_1, rob_joint_2, rob_joint_3, raw_rob_arm = rob_joint_1_3
    
    #TODO: mapping right hand to kinova Gen3 Lite
    rob_joint_4_6 = get_rob_hand_joint_angles(r_hand_joints, rob_joint_1_3)
    rob_joint_4, rob_joint_5, rob_joint_6, gripper = rob_joint_4_6

    rob_joint = [rob_joint_1, rob_joint_2, rob_joint_3, rob_joint_4, rob_joint_5, rob_joint_6]

    info = "Reaching Mocap Joint Angles:" + str(rob_joint)
    rospy.loginfo(info)
    success = example.reach_joint_angles(tolerance=0.01, joint_pose=rob_joint) #rad
    print (success)
    actual_pose = example.get_cartesian_pose()
    info = "Closing the gripper " + str((1 - gripper) * 100) + "%..."
    rospy.loginfo(info)
    success = example.reach_gripper_position(1 - gripper)
    print (success)

def main():
    global success, example
    example = ExampleMoveItTrajectories()
    # For testing purposes
    success = example.is_init_success
    if success:
        rospy.loginfo("Reaching Named Target Vertical...")
        success &= example.reach_named_position("vertical")
        print (success)
    
    if success:
        while not rospy.is_shutdown():
            #TODO: seems like the mocap Subscriber is not working
            mocap(example)
        rospy.spin()

if __name__ == '__main__':
    main()