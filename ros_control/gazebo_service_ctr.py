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
        global reached_frame, total_frame

        arm_group = self.arm_group
        success = True

        # Get the current joint positions
        joint_positions = arm_group.get_current_joint_values()
        #rospy.loginfo("Printing current joint positions before movement :")
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

        # Plan and execute in one command


        # Set the joint target configuration
        set_suss = False
        try:
            arm_group.set_joint_value_target(joint_positions)
            set_suss = True
        except Exception as e:
            arm_group.set_joint_value_target(joint_positions_bef)
            print("Exception in set_joint_value_target")
            success = False

        try:
            arm_group.go(wait=True)
            if set_suss:
                reached_frame += 1
                print("Reached the target")
        except Exception as e:
            print(type(e))
            print(e)
            success = False


        #success &= arm_group.go(wait=True)

        # Show joint positions after movement
        new_joint_positions = arm_group.get_current_joint_values()
        #rospy.loginfo("Printing current joint positions after movement :")
        #for p in new_joint_positions: rospy.loginfo(p)
        #print(new_joint_positions)
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


def mocap_process(joints):
    global success, example, reached_frame, total_frame

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



    total_frame += 1
    print('Execute the ' + str(total_frame) + 'th command')
    print('-------------------------------------------------------------')
    info = "Reaching Mocap Joint Angles:" + str(rob_joint)
    rospy.loginfo(info)
    success = example.reach_joint_angles(tolerance=0.01, joint_pose=rob_joint) #rad
    print (success)
    if success:
        raw_rob_arm = np.array(raw_rob_arm).reshape(-1).tolist()
        rob_joint = np.array(rob_joint).reshape(-1).tolist()
        can_pub = True
        for i in range(len(raw_rob_arm)):
            if not isinstance(raw_rob_arm[i], float):
                can_pub = False
                print('raw_rob_arm is not float')
                print(raw_rob_arm[i])
                print(type(raw_rob_arm[i]))
                print(raw_rob_arm)
                break
        for i in range(len(rob_joint)):
            if not isinstance(rob_joint[i], float):
                can_pub = False
                print('rob_joint is not float')
                print(rob_joint[i])
                print(type(rob_joint[i]))
                print(rob_joint)
                break
        if can_pub:
            draw_msg = draw_joints()
            draw_msg.hands = r_hand_joints.reshape(-1).tolist()
            draw_msg.real_arm = raw_rob_arm
            draw_msg.cal_joints = rob_joint
            pub.publish(draw_msg)
            print(draw_msg)
    actual_pose = example.get_cartesian_pose()
    info = "Closing the gripper " + str((1 - gripper) * 100) + "%..."
    rospy.loginfo(info)
    success = example.reach_gripper_position(gripper)
    print (success)
    print("Success rate: " + str(reached_frame) + "/" + str(total_frame))

def main():
    global success, example, pub
    example = ExampleMoveItTrajectories()
    pub = rospy.Publisher('/12345', draw_joints, queue_size=10)
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
    global reached_frame, total_frame
    reached_frame = 0
    total_frame = 0
    main()