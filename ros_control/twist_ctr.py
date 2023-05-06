import time
import sys
import os
import threading

from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient

from kortex_api.autogen.messages import Session_pb2, Base_pb2

from math import pi
import numpy as np
from utils.process_joints import *
# import kinematics

import rospy
from rcsmbvd.srv import mocap_joints
from rcsmbvd.msg import *


# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20

# ros control code
# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        print(notification.action_event)
        if notification.action_event == Base_pb2.ACTION_END :
            e.set()
        elif notification.action_event == Base_pb2.ACTION_ABORT:
            exit(1)
    return check

def example_move_to_home_position(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    #print(action_list)
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle

    if action_handle == None:
        print("Can't reach safe position. Exiting")
        sys.exit(0)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )
    
    base.ExecuteActionFromReference(action_handle)
    
    # Leave time to action to complete
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Safe position reached")
    else:
        print("Timeout on action notification wait")
    return finished

def Initialize_pose(base):
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    waypoints = Base_pb2.WaypointList()
    
    waypoints.duration = 0.0
    waypoints.use_optimal_blending = False
    
    def populateCartesianCoordinate(waypointInformation):
    
        waypoint = Base_pb2.CartesianWaypoint()  
        waypoint.pose.x = waypointInformation[0]
        waypoint.pose.y = waypointInformation[1]
        waypoint.pose.z = waypointInformation[2]
        waypoint.blending_radius = waypointInformation[3]
        waypoint.pose.theta_x = waypointInformation[4]
        waypoint.pose.theta_y = waypointInformation[5]
        waypoint.pose.theta_z = waypointInformation[6] 
        waypoint.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE
        
        return waypoint

    index = 0
    Initialize_waypoints = (0.3, 0.0, 0.1, 0.0, 180.0, 0.0, 90)
    waypoint = waypoints.waypoints.add()
    waypoint.name = "waypoint_" + str(index)   
    waypoint.cartesian_waypoint.CopyFrom(populateCartesianCoordinate(Initialize_waypoints))
    index = index + 1 

    # Verify validity of waypoints
    result = base.ValidateWaypointList(waypoints);
    if(len(result.trajectory_error_report.trajectory_error_elements) == 0):
        e = threading.Event()
        notification_handle = base.OnNotificationActionTopic(   check_for_end_or_abort(e),
                                                                Base_pb2.NotificationOptions())

        print("Moving cartesian trajectory...")
        
        base.ExecuteWaypointTrajectory(waypoints)

        print("Waiting for trajectory to finish ...")
        finished = e.wait(TIMEOUT_DURATION)
        base.Unsubscribe(notification_handle)

        if finished:
            print("Cartesian trajectory with no optimization completed ")
            e_opt = threading.Event()
            notification_handle_opt = base.OnNotificationActionTopic(   check_for_end_or_abort(e_opt),
                                                                Base_pb2.NotificationOptions())

            waypoints.use_optimal_blending = True
            base.ExecuteWaypointTrajectory(waypoints)

            print("Waiting for trajectory to finish ...")
            finished_opt = e_opt.wait(TIMEOUT_DURATION)
            base.Unsubscribe(notification_handle_opt)

            if(finished_opt):
                print("Cartesian trajectory with optimization completed ")
            else:
                print("Timeout on action notification wait for optimized trajectory")

            return finished_opt
        else:
            print("Timeout on action notification wait for non-optimized trajectory")

        return finished
        
    else:
        print("Error found in trajectory") 
        result.trajectory_error_report.PrintDebugString();  

def twist_command(base, terget_twist, duration=1):

    command = Base_pb2.TwistCommand()
    '''
    CARTESIAN_REFERENCE_FRAME_UNSPECIFIED = 0
    CARTESIAN_REFERENCE_FRAME_MIXED = 1
    CARTESIAN_REFERENCE_FRAME_TOOL = 2
    CARTESIAN_REFERENCE_FRAME_BASE = 3
    '''
    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE
    command.duration = 0

    lx, ly, lz, ax, ay, az = terget_twist
    
    twist = command.twist
    twist.linear_x = lx
    twist.linear_y = ly
    twist.linear_z = lz
    twist.angular_x = ax
    twist.angular_y = ay
    twist.angular_z = az

    print('Sending the twist command for {} seconds...'.format(duration))
    print("Twist command is: ")
    print(command)
    base.SendTwistCommand(command)

    # Let time for twist to be executed
    time.sleep(duration)

    print ("Stopping the robot...")
    base.Stop()
    #time.sleep(1)

    return True

def gripper_command(base, terget_gripper):
    '''
    control the gripper with position increments

    terget_gripper: 0.0 - 1.0
    total open: 0.0
    total close: 1.0
    '''
    # Create the GripperCommand we will send
    gripper_command = Base_pb2.GripperCommand()
    finger = gripper_command.gripper.finger.add()

    # Close the gripper with position increments
    gripper_command.mode = Base_pb2.GRIPPER_POSITION
    finger.finger_identifier = 1
    finger.value = terget_gripper
    print("Going to position {:0.2f}...".format(finger.value))
    base.SendGripperCommand(gripper_command)
    #time.sleep(1)
    return True


# mocap core function
def mocap(base, mocap_service):
    #import pdb; pdb.set_trace()
    res_mocap = mocap_service()
    curent_mocap_joint = res_mocap.mocap_data
    diff_mocap_joint = res_mocap.mocap_joint_diff
    rospy.loginfo('get mocap joints')
    return mocap_process(base, curent_mocap_joint, diff_mocap_joint)
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
#     gripper = get_gripper(r_hand_joints, hand_center)


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
#     rob_joint_4 = np.array(np.arctan2(tamp_sin, tamp_cos) - pi/2)[0][0]
    
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
#     rob_joint_6 = np.array(np.arctan2(- palm_facing_I_5[0], - palm_facing_I_5[-1]) + pi / 2)[0][0]

#     rob_joint_4 = convert_angle_range(rob_joint_4, -148.98, 148.98)
#     rob_joint_5 = convert_angle_range(rob_joint_5, -144.97, 145.0)
#     rob_joint_6 = convert_angle_range(rob_joint_6, -148.98, 148.98)

#     return rob_joint_4, rob_joint_5, rob_joint_6, gripper


# def top_down_follow(r_hand_joints):
#     hand_root = r_hand_joints[0]
#     hand_center = np.sum(r_hand_joints[1:], axis=0) / 20
    
#     #1: calculate gripper present
#     gripper = get_gripper(r_hand_joints, hand_center)

    
#     return hand_center, gripper


# def get_gripper(hands, hand_center):
#     '''
#     Take the angle between the thumb, palm and middle finger as the gripper angle
    
#     gripper: 0.0 - 1.0
#     total open: 1.0
#     total close: 0.0
#     '''
#     #import pdb; pdb.set_trace()
#     gripper_ver1 = hands[4] - hand_center
#     gripper_ver2 = hands[12] - hand_center
#     gripper_cos = np.dot(gripper_ver1, gripper_ver2) / (np.linalg.norm(gripper_ver1) * np.linalg.norm(gripper_ver2))
#     gripper_sin = np.linalg.norm(np.cross(gripper_ver1, gripper_ver2)) / (np.linalg.norm(gripper_ver1) * np.linalg.norm(gripper_ver2))
#     gripper = np.abs(np.arctan2(gripper_sin, gripper_cos))
#     #gripper = convert_angle_range(gripper, 0.0, 2 * pi)
#     gripper = 1.0 if gripper > (pi* 5 / 6) else (6 * gripper / (pi * 5))

#     return gripper


# def Get_end_effector_velocity_command(r_hand_joints, diff_hand_joints):
#     '''
#     Obtain the twist command of the end effector from 
#     space coordinates of the previous frame and 
#     velocity of the current frame

#     Input:
#         r_hand_joints: 21x3 array, space coordinates of the previous frame
#         diff_hand_joints: 21x3 array, velocity of the current frame
#     Output:
#         twist = [lx, ly, lz, ax, ay, az]
#         lx, ly, lz: velocity of the end effector in the x, y, z direction
#         ax, ay, az: angular velocity of the end effector around the x, y, z axis
#     '''
#     hands_center_velocity = np.sum(diff_hand_joints[1:], axis=0) / 20
#     lx, ly, lz = hands_center_velocity

#     def limit_maximum_speed(v, max):
#         if v > max:
#             v = max
#         elif v < -max:
#             v = -max
#         return v
#     max_v = 0.1
#     lx = - limit_maximum_speed(lx, max_v) * 0.6
#     ly = - limit_maximum_speed(ly, max_v) *0.6
#     lz = limit_maximum_speed(lz, max_v) * 0.4

#     ax, ay = 0.0, 0.0

#     pre_hand_center = np.sum(r_hand_joints[1:], axis=0) / 20
#     pre_hand_vertical = pre_hand_center - r_hand_joints[0]
#     pre_palm_facing = r_hand_joints[9] - r_hand_joints[4]
#     pre_palm_facing_I = np.cross(pre_hand_vertical, np.cross(pre_hand_vertical, pre_palm_facing))
#     pre_palm_facing_I = pre_palm_facing_I[:-1]
#     pre_palm_facing_I = pre_palm_facing_I / np.linalg.norm(pre_palm_facing_I)


#     curent_hand_center = pre_hand_center + hands_center_velocity
#     curent_hand_vertical = curent_hand_center - (diff_hand_joints[0] + r_hand_joints[0])
#     curent_palm_facing = diff_hand_joints[9] + r_hand_joints[9] - (diff_hand_joints[4] + r_hand_joints[4])
#     curent_palm_facing_I = np.cross(curent_hand_vertical, np.cross(curent_hand_vertical, curent_palm_facing))
#     curent_palm_facing_I = curent_palm_facing_I[:-1]
#     curent_palm_facing_I = curent_palm_facing_I / np.linalg.norm(curent_palm_facing_I)
#     #import pdb; pdb.set_trace()
#     az = np.arctan2(np.cross(pre_palm_facing_I, curent_palm_facing_I), np.dot(pre_palm_facing_I, curent_palm_facing_I))
#     az = az * 180 / pi
    
#     #az *= 2
    
#     twist = np.array([lx, ly, lz, ax, ay, az])

#     gripper = get_gripper(r_hand_joints, pre_hand_center)

#     return twist, gripper



# def Convert_img_to_world(joints):
#     '''
#     Convert from image coordinates to world coordinates
#     img: x axis: left to right, y axis: top to bottom, z axis: backward
#     world: x axis: forward, y axis: left, z axis: up
#     '''
#     word_joint = joints.copy()
#     word_joint[:, 0] = - joints[:, 2]
#     word_joint[:, 1] = joints[:, 0]
#     word_joint[:, 2] = joints[:, 1]
#     return word_joint


# def Scale_pixel_to_world(joints, scale):
#     '''
#     Scale from pixel size to world size
#     Assumed height difference from navel(point 8) to shoulders(point 1) for colonization: 0.556m
#     '''
#     return joints * scale


def mocap_process(base, joints, diff):
    success = False
    if diff.valid != 1:
        return success
    # Space coordinate vector for each joint
    r_arm_joints = np.array(joints.body_joints).reshape(-1, 3)
    r_arm_joints = Convert_img_to_world(r_arm_joints)
    Scale_p2w = 0.556 / (r_arm_joints[1,2] - r_arm_joints[0,2])
    r_arm_joints = Scale_pixel_to_world(r_arm_joints, Scale_p2w)

    r_hand_joints = np.array(joints.rhand_joints).reshape(-1, 3)
    r_hand_joints = Convert_img_to_world(r_hand_joints)
    r_hand_joints = Scale_pixel_to_world(r_hand_joints, Scale_p2w)
    r_hand_joints = r_hand_joints - r_arm_joints[0]


    # Velocity vector for each joint
    diff_body_joints = np.array(diff.body_joints_diff).reshape(-1, 3)
    diff_body_joints = Convert_img_to_world(diff_body_joints)
    diff_body_joints = Scale_pixel_to_world(diff_body_joints, Scale_p2w)

    diff_hand_joints = np.array(diff.rhand_joints_diff).reshape(-1, 3)
    diff_hand_joints = Convert_img_to_world(diff_hand_joints)
    diff_hand_joints = Scale_pixel_to_world(diff_hand_joints, Scale_p2w)
    diff_hand_joints = diff_hand_joints - diff_body_joints[0]


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
    full_follow = 0
    if full_follow == 1:
        rob_joint_1_3 = get_rob_arm_joint_angles(r_arm_joints)
    
        rob_joint_1, rob_joint_2, rob_joint_3, raw_rob_arm = rob_joint_1_3
        
        #TODO: mapping right hand to kinova Gen3 Lite
        rob_joint_4_6 = get_rob_hand_joint_angles(r_hand_joints, rob_joint_1_3)
        rob_joint_4, rob_joint_5, rob_joint_6, gripper = rob_joint_4_6
    
        rob_joint = [rob_joint_1, rob_joint_2, rob_joint_3, rob_joint_4, rob_joint_5, rob_joint_6]
    
        for i in range(len(rob_joint)):
            rob_joint[i]  = rob_joint[i] * 180 / pi
    
        info = "Reaching Mocap Joint Angles:" + str(rob_joint)
        rospy.loginfo(info)
        #success = full_arm.example_send_joint_angles(tolerance=0.01, joint_pose=rob_joint) #rad
        if not success:
            print('-----------------------------------------------------')
            print('Failed to reach the Mocap Joint Angles')
        print(success)
        info = "Closing the gripper " + str((1 - gripper) * 100) + "%..."
        rospy.loginfo(info)
        #success = full_arm.example_send_gripper_command(gripper)
        print(success)
        return success

    else:
        #hand_center, gripper = top_down_follow(r_hand_joints)
        #print(hand_center, gripper)
        twist, gripper = Get_end_effector_velocity_command(r_hand_joints, diff_hand_joints)
        twist *= diff.fps
        success = twist_command(base, twist, 1 / diff.fps)
        print(success)

        #TODO: Control gripper closed
        info = "Closing the gripper " + str((1 - gripper) * 100) + "%..."
        rospy.loginfo(info)
        success = gripper_command(base, (1 - gripper))
        print(success)
        #success = True
        return success



def main():
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import ros_control.utilities as utilities

    # Parse arguments
    args = utilities.parseConnectionArguments()
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        # Create required services
        base = BaseClient(router)
        
        rospy.init_node('top_down_arm_movement')

        # Example core
        success = True
        #success &= example_move_to_home_position(base)
        success &= Initialize_pose(base)


        #setup mocap client
        mocap_joint_service_name = '/mocap_joints'
        rospy.wait_for_service(mocap_joint_service_name)
        mocap_service = rospy.ServiceProxy(mocap_joint_service_name, mocap_joints)
        while not rospy.is_shutdown():
            success &= mocap(base, mocap_service)

            if not success:
                print("The twistcommand encountered an error.")
                return 1

        # terget_twist_list = [[0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
        #                      [0.0, 0.1, 0.0, 0.0, 0.0, 0.0],
        #                      [-0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
        #                      [0.0, -0.1, 0.0, 0.0, 0.0, 0.0],
        #                      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #                      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
        # while True:
        #     for terget_twist in terget_twist_list:
        #         success &= twist_command(base, terget_twist)
        #         if not success:
        #             print("The example encountered an error.")
        #             return 1


        return 0 if success else 1

if __name__ == "__main__":
    exit(main())
