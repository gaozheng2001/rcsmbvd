#import sys
import rospy
import time

from kortex_driver.srv import *
from kortex_driver.msg import *
from math import pi
import numpy as np

from utils.process_joints import *

from rcsmbvd.srv import mocap_joints
from rcsmbvd.msg import *


class ExampleFullArmMovement:
    def __init__(self):
        try:
            rospy.init_node('example_full_arm_movement_python')

            self.HOME_ACTION_IDENTIFIER = 2

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3_lite")
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 6)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

            rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(self.degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(self.is_gripper_present))

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
        
            get_product_configuration_full_name = '/' + self.robot_name + '/base/get_product_configuration'
            rospy.wait_for_service(get_product_configuration_full_name)
            self.get_product_configuration = rospy.ServiceProxy(get_product_configuration_full_name, GetProductConfiguration)

            validate_waypoint_list_full_name = '/' + self.robot_name + '/base/validate_waypoint_list'
            rospy.wait_for_service(validate_waypoint_list_full_name)
            self.validate_waypoint_list = rospy.ServiceProxy(validate_waypoint_list_full_name, ValidateWaypointList)

            #setup mocap client
            mocap_joint_service_name = '/mocap_joints'
            rospy.wait_for_service(mocap_joint_service_name)
            self.mocap = rospy.ServiceProxy(mocap_joint_service_name, mocap_joints)
            
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event
    
    def FillCartesianWaypoint(self, new_x, new_y, new_z, new_theta_x, new_theta_y, new_theta_z, blending_radius):
        waypoint = Waypoint()
        cartesianWaypoint = CartesianWaypoint()

        cartesianWaypoint.pose.x = new_x
        cartesianWaypoint.pose.y = new_y
        cartesianWaypoint.pose.z = new_z
        cartesianWaypoint.pose.theta_x = new_theta_x
        cartesianWaypoint.pose.theta_y = new_theta_y
        cartesianWaypoint.pose.theta_z = new_theta_z
        cartesianWaypoint.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE
        cartesianWaypoint.blending_radius = blending_radius
        waypoint.oneof_type_of_waypoint.cartesian_waypoint.append(cartesianWaypoint)

        return waypoint

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            print(self.last_action_notif_type )
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            elif (self.last_action_notif_type == ActionEvent.ACTION_PREPROCESS_ABORT):
                rospy.loginfo("Received ACTION_PREPROCESS_ABORT notification")
                return False
            else:
                time.sleep(0.01)

    def example_subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)
        return True

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def example_home_the_robot(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        self.last_action_notif_type = None
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()

    def example_set_cartesian_reference_frame(self):
        self.last_action_notif_type = None
        # Prepare the request with the frame we want to set
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED

        # Call the service
        try:
            self.set_cartesian_reference_frame(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")

        # Wait a bit
        rospy.sleep(0.25)
        return True

    def example_send_cartesian_pose(self, x, y, z, theta_x, theta_y, theta_z):
        self.last_action_notif_type = None
        # Get the actual cartesian pose to increment it
        # You can create a subscriber to listen to the base_feedback
        # Here we only need the latest message in the topic though
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)

        # Possible to execute waypointList via execute_action service or use execute_waypoint_trajectory service directly

        trajectory = WaypointList()
   
        trajectory.waypoints.append(
            self.FillCartesianWaypoint(
                x,
                y,
                z,
                theta_x,
                theta_y,
                theta_z,
                0)
        )

        trajectory.duration = 0
        trajectory.use_optimal_blending = False

        req = ExecuteActionRequest()
        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)

        # Call the service
        rospy.loginfo("Sending the robot to the cartesian pose...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteWaypointTrajectory")
            return False
        else:
            print("Waiting for the action to end...")
            return self.wait_for_action_end_or_abort()

    def example_send_joint_angles(self, tolerance=0.01, joint_pose: list[float] = None):
        self.last_action_notif_type = None

        req = ExecuteActionRequest()

        trajectory = WaypointList()
        waypoint = Waypoint()
        angularWaypoint = AngularWaypoint()

        # Angles to send the arm to vertical position (all zeros)
        angularWaypoint.angles = joint_pose

        # for _ in range(self.degrees_of_freedom):
        #     angularWaypoint.angles.append(0.0)

        # Each AngularWaypoint needs a duration and the global duration (from WaypointList) is disregarded. 
        # If you put something too small (for either global duration or AngularWaypoint duration), the trajectory will be rejected.
        angular_duration = 0
        angularWaypoint.duration = angular_duration

        # Initialize Waypoint and WaypointList
        waypoint.oneof_type_of_waypoint.angular_waypoint.append(angularWaypoint)
        trajectory.duration = 0
        trajectory.use_optimal_blending = False
        trajectory.waypoints.append(waypoint)

        
        try:
            res = self.validate_waypoint_list(trajectory)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ValidateWaypointList")
            return False

        error_number = len(res.output.trajectory_error_report.trajectory_error_elements)
        MAX_ANGULAR_DURATION = 30

        print("error_number: ", error_number)

        while (error_number >= 1 and angular_duration != MAX_ANGULAR_DURATION) :
            angular_duration += 1
            trajectory.waypoints[0].oneof_type_of_waypoint.angular_waypoint[0].duration = angular_duration
            print("angular_duration: ", angular_duration)
            try:
                res = self.validate_waypoint_list(trajectory)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ValidateWaypointList")
                return False

            error_number = len(res.output.trajectory_error_report.trajectory_error_elements)

        if (angular_duration == MAX_ANGULAR_DURATION) :
            # It should be possible to reach position within 30s
            # WaypointList is invalid (other error than angularWaypoint duration)
            rospy.loginfo("WaypointList is invalid")
            return False
        
        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)
        
        # Send the angles
        rospy.loginfo("Sending the robot vertical...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteWaypointjectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def example_send_gripper_command(self, value):
        # Initialize the request
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")

        # Call the service 
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            time.sleep(2)
            return True

    def example_cartesian_waypoint_action(self):
        self.last_action_notif_type = None

        req = ExecuteActionRequest()
        trajectory = WaypointList()

        config = self.get_product_configuration()

        if config.output.model == ModelId.MODEL_ID_L31:
        
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.439,  0.194,  0.448, 90.6, -1.0, 150, 0))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.200,  0.150,  0.400, 90.6, -1.0, 150, 0))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.350,  0.050,  0.300, 90.6, -1.0, 150, 0))
        else:
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.7,  0.0,   0.5,  90, 0, 90, 0))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.7,  0.0,   0.33, 90, 0, 90, 0.1))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.7,  0.48,  0.33, 90, 0, 90, 0.1))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.61, 0.22,  0.4,  90, 0, 90, 0.1))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.7,  0.48,  0.33, 90, 0, 90, 0.1))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.63, -0.22, 0.45, 90, 0, 90, 0.1))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.65, 0.05,  0.45, 90, 0, 90, 0))
        
        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)
        
        # Call the service
        rospy.loginfo("Executing Kortex action ExecuteWaypointTrajectory...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call action ExecuteWaypointTrajectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def main(self):
        # For testing purposes
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/full_arm_movement_python")
        except:
            pass

        if success:
            #*******************************************************************************
            # Make sure to clear the robot's faults else it won't move if it's already in fault
            success &= self.example_clear_faults()
            #*******************************************************************************
            
            #*******************************************************************************
            # Activate the action notifications
            success &= self.example_subscribe_to_a_robot_notification()
            #*******************************************************************************

            #*******************************************************************************
            # Move the robot to the Home position with an Action
            success &= self.example_home_the_robot()
            #*******************************************************************************

            #*******************************************************************************
            # Example of gripper command
            # Let's fully open the gripper
            if self.is_gripper_present:
                success &= self.example_send_gripper_command(0.0)
            else:
                rospy.logwarn("No gripper is present on the arm.")  
            #*******************************************************************************

            #*******************************************************************************
            # Set the reference frame to "Mixed"
            success &= self.example_set_cartesian_reference_frame()

            # Example of cartesian pose
            # Let's make it move in Z
            #Introduce the debug package
            #import pdb; pdb.set_trace()
            success &= self.example_send_cartesian_pose(0.5, 0.0, 0.1, 180.0, 0.0, 90)
            success &= self.example_send_cartesian_pose(0.5, 0.0, 0.002, 180.0, 0.0, 90)
            if self.is_gripper_present:
                success &= self.example_send_gripper_command(1.0)
            else:
                rospy.logwarn("No gripper is present on the arm.") 
            '''
            if self.is_gripper_present:
                success &= self.example_send_gripper_command(1.0)
            else:
                rospy.logwarn("No gripper is present on the arm.")
            '''
            success &= self.example_send_cartesian_pose(0.5, 0.0, 0.1, 180.0, 0.0, 90)
            success &= self.example_send_cartesian_pose(0.3, 0.0, 0.1, 180.0, 0.0, 90)
            success &= self.example_send_cartesian_pose(0.3, 0.0, 0.002, 180.0, 0.0, 90)
            if self.is_gripper_present:
                success &= self.example_send_gripper_command(0.0)
            else:
                rospy.logwarn("No gripper is present on the arm.")
            success &= self.example_send_cartesian_pose(0.3, 0.0, 0.1, 180.0, 0.0, 90)

            #*******************************************************************************

            #*******************************************************************************
            # Example of angular position
            # Let's send the arm to vertical position
            # success &= self.example_send_joint_angles()
            #*******************************************************************************

            #*******************************************************************************
            # Example of gripper command
            # Let's close the gripper at 50%
            #if self.is_gripper_present:
            #    success &= self.example_send_gripper_command(0.0)
            #else:
            #    rospy.logwarn("No gripper is present on the arm.")    
            #*******************************************************************************
        
            #*******************************************************************************
            # Move the robot to the Home position with an Action
            success &= self.example_home_the_robot()
            #*******************************************************************************

            #*******************************************************************************
            # Example of waypoint
            # Let's move the arm
            #success &= self.example_cartesian_waypoint_action()

            #*******************************************************************************
            # Move the robot to the Home position with an Action
            #success &= self.example_home_the_robot()
            #*******************************************************************************

        # For testing purposes
        rospy.set_param("/kortex_examples_test_results/full_arm_movement_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")



def mocap(full_arm):
    global success
    #import pdb; pdb.set_trace()
    mocap_joint = full_arm.mocap()
    rospy.loginfo('get mocap joints')
    mocap_process(mocap_joint.mocap_data)
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def mocap_process(joints):
    global success, full_arm, reached_frame, total_frame
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
    full_follow = 1
    if full_follow == 1:
        rob_joint_1_3 = get_rob_arm_joint_angles(r_arm_joints)
    
        rob_joint_1, rob_joint_2, rob_joint_3, raw_rob_arm = rob_joint_1_3
        
        #TODO: mapping right hand to kinova Gen3 Lite
        rob_joint_4_6 = get_rob_hand_joint_angles(r_hand_joints, rob_joint_1_3)
        rob_joint_4, rob_joint_5, rob_joint_6, gripper = rob_joint_4_6
    
        rob_joint = [rob_joint_1, rob_joint_2, rob_joint_3, rob_joint_4, rob_joint_5, rob_joint_6]
    
        for i in range(len(rob_joint)):
            rob_joint[i]  = rob_joint[i] * 180 / pi

        total_frame += 1
        print('Execute the ' + str(total_frame) + 'th command')
        print('-------------------------------------------------------------')
        info = "Reaching Mocap Joint Angles:" + str(rob_joint)
        rospy.loginfo(info)
        success = full_arm.example_send_joint_angles(tolerance=0.01, joint_pose=rob_joint) #rad
        if not success:
            print('-----------------------------------------------------')
            print('Failed to reach the Mocap Joint Angles')
        else:
            reached_frame += 1
        print(success)
        info = "Closing the gripper " + str((1 - gripper) * 100) + "%..."
        rospy.loginfo(info)
        success = full_arm.example_send_gripper_command(1 - gripper)
        print(success)
        print("Success rate: " + str(reached_frame) + "/" + str(total_frame))
    else:
        hand_center, gripper = top_down_follow(r_hand_joints)
        print(hand_center, gripper)
        print(success)

        success = full_arm.example_send_cartesian_pose(hand_center[0], hand_center[1], hand_center[2], 0.0, 0.0, 0.0)
        info = "Closing the gripper " + str((1 - gripper) * 100) + "%..."
        rospy.loginfo(info)
        success = full_arm.example_send_gripper_command(1 - gripper)
        print(success)


def main():
    global success, full_arm
    full_arm = ExampleFullArmMovement()

    # For testing purposes
    success = full_arm.is_init_success
    if success:
        #*******************************************************************************
        # Make sure to clear the robot's faults else it won't move if it's already in fault
        success &= full_arm.example_clear_faults()
        #*******************************************************************************
        
        #*******************************************************************************
        # Activate the action notifications
        success &= full_arm.example_subscribe_to_a_robot_notification()
        #*******************************************************************************

        #*******************************************************************************
        # Move the robot to the Home position with an Action
        success &= full_arm.example_home_the_robot()
        #*******************************************************************************

        #*******************************************************************************
        # Example of gripper command
        # Let's fully open the gripper
        if full_arm.is_gripper_present:
            success &= full_arm.example_send_gripper_command(0.0)
        else:
            rospy.logwarn("No gripper is present on the arm.")  
        #*******************************************************************************

        #*******************************************************************************
        # Set the reference frame to "Mixed"
        success &= full_arm.example_set_cartesian_reference_frame()

        rospy.loginfo("Reaching mocap position...")
        while True:
            mocap(full_arm)

        rospy.spin()


if __name__ == "__main__":
    global reached_frame, total_frame
    reached_frame = 0
    total_frame = 0
    main()
