import _thread

from matplotlib import pyplot as plt
import rospy
from rcsmbvd.msg import *
from utils.process_joints import *

import numpy as np



def callback(data):
    global rob_hands, finger_1, finger_2, finger_3, finger_4, finger_5, hand_body, hand_center, calculate_rob_arms, raw_rob_arm, hand_init
    r_5 = 28.5 * 2 / 1000
    r_6 = (130 +105) / 1000
    r_hand_joints = np.array(data.hands).reshape(-1, 3)
    raw_rob_arm = np.array(data.real_arm).reshape(-1, 3)
    rob_joint = data.cal_joints

    hand_center = np.sum(r_hand_joints[1:], axis=0) / 20
    hand_vertical = hand_center - r_hand_joints[0]
    hand_vertical = Scale(hand_vertical, np.sqrt(r_5**2 + r_6**2))

    palm_facing = r_hand_joints[9] - r_hand_joints[4]
    palm_facing_I = np.cross(hand_vertical, np.cross(hand_vertical, palm_facing))
    palm_facing_I = palm_facing_I / np.linalg.norm(palm_facing_I) * 0.05

    rob_hands = np.array([raw_rob_arm[-1], raw_rob_arm[-1] + hand_vertical, raw_rob_arm[-1] + hand_vertical + palm_facing_I]).reshape(3, 3)


    calculate_rob_arms = []
    rob_joint = np.array(rob_joint)
    calculate_rob_arms.append([0.0, 0.0, 0.0])
    for i in range(1, 7):
        T = transformation_matrix(rob_joint[:i], 0, i)
        calculate_rob_arms.append(T[:-1, -1].T)
        print(T[:-1, -1].T)
    calculate_rob_arms = np.array(calculate_rob_arms)
    raw_rob_arm = np.array(raw_rob_arm)

    r_hand_joints = r_hand_joints - r_hand_joints[0] + raw_rob_arm[-1]
    hand_center = np.sum(r_hand_joints[1:], axis=0) / 20
    finger_1 = r_hand_joints[:5]
    finger_2 = r_hand_joints[5:9]
    finger_3 = r_hand_joints[9:13]
    finger_4 = r_hand_joints[13:17]
    finger_5 = r_hand_joints[17:]
    hand_body = r_hand_joints[[0, 5, 9, 13, 17, 0]]

    hand_init = True
    print('call_rosspin: callback done'
          '\n rob_hands: ', rob_hands,
          '\n finger_1: ', finger_1,
          '\n finger_2: ', finger_2,
          '\n finger_3: ', finger_3,
          '\n finger_4: ', finger_4,
          '\n finger_5: ', finger_5,
          '\n hand_body: ', hand_body,
          '\n hand_center: ', hand_center,
          '\n calculate_rob_arms: ', calculate_rob_arms,
          '\n raw_rob_arm: ', raw_rob_arm
          )

    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def ros_callback_wait():
    print('call_rosspin: running rosspin')
    rospy.spin()

if __name__ == '__main__':

    rob_hands = np.zeros((3, 3))
    finger_1 = np.zeros((4, 3))
    finger_2 = np.zeros((4, 3))
    finger_3 = np.zeros((4, 3))
    finger_4 = np.zeros((4, 3))
    finger_5 = np.zeros((4, 3))
    hand_body =  np.zeros((6, 3))
    hand_center = np.zeros((1, 3))
    calculate_rob_arms = np.zeros((7, 3))
    raw_rob_arm = np.zeros((4, 3))
    hand_init = False

    rospy.init_node('listener', anonymous=True)
    fig = plt.figure(figsize=(7, 10))
    plt.show(block=False)
    plt.ion()
    ax = fig.add_subplot(projection='3d')


    rospy.Subscriber("/12345", draw_joints, callback)
    _thread.start_new_thread(ros_callback_wait, ())

    while not rospy.is_shutdown():
        if hand_init:
            hand_init = False

            plt.cla()
            ax.set_xlim3d(-0.3, 0.7)
            ax.set_ylim3d(-0.5, 0.5 )
            ax.set_zlim3d(-0.2, 0.8)
            ax.set_xscale('linear')
            ax.set_yscale('linear')
            ax.set_zscale('linear')
            ax.set_xlabel('X Label')
            ax.set_ylabel('Y Label')
            ax.set_zlabel('Z Label')
            ax.view_init(30, 30)
            
            ax.plot(rob_hands[:, 0], rob_hands[:, 1], rob_hands[:, 2], c='k')
            ax.plot(finger_1[:, 0], finger_1[:, 1], finger_1[:, 2], c='g')
            ax.plot(finger_2[:, 0], finger_2[:, 1], finger_2[:, 2], c='g')
            ax.plot(finger_3[:, 0], finger_3[:, 1], finger_3[:, 2], c='g')
            ax.plot(finger_4[:, 0], finger_4[:, 1], finger_4[:, 2], c='g')
            ax.plot(finger_5[:, 0], finger_5[:, 1], finger_5[:, 2], c='g')
            ax.plot(hand_body[:, 0], hand_body[:, 1], hand_body[:, 2], c='r')
            ax.plot(hand_center[0], hand_center[1], hand_center[2], c='b', marker='o')
            ax.plot(calculate_rob_arms[:, 0], calculate_rob_arms[:, 1], calculate_rob_arms[:, 2], c='#FFC0CB')
            ax.plot(raw_rob_arm[:, 0], raw_rob_arm[:, 1], raw_rob_arm[:, 2], c='k')

            plt.pause(0.001)
            plt.show()
    
    plt.ioff()
    plt.show()


