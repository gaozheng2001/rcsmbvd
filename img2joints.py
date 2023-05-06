# Copyright (c) Facebook, Inc. and its affiliates.

import os
import sys
import os.path as osp
import time
import torch
from torchvision.transforms import Normalize
import numpy as np
import cv2
import argparse
import json
import pickle
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import rospy
from std_msgs.msg import String, Float64MultiArray
from rcsmbvd.msg import mocap_data, mocap_joint_diff
from rcsmbvd.srv import mocap_joints, mocap_jointsResponse
import pyrealsense2 as rs

############# input parameters  #############
from frankmocap.arges import FrankMocapArges
from frankmocap.bodymocap.body_mocap_api import BodyMocap
from frankmocap.handmocap.hand_mocap_api import HandMocap
import frankmocap.mocap_utils.demo_utils as demo_utils
import frankmocap.mocap_utils.general_utils as gnu
from frankmocap.mocap_utils.timer import Timer
from datetime import datetime

from frankmocap.bodymocap.body_bbox_detector import BodyPoseEstimator
from frankmocap.handmocap.hand_bbox_detector import HandBboxDetector
from frankmocap.integration.copy_and_paste import integration_copy_paste

import frankmocap.renderer.image_utils as imu
from frankmocap.renderer.viewer2D import ImShow



"""
      15 16
    17  0  18
        1
    2       5
    3       6
   4    8    7
      9   12
     10   13
     11   14
23  24     21  20
  22         19
"""

rarm = [8, 1, 2, 4]
prarm = [8, 1, 2, 3, 4]
larm = [0, 1, 5, 6, 7]
rleg = [8, 9, 10, 11]
lleg = [8, 12, 13, 14]


def __filter_bbox_list(body_bbox_list, hand_bbox_list, single_person):
    # (to make the order as consistent as possible without tracking)
    bbox_size =  [ (x[2] * x[3]) for x in body_bbox_list]
    idx_big2small = np.argsort(bbox_size)[::-1]
    body_bbox_list = [ body_bbox_list[i] for i in idx_big2small ]
    hand_bbox_list = [hand_bbox_list[i] for i in idx_big2small]

    if single_person and len(body_bbox_list)>0:
        body_bbox_list = [body_bbox_list[0], ]
        hand_bbox_list = [hand_bbox_list[0], ]

    return body_bbox_list, hand_bbox_list


def run_regress(
    args, img_original_bgr, 
    body_bbox_list, hand_bbox_list, bbox_detector,
    body_mocap, hand_mocap
):
    cond1 = len(body_bbox_list) > 0 and len(hand_bbox_list) > 0
    cond2 = not args.frankmocap_fast_mode

    # use pre-computed bbox or use slow detection mode
    if cond1 or cond2:
        if not cond1 and cond2:
            # run detection only when bbox is not available
            body_pose_list, body_bbox_list, hand_bbox_list, _ = \
                bbox_detector.detect_hand_bbox(img_original_bgr.copy())
        else:
            print("Use pre-computed bounding boxes")
        assert len(body_bbox_list) == len(hand_bbox_list)

        if len(body_bbox_list) < 1: 
            return list(), list(), list()

        # sort the bbox using bbox size 
        # only keep on bbox if args.single_person is set
        body_bbox_list, hand_bbox_list = __filter_bbox_list(
            body_bbox_list, hand_bbox_list, args.single_person)

        # hand & body pose regression
        pred_hand_list = hand_mocap.regress(
            img_original_bgr, hand_bbox_list, add_margin=True)
        pred_body_list = body_mocap.regress(img_original_bgr, body_bbox_list)
        assert len(hand_bbox_list) == len(pred_hand_list)
        assert len(pred_hand_list) == len(pred_body_list)

    else:
        _, body_bbox_list = bbox_detector.detect_body_bbox(img_original_bgr.copy())

        if len(body_bbox_list) < 1: 
            return list(), list(), list()

        # sort the bbox using bbox size 
        # only keep on bbox if args.single_person is set
        hand_bbox_list = [None, ] * len(body_bbox_list)
        body_bbox_list, _ = __filter_bbox_list(
            body_bbox_list, hand_bbox_list, args.single_person)

        # body regression first 
        pred_body_list = body_mocap.regress(img_original_bgr, body_bbox_list)
        assert len(body_bbox_list) == len(pred_body_list)

        # get hand bbox from body
        hand_bbox_list = body_mocap.get_hand_bboxes(pred_body_list, img_original_bgr.shape[:2])
        assert len(pred_body_list) == len(hand_bbox_list)

        # hand regression
        pred_hand_list = hand_mocap.regress(
            img_original_bgr, hand_bbox_list, add_margin=True)
        assert len(hand_bbox_list) == len(pred_hand_list) 

    # integration by copy-and-paste
    integral_output_list = integration_copy_paste(
        pred_body_list, pred_hand_list, body_mocap.smpl, img_original_bgr.shape)
    
    return body_bbox_list, hand_bbox_list, integral_output_list


def handle_mocap(req):
    return mocap_jointsResponse(detect_mocap, detect_mocap_diff)


def run_frank_mocap(args, bbox_detector, body_mocap, hand_mocap):
    global detect_mocap, detect_mocap_diff
    #Setup input data to handle different types of inputs
    input_type, input_data = demo_utils.setup_input(args)

    if input_type == 'd455':
        pipeline = rs.pipeline()
        pipeline.start()


    cur_frame = args.start_frame
    video_frame = 0

    last_r_arm_joints = None
    last_r_hand_joints = None
    
    show_joint = 1
    show_plt = 0

    if show_joint ==1 and show_plt == 1:
        fig = plt.figure(figsize=(7, 10))
        plt.show(block=False)
        plt.ion()
        ax = fig.add_subplot(projection='3d')
        ax.set_xlim3d(-100, 700)
        ax.set_ylim3d(200, 1000)
        ax.set_zlim3d(-350, 350)
        ax.set_xscale('linear')
        ax.set_yscale('linear')
        ax.set_zscale('linear')
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')
    rospy.init_node('mocap_server')
    mocap = rospy.Service('mocap_joints', mocap_joints, handle_mocap)
    # rospy.init_node('talker', anonymous=True)
    # pub2ros = rospy.Publisher('mocap', Float64MultiArray, queue_size=10)
    # rosrate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
        start_time = time.time()
        # load image
        if input_type == 'video':
            _, img_original_bgr = input_data.read()
            if video_frame < cur_frame:
                video_frame += 1
                continue
            # save the obtained video frames
            image_path = osp.join(args.out_dir, "frames", f"{cur_frame:05d}.jpg")
            if img_original_bgr is not None:
                video_frame += 1
                if args.save_frame:
                    gnu.make_subdir(image_path)
                    cv2.imwrite(image_path, img_original_bgr)
        elif input_type == 'webcam':
            _, img_original_bgr = input_data.read()

            if video_frame < cur_frame:
                video_frame += 1
                continue
            # save the obtained video frames
            image_path = osp.join(args.out_dir, "frames", f"scene_{cur_frame:05d}.jpg")
            if img_original_bgr is not None:
                video_frame += 1
                if args.save_frame:
                    gnu.make_subdir(image_path)
                    cv2.imwrite(image_path, img_original_bgr)
        elif input_type == 'd455':
            frames = pipeline.wait_for_frames()

            frame = frames.get_color_frame()
            img_original_bgr = np.asanyarray(frame.get_data())

            depth = frames.get_depth_frame()
            if not depth: continue
            img_original_depth = np.asanyarray(depth.as_frame().get_data())
            
            if video_frame < cur_frame:
                video_frame += 1
                continue
            # save the obtained video frames
            image_path = osp.join(args.out_dir, "frames", f"d455_scene_{cur_frame:05d}.jpg")
            if img_original_bgr is not None:
                video_frame += 1
                if args.save_frame:
                    gnu.make_subdir(image_path)
                    cv2.imwrite(image_path, img_original_bgr)
            #import pdb; pdb.set_trace()
        else:
            assert False, "Unknown input_type"

        # make sure the image is not empty
        cur_frame +=1
        if img_original_bgr is None or cur_frame > args.end_frame:
            break
        
        # bbox detection
        body_bbox_list, hand_bbox_list = list(), list()
        
        # regression (includes integration)
        body_bbox_list, hand_bbox_list, pred_output_list = run_regress(
            args, img_original_bgr, 
            body_bbox_list, hand_bbox_list, bbox_detector,
            body_mocap, hand_mocap)
        if pred_output_list is None or len(pred_output_list) < 1 \
        or pred_output_list[0]['pred_body_joints_img'] is None \
        or pred_output_list[0]['pred_rhand_joints_img'] is None:
            continue
        
        rospy.loginfo("--------------------------------------")

        # get the 3D pose from mocap
        body_joints = pred_output_list[0]['pred_body_joints_img'][:25] # (49, 3)
        lhand_joints = pred_output_list[0]['pred_lhand_joints_img']
        rhand_joints = pred_output_list[0]['pred_rhand_joints_img'] # (21, 3)

        pra = body_joints[prarm]
        pla = body_joints[larm]
        prl = body_joints[rleg]
        pll = body_joints[lleg]

        # get the selected right arm joints
        rarm_joint = body_joints[rarm] # (4, 3)
        
        # reshape the joints to (1, -1)
        r_arm_joints = rarm_joint.reshape(1, -1)
        r_hand_joints = rhand_joints.reshape(1, -1)

        detect_fps = 1.0 / (time.time() - start_time)
        rospy.loginfo("detect_fps: {}".format(detect_fps))
        detect_mocap = mocap_data()
        detect_mocap.body_joints = r_arm_joints.tolist()[0]
        detect_mocap.rhand_joints = r_hand_joints.tolist()[0]
        
        detect_mocap_diff = mocap_joint_diff()
        if last_r_arm_joints is not None and last_r_hand_joints is not None:
            detect_mocap_diff.valid = 1
            detect_mocap_diff.fps = detect_fps
            detect_mocap_diff.body_joints_diff = np.subtract(r_arm_joints, last_r_arm_joints).tolist()[0]
            detect_mocap_diff.rhand_joints_diff = np.subtract(r_hand_joints, last_r_hand_joints).tolist()[0]
        else:
            detect_mocap_diff.valid = 0

        # update the last joints
        last_r_arm_joints = r_arm_joints.copy()
        last_r_hand_joints = r_hand_joints.copy()
        # # publish the data to ROS
        # mocapdata = Float64MultiArray()
        # mocapdata.data = np.concatenate((r_arm_joints, r_hand_joints), axis=None).tolist()
        # loginfo = 'publishing mocap data: right arm ' + str(rarm_joint.shape) + ' and hand ' + str(rhand_joints.shape)
        # rospy.loginfo(loginfo)
        # #print(mocap_data.data)
        # pub2ros.publish(mocapdata)
        # #print(mocap_data.data)
        # rosrate.sleep()
        
        # show the joints
        if show_joint == 1:
            for body_joint in body_joints:
                if body_joint is not None and body_joint[0] < img_original_bgr.shape[1] and body_joint[1] < img_original_bgr.shape[0]:
                    cv2.circle(img_original_bgr, (int(body_joint[0]), int(body_joint[1])), 5, (0, 0, 255), -1)
            for rh_joint in rhand_joints:
                if rh_joint is not None and rh_joint[0] < img_original_bgr.shape[1] and rh_joint[1] < img_original_bgr.shape[0]:
                    cv2.circle(img_original_bgr, (int(rh_joint[0]), int(rh_joint[1])), 5, (0, 255, 0), -1)
            cv2.imshow('img_original_bgr', img_original_bgr)
            if show_plt == 1:
                plt.cla()
                ax.plot(pra[:, 0], pra[:, 1], pra[:, 2], c='r')
                ax.plot(pla[:, 0], pla[:, 1], pla[:, 2], c='r')
                ax.plot(prl[:, 0], prl[:, 1], prl[:, 2], c='r')
                ax.plot(pll[:, 0], pll[:, 1], pll[:, 2], c='r')

                #ax.plot(body_joints[:, 0], body_joints[:, 1], -1 * body_joints[:, 2], 'o', c='b')

                plt.pause(0.001)
                plt.show()
                
            cv2.waitKey(1)

        if len(body_bbox_list) < 1: 
            rospy.loginfo(f"No body deteced: {image_path}")
            continue

        rospy.loginfo(f"Processed : {image_path}")
        process_fps = 1.0 / (time.time() - start_time)
        rospy.loginfo(f"process_fps: {process_fps}")

    if input_type =='webcam' and input_data is not None:
        input_data.release()

    if input_type =='d455' and pipeline is not None:
        pipeline.stop()

    if show_joint ==1 and show_plt == 1:
        plt.ioff()
        plt.show()

    cv2.destroyAllWindows()

def main():
    args = FrankMocapArges().parse()
    #args.input_path = 'sample_data/han_hand_long.mp4'
    # args.input_path = 'sample_data/my_test.mp4'
    # args.input_path = 'd455'
    # args.input_path = 'sample_data/top_down_test.mp4'

    
    # args.input_path = 'sample_data/single_totalbody.mp4'
    # args.input_path = 'sample_data/han_long.mp4'
    # args.input_path = 'sample_data/000.mp4'
    # args.input_path = 'sample_data/my_forword.mp4'
    # args.input_path = 'sample_data/my_sideword.mp4'
    # args.input_path = 'sample_data/my_sideword_grapper.mp4'
    # args.input_path = 'sample_data/my_grapper.mp4'
    # args.input_path = 'webcam'

    args.out_dir = 'mocap_output'
    args.use_smplx = True
    #args.save_pred_pkl = 'mocap_output/mocap'
    #args.use_smplx = False

    device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
    assert torch.cuda.is_available(), "Current version only supports GPU"

    hand_bbox_detector =  HandBboxDetector('third_view', device)
    
    #Set Mocap regressor
    body_mocap = BodyMocap(args.checkpoint_body_smplx, args.smpl_dir, device = device, use_smplx= True)
    hand_mocap = HandMocap(args.checkpoint_hand, args.smpl_dir, device = device)

    run_frank_mocap(args, hand_bbox_detector, body_mocap, hand_mocap)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
