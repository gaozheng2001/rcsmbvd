import os
import sys
import time
import cv2
import numpy as np
import torch
import argparse

from frankmocap.arges import FrankMocapArges
from frankmocap.bodymocap.body_mocap_api import BodyMocap
from frankmocap.handmocap.hand_bbox_detector import HandBboxDetector
from frankmocap.handmocap.hand_mocap_api import HandMocap

sys.path.append('../openpose/build/python')
from openpose import pyopenpose as op


output_dir = './output/'
image_ext = ['bmp', 'dib', 'jpeg', 'jpg', 'jpe', 
             'png', 'pbm', 'pgm', 'ppm', 'sr', 
             'ras', 'tiff', 'tif', 'exr', 'jp2']
video_ext = ['avi', 'mp4', 'mov', 'mkv', 'webm']

def isimg(filename):
    return filename.split('.')[-1].lower() in image_ext

def init_args()->argparse.Namespace:
    # Flags
    parser = argparse.ArgumentParser()
    # input
    parser.add_argument("--image_dir", default="../openpose/examples/media/", help="Process an image dir. Read all standard formats (jpg, png, bmp, etc.).")
    parser.add_argument("--input_type", '-t', required=True, help="Input type: image, video, webcam.")
    parser.add_argument("--image_path", default="./test_data/2023-03-22-232247.jpg", help="Process an image. Read all standard formats (jpg, png, bmp, etc.).")
    parser.add_argument("--video_path", default="./sample_data/han_hand_long.mp4", help="Process a video. Read all standard formats (avi, mp4, mov, etc.).")
    
    
    ### openpose set
    parser.add_argument("--openpose_model_folder", default="../openpose/models/", help="Folder path (absolute or relative) where the models (pose, face, ...) are located.")
    parser.add_argument("--openpose_model_pose", default="BODY_25", help="Model to be used. E.g. `COCO` (18 keypoints), \
                        `MPI` (15 keypoints, ~10% faster), `MPI_4_layers` (15 keypoints, even faster but less accurate).")
    parser.add_argument("--openpose_3d", default=False, help="Enable OpenPose 3D.")
    parser.add_argument("--openpose_output_type", default="image", help="Output type: image, heatmaps or keypoint.")
    parser.add_argument("--openpose_net_resolution", default="-1x176", help="Output resolution: The OpenPose default is -1x-1. \
                        Change the resolution of the final output (display and/or save depending on the display flag). \
                        Use \"-1x-1\" to force the program to use the input image resolution.")
    


    ### frankmocap set
    # set default checkpoint
    # parser.add_argument('--checkpoint', required=False, default=default_checkpoint, help='Path to pretrained checkpoint')
    default_checkpoint_body_smpl ='./extra_data/body_module/pretrained_weights/2020_05_31-00_50_43-best-51.749683916568756.pt'
    parser.add_argument('--checkpoint_body_smpl', required=False, default=default_checkpoint_body_smpl, help='Path to pretrained checkpoint')
    default_checkpoint_body_smplx ='./extra_data/body_module/pretrained_weights/smplx-03-28-46060-w_spin_mlc3d_46582-2089_2020_03_28-21_56_16.pt'
    parser.add_argument('--checkpoint_body_smplx', required=False, default=default_checkpoint_body_smplx, help='Path to pretrained checkpoint')
    default_checkpoint_hand = "./extra_data/hand_module/pretrained_weights/pose_shape_best.pth"
    parser.add_argument('--checkpoint_hand', required=False, default=default_checkpoint_hand, help='Path to pretrained checkpoint')
    
    # frankmocap input options
    parser.add_argument('--input_path', type=str, default=None, help="""Path of video, image, or a folder where image files exists""")
    parser.add_argument('--start_frame', type=int, default=0, help='given a sequence of frames, set the starting frame')
    parser.add_argument('--end_frame', type=int, default=float('inf'), help='given a sequence of frames, set the last frame')
    parser.add_argument('--pkl_dir', type=str, help='Path of storing pkl files that store the predicted results')
    parser.add_argument('--openpose_dir', type=str, help='Directory of storing the prediction of openpose prediction')
    
    # frankmocap output options
    parser.add_argument('--out_dir', type=str, default='./mocap_output', help='Folder of output images.')
    # parser.add_argument('--pklout', action='store_true', help='Export mocap output as pkl file')
    parser.add_argument('--save_bbox_output', action='store_true', help='Save the bboxes in json files (bbox_xywh format)')
    parser.add_argument('--save_pred_pkl', action='store_true', help='Save the predictions (bboxes, params, meshes in pkl format')
    parser.add_argument("--save_mesh", action='store_true', help="Save the predicted vertices and faces")
    parser.add_argument("--save_frame", action='store_true', help='Save the extracted frames from video input or webcam')
    
    # Other frankmocap options
    parser.add_argument('--single_person', action='store_true', help='Reconstruct only one person in the scene with the biggest bbox')
    parser.add_argument('--no_display', action='store_true', help='Do not visualize output on the screen')
    parser.add_argument('--no_video_out', action='store_true', help='Do not merge rendered frames to video (ffmpeg)')
    parser.add_argument('--smpl_dir', type=str, default='./extra_data/smpl/', help='Folder where smpl files are located.')
    parser.add_argument('--skip', action='store_true', help='Skip there exist already processed outputs')
    parser.add_argument('--video_url', type=str, default=None, help='URL of YouTube video, or image.')
    parser.add_argument('--download', '-d', action='store_true', help='Download YouTube video first (in webvideo folder), and process it')
    
    # Body mocap specific options
    parser.add_argument('--use_smplx', default=True, help='Use SMPLX model for body mocap')
    
    # Hand mocap specific options
    parser.add_argument('--view_type', type=str, default='third_view', choices=['third_view', 'ego_centric'],
        help = "The view type of input. It could be ego-centric (such as epic kitchen) or third view")
    parser.add_argument('--crop_type', type=str, default='no_crop', choices=['hand_crop', 'no_crop'],
        help = """ 'hand_crop' means the hand are central cropped in input. (left hand should be flipped to right). 
                    'no_crop' means hand detection is required to obtain hand bbox""")
    
    # Whole motion capture (FrankMocap) specific options
    parser.add_argument('--frankmocap_fast_mode', action='store_true', help="Use fast hand detection mode for whole body motion capture (frankmocap)")
    
    # renderer
    parser.add_argument("--renderer_type", type=str, default="opengl_gui", 
        choices=['pytorch3d', 'opendr', 'opengl_gui', 'opengl'], help="type of renderer to use")


    args = parser.parse_known_args()
    return args[0]


class openposeoutput():
    def __init__(self, type, keypoints):
        '''
        type:
            body,    face,    hand_left,    hand_right, 
            body_3d, face_3d, hand_left_3d, hand_right_3d
        keypoints:
            BODY_25_keypoints: 25 * 3
            COCO_keypoints: 18 * 3
            face_keypoints: 70 * 3
            hand_keypoints: 21 * 3
            for 2D, [x, y, confidence]
            for 3D, [x, y, z, confidence]
        croped: 
            croped image
        bounding_box: 
            (min_x, min_y), (max_x, max_y), (min_z, max_z)(for 3d)
        '''
        self.type = type
        self.keypoints = keypoints
        self.croped = []
        self.bounding_box = []

def get_openpose_params(args:argparse.Namespace = None):
    # Custom Params (refer to include/openpose/flags.hpp for more parameters)
    params = dict()
    params["model_folder"] = args.openpose_model_folder
    
    # params["face"] = True
    params["hand"] = True
    #params["hand_detector"] = 2
    #params["body"] = 0

    # select model
    params['model_pose'] = args.openpose_model_pose

    # enable/disable openpose 3d
    if args.openpose_3d:
        params['3d'] = 1

    # reduce model accuracy
    params['net_resolution'] = args.openpose_net_resolution

    # select output type and output directory
    #params['write_images'] = './output_images/'
    #params['write_images_format'] = 'jpg'

    # Set the border thickness of the callout box
    pad = 5

    # Custom Params (refer to include/openpose/flags.hpp for more parameters)
    if args.openpose_output_type == 'heatmaps':
        params["heatmaps_add_parts"] = True
        params["heatmaps_add_bkg"] = True
        params["heatmaps_add_PAFs"] = True
        params["heatmaps_scale"] = 2

    input_path_dict = {'image': args.image_path,
                       'image_dir': args.image_dir, 
                       'video': args.video_path, 
                       'webcam': 0}

    return params, args.input_type, input_path_dict[args.input_type], args.openpose_output_type

def process_input(input_type, input_path, process_num):
    if input_type == 'image':
        assert os.path.isfile(input_path), "Image file not found: {}".format(input_path)
        assert isimg(input_path), "Image file not supported: {}".format(input_path)
        return cv2.imread(input_path)
    
    elif input_type == 'image_dir':
        inputs = os.listdir(input_path)
        if process_num < len(inputs):
            assert isimg(inputs[process_num]), "Image file not supported: {}".format(inputs[process_num])
            return cv2.imread(os.path.join(input_path, inputs[process_num]))
        else:
            return None
    elif input_type == 'video':
        assert os.path.isfile(input_path), "Video file not found: {}".format(input_path)
        assert input_path.split('.')[-1].lower() in video_ext, "Video file not supported: {}".format(input_path)
        
        cap = cv2.VideoCapture(input_path)
        cap.set(cv2.CAP_PROP_POS_FRAMES, process_num)
        return cap
    elif input_type == 'webcam':
        cap = cv2.VideoCapture(0)
        return cap
    else:
        raise ValueError("Input type not supported: {}".format(input_type))
    
def heatmap_from_openpose(datum):
    print("--------------------Start heatmaps Process-------------------------")
    outputImageF = (datum.inputNetData[0].copy())[0,:,:,:] + 0.5
    outputImageF = cv2.merge([outputImageF[0,:,:], outputImageF[1,:,:], outputImageF[2,:,:]])
    outputImageF = (outputImageF*255.).astype(dtype='uint8')
    heatmaps = datum.poseHeatMaps.copy()
    heatmaps = (heatmaps).astype(dtype='uint8')
    for i in range(0, heatmaps.shape[0]):
        heatmap = heatmaps[i, :, :].copy()
        heatmap = cv2.applyColorMap(heatmap, cv2.COLORMAP_JET)
        combined = cv2.addWeighted(outputImageF, 0.5, heatmap, 0.5, 0)
        print("Body Part heatmap: " + str(i))
        print("-------------------------------------------------------------------")
        cv2.imshow("OpenPose 1.7.0 - heatmap", combined)
        cv2.waitKey(0)
    print("-------------------finish heatmaps Process-------------------------")
    return heatmaps

def get_item_box(frame, item, pad=5):
    '''
    BODY_25_keypoints: 25 * 3
    COCO_keypoints: 18 * 3
    face_keypoints: 70 * 3
    hand_keypoints: 21 * 3
    for 2D, [x, y, confidence]
    for 3D, [x, y, z, confidence]
    '''
    x = item[:, 0]
    y = item[:, 1]
    z = item[:, 2] if item.shape[1] == 4 else None
    min_x = int(np.min(x) - pad if np.min(x) - pad > 0 else 0)
    max_x = int(np.max(x) + pad if np.max(x) + pad < frame.shape[1] else frame.shape[1])
    min_y = int(np.min(y) - pad if np.min(y) - pad > 0 else 0)
    max_y = int(np.max(y) + pad if np.max(y) + pad < frame.shape[0] else frame.shape[0])
    min_z = int(np.min(z) - pad) if z is not None else None
    max_z = int(np.max(z) + pad) if z is not None else None
    return (min_x, min_y), (max_x, max_y), (min_z, max_z)

def crop_item(frame, datum, item_type, draw_box=False):
    '''
    item_type: body,    face,    hand_left,    hand_right, 
               body_3d, face_3d, hand_left_3d, hand_right_3d
    '''
    item_types = {'body':datum.poseKeypoints,
                  'face':datum.faceKeypoints,
                  'hand_left':datum.handKeypoints[0],
                  'hand_right':datum.handKeypoints[1],
                  'body_3d':datum.poseKeypoints3D,
                  'face_3d':datum.faceKeypoints3D,
                  'hand_left_3d':datum.handKeypoints3D[0],
                  'hand_right_3d':datum.handKeypoints3D[1]}
    items = item_types[item_type]
    op_out = openposeoutput(item_type, items)
    if draw_box and (item_type == 'hand_left' or item_type == 'hand_right'):
        box_color = (0, 255, 0) if item_type == 'hand_left' else (255, 0, 0)
    if items is not None:
        for item in items:
            if np.max(item[:, -1]) < 1e-5:continue
            item_box = get_item_box(frame, item)
            (min_x, min_y), (max_x, max_y), _ = item_box
            if max_x - min_x < 10 or max_y - min_y < 10:continue
            if draw_box:
                cv2.rectangle(frame, (min_x, min_y), (max_x, max_y), box_color, 2)
            op_out.croped.append(frame[min_y:max_y, min_x:max_x, :])
            op_out.bounding_box.append(item_box)
    return op_out

def run(params, input_type, input_path, output_type,
        args, hand_bbox_detector, body_mocap, hand_mocap, visualizer
        ):
    # Starting OpenPose
    opWrapper = op.WrapperPython()
    opWrapper.configure(params)
    opWrapper.start()
    
    print("--------------------Start Openpose Process-------------------------")

    process_num = 0
    cur_frame = args.start_frame
    video_frame = 0
    while True:
        # Process input data
        if input_type != 'image' and input_type != 'image_dir':
            cap = process_input(input_type, input_path, process_num)
            if cap is None:break
            ret, imageToProcess = cap.read()
            if not ret:break

        else:
            imageToProcess = process_input(input_type, input_path, process_num)
            if imageToProcess is None:break

        process_num += 1
        start = time.time()

        # Process and display image
        datum = op.Datum()
        datum.cvInputData = imageToProcess
        opWrapper.emplaceAndPop(op.VectorDatum([datum]))

        # Process heatmaps from image outputs
        if output_type == 'heatmaps':
            heatmaps = heatmap_from_openpose(datum)
            break
        
        # print("Body keypoints: \n" + str(datum.poseKeypoints))
        # print("Face keypoints: \n" + str(datum.faceKeypoints))
        # print("Left hand keypoints: \n" + str(datum.handKeypoints[0]))
        #  print("Right hand keypoints: \n" + str(datum.handKeypoints[1]))

        # output_image with keypoints rendered
        image_out = datum.cvOutputData.copy()
        left_hands = crop_item(image_out, datum, 'hand_left', draw_box=True)
        right_hands = crop_item(image_out, datum, 'hand_right', draw_box=True)

        # save croped hands
        if True:
            for crop_left_hand in left_hands.croped:
                cv2.imwrite(f'./output_images/{time.time()}-left-hand-output.jpg', crop_left_hand)
            for crop_right_hand in right_hands.croped:
                cv2.imwrite(f'./output_images/{time.time()}-right-hand-output.jpg', crop_right_hand)

        cv2.imshow("OpenPose 1.7.0", image_out)
        print("FPS for openpose: ", 1.0 / (time.time() - start))
        print("-------------------------------------------------------------------")
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Process Mocap

    cap.release()
    cv2.destroyAllWindows()
    print("-----------------Openpose Processing Ending------------------------")


def init_frankmocap(args: argparse.Namespace):
    device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
    assert torch.cuda.is_available(), "Current version only supports GPU"

    hand_bbox_detector =  HandBboxDetector('third_view', device)
    
    #Set Mocap regressor
    body_mocap = BodyMocap(args.checkpoint_body_smplx, args.smpl_dir, device = device, use_smplx= True)
    hand_mocap = HandMocap(args.checkpoint_hand, args.smpl_dir, device = device)

    # Set Visualizer
    if args.renderer_type in ['pytorch3d', 'opendr']:
        from frankmocap.renderer.screen_free_visualizer import Visualizer
    else:
        from frankmocap.renderer.visualizer import Visualizer
    visualizer = Visualizer(args.renderer_type)

    return hand_bbox_detector, body_mocap, hand_mocap, visualizer


def main():
    args = init_args()
    params, input_type, input_path , output_type= get_openpose_params(args)

    # init frankmocap
    hand_bbox_detector, body_mocap, hand_mocap, visualizer = init_frankmocap(args)

    # Starting process
    run(params, input_type, input_path, output_type, 
        args, hand_bbox_detector, body_mocap, hand_mocap, visualizer)

if __name__ == "__main__":
    main()
