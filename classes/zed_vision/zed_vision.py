#!/usr/bin/env python3

import sys
import numpy as np

import argparse
import torch
import cv2
import pyzed.sl as sl
import torch.backends.cudnn as cudnn


from ultralytics import YOLO
from ultralytics.nn.tasks import attempt_load_weights
from ultralytics.yolo.utils.ops import non_max_suppression, scale_segments, xyxy2xywh
from ultralytics.yolo.utils.torch_utils import select_device
from ultralytics.yolo.data.dataloaders.v5augmentations import letterbox
#from ultralytics.yolo.data.augment import LetterBox
from ultralytics.yolo.utils.checks import check_imgsz

#sys.path.append("/home/mechatronics/vision_nodes/Mechatronics-2023/classes/zed_vision/yolov5")
#from models.experimental import attempt_load
#from utils.general import check_img_size, non_max_suppression, scale_coords, xyxy2xywh
#from utils.general import check_img_size, non_max_suppression, scale_segments, xyxy2xywh
#from utils.torch_utils import select_device
#from utils.augmentations import letterbox

from threading import Lock, Thread
from time import sleep

# import ogl_viewer.viewer as gl
# import cv_viewer.tracking_viewer as cv_viewer

lock = Lock()
run_signal = False
exit_signal = False

class Zed_Vision():

    def img_preprocess(self, img, device, half, net_size):
        net_image, ratio, pad = letterbox(img[:, :, :3], net_size, auto=False)
        net_image = net_image.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        net_image = np.ascontiguousarray(net_image)

        img = torch.from_numpy(net_image).to(device)
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.
        # 0 - 1.0

        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        return img, ratio, pad


    def xywh2abcd(self, xywh, im_shape):
        output = np.zeros((4, 2))

        # Center / Width / Height -> BBox corners coordinates
        x_min = (xywh[0] - 0.5*xywh[2]) * im_shape[1]
        x_max = (xywh[0] + 0.5*xywh[2]) * im_shape[1]
        y_min = (xywh[1] - 0.5*xywh[3]) * im_shape[0]
        y_max = (xywh[1] + 0.5*xywh[3]) * im_shape[0]

        # A ------ B
        # | Object |
        # D ------ C

        output[0][0] = x_min
        output[0][1] = y_min

        output[1][0] = x_max
        output[1][1] = y_min

        output[2][0] = x_min
        output[2][1] = y_max

        output[3][0] = x_max
        output[3][1] = y_max
        return output


    def detections_to_custom_box(self, detections, im, im0):
        output = []
        for i, det in enumerate(detections):
            if len(det):
                det[:, :4] = scale_segments(im.shape[2:], det[:, :4], im0.shape).round()
                gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh

                for *xyxy, conf, cls in reversed(det):
                    xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh

                    # Creating ingestable objects for the ZED SDK
                    obj = sl.CustomBoxObjectData()
                    obj.bounding_box_2d = self.xywh2abcd(xywh, im0.shape)
                    obj.label = cls
                    obj.probability = conf
                    obj.is_grounded = False
                    output.append(obj)
        return output


    def torch_thread(self, weights, img_size, conf_thres=0.2, iou_thres=0.45):
        global image_net, exit_signal, run_signal, detections

        print("Intializing Network...")

        device = select_device()
        half = device.type != 'cpu'  # half precision only supported on CUDA
        imgsz = img_size

        # Load model
        model = attempt_load_weights(weights, device=device)
                                        
        #model = attempt_load(weights, map_location=lambda storage, loc: storage)
        stride = max(int(model.stride.max()), 32)  # model stride
        imgsz = check_imgsz(imgsz, stride=stride)  # check img_size
        if half:
            model.half()  # to FP16
        cudnn.benchmark = True

        # Run inference
        if device.type != 'cpu':
            model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once

        while not exit_signal:
            if run_signal:
                lock.acquire()
                img, ratio, pad = self.img_preprocess(image_net, device, half, imgsz)

                pred = model(img)[0]
                det = non_max_suppression(pred, conf_thres, iou_thres)

                # ZED CustomBox format (with inverse letterboxing tf applied)
                detections = self.detections_to_custom_box(det, img, image_net)
                lock.release()
                run_signal = False
            sleep(0.01)


    def initCamera(self):
        parser = argparse.ArgumentParser()
        parser.add_argument('--weights', nargs='+', type=str, default='yolov8m.pt', help='model.pt path(s)') #where we put weights at
        parser.add_argument('--svo', type=str, default=None, help='optional svo file')
        parser.add_argument('--img_size', type=int, default=416, help='inference size (pixels)')
        parser.add_argument('--conf_thres', type=float, default=0.4, help='object confidence threshold')
        opt = parser.parse_args()
        print("Initializing Camera...")

        zed = sl.Camera()

        input_type = sl.InputType()
        if opt.svo is not None:
            input_type.set_from_svo_file(opt.svo)

        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters(input_t=input_type, svo_real_time_mode=True)
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.coordinate_units = sl.UNIT.METER
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # QUALITY
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        init_params.depth_maximum_distance = 50

        status = zed.open(init_params)

        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            exit()


        print("Initialized Camera")

        positional_tracking_parameters = sl.PositionalTrackingParameters()
        # If the camera is static, uncomment the following line to have better performances and boxes sticked to the ground.
        # positional_tracking_parameters.set_as_static = True
        zed.enable_positional_tracking(positional_tracking_parameters)

        obj_param = sl.ObjectDetectionParameters()
        obj_param.detection_model = sl.DETECTION_MODEL.CUSTOM_BOX_OBJECTS
        obj_param.enable_tracking = True               

        zed.enable_object_detection(obj_param)

        return zed ,opt


    def updateCamera(self, zed):
        global image_net, exit_signal, run_signal, detections

        runtime_params = sl.RuntimeParameters()

        image_left_tmp = sl.Mat()

        obj_runtime_param = sl.ObjectDetectionRuntimeParameters()

        objects = sl.Objects()

        
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            # -- Get the image
            lock.acquire()
            zed.retrieve_image(image_left_tmp, sl.VIEW.LEFT)
            image_net = image_left_tmp.get_data()
            lock.release()
            run_signal = True

            # -- Detection running on the other thread
            while run_signal:
            # -- Display
                sleep(0.001)

            # Wait for detections
            lock.acquire()
            # -- Ingest detections
            zed.ingest_custom_box_objects(detections)
            lock.release()
            zed.retrieve_objects(objects, obj_runtime_param)

            # object.id
            #object.position

            # for object in objects.object_list:
            #     print("{} {} {}".format(object.id, object.position, object.dimensions))

            for object in objects.object_list:
                # print("{} {} {}".format(object.id, object.position, object.dimensions))
                vector = []
                for coordinate in object.position:
                    vector.append(float(coordinate))
                print("VECTOR: ", vector)
                return vector


        #     object = sl.ObjectData()
        #     objects.get_object_data_from_id(object, 0); # Get the object with ID = O


        #     key = cv2.waitKey(10)
        #     if key == 27:
        #         exit_signal = True
        # else:
        #     exit_signal = True

        # # viewer.exit()
        # exit_signal = True
        # zed.close()


def main():
    global image_net, exit_signal, run_signal, detections

    zed_vision_object = Zed_Vision()
    zed, opt = zed_vision_object.initCamera()
    capture_thread = Thread(target=zed_vision_object.torch_thread,
                            kwargs={'weights': opt.weights, 'img_size': opt.img_size, "conf_thres": opt.conf_thres})
    capture_thread.start()
    zed_vision_object.updateCamera(zed)
    

if __name__ == '__main__':
    with torch.no_grad():
        main()