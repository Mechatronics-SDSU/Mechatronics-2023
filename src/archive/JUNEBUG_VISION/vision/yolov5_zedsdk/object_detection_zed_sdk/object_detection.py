
"""
if pyzed.sl import error go to enviroment terminal and
run python3 /usr/local/zed/get_python_api.py.
Zed sdk must be properly downloaded inorder for using sdk object dectiotion library
"""
import pyzed.sl as sl
import sys
import viewer as gl


def main():

    """

    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD1080  # Use HD1080 video mode
    init_params.coordinate_units = sl.UNIT.METER
    init_params.camera_fps = 60  # Set fps at 60
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP

    # If applicable, use the SVO given as parameter
    # Otherwise use ZED live stream
    if len(sys.argv) == 2:
        filepath = sys.argv[1]
        print("Using SVO file: {0}".format(filepath))
        init_params.set_from_svo_file(filepath)

    # Set runtime parameters
    runtime_parameters = sl.RuntimeParameters()

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)

    # Enable object detection module
    obj_param = sl.ObjectDetectionParameters()
    # Defines if the object detection will track objects across images flow.
    obj_param.enable_tracking = True  # if True, enable positional tracking

    if obj_param.enable_tracking:
        zed.enable_positional_tracking()

    zed.enable_object_detection(obj_param)

    camera_info = zed.get_camera_information()
    # Create OpenGL viewer
    viewer = gl.GLViewer()
    viewer.init(camera_info.calibration_parameters.left_cam, obj_param.enable_tracking)

    # Configure object detection runtime parameters
    obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
    obj_runtime_param.detection_confidence_threshold = 60
    #obj_runtime_param.object_class_filter = [sl.OBJECT_CLASS.PERSON]  # Only detect Persons

    # Create ZED objects filled in the main loop
    objects = sl.Objects()
    image = sl.Mat()
    """


    """
    create zed camera object
    """
    zed = sl.Camera()

    """Create a InitParameters object and set configuration parameters"""
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD1080  # Use HD1080 video mode
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.METER
    init_params.camera_fps = 60
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init_params.sdk_verbose = True

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)


    """
    enable object detection model and enable paramters
    """
    obj_param = sl.ObjectDetectionParameters()
    obj_param.enable_tracking = True
    obj_param.image_sync = True
    obj_param.enable_mask_output = True

    camera_infos = zed.get_camera_information()
    """enable object detection tracking. Tracks objects detected for 3d computation"""
    if obj_param.enable_tracking:
        positional_tracking_param = sl.PositionalTrackingParameters()
        # positional_tracking_param.set_as_static = True
        #positional_tracking_param.set_floor_as_origin = True
        zed.enable_positional_tracking(positional_tracking_param)

    print("Object Detection: Loading Module...")

    runtime_parameters = sl.RuntimeParameters()

    err = zed.enable_object_detection(obj_param)
    if err != sl.ERROR_CODE.SUCCESS:
        print(repr(err))
        zed.close()
        exit(1)


    """Init runtime parameters """
    #objects = sl.Objects()
    obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
    obj_runtime_param.detection_confidence_threshold = 10
    #obj_runtime_param.object_class_filter = [sl.OBJECT_CLASS.PERSON]  # Only detect Persons
    # Create ZED objects filled in the main loop
    objects = sl.Objects()
    image = sl.Mat()
    # Create OpenGL viewer
    viewer = gl.GLViewer()
    viewer.init(camera_infos.calibration_parameters.left_cam, obj_param.enable_tracking)

    """
    run openGl viewer with zed object detection paramters
    In future can capture data and send with ROS
    """
    while viewer.is_available():
        # Grab an image, a RuntimeParameters object must be given to grab()
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Retrieve left image
            zed.retrieve_image(image, sl.VIEW.LEFT)
            # Retrieve objects
            zed.retrieve_objects(objects, obj_runtime_param)
            # Update GL view
            viewer.update_view(image, objects)

    viewer.exit()

    image.free(memory_type=sl.MEM.CPU)
    # Disable modules and close camera
    zed.disable_object_detection()
    zed.disable_positional_tracking()

    zed.close()

if __name__ == "__main__":
    main()
