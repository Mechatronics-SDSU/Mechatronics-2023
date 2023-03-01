 "distance_detector.py" takes in no arguments, and requires the ZED camera.

"distance_detector.py" is a modified form on the ZED SDK supplied sample code
"object_detection_image_viewer.py".

"object_detection_image_viewer.py" reads the ZED camera's input and then highlights detected human
objects in an image viewer.

"distance_detector.py" modifies this code to calculate the distance from the camera of each object,
and only outputs the distance of the nearest object to standard output.

If the ZED camera detects an object to be within .5 m from the camera, the program outputs "stop"
and exits.

If the ZED camera doesn't detect any object in a frame, the program outputs "no object" and continues.

The ZED SDK is able to supply the (x,y,z) coordinates of any object it detects in relation to the
camera, and the distance is calculated from the coordinates.

The ZED SDK by defaults only detects objects that it's at least 50% confident of, but this setting
can be changed by uncommenting line 70:
#obj_runtime_param.detection_confidence_threshold = 0                # Set minimum confidence to recognize object here.

The ZED SDK by defaults tries to detect objects of any of its built in classes (listed just below),
and this setting can be changed by uncommenting line 70:
#obj_runtime_param.object_class_filter = [sl.OBJECT_CLASS.PERSON]    # Only detect Persons

Other than the .PERSON object class, ZED SDK also can be set to determine the following classes of objects:
  PERSON
  VEHICLE
  BAG
  ANIMAL
  ELECTRONICS
  FRUIT_VEGETABLE
  SPORT
  
Sources used:

  To get object classes:
    https://github.com/stereolabs/zed-unity/blob/master/ZEDCamera/Assets/ZED/SDK/Helpers/Scripts/ZEDManager.cs
    
  To get object position member:
    https://github.com/stereolabs/zed-examples/blob/master/tutorials/tutorial%206%20-%20object%20detection/python/object_detection.py
    
  Base "object_detection_image_viewer.py" code:
    https://github.com/stereolabs/zed-examples/blob/master/object%20detection/image%20viewer/python/object_detection_image_viewer.py
