'''
Joshua Van Doren
2/14/2023
Display video data from zed with opencv and save images to a user defined folder
If you're running the script multiple times using number ordering to the same folder location, make sure to change the
image label starting value to ensure you're not writing over previously saved images
Have ZED SDK and its python-api installed
pip install opencv-python
'''

import pyzed.sl as sl
import cv2 as cv
import os
from datetime import datetime
import numpy as np

zed = sl.Camera()

# set configuration params

init_params = sl.InitParameters()
'''
video mode      side-by-side res        (FPS)               Field of View
2.2K	        4416x1242	            15	                Wide
1080p	        3840x1080	            30, 15	            Wide
720p	        2560x720	            60, 30, 15	        Extra Wide
WVGA	        1344x376	            100, 60, 30, 15	    Extra Wide
'''
init_params.camera_resolution = sl.RESOLUTION.HD720  # Comes in HD1080 , HD2K, HD720, VGA, LAST
init_params.camera_fps = 60  # FPS of 15, 30, 60, or 100

# set up image label starting number
img_label = 0

# set up folder path to save images
numbered_folder_path = "path/to/number/ordered/folder/"
date_folder_path = "path/to/date/organized/folder/"

# open camera
camera_state = zed.open(init_params)

# print camera params (Resolution, fps)
print(f"Resolution: {zed.get_camera_information().camera_resolution} \n FPS: {zed.get_camera_information().camera_fps}")

while True:
    # create image object
    image_zed = sl.Mat()

    # Zed image object exists
    if zed.grab() == sl.ERROR_CODE.SUCCESS:

        # image types can be changed below VIEW.(TYPE)
        zed.retrieve_image(image_zed, sl.VIEW.LEFT)

        # numpy data array for converting zed to open-cv
        image_ocv = image_zed.get_data()

        cv.imshow("Zed-Cam", image_ocv)

        # wait for user to press 'c' key to capture image or 'q' key to quit
        key = cv.waitKey(1) & 0xFF
        if key == ord('c'):
            # save captured image with ordered label in specified folder
            img_path = os.path.join(numbered_folder_path, str(img_label) + ".jpg")
            cv.imwrite(img_path, image_ocv)

            # increment image label for next capture
            img_label += 1

        if key == ord('t'):
            # generate image label with timestamp
            now = datetime.now().strftime("%Y%m%d_%H%M%S")
            date_img_label = f"{now}.jpg"

            # save captured image with ordered label in specified folder
            img_path = os.path.join(date_folder_path, date_img_label)
            cv.imwrite(img_path, image_ocv)


        elif key == ord('q'):
            # user has quit, release camera and close display window
            cv.destroyAllWindows()
            zed.close()
            break
