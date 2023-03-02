'''
Joshua Van Doren
2/26/2023
Display video data from zed with opencv and save images to a user defined folder
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

images = 0
# set up folder path to save images
numbered_folder_path = "C:/Users/j4vdo/Desktop/Games"  # these folder placeholder formats are correct
date_folder_path = "/path/to/date/organized/folder"

def exist(folder_path):
    if not os.path.exists(folder_path):
        print(f"The folder path {folder_path} does not exist.")
    else:
        print(f"The folder path {folder_path} exists.")


exist(numbered_folder_path)
exist(date_folder_path)


contents = os.listdir(numbered_folder_path)
for item in contents:
    images += 1
img_label = images
print(f"There are {img_label} images currently in {numbered_folder_path}.")

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

        # wait for user to press 'c' or 't' key to capture image or 'q' key to quit
        key = cv.waitKey(1) & 0xFF
        if key == ord('c'):
            # increment image label for next capture
            img_label += 1
            # save captured image with ordered label in specified folder
            img_path = os.path.join(numbered_folder_path, str(img_label) + ".jpg")
            cv.imwrite(img_path, image_ocv)
            print(f"Image number {img_label} is now stored in {numbered_folder_path}")

        # take timestamped image
        if key == ord('t'):
            # generate image label with timestamp
            now = datetime.now().strftime("%Y%m%d_%H%M%S")
            date_img_label = f"{now}.jpg"

            # save captured image with ordered label in specified folder
            img_path = os.path.join(date_folder_path, date_img_label)
            cv.imwrite(img_path, image_ocv)
            print(f"Image taken at {now} is now stored in {date_folder_path}")
        # quit
        elif key == ord('q'):
            print("Closing window")
            # user has quit, release camera and close display window
            cv.destroyAllWindows()
            zed.close()
            break
