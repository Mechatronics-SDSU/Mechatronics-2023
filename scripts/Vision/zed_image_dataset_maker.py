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
import argparse
from time import sleep

# Parse command line arguments
parser = argparse.ArgumentParser()
parser.add_argument('-folder', help='Folder path to save images', required=True)
parser.add_argument('-date', help='Folder path to save date of images', required=False)
SAVE_TO_DIRECTORY = "/home/mechatronics/master/Mechatronics-2023/scripts/Vision"

args = parser.parse_args()

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
numbered_folder_path = "datasets/" + args.folder  # these folder placeholder formats are correct

if args.folder is not None:
    date_folder_path = args.date
else:
    date_folder_path = args.folder

def exist(folder_path):
    if not os.path.exists(folder_path):
        print(f"The folder path {folder_path} does not exist.")
        return False
    else:
        print(f"The folder path {folder_path} exists.")
        return True

# Use command line argument to create new folder if it does not already exist
if not exist(numbered_folder_path):
    directory = numbered_folder_path
    parent_dir = SAVE_TO_DIRECTORY
    path = os.path.join(parent_dir, directory)
    os.mkdir(path)

# exist(date_folder_path)

contents = os.listdir(numbered_folder_path)
for item in contents:
    images += 1
img_label = images
print(f"There are {img_label} images currently in {numbered_folder_path}.")

# open camera
camera_state = zed.open(init_params)
# print camera params (Resolution, fps)
# print(f"Resolution: {zed.get_camera_information().camera_resolution} \n FPS: {zed.get_camera_information().camera_fps}")
# Set up frame counter and saving interval
frame_counter = 0
saving_interval = 30
mode = False
while True:
    # create image objesct
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
            now = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
            date_img_label = f"{now}.jpg"

            # save captured image with ordered label in specified folder
            img_path = os.path.join(date_folder_path, date_img_label)
            cv.imwrite(img_path, image_ocv)
            print(f"Image taken at {now} is now stored in {date_folder_path}")

        #Save image every 10 frames
        if mode:
            if frame_counter % saving_interval == 0:
                img_label += 1
                img_path = os.path.join(str(numbered_folder_path), str(str(img_label) + ".jpg"))
                cv.imwrite(img_path, image_ocv)
                print(f"Image number {img_label} is now stored in {numbered_folder_path}")

        if key == ord('m'):
            if mode:
                mode = False
            else:
                mode = True

        # quit
        elif key == ord('q'):
            print("Closing window")
            # user has quit, release camera and close display window
            cv.destroyAllWindows()
            zed.close()
            break

        #increment frame counter
        frame_counter += 1
