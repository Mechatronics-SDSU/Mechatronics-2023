ZED Vision for creating image data sets

One File: zed_image_dataset_maker.py



User Configuration of File:

Set numbered_folder_path to the folder which you want to store images in a number ordered list

If using numbered_folder_path organizational method of storage and you are running this script multiple times to the same folder:
Change the img_label value to a value greater than the value of your last image already stored in that folder


Set date_folder_path to the folder in which you want the images to be stored in order of date and time they were taken



Functionality while script is running:

Pressing 'c':
key to capture the current image and save it with an ordered label (starting from 0) in the specified folder. The user can press the 'q' key to quit the script.
The captured images are saved as JPEG files with filenames that match their respective ordered labels (e.g., "0.jpg", "1.jpg", "2.jpg", etc.).

pressing 't':
generates an image label using the current date and time, in the format "YYYYMMDD_HHMMSS.jpg". 
This ensures that each captured image has a unique filename that includes a timestamp, so the JPEG files won't overwrite one another.

pressing 'q':
ends the program
