ZED Vision for creating image data sets

One File: zed_image_dataset_maker.py



User Configuration of File:

Set numbered_folder_path to the folder which you want to store images in a number ordered list


Set date_folder_path to the folder in which you want the images to be stored in order of date and time they were taken



Run Script: 

The Script will print messages acknowledging if your folders exist or not.


Functionality while script is running:

Pressing 'c':
key to capture the current image and save it with an ordered label in the specified folder.
The captured images are saved as JPEG files with filenames that match their respective ordered labels (e.g., "1.jpg", "2.jpg", etc.).

pressing 't':
generates an image label using the current date and time, in the format "YYYYMMDD_HHMMSS.jpg". 
This ensures that each captured image has a unique filename that includes a timestamp, so the JPEG files won't overwrite one another.

pressing 'q':
ends the program
