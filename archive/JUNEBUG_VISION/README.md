1) Make sure Nvidia Jetpack 5.0.2 is installed
2) Install ZED_SDK
3) Make our environment:
	-Starting in home Directory:
	**clone the repository that contains the yolov5_zedsdk folder
 	
	$cd ~/yolov5_zedsdk
	
	$git clone https://github.com/ultralytics/yolov5 
	
	$cd yolov5 

	$pip install -U -r requirements.txt


4) Uninstall all versions of torch and torchvision
	
	$sudo pip uninstall torch
	
	$sudo pip uninstall torchvision

5) Installing correct torch. IMPORTANT FOLLOW INSTRUCTIONS BEFORE moving forward
Go to https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048

Click PyTorch v1.11.0 and download wheel. Or simply follow this link: https://nvidia.box.com/shared/static/ssf2v7pf5i245fk4i0q926hy4imzs2ph.whl

Open terminal and in the downloaded folder of wheel run: 

$pip3 install torch-1.11.0-cp38-cp38-linux_aarch64.whl

Check if installed correctly with cuda support by starting python3 instance in terminal

$ python3

>>>import torch

>>>print(torch.__version__)

>>>torch.cuda.is_available()



If TRUE torch with cuda support was properly downloaded.

6) Install torchvision compatible with nvidia jetson torch.

Install from source in home (or any) directory

$ git clone https://github.com/pytorch/vision

$ cd vision

$ sudo -E python3 setup.py install

$ cd 

check if installed correctly by opening python3 instance in terminal

$ python3

>>>import torchvision

>>>print(torchvision.__version__)

To make sure we have Opencv module for detector.py

pip install open-cv-contrib-python


7) Now to create your own Custom Model

Follow the steps in this video to Box and label your data (ROBOFLOW)

WHEN GOING TO GENERATE DATASET REMEMBER IMG SIZE = 640

WHEN GOING TO EXPORT DATA EXPORT IN YOLOV5 PYTORCH FORMAT

INSTEAD OF EXPORT BY: DOWNLOAD ZIP TO COMPUTER https://www.youtube.com/watch?v=x0ThXHbtqCQ&t=602s



Once you have downloaded the zipped file extract contents
If an unzipped folder is extracted -> copy and paste all elements in that folder to the yolov5 directory

If these elements: “folder:test, folder:train, folder:valid, file:data.yaml, file:README.dataset.txt, file:README.roboflow.txt” are extracted when the folder is unzipped -> highlight all the elements copy and paste them to yolov5 directory


Within the folder yolov5/models/ there will be different yolov5 yaml files

Yolov5n = nano

Yolov5s = small

Yolov5m = medium

Yolov5l = large

Yolov5x =X Large


These are the neural networks pretrained model that will be used for training our own model.
As the size goes up the complexity goes up taking longer but has more confidence.

THESE YAML FILES NEED TO BE EDITED

THE VARIABLE “nc” ARE THE NUMBER OF CLASSES ( NUMBER OF OBJECTS WE ARE TRYING TO DETECT)

EX:MY MODEL IS DETECTING “PERSON, PHONE, CHAIR” THEN THE FILE SHOULD READ nc: 3

The best way to edit these files is to make a copy of each and rename them

custom_yolov5n.yaml

custom_yolov5s.yaml

custom_yolov5m.yaml

custom_yolov5l.yaml

custom_yolov5x.yaml


Now to Train our Model
While in the yolov5 directory
MAKE SURE data.yaml FILE IS EDITED TO CORRECT PATHS

>data.yaml

train: ./path/to/train/images

val: ./path/to/valid/images

test: ./path/to/test/images

Run the command:

Python3 train.py --img 640  --batch 16  --epochs 400  --data data.yaml --cfg ./models/custom_yolov5s.yaml --weights ‘’
 
--img = size of images when generated in roboflow

--batch = number of training samples it runs through the network before updating the weights and moving to the next batch of samples

--epochs = the number of times it tests all the samples through the network

--data = the yaml file that was created from ROBOFLOW (OUR MODEL DATA)

--cfg = the provided yolov5(n,s,m,l,x).yaml files (decide complexity based on needs)

–- weights = could use a provided pytorch weight (not required)

TRAINING COULD END EARLY: DON'T WORRY, SYSTEM IS DESIGNED TO STOP TRAINING IF NO IMPROVEMENT TO THE MODEL IS MADE IN THE LAST TRAINED 100 EPOCHS

WEIGHT FILEs WILL BE CREATED IN: yolov5/runs/train/exp/weights

These files are best.pt and last.pt

Usually be implementing best.pt

Once training is complete:

INFERENCE TESTING Can be done in the yolov5 directory running the following commands


Test on image:

python3 detect.py –weights runs/train/exp/weights/best.pt --img 640 --conf 0.1 --source img.jpg


Test on folder of images:

python3 detect.py –weights runs/train/exp/weights/best.pt --img 640 --conf 0.1 --source path/


Test on webcam

python3 detect.py –weights runs/train/exp/weights/best.pt --img 640 --conf 0.1 --source 0

weights: this will be the best.pt weights file created when training own model

img: img size specified all the way back in roboflow

conf: the minimum confidence level the model uses to declare a detected object

source: the object (image, folder, screen, video, other implementations) that will run through object detection

The output path and file of these inference tests will be printed in terminal at the end of runtime

IMPLEMENTING trained model with ZED_API

Start in the directory:  ~/yolov5_zedsdk/yolov5/runs/train/exp/


To simplify the implementation of weights in detector.py we will copy the weights folder from the dir ~/junebug-2023/src/yolov5_zedsdk/yolov5/runs/train/exp/ to  ~/junebug-2023/src/yolov5_zedsdk

Within the directory: ~/junebug-2023/src/yolov5_zedsdk/ there should be the folders cv_viewer, ogl_viewer, weights, object_detection-zed-sd


Some changes to detector.py may or may not need to be made

FOR ZED-API  CUSTOM OBJECT DETECTOR detector.py file needs editing

The import of “scale_coords” needs to be changed to “scale_segments’ (MAY ALREADY BE DONE, depending where you pulled your source code)


Also the lines parser.add_argument('--weights', nargs='+', type=str, default='../weights/best.pt', help='model.pt path(s)') needs to be changed to  parser.add_argument('--weights', nargs='+', type=str, default='weights/best.pt', help='model.pt path(s)')  (what changed? default=’../weights/best.pt’ > default=’weights/best.pt’

As well as parser.add_argument('--img_size', type=int, default=416, help='inference size (pixels)') needs to be changed toparser.add_argument('--img_size', type=int, default=640, help='inference size (pixels)')  (what changed? default=416 > default=640)



Run detector.py script in ~/yolov5_zedsdk/

python3 detector.py


GREAT RESOURCES FOR STRONGER UNDERSTANDING
https://towardsdatascience.com/how-to-train-a-custom-object-detection-model-with-yolo-v5-917e9ce13208


https://medium.com/analytics-vidhya/training-a-custom-object-detection-model-with-yolo-v5-aa9974c07088



