#this is the master script for the remote ultrasound suite

""" STEPS
1. connect to wifi (not included b/c might use CAT6 connection
2. initiate video recording and display of video
3. open zoom
4. start meeting and send email to end user with zoom link
5. share screen """

#VIDEO START
# import the necessary packages
from __future__ import print_function
from collections import namedtuple
import util as cm
import pyrealsense2 as rs
import math
import numpy as np
from skeletontracker import skeletontracker

from PIL import Image
from PIL import ImageTk
import tkinter as tki
from tkinter.ttk import *
import threading
import datetime
import imutils
import cv2
import os
import time
import matlab.engine
import sys
import faulthandler

"""SKELETON TRACKING FUNCTIONS"""
def render_ids_3d(
    render_image, skeletons_2d, depth_map, depth_intrinsic, joint_confidence
):
    thickness = 1
    text_color = (255, 255, 255)
    rows, cols, channel = render_image.shape[:3]
    distance_kernel_size = 5
    # calculate 3D keypoints and display them
    for skeleton_index in range(len(skeletons_2d)):
        skeleton_2D = skeletons_2d[skeleton_index]
        joints_2D = skeleton_2D.joints
        did_once = False
        for joint_index in range(len(joints_2D)):
            if did_once == False:
                cv2.putText(
                    render_image,
                    "id: " + str(skeleton_2D.id),
                    (int(joints_2D[joint_index].x), int(joints_2D[joint_index].y - 30)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    text_color,
                    thickness,
                )
                did_once = True
            # check if the joint was detected and has valid coordinate
            if skeleton_2D.confidences[joint_index] > joint_confidence:
                distance_in_kernel = []
                low_bound_x = max(
                    0,
                    int(
                        joints_2D[joint_index].x - math.floor(distance_kernel_size / 2)
                    ),
                )
                upper_bound_x = min(
                    cols - 1,
                    int(joints_2D[joint_index].x + math.ceil(distance_kernel_size / 2)),
                )
                low_bound_y = max(
                    0,
                    int(
                        joints_2D[joint_index].y - math.floor(distance_kernel_size / 2)
                    ),
                )
                upper_bound_y = min(
                    rows - 1,
                    int(joints_2D[joint_index].y + math.ceil(distance_kernel_size / 2)),
                )
                for x in range(low_bound_x, upper_bound_x):
                    for y in range(low_bound_y, upper_bound_y):
                        distance_in_kernel.append(depth_map.get_distance(x, y))
                median_distance = np.percentile(np.array(distance_in_kernel), 50)
                depth_pixel = [
                    int(joints_2D[joint_index].x),
                    int(joints_2D[joint_index].y),
                ]
                if median_distance >= 0.3:
                    point_3d = rs.rs2_deproject_pixel_to_point(
                        depth_intrinsic, depth_pixel, median_distance
                    )
                    point_3d = np.round([float(i) for i in point_3d], 3)
                    point_str = [str(x) for x in point_3d]
                    cv2.putText(
                        render_image,
                        str(point_3d),
                        (int(joints_2D[joint_index].x), int(joints_2D[joint_index].y)),
                        cv2.FONT_HERSHEY_DUPLEX,
                        0.4,
                        text_color,
                        thickness,
                    )


class VideoStream:

    #this is the constructor of our class
        #the outputPath is where we save the videos
    def __init__(self, vs1, vs2, vs3, outputPath):
        # store the video stream object and output path, then initialize
        # the most recently read frame, thread for reading frames, and
        # the thread stop event
        self.record = False
        self.vs1 = vs1
        self.vs2 = vs2
        self.vs3 = vs3
        self.outputPath = outputPath
        self.frame1 = None
        self.thread1 = None
        self.stopEvent = None

        #SKELETON TRACKING STUFF
        print("initiating skeletal tracking pipeline")
        try:
                # Configure depth and color streams of the intel realsense
                self.config = rs.config()
                self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
                self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

                # Start the realsense pipeline
                self.pipeline = rs.pipeline()
                self.pipeline.start()

                # Create align object to align depth frames to color frames
                self.align = rs.align(rs.stream.color)

                # Get the intrinsics information for calculation of 3D point
                self.unaligned_frames = self.pipeline.wait_for_frames()
                self.frames = self.align.process(self.unaligned_frames)
                self.depth =self.frames.get_depth_frame()
                self.depth_intrinsic = self.depth.profile.as_video_stream_profile().intrinsics

                # Initialize the cubemos api with a valid license key in default_license_dir()
                self.skeletrack = skeletontracker(cloud_tracking_api_key="")
                self.joint_confidence = 0.2
        except:
            print("exception occured in skeleton tracking initialization")



        
        # initialize the root window and image panel
        self.root = tki.Tk()
        
        self.panel1 = None
        self.panel2 = None
        self.panel3 = None
        self.mypanels = {"1" : self.panel1, "2": self.panel2, "3": self.panel3}
        
        
        #Tkinter Stuff
        #*************************************************************
        #self.root.geometry('900x300')
        
        # Create style Object
        style = Style()
         
        style.configure('TButton', font =
                       ('calibri', 20, 'bold'),
                            borderwidth = '4')
         
        # Changes will be reflected
        # by the movement of mouse.
        #style.map('TButton', foreground = [('active', '! disabled', 'green')], background = [('active', 'black')])
        
        tki.Label(self.root, text = 'Ultrasound Video', font =('Helvetica', 10, 'bold')).grid(row = 0, column = 0, padx=20, pady=10)
        tki.Label(self.root, text = 'Head-Mounted Feed', font =('Helvetica', 10, 'bold')).grid(row = 0, column = 1,  padx=20, pady=10)
        tki.Label(self.root, text = 'Skeletal Tracking', font =('Helvetica', 10, 'bold')).grid(row = 2, column= 0,  padx=20, pady=10)

        self.f1 = tki.Frame(self.root)
        self.f1.grid(row = 3, column = 1)

        
        self.button = tki.Button(self.f1, text = 'Start Video Recording', width = 25, command =self.record_flag)
        self.button.pack(side="top")
        #self.button.grid(row = 2, column = 0)
        
        self.button1 = tki.Button(self.f1, text = 'Display Real-time Orientation', width = 25, command = self.theCall)
        self.button1.pack(side="top")

        #self.button1.grid(row = 3, column = 1)


       #**************************************************************
        
        
        # start a THREAD that constantly pools the video sensor for
        # the most recently read frame
        sys.setrecursionlimit(2097152)    # adjust numbers
        threading.stack_size(134217728)   # for your needs
        self.lock = threading.Lock()
        self.stopEvent = threading.Event()
        self.thread1 = threading.Thread(target=self.videoLoop, args=(vs1,'1',))
        self.thread2 = threading.Thread(target=self.videoLoop, args=(vs2,'2',))
        self.thread3 = threading.Thread(target=self.videoLoop, args=(vs3,'3',))
        
        self.thread1.setDaemon(True)
        self.thread2.setDaemon(True)
        self.thread3.setDaemon(True)

        
        self.thread1.start()
        self.thread2.start()
        self.thread3.start()
        
        # set a callback to handle when the window is closed
        self.root.wm_title("Remote Ultrasound Suite")
        self.root.wm_protocol("WM_DELETE_WINDOW", self.onClose)
        
        
        
        
    
    def videoLoop(self, vs, panel):
    
            font = cv2.FONT_HERSHEY_SIMPLEX
            #NOTE: panel is a string that is the key to the mypanels dictionary
            
            try:
                # keep looping over frames until we are instructed to stop
                while not self.stopEvent.is_set():
                    # grab the frame from the video stream and resize it to
                    # have a maximum width of 300 pixels
                    if panel =="3":
                        frame = self.skeletonOverlay()
                    else:
                        ret, frame = vs.read()
                    
                    cv2.putText(frame,str(datetime.now()),(10,30), font, .5,(255,255,255),2,cv2.LINE_AA)
                    
                    if self.record == True:
                        #print("recording")
                        if panel == "1":
                            self.writer1.write(frame)
                        elif panel == "2":
                            self.writer2.write(frame)
                    
                    frame = imutils.resize(frame, width=400)
                    # OpenCV represents images in BGR order; however PIL
                    # represents images in RGB order, so we need to swap
                    # the channels, then convert to PIL and ImageTk format
                    image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    image = Image.fromarray(image)
                    image = ImageTk.PhotoImage(image)
            
            
                    #here we test out the locking theory
                    
                    try:
                        acquired = self.lock.acquire(0)
                        if acquired:
                            # if the panel is not None, we need to initialize it
                            if self.mypanels[panel] is None and panel =='1':
                                self.mypanels[panel] = tki.Label(image=image)
                                self.mypanels[panel].image = image
                                self.mypanels[panel].grid(row = 1, column = 0)
        #                        self.mypanels[panel].pack(side="left", padx=10, pady=10)
                                
                            elif self.mypanels[panel] is None and panel =='2':
                                self.mypanels[panel] = tki.Label(image=image)
                                self.mypanels[panel].image = image
                                self.mypanels[panel].grid(row = 1, column = 1)
        #                        self.mypanels[panel].pack(side="right", padx=10, pady=10)
                            
                            elif self.mypanels[panel] is None and panel =='3':
                                self.mypanels[panel] = tki.Label(image=image)
                                self.mypanels[panel].image = image
                                self.mypanels[panel].grid(row = 3, column = 0)
                    
                            # otherwise, simply update the panel
                            else:
                                self.mypanels[panel].configure(image=image)
                                self.mypanels[panel].image = image
                            
                            self.lock.release()
                        #else:
                            #print("lock not acquired")
                    except:
                        print("something failed in locking mechanism")
                            
            except RuntimeError:
                print("[INFO] caught a RuntimeError")

    def skeletonOverlay(self):
        # Called every loop
    
        # Create a pipeline object. This object configures the streaming camera and owns it's handle
        self.unaligned_frames = self.pipeline.wait_for_frames()
        self.frames = self.align.process(self.unaligned_frames)
        self.depth = self.frames.get_depth_frame()
        color = self.frames.get_color_frame()
        if not self.depth or not color:
            return None

        # Convert images to numpy arrays
        depth_image = np.asanyarray(self.depth.get_data())
        color_image = np.asanyarray(color.get_data())

        # perform inference and update the tracking id
        skeletons = self.skeletrack.track_skeletons(color_image)

        # render the skeletons on top of the acquired image and display it
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        cm.render_result(skeletons, color_image, self.joint_confidence)
        render_ids_3d(
            color_image, skeletons, self.depth, self.depth_intrinsic, self.joint_confidence
        )
        return color_image

                

        
                
                
    def record_flag(self):
        #PURPOSE: To set the record_flag to True on button press so that recording can begin
        
        self.button.configure(bg='red',)
        #video saving
        width= int(self.vs1.get(cv2.CAP_PROP_FRAME_WIDTH))
        height= int(self.vs1.get(cv2.CAP_PROP_FRAME_HEIGHT))
        #start the writer's for saving the video
        self.writer1= cv2.VideoWriter('UltrasoundVideo.mp4', cv2.VideoWriter_fourcc(*'DIVX'), 20, (width,height))
        
        self.writer2= cv2.VideoWriter('HeadmountVideo.mp4', cv2.VideoWriter_fourcc(*'DIVX'), 20, (width,height))
        self.record = True
        print("starting recording")
    
                
    def onClose(self):
        # set the stop event, cleanup the camera, and allow the rest of
        # the quit process to continue
        print("[INFO] closing...")
        self.stopEvent.set()
        print("stop event set")
        self.vs1.release()
        self.vs2.release()
        if self.record == True:
            self.writer1.release()
            self.writer2.release()
        

        #close the pipeline for skeleton tracking 
        self.pipeline.stop()
        cv2.destroyAllWindows()

        print("capture stream released")
        self.root.destroy()
    
    
    #FUNCTION: To call the Matlab Engine to display real time orientation of ultrasound probe
    def pythonMatlabCall(self):
        print("starting matlab engine....")
        eng = matlab.engine.start_matlab()
        print("engine started")
        try:
            eng.realTimeOrientationSensor(nargout=0) #simple_script is the name of the .m file!
            
        except:
            print("there was an error when stopping matlab script")
            print("shutting down matlab engine")
            eng.quit()
            
        else:
            print("shutting down matlab engine")
            eng.quit()
            
    #PURPOSE: To start a new thread for the matlab script to run
    def theCall(self):
        self.threadMAT = threading.Thread(target=self.pythonMatlabCall, args=())
        
        self.threadMAT.setDaemon(True)
        
        self.threadMAT.start()
    
    
    
    
##################### BODY OF THE CODE ###########################

from datetime import datetime
faulthandler.enable()
print("warming up headmounted camera...")

### CAMERA 1 (headmounted cam)
vs1 = cv2.VideoCapture()
#below is the index (0) to get ultrasound video feed
vs1.open(3)
time.sleep(1)


### CAMERA 2 (ultrasound feed)
print("initiating ultrasound feed...")
vs2 = cv2.VideoCapture()
#NOTE: open(1) opens the Microsoft LifeCam Cinema HD USB webcam 
#NOTE: open(2) opens the RealSense USB camera connection 
vs2.open(0)
time.sleep(1)

###CAMERA 3 (posture measurement feed)
#NOTE: open(3) opens third camera
print("warming up camera up skeleton tracker camera...")
# vs3 = cv2.VideoCapture()
# vs3.open(4)
# time.sleep(1)
vs3 = 0


#start the app

pba = VideoStream(vs1,vs2, vs3, "")
pba.root.mainloop()
