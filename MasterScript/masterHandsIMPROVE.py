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
import csv
from NDI_PyTrack import NDI_pytrack
import mediapipe as mp


"""SKELETON TRACKING FUNCTIONS"""




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
        self.dataFrame = [["Nose", "Center of Chest", "right shoulder", "right elbow", "right wrist", "left shoulder", "left elbow", "left wrist", "right hip", "right knee", "right ankle", "left hip", "left knee", "right ankle", "right eye", "left eye", "right ear", "left ear", "time"]]


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

        #hand tracking stuff
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_hands = mp.solutions.hands
        self.drawing_styles = mp.solutions.drawing_styles
        self.hands = self.mp_hands.Hands(min_detection_confidence=0.5,min_tracking_confidence=0.5)

        
        # initialize the root window and image panel
        self.root = tki.Tk()
        self.root.configure(background = 'grey')
        self.root.resizable(False, False)
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
        
        tki.Label(self.root, text = 'Ultrasound Video', font =('Helvetica', 10, 'bold')).grid(row = 0, column = 1, padx=20, pady=10)
        tki.Label(self.root, text = 'Head-Mounted Feed', font =('Helvetica', 10, 'bold')).grid(row = 0, column = 2,  padx=20, pady=10)
        tki.Label(self.root, text = 'Skeletal Tracking', font =('Helvetica', 10, 'bold')).grid(row = 2, column= 1,  padx=20, pady=10)

        self.f1 = tki.Frame(self.root, borderwidth = 1)
        self.f1.grid(row = 1, column = 0, padx = 2)

        
        self.button = tki.Button(self.f1, text = 'Start Video Recording', width = 25, command =self.record_flag)
        self.button.pack(side="top")
        #self.button.grid(row = 2, column = 0)
        
        self.button1 = tki.Button(self.f1, text = 'Start Magentic Tracking', width = 25, command = self.theCall)
        self.button1.pack(side="top")

        
        #separator = Separator(self.root, orient='vertical').grid(column=0, row=0, rowspan=4, sticky='ns', ipadx=2)
        separator = Separator(self.root, orient = 'vertical')
        separator.place( x=190, y=0, relwidth=0.005, relheight=1)


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
                    elif panel =="2":
                        frame = self.hand_tracking(vs2)
                    else:
                        ret, frame = vs.read()
                    
                    cv2.putText(frame,str(datetime.now()),(10,30), font, .5,(255,255,255),2,cv2.LINE_AA)
                    
                    if self.record == True:
                        #print("recording")
                        if panel == "1":
                            self.writer1.write(frame)
                        elif panel == "2":
                            self.writer2.write(frame)

                    if panel == '3':
                        frame = imutils.resize(frame, width=500)

                    else:
                        frame = imutils.resize(frame, width=525)
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
                                self.mypanels[panel].grid(row = 1, column = 1)
                                
                                
                            elif self.mypanels[panel] is None and panel =='2':
                                self.mypanels[panel] = tki.Label(image=image)
                                self.mypanels[panel].image = image
                                self.mypanels[panel].grid(row = 1, column = 2)
        #                        self.mypanels[panel].pack(side="right", padx=10, pady=10)
                            
                            elif self.mypanels[panel] is None and panel =='3':
                                self.mypanels[panel] = tki.Label(image=image)
                                self.mypanels[panel].image = image
                                self.mypanels[panel].grid(row = 3, column = 1)
                    
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
        self.render_ids_3d(
            color_image, skeletons, self.depth, self.depth_intrinsic, self.joint_confidence
        )
        return color_image



        """MODIFIED TO ALLOW US TO SAVE THE JOINT VALUES"""
        #NOTE: 
        #18 points in the skeleton tracker 
    def render_ids_3d(
        self, render_image, skeletons_2d, depth_map, depth_intrinsic, joint_confidence
    ):
        zeroList = ["0,0,0"]
        thickness = 1
        text_color = (255, 255, 255)
        rows, cols, channel = render_image.shape[:3]
        distance_kernel_size = 5
        intermediateSkeleJoint =[]
        # calculate 3D keypoints and display them
        for skeleton_index in range(len(skeletons_2d)):
            skeleton_2D = skeletons_2d[skeleton_index]
            joints_2D = skeleton_2D.joints
            did_once = False
            #joint index corresponds to joint
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
                # print("[0, 0, 0]", )
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
                        #appends the calculated joint to the 3D value
                        

                        point_str = [str(x) for x in point_3d]
                        #this part combines all into a single string
                        mySeparator = ","
                        intermediateSkeleJoint = np.append(intermediateSkeleJoint, mySeparator.join(point_str) )
                        
                        #print(point_str, ",")
                        cv2.putText(
                            render_image,
                            str(point_3d),
                            (int(joints_2D[joint_index].x), int(joints_2D[joint_index].y)),
                            cv2.FONT_HERSHEY_DUPLEX,
                            0.4,
                            text_color,
                            thickness,
                        )
                    else:
                        #print("[0,0,0]")
                        intermediateSkeleJoint =np.append(intermediateSkeleJoint, zeroList)
                else:
                    #print(zeroList)
                    intermediateSkeleJoint = np.append(intermediateSkeleJoint, zeroList)
            #timestamp
            intermediateSkeleJoint = np.append(intermediateSkeleJoint, str(time.time()))
            self.dataFrame.append(intermediateSkeleJoint)

    def hand_tracking(self, cap):
        success, image = cap.read()
        image_height, image_width, _ = image.shape
        if not success:
            return 0
        # Flip the image horizontally for a later selfie-view display, and convert
        # the BGR image to RGB.
        image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        results = self.hands.process(image)
        # Draw the hand annotations on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
               
        
      #this appears to be the coordinates for finger tips in pixels? 
        
                self.mp_drawing.draw_landmarks(
                image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS,
                self.drawing_styles.get_default_hand_landmark_style(),
                self.drawing_styles.get_default_hand_connection_style())
        return image

    def record_flag(self):
        #PURPOSE: To set the record_flag to True on button press so that recording can begin
        
        self.button.configure(bg='red',)
        #video saving
        width= int(self.vs1.get(cv2.CAP_PROP_FRAME_WIDTH))
        height= int(self.vs1.get(cv2.CAP_PROP_FRAME_HEIGHT))
        #start the writer's for saving the video
        self.writer1= cv2.VideoWriter('UltrasoundVideo.mp4', cv2.VideoWriter_fourcc(*'DIVX'), 20, (width,height))
        width= int(self.vs2.get(cv2.CAP_PROP_FRAME_WIDTH))
        height= int(self.vs2.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.writer2= cv2.VideoWriter('HeadmountVideo.mp4', cv2.VideoWriter_fourcc(*'DIVX'), 20, (width,height))
        self.record = True
        print("starting recording")
    
                
    def onClose(self):
        # set the stop event, cleanup the camera, and allow the rest of
        # the quit process to continue
        print("[INFO] closing...")
        self.stopEvent.set()
        print("stop event set")

        print("saving skeletal data to csv...")
        #insert header to start of the dataFrame
        file = open('skeleData.csv', 'w', newline ="")

        with file: 
            write = csv.writer(file)
            write.writerows(self.dataFrame)
        
        print("...skeleton data saved successfully")
        if self.vs1.isOpened(): 
            self.vs1.release()

        if self.vs2.isOpened():
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
            eng.realTimeOrientationSensorSAVE(nargout=0) #simple_script is the name of the .m file!
            
        except:
            print("there was an error when stopping matlab script")
            print("shutting down matlab engine")
            eng.quit()
            
        else:
            print("shutting down matlab engine")
            eng.quit()
            
    #PURPOSE: To start a new thread for the matlab script to run
    def theCall(self):
        print("starting data collection")
        self.threadMAT = threading.Thread(target=self.trial, args=())
        
        self.threadMAT.setDaemon(True)
        
        self.threadMAT.start()
    
    def trial(self):
        while(1):
            start = time.time()

            x = 1+ 2
            end = time.time()
            print(end - start)
            time.sleep(0.00000000000000001)
    
    
##################### BODY OF THE CODE ###########################

from datetime import datetime
faulthandler.enable()
print("warming up ultrasound feed...", end = "")

### CAMERA 1 (Ultrasound)
vs1 = cv2.VideoCapture()
#below is the index (0) to get ultrasound video feed
if vs1.open(3) == True:
    print(" ultrasound feed successfully opened")
else:
    print(" ultrasound feed did not open")
time.sleep(2)



### CAMERA 2 (haedmount Feed)
print("initiating headmount cam...", end =' ')
vs2 = cv2.VideoCapture()
#NOTE: open(1) opens the Microsoft LifeCam Cinema HD USB webcam 
#NOTE: open(2) opens the RealSense USB camera connection 

if vs2.open(0) == True:
    print(" headmounted camera started")
else: 
    print("headmounted camera did not open")
time.sleep(2)

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
