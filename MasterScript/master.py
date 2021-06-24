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

class VideoStream:

    #this is the constructor of our class
        #the outputPath is where we save the videos
    def __init__(self, vs1, vs2, outputPath):
        # store the video stream object and output path, then initialize
        # the most recently read frame, thread for reading frames, and
        # the thread stop event
        self.record = False
        self.vs1 = vs1
        self.vs2 = vs2
        self.outputPath = outputPath
        self.frame1 = None
        self.thread1 = None
        self.stopEvent = None
        
        # initialize the root window and image panel
        self.root = tki.Tk()
        
        self.panel1 = None
        self.panel2 = None
        self.mypanels = {"1" : self.panel1, "2": self.panel2}
        
        
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
        
        tki.Label(self.root, text = 'Ultrasound Video').grid(row = 0, column = 0)
        tki.Label(self.root, text = 'Head-Mounted Feed').grid(row = 0, column = 1)
        
        self.button = tki.Button(self.root, text = 'Begin Recording', width = 25, command =self.record_flag)
        self.button.grid(row = 2, column = 0)
        
        self.button1 = tki.Button(self.root, text = 'Display Real-time Orientation', width = 25, command = self.theCall)
        self.button1.grid(row = 2, column = 1)
       #**************************************************************
        
        
        # start a THREAD that constantly pools the video sensor for
        # the most recently read frame
        sys.setrecursionlimit(2097152)    # adjust numbers
        threading.stack_size(134217728)   # for your needs
        self.lock = threading.Lock()
        self.stopEvent = threading.Event()
        self.thread1 = threading.Thread(target=self.videoLoop, args=(vs1,'1',))
        self.thread2 = threading.Thread(target=self.videoLoop, args=(vs2,'2',))
        
        self.thread1.setDaemon(True)
        self.thread2.setDaemon(True)
        
        self.thread1.start()
        self.thread2.start()
        
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
                    ret, frame = vs.read()
                    
                    cv2.putText(frame,str(datetime.now()),(10,30), font, .5,(255,255,255),2,cv2.LINE_AA)
                    
                    if self.record == True:
                        #print("recording")
                        if panel == "1":
                            self.writer1.write(frame)
                        else:
                            self.writer2.write(frame)
                    
                    frame = imutils.resize(frame, width=600)
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
                    
                            # otherwise, simply update the panel
                            else:
                                self.mypanels[panel].configure(image=image)
                                self.mypanels[panel].image = image
                            
                            self.lock.release()
                        else:
                            print("lock not acquired")
                    except:
                        print("something failed in locking mechanism")
                            
            except RuntimeError:
                print("[INFO] caught a RuntimeError")
                
                
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
        
        cv2.destroyAllWindows()

        print("capture stream released")
        self.root.destroy()
    
    
    #FUNCTION: To call the Matlab Engine to display real time orientation of ultrasound probe
    def pythonMatlabCall(self):
        print("starting matlab engine....")
        eng = matlab.engine.start_matlab()
        print("engine started")
        try:
            eng.realTimeOrientationAndPosition(nargout=0) #simple_script is the name of the .m file!
            
        except:
            print("there was an error when stopping matlab script")
            print("shutting down matlab engine")
            eng.quit()
            
        else:
            print("shutting down matlab engine")
            eng.quit()
            
    #PURPOSE: To start a new thread for the matlab script to run
    def theCall(self):
        self.thread3 = threading.Thread(target=self.pythonMatlabCall, args=())
        
        self.thread3.setDaemon(True)
        
        self.thread3.start()
    
    
    
    
##################### BODY OF THE CODE ###########################

from datetime import datetime
faulthandler.enable()
print("warming up camera...")

vs1 = cv2.VideoCapture()
#below is the index (0) to get ultrasound video feed
vs1.open(0)
time.sleep(2.0)

print("warming up camera 2...")
vs2 = cv2.VideoCapture()

#NOTE: open(1) opens the Microsoft LifeCam Cinema HD USB webcam 
vs2.open(1)
time.sleep(2.0)

#start the app

pba = VideoStream(vs1,vs2, "")
pba.root.mainloop()
