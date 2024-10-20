# Remote Ultrasound Code
This project represents a deployable system for quantitatively assessing ultrasound examination performace partially funded by the University of Calgary's Department of Surgery and supervised by Dr. Paul B. McBeth. 


This project aims to address the incoming need for infrastructure and evidence needed to implement telehealth in light of improved satellite communications. StarLink is set to revolutionize how we access the internet. Before utilizing this resource to improve healthcare delivery in remote regions, proof is needed of it's utility, and a plan is needed for its implementation. 

This software suite tackles two goals:

 <b>1: To quantitatively analyze ultrasound examinations to validate how well ultrasound exams can be performed through remote (telemonitored) guidance. 

 2: To facilitate a remote, telemonitored ultrasound examination by recording, and streaming live video. </b>
 
Goal 1. is achieved by interfacing with NDI's Aurora Magnetic Tracking device to track ultrasound probe movement to a high degree of precision. Written in Python, this program interfaces with the MATLAB engine to connect and process the magnetic tracking data. Two neural networks using the [mediaPipe framework](https://ai.google.dev/edge/mediapipe/solutions/guide) are concurrently run to process live video in order to track both skeleton and hand movements. 
 
Goal 2. is achieved via an ADC conversion of incoming ultrasound images from a Sonosite NanoMaxx Transducer, which is then displayed in a GUI served to a remote user via the Zoom Teleconference Platform. The Tx and Rx speeds of the current connection are also monitored and displayed to the user. 

This code was optimized to run on standard OS and PC's found in hospital settings. As such, the below demo was conducted on a Windows PC with a quadcore i7 of the 10th generation. 

![alt text](https://github.com/rwjmoore/RemoteUltrasoundCode/blob/4bc53d44e5bd929fcfeeb0514584e37832f77591/US%20screen%20capture.png?raw=true)

The figure above depicts the Remote Ultrasound Code Graphical User Interface that is shown to the remote clinican telemonitoring the procedure in real-time. Notable features include the ultrasound image, the frame used for tracking the posture of the sonographer, and the head-mounted Point of View (POV) camera used to facilitate the telemonitoring as well as track the pose of the sonographers hands. 
