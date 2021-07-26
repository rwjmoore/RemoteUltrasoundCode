
def NDI_pytrack():
    # Install a pip package in the current Jupyter kernel
    import numpy as np
    import sys
    import pandas

    from sksurgerynditracker.nditracker import NDITracker
    settings_aurora = {'tracker type':'aurora', 'verbose': True,  "use quaternions": True}
    TRACKER = NDITracker(settings_aurora)



    #'serial port' : '/dev/cu.usbserial-00003014'


    #Don't hit the above one again unless you want some serious beeping going on 

    ports =[]
    time = []
    frames = []
    trackingData = []
    error = []
    print("thread initiated")
    TRACKER.start_tracking()
    print("starting tracking")
    try: 
        for i in range(100000):
                port_handles, timestamps, framenumbers, tracking, quality = TRACKER.get_frame()
    #             for t in tracking:
    #                 print (t)
                ports = port_handles
                time.append(timestamps)
                frames.append(framenumbers)
                trackingData.append(tracking)
                error.append(quality)
    except: 
        print("Something went wrong... stopping tracking")
        TRACKER.stop_tracking()

    else: 
        TRACKER.stop_tracking()

    TRACKER.close()

        #Grab the frame difference to see if the data skipped any sections  
    frameDiff = []

    for i in range(1, len(frames) - 2):
        #have to do this because they are saved as lists
        frameDiff.append(frames[i + 1][0] - frames[i][0])
        
    print("\nThe max frame difference was " + str(max(frameDiff)))

    def average(array):
        return sum(array)/len(array)

    print("The average frame difference was " + str(average(frameDiff)))

    #parsing data into saveable format 

    from pandas.core.common import flatten
    time = list(flatten(time))
    error = list(flatten(error))
    TrackingData = []

    #formats the tracking data in a list of numpy arrays. First index is for index in list, second is for index in numpy array
    for i in range(len(trackingData)):
        TrackingData.append(trackingData[i][0][0])
    TrackingData = np.array(TrackingData)

    #get into a data frame by putting all into dictionaries 
    """note that the columns 0-2 equal x,y,z and 3-6 are q0 qx qy qz"""
    #leaving out "ports": ports
    myDict = { "Tx": TrackingData[:, 0], "Ty": TrackingData[:, 1], "Tz": TrackingData[:, 2], "Qo": TrackingData[:,3], "Qx": TrackingData[:, 4], "Qy": TrackingData[:, 5], "Qz": TrackingData[:, 6],"time": time, "frames": frames, "error": error }

    #now write it to a dataframe
    df = pd.DataFrame(myDict)

    df.to_csv("magProbe.csv")