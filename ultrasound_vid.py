#this script is capable of grabbing video from the
#digital to analog converter

import cv2
from datetime import datetime
cap = cv2.VideoCapture()
# The device number might be 0 or 1 depending on the device and the webcam
#cap.open(0, cv2.CAP_AVFOUNDATION)
cap.open(3)



while(True):
    ret, frame = cap.read()
    
    #put the date and time over the video feed
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame,str(datetime.now()),(10,30), font, .5,(255,255,255),2,cv2.LINE_AA)
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
