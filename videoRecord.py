import cv2
from datetime import datetime
cap= cv2.VideoCapture()
import time
#below is the index (0) to get ultrasound video feed
#cap.open(0, cv2.CAP_AVFOUNDATION) this is for MAC!
#cap.open(5) Works for US when connected into the USB3.0 port 

#0 opens Lidar cam / ultrasound /ultrasound
#1 lidar / lidar / lidar
#2
#3 normal cam / / nothing 
#4 opens stereo depth / normal/ normal cam
#5 ultrasound / steroe / sterepo
cap.open(0)
width= int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height= int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))


writer= cv2.VideoWriter('Test.mp4', cv2.VideoWriter_fourcc(*'DIVX'), 20, (width,height))

while True:
    ret,frame= cap.read()

    
    #put the date and time over the video feed
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame,str(datetime.now()),(10,30), font, .5,(255,255,255),2,cv2.LINE_AA)

    writer.write(frame)
    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break
    time.sleep(0.00001)

cap.release()
writer.release()
cv2.destroyAllWindows()
