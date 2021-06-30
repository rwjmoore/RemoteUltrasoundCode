import cv2
from datetime import datetime
cap= cv2.VideoCapture()
#below is the index (0) to get ultrasound video feed
#cap.open(0, cv2.CAP_AVFOUNDATION) this is for MAC!
cap.open(2)
width= int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height= int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))


writer= cv2.VideoWriter('UltrasoundVideo.mp4', cv2.VideoWriter_fourcc(*'DIVX'), 20, (width,height))

while True:
    ret,frame= cap.read()

    
    #put the date and time over the video feed
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame,str(datetime.now()),(10,30), font, .5,(255,255,255),2,cv2.LINE_AA)

    writer.write(frame)
    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break


cap.release()
writer.release()
cv2.destroyAllWindows()
