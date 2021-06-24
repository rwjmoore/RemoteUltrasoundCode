"""Connection procedure based off of this website tutorial:
 http://dfrobot.blogspot.com/2018/07/esp32-bluetooth-tutorial-receiving-data.html"""

#use this function to find the BT device address that you want to use
def findMyBT():
    devices = discover_devices(lookup_names=True)
    print(type(devices))
    
    print("Devices found: %s" % len(devices))
    
    for item in devices:
        print(item)

from bluetooth import *

findMyBT()
#RFCOMM emulates a serial port but in bluetooth format 
BTsocket = BluetoothSocket(RFCOMM)
BTsocket.connect((('FC:F5:C4:3D:27:8E', 'ESP32test')))


