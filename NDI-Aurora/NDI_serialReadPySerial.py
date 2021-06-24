import serial
import serial.tools.list_ports
print(serial.tools.list_ports.comports)
ser = serial.Serial('/dev/tty.usbserial-00005014') #this is the usb port found in
ser_bytes= print(ser.read())
#print("read the bytes")
#print(ser_bytes)
