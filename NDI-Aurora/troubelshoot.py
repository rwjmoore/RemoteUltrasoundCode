import ndicapy
for i in range(-5000, 0):
    serial_port = i # the integer address of the serial port if you know it
    name = ndicapy.ndiDeviceName(serial_port)
    if name != None:
        print (name)



# result = ndicapy.ndiProbe(name)
# assert result == ndicapy.NDI_OKAY
