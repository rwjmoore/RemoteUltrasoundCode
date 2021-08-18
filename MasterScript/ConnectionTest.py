import speedtest
s = speedtest.Speedtest()
currentDSpeed = s.download()
currentUspeed = s.upload()

#convert to Mbps
currentDSpeed = currentDSpeed/1000000
print(currentDSpeed)