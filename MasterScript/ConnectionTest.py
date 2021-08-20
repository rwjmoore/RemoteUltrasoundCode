import speedtest
count = 0
while(1):
    if count%30 == 0:
        s = speedtest.Speedtest()
        print("testing network download speed...")
        currentDSpeed = s.download()
        print("testing network upload speed...")
        currentUSpeed = s.upload()


        #convert to Mbps
        currentDSpeed = currentDSpeed/1000000
        print("Download Speed = "+ str(currentDSpeed))

        currentUSpeed = currentUSpeed/1000000
        print("Upload Speed = "+ str(currentUSpeed))
        count = count + 1
    else:
        count = count + 1



