from sksurgerynditracker.nditracker import NDITracker
settings_aurora = {'tracker type':'aurora', 'ports to probe': 20, 'verbose': True, 'serial port' : '/dev/cu.usbserial-00003014'}
TRACKER = NDITracker(settings_aurora)

TRACKER.start_tracking()
for i in range(30):
    port_handles, timestamps, framenumbers, tracking, quality = TRACKER.get_frame()
    for t in tracking:
        print (t)
    
TRACKER.stop_tracking()
TRACKER.close()
