import matlab.engine
print("starting matlab engine....")
eng = matlab.engine.start_matlab()
print("engine started")

#first need a function that: 
# - initiates the object for the aurora NDI 
# - returns the NDI object
# try:
#     eng.workspace['x'] = eng.rand(7000, 0)
#     eng.clear

# except:
#     print("failed")
#     eng.quit() 
# else:
#     print("worked!")

try: 
    eng.workspace['device'] = eng.AuroraSetup()
    
except Exception as e: 
    print("device was not successfully initiated")
    print(e)
else:
    print("device successfully initiated... ready in python")
#second need a function that: 
# - recieves NDI object and begins tracking on this object when a button is pressed 
try:
    eng.realTimeOrientationSensorSAVEFModular("ok", eng.workspace['device'], nargout=0) #simple_script is the name of the .m file!
    
except Exception as e:
    print("there was an error when stopping matlab script")
    print(e)
    print("shutting down matlab engine")
    eng.clear
    eng.quit()
    
else:
    print("it worked!!!")
    eng.quit()
