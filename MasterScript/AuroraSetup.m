function device = AuroraSetup()
fprintf("starting setup \n")
fprintf("searching for device \n");
aurora_device = AuroraDriver('COM8');
serial_present = instrfind;

if(~isempty(serial_present)) 
    fprintf("device found... connecting \n" );
    aurora_device.openSerialPort();
    aurora_device.setBaudRate(230400);
    fprintf("device connected... initializing Aurora \n");
    aurora_device.init();
    aurora_device.detectAndAssignPortHandles();
    aurora_device.initPortHandleAll();
    aurora_device.enablePortHandleDynamicAll();
    
    fprintf("aurora ready for tracking \n");
    device = aurora_device;
%   aurora_device.startTracking();
%     aurora_device.BEEP('1');
%     aurora_device.stopTracking();
%     delete(aurora_device);
    
end
