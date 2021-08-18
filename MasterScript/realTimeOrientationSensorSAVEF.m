%This is the function version of realTimeOrientationSensorSAVE
%runs the collection of coordinates for the magnetic tracker and saves the data to a .txt file. This is designed for Python GUI integration 
function [] = realTimeOrientationSensorSAVEF(segment)

    segment
    fprintf("starting setup \n");
    aurora_device = AuroraSetup()
    fprintf("setup complete \n");
    
    %segmentName = input("Please enter procedure name: ", 's');
    rot1 = [];
    rot2 = [] ;
    trans1 = [] ;
    trans2 = [];
    error = [];
    frame = [];
    time = [];
    sensorStat = [];
    
    aurora_device.startTracking();

   
%   aurora_device.BEEP('1');
    

hFigure = figure('Name','US Probe Orientation','NumberTitle','off','Position',[845 268 500 540])
set(hFigure, 'MenuBar', 'none');
set(hFigure, 'ToolBar', 'none');
WindowAPI(hFigure, 'TopMost')

% Create a 3-D plot
ax = axes('XLim', [-500 500], 'YLim', [-500 500], 'ZLim', [-500 500]);
xlabel(ax, 'X-axis');
ylabel(ax, 'Y-axis');
zlabel(ax, 'Z-axis');
% Reverse the 2 axis directions to match the device coordinate system
set(ax, 'Zdir', 'reverse');
set(ax, 'Xdir', 'reverse');
grid on; view(3);

% Define the surface color
color = [0.3010 0.7450 0.9330];

% Create patches for all cube surfaces by specifying the four corners of each surface
top = [-0.5 -0.5 2; 0.5 -0.5 2; 0.5 0.5 2; -0.5 0.5 2];
top = top*100;
p(1) = patch(top(:,1), top(:,2), top(:,3), color);

bottom = [-0.5 -0.5 0; 0.5 -0.5 0; 0.5 0.5 0; -0.5 0.5 0];
bottom = bottom*100;
p(2) = patch(bottom(:,1), bottom(:,2), bottom(:,3), [1, 0, 0]);

front = [0.5 -0.5 0; 0.5 0.5 0; 0.5 0.5 2; 0.5 -0.5 2];
front = front*100;
p(3) = patch(front(:,1), front(:,2), front(:,3), color);

back = [-0.5 -0.5 0; -0.5 0.5 0; -0.5 0.5 2; -0.5 -0.5 2];
back = back*100;
p(4) = patch(back(:,1), back(:,2), back(:,3), color);

left = [0.5 -0.5 0; -0.5 -0.5 0; -0.5 -0.5 2; 0.5 -0.5 2];
left = left*100;
p(5) = patch(left(:,1), left(:,2), left(:,3), color);

right = [0.5 0.5 0; -0.5 0.5 0; -0.5 0.5 2; 0.5 0.5 2];
right = right*100;
p(6) = patch(right(:,1), right(:,2), right(:,3), color);
%0.9 = 0, 0.7 = 0.3
mark = [0 -0.3 2.01; 0.3 -0.3 2.01; 0.3 0 2.01; 0 -0 2.01];
mark = mark*100;
p(7) = patch(mark(:,1), mark(:,2), mark(:,3), 'black');
%ADD THIS PORTION TO THE NEW SCRIPT AND TEST THAT IT SHOWS THE RIGHT SIZE
%AND IS NOT HIGH LATENCY
[x,y,z] = sphere(1);
g = 60;
fvc = surf2patch(x*g,y*g,z*g - 50);
X = patch('Faces', fvc.faces, 'Vertices', fvc.vertices, 'FaceColor', [1, 0, 0]);
p(8) = X;

% Set the object transparency
alpha(0.8)


%UNFINISHED - ADD THIS PART TWEAK CODE TO GET IT TO RUN WITH MY STUFF 

%Now use a Transform object to handle the rotations 

tfObject = hgtransform('Parent', ax); %hgtransform creates a transform object and returns its handle


set(p, 'Parent', tfObject); %sets the parent of 'p' to the tfObject


  
   x = 0;
   hWaitbar = waitbar(0, 'Stop', 'Name', 'Collecting Data','CreateCancelBtn','delete(gcbf)');
   drawnow; 
   while(1)
        
        tic
        aurora_device.updateSensorDataAll()
        trans = aurora_device.port_handles.trans;
        rot =  aurora_device.port_handles.rot;
        trans1 = [trans1; trans];
        rot1 = [rot1; rot];
        error = [error; aurora_device.port_handles(1,1).error];
        frame = [frame; aurora_device.port_handles(1,1).frame_number];
        sensorStat = [sensorStat; aurora_device.port_handles(1,1).sensor_status];

        %plot in real-time






%now, we want to pull device data in a loop and use the data to update the
%object orientation. 

            

                % Acquire device data
                %access quaternion by the following if you are reading in data from the
                %workspace: 
                quat = rot;
                if quat == [0,0,0,0]
                    time = [time; toc];
                    continue 
                end 

                % Prepare 4-by-4 transform matrix to plot later
                transformMatrix = eye(4);

                %convert quat to rotation matrix 
                rotm = quat2rotm(quat);
                
                %%%WHEN STANDING UP I NEED TO ROTATE Y AXIS -90 degrees
                b = pi/2;
                R = [cos(b) 0 sin(b); 0 1 0; -1*sin(b) 0 cos(b)];
                rotm = R*rotm;
                
                
                % Populate the transform matrix with 9 rotation matrix elements
                
                for row = 1:3
                    for column = 1:3
                        % Extract the 2 bytes representing the current element in the rotation matrix
                        element = rotm(row, column);
                        transformMatrix(row, column) = element;
                    end
                end

                %populate the translation entries in the transform matrix with position
                  
                %for up on its side
                transformMatrix(1, 4) = -1*trans(3);
                transformMatrix(2, 4) = 1*trans(2);
                transformMatrix(3, 4) = 1*trans(1);
                
                
                %for on the bottom, use this code instead and comment the
                %above part out
%                  for row = 3:1
%                         transformMatrix(row, 4) = trans(row);
%                  end

                
                % Update plot
                set(tfObject, 'Matrix', transformMatrix);
                time = [time; toc];
                pause(0.00000000000000001);
                
                
                if ~ishandle(hWaitbar)
                    % Stop the if cancel button was pressed
                    disp('Stopped by user');
                    break;
                else
                    % Update the wait bar
                    waitbar(0, hWaitbar, ['Close this window to cancel']);
                end

    end
    
    
    frameDiff = checkFrameDiff(frame);

    avgDiff = average(frameDiff)

       
    
    aurora_device.stopTracking();
    dir = "C:\Users\randy\Documents\GitHub\RemoteUltrasoundCode\MasterScript\projectOut\";
    filetype = ".csv";
    filepath = append(dir, char(segment), filetype);    
    data = save2File(rot1, [], trans1, [], error, time, sensorStat, frame, filepath);
    close(hFigure);
end


