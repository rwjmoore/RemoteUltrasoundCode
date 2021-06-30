%This scripts runs the collection of coordinates for the magnetic tracker
%Script returns coordinates in terms of milimetres 

    clear;
    fprintf("starting setup \n");
    aurora_device = AuroraSetup()
    fprintf("setup complete \n");
    
    aurora_device.startTracking();

   
%   aurora_device.BEEP('1');
    

hFigure = figure('Name','US Probe Orientation','NumberTitle','off','Position',[845 268 500 540])
set(hFigure, 'MenuBar', 'none');
set(hFigure, 'ToolBar', 'none');

% Create a 3-D plot
ax = axes('XLim', [-200 200], 'YLim', [-200 200], 'ZLim', [-500 200]);
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
top = [-1 -1 1; 1 -1 1; 1 1 1; -1 1 1];
top = top*100;
p(1) = patch(top(:,1), top(:,2), top(:,3), color);

bottom = [-1 -1 0; 1 -1 0; 1 1 0; -1 1 0];
bottom = bottom*100;
p(2) = patch(bottom(:,1), bottom(:,2), bottom(:,3), color);

front = [1 -1 0; 1 1 0; 1 1 1; 1 -1 1];
front = front*100;
p(3) = patch(front(:,1), front(:,2), front(:,3), color);

back = [-1 -1 0; -1 1 0; -1 1 1; -1 -1 1];
back = back*100;
p(4) = patch(back(:,1), back(:,2), back(:,3), color);

left = [1 -1 0; -1 -1 0; -1 -1 1; 1 -1 1];
left = left*100;
p(5) = patch(left(:,1), left(:,2), left(:,3), color);

right = [1 1 0; -1 1 0; -1 1 1; 1 1 1];
right = right*100;
p(6) = patch(right(:,1), right(:,2), right(:,3), color);

mark = [0.9 -0.7 -0.01; 0.7 -0.7 -0.01; 0.7 -0.9 -0.01; 0.9 -0.9 -0.01];
mark = mark*100;
p(7) = patch(mark(:,1), mark(:,2), mark(:,3), 'black');

% Set the object transparency
alpha(0.5)


%UNFINISHED - ADD THIS PART TWEAK CODE TO GET IT TO RUN WITH MY STUFF 

%Now use a Transform object to handle the rotations 

tfObject = hgtransform('Parent', ax); %hgtransform creates a transform object and returns its handle


set(p, 'Parent', tfObject); %sets the parent of 'p' to the tfObject


    time = [];
    frame = [];
   x = 0;
    for loop = 1:1000
        
        tic
        aurora_device.updateSensorDataAll()
        trans1 = aurora_device.port_handles.trans;
        rot1 =  aurora_device.port_handles.rot;
        frame = [frame; aurora_device.port_handles.frame_number];
        
        %plot in real-time






%now, we want to pull device data in a loop and use the data to update the
%object orientation. 

            

                % Acquire device data
                %access quaternion by the following if you are reading in data from the
                %workspace: 
                quat = rot1;
                if quat == [0,0,0,0]
                    continue 
                end 

                % Prepare 4-by-4 transform matrix to plot later
                transformMatrix = eye(4);

                %convert quat to rotation matrix 
                rotm = quat2rotm(quat);
                
                % Populate the transform matrix with 9 rotation matrix elements
                for row = 1:3
                    for column = 1:3
                        % Extract the 2 bytes representing the current element in the rotation matrix
                        element = rotm(row, column);
                        transformMatrix(row, column) = element;
                    end
                end

                %populate the translation entries in the transform matrix with position

                 for row = 1:3
                        transformMatrix(row, 4) = trans1(row);
                 end

                
                % Update plot
                set(tfObject, 'Matrix', transformMatrix);
                time = [time; toc];
                pause(0.00000000000000001);

            end


       
    
    aurora_device.stopTracking();

% % fires when main function terminates
%     function cleanMeUp(aurora_device)
%         
%         fprintf('Stopping Tracking...\n');
%         aurora_device.stopTracking();
%     end


% 
% function [] = plotPoints(aurora_device)
%     plot3(x, y, z, "r*"), grid on, xlabel("x"), ylabel("y"), zlabel("z")
%     xlim([-260, 400]), ylim([-260, 400]), zlim([0, 500])
%     drawnow;

