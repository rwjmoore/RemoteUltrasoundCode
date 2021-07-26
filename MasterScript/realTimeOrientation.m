%Follow this website to provide a real-time orientation of the device
%Here is the link: https://www.mathworks.com/help/matlab/import_export/track-orientation-of-bluetooth-low-energy-device.html


clear;
load("C:\Users\randy\Documents\GitHub\Summer21Research\API-AuroraSystem\MatlabWorkspaces\Workspace_for_orientation_testing.mat")


% Create a 3-D plot
ax = axes('XLim', [-2 2], 'YLim', [-2 2], 'ZLim', [-1 2]);
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
p(1) = patch(top(:,1), top(:,2), top(:,3), color);

bottom = [-1 -1 0; 1 -1 0; 1 1 0; -1 1 0];
p(2) = patch(bottom(:,1), bottom(:,2), bottom(:,3), color);

front = [1 -1 0; 1 1 0; 1 1 1; 1 -1 1];
p(3) = patch(front(:,1), front(:,2), front(:,3), color);

back = [-1 -1 0; -1 1 0; -1 1 1; -1 -1 1];
p(4) = patch(back(:,1), back(:,2), back(:,3), color);

left = [1 -1 0; -1 -1 0; -1 -1 1; 1 -1 1];
p(5) = patch(left(:,1), left(:,2), left(:,3), color);

right = [1 1 0; -1 1 0; -1 1 1; 1 1 1];
p(6) = patch(right(:,1), right(:,2), right(:,3), color);

mark = [0.9 -0.7 -0.01; 0.7 -0.7 -0.01; 0.7 -0.9 -0.01; 0.9 -0.9 -0.01];
p(7) = patch(mark(:,1), mark(:,2), mark(:,3), 'black');

% Set the object transparency
alpha(0.5)


%UNFINISHED - ADD THIS PART TWEAK CODE TO GET IT TO RUN WITH MY STUFF 

%Now use a Transform object to handle the rotations 

tfObject = hgtransform('Parent', ax); %hgtransform creates a transform object and returns its handle


set(p, 'Parent', tfObject); %sets the parent of 'p' to the tfObject


%now, we want to pull device data in a loop and use the data to update the
%object orientation. 
while(1)
for loop = 1:(length(rot1)-1)
    % Acquire device data
    %access quaternion by the following if you are reading in data from the
    %workspace: 
    quat = rot1(loop,:)
    
    
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
%     
%      for row = 1:3
%         
%             element = rotm(row, 4);
%             transformMatrix(row, 4) = trans(row, 4);
%     end
    
    
    % Update plot
    set(tfObject, 'Matrix', transformMatrix);
    pause(0.1);
end

end
