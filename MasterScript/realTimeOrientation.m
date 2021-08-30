%Follow this website to provide a real-time orientation of the device
%Here is the link: https://www.mathworks.com/help/matlab/import_export/track-orientation-of-bluetooth-low-energy-device.html


clear;
load("C:\Users\randy\Documents\GitHub\Summer21Research\API-AuroraSystem\MatlabWorkspaces\Workspace_for_orientation_testing.mat")

%hFigure = figure('Name','US Probe
%Orientation','NumberTitle','off','Position',[1032 303.5 320 172])  %OLD
%COORDINATES
hFigure = figure('Name','US Probe Orientation','NumberTitle','off','Position',[1032 331.5 320 172]) 

set(hFigure, 'MenuBar', 'none');
set(hFigure, 'ToolBar', 'none');

WindowAPI(hFigure, 'TopMost')
% Create a 3-D plot
ax = axes('XLim', [-500 500], 'YLim', [-500 500], 'ZLim', [-500 200]);
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
[x,y,z] = sphere(6);
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
