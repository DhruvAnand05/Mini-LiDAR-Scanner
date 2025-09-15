%dhruv anand - 400535222 - 3d tof sensor scan

clear;
portList = serialportlist("available"); %list available serial ports
%display(portList)
s = serialport(portList(1), 115200, 'Timeout', 100); %open serial connection
flush(s);%clear serial buffer

%parameters
maxDepth = 3;              %number of depth layers to scan
rotSteps = 32;             %32 steps per full rotation (360°/11.25°)
result = zeros(0, 3);      %matrix to store [distance(mm), angle(deg), depth(mm)]

%data collection from uart
i = 0;
while(i <= maxDepth * rotSteps) %amount of data to read is 32*depth
    data = readline(s);%read one line from uart
    if isempty(data)%skip if no data received
        break;
    end

    tempData = sscanf(data, '%f, %f, %f')';%parse data into 3 columns

    if numel(tempData) == 3%only proceed if all 3 values were read
        distance = tempData(1);%distance in mm
        angleDeg = tempData(2) * (360 / 512);%convert steps to degrees
        depth = tempData(3) * 20;%scale depth to (20cm increments)
        result = [result; [distance, angleDeg, depth]];%append to results
        disp([distance, angleDeg, depth]); %debug print
        i = i + 1;
    end
end

%convert polar to cartesian coordinates
theta = result(:, 2) * (pi / 180); %convert degrees to radians
x = result(:, 3);                  %x-axis = depth
y = result(:, 1) .* cos(theta);    %y-axis = distance x cos
z = result(:, 1) .* sin(theta);    %z-axis = distance × sin

%create 3d plot
figure;
scatter3(x, y, z, 20, 'filled'); %plot points as filled circles
hold on;
title('3d tof sensor scan - dhruv anand (400535222)');
xlabel('x depth [mm]');
ylabel('y width [mm]');
zlabel('z height [mm]');
grid on;

%find all unique depth values in the data
uniqueDepths = unique(result(:, 3));  

%connect points at same depth (black circles)
for d = 1:length(uniqueDepths) %loop through each unique depth
    depthPoints = find(result(:, 3) == uniqueDepths(d)); %find indices for current depth
    [~, sortedIdx] = sort(result(depthPoints, 2)); %sort points by angle
    plot3(x(depthPoints(sortedIdx)), y(depthPoints(sortedIdx)), z(depthPoints(sortedIdx)), 'k-'); %connect sorted points with black lines
end

%connect adjacent depths (blue helical lines)
for d = 1:length(uniqueDepths)-1 %loop through each depth except the last
    currentDepthPoints = find(result(:, 3) == uniqueDepths(d)); %find indices of points at the current depth
    nextDepthPoints = find(result(:, 3) == uniqueDepths(d+1)); %find indices of points at the next depth
    [~, currSorted] = sort(result(currentDepthPoints, 2)); %sort current depth points by angle
    [~, nextSorted] = sort(result(nextDepthPoints, 2)); %sort next depth points by angle
    if length(currentDepthPoints) == length(nextDepthPoints) %only connect if same number of points at both depths
        for i = 1:length(currentDepthPoints) %loop through each point at the current depth
            plot3([x(currentDepthPoints(currSorted(i))), x(nextDepthPoints(nextSorted(i)))],... %x-coordinates of connected points
                  [y(currentDepthPoints(currSorted(i))), y(nextDepthPoints(nextSorted(i)))],... %y-coordinates of connected points
                  [z(currentDepthPoints(currSorted(i))), z(nextDepthPoints(nextSorted(i)))], 'b-'); %z-coordinates of connected points, draw blue lines
        end
    end
end
hold off; %release hold on the figure
