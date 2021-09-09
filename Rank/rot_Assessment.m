%原地转向时长rot：泊车过程中驾驶员原地打方向盘的总时长

function [score]=rot_Assessment(VehicleSpeed,angle)
% pick the data fulfilled requirement
VehicleSpeed = VehicleSpeed .* (VehicleSpeed(:,2) <= speedLimit);
VehicleSpeed(VehicleSpeed(:, 1) == 0, :) = [];
% change the resolution of time for data alignment
VehicleSpeed(:,1) = fix(VehicleSpeed(:,1) ./ timeRange);
angle(:,1) = fix(angle(:,1) ./ timeRange);

% find the angle data in the specific time
rowStart = 1;
rowEnd = size(angle,1);
rotCount = 0;
for counter = 1 : size(VehicleSpeed, 1)
    row = (find(angle(rowStart:rowEnd,1) == VehicleSpeed(counter, 1), 10));
    row = row + rowStart - 1;
    if isempty(row)
        continue
    end
    angleChange = max(angle(max(row(1), 1): min(row(end), rowEnd), 2)) - ...
        min(angle(max(row(1), 1): min(row(end), rowEnd), 2)); 
    % count the rot times
    if angleChange >= angleLimit
        rotCount = rotCount + 1;
    end
    rowStart = row(end); 
end

% calculate the rot time percentage
rotPercent = min(1, rotCount / fix(angle(end,1)-angle(1,1)));

%原地转向分值
if rotPercent <= 0.1 % 10%以内的时间原地转向
    score = 10;
elseif 0.1 < rotPercent && rotPercent <= 0.2
    score = 6;
elseif 0.2 < rotPercent && rotPercent <= 0.3
    score = 2;
else
    score = 0;
end

end

% my Constant
function const = speedLimit
    const = 0.01;  % unit: m/s
end

function const = timeRange
    const = 0.3;  % unit: s
end

function const = angleLimit
    const = 1;   % unit: degree
end