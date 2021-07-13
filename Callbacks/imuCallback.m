%imuCallback

function imuCallback(~,message,buffer)
global LocalAx
global LocalAy

% imu_msgStructs = readMessages(message,'DataFormat','struct');
% Ax=cellfun(@(imu) double(imu.LinearAcceleration.X),imu_msgStructs);                  %纵向加速度
% Ay=cellfun(@(imu) double(imu.LinearAcceleration.Y),imu_msgStructs);                  %横向加速度
% time4 = cellfun(@(imu) double(imu.Header.Stamp.Sec+0.001*imu.Header.Stamp.Nsec),imu_msgStructs);
% LocalAx = [time4,Ax];
% LocalAy = [time4,Ay];
LocalAx = [message.Header.Stamp.seconds,message.LinearAcceleration.X];
LocalAy = [message.Header.Stamp.seconds,message.LinearAcceleration.Y];
Ax = message.LinearAcceleration.X;
Ay = message.LinearAcceleration.Y;
buffer.push_back([message.Header.Stamp.seconds, Ax, Ay])
end
