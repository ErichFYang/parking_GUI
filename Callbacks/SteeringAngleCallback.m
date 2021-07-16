%SteeringAngleCallback

function SteeringAngleCallback(~,message,buffer)
%global angle        % 方向盘转角
% 
% SteeringAngle_msgStructs = readMessages(message,'DataFormat','struct');
% Angle= cellfun(@(SteeringAngle) double(SteeringAngle.Angle),SteeringAngle_msgStructs);
% time2= cellfun(@(SteeringAngle) double(SteeringAngle.Header.Stamp.Sec+0.001*SteeringAngle.Header.Stamp.Nsec),SteeringAngle_msgStructs);
% angle = [time2,Angle];
buffer.push_back([message.Header.Stamp.seconds, message.Angle]);
end