%velometerCallback

function velometerCallback(~,message,buffer)
global LocalVx      %纵向速度
global LocalVy      %横向速度
global VehicleSpeed % 车速

% 
% velometer_msgStructs = readMessages(message,'DataFormat','struct');
% Vx= cellfun(@(velometer) double(velometer.Twist.Linear.X),velometer_msgStructs);
% Vy= cellfun(@(velometer) double(velometer.Twist.Linear.Y),velometer_msgStructs); 
% Speed=sqrt(Vx.*Vx+Vy.*Vy);
% time3 = cellfun(@(velometer) double(velometer.Header.Stamp.Sec+0.001*velometer.Header.Stamp.Nsec),velometer_msgStructs);
% LocalVx = [time3,Vx]; 
% LocalVy = [time3,Vy];
% VehicleSpeed=[time3,Speed];

LocalVx.push_back([message.Header.Stamp.seconds,message.Twist.Linear.X])
LocalVy.push_back([message.Header.Stamp.seconds,message.Twist.Linear.Y])
Speed=sqrt(Vx.*Vx+Vy.*Vy);
buffer.push_back([message.Header.Stamp.seconds, Speed]);
end