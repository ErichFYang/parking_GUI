%imuCallback

function imuCallback(~,message,buffer)
Ax = message.LinearAcceleration.X;
Ay = message.LinearAcceleration.Y;
buffer.push_back([message.Header.Stamp.seconds, Ax, Ay])
end
