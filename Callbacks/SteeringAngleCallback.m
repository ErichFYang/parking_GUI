%SteeringAngleCallback

function SteeringAngleCallback(~, message)
global angle;        % 方向盘转角
angle.push_back([message.Header.Stamp.seconds, -message.Angle]);
end