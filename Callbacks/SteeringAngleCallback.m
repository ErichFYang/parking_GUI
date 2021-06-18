%SteeringAngleCallback

function SteeringAngleCallback(~, message)
global angle;        % 方向盘转角
angle = [message.Header.Stamp.seconds, -message.Angle];
end