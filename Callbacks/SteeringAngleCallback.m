%SteeringAngleCallback

function SteeringAngleCallback(~, message, buffer)
buffer.push_back([message.Header.Stamp.seconds, -message.Angle]);
end