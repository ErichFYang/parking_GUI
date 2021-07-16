%Vehicle_pose2DCallback

function Vehicle_pose2DCallback(~,message,buffer1,buffer2,buffer3)

    LocalX = message.Pose.Pose.Position.X;
    LocalY = message.Pose.Pose.Position.Y;
    q = [message.Pose.Pose.Orientation.W message.Pose.Pose.Orientation.X ...
        message.Pose.Pose.Orientation.Y message.Pose.Pose.Orientation.Z];
    EulAngle = quat2eul(q);

    buffer1.push_back([message.Header.Stamp.seconds,LocalX]);
    buffer2.push_back([message.Header.Stamp.seconds,LocalY]);
    buffer3.push_back([message.Header.Stamp.seconds,EulAngle(1)]);

end