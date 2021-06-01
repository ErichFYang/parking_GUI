%Vehicle_pose2DCallback

function Vehicle_pose2DCallback(src,message)
global LocalX       % 纵向位置
global LocalY       % 横向位置
global Yaw          % 汽车偏航角
Vehicle_pose2D_msgStructs = readMessages(message,'DataFormat','struct');
X = cellfun(@(Vehicle_pose2D) double(Vehicle_pose2D.Pose.Pose.Position.X),Vehicle_pose2D_msgStructs);
Y = cellfun(@(Vehicle_pose2D) double(Vehicle_pose2D.Pose.Pose.Position.Y),Vehicle_pose2D_msgStructs);
YawX = cellfun(@(Vehicle_pose2D) double(Vehicle_pose2D.Pose.Pose.Orientation.X),Vehicle_pose2D_msgStructs);
YawY = cellfun(@(Vehicle_pose2D) double(Vehicle_pose2D.Pose.Pose.Orientation.Y),Vehicle_pose2D_msgStructs);
time1=cellfun(@(Vehicle_pose2D) double(Vehicle_pose2D.Header.Stamp.Sec+0.001*Vehicle_pose2D.Header.Stamp.Nsec),Vehicle_pose2D_msgStructs);
Yaw = [time1,atan(YawY/YawX)];
LocalX = [time1,X];
LocalY = [time1,Y];
end