%%parkingslotCallback

function parkingslotCallback(~,message,buffer)

RefPoseX = message.RefPose.X;
RefPoseY = message.RefPose.Y;
RefPoseTheta = message.RefPose.Theta;
RefExtend_X = message.RefExtendX;
RefExtend_Y = message.RefExtendY;

T1 = [cos(RefPoseTheta), -sin(RefPoseTheta), RefPoseX; sin(RefPoseTheta), cos(RefPoseTheta), RefPoseY; 0, 0, 1];
RefPose1 = [RefPoseX;RefPoseY;1];
b1 = [ 0; RefExtend_Y; 1];
c1 = [ RefExtend_X;0;1];
d1 = [ RefExtend_X;RefExtend_Y; 1];
RefPose2 = T1*b1;
RefPose3 = T1*c1;
RefPose4 = T1*d1;

ObstaclePoseX = message.ObstaclePose.X;
ObstaclePoseY = message.ObstaclePose.Y;
ObstacleTheta = message.ObstaclePose.Theta;
ObstacleExtend_X = message.ObstacleExtendX;
ObstacleExtend_Y = message.ObstacleExtendY;
T2 = [cos(ObstacleTheta), -sin(ObstacleTheta), ObstaclePoseX; sin(ObstacleTheta), cos(ObstacleTheta), ObstaclePoseY; 0, 0, 1];
ObstaclePose1 = [ObstaclePoseX,ObstaclePoseY];
b2 = [ 0; ObstacleExtend_Y; 1];
c2 = [ ObstacleExtend_X;0;1];
d2 = [ ObstacleExtend_X;ObstacleExtend_Y; 1];
ObstaclePose2 = T2*b2;
ObstaclePose3 = T2*c2;
ObstaclePose4 = T2*d2;

buffer.push_back([message.Header.Stamp.seconds,RefPose1(1),RefPose1(2), ...
    RefPose2(1),RefPose2(2),RefPose3(1),RefPose3(2),RefPose4(1),RefPose4(2), ...
    ObstaclePose1(1), ObstaclePose1(2),ObstaclePose2(1), ObstaclePose2(2), ...
    ObstaclePose3(1), ObstaclePose3(2),ObstaclePose4(1), ObstaclePose4(2), ...
    RefPoseTheta])
end


% function parkingslotCallback(~,message,buffer1,buffer2,buffer3,buffer4,buffer5,buffer6,buffer7,buffer8,buffer9)
% 
% RefPoseX = message.RefPose.X;
% RefPoseY = message.RefPose.Y;
% RefPoseTheta = message.RefPose.Theta;
% RefExtend_X = message.RefExtendX;
% RefExtend_Y = message.RefExtendY;
% 
% T1 = [cos(RefPoseTheta), -sin(RefPoseTheta), RefPoseX; sin(RefPoseTheta), cos(RefPoseTheta), RefPoseY; 0, 0, 1];
% RefPose1 = [RefPoseX;RefPoseY;1];
% b1 = [ 0; RefExtend_Y; 1];
% c1 = [ RefExtend_X;0;1];
% d1 = [ RefExtend_X;RefExtend_Y; 1];
% RefPose2 = T1*b1;
% RefPose3 = T1*c1;
% RefPose4 = T1*d1;
% buffer1.push_back([message.Header.Stamp.seconds, RefPose1(1),RefPose1(2)]); 
% buffer2.push_back([message.Header.Stamp.seconds, RefPose2(1),RefPose2(2)]); 
% buffer3.push_back([message.Header.Stamp.seconds, RefPose3(1),RefPose3(2)]);
% buffer4.push_back([message.Header.Stamp.seconds, RefPose4(1),RefPose4(2)]); 
% 
% ObstaclePoseX = message.ObstaclePose.X;
% ObstaclePoseY = message.ObstaclePose.Y;
% ObstacleTheta = message.ObstaclePose.Theta;
% ObstacleExtend_X = message.ObstacleExtendX;
% ObstacleExtend_Y = message.ObstacleExtendY;
% T2 = [cos(ObstacleTheta), -sin(ObstacleTheta), ObstaclePoseX; sin(ObstacleTheta), cos(ObstacleTheta), ObstaclePoseY; 0, 0, 1];
% ObstaclePose1 = [ObstaclePoseX,ObstaclePoseY];
% b2 = [ 0; ObstacleExtend_Y; 1];
% c2 = [ ObstacleExtend_X;0;1];
% d2 = [ ObstacleExtend_X;ObstacleExtend_Y; 1];
% ObstaclePose2 = T2*b2;
% ObstaclePose3 = T2*c2;
% ObstaclePose4 = T2*d2;
% buffer5.push_back([message.Header.Stamp.seconds, ObstaclePose1(1), ObstaclePose1(2)]);
% buffer6.push_back([message.Header.Stamp.seconds, ObstaclePose2(1), ObstaclePose2(2)]);
% buffer7.push_back([message.Header.Stamp.seconds, ObstaclePose3(1), ObstaclePose3(2)]);
% buffer8.push_back([message.Header.Stamp.seconds, ObstaclePose4(1), ObstaclePose4(2)]);
% buffer9.push_back([message.Header.Stamp.seconds, RefPoseTheta]);
% 
% end
