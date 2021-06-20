%%parkingslotCallback

function parkingslotCallback(~,message)
global RefPose1
global RefPose2
global RefPose3
global RefPose4
global RefPoseTheta
global ObstaclePose1
global ObstaclePose2
global ObstaclePose3
global ObstaclePose4
%parking_slot_msgStructs = readMessages(message,'DataFormat','struct');
%RefPoseX = cellfun(@(Ref) double(Ref.RefPose.X),parking_slot_msgStructs);
%RefPoseY = cellfun(@(Ref) double(Ref.RefPose.Y),parking_slot_msgStructs);
%RefPoseTheta = cellfun(@(Ref) double(Ref.RefPose.Theta),parking_slot_msgStructs);
%RefExtend_X = cellfun(@(Ref) double(Ref.RefExtendX),parking_slot_msgStructs);
%RefExtend_Y = cellfun(@(Ref) double(Ref.RefExtendY),parking_slot_msgStructs);
RefPoseX.push_back(message.RefPose.X)
RefPoseY.push_back(message.RefPose.Y)
RefPoseTheta.push_back(message.RefPose.Theta)
RefExtend_X.push_back(message.RefExtendX)
RefExtend_Y.push_back(message.RefExtendY)

T1 = [cos(RefPoseTheta), -sin(RefPoseTheta), RefPoseX; sin(RefPoseTheta), cos(RefPoseTheta), RefPoseY; 0, 0, 1];
RefPose1 = [RefPoseX;RefPoseY;1];
b1 = [ 0; -RefExtend_Y; 1];
c1 = [ RefExtend_X;0;1];
d1 = [ RefExtend_X;-RefExtend_Y; 1];
RefPose2 = T1*b1;
RefPose3 = T1*c1;
RefPose4 = T1*d1;

ObstaclePoseX = cellfun(@(Ref) double(Ref.ObstaclePose.X),parking_slot_msgStructs);
ObstaclePoseY = cellfun(@(Ref) double(Ref.ObstaclePose.Y),parking_slot_msgStructs);
ObstacleTheta = cellfun(@(Ref) double(Ref.ObstaclePose.Theta),parking_slot_msgStructs);
ObstacleExtend_Y = cellfun(@(Ref) double(Ref.ObstacleExtendY),parking_slot_msgStructs);
T2 = [cos(ObstacleTheta), -sin(ObstacleTheta), ObstaclePoseX; sin(ObstacleTheta), cos(ObstacleTheta), ObstaclePoseY; 0, 0, 1];
ObstaclePose1 = [ObstaclePoseX,ObstaclePoseY];
b2 = [ 0; -ObstacleExtend_Y; 1];
c2 = [ -ObstacleExtend_X;0;1];
d2 = [ -ObstacleExtend_X;-ObstacleExtend_Y; 1];
ObstaclePose2 = T2*b2;
ObstaclePose3 = T2*c2;
ObstaclePose4 = T2*d2;
end
