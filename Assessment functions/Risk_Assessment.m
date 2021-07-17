%碰撞风险Risk：泊车过程中车辆与障碍车距离小于10cm的总时长
% Vehicle??

%整车参数：
%vehicle_width = 1.551; %车宽
%vehicle_length = 3.569; %车长
%rear_overhang = 0.544; %后悬
%front_overhang = 0.72; %前悬
%wheel_base = 2.305;%轴距
%front_wheel_track = 1.324;%前轮距
%rear_wheel_track = 1.292;%后轮距
%front_vehicle_width = 0.791; %对应八边形f边
%rear_vehicle_width = 0.821; %对应八边形c边

%Vehicle.Wf = front_vehicle_width;
%Vehicle.Wr = rear_vehicle_width;
%Vehicle.Lf = front_overhang + wheel_base;
%Vehicle.Lr = rear_overhang;

%Cord_ObsF1 = [0;0];
%Cord_ObsF2 = [0;-5];
%Cord_ObsR1 = [-5;0];
%Cord_ObsR2 = [-5;-5];
%Pos_Car = [-3;-2.5;pi/9];

%Is_High_Risk = RiskAssess(Cord_ObsF1,Cord_ObsF2,Cord_ObsR1,Cord_ObsR2,Pos_Car,Vehicle);

function Is_High_Risk = RiskAssess(RefPose1,RefPose2,ObstaclePose1,ObstaclePose2,Pos_Car,Vehicle)
% function Is_High_Risk = RiskAssess(Cord_ObsF1,Cord_ObsF2,Cord_ObsR1,Cord_ObsR2,Pos_Car,Vehicle)
%**********************************************************************************************
%判断车辆是否存在与前后障碍车碰撞的风险
%与障碍车距离<0.1m时，Is_High_Risk = true
%长度单位统一为m 角度单位统一为rad
%Cord_ObsF1：前障碍车外侧角点坐标[x;y]
%Cord_ObsF2：前障碍车内侧角点坐标[x;y]
%Cord_ObsR1：后障碍车外侧角点坐标[x;y]
%Cord_ObsR2：后障碍车内侧角点坐标[x;y]
%Pos_Car：车辆位姿[x;y;theta];
% *********************************************************************************************
Is_High_Risk = false;

Vec_Obs_F = RefPose1 - RefPose2; %前障碍车角点连线向量
Vec_Obs_R = ObstaclePose2 - ObstaclePose1; %后障碍车角点连线向量
len_Vec_Obs_F = norm(Vec_Obs_F);
len_Vec_Obs_R = norm(Vec_Obs_R);
%车辆八边形点1 2 5 6的坐标
Point1 = [Vehicle.Lf; Vehicle.Wf/2; 0];
Point2 = [Vehicle.Lf; -Vehicle.Wf/2; 0];
Point5 = [-Vehicle.Lr; -Vehicle.Wr/2; 0];
Point6 = [-Vehicle.Lr; Vehicle.Wr/2; 0];
dcm = angle2dcm(-Pos_Car(3),0,0);
Point1 = dcm*Point1 + Pos_Car;
Point2 = dcm*Point2 + Pos_Car; 
Point5 = dcm*Point5 + Pos_Car;
Point6 = dcm*Point6 + Pos_Car;
%求车辆八边形点1、2到前障碍车的距离
dis1toObsF = cross([Vec_Obs_F;0],[Point1(1:2)-RefPose2;0]);
dis2toObsF = cross([Vec_Obs_F;0],[Point2(1:2)-RefPose2;0]);
%得到点1、2中距离前障碍车最危险的值
%这里采用叉乘求点到前障碍车的距离，>0在库位内，<0发生碰撞,注意向量方向的选取
distoF = min(dis1toObsF(3),dis2toObsF(3));
%求车辆八边形点5、6到后障碍车的距离
dis5toObsR = cross([Vec_Obs_R;0],[Point5(1:2)-ObstaclePose1;0]);
dis6toObsR = cross([Vec_Obs_R;0],[Point6(1:2)-ObstaclePose1;0]);
distoR = min(dis5toObsR(3),dis6toObsR(3));
%求车辆到障碍车的最小距离
distoobs = min(distoF/len_Vec_Obs_F,distoR/len_Vec_Obs_R);
if distoobs < 0.1
    Is_High_Risk = true;
end
end