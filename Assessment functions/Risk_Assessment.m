%��ײ����Risk�����������г������ϰ�������С��10cm����ʱ��
% Vehicle??

%����������
%vehicle_width = 1.551; %����
%vehicle_length = 3.569; %����
%rear_overhang = 0.544; %����
%front_overhang = 0.72; %ǰ��
%wheel_base = 2.305;%���
%front_wheel_track = 1.324;%ǰ�־�
%rear_wheel_track = 1.292;%���־�
%front_vehicle_width = 0.791; %��Ӧ�˱���f��
%rear_vehicle_width = 0.821; %��Ӧ�˱���c��

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
%�жϳ����Ƿ������ǰ���ϰ�����ײ�ķ���
%���ϰ�������<0.1mʱ��Is_High_Risk = true
%���ȵ�λͳһΪm �Ƕȵ�λͳһΪrad
%Cord_ObsF1��ǰ�ϰ������ǵ�����[x;y]
%Cord_ObsF2��ǰ�ϰ����ڲ�ǵ�����[x;y]
%Cord_ObsR1�����ϰ������ǵ�����[x;y]
%Cord_ObsR2�����ϰ����ڲ�ǵ�����[x;y]
%Pos_Car������λ��[x;y;theta];
% *********************************************************************************************
Is_High_Risk = false;

Vec_Obs_F = RefPose1 - RefPose2; %ǰ�ϰ����ǵ���������
Vec_Obs_R = ObstaclePose2 - ObstaclePose1; %���ϰ����ǵ���������
len_Vec_Obs_F = norm(Vec_Obs_F);
len_Vec_Obs_R = norm(Vec_Obs_R);
%�����˱��ε�1 2 5 6������
Point1 = [Vehicle.Lf; Vehicle.Wf/2; 0];
Point2 = [Vehicle.Lf; -Vehicle.Wf/2; 0];
Point5 = [-Vehicle.Lr; -Vehicle.Wr/2; 0];
Point6 = [-Vehicle.Lr; Vehicle.Wr/2; 0];
dcm = angle2dcm(-Pos_Car(3),0,0);
Point1 = dcm*Point1 + Pos_Car;
Point2 = dcm*Point2 + Pos_Car; 
Point5 = dcm*Point5 + Pos_Car;
Point6 = dcm*Point6 + Pos_Car;
%�����˱��ε�1��2��ǰ�ϰ����ľ���
dis1toObsF = cross([Vec_Obs_F;0],[Point1(1:2)-RefPose2;0]);
dis2toObsF = cross([Vec_Obs_F;0],[Point2(1:2)-RefPose2;0]);
%�õ���1��2�о���ǰ�ϰ�����Σ�յ�ֵ
%������ò����㵽ǰ�ϰ����ľ��룬>0�ڿ�λ�ڣ�<0������ײ,ע�����������ѡȡ
distoF = min(dis1toObsF(3),dis2toObsF(3));
%�����˱��ε�5��6�����ϰ����ľ���
dis5toObsR = cross([Vec_Obs_R;0],[Point5(1:2)-ObstaclePose1;0]);
dis6toObsR = cross([Vec_Obs_R;0],[Point6(1:2)-ObstaclePose1;0]);
distoR = min(dis5toObsR(3),dis6toObsR(3));
%�������ϰ�������С����
distoobs = min(distoF/len_Vec_Obs_F,distoR/len_Vec_Obs_R);
if distoobs < 0.1
    Is_High_Risk = true;
end
end