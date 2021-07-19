%��λ��ʱ��T����̬����acc����ײ����crash��������com��ԭ��ת��ʱ��rot
%������λ��ʱ��T���ӿ�ʼ������������������ʱ��
%ͣ����̬����acc����������λ����Ŀ��λ�˵ĺ�0.3������ƫ��0.3�ͺ����ƫ��0.4�ļ�Ȩֵ
%��ײ����risk�����������г������ϰ�������С��10cm����ʱ��
%������com�����������к�����ٶȴ���3.5m/s^2��������ٶȴ���3.5m/s^2����ʱ��(��ֵ�ɸ�������Լ�����)
%ԭ��ת��ʱ��rot�����������м�ʻԱԭ�ش����̵���ʱ��
% function [score]=eva(Time,xError,yError,HeadingAngelError,RefPose1,RefPose2,ObstaclePose1,ObstaclePose2,Pos_Car,Vehicle,LocalAx,LocalAy,VehicleSpeed,angle)

Time_score=T_Assessment(Time);
acc_score=acc_Assessment(xError,yError,HeadingAngelError);
risk_score=RiskAssess(RefPose1,RefPose2,ObstaclePose1,ObstaclePose2,Pos_Car,Vehicle);
com_score=com_Assessment(LocalAx,LocalAy);
rot_score=rot_Assessment(VehicleSpeed,angle);

function [score]=eva0(Time_score,acc_score,risk_score,com_score,rot_score)
A0=[1, 3, 4, 7,	9;1/3, 1, 2, 5, 7;1/4, 1/2, 1, 4, 6;1/7, 1/5, 1/4, 1, 3;1/9, 1/7, 1/6, 1/3, 1];
[v,d]=eigs(A0);
lamdamax=max(d(:));
[m,n]=size(v); 
%������������׼��  
sum = 0;  
for i=1:m  
    sum = sum + v(i,1);  
end  
w0 = v(:,1);  
for i=1:m  
    w0(i,1)= v(i,1)/sum;  
end  

CI=(lamdamax-5)/4;

score=Time_score*w0(1,1)+acc_score*w0(2,1)+risk_score*w0(3,1)+com_score*w0(4,1)+rot_score*w0(5,1);

end



