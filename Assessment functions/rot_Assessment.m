%ԭ��ת��ʱ��rot�����������м�ʻԱԭ�ش����̵���ʱ��
function [score]=rot_Assessment(VehicleSpeed,angle)

[rv,cv]=size(VehicleSpeed);
[ra,ca]=size(angle); %�ǶȻ��ǻ��ȣ�
num_rot=0

if rv==ra
    r=rv;
else
    r=min(rv,ra); %ȡСֵ
    fprintf("�ٶ��뷽����ת��ά�Ȳ�һ��");
for i=1:r
    if VehicleSpeed(i,2)<0.05 && angle(i,2)>0.1
        num_rot=num_rot+1;
    end
end

%ԭ��ת���ֵ
if num_rot<=(0.1*r) % 10%���ڵ�ʱ��ԭ��ת��
    score=2;
elseif (0.1*r)<num_rot && num_rot<=(0.2*r)
    score=1.2;
elseif (0.2*r)<num_rot && num_rot<=(0.3*r)
    score=0.4;
else
    score=0;
end

end