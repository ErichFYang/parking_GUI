%原地转向时长rot：泊车过程中驾驶员原地打方向盘的总时长
function [score]=rot_Assessment(VehicleSpeed,angle)

[rv,cv]=size(VehicleSpeed);
[ra,ca]=size(angle); %角度还是弧度？
num_rot=0

if rv==ra
    r=rv;
else
    r=min(rv,ra); %取小值
    fprintf("速度与方向盘转角维度不一致");
for i=1:r
    if VehicleSpeed(i,2)<0.05 && angle(i,2)>0.1
        num_rot=num_rot+1;
    end
end

%原地转向分值
if num_rot<=(0.1*r) % 10%以内的时间原地转向
    score=2;
elseif (0.1*r)<num_rot && num_rot<=(0.2*r)
    score=1.2;
elseif (0.2*r)<num_rot && num_rot<=(0.3*r)
    score=0.4;
else
    score=0;
end

end