%原地转向时长rot：泊车过程中驾驶员原地打方向盘的总时长
function [score]=rot_Assessment(VehicleSpeed,angle)

rv=size(VehicleSpeed,1);
ra=size(angle,1); %弧度
num_rot=0;

if rv==ra
    r=rv;
else
    r=min(rv,ra); %取小值
    fprintf("速度与方向盘转角维度不一致\n");   % 即丢失数据，这样数据对不齐，下面的判断方式有问题
for i=2:r
    if VehicleSpeed(i,2)<0.05 && (angle(i,2) - angle(i-1,2))>0.1
        num_rot=num_rot+1;
    end
end

%原地转向分值
if num_rot<=(0.1*r) % 10%以内的时间原地转向
    score=10;
elseif (0.1*r)<num_rot && num_rot<=(0.2*r)
    score=6;
elseif (0.2*r)<num_rot && num_rot<=(0.3*r)
    score=2;
else
    score=0;
end

end