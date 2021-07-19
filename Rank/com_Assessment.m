%舒适性com：泊车过程中横向加速度大于3.5m/s^2或纵向加速度大于3.5m/s^2的总时长(阈值可根据情况自己定义)
function [score]=com_Assessment(LocalAx,LocalAy)
num_LocalAx=0;
num_LocalAy=0;

%纵向加速度时长计数
[rx,cx]=size(LocalAx); %认为LocalAx是两列的矩阵，即c=2，第一列为数值第二列为加速度
for i=1:rx
    if LocalAx(i,2)>3.5
        num_LocalAx=num_LocalAx+1;
    end
end

%横向加速度时长计数
[ry,cy]=size(LocalAy); %认为LocalAx是两列的矩阵，即c=2，第一列为数值第二列为加速度
for i=1:ry
    if LocalAy(i,2)>3.5
        num_LocalAy=num_LocalAy+1;
    end
end

%纵向分值
if num_LocalAx<=(0.05*rx) % 5%以内的时间超过阈值
    score_LocalAx=2;
elseif ((0.05*rx)<num_LocalAx)&&(num_LocalAx<=(0.1*rx))
    score_LocalAx=1.2;
elseif ((0.1*rx)<num_LocalAx)&&(num_LocalAx<=(0.2*rx))
    score_LocalAx=0.4;
else
    score_LocalAx=0;
end

%横向分值
if num_LocalAy<=(0.05*ry) % 5%以内的时间超过阈值
    score_LocalAy=2;
elseif ((0.05*ry)<num_LocalAy)&&(num_LocalAy<=(0.1*ry))
    score_LocalAy=1.2;
elseif ((0.1*ry)<num_LocalAy)&&(num_LocalAy<=(0.2*ry))
    score_LocalAy=0.4;
else
    score_LocalAy=0;
end

score=score_LocalAx*0.5+score_LocalAy*0.5;
    
end
