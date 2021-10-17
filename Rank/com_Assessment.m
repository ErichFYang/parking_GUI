%舒适性com：泊车过程中横向加速度大于3.5m/s^2或纵向加速度大于3.5m/s^2的总时长(阈值可根据情况自己定义)
function [score]=com_Assessment(LocalAx,LocalAy)
axThreshold = 3.5;
ayThreshold = 3.5;
numAll = size(LocalAx, 1);

%纵向加速度时长计数
numLocalAx = sum(LocalAx > axThreshold);

%横向加速度时长计数
numLocalAy = sum(LocalAy > ayThreshold);

%纵向分值
if numLocalAx<=(0.05*numAll) % 5%以内的时间超过阈值
    score_LocalAx=10;
elseif ((0.05*numAll)<numLocalAx)&&(numLocalAx<=(0.1*numAll))
    score_LocalAx=6;
elseif ((0.1*numAll)<numLocalAx)&&(numLocalAx<=(0.2*numAll))
    score_LocalAx=2;
else
    score_LocalAx=0;
end

%横向分值
if numLocalAy<=(0.05*numAll) % 5%以内的时间超过阈值
    score_LocalAy=10;
elseif ((0.05*numAll)<numLocalAy)&&(numLocalAy<=(0.1*numAll))
    score_LocalAy=6;
elseif ((0.1*numAll)<numLocalAy)&&(numLocalAy<=(0.2*numAll))
    score_LocalAy=2;
else
    score_LocalAy=0;
end

score=score_LocalAx*0.5+score_LocalAy*0.5;
    
end
