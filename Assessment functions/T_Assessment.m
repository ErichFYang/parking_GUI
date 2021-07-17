%泊车入位总时长T：从开始泊车到泊车结束的总时间
function [score]=T_Assessment(Time)
if Time<=61.9
    score=2;
elseif Time>104.49
    score=0.4;
else
    score=1.2;
end

    