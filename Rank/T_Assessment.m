%泊车入位总时长T：从开始泊车到泊车结束的总时间
function [score]=T_Assessment(Time)
if Time<=50
    score=10;
elseif Time>70
    score=2;
else
    score=6;
end

    