%停车姿态精度acc：泊车最终位姿与目标位姿的横、纵向偏差和航向角偏差的加权值
% 0-5 5-10 10-15 
% 0-1 1-2 2-3 3+
function [score]=acc_Assessment(xError,yError,HeadingAngleError)
%纵向偏差
if (0<=xError)&&(xError<=5)
    score_xError=2;
elseif (5<=xError)&&(xError<=10)
    score_xError=1.2;
elseif (10<=xError)&&(xError<=15)
    score_xError=0.4;
elseif xError<0
    fprintf("xError<0\n")
    score_xError=0;
else
    score_xError=0;
end

%横向偏差
if (0<=yError)&&(yError<=5)
    score_yError=2;
elseif (5<=yError)&&(yError<=10)
    score_yError=1.2;
elseif (10<=yError)&&(yError<=15)
    score_yError=0.4;
elseif yError<0
    fprintf("yError<0\n")
    score_yError=0;
else
    score_yError=0;
end

%航向角偏差
if (0<=HeadingAngleError)&&(HeadingAngleError<=1)
    score_HeadingAngleError=2;
elseif (1<=HeadingAngleError)&&(HeadingAngleError<=2)
    score_HeadingAngleError=1.2;
elseif (2<=HeadingAngleError)&&(HeadingAngleError<=3)
    score_HeadingAngleError=0.4;
elseif HeadingAngleError<0
    fprintf("HeadingAngelError<0\n")
    score_HeadingAngleError=0;
else
    score_HeadingAngleError=0;
end

score=score_xError*0.3+score_yError*0.3+score_HeadingAngleError*0.4;
end