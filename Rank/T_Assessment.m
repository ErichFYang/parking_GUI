%������λ��ʱ��T���ӿ�ʼ������������������ʱ��
function [score]=T_Assessment(Time)
if Time<=61.9
    score=2;
elseif Time>104.49
    score=0.4;
else
    score=1.2;
end

    