%������com�����������к�����ٶȴ���3.5m/s^2��������ٶȴ���3.5m/s^2����ʱ��(��ֵ�ɸ�������Լ�����)
function [score]=com_Assessment(LocalAx,LocalAy)
num_LocalAx=0;
num_LocalAy=0;

%������ٶ�ʱ������
[rx,cx]=size(LocalAx); %��ΪLocalAx�����еľ��󣬼�c=2����һ��Ϊ��ֵ�ڶ���Ϊ���ٶ�
for i=1:rx
    if LocalAx(i,2)>3.5
        num_LocalAx=num_LocalAx+1;
    end
end

%������ٶ�ʱ������
[ry,cy]=size(LocalAy); %��ΪLocalAx�����еľ��󣬼�c=2����һ��Ϊ��ֵ�ڶ���Ϊ���ٶ�
for i=1:ry
    if LocalAy(i,2)>3.5
        num_LocalAy=num_LocalAy+1;
    end
end

%�����ֵ
if num_LocalAx<=(0.05*rx) % 5%���ڵ�ʱ�䳬����ֵ
    score_LocalAx=2;
elseif ((0.05*rx)<num_LocalAx)&&(num_LocalAx<=(0.1*rx))
    score_LocalAx=1.2;
elseif ((0.1*rx)<num_LocalAx)&&(num_LocalAx<=(0.2*rx))
    score_LocalAx=0.4;
else
    score_LocalAx=0;
end

%�����ֵ
if num_LocalAy<=(0.05*ry) % 5%���ڵ�ʱ�䳬����ֵ
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
