
function varargout = parking_GUI(varargin)
% PARKING_GUI MATLAB code for parking_GUI.fig
%      PARKING_GUI, by itself, creates a new PARKING_GUI or raises the existing
%      singleton*.
%
%      H = PARKING_GUI returns the handle to a new PARKING_GUI or the handle to
%      the existing singleton*.
%
%      PARKING_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PARKING_GUI.M with the given input arguments.
%
%      PARKING_GUI('Property','Value',...) creates a new PARKING_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before parking_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to parking_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help parking_GUI

% Last Modified by GUIDE v2.5 31-May-2021 02:04:45

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @parking_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @parking_GUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT
end

% --- Executes just before parking_GUI is made visible.
function parking_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to parking_GUI (see VARARGIN)


% 声明一些全局变量

% 汽车运动信息
global VehicleSpeed % 车速
global angle        % 方向盘转角
global LocalX       % 纵向位置
global LocalY       % 横向位置
global LocalVx      %纵向速度
global LocalVy      %横向速度
global LocalAx      %纵向加速度
global LocalAy      %横向加速度
global CollisonDistance   %碰撞距离
global yError             %横向偏差
global xError             %纵向偏差
global HeadingAngelError  %航向角偏差
global Time         %泊车时间
global t            %计时变量
global score        %泊车评分
%参考车位置
global RefPose1
global RefPose2
global RefPose3
global RefPose4
%障碍车位置
global ObstaclePose1
global ObstaclePose2
global ObstaclePose3
global ObstaclePose4
%车辆信息
global Yaw         %行驶偏航角
global stop        %停车结束控制
global stopflag    %泊车时间标志位

global tempAngle   %用于保存上一时刻方向盘转角

%变量初始化
VehicleSpeed = 0;
LocalX = 0;       
LocalY = 0;    
LocalVx = 0;
LocalVy = 0;
LocalAx = 0;
LocalAy = 0;
CollisonDistance = 0;  
yError = 0;      
xError = 0;       
HeadingAngelError = 0;
Time = 0;
t = 0;
score = 0;
RefPose1 = 0;
RefPose2 = 0;
RefPose3 = 0;
RefPose4 = 0;

ObstaclePose1 = 0;
ObstaclePose2 = 0;
ObstaclePose3 = 0;
ObstaclePose4 = 0;

Yaw = 0;
stop = 0;  
stopflag = 0;
tempAngle = 0;
rosshutdown
rosinit


% Choose default command line output for parking_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes parking_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);

end
% --- Outputs from this function are returned to the command line.
function varargout = parking_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


%%泊车过程中的读取信息与界面交互
    %创建订阅器与选择显示模块
    Ref1 = myvector(3);
    Ref2 = myvector(3);
    Ref3 = myvector(3);
    Ref4 = myvector(3);
    Obstacle1 = myvector(3);
    Obstacle2 = myvector(3);
    Obstacle3 = myvector(3);
    Obstacle4 = myvector(3);
    Refposetheta = myvector(2);
    parking_slot = rossubscriber('/parking_slot_info', 'apa_msgs/SlotInfoStamped',{@parkingslotCallback,Ref1,Ref2,Ref3,Ref4,Obstacle1,Obstacle2,Obstacle3,Obstacle4,Refposetheta});
    
    LocalA = myvector(3);
    imu = rossubscriber('/imu/data', 'sensor_msgs/Imu',{@imuCallback,LocalA});
    %LocalayDsp= findobj(0, 'tag', 'Localay');
    LocalaxDsp= findobj(0, 'tag', 'Localax');
    
    VehicleSpeed = myvector(2);
    velometer = rossubscriber('/velometer/base_link_local', 'geometry_msgs/TwistStamped',{@velometerCallback,VehicleSpeed});
    %VehicleSpeedDsp= findobj(0, 'tag', 'VehicleSpeedDsp');
    
    localx = myvector(2);
    localy = myvector(2);
    yaw = myvector(2);
    Vehicle_pose2D = rossubscriber('/odometer/local_map/base_link', 'nav_msgs/Odometry',{@Vehicle_pose2DCallback,localx,localy,yaw});
    
    angle = myvector(2);
    sub_steering_angle = rossubscriber('/steering_angle_deg', 'apa_msgs/SteeringAngleStamped',{@SteeringAngleCallback, angle});
    angleDsp= findobj(0, 'tag', 'angle');
    
    %泊车过程（while（1）循环内）
    while(1)
%         if(VehicleSpeed>0)
%             tic;
%             stopflag=0;
%         else
%              t=toc;
%              if(stopflag==0)
%                  Time= Time+t;
%              end
%              stopflag=1;
%         end            %%记录泊车时间
        if(angle.empty()||VehicleSpeed.empty())
            fprintf('no data\n')
            pause(0.1);
            continue;
        end
        latest_angle_record = angle.back();
        latest_speed_record = VehicleSpeed.back();
        latest_A_record = LocalA.back();
        RefPose1 = Ref1.back(); RefPose2 = Ref2.back(); 
        RefPose3 = Ref3.back(); RefPose4 = Ref4.back();
        ObstaclePose1 = Obstacle1.back(); ObstaclePose2 = Obstacle2.back(); 
        ObstaclePose3 = Obstacle3.back(); ObstaclePose4 = Obstacle4.back();
        RefPoseTheta = Refposetheta.back();
        LocalX = localx.back();
        LocalY = localy.back();
        Yaw = yaw.back();
        
        fprintf('current log length of steering_angle: %d\nlatest steering_angle: %f deg\n', angle.size(), latest_angle_record(2));
        fprintf('current log length of vehicle_speed: %d\nlatest vehicle_speed: %f deg\n', VehicleSpeed.size(), latest_speed_record(2));
        fprintf('current log length of Local_A: %d\nlatest Local_Ax: %f deg\nlatest Local_Ay: %f deg\n', VehicleSpeed.size(), latest_A_record(2),latest_A_record(3));
        %方向盘转角
        set(angleDsp,'string',num2str(latest_angle_record(2)));
        %车速
        set(handles.VehicleSpeed,'string',num2str(latest_speed_record(2)));
        %横向加速度      
        set(handles.Localay,'string',num2str(latest_A_record(3)));
        %纵向加速度        
        set(LocalaxDsp,'string',num2str(latest_A_record(2)));
        pause(0.01);
        
        TrajectoryDsp= findobj(0, 'tag', 'Trajectory');
        %绘制目标车位
        set(TrajectoryDsp,'UserData',line([RefPose1(2),RefPose2(2)],[RefPose1(3),RefPose2(3)])); %%X坐标，Y坐标
        set(TrajectoryDsp,'UserData',line([ObstaclePose1(2),ObstaclePose2(2)],[ObstaclePose1(3),ObstaclePose2(3)]));
        set(TrajectoryDsp,'UserData',line([ObstaclePose1(2),RefPose1(2)],[ObstaclePose1(3),RefPose1(3)]));
        set(TrajectoryDsp,'UserData',line([ObstaclePose2(2),RefPose2(2)],[ObstaclePose2(3),RefPose2(3)]));
    
        %绘制参考车位(前车)
        set(TrajectoryDsp,'UserData',line([RefPose1(2),RefPose3(2)],[RefPose1(3),RefPose3(3)]));
        set(TrajectoryDsp,'UserData',line([RefPose2(2),RefPose4(2)],[RefPose2(3),RefPose4(3)]));
    
        %绘制障碍车位(后车)
        set(TrajectoryDsp,'UserData',line([ObstaclePose1(2),ObstaclePose3(2)],[ObstaclePose1(3),ObstaclePose3(3)]));
        set(TrajectoryDsp,'UserData',line([ObstaclePose2(2),ObstaclePose4(2)],[ObstaclePose2(3),ObstaclePose4(3)]));
    
        %记录当前车辆位置
          %求从车身坐标系到全局坐标系的刚体变换矩阵
        T = [cos(Yaw(2)), -sin(Yaw(2)), LocalX(2); sin(RefPoseTheta(2)), cos(RefPoseTheta(2)), LocalY(2); 0, 0, 1]; 
          %求车辆八角点在车身坐标系下的位置
        V1L = [3.026;0.3955;1];V2L=[3.026;-0.3955;1];V3L=[2.646;-0.7755;1]; V4L=[-0.384;-0.7755;1];
        V5L = [-0.544;-0.4105;1]; V6L = [-0.544;0.4105;1]; V7L = [-0.384;0.7755;1]; V8L = [2.646;0.7755;1];
          %求车辆八角点在全局坐标系下的位置
        V1G = T*V1L; V2G = T*V2L; V3G = T*V3L; V4G = T*V4L;
        V5G = T*V5L; V6G = T*V6L; V7G = T*V7L; V8G = T*V8L;
          %绘制车辆模型，以长方形框表示实时位置
        set(TrajectoryDsp,'UserData',line([V1G(1),V2G(1)],[V1G(2),V2G(2)]));
        set(TrajectoryDsp,'UserData',line([V3G(1),V2G(1)],[V3G(2),V2G(2)]));
        set(TrajectoryDsp,'UserData',line([V3G(1),V4G(1)],[V3G(2),V4G(2)]));
        set(TrajectoryDsp,'UserData',line([V5G(1),V4G(1)],[V5G(2),V4G(2)]));
        set(TrajectoryDsp,'UserData',line([V5G(1),V6G(1)],[V5G(2),V6G(2)]));
        set(TrajectoryDsp,'UserData',line([V7G(1),V6G(1)],[V7G(2),V6G(2)]));
        set(TrajectoryDsp,'UserData',line([V7G(1),V8G(1)],[V7G(2),V8G(2)]));
        set(TrajectoryDsp,'UserData',line([V1G(1),V8G(1)],[V1G(2),V8G(2)]));
        
        global stop
        if(stop==1) %%手动终止泊车过程
            break;
        end
        
    end
    
    %泊车结束
     %碰撞距离
    %CollisonDistance=  ;           ????????
%     CollisonDistanceDsp= findobj(0, 'tag', 'CollisonDistance');   
%     set(CollisonDistanceDsp,'string',num2str(CollisonDistance));
    
%     %横向偏差
%     %yError = LocalY(1,2) - (RefPose1(3) + RefPose2(3) + ObstaclePose1(3) + ObstaclePose2(3))/4;
%     yErrorDsp= findobj(0, 'tag', 'yError');           
%     set(yErrorDsp,'string',num2str(yError));
    
%     %纵向偏差
%     %xError = LocalX(1,2) - (ObstaclePose1(1,1) + RefPose1(1,1) + ObstaclePose2(1,1) + RefPose2(1,1))/4;
%     xErrorDsp= findobj(0, 'tag', 'xError');           
%     set(xErrorDsp,'string',num2str(xError)); 
    
%     %航向角偏差
%     %HeadingAngelError = Yaw-RefPoseTheta;
%     HeadingAngelErrorDsp = findobj(0, 'tag', 'HeadingAngelError');   
%     set(HeadingAngelErrorDsp,'string',num2str(HeadingAngelError));

    %计算泊车评分
    
    
    %显示泊车时间与评分
%     TimeDsp= findobj(0, 'tag', 'Time');
%     set(TimeDsp,'string',num2str(Time));
%     scoreDsp= findobj(0, 'tag', 'score');
%     set(scoreDsp,'string',num2str(score));
    
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
end

% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over pushbutton1.

function pushbutton1_ButtonDownFcn(hObject, eventdata, handles)  
%%结束泊车
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 global stop
 stop=1;
 fprintf('stop')
end
