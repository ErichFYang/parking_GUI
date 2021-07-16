
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

% Last Modified by GUIDE v2.5 15-Jul-2021 19:11:15

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


% --- Executes just before parking_GUI is made visible.
function parking_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to parking_GUI (see VARARGIN)

global stop;
stop = 0;

rosshutdown;
rosinit;
pause(1);

% Choose default command line output for parking_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes parking_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = parking_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

global stop;

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

    axes(handles.Trajectory);
    set(handles.Trajectory,'XLim',[0, 10],'YLim',[-4, 4]);
    set(handles.Trajectory,'UserData',[line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0]),...
        line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0]),...
        line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0]),...
        line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0])]);
    
    %泊车过程（while（1）循环内）
    while(1)
        
        if(stop==1) %%手动终止泊车过程
            break
        end
        
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
        %记录当前车辆位置
        %求从车身坐标系到全局坐标系的刚体变换矩阵
        T = [cos(Yaw(2)), -sin(Yaw(2)), LocalX(2); sin(Yaw(2)), cos(Yaw(2)), LocalY(2); 0, 0, 1];
        %求车辆八角点在车身坐标系下的位置
        V1L = [3.026;0.3955;1];V2L=[3.026;-0.3955;1];V3L=[2.646;-0.7755;1]; V4L=[-0.384;-0.7755;1];
        V5L = [-0.544;-0.4105;1]; V6L = [-0.544;0.4105;1]; V7L = [-0.384;0.7755;1]; V8L = [2.646;0.7755;1];
        %求车辆八角点在全局坐标系下的位置
        V1G = T*V1L; V2G = T*V2L; V3G = T*V3L; V4G = T*V4L;
        V5G = T*V5L; V6G = T*V6L; V7G = T*V7L; V8G = T*V8L;
        
%% Display modules

%         fprintf('current log length of steering_angle: %d\nlatest steering_angle: %f deg\n', angle.size(), latest_angle_record(2));
%         fprintf('current log length of vehicle_speed: %d\nlatest vehicle_speed: %f deg\n', VehicleSpeed.size(), latest_speed_record(2));
%         fprintf('current log length of Local_A: %d\nlatest Local_Ax: %f deg\nlatest Local_Ay: %f deg\n', VehicleSpeed.size(), latest_A_record(2),latest_A_record(3));
        %方向盘转角
        set(handles.angle,'string',num2str(latest_angle_record(2)));
        %车速
        set(handles.VehicleSpeed,'string',num2str(latest_speed_record(2)));
        %横向加速度      
        set(handles.Localay,'string',num2str(latest_A_record(3)));
        %纵向加速度        
        set(handles.Localax,'string',num2str(latest_A_record(2)));
        
%         cla(handles.Trajectory);
        axes(handles.Trajectory);

        
        
        %绘制目标车位
        set(handles.Trajectory.UserData(1),'XData',[RefPose1(2),RefPose2(2)],'YData',[RefPose1(3),RefPose2(3)],'Color','red');
        set(handles.Trajectory.UserData(2),'XData',[ObstaclePose1(2),ObstaclePose2(2)],'YData',[ObstaclePose1(3),ObstaclePose2(3)],'Color','red');
        set(handles.Trajectory.UserData(3),'XData',[ObstaclePose1(2),RefPose1(2)],'YData',[ObstaclePose1(3),RefPose1(3)],'Color','red');
        set(handles.Trajectory.UserData(4),'XData',[ObstaclePose2(2),RefPose2(2)],'YData',[ObstaclePose2(3),RefPose2(3)],'Color','red');
        
%         set(handles.Trajectory,'UserData',line([RefPose1(2),RefPose2(2)],[RefPose1(3),RefPose2(3)],'Color','red')); %%X坐标，Y坐标
%         set(handles.Trajectory,'UserData',line([ObstaclePose1(2),ObstaclePose2(2)],[ObstaclePose1(3),ObstaclePose2(3)],'Color','red'));
%         set(handles.Trajectory,'UserData',line([ObstaclePose1(2),RefPose1(2)],[ObstaclePose1(3),RefPose1(3)],'Color','red'));
%         set(handles.Trajectory,'UserData',line([ObstaclePose2(2),RefPose2(2)],[ObstaclePose2(3),RefPose2(3)],'Color','red'));

        %绘制参考车位(前车)
        set(handles.Trajectory.UserData(5),'XData',[RefPose1(2),RefPose3(2)],'YData',[RefPose1(3),RefPose3(3)],'Color','black');
        set(handles.Trajectory.UserData(6),'XData',[RefPose2(2),RefPose4(2)],'YData',[RefPose2(3),RefPose4(3)],'Color','black');
        
%         set(handles.Trajectory,'UserData',line([RefPose1(2),RefPose3(2)],[RefPose1(3),RefPose3(3)],'Color','black'));
%         set(handles.Trajectory,'UserData',line([RefPose2(2),RefPose4(2)],[RefPose2(3),RefPose4(3)],'Color','black'));
        
        %绘制障碍车位(后车)
        set(handles.Trajectory.UserData(7),'XData',[ObstaclePose1(2),ObstaclePose3(2)],'YData',[ObstaclePose1(3),ObstaclePose3(3)],'Color','black');
        set(handles.Trajectory.UserData(8),'XData',[ObstaclePose2(2),ObstaclePose4(2)],'YData',[ObstaclePose2(3),ObstaclePose4(3)],'Color','black');
        
%         set(handles.Trajectory,'UserData',line([ObstaclePose1(2),ObstaclePose3(2)],[ObstaclePose1(3),ObstaclePose3(3)],'Color','black'));
%         set(handles.Trajectory,'UserData',line([ObstaclePose2(2),ObstaclePose4(2)],[ObstaclePose2(3),ObstaclePose4(3)],'Color','black'));
        
        %绘制车辆模型，以长方形框表示实时位置
        set(handles.Trajectory.UserData(9),'XData',[V1G(1),V2G(1)],'YData',[V1G(2),V2G(2)]);
        set(handles.Trajectory.UserData(10),'XData',[V3G(1),V2G(1)],'YData',[V3G(2),V2G(2)]);
        set(handles.Trajectory.UserData(11),'XData',[V3G(1),V4G(1)],'YData',[V3G(2),V4G(2)]);
        set(handles.Trajectory.UserData(12),'XData',[V5G(1),V4G(1)],'YData',[V5G(2),V4G(2)]);
        set(handles.Trajectory.UserData(13),'XData',[V5G(1),V6G(1)],'YData',[V5G(2),V6G(2)]);
        set(handles.Trajectory.UserData(14),'XData',[V7G(1),V6G(1)],'YData',[V7G(2),V6G(2)]);
        set(handles.Trajectory.UserData(15),'XData',[V7G(1),V8G(1)],'YData',[V7G(2),V8G(2)]);
        set(handles.Trajectory.UserData(16),'XData',[V1G(1),V8G(1)],'YData',[V1G(2),V8G(2)]);
        
        
%         set(handles.Trajectory,'UserData',line([V1G(1),V2G(1)],[V1G(2),V2G(2)]));
%         set(handles.Trajectory,'UserData',line([V3G(1),V2G(1)],[V3G(2),V2G(2)]));
%         set(handles.Trajectory,'UserData',line([V3G(1),V4G(1)],[V3G(2),V4G(2)]));
%         set(handles.Trajectory,'UserData',line([V5G(1),V4G(1)],[V5G(2),V4G(2)]));
%         set(handles.Trajectory,'UserData',line([V5G(1),V6G(1)],[V5G(2),V6G(2)]));
%         set(handles.Trajectory,'UserData',line([V7G(1),V6G(1)],[V7G(2),V6G(2)]));
%         set(handles.Trajectory,'UserData',line([V7G(1),V8G(1)],[V7G(2),V8G(2)]));
%         set(handles.Trajectory,'UserData',line([V1G(1),V8G(1)],[V1G(2),V8G(2)]));

        drawnow

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
    



% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global stop

if get(handles.pushbutton1, 'value') 
    stop = 1;
    fprintf('stop');
end

% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over pushbutton1.
