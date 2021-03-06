
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

% Last Modified by GUIDE v2.5 22-Jul-2021 21:52:06

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
% This function has no outplineut args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to parking_GUI (see VARARGIN)


% ????????????????????????

% ??????????????????
global VehicleSpeed % ??????
global angle        % ???????????????
global LocalX       % ????????????
global LocalY       % ????????????
global LocalVx      % ????????????
global LocalVy      % ????????????
global LocalAx      % ???????????????
global LocalAy      % ???????????????
global CollisonDistance   % ????????????
global yError             % ????????????
global xError             % ????????????
global HeadingAngleError  % ???????????????
global Time         % ????????????
global t            % ????????????
global risk_score   % ??????????????????
global score        % ????????????
%???????????????
global RefPose1
global RefPose2
global RefPose3
global RefPose4
%???????????????
global ObstaclePose1
global ObstaclePose2
global ObstaclePose3
global ObstaclePose4
%????????????
global Yaw         %???????????????
global stop        %??????????????????
global stopflag    %?????????????????????

global tempAngle   %???????????????????????????????????????

%???????????????
VehicleSpeed = 0;
angle = 0;
LocalX = 0;       
LocalY = 0;    
LocalVx = 0;
LocalVy = 0;
LocalAx = 0;
LocalAy = 0;
CollisonDistance = 0;  
yError = 0;      
xError = 0;       
HeadingAngleError = 0;
Time = 0;
t = 0;
risk_score = 0;
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


%%?????????????????????????????????????????????
    %????????????????????????????????????
    Ref1 = myvector(3);
    Ref2 = myvector(3);
    Ref3 = myvector(3);
    Ref4 = myvector(3);
    Obstacle1 = myvector(3);
    Obstacle2 = myvector(3);
    Obstacle3 = myvector(3);
    Obstacle4 = myvector(3);
    posetheta = myvector(2);
    
    parking_slot = rossubscriber('/parking_slot_info', 'apa_msgs/SlotInfoStamped',{@parkingslotCallback,Ref1,Ref2,Ref3,Ref4,Obstacle1,Obstacle2,Obstacle3,Obstacle4,posetheta});
    
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
    
    axes(handles.Trajectory);
    set(handles.Trajectory,'XLim',[0, 20],'YLim',[-5, 5]);
    set(handles.Trajectory,'UserData',[line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0]),...
        line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0]),...
        line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0]),...
        line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0])]);   
    
    if(VehicleSpeed.empty())
        fprintf('no data input yet\n')
        pause(0.1);
    end
    latest_speed_record = VehicleSpeed.back();
    StartTime = latest_speed_record(1);
    
    %???????????????while???1???????????????
    while(1)
        global stop
        if(stop==1) %%????????????????????????
            break;
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
%         end            %%??????????????????
        if(angle.empty()||VehicleSpeed.empty()||LocalA.empty())
            fprintf('no angle data\n')
            pause(0.1);
            continue;
        end
        latest_angle_record = angle.back();       
        if(Ref1.empty()||Ref2.empty()||Ref3.empty()||Ref4.empty()||...
            Obstacle1.empty()||Obstacle2.empty()||Obstacle3.empty()||Obstacle4.empty()||posetheta.empty())
            fprintf('no parking slot data\n')
            pause(0.1);
            continue;
        end   
        RefPose1 = Ref1.back(); RefPose2 = Ref2.back(); 
        RefPose3 = Ref3.back(); RefPose4 = Ref4.back();
        ObstaclePose1 = Obstacle1.back(); ObstaclePose2 = Obstacle2.back(); 
        ObstaclePose3 = Obstacle3.back(); ObstaclePose4 = Obstacle4.back();
        PoseTheta = posetheta.back(); 
        if(VehicleSpeed.empty()||LocalA.empty()||localx.empty||localy.empty()||yaw.empty())
            fprintf('no vehicle data\n')
            pause(0.1);
            continue;
        end            
        latest_speed_record = VehicleSpeed.back();
        latest_A_record = LocalA.back();
        LocalX = localx.back();
        LocalY = localy.back();
        Yaw = yaw.back();
        
        %????????????
        vehicle_width = 1.551; %??????
        vehicle_length = 3.569; %??????
        rear_overhang = 0.544; %??????
        front_overhang = 0.72; %??????
        wheel_base = 2.305;%??????
        front_wheel_track = 1.324;%?????????
        rear_wheel_track = 1.292;%?????????
        front_vehicle_width = 0.791; %???????????????f???
        rear_vehicle_width = 0.821; %???????????????c???

        Vehicle.Wf = front_vehicle_width;
        Vehicle.Wr = rear_vehicle_width;
        Vehicle.Lf = front_overhang + wheel_base;
        Vehicle.Lr = rear_overhang;
        Pos_Car=[LocalX(2);LocalY(2);Yaw(2)];
        global risk_score
        rfp1=[RefPose1(2);RefPose1(3)]; rfp2=[RefPose2(2);RefPose2(3)];
        obp1=[ObstaclePose1(2);ObstaclePose1(3)]; obp2=[ObstaclePose2(2);ObstaclePose2(3)]; 
        risk_score = risk_score+ Risk_Assessment(rfp1,rfp2,obp1,obp2,Pos_Car,Vehicle);
        
        %????????????????????????
          %????????????????????????????????????????????????????????????
        T = [cos(Yaw(2)), -sin(Yaw(2)), LocalX(2); sin(Yaw(2)), cos(Yaw(2)), LocalY(2); 0, 0, 1]; 
          %????????????????????????????????????????????????????????????
        PoseTheta1=deg2rad(PoseTheta(2)); %???????????????
        T0 = [cos(PoseTheta1),-sin(PoseTheta1),-ObstaclePose1(2);sin(PoseTheta1),cos(PoseTheta1),-ObstaclePose1(3);0,0,1];
          %????????????????????????????????????????????????
        V1L = [3.026;0.3955;1];V2L=[3.026;-0.3955;1];V3L=[2.646;-0.7755;1]; V4L=[-0.384;-0.7755;1];
        V5L = [-0.544;-0.4105;1]; V6L = [-0.544;0.4105;1]; V7L = [-0.384;0.7755;1]; V8L = [2.646;0.7755;1];
          %????????????????????????????????????????????????
        V1G = T*V1L; V2G = T*V2L; V3G = T*V3L; V4G = T*V4L;
        V5G = T*V5L; V6G = T*V6L; V7G = T*V7L; V8G = T*V8L;
          %???????????????????????????????????????????????????
        V0 = [LocalX(2);LocalY(2);1];
        V = T0*V0; %V???????????????????????????????????????????????????????????????????????????????????????
        R1 = T0*[RefPose1(2);RefPose1(3);1];  %R1??????????????????????????????????????????
        O1=T0*[ObstaclePose1(2);ObstaclePose1(3);1]; 
        
        %fprintf('current log length of steering_angle: %d\nlatest steering_angle: %f deg\n', angle.size(), latest_angle_record(2));
        %fprintf('current log length of vehicle_speed: %d\nlatest vehicle_speed: %f deg\n', VehicleSpeed.size(), latest_speed_record(2));
        %fprintf('current log length of Local_A: %d\nlatest Local_Ax: %f deg\nlatest Local_Ay: %f deg\n', VehicleSpeed.size(), latest_A_record(2),latest_A_record(3));
        
        % Display
        %???????????????
        set(angleDsp,'string',num2str(latest_angle_record(2)));
        %??????
        set(handles.VehicleSpeed,'string',num2str(latest_speed_record(2)));
        %???????????????      
        set(handles.Localay,'string',num2str(latest_A_record(3)));
        %???????????????        
        set(LocalaxDsp,'string',num2str(latest_A_record(2)));
        
        %????????????
        global yError
        h = abs(V(2));
        yError = h-0.8;
       % yError = LocalY(2) - (RefPose1(2) + RefPose2(2) + ObstaclePose1(2) + ObstaclePose2(2))/4;
        yErrorDsp= findobj(0, 'tag', 'yError');           
        set(yErrorDsp,'string',num2str(yError));    
        %????????????
        global xError
        x_= (R1(1)-O1(1)-3.57)/2+0.544; %??????????????????
        xError = V(1) - x_;
        %xError = LocalX(2) - (ObstaclePose1(2) + RefPose1(2) + ObstaclePose2() + RefPose2(2))/4;
        xErrorDsp= findobj(0, 'tag', 'xError');           
        set(xErrorDsp,'string',num2str(xError));    
        %???????????????
        global HeadingAngleError
        HeadingAngleError = rad2deg(Yaw(2))-PoseTheta(2);
        %HeadingAngleError = Yaw-PoseTheta;       
        %HeadingAngleError = findobj(0, 'tag', 'HeadingAngleError');   
        set(handles.HeadingAngleError,'string',num2str(HeadingAngleError));        
        pause(0.01);
                
        %draw parking slot 
        TrajectoryDsp= findobj(0, 'tag', 'Trajectory');
        axes(handles.Trajectory);

        %??????????????????
        set(handles.Trajectory.UserData(1),'XData',[RefPose1(2),RefPose2(2)],'YData',[RefPose1(3),RefPose2(3)],'Color','red');
        set(handles.Trajectory.UserData(2),'XData',[ObstaclePose1(2),ObstaclePose2(2)],'YData',[ObstaclePose1(3),ObstaclePose2(3)],'Color','red');
        set(handles.Trajectory.UserData(3),'XData',[ObstaclePose1(2),RefPose1(2)],'YData',[ObstaclePose1(3),RefPose1(3)],'Color','red');
        set(handles.Trajectory.UserData(4),'XData',[ObstaclePose2(2),RefPose2(2)],'YData',[ObstaclePose2(3),RefPose2(3)],'Color','red');
        
        %set(TrajectoryDsp,'UserData',line([RefPose1(2),RefPose2(2)],[RefPose1(3),RefPose2(3)])); %%X?????????Y??????
        %set(TrajectoryDsp,'UserData',line([ObstaclePose1(2),ObstaclePose2(2)],[ObstaclePose1(3),ObstaclePose2(3)]));
        %set(TrajectoryDsp,'UserData',line([ObstaclePose1(2),RefPose1(2)],[ObstaclePose1(3),RefPose1(3)]));
        %set(TrajectoryDsp,'UserData',line([ObstaclePose2(2),RefPose2(2)],[ObstaclePose2(3),RefPose2(3)]));
    
        %??????????????????(??????)
        set(handles.Trajectory.UserData(5),'XData',[RefPose1(2),RefPose3(2)],'YData',[RefPose1(3),RefPose3(3)],'Color','black');
        set(handles.Trajectory.UserData(6),'XData',[RefPose2(2),RefPose4(2)],'YData',[RefPose2(3),RefPose4(3)],'Color','black');
        
        %set(TrajectoryDsp,'UserData',line([RefPose1(2),RefPose3(2)],[RefPose1(3),RefPose3(3)]));
        %set(TrajectoryDsp,'UserData',line([RefPose2(2),RefPose4(2)],[RefPose2(3),RefPose4(3)]));
    
        %??????????????????(??????)
        set(handles.Trajectory.UserData(7),'XData',[ObstaclePose1(2),ObstaclePose3(2)],'YData',[ObstaclePose1(3),ObstaclePose3(3)],'Color','black');
        set(handles.Trajectory.UserData(8),'XData',[ObstaclePose2(2),ObstaclePose4(2)],'YData',[ObstaclePose2(3),ObstaclePose4(3)],'Color','black');
                
        %set(TrajectoryDsp,'UserData',line([ObstaclePose1(2),ObstaclePose3(2)],[ObstaclePose1(3),ObstaclePose3(3)]));
        %set(TrajectoryDsp,'UserData',line([ObstaclePose2(2),ObstaclePose4(2)],[ObstaclePose2(3),ObstaclePose4(3)]));
    
        
        %??????????????????????????????????????????????????????
        set(handles.Trajectory.UserData(9),'XData',[V1G(1),V2G(1)],'YData',[V1G(2),V2G(2)]);
        set(handles.Trajectory.UserData(10),'XData',[V3G(1),V2G(1)],'YData',[V3G(2),V2G(2)]);
        set(handles.Trajectory.UserData(11),'XData',[V3G(1),V4G(1)],'YData',[V3G(2),V4G(2)]);
        set(handles.Trajectory.UserData(12),'XData',[V5G(1),V4G(1)],'YData',[V5G(2),V4G(2)]);
        set(handles.Trajectory.UserData(13),'XData',[V5G(1),V6G(1)],'YData',[V5G(2),V6G(2)]);
        set(handles.Trajectory.UserData(14),'XData',[V7G(1),V6G(1)],'YData',[V7G(2),V6G(2)]);
        set(handles.Trajectory.UserData(15),'XData',[V7G(1),V8G(1)],'YData',[V7G(2),V8G(2)]);
        set(handles.Trajectory.UserData(16),'XData',[V1G(1),V8G(1)],'YData',[V1G(2),V8G(2)]);
        
        %set(TrajectoryDsp,'UserData',line([V1G(1),V2G(1)],[V1G(2),V2G(2)]));
        %set(TrajectoryDsp,'UserData',line([V3G(1),V2G(1)],[V3G(2),V2G(2)]));
        %set(TrajectoryDsp,'UserData',line([V3G(1),V4G(1)],[V3G(2),V4G(2)]));
        %set(TrajectoryDsp,'UserData',line([V5G(1),V4G(1)],[V5G(2),V4G(2)]));
        %set(TrajectoryDsp,'UserData',line([V5G(1),V6G(1)],[V5G(2),V6G(2)]));
        %set(TrajectoryDsp,'UserData',line([V7G(1),V6G(1)],[V7G(2),V6G(2)]));
        %set(TrajectoryDsp,'UserData',line([V7G(1),V8G(1)],[V7G(2),V8G(2)]));
        %set(TrajectoryDsp,'UserData',line([V1G(1),V8G(1)],[V1G(2),V8G(2)]));
        
        global Time
        Time = latest_speed_record(1);
        
        drawnow
        pause(0.1)
    
    end
    
    %????????????
    %??????????????????
    Time=Time-StartTime;
    Time_score = T_Assessment(Time);
    fprintf('Time_score =%d\n', Time_score);
    %??????????????????
    angleError=deg2rad(HeadingAngleError);
    acc_score = acc_Assessment(xError,yError,angleError);
    fprintf('acc_score =%d\n', acc_score);
    %???????????????
    Acc = get_data(LocalA);
    LocalAx = Acc(:,2);   %???????????????
    LocalAy = Acc(:,3);   %???????????????
    com_score = com_Assessment(LocalAx,LocalAy);
    fprintf('com_score =%d\n', com_score);
    %????????????????????????
    VehicleSpeed_data=get_data(VehicleSpeed);
    angle_data=get_data(angle);    
    rot_score = rot_Assessment(VehicleSpeed_data,angle_data);
    
    %??????????????????    
    global score
    score = eva(Time_score,acc_score,risk_score,com_score,rot_score);
    %???????????????????????????
    TimeDsp= findobj(0, 'tag', 'Time');
    set(TimeDsp,'string',num2str(Time));
    scoreDsp= findobj(0, 'tag', 'score');
    set(scoreDsp,'string',num2str(score));
    

     %????????????
    %CollisonDistance=  ;           ????????
%     CollisonDistanceDsp= findobj(0, 'tag', 'CollisonDistance');   
%     set(CollisonDistanceDsp,'string',num2str(CollisonDistance));
    
end


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
end
