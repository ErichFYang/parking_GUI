
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

% Last Modified by GUIDE v2.5 09-Sep-2021 17:36:44

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
addpath('Callbacks');
addpath('Rank');
global PubArrayText % show interaction message
global flag_show    % related to trajectory showing
global h_tr         % handles of trajectory
flag_show = 0;
h_tr = 0;

rosshutdown;
rosinit;

PubArrayText = sprintf('%s\n%s\n','点击“开始泊车”进行泊车数据显示','点击“退出”关闭界面');
set(handles.Notice,'String',PubArrayText);

% Choose default command line output for parking_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

set(handles.ui_paraset, 'Visible', 1);
set(handles.uipanel1, 'Visible', 0);

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
global stop;   % stop recording
global exit;  % close GUI
global tr_xlim;  % xlim of handles.Trajectory
global tr_ylim;  % ylim of handles.Trajectory
global flag_loop2;  % to show if the program run in loop2
global p_width; % width of target position
global vehiclePose;
global last_Ref;
global xError;
global yError;
global HeadingAngleError;
global risk_score;

% initialize score graph
set(handles.sc1, 'XTick', {}, 'YTick', {});

risk_score = 0;
stop = 1;   
exit = 0;  
flag_loop2 = 0;
last_Ref = zeros(12,2);

%整车参数
      vehicle_width = 1.551; %车宽
      vehicle_length = 3.569; %车长
      rear_overhang = 0.544; %后悬
      front_overhang = 0.72; %前悬
      wheel_base = 2.305;%轴距
      front_wheel_track = 1.324;%前轮距
      rear_wheel_track = 1.292;%后轮距
      front_vehicle_width = 0.791; %对应八边形f边
      rear_vehicle_width = 0.821; %对应八边形c边
      
      Vehicle.Wf = front_vehicle_width;
      Vehicle.Wr = rear_vehicle_width;
      Vehicle.Lf = front_overhang + wheel_base;
      Vehicle.Lr = rear_overhang;

% main
while(~exit)
    pause(0.001);
    if ~(stop||exit)
    %%泊车过程中的读取信息与界面交互
        %创建订阅器与选择显示模块
        angle = myvector(2);
        angle_indice = 0;
        sub_steering_angle = rossubscriber('/steering_angle_deg', 'apa_msgs/SteeringAngleStamped',{@SteeringAngleCallback, angle});

        VehicleSpeed = myvector(2);
        VehicleSpeed_indice = 0;
        velometer = rossubscriber('/velometer/base_link_local', 'geometry_msgs/TwistStamped',{@velometerCallback,VehicleSpeed});

        LocalA = myvector(3);
        LocalA_indice = 0;
        imu = rossubscriber('/imu/data', 'sensor_msgs/Imu',{@imuCallback,LocalA});

%       {time,Ref1,Ref2,Ref3,Ref4,Obstacle1,Obstacle2,Obstacle3,Obstacle4,refposetheta}
        parkingSlot = myvector(18);
        parkingSlot_indice = 0;
        parking_slot = rossubscriber('/parking_slot_info', 'apa_msgs/SlotInfoStamped',{@parkingslotCallback,parkingSlot});
        
%       {time,localx,localy,yaw}
        vehiclePose = myvector(4);
        vehiclePose_indice = 0;
        Vehicle_pose2D = rossubscriber('/odometer/local_map/base_link', 'nav_msgs/Odometry',{@Vehicle_pose2DCallback,vehiclePose});

        setlog(handles, '话题订阅成功！');

        %Display modules initialization
        clear_graph(handles);
        
        set(handles.Time, 'String', '');
        set(handles.score, 'String', '');
    end
          
    %泊车过程
    while ~(stop||exit)
        if ~flag_loop2
            time_start = now;
            flag_loop2 = 1;
        end
        % Get and display message
        [latest_angle_record, angle_indice] = getmsg(angle, angle_indice, handles);
        [latest_speed_record, VehicleSpeed_indice] = getmsg(VehicleSpeed, VehicleSpeed_indice, handles);
        [latest_A_record, LocalA_indice] = getmsg(LocalA, LocalA_indice, handles);  
        [latest_parkingSlot_record, parkingSlot_indice] = getmsg(parkingSlot, parkingSlot_indice, handles);
        RefPose1 = latest_parkingSlot_record(2:3); RefPose2 = latest_parkingSlot_record(4:5);
        RefPose3 = latest_parkingSlot_record(6:7); RefPose4 = latest_parkingSlot_record(8:9);
        ObstaclePose1 = latest_parkingSlot_record(10:11); ObstaclePose2 = latest_parkingSlot_record(12:13);
        ObstaclePose3 = latest_parkingSlot_record(14:15); ObstaclePose4 = latest_parkingSlot_record(16:17);
        RefPoseTheta = latest_parkingSlot_record(18);
        [latest_vehiclePose_record, vehiclePose_indice] = getmsg(vehiclePose, vehiclePose_indice, handles);
        LocalX = latest_vehiclePose_record(2); LocalY = latest_vehiclePose_record(3);
        Yaw = latest_vehiclePose_record(4);
        
        %方向盘转角
        set(handles.angle,'string',num2str(latest_angle_record(2),'%.2f'));
        %车速
        set(handles.VehicleSpeed,'string',num2str(latest_speed_record(2),'%.2f'));
        %横向加速度
        set(handles.Localay,'string',num2str(latest_A_record(3),'%.2f'));
        %纵向加速度
        set(handles.Localax,'string',num2str(latest_A_record(2),'%.2f'));

        if ~isnan(RefPose1)
            hold off        
            % Target parking position
            VecObs = [ObstaclePose2(1) - ObstaclePose1(1), ObstaclePose2(2) - ObstaclePose1(2)];
            Theta_Obs = atan2(VecObs(2), VecObs(1));
            T_Obs = [cos(Theta_Obs), -sin(Theta_Obs), ObstaclePose1(1); ...
                sin(Theta_Obs), cos(Theta_Obs), ObstaclePose1(2); 0, 0, 1];
            Obs_fr = T_Obs * [p_width; 0; 1];
            Obs_rr = T_Obs * [p_width; -p_width; 1];
            Obs_fl = ObstaclePose1;
            Obs_rl = ObstaclePose3;
            VecRef = [RefPose2(1) - RefPose1(1), RefPose2(2) - RefPose1(2)];
            Theta_Ref = atan2(VecRef(2), VecRef(1));
            T_Ref = [cos(Theta_Ref), -sin(Theta_Ref), RefPose1(1); ...
                sin(Theta_Ref), cos(Theta_Ref), RefPose1(2); 0, 0, 1];
            Ref_rr = T_Ref * [p_width; 0; 1];
            Ref_fr = T_Ref * [p_width; p_width; 1];
            Ref_rl = RefPose1;
            Ref_fl = RefPose3;
            [p_fl, p_fr, p_rr, p_rl, p_length] = calpp(Ref_rl, Ref_rr, Obs_fl, Obs_fr, p_width);
            set(handles.P_length,'string',num2str(p_length,'%.2f'));
            PoseTheta = atan2(p_fl(2) - p_rl(2), p_fl(1) - p_rl(1));
            
            
            %绘制算法目标车位
            axes(handles.Trajectory);           
            color = 'red';
            linewidth = 0.5;
            linestyle = '-';
                        
            plot([p_fl(1), p_fr(1)],[p_fl(2), p_fr(2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
            hold on
            plot([p_fr(1), p_rr(1)],[p_fr(2), p_rr(2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
            plot([p_rr(1), p_rl(1)],[p_rr(2), p_rl(2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
            plot([p_rl(1), p_fl(1)],[p_rl(2), p_fl(2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
            
%             set(handles.Trajectory.UserData(1),'XData',[p_fl(1), p_fr(1)],'YData',[p_fl(2), p_fr(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
%             set(handles.Trajectory.UserData(2),'XData',[p_fr(1), p_rr(1)],'YData',[p_fr(2), p_rr(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
%             set(handles.Trajectory.UserData(3),'XData',[p_rr(1), p_rl(1)],'YData',[p_rr(2), p_rl(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
%             set(handles.Trajectory.UserData(4),'XData',[p_rl(1), p_fl(1)],'YData',[p_rl(2), p_fl(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
        %绘制实际泊车空间
            color = 'green';
            linewidth = 0.5;
            linestyle = '-';
                       
            plot([Ref_rl(1), Ref_rr(1)],[Ref_rl(2), Ref_rr(2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
            plot([Ref_rr(1), Obs_fr(1)],[Ref_rr(2), Obs_fr(2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
            plot([Obs_fr(1), Obs_fl(1)],[Obs_fr(2), Obs_fl(2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
            plot([Obs_fl(1), Ref_rl(1)],[Obs_fl(2), Ref_rl(2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
            
%             set(handles.Trajectory.UserData(5),'XData',[Ref_rl(1), Ref_rr(1)],'YData',[Ref_rl(2), Ref_rr(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
%             set(handles.Trajectory.UserData(6),'XData',[Ref_rr(1), Obs_fr(1)],'YData',[Ref_rr(2), Obs_fr(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
%             set(handles.Trajectory.UserData(7),'XData',[Obs_fr(1), Obs_fl(1)],'YData',[Obs_fr(2), Obs_fl(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
%             set(handles.Trajectory.UserData(8),'XData',[Obs_fl(1), Ref_rl(1)],'YData',[Obs_fl(2), Ref_rl(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);

        %绘制障碍物方块
            color = 'black';
            linewidth = 0.5;
            linestyle = '-';  
            
            plot([Obs_fl(1),Obs_rl(1)],[Obs_fl(2),Obs_rl(2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
            plot([Obs_rl(1),Obs_rr(1)],[Obs_rl(2),Obs_rr(2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
            plot([Obs_rr(1),Obs_fr(1)],[Obs_rr(2),Obs_fr(2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
            plot([Ref_rl(1),Ref_fl(1)],[Ref_rl(2),Ref_fl(2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
            plot([Ref_fl(1),Ref_fr(1)],[Ref_fl(2),Ref_fr(2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
            plot([Ref_fr(1),Ref_rr(1)],[Ref_fr(2),Ref_rr(2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
            
%             set(handles.Trajectory.UserData(9),'XData',[Obs_fl(1),Obs_rl(1)],'YData',[Obs_fl(2),Obs_rl(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
%             set(handles.Trajectory.UserData(10),'XData',[Obs_rl(1),Obs_rr(1)],'YData',[Obs_rl(2),Obs_rr(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
%             set(handles.Trajectory.UserData(11),'XData',[Obs_rr(1),Obs_fr(1)],'YData',[Obs_rr(2),Obs_fr(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
%             set(handles.Trajectory.UserData(12),'XData',[Ref_rl(1),Ref_fl(1)],'YData',[Ref_rl(2),Ref_fl(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
%             set(handles.Trajectory.UserData(13),'XData',[Ref_fl(1),Ref_fr(1)],'YData',[Ref_fl(2),Ref_fr(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
%             set(handles.Trajectory.UserData(14),'XData',[Ref_fr(1),Ref_rr(1)],'YData',[Ref_fr(2),Ref_rr(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
        %填充
            color = 'black';
            linewidth = 0.5;
            linestyle = '--';
            facealpha = 0.5;
            
            fill([Obs_fl(1), Obs_fr(1), Obs_rr(1), Obs_rl(1)],[Obs_fl(2), Obs_fr(2), Obs_rr(2), Obs_rl(2)],... 
                color,'FaceAlpha',facealpha);
            fill([Ref_fl(1), Ref_fr(1), Ref_rr(1), Ref_rl(1)],[Ref_fl(2), Ref_fr(2), Ref_rr(2), Ref_rl(2)],... 
                color,'FaceAlpha',facealpha);
            set(handles.Trajectory,'XLim',tr_xlim,'YLim',tr_ylim);
        
%             set(handles.Trajectory.UserData(15),'XData',[Obs_fl(1)+(Obs_fr(1)-Obs_fl(1))*3/4,Obs_rr(1)+(Obs_fr(1)-Obs_rr(1))*3/4],... 
%                 'YData',[Obs_fl(2)+(Obs_fr(2)-Obs_fl(2))*3/4,Obs_rr(2)+(Obs_fr(2)-Obs_rr(2))*3/4],'Color',color,'LineStyle',linestyle,'LineWidth',linewidth);
%             set(handles.Trajectory.UserData(16),'XData',[Obs_fl(1)+(Obs_fr(1)-Obs_fl(1))*2/4,Obs_rr(1)+(Obs_fr(1)-Obs_rr(1))*2/4],... 
%                 'YData',[Obs_fl(2)+(Obs_fr(2)-Obs_fl(2))*2/4,Obs_rr(2)+(Obs_fr(2)-Obs_rr(2))*2/4],'Color',color,'LineStyle',linestyle,'LineWidth',linewidth);
%             set(handles.Trajectory.UserData(17),'XData',[Obs_fl(1)+(Obs_fr(1)-Obs_fl(1))*1/4,Obs_rr(1)+(Obs_fr(1)-Obs_rr(1))*1/4],... 
%                 'YData',[Obs_fl(2)+(Obs_fr(2)-Obs_fl(2))*1/4,Obs_rr(2)+(Obs_fr(2)-Obs_rr(2))*1/4],'Color',color,'LineStyle',linestyle,'LineWidth',linewidth);
%             set(handles.Trajectory.UserData(18),'XData',[Obs_fl(1),Obs_rr(1)],'YData',[Obs_fl(2),Obs_rr(2)],'Color',color,'LineStyle',linestyle,'LineWidth',linewidth);
%             set(handles.Trajectory.UserData(19),'XData',[Obs_fl(1)+(Obs_rl(1)-Obs_fl(1))*1/4,Obs_rr(1)+(Obs_rl(1)-Obs_rr(1))*1/4],... 
%                 'YData',[Obs_fl(2)+(Obs_rl(2)-Obs_fl(2))*1/4,Obs_rr(2)+(Obs_rl(2)-Obs_rr(2))*1/4],'Color',color,'LineStyle',linestyle,'LineWidth',linewidth);
%             set(handles.Trajectory.UserData(20),'XData',[Obs_fl(1)+(Obs_rl(1)-Obs_fl(1))*2/4,Obs_rr(1)+(Obs_rl(1)-Obs_rr(1))*2/4],... 
%                 'YData',[Obs_fl(2)+(Obs_rl(2)-Obs_fl(2))*2/4,Obs_rr(2)+(Obs_rl(2)-Obs_rr(2))*2/4],'Color',color,'LineStyle',linestyle,'LineWidth',linewidth);
%             set(handles.Trajectory.UserData(21),'XData',[Obs_fl(1)+(Obs_rl(1)-Obs_fl(1))*3/4,Obs_rr(1)+(Obs_rl(1)-Obs_rr(1))*3/4],... 
%                 'YData',[Obs_fl(2)+(Obs_rl(2)-Obs_fl(2))*3/4,Obs_rr(2)+(Obs_rl(2)-Obs_rr(2))*3/4],'Color',color,'LineStyle',linestyle,'LineWidth',linewidth);
%             
%             set(handles.Trajectory.UserData(22),'XData',[Ref_rl(1)+(Ref_rr(1)-Ref_rl(1))*3/4,Ref_rl(1)+(Ref_fl(1)-Ref_rl(1))*3/4],... 
%                 'YData',[Ref_rl(2)+(Ref_rr(2)-Ref_rl(2))*3/4,Ref_rl(2)+(Ref_fl(2)-Ref_rl(2))*3/4],'Color',color,'LineStyle',linestyle,'LineWidth',linewidth);
%             set(handles.Trajectory.UserData(23),'XData',[Ref_rl(1)+(Ref_rr(1)-Ref_rl(1))*2/4,Ref_rl(1)+(Ref_fl(1)-Ref_rl(1))*2/4],... 
%                 'YData',[Ref_rl(2)+(Ref_rr(2)-Ref_rl(2))*2/4,Ref_rl(2)+(Ref_fl(2)-Ref_rl(2))*2/4],'Color',color,'LineStyle',linestyle,'LineWidth',linewidth);
%             set(handles.Trajectory.UserData(24),'XData',[Ref_rl(1)+(Ref_rr(1)-Ref_rl(1))*1/4,Ref_rl(1)+(Ref_fl(1)-Ref_rl(1))*1/4],... 
%                 'YData',[Ref_rl(2)+(Ref_rr(2)-Ref_rl(2))*1/4,Ref_rl(2)+(Ref_fl(2)-Ref_rl(2))*1/4],'Color',color,'LineStyle',linestyle,'LineWidth',linewidth);
%             set(handles.Trajectory.UserData(25),'XData',[Ref_rr(1),Ref_fl(1)],'YData',[Ref_rr(2),Ref_fl(2)],'Color',color,'LineStyle',linestyle,'LineWidth',linewidth);
%             set(handles.Trajectory.UserData(26),'XData',[Ref_fr(1)+(Ref_fl(1)-Ref_fr(1))*3/4,Ref_fr(1)+(Ref_rr(1)-Ref_fr(1))*3/4],... 
%                 'YData',[Ref_fr(2)+(Ref_fl(2)-Ref_fr(2))*3/4,Ref_fr(2)+(Ref_rr(2)-Ref_fr(2))*3/4],'Color',color,'LineStyle',linestyle,'LineWidth',linewidth);
%             set(handles.Trajectory.UserData(27),'XData',[Ref_fr(1)+(Ref_fl(1)-Ref_fr(1))*2/4,Ref_fr(1)+(Ref_rr(1)-Ref_fr(1))*2/4],... 
%                 'YData',[Ref_fr(2)+(Ref_fl(2)-Ref_fr(2))*2/4,Ref_fr(2)+(Ref_rr(2)-Ref_fr(2))*2/4],'Color',color,'LineStyle',linestyle,'LineWidth',linewidth);
%             set(handles.Trajectory.UserData(28),'XData',[Ref_fr(1)+(Ref_fl(1)-Ref_fr(1))*1/4,Ref_fr(1)+(Ref_rr(1)-Ref_fr(1))*1/4],... 
%                 'YData',[Ref_fr(2)+(Ref_fl(2)-Ref_fr(2))*1/4,Ref_fr(2)+(Ref_rr(2)-Ref_fr(2))*1/4],'Color',color,'LineStyle',linestyle,'LineWidth',linewidth);
        end

        if ~isnan(LocalX)
            %% 记录当前车辆位置
            %求从车身坐标系到全局坐标系的刚体变换矩阵
            T = [cos(Yaw), -sin(Yaw), LocalX; sin(Yaw), cos(Yaw), LocalY; 0, 0, 1];
            %求车辆八角点在车身坐标系下的位置
            V1L = [3.026;0.3955;1];V2L=[3.026;-0.3955;1];V3L=[2.646;-0.7755;1]; V4L=[-0.384;-0.7755;1];
            V5L = [-0.544;-0.4105;1]; V6L = [-0.544;0.4105;1]; V7L = [-0.384;0.7755;1]; V8L = [2.646;0.7755;1];
            VCL = [1.131;0;1];
            %求车辆八角点在全局坐标系下的位置
            V1G = T*V1L; V2G = T*V2L; V3G = T*V3L; V4G = T*V4L;
            V5G = T*V5L; V6G = T*V6L; V7G = T*V7L; V8G = T*V8L;
            VCG = T*VCL;           
            
            %绘制车辆模型，以长方形框表示实时位置
            color = 'blue';
            linewidth = 0.5;
            linestyle = '-';
            
            plot([V1G(1),V2G(1)],[V1G(2),V2G(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
            hold on
            plot([V3G(1),V2G(1)],[V3G(2),V2G(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
            plot([V3G(1),V4G(1)],[V3G(2),V4G(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
            plot([V5G(1),V4G(1)],[V5G(2),V4G(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
            plot([V5G(1),V6G(1)],[V5G(2),V6G(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
            plot([V7G(1),V6G(1)],[V7G(2),V6G(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
            plot([V7G(1),V8G(1)],[V7G(2),V8G(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
            plot([V1G(1),V8G(1)],[V1G(2),V8G(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
            set(handles.Trajectory,'XLim',tr_xlim,'YLim',tr_ylim);
            
            %             set(handles.Trajectory.UserData(29),'XData',[V1G(1),V2G(1)],'YData',[V1G(2),V2G(2)]);
            %             set(handles.Trajectory.UserData(30),'XData',[V3G(1),V2G(1)],'YData',[V3G(2),V2G(2)]);
            %             set(handles.Trajectory.UserData(31),'XData',[V3G(1),V4G(1)],'YData',[V3G(2),V4G(2)]);
            %             set(handles.Trajectory.UserData(32),'XData',[V5G(1),V4G(1)],'YData',[V5G(2),V4G(2)]);
            %             set(handles.Trajectory.UserData(33),'XData',[V5G(1),V6G(1)],'YData',[V5G(2),V6G(2)]);
            %             set(handles.Trajectory.UserData(34),'XData',[V7G(1),V6G(1)],'YData',[V7G(2),V6G(2)]);
            %             set(handles.Trajectory.UserData(35),'XData',[V7G(1),V8G(1)],'YData',[V7G(2),V8G(2)]);
            %             set(handles.Trajectory.UserData(36),'XData',[V1G(1),V8G(1)],'YData',[V1G(2),V8G(2)]);
        end

        if ~isnan(LocalX)
            if ~isnan(RefPose1)
            %求从全局坐标系到车位坐标系的刚体变换矩阵
            % Translate
            Tt = [1 0 -p_rl(1); 0 1 -p_rl(2); 0 0 1];
            % Rotate
            Tr = [cos(PoseTheta) sin(PoseTheta) 0; -sin(PoseTheta) cos(PoseTheta) 0; 0 0 1];
            %求车辆质心在目标车位坐标系下的位置
            V0 = [LocalX;LocalY;1];
            V = Tt * Tr * V0; %V为车位坐标系下车辆质心位置，车位坐标系以障碍车外角点为原点
            
            %横向偏差
            h = abs(V(2));
            yError = h-0.8;  % positive -- outside         
            set(handles.yError,'string',num2str(yError,'%.2f'));
            
            %纵向偏差
            x_= (p_length - 3.57)/2 + 0.544; %标准纵向位置
            xError = V(1) - x_;      % positive -- close to the reference         
            set(handles.xError,'string',num2str(xError,'%.2f'));
            
            %航向角偏差
            HeadingAngleError = rad2deg(Yaw-PoseTheta);  % positive -- counterclockwise
            set(handles.HeadingAngleError,'string',num2str(HeadingAngleError,'%.2f'));        
            
            Pos_Car=[LocalX;LocalY;Yaw];
%             rfp1=[RefPose1(1);RefPose1(2)]; rfp2=[RefPose2(1); RefPose2(2)];
%             obp1=[ObstaclePose1(1);ObstaclePose1(2)]; obp2=[ObstaclePose2(1);ObstaclePose2(2)];
            risk_score = risk_score + Risk_Assessment(RefPose1',RefPose2',ObstaclePose1',ObstaclePose2',Pos_Car,Vehicle);
            end
        end
        pause(0.06);
    end
    
    if flag_loop2 && ~exit && ~empty(angle)
        time_stop = now;   % record stop time
        % clear all the subscribers
        sub_steering_angle = 0;
        velometer = 0;
        imu = 0;
        parking_slot = 0;
        Vehicle_pose2D = 0;
        
        parkingtime = caltime(time_start, time_stop);
        flag_loop2 = 0;
        % record the latest position
        last_Ref = [p_fl(1:2)'; p_fr(1:2)'; p_rr(1:2)'; p_rl(1:2)'; ...
            Ref_fl; Ref_fr(1:2)'; Ref_rr(1:2)'; Ref_rl; ...
            Obs_fl; Obs_fr(1:2)'; Obs_rr(1:2)'; Obs_rl; ...
            V1G(1:2)';V2G(1:2)';V3G(1:2)';V4G(1:2)'; ...
            V5G(1:2)';V6G(1:2)';V7G(1:2)';V8G(1:2)'];
        
        % judge: did the car drive into the parking lot? (1-no, 0-yes)
        standardLine = Ref_rl - Obs_fl;
        standardLine(3) = 0;
        lineGroup = last_Ref(13:20,:) - Obs_fl;
        lineGroup(:,3) = 0;
        judgeMatrix = zeros(8, 3);
        for number = 1 : 8
            judgeMatrix(number, :) = cross(standardLine, lineGroup(number,:)) > 0;
        end
        if sum(judgeMatrix(:,3)) == 0
            %泊车时间评分
            Time_score = T_Assessment(parkingtime);
            fprintf('Time_score =%d\n', Time_score);

            %姿态精度评分
            angleError = deg2rad(HeadingAngleError);
            acc_score = acc_Assessment(abs(xError),abs(yError),abs(angleError));
            fprintf('acc_score =%d\n', acc_score);

            %舒适度评分
            Acc = get_data(LocalA);
%             LocalAx = ;   %纵向加速度
%             LocalAy = ;   %横向加速度
            com_score = com_Assessment(Acc(:,2),Acc(:,3));
            fprintf('com_score =%d\n', com_score);

            %原地转向时长评分
%             VehicleSpeed_data = ;
%             angle_data = ;
            rot_score = rot_Assessment(get_data(VehicleSpeed),get_data(angle));
            fprintf('rot_score =%d\n', rot_score);

            %计算泊车评分         
            risk=Risk(risk_score, size(parkingSlot));
            fprintf('risk_score =%d\n', risk);
            % score = [weight of every element; score of every element; percent of every element]
            score = eva(Time_score,acc_score,risk,com_score,rot_score);


            %显示泊车时间与评分
            %x=[Time_score, acc_score, com_score, rot_score, risk, 10-score];
            axes(handles.sc1); set(handles.sc1,'fontsize',15);
            pie([score(1,3), score(2,3), score(3,3), score(4,3), score(5,3)], ...
                {sprintf('%s\n','Time')+string(round(100 * score(1,3)))+'%', ...
                sprintf('%s\n','Acc')+string(round(100 * score(2,3)))+'%', ...
                sprintf('%s\n','Risk')+string(round(100 * score(3,3)))+'%', ...
                sprintf('%s\n','Com')+string(round(100 * score(4,3)))+'%', ...
                sprintf('%s\n','Rot')+string(round(100 * score(5,3)))+'%'}); 
            set(handles.timeScore, 'String', [num2str(score(1,2), '%.2f'), '/', num2str(10*score(1,1),'%.2f'), '            ', num2str(round(100 * score(1,4)), '%d'), '%']);
            set(handles.accScore, 'String', [num2str(score(2,2), '%.2f'), '/', num2str(10*score(2,1),'%.2f'), '            ', num2str(round(100 * score(2,4)), '%d'), '%']);
            set(handles.safetyScore, 'String', [num2str(score(3,2), '%.2f'), '/', num2str(10*score(3,1),'%.2f'), '            ', num2str(round(100 * score(3,4)), '%d'), '%']);
            set(handles.comScore, 'String', [num2str(score(4,2), '%.2f'), '/', num2str(10*score(4,1),'%.2f'), '            ', num2str(round(100 * score(4,4)), '%d'), '%']);
            set(handles.rotScore, 'String', [num2str(score(5,2), '%.2f'), '/', num2str(10*score(5,1),'%.2f'), '            ', num2str(round(100 * score(5,4)), '%d'), '%']);
        else
            score = zeros(1,4);
            setlog(handles, '车辆未完全泊入库位中，本次泊车成绩为0分。');
        end        
        set(handles.Time,'string',[num2str(parkingtime,'%.2f'), ' s']);
        set(handles.score,'string',num2str(sum(score(:,2)),'%.2f'));
        
        % saving data
        if get(handles.St_save,'Value')
            saveFileName = [get(handles.set_name,'String') '_' datestr(now,'yymmddHHMM')];
            if ~exist('DataSave', 'dir')
                mkdir('DataSave');
            end
            saveFileName = ['DataSave/' saveFileName '.xls'];
            setlog(handles, '正在保存数据...');
            writematrix(angle.get_data(), saveFileName, 'Sheet', '!steering_angle_deg');
            writematrix(VehicleSpeed.get_data(), saveFileName, 'Sheet', '!velometer!base_link_local');
            writematrix(LocalA.get_data(), saveFileName, 'Sheet', '!imu!data');
            writematrix(parkingSlot.get_data(), saveFileName, 'Sheet', '!parking_slot_info');
            writematrix(vehiclePose.get_data(), saveFileName, 'Sheet', '!odometer!local_map!base_link');
            setlog(handles, '保存完毕，请继续。');
        end
        % show the button
        set(handles.push_show,'Visible',1);
    end
end    

% --- Start parking
function StartStop(handles)
global stop;
if ~get(handles.ui_paraset,'Visible')
    stop = ~stop;
    if ~stop
        set(handles.push_start,'String','结束泊车');
        setlog(handles, '泊车开始。');
        setlog(handles, datestr(now));
        set(handles.push_show,'Visible',0);
    else
        set(handles.push_start,'String','开始泊车');
        setlog(handles, '泊车结束。');
        setlog(handles, datestr(now));
    end
end

% --- Set GUI
function SetGUI(handles)
global tr_xlim
global tr_ylim
global p_width
global displaymethod

if get(handles.ui_paraset,'Visible')
    set(handles.ui_paraset, 'Visible', 0);
    set(handles.uipanel1, 'Visible', 1);
    xmin = str2num(get(handles.set_xmin,'String'));
    xmax = str2num(get(handles.set_xmax,'String'));
    ymin = str2num(get(handles.set_ymin,'String'));
    ymax = str2num(get(handles.set_ymax,'String'));
    [xmin, xmax] = inorder(xmin, xmax);
    [ymin, ymax] = inorder(ymin, ymax);
    tr_xlim = [xmin, xmax];
    tr_ylim = [ymin, ymax];
    p_width = str2num(get(handles.set_pkwidth,'String'));
    displaymethod = get(handles.St_show, 'Value');
end

% --- Reset GUI
function ResetGUI(handles)
global stop;
global flag_loop2;
global PubArrayText;
global h_tr;
global flag_show;
global last_Ref;

if ~get(handles.ui_paraset,'Visible')
    stop = 1;
    flag_loop2 = 0;
    h_tr = 0;
    flag_show = 0;
    PubArrayText = sprintf('%s\n','');
    last_Ref = zeros(12,2);

    set(handles.ui_paraset, 'Visible', 1);
    set(handles.uipanel1, 'Visible', 0);
    set(handles.push_show,'Visible',0);

    set(handles.push_start,'String','开始泊车');
    set(handles.push_show,'String','显示轨迹');
    set(handles.Time, 'String', '');
    set(handles.Notice,'String',PubArrayText);

    clear_graph(handles);

    set(handles.P_length,'string','');
    set(handles.angle,'string','');
    set(handles.VehicleSpeed,'string','');
    set(handles.Localay,'string','');
    set(handles.Localax,'string','');
    set(handles.xError,'string','');
    set(handles.yError,'string','');
    set(handles.HeadingAngleError,'string','');
    set(handles.score,'string','');

    set(handles.St_angle,'Value',0);
    set(handles.St_VehicleSpeed,'Value',0);
    set(handles.St_LocalA,'Value',0);
    set(handles.St_parkingSlot,'Value',0);
    set(handles.St_vehiclePose,'Value',0);
end
% --- Exit GUI
function ExitGUI(handles)
global exit

rosshutdown;
setlog(handles, '正在退出......');
exit = 1;

close(gcf);

% --- Update topic message
function [msgback, vector_indice] = getmsg(vector, vector_indice, handles, varargin)
if isempty(varargin)
    if vector.indice > vector_indice
        eval(['set(' 'handles.St_' inputname(1) ', ''Value'', 1)']);
%         set(eval(['handles.St_',inputname(1)]),'Value',1);
        msgback = vector.back();
        vector_indice = vector.indice;
    else
        eval(['set(' 'handles.St_' inputname(1) ', ''Value'', 0)']);
%         set(eval(['handles.St_',inputname(1)]),'Value',0);
        if vector.indice == 0
            msgback = repelem(NaN, vector.dim);
        else
            msgback = vector.back();
        end
    end
else
    msgback = repelem({NaN},varargin{1});
    if vector{1}.indice > vector_indice
        eval(['set(' 'handles.St_' inputname(1) ', ''Value'', 1)']);
%         set(eval(['handles.St_',inputname(1)]),'Value',1);
        for num = 1 : varargin{1}
            msgback{num} = vector{num}.back();
        end
        vector_indice = vector{1}.indice;
    else
        eval(['set(' 'handles.St_' inputname(1) ', ''Value'', 0)']);
%         set(eval(['handles.St_',inputname(1)]),'Value',0);
        if vector{1}.indice ~= 0
            for num = 1 : varargin{1}
                msgback{num} = vector{num}.back();
            end
        end
    end
end
    
% --- Display Notice
function setlog(handles, str)
string = sprintf('%s\n',str);
global PubArrayText
PubArrayText = horzcat(string, PubArrayText);
set(handles.Notice,'String',PubArrayText);

% --- Calculate parking time
function time = caltime(t1, t2)
timestr = datestr(t2 - t1, 'HH:MM:SS.FFF');
timevec = datevec(timestr);
time = (timevec(4)*60 + timevec(5))*60 + timevec(6);

% --- Adjust the xlim/ylim order
function [xmin, xmax] = inorder(xmin, xmax)
if xmin > xmax
    t = xmax;
    xmax = xmin;
    xmin = t;
elseif xmin == xmax
    xmax = xmin + 1;
end
    
% --- Calculate the target position
function [p_fl, p_fr, p_rr, p_rl, Rec_l] = calpp(R1, R2, O1, O2, width)
VecL = [R1(1)-O1(1), R1(2)-O1(2)];
VecObs = [O2(1)-O1(1), O2(2)-O1(2)];
VecRef = [R2(1)-R1(1), R2(2)-R1(2)];
theta = atan2(VecL(2), VecL(1));
Flag = [dot(VecL, VecObs) <= 0, dot(-VecL, VecRef) <= 0]; % 1-obtuse angle, 0-acute angle
if isequal(Flag, [1, 1])
    Rec_l = norm(VecL);
    T = [cos(theta), -sin(theta), O1(1); sin(theta), cos(theta), O1(2); 0, 0, 1];
    p_fl = [R1(1); R1(2); 1];
    p_fr = T*[Rec_l; -width; 1];
    p_rr = T*[0; -width; 1];
    p_rl = [O1(1); O1(2); 1];
elseif isequal(Flag, [1, 0])
    Rec_l = norm(VecL) - dot(-VecL, VecRef)/norm(VecL);
    T = [cos(theta), -sin(theta), O1(1); sin(theta), cos(theta), O1(2); 0, 0, 1];
    p_fl = T*[Rec_l; 0; 1];
    p_fr = T*[Rec_l; -width; 1];
    p_rr = T*[0; -width; 1];
    p_rl = [O1(1); O1(2); 1];
elseif isequal(Flag, [0, 0])
    Rec_l = norm(VecL) - (dot(VecL, VecObs) + dot(-VecL, VecRef))/norm(VecL);
    Obsw = abs(det([VecL; VecObs]))/norm(VecL);
    T = [cos(theta), -sin(theta), O2(1); sin(theta), cos(theta), O2(2); 0, 0, 1];
    p_fl = T*[Rec_l; Obsw; 1];
    p_fr = T*[Rec_l; Obsw-width; 1];
    p_rr = T*[0; Obsw-width; 1];
    p_rl = T*[0; Obsw; 1];
else
    Rec_l = norm(VecL) - dot(VecL, VecObs)/norm(VecL);
    T = [cos(theta), -sin(theta), R1(1); sin(theta), cos(theta), R1(2); 0, 0, 1];
    p_fl = [R1(1); R1(2); 1];
    p_fr = T*[0; -width; 1];
    p_rr = T*[-Rec_l; -width; 1];
    p_rl = T*[-Rec_l; 0; 1];
end

% --- initial graphics handles
function [h, n] = initgobj(r,c)
h = gobjects(r,c);
n = [1,1];

% --- plot using handles
function [h, n] = myplot(h, n, x, y)
increase = 1;
if size(h,1) < n(1)
    h = [h; gobjects(increase, size(h,2))];
end
h(n(1),n(2)) = plot(x, y);
n(2) = n(2) + 1;
if n(2) > size(h,2)
    n(2) = 1;
    n(1) = n(1) + 1;
end

% --- fill using handles
function [h, n] = myfill(h, n, x, y)
if size(h,1) < n(1)
    h = [h; gobjects(size(h,1), size(h,2))];
end
h(n(1),n(2)) = fill(x, y, 'black');
n(2) = n(2) + 1;
if n(2) > size(h,2)
    n(2) = 1;
    n(1) = n(1) + 1;
end 

% --- show trajectory
function showtrajectory(handles, vehiclePose)
global h_tr

if ~get(handles.St_show, 'Value')
    %% use handles
    
    if h_tr ~= 0
        set(h_tr, 'Visible', 1);
    else
        data = vehiclePose.get_data();
        NumberofTr = [1, 1];
        [h_tr, n_tr] = initgobj(NumberofTr(1),NumberofTr(2));
        axes(handles.Trajectory)
        %求车辆八角点在车身坐标系下的位置
        V1L = [3.026;0.3955;1];V2L=[3.026;-0.3955;1];V3L=[2.646;-0.7755;1]; V4L=[-0.384;-0.7755;1];
        V5L = [-0.544;-0.4105;1]; V6L = [-0.544;0.4105;1]; V7L = [-0.384;0.7755;1]; V8L = [2.646;0.7755;1];
        color = 'blue';
        linewidth = 0.5;
        linestyle = '-';
        for n = 1 : size(data, 1)
            T = [cos(data(n,4)), -sin(data(n,4)), data(n,2);...
                sin(data(n,4)), cos(data(n,4)), data(n,3); 0, 0, 1];
            %求车辆八角点在全局坐标系下的位置
            V1G = T*V1L; V2G = T*V2L; V3G = T*V3L; V4G = T*V4L;
            V5G = T*V5L; V6G = T*V6L; V7G = T*V7L; V8G = T*V8L;
            % Draw
            hold on
            [h_tr, n_tr] = myplot(h_tr, n_tr, [V1G(1),V2G(1)],[V1G(2),V2G(2)]);
            [h_tr, n_tr] = myplot(h_tr, n_tr, [V3G(1),V2G(1)],[V3G(2),V2G(2)]);
            [h_tr, n_tr] = myplot(h_tr, n_tr, [V3G(1),V4G(1)],[V3G(2),V4G(2)]);
            [h_tr, n_tr] = myplot(h_tr, n_tr, [V5G(1),V4G(1)],[V5G(2),V4G(2)]);
            [h_tr, n_tr] = myplot(h_tr, n_tr, [V5G(1),V6G(1)],[V5G(2),V6G(2)]);
            [h_tr, n_tr] = myplot(h_tr, n_tr, [V7G(1),V6G(1)],[V7G(2),V6G(2)]);
            [h_tr, n_tr] = myplot(h_tr, n_tr, [V7G(1),V8G(1)],[V7G(2),V8G(2)]);
            [h_tr, n_tr] = myplot(h_tr, n_tr, [V1G(1),V8G(1)],[V1G(2),V8G(2)]);
        end
        set(h_tr(1:n_tr(1)-1, 1:size(h_tr,2)), 'Color', color, 'LineWidth', linewidth, 'LineStyle', linestyle);
    end
    
else
    %% abort handles
    data = vehiclePose.get_data();
    axes(handles.Trajectory)
    %求车辆八角点在车身坐标系下的位置
    V1L = [3.026;0.3955;1];V2L=[3.026;-0.3955;1];V3L=[2.646;-0.7755;1]; V4L=[-0.384;-0.7755;1];
    V5L = [-0.544;-0.4105;1]; V6L = [-0.544;0.4105;1]; V7L = [-0.384;0.7755;1]; V8L = [2.646;0.7755;1];
    color = 'blue';
    linewidth = 0.5;
    linestyle = '-';
    for n = 1 : size(data, 1)
        T = [cos(data(n,4)), -sin(data(n,4)), data(n,2);...
            sin(data(n,4)), cos(data(n,4)), data(n,3); 0, 0, 1];
        %求车辆八角点在全局坐标系下的位置
        V1G = T*V1L; V2G = T*V2L; V3G = T*V3L; V4G = T*V4L;
        V5G = T*V5L; V6G = T*V6L; V7G = T*V7L; V8G = T*V8L;
        hold on
        plot([V1G(1),V2G(1)],[V1G(2),V2G(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
        plot([V3G(1),V2G(1)],[V3G(2),V2G(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
        plot([V3G(1),V4G(1)],[V3G(2),V4G(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
        plot([V5G(1),V4G(1)],[V5G(2),V4G(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
        plot([V5G(1),V6G(1)],[V5G(2),V6G(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
        plot([V7G(1),V6G(1)],[V7G(2),V6G(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
        plot([V7G(1),V8G(1)],[V7G(2),V8G(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
        plot([V1G(1),V8G(1)],[V1G(2),V8G(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
    end
    
end
setlog(handles, '显示完毕。');

% --- hide trajectory
function hidetrajectory(handles, h, Ref)
global tr_xlim
global tr_ylim

if h == 0
    cla(handles.Trajectory)
    axes(handles.Trajectory)
    %绘制算法目标车位
    hold on
    color = 'red';
    linewidth = 0.5;
    linestyle = '-';
    plot([Ref(1,1), Ref(2,1)],[Ref(1,2), Ref(2,2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
    plot([Ref(2,1), Ref(3,1)],[Ref(2,2), Ref(3,2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
    plot([Ref(3,1), Ref(4,1)],[Ref(3,2), Ref(4,2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
    plot([Ref(4,1), Ref(1,1)],[Ref(4,2), Ref(1,2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
    %绘制实际泊车空间
    color = 'green';
    linewidth = 0.5;
    linestyle = '-';
    
    plot([Ref(8,1), Ref(7,1)],[Ref(8,2), Ref(7,2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
    plot([Ref(7,1), Ref(10,1)],[Ref(7,2), Ref(10,2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
    plot([Ref(10,1), Ref(9,1)],[Ref(10,2), Ref(9,2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
    plot([Ref(9,1), Ref(8,1)],[Ref(9,2), Ref(8,2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
    %绘制障碍物方块
    color = 'black';
    linewidth = 0.5;
    linestyle = '-';
    
    plot([Ref(9,1),Ref(12,1)],[Ref(9,2),Ref(12,2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
    plot([Ref(12,1),Ref(11,1)],[Ref(12,2),Ref(11,2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
    plot([Ref(11,1),Ref(10,1)],[Ref(11,2),Ref(10,2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
    plot([Ref(8,1),Ref(5,1)],[Ref(8,2),Ref(5,2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
    plot([Ref(5,1),Ref(6,1)],[Ref(5,2),Ref(6,2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
    plot([Ref(6,1),Ref(7,1)],[Ref(6,2),Ref(7,2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
    %填充
    color = 'black';
    facealpha = 0.5;
    
    fill([Ref(9,1), Ref(10,1), Ref(11,1), Ref(12,1)],[Ref(9,2), Ref(10,2), Ref(11,2), Ref(12,2)],...
        color,'FaceAlpha',facealpha);
    fill([Ref(5,1), Ref(6,1), Ref(7,1), Ref(8,1)],[Ref(5,2), Ref(6,2), Ref(7,2), Ref(8,2)],...
        color,'FaceAlpha',facealpha);
    set(handles.Trajectory,'XLim',tr_xlim,'YLim',tr_ylim);
else
    set(h,'Visible',0);
end
%车辆
color = 'blue';
linewidth = 0.5;
linestyle = '-';

plot([Ref(13,1),Ref(14,1)],[Ref(13,2),Ref(14,2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
plot([Ref(15,1),Ref(14,1)],[Ref(15,2),Ref(14,2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
plot([Ref(15,1),Ref(16,1)],[Ref(15,2),Ref(16,2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
plot([Ref(17,1),Ref(16,1)],[Ref(17,2),Ref(16,2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
plot([Ref(17,1),Ref(18,1)],[Ref(17,2),Ref(18,2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
plot([Ref(19,1),Ref(18,1)],[Ref(19,2),Ref(18,2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
plot([Ref(19,1),Ref(20,1)],[Ref(19,2),Ref(20,2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
plot([Ref(13,1),Ref(20,1)],[Ref(13,2),Ref(20,2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);

% --- Clear graph
function clear_graph(handles)
global tr_xlim;  % xlim of handles.Trajectory
global tr_ylim;  % ylim of handles.Trajectory

axes(handles.sc1); cla(handles.sc1); set(handles.sc1, 'XTick', {}, 'YTick', {});
set(handles.timeScore, 'string', ' ');
set(handles.accScore, 'string', ' ');
set(handles.safetyScore, 'string', ' ');
set(handles.comScore, 'string', ' ');
set(handles.rotScore, 'string', ' ');

axes(handles.Trajectory);
cla(handles.Trajectory);
set(handles.Trajectory,'XLim',tr_xlim,'YLim',tr_ylim);

% --- Executes on button press in push_paraset.
function push_paraset_Callback(hObject, eventdata, handles)
% hObject    handle to push_paraset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Xlim & Ylim for trajectory display    
    
if get(handles.push_paraset, 'value')
    SetGUI(handles);
end



% --- Executes on button press in push_start.
function push_start_Callback(hObject, eventdata, handles)
% hObject    handle to push_start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if get(handles.push_start, 'value')
    StartStop(handles);
end

% --- Executes on button press in push_exit.
function push_exit_Callback(hObject, eventdata, handles)
% hObject    handle to push_exit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if get(handles.push_exit, 'value')
    ExitGUI(handles);
end



% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
delete(hObject);

function set_xmin_Callback(hObject, eventdata, handles)
% hObject    handle to set_xmin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of set_xmin as text
%        str2double(get(hObject,'String')) returns contents of set_xmin as a double


% --- Executes during object creation, after setting all properties.
function set_xmin_CreateFcn(hObject, eventdata, handles)
% hObject    handle to set_xmin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function set_xmax_Callback(hObject, eventdata, handles)
% hObject    handle to set_xmax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of set_xmax as text
%        str2double(get(hObject,'String')) returns contents of set_xmax as a double


% --- Executes during object creation, after setting all properties.
function set_xmax_CreateFcn(hObject, eventdata, handles)
% hObject    handle to set_xmax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function set_ymin_Callback(hObject, eventdata, handles)
% hObject    handle to set_ymin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of set_ymin as text
%        str2double(get(hObject,'String')) returns contents of set_ymin as a double


% --- Executes during object creation, after setting all properties.
function set_ymin_CreateFcn(hObject, eventdata, handles)
% hObject    handle to set_ymin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function set_ymax_Callback(hObject, eventdata, handles)
% hObject    handle to set_ymax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of set_ymax as text
%        str2double(get(hObject,'String')) returns contents of set_ymax as a double


% --- Executes during object creation, after setting all properties.
function set_ymax_CreateFcn(hObject, eventdata, handles)
% hObject    handle to set_ymax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in push_reset.
function push_reset_Callback(hObject, eventdata, handles)
% hObject    handle to push_reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if get(handles.push_reset, 'value')
    ResetGUI(handles);
end



function set_pkwidth_Callback(hObject, eventdata, handles)
% hObject    handle to set_pkwidth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of set_pkwidth as text
%        str2double(get(hObject,'String')) returns contents of set_pkwidth as a double


% --- Executes during object creation, after setting all properties.
function set_pkwidth_CreateFcn(hObject, eventdata, handles)
% hObject    handle to set_pkwidth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in push_show.
function push_show_Callback(hObject, eventdata, handles)
% hObject    handle to push_show (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global vehiclePose
global flag_show
global h_tr
global last_Ref

if get(handles.push_show, 'value')
    flag_show = ~flag_show;
    if flag_show
        setlog(handles, '正在显示......');
        showtrajectory(handles, vehiclePose);
        set(handles.push_show,'String','隐藏轨迹');
    else
        setlog(handles,'隐藏轨迹。');
        hidetrajectory(handles, h_tr, last_Ref);
        set(handles.push_show,'String','显示轨迹');
    end
end


% --- Executes on button press in St_show.
function St_show_Callback(hObject, eventdata, handles)
% hObject    handle to St_show (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of St_show


% --- Executes on button press in push_close.
function push_close_Callback(hObject, eventdata, handles)
% hObject    handle to push_close (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global exit

if get(handles.push_close, 'value')
    rosshutdown;
    setlog(handles, '正在退出......');
    exit = 1;
end
close(gcf)


% --- Executes on button press in St_save.
function St_save_Callback(hObject, eventdata, handles)
% hObject    handle to St_save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of St_save


% --- Executes on button press in St_parkingSlot.
function St_parkingSlot_Callback(hObject, eventdata, handles)
% hObject    handle to St_parkingSlot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of St_parkingSlot


% --- Executes on button press in St_vehiclePose.
function St_vehiclePose_Callback(hObject, eventdata, handles)
% hObject    handle to St_vehiclePose (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of St_vehiclePose



function set_name_Callback(hObject, eventdata, handles)
% hObject    handle to set_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of set_name as text
%        str2double(get(hObject,'String')) returns contents of set_name as a double


% --- Executes during object creation, after setting all properties.
function set_name_CreateFcn(hObject, eventdata, handles)
% hObject    handle to set_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on key press with focus on figure1 or any of its controls.
function figure1_WindowKeyPressFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.FIGURE)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)

switch double(get(gcf,'CurrentCharacter'))
    case 13   % enter
        StartStop(handles)
    case 27   % esc
        ExitGUI(handles)
    case 114  % 'r'
        ResetGUI(handles)
    case 115  % 's'
        SetGUI(handles)
    otherwise
end
