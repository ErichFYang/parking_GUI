
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

% Last Modified by GUIDE v2.5 23-Jul-2021 01:14:32

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
global localx;
global localy;
global yaw;
global last_Ref;
global xError
global yError
global HeadingAngleError
global risk_score

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
    
    Ref1 = myvector(3);
    Ref2 = myvector(3);
    Ref3 = myvector(3);
    Ref4 = myvector(3);
    Obstacle1 = myvector(3);
    Obstacle2 = myvector(3);
    Obstacle3 = myvector(3);
    Obstacle4 = myvector(3);
    posetheta = myvector(2);
    Ref_indice = 0;
    parking_slot = rossubscriber('/parking_slot_info', 'apa_msgs/SlotInfoStamped',{@parkingslotCallback,Ref1,Ref2,Ref3,Ref4,Obstacle1,Obstacle2,Obstacle3,Obstacle4,posetheta});
    
    localx = myvector(2);
    localy = myvector(2);
    yaw = myvector(2);
    local_indice = 0;
    Vehicle_pose2D = rossubscriber('/odometer/local_map/base_link', 'nav_msgs/Odometry',{@Vehicle_pose2DCallback,localx,localy,yaw});
    
    %Timer
    setlog(handles, '话题订阅成功！');
    
    %Display modules initialization
    axes(handles.Trajectory);
    cla(handles.Trajectory);
    set(handles.Trajectory,'XLim',tr_xlim,'YLim',tr_ylim);    
%     NumberofV = [10, 8];
%     NumberofP = [1, 14];
%     NumberofF = [1, 2];
%     [h_v, n_hv] = initgobj(NumberofV(1),NumberofV(2));
%     [h_p, n_p] = initgobj(NumberofP(1),NumberofP(2));
%     [h_f, n_f] = initgobj(NumberofF(1),NumberofF(2));
    

%     for num = 1 : 36
%     set(handles.Trajectory,'UserData',[line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0]),...
%         line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0]),...
%         line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0]),...
%         line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0]),... 
%         ]);
%     end
    set(handles.Time, 'String', '');
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
        Ref = {Ref1, Ref2, Ref3, Ref4, Obstacle1, Obstacle2, Obstacle3, Obstacle4, posetheta};
        [Ref_msg, Ref_indice] = getmsg(Ref, Ref_indice, handles, 9);
        [RefPose1, RefPose2, RefPose3, RefPose4, ...
            ObstaclePose1, ObstaclePose2, ObstaclePose3, ObstaclePose4, PoseTheta] = Ref_msg{:};
        local = {localx, localy, yaw};
        [local_msg, local_indice] = getmsg(local, local_indice, handles, 3);
        [LocalX, LocalY, Yaw] = local_msg{:};
        
        
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
            VecObs = [ObstaclePose2(2) - ObstaclePose1(2), ObstaclePose2(3) - ObstaclePose1(3)];
            Theta_Obs = atan2(VecObs(2), VecObs(1));
            T_Obs = [cos(Theta_Obs), -sin(Theta_Obs), ObstaclePose1(2); ...
                sin(Theta_Obs), cos(Theta_Obs), ObstaclePose1(3); 0, 0, 1];
            Obs_fr = T_Obs * [p_width; 0; 1];
            Obs_rr = T_Obs * [p_width; -p_width; 1];
            Obs_fl = ObstaclePose1(2:3);
            Obs_rl = ObstaclePose3(2:3);
            VecRef = [RefPose2(2) - RefPose1(2), RefPose2(3) - RefPose1(3)];
            Theta_Ref = atan2(VecRef(2), VecRef(1));
            T_Ref = [cos(Theta_Ref), -sin(Theta_Ref), RefPose1(2); ...
                sin(Theta_Ref), cos(Theta_Ref), RefPose1(3); 0, 0, 1];
            Ref_rr = T_Ref * [p_width; 0; 1];
            Ref_fr = T_Ref * [p_width; p_width; 1];
            Ref_rl = RefPose1(2:3);
            Ref_fl = RefPose3(2:3);
            [p_fl, p_fr, p_rr, p_rl, p_length] = calpp(Ref_rl, Ref_rr, Obs_fl, Obs_fr, p_width);
            set(handles.P_length,'string',num2str(p_length,'%.2f'));
            
            
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
            T = [cos(Yaw(2)), -sin(Yaw(2)), LocalX(2); sin(Yaw(2)), cos(Yaw(2)), LocalY(2); 0, 0, 1];
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
            %             set(handles.Traject标注重合ory.UserData(31),'XData',[V3G(1),V4G(1)],'YData',[V3G(2),V4G(2)]);
            %             set(handles.Trajectory.UserData(32),'XData',[V5G(1),V4G(1)],'YData',[V5G(2),V4G(2)]);
            %             set(handles.Trajectory.UserData(33),'XData',[V5G(1),V6G(1)],'YData',[V5G(2),V6G(2)]);
            %             set(handles.Trajectory.UserData(34),'XData',[V7G(1),V6G(1)],'YData',[V7G(2),V6G(2)]);
            %             set(handles.Trajectory.UserData(35),'XData',[V7G(1),V8G(1)],'YData',[V7G(2),V8G(2)]);
            %             set(handles.Trajectory.UserData(36),'XData',[V1G(1),V8G(1)],'YData',[V1G(2),V8G(2)]);
        end
        
        if ~isnan(LocalX)
            if ~isnan(RefPose1)
            %求从全局坐标系到车位坐标系的刚体变换矩阵
            PoseTheta1=deg2rad(PoseTheta(2)); %转化为弧度
            T0 = [cos(PoseTheta1),-sin(PoseTheta1),-ObstaclePose1(2);sin(PoseTheta1),cos(PoseTheta1),-ObstaclePose1(3);0,0,1];
            %求车辆质心在目标车位坐标系下的位置
            V0 = [LocalX(2);LocalY(2);1];
            V = T0*V0; %V为车位坐标系下车辆质心位置，车位坐标系以障碍车外角点为原点
            R1 = T0*[RefPose1(2);RefPose1(3);1];  %R1为车位坐标系下参考车角点位置
            O1=T0*[ObstaclePose1(2);ObstaclePose1(3);1]; 
            
            %横向偏差
            h = abs(V(2));
            yError = h-0.8;
            yErrorDsp= findobj(0, 'tag', 'yError');           
            set(yErrorDsp,'string',[num2str(yError,'%.2f'),'    m']);
            
            %纵向偏差
            x_= (R1(1)-O1(1)-3.57)/2+0.544; %标准纵向位置
            xError = V(1) - x_;      
            xErrorDsp= findobj(0, 'tag', 'xError');           
            set(xErrorDsp,'string',[num2str(xError,'%.2f'),'    m']);
            
            %航向角偏差
            HeadingAngleError = rad2deg(Yaw(2))-PoseTheta(2);
            set(handles.HeadingAngleError,'string',[num2str(HeadingAngleError,'%.2f'),'  deg']);        
            
            Pos_Car=[LocalX(2);LocalY(2);Yaw(2)];
            rfp1=[RefPose1(2);RefPose1(3)]; rfp2=[RefPose2(2); RefPose2(3)];
            obp1=[ObstaclePose1(2);ObstaclePose1(3)]; obp2=[ObstaclePose2(2);ObstaclePose2(3)];
            risk_score = risk_score+ Risk_Assessment(rfp1,rfp2,obp1,obp2,Pos_Car,Vehicle);
            end
        end
        pause(0.1);
    end
    
    if flag_loop2 && ~exit
        time_stop = now;
        sub_steering_angle = 0;
        velometer = 0;
        imu = 0;
        parking_slot = 0;
        Vehicle_pose2D = 0;
        parkingtime = caltime(time_start, time_stop);
        flag_loop2 = 0;
        last_Ref = [p_fl(1:2)'; p_fr(1:2)'; p_rr(1:2)'; p_rl(1:2)'; ...
            Ref_fl; Ref_fr(1:2)'; Ref_rr(1:2)'; Ref_rl; ...
            Obs_fl; Obs_fr(1:2)'; Obs_rr(1:2)'; Obs_rl; ...
            V1G(1:2)';V2G(1:2)';V3G(1:2)';V4G(1:2)'; ...
            V5G(1:2)';V6G(1:2)';V7G(1:2)';V8G(1:2)'];
        
    %泊车结束
        %泊车时间评分
        Time_score = T_Assessment(parkingtime);
        fprintf('Time_score =%d\n', Time_score);
        
        %姿态精度评分
        angleError=deg2rad(HeadingAngleError);
        acc_score = acc_Assessment(xError,yError,angleError);
        fprintf('acc_score =%d\n', acc_score);
        
        %舒适度评分
        Acc = get_data(LocalA);
        LocalAx = Acc(:,2);   %纵向加速度
        LocalAy = Acc(:,3);   %横向加速度
        com_score = com_Assessment(LocalAx,LocalAy);
        fprintf('com_score =%d\n', com_score);
        
        %原地转向时长评分
        VehicleSpeed_data=get_data(VehicleSpeed);
        angle_data=get_data(angle);    
        rot_score = rot_Assessment(VehicleSpeed_data,angle_data);
        fprintf('rot_score =%d\n', rot_score);
    
        %计算泊车评分         
        risk=Risk(risk_score);
        fprintf('risk_score =%d\n', risk);
        score = eva(Time_score,acc_score,risk,com_score,rot_score);
    
         %碰撞距离

        %显示泊车时间与评分
        set(handles.Time,'string',[num2str(parkingtime,'%.2f'), ' s']);
        set(handles.push_show,'Visible',1);
        scoreDsp= findobj(0, 'tag', 'score');
        set(scoreDsp,'string',num2str(score,'%.2f'));
        
        %%
        %x=[Time_score, acc_score, com_score, rot_score, risk, 10-score];
        axes(handles.judge); set(gca,'fontsize',15)
        subplot(3,2,1);
        pie([Time_score, acc_score, com_score, rot_score, risk],{'time score','acc score','com score','rot score', 'risk score'});
        title('Total score components');
        subplot(3,2,2);
        pie([Time_score, 10-Time_score]); title('Time score');
        subplot(3,2,3);
        pie([acc_score, 10- acc_score]); title('Acc score');
        subplot(3,2,4);
        pie([com_score, 10-com_score]); title('Com score');
        subplot(3,2,5);
        pie([rot_score, 10- rot_score]); title('Rot score');
        subplot(3,2,6);
        pie([risk, 10-risk]); title('Risk score');
    end
end    


% --- Update topic message
function [msgback, vector_indice] = getmsg(vector, vector_indice, handles, varargin)
if isempty(varargin)
    if vector.indice > vector_indice
        set(eval(['handles.St_',inputname(1)]),'Value',1);
        msgback = vector.back();
        vector_indice = vector.indice;
    else
        set(eval(['handles.St_',inputname(1)]),'Value',0);
        msgback = repelem(NaN, vector.dim);
    end
else
    msgback = repelem({NaN},varargin{1});
    if vector{1}.indice > vector_indice
        set(eval(['handles.St_',inputname(1)]),'Value',1);
        for num = 1 : varargin{1}
            msgback{num} = vector{num}.back();
        end
        vector_indice = vector{1}.indice;
    else
        set(eval(['handles.St_',inputname(1)]),'Value',0);
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
function showtrajectory(handles, localx, localy, yaw)
global h_tr
global displaymethod

if ~displaymethod
    %% use handles
    
    if h_tr ~= 0
        set(h_tr, 'Visible', 1);
    else
        datax = localx.get_data();
        datay = localy.get_data();
        datayaw = yaw.get_data();
        NumberofTr = [1, 1];
        [h_tr, n_tr] = initgobj(NumberofTr(1),NumberofTr(2));
        axes(handles.Trajectory)
        %求车辆八角点在车身坐标系下的位置
        V1L = [3.026;0.3955;1];V2L=[3.026;-0.3955;1];V3L=[2.646;-0.7755;1]; V4L=[-0.384;-0.7755;1];
        V5L = [-0.544;-0.4105;1]; V6L = [-0.544;0.4105;1]; V7L = [-0.384;0.7755;1]; V8L = [2.646;0.7755;1];
        color = 'blue';
        linewidth = 0.5;
        linestyle = '-';
        for n = 1 : size(datax, 1)
            T = [cos(datayaw(n,2)), -sin(datayaw(n,2)), datax(n,2);...
                sin(datayaw(n,2)), cos(datayaw(n,2)), datay(n,2); 0, 0, 1];
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
    datax = localx.get_data();
    datay = localy.get_data();
    datayaw = yaw.get_data();
    axes(handles.Trajectory)
    %求车辆八角点在车身坐标系下的位置
    V1L = [3.026;0.3955;1];V2L=[3.026;-0.3955;1];V3L=[2.646;-0.7755;1]; V4L=[-0.384;-0.7755;1];
    V5L = [-0.544;-0.4105;1]; V6L = [-0.544;0.4105;1]; V7L = [-0.384;0.7755;1]; V8L = [2.646;0.7755;1];
    color = 'blue';
    linewidth = 0.5;
    linestyle = '-';
    for n = 1 : size(datax, 1)
        T = [cos(datayaw(n,2)), -sin(datayaw(n,2)), datax(n,2);...
            sin(datayaw(n,2)), cos(datayaw(n,2)), datay(n,2); 0, 0, 1];
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

% --- Executes on button press in push_paraset.
function push_paraset_Callback(hObject, eventdata, handles)
% hObject    handle to push_paraset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Xlim & Ylim for trajectory display    
global tr_xlim
global tr_ylim
global p_width
global displaymethod
    
if get(handles.push_paraset, 'value')
    set(handles.ui_paraset, 'Visible', 0);
    set(handles.uipanel1, 'Visible', 1);
end
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


% --- Executes on button press in push_start.
function push_start_Callback(hObject, eventdata, handles)
% hObject    handle to push_start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global stop

if get(handles.push_start, 'value')
    stop = ~stop;
    if ~stop
        set(handles.push_start,'String','结束泊车');
        setlog(handles, '泊车开始。');
        set(handles.push_show,'Visible',0);
    else
        set(handles.push_start,'String','开始泊车');
        setlog(handles, '泊车结束。');
    end
end

% --- Executes on button press in push_exit.
function push_exit_Callback(hObject, eventdata, handles)
% hObject    handle to push_exit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global exit

if get(handles.push_exit, 'value')
    setlog(handles, '正在退出......');
    exit = 1;
end
close(gcf)


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
global stop;
global flag_loop2;
global PubArrayText;
global h_tr;
global flag_show;
global last_Ref;

if get(handles.push_reset, 'value')
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
    
    cla(handles.Trajectory);
    cla(handles.judge);
    
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
    set(handles.St_Ref,'Value',0);
    set(handles.St_local,'Value',0);

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
global localx
global localy
global yaw
global flag_show
global h_tr
global last_Ref

if get(handles.push_show, 'value')
    flag_show = ~flag_show;
    if flag_show
        setlog(handles, '正在显示......');
        showtrajectory(handles, localx, localy, yaw);
        set(handles.push_show,'String','隐藏轨迹');
    else
        setlog(handles,'隐藏轨迹');
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
