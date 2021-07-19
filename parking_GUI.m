
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

% Last Modified by GUIDE v2.5 19-Jul-2021 20:47:39

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
addpath('/home/gxb/ProjectTeam/Ranking/Code/parking_GUI/Callbacks');
global PubArrayText

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

stop = 1;   
exit = 0;  
flag_loop2 = 0;

% main
while(~exit)
    pause(0.1);
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
    Refposetheta = myvector(2);
    Ref_indice = 0;
    parking_slot = rossubscriber('/parking_slot_info', 'apa_msgs/SlotInfoStamped',{@parkingslotCallback,Ref1,Ref2,Ref3,Ref4,Obstacle1,Obstacle2,Obstacle3,Obstacle4,Refposetheta});
    
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
    set(handles.Trajectory,'UserData',[line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0]),...
        line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0]),...
        line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0]),...
        line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0]),line([0 0],[0 0])]);
    
    set(handles.Time, 'String', '');
    end
    
    %泊车过程（while（1）循环内）
    while ~(stop||exit)
        if ~flag_loop2
            time_start = now;
            flag_loop2 = 1;
        end
        % Get message
        [latest_angle_record, angle_indice] = getmsg(angle, angle_indice, handles);
        [latest_speed_record, VehicleSpeed_indice] = getmsg(VehicleSpeed, VehicleSpeed_indice, handles);
        [latest_A_record, LocalA_indice] = getmsg(LocalA, LocalA_indice, handles);
        Ref = {Ref1, Ref2, Ref3, Ref4, Obstacle1, Obstacle2, Obstacle3, Obstacle4, Refposetheta};
        [Ref_msg, Ref_indice] = getmsg(Ref, Ref_indice, handles, 9);
        [RefPose1, RefPose2, RefPose3, RefPose4, ...
            ObstaclePose1, ObstaclePose2, ObstaclePose3, ObstaclePose4, RefPoseTheta] = Ref_msg{:};
        local = {localx, localy, yaw};
        [local_msg, local_indice] = getmsg(local, local_indice, handles, 3);
        [LocalX, LocalY, Yaw] = local_msg{:};
        if ~isnan(LocalX)
            %记录当前车辆位置
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
        end
%% Display modules

%         fprintf('current log length of steering_angle: %d\nlatest steering_angle: %f deg\n', angle.size(), latest_angle_record(2));
%         fprintf('current log length of vehicle_speed: %d\nlatest vehicle_speed: %f deg\n', VehicleSpeed.size(), latest_speed_record(2));
%         fprintf('current log length of Local_A: %d\nlatest Local_Ax: %f deg\nlatest Local_Ay: %f deg\n', VehicleSpeed.size(), latest_A_record(2),latest_A_record(3));
        %方向盘转角
        set(handles.angle,'string',num2str(latest_angle_record(2),'%.2f'));
        %车速
        set(handles.VehicleSpeed,'string',num2str(latest_speed_record(2),'%.2f'));
        %横向加速度      
        set(handles.Localay,'string',num2str(latest_A_record(3),'%.2f'));
        %纵向加速度        
        set(handles.Localax,'string',num2str(latest_A_record(2),'%.2f'));
        if ~isnan(RefPose1)
        %绘制目标车位
            axes(handles.Trajectory);
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
        end
        if ~isnan(LocalX)
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
        end
        drawnow
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
        %泊车结束
         %碰撞距离
        %CollisonDistance=  ;           ????????
    %     CollisonDistanceDsp= findobj(0, 'tag', 'CollisonDistance');   
    %     set(CollisonDistanceDsp,'string',num2str(CollisonDistance));

    %     %纵向偏差
    %     xError = VCG(1) - (ObstaclePose1(1,1) + RefPose1(1,1) + ObstaclePose2(1,1) + RefPose2(1,1))/4;   
    %     set(handles.xError,'string',num2str(xError)); 
    % 
    %     %横向偏差
    %     yError = VCG(2) - (RefPose1(3) + RefPose2(3) + ObstaclePose1(3) + ObstaclePose2(3))/4;       
    %     set(handles.yError,'string',num2str(yError));
    %         
    %     %航向角偏差
    %     HeadingAngelError = Yaw-RefPoseTheta;
    %     set(handles.HeadingAngelError,'string',num2str(HeadingAngelError));
    % 
    %     %计算泊车评分
    %     acc_score = acc_Assessment(xError,yError,HeadingAngelError);

        %显示泊车时间与评分
        set(handles.Time,'string',[num2str(parkingtime), ' s']);
    %     scoreDsp= findobj(0, 'tag', 'score');
    %     set(scoreDsp,'string',num2str(score));
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
    
function [xmin, xmax] = inorder(xmin, xmax)
    if xmin > xmax
        t = xmax;
        xmax = xmin;
        xmin = t;
    elseif xmin == xmax
        xmax = xmin + 1;
    end
    

% --- Executes on button press in push_paraset.
function push_paraset_Callback(hObject, eventdata, handles)
% hObject    handle to push_paraset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Xlim & Ylim for trajectory display    
global tr_xlim
global tr_ylim
    
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

if get(handles.push_reset, 'value')
    stop = 1;
    flag_loop2 = 0;
    PubArrayText = sprintf('%s\n','');
    set(handles.ui_paraset, 'Visible', 1);
    set(handles.uipanel1, 'Visible', 0);
    set(handles.push_start,'String','开始泊车');
    set(handles.Time, 'String', '');
    set(handles.Notice,'String',PubArrayText);
end
