%% important data
% joy mapping
% up: joy.Axes(2) == 1
% down joy.Axes(2) == -1
% left: joy.Axes(1) == 1
% right: joy.Axes(1) == -1

% GUI example
% cd(setupExample('matlab/InteractiveListBoxGUIDEExample'));guide('lbox2.fig')

%% my code

function varargout = my_gui1(varargin)
% MY_GUI1 MATLAB code for my_gui1.fig
%      MY_GUI1, by itself, creates a new MY_GUI1 or raises the existing
%      singleton*.
%
%      H = MY_GUI1 returns the handle to a new MY_GUI1 or the handle to
%      the existing singleton*.
%
%      MY_GUI1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MY_GUI1.M with the given input arguments.
%
%      MY_GUI1('Property','Value',...) creates a new MY_GUI1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before my_gui1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to my_gui1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help my_gui1

% Last Modified by GUIDE v2.5 08-Aug-2019 08:39:05

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @my_gui1_OpeningFcn, ...
                   'gui_OutputFcn',  @my_gui1_OutputFcn, ...
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

% --- Executes just before my_gui1 is made visible.
function my_gui1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to my_gui1 (see VARARGIN)

% Choose default command line output for my_gui1
handles.output = hObject;
init_index = 1;
handles.files = dir('train_data/3objects/img');
files = ["boundbox_food.png", "boundbox_water.png", "boundbox_cola.png"];
% for i = init_index : 50
%     handles.X{i} = imread(fullfile('train_data/3objects/img',handles.files(i).name));   
% end
for i = 1:length(files)
    handles.X{i} = imread(files(i));   
end
axes(handles.axes1)
%imshow(handles.X{init_index})
% imshow('boundbox1.png')
% drawnow
% axis off
% axis image

% imshow(handles.X{init_index},[]);
handles.index = init_index;
handles.update_flag = 1;

handles.timer = timer(...
    'ExecutionMode', 'fixedRate', ...       % Run timer repeatedly
    'Period', 0.1, ...                        % Initial period is 1 sec.
    'StartDelay', 0.5,...
    'TimerFcn', {@update_display,hObject}); % Specify callback function
handles.timer2 = timer(...
    'ExecutionMode', 'fixedRate', ...       % Run timer repeatedly
    'Period', 0.1, ...                        % Initial period is 1 sec.
    'TimerFcn', {@joystick,hObject}); % Specify callback function
% handles.t = timer('BusyMode', 'queue', 'ExecutionMode',...
% 'fixedRate', 'Period', 4.0);
% set(handles.t, 'TimerFcn', @(hObject, event) funkcija(event, handles));
% disp(3)
% set(handles.t, 'TimerFcn', @(hObject, event) my_gui1('funkcija',event,guidata(hObject)));

%% define the UI global variable
% load the neural network model
load('model/rcnn_3objects.mat')
handles.rcnn = rcnn_food_water;
% the state of user.
handles.listbox1.UserData.cur_state = 0;
% the flag whether move forward or not.
handles.move_forward = 1;
handles.locations.deliever.table = [-0.3519; -0.4546; 0.2916; -0.5186; 0.5133; 0.4944; -0.4723];
handles.locations.initial.table = [-0.0515; -0.4863; 0.1694; 0.6935; -0.0466; -0.0242; 0.7185];
handles.grab_ori = [0.7057; 0.0761; -0.0410; 0.7032];
handles.process_food = ["Re-grab"; "Deliever"; "Move into microwave"];
handles.cur_location = "";
handles.selected_food = ""; % food selected to grab.
handles.joy = RobotRaconteur.Connect('tcp://localhost:7890/KinovaUIServer/UI');
handles.camera = RobotRaconteur.Connect('tcp://192.168.1.117:2345/KinovaCameraServer/Camera');
robotArm = RobotRaconteur.Connect('tcp://192.168.1.117:4567/KinovaJointServer/Kinova');
robotArm.closeFinger([0.0; 0.0; 0.0])

%% Update handles structure
if nargin == 3,
    initial_dir = pwd;
elseif nargin > 4
    if strcmpi(varargin{1},'dir')
        if exist(varargin{2},'dir')
            initial_dir = varargin{2};
        else
            errordlg('Input argument must be a valid directory','Input Argument Error!')
            return
        end
    else
        errordlg('Unrecognized input argument','Input Argument Error!');
        return;
    end
end
% Populate the listbox
load_listbox(initial_dir,handles)
guidata(hObject, handles);
% 
if strcmp(get(handles.timer, 'Running'), 'off')
    start(handles.timer);
end

if strcmp(get(handles.timer2, 'Running'), 'off')
    start(handles.timer2);
end



% UIWAIT makes my_gui1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = my_gui1_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
%     handles.output = hObject;
%     handles.index = handles.index + 1;
%     imshow(handles.X{handles.index},[]);
%     guidata(hObject, handles);
%     imgSub = rossubscriber('/camera/color/image_raw');
%     imgMsg = receive(imgSub);
%     if 1
%         img = readImage(imgMsg);
%         handles.output = hObject;
%         handles.index = handles.index + 1;
%         imshow(img,[]);
%         % guidata(hObject, handles);
%     end
% function update_display(hObject, eventdata, handles)
%     imgSub = rossubscriber('/camera/color/image_raw');
%     imgMsg = receive(imgSub);
%     if 1
%         img = readImage(imgMsg);
%         handles.output = hObject;
%         handles.index = handles.index + 1;
%         imshow(img,[]);
%         guidata(hObject, handles);
%     end
    

    
% START USER CODE
function update_display(hObject,eventdata,hfigure)
% disp('img')
% handles = guidata(hfigure);
% img = get_img_from_rr();
% axes(handles.axes1)
% cla reset
% imshow(img)
% drawnow;
% if handles.update_flag == 1
%     imgSub = rossubscriber('/camera/color/image_raw');
%     imgMsg = receive(imgSub);
%     img = readImage(imgMsg);
%     axes(handles.axes1)
%     cla reset
%     imshow(img)
%     drawnow;
% end


% --- Executes on selection change in listbox1.
function [handles] = my_listbox1_Callback(hObject, eventdata)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox1
handles = guidata(hObject);
robotArm = RobotRaconteur.Connect('tcp://192.168.1.117:4567/KinovaJointServer/Kinova');

% state 0 will open the fridge and microwave
if handles.listbox1.UserData.cur_state == 0
    handles.listbox1.UserData.cur_state = 1;
    location = handles.listbox1.String(handles.listbox1.Value);
    init_sys(hObject);
    open_fridge(hObject);
    open_microwave(hObject);
    % move toward to the fridge
    %robotArm.moveBase([0.0; -0.15])
    %pause(4)
    %robotArm.moveBase([-0.0005; 0])
end

% position before the fridge to detect the food.
cur_pos = [-0.1961; 0.4980; -0.0298];
cur_ori = [0.0026; 0.7361; 0.6761; 0.0310];

% state 1 will detected food.
if handles.listbox1.UserData.cur_state == 1
    handles = guidata(hObject);
    location = handles.listbox1.String(handles.listbox1.Value);
    handles.listbox1.UserData.cur_state = 2;
    % the objects neural network will detect.
    objects = ["water", "food", "cola"];
    guidata(hObject, handles);
    
    robotArm.cartesian_pose_client(cur_pos, cur_ori, 0);

    handles = guidata(hObject);
    
    % detecting the food
    handles = detect_stage(handles);
    guidata(hObject, handles);
    
elseif handles.listbox1.UserData.cur_state == 2
    handles.listbox1.UserData.cur_state = 3;
    % the first time be this state

    if strcmp(handles.selected_food, "")
        % output is {'cola'}
        obj_name = handles.listbox1.String(handles.listbox1.Value); 
        obj_name = convertCharsToStrings(obj_name{1});
    else
        obj_name = handles.selected_food;
        robotArm.cartesian_pose_client(cur_pos, cur_ori, 0);
    end
    if strcmp(convertCharsToStrings(obj_name{1}), "Re-detect")
        handles.listbox1.UserData.cur_state = 2;
        handles = detect_stage(handles);
        guidata(hObject, handles);   
    else
        index = 0;
        for i=1:length(handles.detect_result)
            if handles.detect_result(i).name == obj_name
                index = i;
                break;
            end
        end
        handles.text2.String = 'Localizing and grabbing the food';
        handles.listbox1.String = [];
        handles.update_flag = 1;     
        if handles.move_forward == 1
            % move toward to the fridge
            robotArm.moveBase([0.0; -0.15])
            pause(4)
            robotArm.moveBase([-0.0005; 0])
            handles.move_forward = 0;
        end
        objLocalization(handles.rcnn, [obj_name] , handles.detect_result(index).bbox);
        handles.selected_food = obj_name;
        disp(strcat("User chooses object ", handles.selected_food));
        % deliver or move to microwave or re-grab
        handles.listbox1.String = handles.process_food;
        handles.text2.String = 'Choose an option for the food';
        handles.listbox1.Value = 1;
        guidata(hObject, handles);
    end
    

elseif handles.listbox1.UserData.cur_state == 3
    handles.listbox1.UserData.cur_state = 4;
    option = lower(handles.listbox1.String(handles.listbox1.Value));
    if contains(convertCharsToStrings(option{1}), 'deliver')
        deliver(hObject)
        obj_name = ["Deliver"];
        handles.listbox1.String = obj_name;
        guidata(hObject, handles);
    elseif contains(option, 'microwave')
        handles.listbox1.UserData.cur_state = 3;
        before_mv = [-0.203; 0.4661; 0.3216];
        robotArm.cartesian_pose_client(before_mv, cur_ori, 0);
        pause(0.1)
        robotArm.cartesian_pose_client([0.0; 0.29; 0.0], robotArm.getOri(), 1);
        robotArm.closeFinger([0.0; 0.0; 0.0]);
        robotArm.cartesian_pose_client(before_mv, cur_ori, 0);
        
        % wait for heat the food
        pause(5)
        grab_pos = before_mv + [0; 0.29; 0];
        robotArm.cartesian_pose_client(grab_pos, cur_ori, 0);
        robotArm.closeFinger([5500.0; 5500.0;5500.0]);
        robotArm.cartesian_pose_client(before_mv, cur_ori, 0);
        
    elseif contains(option, 'grab')
        handles.listbox1.UserData.cur_state = 2;
        robotArm.closeFinger([0.0; 0.0; 0.0])
        guidata(hObject, handles);
        my_listbox1_Callback(hObject)
    end
elseif handles.listbox1.UserData.cur_state == 4
    deliver(hObject)
    %pause(5)
    robotArm.closeFinger([0.0; 0.0; 0.0])
    robotArm.cartesian_pose_client([-0.2478; 0.4260; 0.3789], [0.3346; -0.6231; -0.6135; 0.3513], 0);
    pause(1)
    robotArm.cartesian_pose_client(cur_pos, cur_ori, 0);
end
guidata(hObject, handles);

% if strcmp(get(handles.figure1,'SelectionType'), 'open') 
%     handles.listbox1.String = {'obj1'; 'onj2'; 'obj3'};
%     guidata(hObject, handles)
%     disp(1)
% end


% Error in matlab.graphics.internal.figfile.FigFile/read>@(hObject,eventdata)my_gui1('listbox1_Callback',hObject,eventdata,guidata(hObject))
function listbox1_Callback(hObject, eventdata, handles)
% if strcmp(get(handles.figure1,'SelectionType'),'open')
%     index_selected = get(handles.listbox1,'Value');
%     file_list = get(handles.listbox1,'String');
%     filename = file_list{index_selected};
%     if  handles.is_dir(handles.sorted_index(index_selected))
%         cd (filename)
%         load_listbox(pwd,handles)
%     else
%         [path,name,ext] = fileparts(filename);
%         switch ext
%             case '.fig'
%                 guide (filename)
%             otherwise
%                 try
%                     open(filename)
%                 catch ex
%                     errordlg(...
%                       ex.getReport('basic'),'File Type Error','modal')
%                 end
%         end
%     end
% end
% function load_listbox(dir_path,handles)
% cd (dir_path)
% dir_struct = dir(dir_path);
% [sorted_names,sorted_index] = sortrows({dir_struct.name}');
% handles.file_names = sorted_names;
% handles.is_dir = [dir_struct.isdir];
% handles.sorted_index = sorted_index;
% guidata(handles.figure1,handles)
% set(handles.listbox1,'String',handles.file_names,...
% 	'Value',1)
% set(handles.text1,'String',pwd)

function load_listbox(dir_path,handles)
% cd (dir_path)
% dir_struct = dir(dir_path);
[sorted_names,sorted_index] = sortrows({'Choose location of robot'; 'On the table'; 'Before the Fridge'});
% TODO: modify from cell to list
handles.file_names = {'Choose location of robot'; 'On the table'; 'Before the Fridge'};
% handles.is_dir = [dir_struct.isdir];
% handles.sorted_index = sorted_index;
guidata(handles.figure1,handles)
set(handles.listbox1,'String',handles.file_names,...
	'Value',1)
% %set(handles.text1,'String',pwd)

% --- Executes during object creation,
% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% START USER CODE
% Necessary to provide this function to prevent timer callback
% from causing an error after GUI c after setting all properties.
function listbox1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1

% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function joystick(hObject,eventdata,hfigure)
% Timer timer1 callback, called each time timer iterates.
% Gets surface Z data, adds noise, and writes it back to surface object.
%
% handles = 
% 
%   struct with fields:
% 
%         figure1: [1×1 Figure]
%      popupmenu1: [1×1 UIControl]
%        listbox1: [1×1 UIControl]
%     pushbutton1: [1×1 UIControl]
%           axes1: [1×1 Axes]
%          output: [1×1 Figure]
%           files: [7788×1 struct]
%               X: {1×50 cell}
%           index: 3
%           timer: [1×1 timer]
%          timer2: [1×1 timer]

%% my code
% two methods to update the images:
% 1. use this function and while loop to get the img from ros
% 2. use timer function to update the img from ros

% hObject.TimerFcn{1};
% handles = guidata(hfigure);
% 
% imgSub = rossubscriber('/camera/color/image_raw');

% joySub = rossubscriber('/joy');

while 1
    handles = guidata(hfigure);
    % img = get(handles.figure1, 'Figure');
%     imgMsg = receive(imgSub);
%     img = readImage(imgMsg);
    user_input = handles.joy.getInput();
    if handles.listbox1.Value > length(handles.listbox1.String) 
        handles.listbox1.Value = 1;
    end
    if -1.5 < user_input && user_input < 1.5
        val_len = length(handles.listbox1.String);
        if handles.listbox1.Value == val_len && user_input==1
            handles.listbox1.Value = 1;
        elseif handles.listbox1.Value == 1 && user_input == -1
            handles.listbox1.Value = val_len;
        else
            handles.listbox1.Value = handles.listbox1.Value + int8(user_input);
        end
        % show user the objects detected by the camera. The names and bound
        % box will show in the UI
        if handles.listbox1.UserData.cur_state == 2 &&  int8(user_input) ~= 0
            if handles.listbox1.Value>1
                for i=1:length(handles.detect_result)
                    if strcmp(handles.detect_result(i).name, handles.listbox1.String(handles.listbox1.Value))
                        img = handles.detect_result(i).img;
                    end
                end
            else
                img = handles.cur_img;
            end
            axes(handles.axes1)
            cla reset
            imshow(img)
            drawnow;
        end
    elseif user_input == 2
        % handles.listbox1.String = {'123'; '456'; '789'};
            
        set(handles.figure1, 'SelectionType', 'open');
        my_listbox1_Callback(hfigure);
        handles = guidata(hfigure);
    elseif user_input == -2
        load_listbox('', handles);
    end
%     
%     if handles.update_flag == 1
%         axes(handles.axes1)
%         cla reset
%         imshow(img)
%         drawnow;
%     end
    % disp('in timer1')
    pause(0.1)
    if strcmp(get(handles.timer2, 'Running'), 'on')
        stop(handles.timer2);
    end

end


% --- Executes when user attempts to close figure1.
% function figure1_CloseRequestFcn(hObject, eventdata, handles)
% % hObject    handle to figure1 (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% 
% % START USER CODE
% % Necessary to provide this function to prevent timer callback
% % from causing an error after GUI code stops executing.
% % Before exiting, if the timer is running, stop it.
% 
% disp('stop')
% % Destroy timer
% delete(handles.timer)
% % END USER CODE
% 
% % Hint: delete(hObject) closes the figure
% delete(hObject);
