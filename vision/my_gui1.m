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
init_index = 3;
handles.files = dir('train_data/3objects/img');
for i = init_index : 50
    handles.X{i} = imread(fullfile('train_data/3objects/img',handles.files(i).name));   
end
axes(handles.axes1)
imshow(handles.X{init_index})
drawnow
% axis off
% axis image

% imshow(handles.X{init_index},[]);
handles.index = init_index;
handles.update_flag = 1;
handles.listbox1.UserData.cur_state = 1;

handles.timer = timer(...
    'ExecutionMode', 'fixedRate', ...       % Run timer repeatedly
    'Period', 0.001, ...                        % Initial period is 1 sec.
    'TimerFcn', {@update_display,hObject}); % Specify callback function
handles.timer2 = timer(...
    'ExecutionMode', 'fixedRate', ...       % Run timer repeatedly
    'Period', 0.01, ...                        % Initial period is 1 sec.
    'TimerFcn', {@joystick,hObject}); % Specify callback function
% handles.t = timer('BusyMode', 'queue', 'ExecutionMode',...
% 'fixedRate', 'Period', 4.0);
% set(handles.t, 'TimerFcn', @(hObject, event) funkcija(event, handles));
% disp(3)
% set(handles.t, 'TimerFcn', @(hObject, event) my_gui1('funkcija',event,guidata(hObject)));
handles.locations.deliever.table = [-0.3519; -0.4546; 0.2916; -0.5186; 0.5133; 0.4944; -0.4723];
handles.locations.initial.table = [-0.0515; -0.4863; 0.1694; 0.6935; -0.0466; -0.0242; 0.7185];
handles.grab_ori = [0.7057; 0.0761; -0.0410; 0.7032];
handles.process_food = ["Deliever"; "Move into microwave"];
handles.cur_location = "";
robotArm = RobotRaconteur.Connect('tcp://localhost:4567/KinovaJointServer/Kinova');
robotArm.closeFinger([0.0; 0.0; 0.0])
% Update handles structure

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

% if strcmp(get(handles.timer2, 'Running'), 'off')
%     start(handles.timer2);
% end



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
    
 function funkcija(hObject, EventData)
    hObject
    EventData
    disp(1)
    imgSub = rossubscriber('/camera/color/image_raw');
    imgMsg = receive(imgSub);
    if 1
        img = readImage(imgMsg);
        hObject.output = EventData;
        cla
        imshow(img,[]);
        disp(2)
        guidata(EventData, hObject);
    end
    
% START USER CODE
function update_display(hObject,eventdata,hfigure)
% Timer timer1 callback, called each time timer iterates.
% Gets surface Z data, adds noise, and writes it back to surface object.
%%
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
handles = guidata(hfigure);

imgSub = rossubscriber('/camera/color/image_raw');
joySub = rossubscriber('/joy');
joy = RobotRaconteur.Connect('tcp://localhost:7890/KinovaUIServer/UI');

%handles.figure1
while 1
    handles = guidata(hfigure);

    % img = get(handles.figure1, 'Figure');
    imgMsg = receive(imgSub);

    img = readImage(imgMsg);
    % handles.output = hObject;
    % ClearImagesFromAxes(handles.axes1)
    
%     try
%         joystick = receive(joySub, 0.1);
% %         joystick.Axes
%         if joystick.Axes(2) < -0.5
%             if handles.listbox1.Value == length(handles.listbox1.String)
%                 handles.listbox1.Value = 0;
%             else
%                 handles.listbox1.Value = handles.listbox1.Value + 1;
%             end
%         end
%     catch
%        disp('no joystick input')
%     end
    
    user_input = joy.getInput();
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
    elseif user_input == 2
        % handles.listbox1.String = {'123'; '456'; '789'};
        set(handles.figure1, 'SelectionType', 'open');
        my_listbox1_Callback(hfigure);
        handles = guidata(hfigure);
    elseif user_input == -2
        load_listbox('', handles);
    end
    
    if handles.update_flag == 1
        axes(handles.axes1)
        cla reset
        imshow(img)
        drawnow;
    end
    pause(0.05)
    if strcmp(get(handles.timer, 'Running'), 'on')
        stop(handles.timer);
    end

end


% --- Executes on selection change in listbox1.
function [handles] = my_listbox1_Callback(hObject, eventdata)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox1
handles = guidata(hObject);
if handles.listbox1.UserData.cur_state == 1
    location = handles.listbox1.String(handles.listbox1.Value);
    rcnn = init_sys(hObject);
    handles = guidata(hObject);
    handles.rcnn = rcnn;
    handles.listbox1.UserData.cur_state = 2;
    objects = ["water", "food"];
    handles.text2.String = 'Detecting the Food';
    handles.listbox1.String = [];
    
    detect_result = objDetection(handles.rcnn, objects, 8);
    
    handles.text2.String = 'Detected the food, please choose the food you want';
    obj_name = [];
    for i=1:length(detect_result)
        if  detect_result(i).bbox ~= [0 0 0 0]
            obj_name = [obj_name; detect_result(i).name];
        end
    end
        
    obj_name = ["Re-detect"; obj_name];
    handles.listbox1.String = obj_name;
    handles.update_flag = 0;
    handles.detect_result = detect_result;
    guidata(hObject, handles);
elseif handles.listbox1.UserData.cur_state == 2
    handles.listbox1.UserData.cur_state = 3;
    obj_name = handles.listbox1.String(handles.listbox1.Value);
    if strcmp(convertCharsToStrings(obj_name{1}), "Re-detect")
        handles.listbox1.UserData.cur_state = 1;
        guidata(hObject, handles);
        my_listbox1_Callback(hObject)
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
        objLocalization(handles.rcnn, [convertCharsToStrings(obj_name{1})] , handles.detect_result(index).bbox);
        handles.listbox1.String = handles.process_food;
        handles.text2.String = 'Choose an option for the food';
        handles.listbox1.Value = 1;
    end

elseif handles.listbox1.UserData.cur_state == 3
    option = lower(handles.listbox1.String(handles.listbox1.Value));
    if contains(convertCharsToStrings(option{1}), 'deliever')
        deliever(hObject)
    elseif contains(option, 'microwave')
        disp("Need Implement")
    end
end

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

% --- Executes during object creation, after setting all properties.
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

handles = guidata(hfigure);
disp('joystick')

joySub = rossubscriber('/joy');

if 1
    disp('joystick')
    joystick = receive(joySub)
%     if strcmp(get(handles.timer, 'Running'), 'on')
%         stop(handles.timer);
%     end

end

% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% START USER CODE
% Necessary to provide this function to prevent timer callback
% from causing an error after GUI code stops executing.
% Before exiting, if the timer is running, stop it.

disp('stop')
% Destroy timer
delete(handles.timer)
% END USER CODE

% Hint: delete(hObject) closes the figure
delete(hObject);
