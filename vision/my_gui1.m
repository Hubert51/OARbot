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

% Last Modified by GUIDE v2.5 05-Aug-2019 11:37:35

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
% axis off
% axis image

% imshow(handles.X{init_index},[]);
handles.index = init_index;

handles.timer = timer(...
    'ExecutionMode', 'fixedRate', ...       % Run timer repeatedly
    'Period', 1, ...                        % Initial period is 1 sec.
    'TimerFcn', {@update_display,hObject}); % Specify callback function

% handles.t = timer('BusyMode', 'queue', 'ExecutionMode',...
% 'fixedRate', 'Period', 4.0);
% set(handles.t, 'TimerFcn', @(hObject, event) funkcija(event, handles));
% disp(3)
% set(handles.t, 'TimerFcn', @(hObject, event) my_gui1('funkcija',event,guidata(hObject)));


% Update handles structure
guidata(hObject, handles);


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
%     
    hObject
    eventdata
    handles
    start(handles.timer);

    imgSub = rossubscriber('/camera/color/image_raw');
    imgMsg = receive(imgSub);
    if 1
        img = readImage(imgMsg);
        handles.output = hObject;
        handles.index = handles.index + 1;
        imshow(img,[]);
        guidata(hObject, handles);
    end

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


disp(1)
hObject.TimerFcn{1}
handles = guidata(hfigure);
handles.output;
% Z = get(handles.surf,'ZData');
% Z = Z + 0.1*randn(size(Z));
% set(handles.surf,'ZData',Z);
% END USER CODE
imgSub = rossubscriber('/camera/color/image_raw');
joySub = rossubscriber('/joy');

%handles.figure1
while 1
    if strcmp(get(handles.timer, 'Running'), 'on')
        stop(handles.timer);
    end
    % img = get(handles.figure1, 'Figure');
    imgMsg = receive(imgSub);

    img = readImage(imgMsg);
    handles.output = hObject;
    % ClearImagesFromAxes(handles.axes1)
    
    joyMsg = receive(joySub);
    joyMsg.Axes
    
    axes(handles.axes1)
    imshow(img)
    drawnow;
    
    pause(0.1)

% set(handles, 'figure1' , [img])
    % guidata(eventdata, hObject);
end
