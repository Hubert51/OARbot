function varargout = my_gui(varargin)
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @untitled_OpeningFcn, ...
                   'gui_OutputFcn',  @untitled_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})    gui_State.gui_Callback = str2func(varargin{1});    end
if nargout    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else    gui_mainfcn(gui_State, varargin{:});    end

function untitled_OpeningFcn(hObject, eventdata, handles, varargin)
    handles.output = hObject;
    handles.files = dir(fullfile('home','ruijie','kinova_ws', 'src', 'vision', 'train_data', '3objects','img\*.png'));
    for i = 1 : length(handles.files)
        handles.X{i} = dicomread(fullfile('home','ruijie','kinova_ws', 'src', 'vision', 'train_data', '3objects','img',handles.files(i).name));   
    end
    imshow(handles.X{1},[]);
    handles.index = 1;
    Cek(hObject, eventdata, handles);
    guidata(hObject, handles);
end


function varargout = untitled_OutputFcn(hObject, eventdata, handles) 
    varargout{1} = handles.output;
end

function pushbutton1_Callback(hObject, eventdata, handles)
    handles.output = hObject;
    handles.index = handles.index - 1;
    Cek(hObject, eventdata, handles);
    imshow(handles.X{handles.index},[]);
    guidata(hObject, handles);
end


function pushbutton2_Callback(hObject, eventdata, handles)
    handles.output = hObject;
    handles.index = handles.index + 1;
    Cek(hObject, eventdata, handles);
    imshow(handles.X{handles.index},[]);
    guidata(hObject, handles);
end

    
function Cek(hObject, eventdata, handles)
    handles.output = hObject;
    n = length(handles.files);
    if handles.index > 1,  
        set(handles.pushbutton1,'enable','on');
    else
        set(handles.pushbutton1,'enable','off');
    end
    
    if handles.index < n, 
        set(handles.pushbutton2,'enable','on');
    else
        set(handles.pushbutton2,'enable','off'); 
    end
    guidata(hObject, handles);
    
    
end