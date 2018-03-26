function varargout = Matlab_Gui(varargin)
% MATLAB_GUI MATLAB code for Matlab_Gui.fig
%      MATLAB_GUI, by itself, creates a new MATLAB_GUI or raises the existing
%      singleton*.
%
%      H = MATLAB_GUI returns the handle to a new MATLAB_GUI or the handle to
%      the existing singleton*.
%
%      MATLAB_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MATLAB_GUI.M with the given input arguments.
%
%      MATLAB_GUI('Property','Value',...) creates a new MATLAB_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Matlab_Gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Matlab_Gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Matlab_Gui

% Last Modified by GUIDE v2.5 23-Mar-2018 12:41:59

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Matlab_Gui_OpeningFcn, ...
                   'gui_OutputFcn',  @Matlab_Gui_OutputFcn, ...
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


% --- Executes just before Matlab_Gui is made visible.
function Matlab_Gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Matlab_Gui (see VARARGIN)

% Choose default command line output for Matlab_Gui
global connexion_state ; % indicate if the robot si connected

rosshutdown; 
connexion_state =0; 
handles.output = hObject;
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Matlab_Gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Matlab_Gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
axes(handles.axes1);
imshow('logo.png');
axes(handles.axes2);
imshow('Matlab_Logo.png');


% --- Executes on button press in connectionButton.
function connectionButton_Callback(hObject, eventdata, handles)
set(handles.connectionPanel,'visible','on')
set(handles.commandPanel,'visible','off')
set(handles.hwPanel,'visible','off')
set(handles.logsPanel,'visible','off')
set(handles.plotPanel,'visible','off')

% --- Executes on button press in commandButton.
function commandButton_Callback(hObject, eventdata, handles)
% hObject    handle to commandButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.connectionPanel,'visible','off')
set(handles.commandPanel,'visible','on')
set(handles.hwPanel,'visible','off')
set(handles.logsPanel,'visible','off')
set(handles.plotPanel,'visible','off')

% --- Executes on button press in plotButton.
function plotButton_Callback(hObject, eventdata, handles)

% hObject    handle to plotButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.connectionPanel,'visible','off')
set(handles.commandPanel,'visible','off')
set(handles.hwPanel,'visible','on')
set(handles.logsPanel,'visible','off')
set(handles.plotPanel,'visible','on')
%..........................................%




% --- Executes on button press in logsButton.
function logsButton_Callback(hObject, eventdata, handles)
global logs; 
% hObject    handle to logsButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.connectionPanel,'visible','off')
set(handles.commandPanel,'visible','off')
set(handles.hwPanel,'visible','off')
set(handles.logsPanel,'visible','on')
set(handles.plotPanel,'visible','off')
set(handles.editlogs,'string',logs)


% --- Executes on button press in hwButton.
function hwButton_Callback(hObject, eventdata, handles)
global hw_status;
global logs; 
% hObject    handle to hwButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.connectionPanel,'visible','off')
set(handles.commandPanel,'visible','off')
set(handles.hwPanel,'visible','on')
set(handles.logsPanel,'visible','off')
set(handles.plotPanel,'visible','off')

 hw_status_msg=receive(hw_status,10); 
 logs=logs+ newline+'Get hardware status'
 set(handles.edit21,'string', hw_status_msg.RpiTemperature);
 set(handles.edit22,'string', hw_status_msg.CalibrationNeeded);
 %set(handles.edit20,'logical', hw_status_msg.CalibrationInProgress);
 %set(handles.edit23,'string', hw_status_msg.Temperatures);
 %set(handles.edit24,'string', hw_status_msg.HardwareErrors);
 set(handles.edit25,'string', hw_status_msg.Voltages);
% --- Executes on key press with focus on connectionButton and none of its controls.


% --- Executes during object creation, after setting all properties.
function commandButton_CreateFcn(hObject, eventdata, handles)
% hObject    handle to commandButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
function connectionButton_CreateFcn(hObject, eventdata, handles)
% hObject    handle to commandButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


function edit1_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in connect
function connect_Callback(hObject, eventdata, handles)
global state; 
global connexion_state ;
global hw_status;
global learning_mode_client;
 global theor_trajectory;
%global learning_mode_req ;
global new_calibration ; 
global new_calibration_msg; 
% global activate_motors_client; 
% global activate_motors_msg; 
global calibrate_motors_client; 
global calibrate_motors_msg; 
%global move_joint_msg;
global logs; 
global jointState; 
logs='';
state=0; 
if (connexion_state  == 0)
    Robot_ip_adress = get(handles.edit1,'String'); % get robot ip adress 
    Robot_ip_adress = strcat('http://',Robot_ip_adress,':11311');
    Computer_ip_adress = get(handles.edit2,'String');% get computer ip adress
    setenv('ROS_MASTER_URI',Robot_ip_adress); % set  ros matser URI to specifie the ros master location 
    setenv('ROS_IP',Computer_ip_adress); % set environment variables ROS_IP( network address of a ROS Node ), It's must be ip adress of the pc where matlab is installed 
    rosinit; % initialize ROS
    %pause(2);
    logs="connected to niryo one"
    connexion_state =1; 
    % hardware status
    hw_status=rossubscriber('/niryo_one/hardware_status'); %create a subscriber for  hardware statuts topic  
    % learning mode 
    learning_mode_client=rossvcclient('/niryo_one/activate_learning_mode');% create a ros service client for activate learning mode 
    learning_mode_req = rosmessage(learning_mode_client); % create message for  learning_mode_client
    disp('..............create learning mode message.......');
    logs=logs+ newline+  "create learning mode message.";
   %set(handles.editlogs,'string','..............create learning mode message.......');
    % calibrate robot% 
    new_calibration=rossvcclient('/niryo_one/request_new_calibration');
    new_calibration_msg= rosmessage(new_calibration);
    disp('..............create request new calibration message .......');
    logs=logs +newline +"create request new calibration message .";
    
    % calibrate motors%
    calibrate_motors_client=rossvcclient('/niryo_one/calibrate_motors');
    calibrate_motors_msg=rosmessage(calibrate_motors_client);
    disp('create calibrate maoters message');
   logs=logs +newline +'create calibrate motors message ';
   
   % get trajectory  
 theor_trajectory=rossubscriber('/niryo_one_follow_joint_trajectory_controller/follow_joint_trajectory/goal') ;
   
   % joint state 
   jointState=rossubscriber('/joint_states')
else 
    logs=logs+ newline+ "you are already connected to your robot."
     disp('.............you are already connected to your robot ...................')

end 



% --- Executes on button press in disconnect.
function disconnect_Callback(hObject, eventdata, handles)
global connexion_state ; 
global logs; 
rosshutdown;
 connexion_state =0;
  logs=logs+ newline+ "disconnect form niryo one"



function joint3_Callback(hObject, eventdata, handles)
% hObject    handle to joint3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function joint3_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function joint1_Callback(hObject, eventdata, handles)
% hObject    handle to joint1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

function joint1_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function joint2_Callback(hObject, eventdata, handles)

function joint2_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function joint4_Callback(hObject, eventdata, handles)

function joint4_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function joint5_Callback(hObject, eventdata, handles)
% hObject    handle to joint5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)




% --- Executes during object creation, after setting all properties.
function joint5_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function joint6_Callback(hObject, eventdata, handles)
% hObject    handle to joint6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


function joint6_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in moveButton.
function moveButton_Callback(hObject, eventdata, handles)
% global move_joint_client ;
global move_joint_msg;
global logs; 

global joint1;
global joint2;
global joint3;
global joint4;
global joint5;
global joint6;

 move_joint_client=rossvcclient('/niryo_one_matlab');
 move_joint_msg=rosmessage(move_joint_client);

%  % Move Joints 

joint1=str2double(get(handles.joint1,'String'));
joint2=str2double(get(handles.joint2,'String'));
joint3=str2double(get(handles.joint3,'String'));
joint4=str2double(get(handles.joint4,'String'));
joint5=str2double(get(handles.joint5,'String'));
joint6=str2double(get(handles.joint6,'String'));
move_joint_msg.Cmd.CmdType=1; 
move_joint_msg.Cmd.Joints=[joint1,joint2,joint3,joint4,joint5,joint6];
tic; 
rep=call(move_joint_client,move_joint_msg)
toc;
 logs=logs+ newline+ rep.Message; 
 
 
  function real()
global jointState; 
global state; 
global logs ;

j=1
while(state==0) 
jointStateMsg(j)=receive(jointState)
j=j+1
end 
state=1; 




      
     
  function function2() 
 global data ; 
 global logs; 
 global theor_trajectory;
 data=receive(theor_trajectory);
 logs= logs+newline+"get a new trajectory";
     
    




 



     
     
     % --- Executes on button press in learningModeButton.
function learningModeButton_Callback(hObject, eventdata, handles)
global learning_mode_client;
global learning_mode_req ;
global logs; 

 if (learning_mode_req.Value==1)
     learning_mode_req.Value=0;
     learning_mode_resp = call(learning_mode_client,learning_mode_req,'Timeout',3);
     disp(learning_mode_resp.Message);
     logs=logs+ newline+ learning_mode_resp.Message
 else 
     learning_mode_req.Value=1;
     learning_mode_resp = call(learning_mode_client,learning_mode_req,'Timeout',3);
      disp(learning_mode_resp.Message);
       logs=logs+ newline+ learning_mode_resp.Message;
 end

% --- Executes on button press in motorbutton.
function motorbutton_Callback(hObject, eventdata, handles)
%global new_calibration_resp ;
 global calibrate_motors_msg;
global calibrate_motors_client;
global logs; 

 motor_calibration_resp = call(calibrate_motors_client,calibrate_motors_msg);
 disp(new_calibration_resp.Message);
 logs=logs+ newline + motor_calibration_resp.Message;
 


% --- Executes on button press in newCalibrationButton.
function newCalibrationButton_Callback(hObject, eventdata, handles)
 global new_calibration ; 
global new_calibration_msg ;
global logs; 

 new_calibration_resp = call(new_calibration,new_calibration_msg);
 disp('.......new calibration requested.........')

 logs=logs+ newline+new_calibration_resp.Message;
 %disp(new_calibration_resp.Message)



function edit20_Callback(hObject, eventdata, handles)
% hObject    handle to edit20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit20 as text
%        str2double(get(hObject,'String')) returns contents of edit20 as a double


% --- Executes during object creation, after setting all properties.
function edit20_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit21_Callback(hObject, eventdata, handles)
% hObject    handle to edit21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit21 as text
%        str2double(get(hObject,'String')) returns contents of edit21 as a double


% --- Executes during object creation, after setting all properties.
function edit21_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit22_Callback(hObject, eventdata, handles)
% hObject    handle to edit22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit22 as text
%        str2double(get(hObject,'String')) returns contents of edit22 as a double


% --- Executes during object creation, after setting all properties.
function edit22_CreateFcn(hObject, ~, handles)
% hObject    handle to edit22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit23_Callback(hObject, eventdata, handles)
% hObject    handle to edit23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit23 as text
%        str2double(get(hObject,'String')) returns contents of edit23 as a double


% --- Executes during object creation, after setting all properties.
function edit23_CreateFcn(hObject, ~, handles)
% hObject    handle to edit23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit24_Callback(hObject, eventdata, handles)
% hObject    handle to edit24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit24 as text
%        str2double(get(hObject,'String')) returns contents of edit24 as a double


% --- Executes during object creation, after setting all properties.
function edit24_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit25_Callback(hObject, ~, handles)
% hObject    handle to edit25 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit25 as text
%        str2double(get(hObject,'String')) returns contents of edit25 as a double


% --- Executes during object creation, after setting all properties.
function edit25_CreateFcn(hObject, ~, ~)
% hObject    handle to edit25 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in getHwButton.
function getHwButton_Callback(~, eventdata, handles)
% hObject    handle to getHwButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)




function edit26_Callback(hObject, eventdata, handles)
% hObject    handle to edit20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit20 as text
%        str2double(get(hObject,'String')) returns contents of edit20 as a double


% --- Executes during object creation, after setting all properties.
function edit26_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit27_Callback(hObject, eventdata, handles)
% hObject    handle to edit21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit21 as text
%        str2double(get(hObject,'String')) returns contents of edit21 as a double


% --- Executes during object creation, after setting all properties.
function edit27_CreateFcn(hObject, ~, handles)
% hObject    handle to edit21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit28_Callback(hObject, eventdata, handles)
% hObject    handle to edit22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit22 as text
%        str2double(get(hObject,'String')) returns contents of edit22 as a double


% --- Executes during object creation, after setting all properties.
function edit28_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit29_Callback(hObject, eventdata, ~)
% hObject    handle to edit23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit23 as text
%        str2double(get(hObject,'String')) returns contents of edit23 as a double


% --- Executes during object creation, after setting all properties.
function edit29_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit30_Callback(hObject, eventdata, handles)
% hObject    handle to edit30 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit30 as text
%        str2double(get(hObject,'String')) returns contents of edit30 as a double


% --- Executes during object creation, after setting all properties.
function edit30_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit30 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit31_Callback(~, eventdata, handles)
% hObject    handle to edit31 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit31 as text
%        str2double(get(hObject,'String')) returns contents of edit31 as a double


% --- Executes during object creation, after setting all properties.
function edit31_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit31 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in getHwButton.
function pushbutton24_Callback(hObject, eventdata, handles)
% hObject    handle to getHwButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function editlogs_Callback(hObject, eventdata, handles)
% hObject    handle to editlogs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editlogs as text
%        str2double(get(hObject,'String')) returns contents of editlogs as a double


% --- Executes during object creation, after setting all properties.
function editlogs_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editlogs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit41_Callback(~, eventdata, handles)
% hObject    handle to edit41 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit41 as text
%        str2double(get(hObject,'String')) returns contents of edit41 as a double


% --- Executes during object creation, after setting all properties.
function edit41_CreateFcn(hObject, eventdata, ~)
% hObject    handle to edit41 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit42_Callback(~, eventdata, handles)
% hObject    handle to edit42 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit42 as text
%        str2double(get(hObject,'String')) returns contents of edit42 as a double


% --- Executes during object creation, after setting all properties.
function edit42_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit42 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit43_Callback(hObject, eventdata, handles)
% hObject    handle to edit43 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit43 as text
%        str2double(get(hObject,'String')) returns contents of edit43 as a double


% --- Executes during object creation, after setting all properties.
function edit43_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit43 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit44_Callback(hObject, eventdata, handles)
% hObject    handle to edit44 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit44 as text
%        str2double(get(hObject,'String')) returns contents of edit44 as a double


% --- Executes during object creation, after setting all properties.
function edit44_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit44 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit45_Callback(hObject, eventdata, handles)
% hObject    handle to edit45 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit45 as text
%        str2double(get(hObject,'String')) returns contents of edit45 as a double


% --- Executes during object creation, after setting all properties.
function edit45_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit45 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit46_Callback(hObject, eventdata, handles)
% hObject    handle to edit46 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit46 as text
%        str2double(get(hObject,'String')) returns contents of edit46 as a double


% --- Executes during object creation, after setting all properties.
function edit46_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit46 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




    
    
    
% --- Executes when selected object is changed in uibuttongroup3.
function uibuttongroup3_SelectionChangedFcn(hObject, eventdata, handles)
global data ; 
global x;
global y; 
global joint_position; 

% hObject    handle to the selected object in uibuttongroup3 
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
array_time=[data.Goal.Trajectory.Points(:,1).TimeFromStart];
 x=[array_time.Nsec]/10^9+[array_time.Sec];
 joint_position=[data.Goal.Trajectory.Points(:,1).Positions];
switch get(eventdata.NewValue,'Tag')   % Get Tag of selected object
    case 'radiobutton7'
      %execute this code when fontsize08_radiobutton is selected
      y=joint_position(1,:);
      axes(handles.axestheorique);
      plot(x,y);
      
    case 'radiobutton8'
      %execute this code when fontsize12_radiobutton is selected
      y=joint_position(2,:);
       axes(handles.axestheorique);
        plot(x,y);
        
    case 'radiobutton9'
      %execute this
      y=joint_position(3,:);
       axes(handles.axestheorique);
        plot(x,y);
       
case 'radiobutton10'
      %execute this code when fontsize08_radiobutton is selected
      y=joint_position(4,:);
      axes(handles.axestheorique);
        plot(x,y);
        

    case 'radiobutton11'
      %execute this code when fontsize12_radiobutton is selected
      y=joint_position(5,:);
       axes(handles.axestheorique);
        plot(x,y);
     

    case 'radiobutton12'
      %execute this
      y=joint_position(6,:);
       axes(handles.axestheorique);
        plot(x,y);
   
    otherwise
       % Code for when there is no match.

end





% --- Executes during object creation, after setting all properties.
function commandPanel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to commandPanel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes get theroical trajectory 
function pushbutton26_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton26 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
function2(); 


% --- Executes on button press in pushbutton27.
function pushbutton27_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton27 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global x; 
global joint_position;
global logs ; 
global fileId; 

fileId=fopen('C:\Users\SARRA\Desktop\matlabgui/data.txt','w')
temps=x';
fprintf(fileId,'Applixation Logs\r\n') 
fprintf(fileId,'%s\r\n',logs);
fprintf(fileId,'%s\r\n','--------------------------------------------')
fprintf(fileId,'%s\r\n','trajectory data')
fprintf(fileId,'%s\r\n','--------------------------------------------')
joint1=(joint_position(1,:))';
joint2=(joint_position(2,:))';
joint3=(joint_position(3,:))';
joint4=(joint_position(4,:))';
joint5=(joint_position(5,:))';
joint6=(joint_position(6,:))';
Mo=['temps',' joint 1',' joint 2 ',' joint 3',' joint 4',' joint 5 ',' joint 6']
T=[temps,joint1,joint2,joint3 ,joint4, joint5 ,joint6]
%fprintf(fileId,'temps \t  joint 1 \t joint 2  \t  joint 3  \t  joint 4 \t  joint 5 \t  joint 6 \n')
%fprintf(fileId,'%2.4f\t %2.4f \t %2.4f\t %2.4f\t %2.4f\t %2.4f\n',T)  
fprintf(fileId,'%5s %5s %5s,%5s %5s %5s %5s\r\n',Mo); 
fprintf(fileId,'\r\n');
fprintf(fileId,'%s\r\n','--------------------------------------------')
fprintf(fileId,'\n');
for ii=1:size(T,1)
    fprintf(fileId,'%5.5f %5.5f %5.5f %5.5f %5.5f %5.5f %5.5f\r\n',T(ii,:));
end 
%fclose(fileId); 


% --- Executes on button press in pushbutton29.
function pushbutton29_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton29 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
real()

