
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

% Last Modified by GUIDE v2.5 30-Mar-2018 16:30:20

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
function Matlab_Gui_OpeningFcn(hObject, ~, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Matlab_Gui (see VARARGIN)

% Choose default command line output for Matlab_Gui
global connexion_state ; % indicate if the robot si connected
global logs ; 
logs='';
rosshutdown; 
connexion_state =0; 
handles.output = hObject;
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
set(handles.connect,'string','Connect to Niryo One')
set(handles.connectionButton,'BackgroundColor',[0.1,0.67,0.89],'ForegroundColor','white')
set(handles.plotButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
set(handles. logsButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
set(handles.hwButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
set(handles.commandButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
set(handles.connectionPanel,'visible','on')
set(handles.commandPanel,'visible','off')
set(handles.hwPanel,'visible','off')
set(handles.logsPanel,'visible','off')
set(handles.plotPanel,'visible','off')


    
% UIWAIT makes Matlab_Gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Matlab_Gui_OutputFcn(~, ~, handles) 
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
function connectionButton_Callback(~, ~, handles)
set(handles.connectionPanel,'visible','on')
set(handles.commandPanel,'visible','off')
set(handles.hwPanel,'visible','off')
set(handles.logsPanel,'visible','off')
set(handles.plotPanel,'visible','off')
set(handles.connectionButton,'BackgroundColor',[0.1,0.67,0.89],'ForegroundColor','white')
set(handles.commandButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
set(handles.plotButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
set(handles. logsButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
set(handles.hwButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])


% --- Executes on button press in commandButton.
function commandButton_Callback(hObject, ~, handles)
% hObject    handle to commandButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.connectionPanel,'visible','off')
set(handles.commandPanel,'visible','on')
set(handles.hwPanel,'visible','off')
set(handles.logsPanel,'visible','off')
set(handles.plotPanel,'visible','off')
set(handles.edit56,'visible','off')
set(handles.commandButton,'BackgroundColor',[0.1,0.67,0.89],'ForegroundColor','white')
set(handles.connectionButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
set(handles.plotButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
set(handles. logsButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
set(handles.hwButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])


% --- Executes on button press in plotButton.
function plotButton_Callback(~, eventdata, handles)

% hObject    handle to plotButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.connectionPanel,'visible','off')
set(handles.commandPanel,'visible','off')
set(handles.hwPanel,'visible','on')
set(handles.logsPanel,'visible','off')
set(handles.plotPanel,'visible','on')
set(handles.edit55,'visible','off')
set(handles.plotButton,'BackgroundColor',[0.1,0.67,0.89],'ForegroundColor','white')
set(handles.connectionButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
set(handles.commandButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
set(handles. logsButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
set(handles.hwButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
%..........................................%




% --- Executes on button press in logsButton.
function logsButton_Callback(hObject, ~, handles)
global logs; 
% hObject    handle to logsButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.connectionPanel,'visible','off')
set(handles.commandPanel,'visible','off')
set(handles.hwPanel,'visible','off')
set(handles.logsPanel,'visible','on')
set(handles.plotPanel,'visible','off')
set(handles.logsButton,'BackgroundColor',[0.1,0.67,0.89],'ForegroundColor','white')
set(handles.connectionButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
set(handles.commandButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
set(handles.plotButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
set(handles.hwButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
set(handles.editlogs,'string',logs)


% --- Executes on button press in hwButton.
function hwButton_Callback(hObject, eventdata, handles)
global hw_status;
global logs; 
global connexion_state; 
if connexion_state==0
   logs=logs+newline+"you should connect to you robot first"; 
   return;
else 
% hObject    handle to hwButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.connectionPanel,'visible','off')
set(handles.commandPanel,'visible','off')
set(handles.hwPanel,'visible','on')
set(handles.logsPanel,'visible','off')
set(handles.plotPanel,'visible','off')
set(handles.hwButton,'BackgroundColor',[0.1,0.67,0.89],'ForegroundColor','white')
set(handles.connectionButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
set(handles.commandButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
set(handles.plotButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
set(handles.logsButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])

 hw_status_msg=receive(hw_status,10); 
 logs=logs+ newline+'Get hardware status'
 set(handles.edit21,'string', hw_status_msg.RpiTemperature);
 set(handles.edit22,'string', hw_status_msg.CalibrationNeeded);
% set(handles.edit20,'string', hw_status_msg.CalibrationInProgress);
 set(handles.edit23,'string', hw_status_msg.Temperatures);
 set(handles.edit24,'string', hw_status_msg.HardwareErrors);
 set(handles.edit25,'string',hw_status_msg.Voltages);
% --- Executes on key press with focus on connectionButton and none of its controls.
end 






% --- Executes on button press in connect
function connect_Callback(hObject, eventdata, handles)

global state; 
global connexion_state ;
global hw_status;
global learning_mode_client;
 global theor_trajectory;
global learning_mode_req ;
global new_calibration ; 
global new_calibration_msg; 
 
global calibrate_motors_client; 
global calibrate_motors_msg; 
global Learning_mode_state; 

global logs; 
global jointState; 
set(handles.moveButton,'string','Move Joints ','BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
set(handles.disconnect,'string','Disconnect from Niryo One')
logs='';
state=0; 
set(handles.connect,'BackgroundColor',[0.1,0.67,0.89],'ForegroundColor','white')
if (connexion_state  == 0)
    Robot_ip_adress = get(handles.edit1,'String'); % get robot ip adress 
    Robot_ip_adress = strcat('http://',Robot_ip_adress,':11311');
    Computer_ip_adress = get(handles.edit2,'String');% get computer ip adress
    setenv('ROS_MASTER_URI',Robot_ip_adress); % set  ros matser URI to specifie the ros master location 
    setenv('ROS_IP',Computer_ip_adress); % set environment variables ROS_IP( network address of a ROS Node ), It's must be ip adress of the pc where matlab is installed 
try
    rosinit; % initialize ROS
catch e
    logs= e.message
    set(handles.connect,'string','connected ','BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
    return; 
end 
  
    %pause(2);
    logs="connected to niryo one";
    % hardware status
        try 
        hw_status=rossubscriber('/niryo_one/hardware_status'); %create a subscriber for  hardware statuts topic
        catch e 
            logs= e.message
            roshutdown
            return; 
        end 
    
    % learning mode 
   
    learning_mode_client=rossvcclient('/niryo_one/activate_learning_mode');% create a ros service client for activate learning mode 
   
        
    learning_mode_req = rosmessage(learning_mode_client); % create message for  learning_mode_client
    disp('..............learning mode .......');
    Learning_mode_state=rossubscriber('/niryo_one/learning_mode'); 
    
    
    logs=logs+ newline+  " Connect to Learning Mode service ";
    % calibrate Motors 
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
   jointState=rossubscriber('/joint_states','BufferSize',20)
   
      set(handles.connect,'string','connected ','BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
   
   connexion_state =1; 
else 
    logs=logs+ newline+ "you are already connected to your robot."
     disp('.............you are already connected to your robot ...................')

end 



% --- Executes on button press in disconnect.
function disconnect_Callback(~, ~, handles)
global connexion_state ; 
global logs; 

set(handles.disconnect,'BackgroundColor',[0.1,0.67,0.89],'ForegroundColor','white')
if (connexion_state==1)
rosshutdown;
 connexion_state =0;
  logs=logs+ newline + "disconnect form niryo one";
 set(handles.connect,'string','Connect to Niryo One ')
  set(handles.disconnect,'string','disconnected','BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
else 
    logs=logs+ newline + "you are already disconnect form niryo one";
     set(handles.disconnect,'string','disconnected','BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
      set(handles.connect,'string','Connect to Niryo One ')

end 



 


% --- Executes on button press in moveButton.
function moveButton_Callback(hObject, eventdata, handles)
global connexion_state ; 
global logs; 
global Learning_mode_state;

if (connexion_state==0)
     logs=logs+newline+"you should connect to you robot first"; 
    return; 
else 
[validation,joint]=validate_joints(handles,Learning_mode_state,logs) ;

if validation==0
    return ; 
else 
    
 move_joint_sub=rossubscriber('/niryo_one_matlab/result') ;
 move_joint_pub=rospublisher('/niryo_one_matlab/command');
  set(handles.moveButton,'string','Move Joints .....','BackgroundColor',[0.1,0.67,0.89],'ForegroundColor','white')
 move_joint_msg=rosmessage(move_joint_pub);

move_joint_msg.CmdType=1; 
move_joint_msg.Joints=joint;
send(move_joint_pub,move_joint_msg);


 set(handles.moveButton,'string','Move Joints','BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])

end 
end  
        
     % --- Executes on button press in learningModeButton.
function learningModeButton_Callback(hObject, eventdata, handles)
global learning_mode_client;
global learning_mode_req ;
global logs; 
global connexion_state ; 
global Learning_mode_state;
if (connexion_state==0)
    logs=logs+newline+"you should connect to you robot first" 
    return; 
else 
    Learning_mode_state_msg=receive(Learning_mode_state);
    set(handles.learningModeButton,'BackgroundColor',[0.1,0.67,0.89],'ForegroundColor','white')
 if (Learning_mode_state_msg.Data==1)
     learning_mode_req.Value=0;
     learning_mode_resp = call(learning_mode_client,learning_mode_req,'Timeout',3);
     disp(learning_mode_resp.Message);
     logs=logs+ newline+ learning_mode_resp.Message;
 else 
     learning_mode_req.Value=1;
     learning_mode_resp = call(learning_mode_client,learning_mode_req,'Timeout',3);
      disp(learning_mode_resp.Message);
       logs=logs+ newline+ learning_mode_resp.Message;
 end
 set(handles.learningModeButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
set(handles.edit56,'string',learning_mode_resp.Message,'visible','on')
pause(1)
set(handles.edit56,'visible','off')
end 

% --- Executes on button press in motorbutton.
function motorbutton_Callback(~, ~, handles)
 global calibrate_motors_msg;
global calibrate_motors_client;
global logs; 
global connexion_state ; 
if connexion_state==0
   logs=logs+newline+"you should connect to you robot first" 
   return; 
else 
  set(handles.motorbutton,'BackgroundColor',[0.1,0.67,0.89],'ForegroundColor','white')
 motor_calibration_resp = call(calibrate_motors_client,calibrate_motors_msg);
 disp( motor_calibration_resp.Message);
 logs=logs+ newline + motor_calibration_resp.Message;
 set(handles.edit56,'string',motor_calibration_resp.Message,'visible','on')
pause(1)
set(handles.edit56,'visible','off')
set(handles.motorbutton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
end 

 


% --- Executes on button press in newCalibrationButton.
function newCalibrationButton_Callback(hObject, eventdata,handles)
 global new_calibration ; 
global new_calibration_msg ;
global logs; 
global connexion_state ; 
if connexion_state==0
   logs=logs+newline+"you should connect to you robot first" 
   return;
else 
   set(handles.newCalibrationButton,'BackgroundColor',[0.1,0.67,0.89],'ForegroundColor','white')
 new_calibration_resp = call(new_calibration,new_calibration_msg);
 disp('.......new calibration requested.........');
set(handles.edit56,'string',new_calibration_resp.Message,'visible','on')
pause(1)
set(handles.edit56,'visible','off')
 logs=logs+ newline+new_calibration_resp.Message;
 set(handles.newCalibrationButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
 %disp(new_calibration_resp.Message)
end 
    
  % choose joint graph %%%%%%%%%%%%%%%%%%%%%%%%%%שששש  

function jointGroup_SelectionChangedFcn(hObject, eventdata, handles)
global time; 
global jointStateMsg; % joint State Message : real trajectory 
global data ; %follow joint trajectory Goal message : thero trajectory 
global joint_position; % follow joint trajectory Goal position : thero trajectory 
global connexion_state ;
if connexion_state==0

logs=logs+newline+"you should connect to you robot first"; 
   return;
else 
diff=CalculDiff(data,jointStateMsg,time)
JointStatePosition=[jointStateMsg.Position];
array_time=[data.Goal.Trajectory.Points(:,1).TimeFromStart];
x=[array_time.Nsec]/10^9+[array_time.Sec];
joint_position=[data.Goal.Trajectory.Points(:,1).Positions];
switch get(eventdata.NewValue,'Tag')   % Get Tag of selected object
    case 'radiobutton7'
      %execute this code when fontsize08_radiobutton is selected
      axes(handles.axestheorique);
      plot(x,joint_position(1,:),'b',time,JointStatePosition(1,:),'--r');
      xlabel('time')
      ylabel('trajectory')
      legend('theorical','real')
      axes(handles.axesdiff);
      plot(x,diff(1,:),'g');
      
    case 'radiobutton8'
      %execute this code when fontsize12_radiobutton is selected
       axes(handles.axestheorique);
        plot(x,joint_position(2,:),'b',time,JointStatePosition(2,:),'--r');
         xlabel('time')
      ylabel('trajectory')
         legend('theorical','real')
      axes(handles.axesdiff);
        plot(x,diff(2,:),'g');
        
    case 'radiobutton9'
      %execute this
       axes(handles.axestheorique);
        plot(x,joint_position(3,:),'b',time,JointStatePosition(3,:),'--r');
         xlabel('time')
      ylabel('trajectory')
         legend('theorical','real');
        axes(handles.axesdiff);
        plot(x,diff(3,:),'g');
       
case 'radiobutton10'
      %execute this code when fontsize08_radiobutton is selected
      axes(handles.axestheorique);
        plot(x,joint_position(4,:),'b',time,JointStatePosition(4,:),'--r');
         xlabel('time')
      ylabel('trajectory')
         legend('theorical','real');
      axes(handles.axesdiff);
        plot(x,diff(4,:),'g');
        

    case 'radiobutton11'
      %execute this code when fontsize12_radiobutton is selected
       axes(handles.axestheorique);
        plot(x,joint_position(5,:),'b',time,JointStatePosition(5,:),'--r');
         xlabel('time')
      ylabel('trajectory')
         legend('theorical','real')
      axes(handles.axesdiff);
        plot(x,diff(5,:),'g');
     

    case 'radiobutton12'
      %execute this
       axes(handles.axestheorique);
        plot(x,joint_position(6,:),'b',time,JointStatePosition(6,:),'--r');
         xlabel('time')
      ylabel('trajectory')
        legend('theorical','real')  
      axes(handles.axesdiff);
        plot(x,diff(6,:),'g')
   
    otherwise
       % Code for when there is no match.

end
end 

% --- Executes on button press in trajectoryButton. 
% on plot Panel 
function trajectoryButton_Callback(hObject, eventdata, handles)
global new_data;
global data ; 
global logs; 
global theor_trajectory;
global jointStateMsg;
global time; 
global connexion_state ;
if connexion_state==0
   logs=logs+newline+"you should connect to you robot first" 
   return;
else 
set(handles.trajectoryButton,'BackgroundColor',[0.1,0.67,0.89],'ForegroundColor','white');
 data=receive(theor_trajectory);
logs= logs+newline+"get a new trajectory";
[jointStateMsg,time]=JointState(handles,data);
logs= logs+newline+"get real trajectory";
set(handles.trajectoryButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89]);
new_data=1;
end





% --- Executes get trajectory 
% On command Panel 
function getTrajectoryButton_Callback(hObject, eventdata, handles)
global new_data;
global data ; 
global logs; 
global theor_trajectory;
global jointStateMsg;
global time; 
global connexion_state ;
if connexion_state==0
   logs=logs+newline+"you should connect to you robot first" 
   return;
else 
 
set(handles.getTrajectoryButton,'BackgroundColor',[0.1,0.67,0.89],'ForegroundColor','white');
 data=receive(theor_trajectory)
  
    
logs= logs+newline+"get theoretical trajectory ";
[jointStateMsg,time]=JointState(handles,data);
logs= logs+newline+" get real trajectory ";
set(handles.getTrajectoryButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89]);
new_data=1;
 plotButton_Callback(hObject, eventdata, handles)
end




% --- Executes export data to a file %%%%%%%%%%%%%%%%
function exportbutton_Callback(hObject, eventdata, handles)
% hObject    handle to exportbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global new_data;
global joint_position;
global data;
global logs ; 
global time;
global jointStateMsg;
global connexion_state; 
set(handles.exportbutton,'BackgroundColor',[0.1,0.67,0.89],'ForegroundColor','white');
if((new_data==0)&&(connexion_state==0) )
    set(handles.edit55,'string','try to get a new trajectory command ','visible','on');
    pause(1);
    set(handles.edit55,'visible','off') ;
    set(handles.exportbutton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89]);
else
    try 
 export_trajectory(time,jointStateMsg,data,joint_position);
    catch e 
        logs= logs+ newline + "you should specify file name , try again";
        set(handles.exportbutton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89]);
        return; 
    end 
        
  logs= logs + newline +"export trajectory " ;
  
set(handles.edit55,'string','data exported successfully','visible','on');
pause(1);
set(handles.edit55,'visible','off');
new_data=0;
end 
set(handles.exportbutton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89]);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function edit56_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit56 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% --- Executes during object creation, after setting all properties.
function edit55_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit55 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
         
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

function edit21_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit22_CreateFcn(hObject, ~, handles)
% hObject    handle to edit22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit23_CreateFcn(hObject, ~, handles)
% hObject    handle to edit23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit24_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit25_CreateFcn(hObject, ~, ~)
% hObject    handle to edit25 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit26_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

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



function edit43_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit43 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




function edit44_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit44 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end






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

function joint1_Callback(hObject, eventdata, handles)
   
function joint2_Callback(hObject, eventdata, handles)

function joint4_Callback(hObject, eventdata, handles)
function joint5_Callback(hObject, eventdata, handles)
function joint6_Callback(hObject, eventdata, handles)


function joint3_Callback(hObject, eventdata, handles)
% hObject    handle to joint3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function joint3_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function joint1_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function joint2_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
    

function joint4_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function joint5_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




function joint6_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes during object creation, after setting all properties.
function commandButton_CreateFcn(hObject, eventdata, handles)
% hObject    handle to commandButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
function connectionButton_CreateFcn(hObject, eventdata, handles)
% hObject    handle to commandButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

function edit1_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit2_Callback(hObject, eventdata, handles)
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
function commandPanel_CreateFcn(hObject, eventdata, handles)
       
function edit1_Callback(hObject, eventdata, handles)    
        
 function editlogs_Callback(hObject, eventdata, handles)  
