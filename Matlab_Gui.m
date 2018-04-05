
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

% Last Modified by GUIDE v2.5 05-Apr-2018 12:14:48

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
    set(handles.editlogs,'string',logs)% dispay application logs 


% --- Executes on button press in hwButton.
function hwButton_Callback(hObject, eventdata, handles)
    global hw_status;
    global logs; 
    global connexion_state; 
    
    % test if the robot is connected 
    if connexion_state==0
       logs=logs+newline+"you should connect to you robot first"; 
       return;
    end
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
      get_motors_names(handles,hw_status_msg);
      get_motors_Temperatures(handles,hw_status_msg);
      get_motors_Voltages(handles,hw_status_msg);
      
%      set(handles.edit23,'string', hw_status_msg.Temperatures);
%      set(handles.edit24,'string', hw_status_msg.HardwareErrors);
%      set(handles.edit25,'string',hw_status_msg.Voltages);
    % --- Executes on key press with focus on connectionButton and none of its controls.
  






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
    
    %global move_joint_sub;
    global move_joint_pub;
    global move_joint_msg;

    global logs; 
    global jointState;
    

    set(handles.moveButton,'string','Move Joints ','BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89]);
    set(handles.disconnect,'string','Disconnect from Niryo One');
    logs='';
    state=0; 
    set(handles.connect,'BackgroundColor',[0.1,0.67,0.89],'ForegroundColor','white');
    if (connexion_state  == 0)
        Robot_ip_adress = get(handles.edit1,'String'); % get robot ip adress 
        Robot_ip_adress = strcat('http://',Robot_ip_adress,':11311');
        Computer_ip_adress = get(handles.edit2,'String');% get computer ip adress
        setenv('ROS_MASTER_URI',Robot_ip_adress); % set  ros matser URI to specifie the ros master location 
        setenv('ROS_IP',Computer_ip_adress); % set environment variables ROS_IP( network address of a ROS Node ), It's must be ip adress of the pc where matlab is installed 
    try
        rosinit; % initialize ROS
    catch e
        logs= e.message;
        set(handles.connect,'string','connected ','BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89]);
        return; 
    end 

        %pause(2);
        logs="connected to niryo one";
        % hardware status
            try 
            hw_status=rossubscriber('/niryo_one/hardware_status'); %create a subscriber for  hardware statuts topic
            catch e 
                logs= e.message
                rosshutdown
                return; 
            end 

        % learning mode 
        learning_mode_client=rossvcclient('/niryo_one/activate_learning_mode');% create a ros service client for activate learning mode 
        learning_mode_req = rosmessage(learning_mode_client); % create message for  learning_mode_client
        disp('..............learning mode .......');
        Learning_mode_state=rossubscriber('/niryo_one/learning_mode'); 
        logs=logs+ newline+  " Connect to Learning Mode service ";
       
        % request new calibartion  
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
       
       jointState=rossubscriber('/joint_states','BufferSize',40);
       
       
       % Matlab node subscirbers and publisher
       move_joint_sub=rossubscriber('/niryo_one_matlab/result') ;
       move_joint_pub=rospublisher('/niryo_one_matlab/command');
       move_joint_msg=rosmessage(move_joint_pub);
       
       % button set 
       set(handles.connect,'string','connected ','BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89]);

       connexion_state =1; 
    else 
        logs=logs+ newline+ "you are already connected to your robot.";
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
   global move_joint_pub;
   global move_joint_msg;
    
   if (connexion_state==0)
       logs=logs+newline+"you should connect to you robot first"; 
        return; 
    end
    [validation,joint]=validate_joints(handles,Learning_mode_state,logs) ;

    if validation==0
        return ; 
    end
    set(handles.moveButton,'string','Move Joints .....','BackgroundColor',[0.1,0.67,0.89],'ForegroundColor','white')
    move_joint_msg.CmdType=1; 
    move_joint_msg.Joints=joint
    send(move_joint_pub,move_joint_msg);
    logs=logs+newline+"move jonit"
     set(handles.moveButton,'string','Move Joints','BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])

    
   
         
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
    end 
        Learning_mode_state_msg=receive(Learning_mode_state);
        set(handles.learningModeButton,'BackgroundColor',[0.1,0.67,0.89],'ForegroundColor','white')
    if (Learning_mode_state_msg.Data==1)
          set_learningModeMsg(learning_mode_client,learning_mode_req,0,logs,handles) ;     
    else 
       set_learningModeMsg(learning_mode_client,learning_mode_req,1,logs,handles) ;   
    end
    set(handles.learningModeButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
    set(handles.edit56,'visible','off')
    

    % --- Executes on button press in motorbutton.
function motorbutton_Callback(~, ~, handles)
    global calibrate_motors_msg;
    global calibrate_motors_client;
    global logs; 
    global connexion_state ; 
    global hw_status;
    if connexion_state==0
       logs=logs+newline+"you should connect to you robot first" 
       return; 
    end 
    hw_status_msg=receive(hw_status,10); % get calibrationNeeded flag 
    set(handles.motorbutton,'BackgroundColor',[0.1,0.67,0.89],'ForegroundColor','white')
    % testif the calibrationNeeded is on or off 
    if hw_status_msg.CalibrationNeeded==0
        logs=logs+newline+"you should request a new calibration first" 
        set(handles.edit56,'string',"you should request a new calibration first" ,'visible','on')
        pause(1)
        set(handles.edit56,'visible','off')
    else
        motor_calibration_resp = call(calibrate_motors_client,calibrate_motors_msg); 
        disp( motor_calibration_resp.Message);
        logs=logs+ newline + motor_calibration_resp.Message;
        set(handles.edit56,'string',motor_calibration_resp.Message,'visible','on')
        pause(1)
        set(handles.edit56,'visible','off')
    end 
    set(handles.motorbutton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
  
   

 


% --- Executes on button press in newCalibrationButton.
function newCalibrationButton_Callback(hObject, eventdata,handles)
    global new_calibration ; 
    global new_calibration_msg ;
    global logs; 
    global connexion_state ; 


    if connexion_state==0
       logs=logs+newline+"you should connect to you robot first" 
       return;
    end 
    set(handles.newCalibrationButton,'BackgroundColor',[0.1,0.67,0.89],'ForegroundColor','white')
    new_calibration_resp = call(new_calibration,new_calibration_msg); 
    disp('.......new calibration requested.........');
    set(handles.edit56,'string',new_calibration_resp.Message,'visible','on')
    pause(1)
    set(handles.edit56,'visible','off')
    logs=logs+ newline+new_calibration_resp.Message;
    set(handles.newCalibrationButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])


    
  % choose joint graph %%%%%%%%%%%%%%%%%%%%%%%%%%שששש  


               
function jointGroup_SelectionChangedFcn(hObject, eventdata, handles)
    global thero_trajectory;
    global real_trajectory; 
    global diff_trajectory;
    global imported_data;
    global time; 
    global jointStateMsg; % joint State Message : real trajectory 
    global thero_trajectory_data ; %follow joint trajectory Goal message : thero trajectory 
    global connexion_state ;
    global logs;
    if connexion_state==0
    logs=logs+newline+"you should connect to you robot first"; 
       return;
    end 
    [theor_time,thero_y,real_time,real_y,diff_y,imported_data]=Get_trajectories(thero_trajectory,real_trajectory,diff_trajectory,imported_data,time,jointStateMsg,thero_trajectory_data);
   
    switch get(eventdata.NewValue,'Tag')   % Get Tag of selected object
        case 'radiobutton7'
          %execute this code when fontsize08_radiobutton is selected
          plot_graphs(handles,real_time,theor_time,diff_y,thero_y,real_y,1,-3.1,3.1);
        case 'radiobutton8'
          %joint2
          plot_graphs(handles,real_time,theor_time,diff_y,thero_y,real_y,2,-1.7,1.7);
        case 'radiobutton9'
          %joint3
           plot_graphs(handles,real_time,theor_time,diff_y,thero_y,real_y,3,-1.6,1.1);
    case 'radiobutton10'
          %joint4
        plot_graphs(handles,real_time,theor_time,diff_y,thero_y,real_y,4,-2.8,2.8);
        case 'radiobutton11'
          % joint5
         plot_graphs(handles,real_time,theor_time,diff_y,thero_y,real_y,5,-2.5,2.5);
        case 'radiobutton12'
         %joint6
          plot_graphs(handles,real_time,theor_time,diff_y,thero_y,real_y,6,-2.8,2.8);
        otherwise
    end
  

% --- Executes on button press in trajectoryButton. 
% on plot Panel 
function trajectoryButton_Callback(hObject, eventdata, handles)
    global new_data_trajectory_received;
    global thero_trajectory_data ; 
    global logs; 
    global theor_trajectory;
    global jointStateMsg;
    global time; 
    global connexion_state ;
    global jointState; 
    if connexion_state==0
       logs=logs+newline+"you should connect to you robot first" 
       return;
    end 
    set(handles.trajectoryButton,'BackgroundColor',[0.1,0.67,0.89],'ForegroundColor','white');
     thero_trajectory_data=receive(theor_trajectory);
    logs= logs+newline+"get a new trajectory";
    [jointStateMsg,time]=get_JointState(handles,thero_trajectory_data,jointState)
    logs= logs+newline+"get real trajectory";
    set(handles.trajectoryButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89]);
    new_data_trajectory_received=1;
   





% --- Executes get trajectory 
% On command Panel 
function getTrajectoryButton_Callback(hObject, eventdata, handles)
    global new_data_trajectory_received;
    global thero_trajectory_data ; 
    global logs; 
    global theor_trajectory;
    global jointStateMsg;
    global time; 
    global connexion_state ;
    global jointState; 
    if connexion_state==0
       logs=logs+newline+"you should connect to you robot first" 
       return;
    end
    set(handles.getTrajectoryButton,'BackgroundColor',[0.1,0.67,0.89],'ForegroundColor','white');
     thero_trajectory_data=receive(theor_trajectory)


    logs= logs+newline+"get theoretical trajectory ";
    [jointStateMsg,time]=get_JointState(handles,thero_trajectory_data,jointState)
    logs= logs+newline+" get real trajectory ";
    set(handles.getTrajectoryButton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89]);
    new_data_trajectory_received=1;
     plotButton_Callback(hObject, eventdata, handles);





% --- Executes export data to a file %%%%%%%%%%%%%%%%
function exportbutton_Callback(hObject, eventdata, handles)
% hObject    handle to exportbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    global new_data_trajectory_received;
    global joint_position;
    global thero_trajectory_data;
    global logs ; 
    global time;
    global jointStateMsg;
    global connexion_state;
    joint_position=[thero_trajectory_data.Goal.Trajectory.Points(:,1).Positions];
    set(handles.exportbutton,'BackgroundColor',[0.1,0.67,0.89],'ForegroundColor','white');
    if((new_data_trajectory_received==0)||(connexion_state==0))
        set(handles.edit55,'string','try to get a new trajectory command ','visible','on');
        pause(1);
        set(handles.edit55,'visible','off') ;
        set(handles.exportbutton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89]);
        return; 
    end
    try 
        export_trajectory(time,jointStateMsg,thero_trajectory_data,joint_position);
    catch e 
        logs= logs+ newline + "you should specify file name , try again";
        set(handles.exportbutton,'BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89]);
        return; 
    end
    logs= logs + newline +"export trajectory " ;
    set(handles.edit55,'string','data exported successfully','visible','on');
    pause(1);
    set(handles.edit55,'visible','off');
    new_data_trajectory_received=0;
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
  
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end

function edit21_CreateFcn(hObject, eventdata, handles)

    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end

function edit22_CreateFcn(hObject, ~, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end



function edit23_CreateFcn(hObject, ~, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end

function edit24_CreateFcn(hObject, eventdata, handles)

    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


function edit25_CreateFcn(hObject, ~, ~)

    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


function edit26_CreateFcn(hObject, eventdata, handles)

    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end

% --- Executes during object creation, after setting all properties.
function edit27_CreateFcn(hObject, ~, handles)

    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


% --- Executes during object creation, after setting all properties.
function edit28_CreateFcn(hObject, eventdata, handles)

    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end



% --- Executes during object creation, after setting all properties.
function edit29_CreateFcn(hObject, eventdata, handles)

    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end

% --- Executes during object creation, after setting all properties.
function edit30_CreateFcn(hObject, eventdata, handles)

    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end

% --- Executes during object creation, after setting all properties.
function edit31_CreateFcn(hObject, eventdata, handles)

    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end

% --- Executes during object creation, after setting all properties.
function editlogs_CreateFcn(hObject, eventdata, handles)
    
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end

% --- Executes during object creation, after setting all properties.
function edit41_CreateFcn(hObject, eventdata, ~)
    
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


% --- Executes during object creation, after setting all properties.
function edit42_CreateFcn(hObject, eventdata, handles)

    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end



function edit43_CreateFcn(hObject, eventdata, handles)

    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end




function edit44_CreateFcn(hObject, eventdata, handles)

    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end






% --- Executes during object creation, after setting all properties.
function edit45_CreateFcn(hObject, eventdata, handles)

    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end






% --- Executes during object creation, after setting all properties.
function edit46_CreateFcn(hObject, eventdata, handles)

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


% --- Executes on button press in importButton.
function importButton_Callback(hObject, eventdata, handles)
% hObject    handle to importButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global thero_trajectory;
global real_trajectory; 
global diff_trajectory;
global imported_data;
[thero_trajectory,real_trajectory,diff_trajectory]=import_trajectory()
imported_data=1; 



function motor1_Callback(hObject, eventdata, handles)
% hObject    handle to motor1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of motor1 as text
%        str2double(get(hObject,'String')) returns contents of motor1 as a double


% --- Executes during object creation, after setting all properties.
function motor1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to motor1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function motor2_Callback(hObject, eventdata, handles)
% hObject    handle to motor2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of motor2 as text
%        str2double(get(hObject,'String')) returns contents of motor2 as a double


% --- Executes during object creation, after setting all properties.
function motor2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to motor2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function motor3_Callback(hObject, eventdata, handles)
% hObject    handle to motor3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of motor3 as text
%        str2double(get(hObject,'String')) returns contents of motor3 as a double


% --- Executes during object creation, after setting all properties.
function motor3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to motor3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function motor4_Callback(hObject, eventdata, handles)
% hObject    handle to motor4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of motor4 as text
%        str2double(get(hObject,'String')) returns contents of motor4 as a double


% --- Executes during object creation, after setting all properties.
function motor4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to motor4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function motor5_Callback(hObject, eventdata, handles)
% hObject    handle to motor5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of motor5 as text
%        str2double(get(hObject,'String')) returns contents of motor5 as a double


% --- Executes during object creation, after setting all properties.
function motor5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to motor5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function motor6_Callback(hObject, eventdata, handles)
% hObject    handle to motor6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of motor6 as text
%        str2double(get(hObject,'String')) returns contents of motor6 as a double


% --- Executes during object creation, after setting all properties.
function motor6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to motor6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function motor7_Callback(hObject, eventdata, handles)
% hObject    handle to motor7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of motor7 as text
%        str2double(get(hObject,'String')) returns contents of motor7 as a double


% --- Executes during object creation, after setting all properties.
function motor7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to motor7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function motor8_Callback(hObject, eventdata, handles)
% hObject    handle to motor8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of motor8 as text
%        str2double(get(hObject,'String')) returns contents of motor8 as a double


% --- Executes during object creation, after setting all properties.
function motor8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to motor8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function t1_Callback(hObject, eventdata, handles)
% hObject    handle to t1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of t1 as text
%        str2double(get(hObject,'String')) returns contents of t1 as a double


% --- Executes during object creation, after setting all properties.
function t1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to t1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function t2_Callback(hObject, eventdata, handles)
% hObject    handle to t2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of t2 as text
%        str2double(get(hObject,'String')) returns contents of t2 as a double


% --- Executes during object creation, after setting all properties.
function t2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to t2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function t3_Callback(hObject, eventdata, handles)
% hObject    handle to t3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of t3 as text
%        str2double(get(hObject,'String')) returns contents of t3 as a double


% --- Executes during object creation, after setting all properties.
function t3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to t3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function t4_Callback(hObject, eventdata, handles)
% hObject    handle to t4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of t4 as text
%        str2double(get(hObject,'String')) returns contents of t4 as a double


% --- Executes during object creation, after setting all properties.
function t4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to t4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function t5_Callback(hObject, eventdata, handles)
% hObject    handle to t5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of t5 as text
%        str2double(get(hObject,'String')) returns contents of t5 as a double


% --- Executes during object creation, after setting all properties.
function t5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to t5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function t6_Callback(hObject, eventdata, handles)
% hObject    handle to t6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of t6 as text
%        str2double(get(hObject,'String')) returns contents of t6 as a double


% --- Executes during object creation, after setting all properties.
function t6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to t6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function t7_Callback(hObject, eventdata, handles)
% hObject    handle to t7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of t7 as text
%        str2double(get(hObject,'String')) returns contents of t7 as a double


% --- Executes during object creation, after setting all properties.
function t7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to t7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function t8_Callback(hObject, eventdata, handles)
% hObject    handle to t8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of t8 as text
%        str2double(get(hObject,'String')) returns contents of t8 as a double


% --- Executes during object creation, after setting all properties.
function t8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to t8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function v1_Callback(hObject, eventdata, handles)
% hObject    handle to v1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of v1 as text
%        str2double(get(hObject,'String')) returns contents of v1 as a double


% --- Executes during object creation, after setting all properties.
function v1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to v1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function v2_Callback(hObject, eventdata, handles)
% hObject    handle to v2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of v2 as text
%        str2double(get(hObject,'String')) returns contents of v2 as a double


% --- Executes during object creation, after setting all properties.
function v2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to v2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function v3_Callback(hObject, eventdata, handles)
% hObject    handle to v3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of v3 as text
%        str2double(get(hObject,'String')) returns contents of v3 as a double


% --- Executes during object creation, after setting all properties.
function v3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to v3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function v4_Callback(hObject, eventdata, handles)
% hObject    handle to v4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of v4 as text
%        str2double(get(hObject,'String')) returns contents of v4 as a double


% --- Executes during object creation, after setting all properties.
function v4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to v4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function v5_Callback(hObject, eventdata, handles)
% hObject    handle to v5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of v5 as text
%        str2double(get(hObject,'String')) returns contents of v5 as a double


% --- Executes during object creation, after setting all properties.
function v5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to v5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function v6_Callback(hObject, eventdata, handles)
% hObject    handle to v6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of v6 as text
%        str2double(get(hObject,'String')) returns contents of v6 as a double


% --- Executes during object creation, after setting all properties.
function v6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to v6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function v7_Callback(hObject, eventdata, handles)
% hObject    handle to v7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of v7 as text
%        str2double(get(hObject,'String')) returns contents of v7 as a double


% --- Executes during object creation, after setting all properties.
function v7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to v7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function v8_Callback(hObject, eventdata, handles)
% hObject    handle to v8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of v8 as text
%        str2double(get(hObject,'String')) returns contents of v8 as a double


% --- Executes during object creation, after setting all properties.
function v8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to v8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox1
