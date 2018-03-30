function [validation,joint1,joint2,joint3,joint4,joint5,joint6]=validate_joints(handles,Learning_mode_state,logs) 
    
        joint1=str2double(get(handles.joint1,'String'));
joint2=str2double(get(handles.joint2,'String'));
joint3=str2double(get(handles.joint3,'String'));
joint4=str2double(get(handles.joint4,'String'));
joint5=str2double(get(handles.joint5,'String'));
joint5=str2double(get(handles.joint5,'String'));
joint6=str2double(get(handles.joint6,'String'));
validation=0;
Learning_mode_state_msg=receive( Learning_mode_state); 
if (Learning_mode_state_msg.Data==1)
    set(handles.edit56,'string','you need to desactivate Learning Mode','visible','on','BackgroundColor','red','ForegroundColor','white')
    pause(1);
    set(handles.edit56,'visible','off','BackgroundColor','green','ForegroundColor','white');
     set(handles.moveButton,'string','Move Joints','BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
     return ; 
elseif (joint1>3.054)||(joint1<-3.054)
    set(handles.joint1,'string',0); 
    set(handles.edit56,'string','joint 1 not in range(-3.054,3.054)','visible','on','BackgroundColor','red','ForegroundColor','white')
    pause(1);
    set(handles.edit56,'visible','off','BackgroundColor','green','ForegroundColor','white');
     set(handles.moveButton,'string','Move Joints','BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
     return;
elseif(joint1>0.628319)||(joint1<-1.5707)
    set(handles.joint2,'string',0); 
    set(handles.edit56,'string','joint 2 not in range(-1.5707,0.628319)','visible','on','BackgroundColor','red','ForegroundColor','white')
    pause(1);
    set(handles.edit56,'visible','off','BackgroundColor','green','ForegroundColor','white');
     set(handles.moveButton,'string','Move Joints','BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
     return;
elseif (joint3>0.994838)||(joint3<-1.4101)
    set(handles.joint3,'string',0); 
    set(handles.edit56,'string','joint 3 not in range(-1.4101,0.994838)','visible','on','BackgroundColor','red','ForegroundColor','white')
    pause(1);
    set(handles.edit56,'visible','off','BackgroundColor','green','ForegroundColor','white');
     set(handles.moveButton,'string','Move Joints','BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
     return;

elseif(joint4>2.61799)||(joint4<-2.61799)
    set(handles.joint4,'string',0); 
    set(handles.edit56,'string','joint 4not in range(-2.61799,2.61799)','visible','on','BackgroundColor','red','ForegroundColor','white')
    pause(0.5);
    set(handles.edit56,'visible','off','BackgroundColor','green','ForegroundColor','white');
     set(handles.moveButton,'string','Move Joints','BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
     return;


elseif(joint5>2.26893)||(joint5<-2.26893)
    set(handles.joint5,'string',0); 
    set(handles.edit56,'string','joint 5 not in range(-2.26893,2.26893)','visible','on','BackgroundColor','red','ForegroundColor','white')
    pause(0.5);
    set(handles.edit56,'visible','off','BackgroundColor','green','ForegroundColor','white');
     set(handles.moveButton,'string','Move Joints','BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
     return;


elseif (joint6>2.57)||(joint6<-2.57)
    set(handles.joint6,'string',0); 
    set(handles.edit56,'string','joint 6 not in range(-2.57,2.57)','visible','on','BackgroundColor','red','ForegroundColor','white')
    pause(0.5);
    set(handles.edit56,'visible','off','BackgroundColor','green','ForegroundColor','white');
     set(handles.moveButton,'string','Move Joints','BackgroundColor','white','ForegroundColor',[0.1,0.67,0.89])
   return;
else disp("joints in range") ;
  logs=logs+newline+"Joints in range";
 validation=1; 
end