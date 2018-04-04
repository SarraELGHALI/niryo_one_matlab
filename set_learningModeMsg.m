function set_learningModeMsg(learning_mode_client,learning_mode_req,value,logs,handles) 
    learning_mode_req.Value=value;
    learning_mode_resp = call(learning_mode_client,learning_mode_req,'Timeout',3);
    disp(learning_mode_resp.Message);
    logs=logs+ newline+ learning_mode_resp.Message;
    set(handles.edit56,'string',learning_mode_resp.Message,'visible','on')
    pause(1)