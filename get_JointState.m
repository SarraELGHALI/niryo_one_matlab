function [jointStateMsg,time]=get_JointState(handles,thero_trajectory_data,jointState)
    theorTime=thero_trajectory_data.Header.Stamp.Sec+(thero_trajectory_data.Header.Stamp.Nsec)/10^9;
  A=[thero_trajectory_data.Goal.Trajectory.Points(:,1).Positions];
    k=size(A);
    k=k(2);
    array_time=[thero_trajectory_data.Goal.Trajectory.Points(:,1).TimeFromStart];
    x=[array_time.Nsec]/10^9+[array_time.Sec];
    jointStateMsg=rosmessage(jointState);
    
    i=1;
    time(i)=0;
    while (time(i)< x(k))
        i=i+1;  
        jointStateMsg(i)=receive(jointState);
        time(i)=(jointStateMsg(i).Header.Stamp.Sec+(jointStateMsg(i).Header.Stamp.Nsec)/10^9)-theorTime;
    
        
    end
    for j=1:i
        time(j)=time(j)-time(1);
    end
    time(i) = [];