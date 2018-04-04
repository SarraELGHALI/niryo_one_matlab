function [jointStateMsg,time]=get_JointState(handles,data,jointState)

 theorTime=data.Header.Stamp.Sec+(data.Header.Stamp.Nsec)/10^9;
 A=[data.Goal.Trajectory.Points(:,1).Positions];
 k=size(A);


jointStateMsg=rosmessage(jointState);
for i=1:k(2)
 jointStateMsg(i)=receive(jointState);
 time(i)=(jointStateMsg(i).Header.Stamp.Sec+(jointStateMsg(i).Header.Stamp.Nsec)/10^9)-theorTime;
 
end
for i=1:k(2)
    time(i)=time(i)-time(1);
end