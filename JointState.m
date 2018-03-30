function [jointStateMsg,time]=JointState(handles,data)

 theorTime=data.Header.Stamp.Sec+(data.Header.Stamp.Nsec)/10^9;
 A=[data.Goal.Trajectory.Points(:,1).Positions];
 k=size(A);

jointState=rossubscriber('/joint_states');
jointStateMsg=rosmessage(jointState);
for i=1:k(2)
 jointStateMsg(i)=receive(jointState);
 time(i)=(jointStateMsg(i).Header.Stamp.Sec+(jointStateMsg(i).Header.Stamp.Nsec)/10^9)-theorTime;
 
end
for i=1:k(2)
    time(i)=time(i)-time(1);
end