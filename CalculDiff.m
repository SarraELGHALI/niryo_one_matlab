function [diff]=CalculDiff(thero_trajectory_data,jointStateMsg,time)

array_time=[thero_trajectory_data.Goal.Trajectory.Points(:,1).TimeFromStart];
time_theor=[array_time.Nsec]/10^9+[array_time.Sec];
A=[thero_trajectory_data.Goal.Trajectory.Points(:,1).Positions];
B=[jointStateMsg(1,:).Position];
k=size(time);
m=size(time_theor);
if k(2)>m(2) 
    l=m(2);
else 
    l=k(2);
end 
for i=1:6
for j=1:l
    X = interp1(time,B(i,:),time_theor(j));
    diff(i,j)=abs(A(i,j)-X);
 
end 
end
 