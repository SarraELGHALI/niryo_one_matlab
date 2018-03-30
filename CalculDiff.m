function [diff]=CalculDiff(data,jointStateMsg,time)

array_time=[data.Goal.Trajectory.Points(:,1).TimeFromStart];
time_theor=[array_time.Nsec]/10^9+[array_time.Sec];
A=[data.Goal.Trajectory.Points(:,1).Positions];
B=[jointStateMsg(1,:).Position];
k=size(A);

for i=1:6
for j=1:k(2)
    X = interp1(time,B(i,:),time_theor(j));
 
 diff(i,j)=abs(A(i,j)-X);
 
end 
end
 