function plot_graphs(handles,real_time,theor_time,diff_trajectory,thero_trajectory,realtrajectory,i,ymin,ymax)
axes(handles.axestheorique);
plot(theor_time,thero_trajectory(i,:),'b',real_time,realtrajectory(i,:),'--r');
xlabel('time');
ylabel('trajectory');
legend('theorical','real');
limits=[ymin,ymax];
ylim(limits);
axes(handles.axesdiff);
plot(theor_time,diff_trajectory(i,:),'g');