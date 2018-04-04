function plot_graphs(handles,time,x,diff,joint_position,JointStatePosition,i,ymin,ymax)
    axes(handles.axestheorique);
  
    plot(x,joint_position(i,:),'b',time,JointStatePosition(i,:),'--r');
    xlabel('time')
    ylabel('trajectory')
    legend('theorical','real')
    limits=[ymin,ymax]
    ylim(limits);
    axes(handles.axesdiff);
    plot(x,diff(i,:),'g');