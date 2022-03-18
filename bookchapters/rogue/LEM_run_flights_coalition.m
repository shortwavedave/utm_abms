function LEM_run_flights_coalition(a1,a2,a3,flights,del_t,fname)
% LEM_run_flights - simulate flights
% On input:
%     flights (flights struct): flights info
%     del_t (float): time step for moving UAS
%     fname (string): name of movie (.mp4) file ([] means make no movie)
% On output:
%     <fname>.mp4 (mp4 movie): movie file of flights
% Call:
%     LEM_run_flights(flights,0.1,'coal1');
% Author:
%     T. Henderson
%     UU
%     Fall 2021
%

INDEX_ISR = 1;
INDEX_MANNED = 2;
INDEX_UAS = 3;

x_min = -100;
x_max = 5100;
y_min = -100;
y_max = 5100;
z_min = -100;
z_max = 35000;
num_flights = length(flights);

t_min = Inf;
t_max = -Inf;
for f = 1:num_flights
    traj = flights(f).traj;
    if ~isempty(traj)
        if traj(1,4)<t_min
            t_min = traj(1,4);
        end
        if traj(end,4)>t_max
            t_max = traj(end,4);
        end
    end
end
clf
plot3(x_min-5,y_min-15,z_min-15,'w.');
hold on
plot3(x_max+5,y_min-20,z_min-10,'w.');
plot3(x_max+5,y_max+5,z_max+15,'w.');
plot3(x_max+5,y_min-20,z_min-20,'w.');
LEM_show_airways3D(a1,[]);
LEM_show_airways3D(a2,[]);
LEM_show_airways3D(a3,[]);
%view(-20.6,3.7077);
view(0,90);
drawnow
if ~isempty(fname)
    writerObj = VideoWriter(fname,'MPEG-4');
    open(writerObj);
end

for t = [t_min:del_t:t_max]
    clf
    plot3(x_min-5,y_min-15,z_min-15,'w.');
    hold on
    plot3(x_min-5,y_min-15,z_max+15,'w.');
    plot3(x_max+5,y_min-20,z_min-10,'w.');
    plot3(x_max+5,y_max+5,z_max+15,'w.');
    LEM_show_airways3D(a1,[]);
    LEM_show_airways3D(a2,[]);
    LEM_show_airways3D(a3,[]);
    title(['Time: ',num2str(t),' of ',num2str(t_max)]);
    for f = 1:num_flights
        traj = flights(f).traj;
        if flights(f).team<5
            if ~isempty(traj)
                [val,index] = min(abs(traj(:,4)-t));
                if abs(val-del_t)<=del_t
                    if flights(f).team==1
                        plot3(traj(index,1),traj(index,2),traj(index,3),'g*');
                        plot3(traj(index,1),traj(index,2),traj(index,3),'ko');
                    elseif flights(f).team==2
                        plot3(traj(index,1),traj(index,2),traj(index,3),'r*');
                        plot3(traj(index,1),traj(index,2),traj(index,3),'ko');
                    elseif flights(f).team==3
                        plot3(traj(index,1),traj(index,2),traj(index,3),'b*');
                        plot3(traj(index,1),traj(index,2),traj(index,3),'ko');
                    elseif flights(f).team==4
                        plot3(traj(index,1),traj(index,2),traj(index,3),'k*');
                        plot3(traj(index,1),traj(index,2),traj(index,3),'ko');
                    end
                end
            end
        else
            len_traj = length(traj(:,4));
            [tt,index2] = min(abs(traj(:,4)-t));
            index1 = max(1,index2-20);
            if len_traj-index2>21
                plot3(traj(index1:index2,1),traj(index1:index2,2),...
                    traj(index1:index2,3),'m.');
            end
        end
    end
    plot3(x_max+5,y_min-20,z_min-10,'w.');
    view(0,90);
%    view(-20.6,3.7077);
    drawnow;
    frame = getframe(gcf);
    if ~isempty(fname)
        writeVideo(writerObj, frame);
    end
end
hold off
if ~isempty(fname)
    close(writerObj);
end
