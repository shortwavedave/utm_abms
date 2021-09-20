function LEM_run_flights(obj,airways,flights,a_on,del_t,fname)
% LEM_run_flights - simulate flights through airway
% On input:
%     airways (airways struct): airways info
%     flights (flights struct): flights info
%     a_on (Boolean): if 1 show airways, else not
%     del_t (float): time step for moving UAS
%     fname (string): name of movie (.mp4) file ([] means make no movie)
% On output:
%     <fname>.mp4 (mp4 movie): movie file of flights
% Call:
%     LEM_run_flights(airways,flights,1,0.1,'sim1');
% Author:
%     T. Henderson
%     UU
%     Fall 2020
%

x_min = min(airways.lane_vertexes(:,1));
x_max = max(airways.lane_vertexes(:,1));
y_min = min(airways.lane_vertexes(:,2));
y_max = max(airways.lane_vertexes(:,2));
z_min = min(airways.lane_vertexes(:,3));
z_max = max(airways.lane_vertexes(:,3));
num_flights = length(flights);
clf
if a_on==1
    LBSD.LEM_show_airways3D(obj,airways,[]);
    hold on
elseif a_on==-1
    hold on
    view(-20.6,38.7077);
    for f = 1:num_flights
        pts = flights(f).path_vertexes;
        plot3([pts(1,1),pts(1,4)],[pts(1,2),pts(1,5)],...
            [pts(1,3),pts(1,6)],':k');
        plot3([pts(2,1),pts(2,4)],[pts(2,2),pts(2,5)],...
            [pts(2,3),pts(2,6)],':k');
        plot3([pts(3,1),pts(3,4)],[pts(3,2),pts(3,5)],...
            [pts(3,3),pts(3,6)],':k');
    end
else
    plot3(x_min-5,y_min-15,z_min-15,'w.');
    hold on
    plot3(x_min-5,y_min-15,z_max+15,'w.');
    plot3(x_max+5,y_min-20,z_min-10,'w.');
    plot3(x_max+5,y_max+5,z_max+15,'w.');
end

num_flights = length(flights);
t_min = Inf;
t_max = -Inf;
poses = zeros(num_flights,3);
for f = 1:num_flights
    traj = flights(f).traj;
    if ~isempty(traj)
        if traj(1,1)<t_min
            t_min = traj(1,1);
        end
        if traj(end,1)>t_max
            t_max = traj(end,1);
        end
        poses(f,:) = traj(1,2:4);
%        plot3(poses(f,1),poses(f,2),poses(f,3),'r*');
    end
end
%view(-20.6,38.7077);
plot3(x_max+5,y_min-20,z_min-20,'w.');
drawnow
if ~isempty(fname)
    writerObj = VideoWriter(fname,'MPEG-4');
    open(writerObj);
end

for t = [t_min:del_t:t_max]
    clf
    if a_on==1
%%%%        LEM_show_airways3D(airways,[]);
plot3(0,0,0,'w.');
hold on
plot3(61,61,61,'w.');
    elseif a_on==-1
        hold on
        view(-20.6,38.7077);
        for f = 1:num_flights
            pts = flights(f).path_vertexes;
            plot3([pts(1,1),pts(1,4)],[pts(1,2),pts(1,5)],...
                [pts(1,3),pts(1,6)],':k');
            plot3([pts(2,1),pts(2,4)],[pts(2,2),pts(2,5)],...
                [pts(2,3),pts(2,6)],':k');
            plot3([pts(3,1),pts(3,4)],[pts(3,2),pts(3,5)],...
                [pts(3,3),pts(3,6)],':k');
        end
    else
        plot3(x_min-5,y_min-15,z_min-15,'w.');
        hold on
        plot3(x_min-5,y_min-15,z_max+15,'w.');
        plot3(x_max+5,y_min-20,z_min-10,'w.');
        plot3(x_max+5,y_max+5,z_max+15,'w.');
    end
    hold on
    title(['Time: ',num2str(t),' of ',num2str(t_max)]);
    for f = 1:num_flights
        traj = flights(f).traj;
        if ~isempty(traj)
            [val,index] = min(abs(traj(:,1)-t));
            if abs(val-del_t)<=del_t
                ss = rem(f,3);
                if ss==0
                    plot3(traj(index,2),traj(index,3),traj(index,4),'ro');
                elseif ss==1
                    plot3(traj(index,2),traj(index,3),traj(index,4),'bo');
                else
                    plot3(traj(index,2),traj(index,3),traj(index,4),'ko');
                end
            end
        end
    end
    plot3(x_max+5,y_min-20,z_min-10,'w.');
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
