function LEM_show_airways3D(airways,path)
% LEM_show_airwas3D - show lanes and path through them, if any
% On input:
%     airways (airways struct): lanes info
%     path (nx1 vector): lane vertex index sequence through airway
% On output:
%     <Display of 3D airway structure>
% Call:
%     LEM_show_airways3D(airways1,[],0);
% Author:
%     T. Henderson
%     UU
%     Fall 2020
%

vertexes = airways.lane_vertexes;
lanes = airways.lanes;

%clf
hold on
view(-25,17.6);
num_lanes = length(lanes(:,1));
for c = 1:num_lanes
    if lanes(c,1)==lanes(c,4)&lanes(c,2)==lanes(c,5)&lanes(c,3)<lanes(c,6)...
            &max(lanes(c,3),lanes(c,6))-min(lanes(c,3),lanes(c,6))>0
        plot3([lanes(c,1),lanes(c,4)],[lanes(c,2),lanes(c,5)],...
            [lanes(c,3),lanes(c,6)],'k');
    elseif lanes(c,1)==lanes(c,4)&lanes(c,2)==lanes(c,5)&lanes(c,3)>lanes(c,6)...
            &max(lanes(c,3),lanes(c,6))-min(lanes(c,3),lanes(c,6))>0
        plot3([lanes(c,1),lanes(c,4)],[lanes(c,2),lanes(c,5)],...
            [lanes(c,3),lanes(c,6)],'k');
    else
        plot3([lanes(c,1),lanes(c,4)],[lanes(c,2),lanes(c,5)],...
            [lanes(c,3),lanes(c,6)],'k');
    end
end

if ~isempty(path)
    len_path = length(path);
    for e = 1:len_path
        pt1 = lanes(path(e),1:3);
        pt2 = lanes(path(e),4:6);
        plot3([pt1(1),pt2(1)],[pt1(2),pt2(2)],[pt1(3),pt2(3)],'r',...
            'LineWidth',6);
        plot3([pt1(1),pt2(1)],[pt1(2),pt2(2)],[pt1(3),pt2(3)],'r*');
    end
end
