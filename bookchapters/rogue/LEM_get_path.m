function [path,v_path] = LEM_get_path(airways,v1,v2,props)
% LEM_get_path - get lane sequence from v1 to v2 (launch/land vertexes)
% On input:
%     airways (airway struct): airway info
%     v1 (int): lane vertex index of launch site
%     v2 (int): lane vertex index of land site
%     props (1xn vector): vector of path properties
% On output:
%     path (1xn vector): lane index sequence
%     v_path (1xn vector): vertex index sequence
% Call:
%     [path,v_path] = LEM_get_path(airways1,70,73,[]);
% Author:
%     T. Henderson
%     UU
%     Fall 2020
%

G = airways.G;
v_path = shortestpath(G,v1,v2);
path = LEM_vertexes2lanes(airways,v_path);
tch = 0;
