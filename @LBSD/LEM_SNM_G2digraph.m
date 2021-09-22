function Gd = LEM_SNM_G2digraph(G)
% LEM_SNM_G2digraph - convert graph to Matlab digraph
% On input:
%     G (Matlab graph struct): undirected graph info
%       .Nodes (nx3 array): vertex locations in cols 1 and 2
%       .Edges (mx2 array): vertex indexes for edges
% On output:
%     Gd (Matlab digraph): digraph for G
% Call:
%     dg = LEM_SNM_G2digaph(G);
% Author:
%     T. Henderson
%     UU
%     Spring 2021
%

if isempty(G)
    Gd = [];
    return
end

V = table2array(G.Nodes);
V = V(:,1:2);
[num_V,dummy] = size(V);
E = table2array(G.Edges);
[num_E,dummy] = size(E);

indexes1 = E(:,1);
indexes2 = E(:,2);
weights = zeros(num_E,1);
for e = 1:num_E
    weights(e) = norm(V(indexes1(e),:)-V(indexes2(e),:));
end
E = [E(:,1:2); [E(:,2),E(:,1)]];
W = [weights;weights];

Gd = digraph(E(:,1),E(:,2),W);
Gd.Nodes = array2table(V);
