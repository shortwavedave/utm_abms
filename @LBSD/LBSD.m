classdef LBSD < handle
    %LBSD An instance of this class represents a Lane Based Strategic
    %   Deconfliction Supplemental Data Service Provider (SDSP).
    %   Detailed explanation goes here
    
    properties
        % A digraph representing the lane network
        lane_graph
    end
    
    methods
        function obj = LBSD()
            %LBSD Construct an instance of this class
            %   Detailed explanation goes here
        end

        function h = plot(obj)
            xdata = obj.lane_graph.Nodes.XData;
            ydata = obj.lane_graph.Nodes.YData;
            zdata = obj.lane_graph.Nodes.ZData;
            h = plot(obj.lane_graph,'XData',xdata,'YData',ydata, ...
                'ZData',zdata);
        end
    end
    
    methods (Static)
        function lbsd = genSampleLanes(lane_length_m, altitude_m)
            % genSampleLanes Create a sample lane system
            %   The generated lane system contains a single planar
            %   roundabout with 8 sides of length lane_length. This is a
            %   static function that instantiates an LBSD object (it is a
            %   factory for LBSD objects)
            %   On Input:
            %       lane_length_m - (float) the length of every lane in the 
            %           system in meters.
            %       altitude_m - (float) the height of the lane system in
            %           meters
            %   On Output:
            %       LBSD - an instance of the LBSD class initialized with
            %       the sample lane network.
            %   Call:
            %       lbsd = LBSD.genSampleLanes(10, 15)
            
            % Create an octagonal roundabout
            roundabout = nsidedpoly(8,'Center',[0 0],'SideLength',...
                lane_length_m);
            num_verts = size(roundabout.Vertices, 1);
            
            % Generate a node table with the roundabout vertexes
            node_table = table(roundabout.Vertices(:,1), ...
                roundabout.Vertices(:,2), ...
                altitude_m*ones(num_verts,1), ...,
                zeros(num_verts, 1), ...
                zeros(num_verts, 1), ...
                'VariableNames', ...
                {'XData', 'YData', 'ZData', 'Launch', 'Land'});
            
            % Generate an edge table for the roundabout vertexes
            edge_table = table([circshift(1:8,1)', (1:8)'], ...
                lane_length_m*ones(num_verts,1), 'VariableNames', ...
                {'EndNodes','Weights'});
            
            % Add alternating land an launch sites
            total_verts = num_verts;
            % Create lookup table for adding wings to the roundabout. I
            % dont know what order matlab creates the nodes of the octagon.
            oct_angles = 22.5:45:337.5;
            oct_angles(oct_angles>180) = oct_angles(oct_angles>180) - 360;
            % Each row in this matrix has this format: [angle, x, y, z, ...
            % launch, land]
            oct_angles = [oct_angles', ...
                [1,0,0,1,0;...
                0,1,0,0,1;...
                0,1,0,1,0;...
                -1,0,0,0,1;...
                -1,0,0,1,0;...
                0,-1,0,0,1;...
                0,-1,0,1,0;...
                1,0,0,0,1]];
            for i = 1:num_verts
                vert = node_table{i,{'XData','YData','ZData'}};
                node_ang = atan2(vert(1),vert(2)) *180/pi;
                wing_ind = find(abs(node_ang-oct_angles(:,1)) < 10*eps);
                wing_vec = vert + oct_angles(wing_ind,2:4)*lane_length;
                ground_vec = [wing_vec(1:2),0];
                
                % Create the new nodes
                new_nodes = [ [wing_vec,0,0]; ...
                    [ground_vec,oct_angles(wing_ind,5,8)] ];
                node_table = [node_table; new_nodes];
                % Create the new edges
                new_edges = [
            end
            % Instantiate an LBSD object
            lbsd = LBSD();
            
            % Initialize the LBSD object with the generated lane graph
            lbsd.lane_graph = digraph(edge_table, node_table);
        end
    end
end

