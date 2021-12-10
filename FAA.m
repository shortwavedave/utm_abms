classdef FAA < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        verts = []
        lanes = []
        vert_len = 0
        lane_len = 0
        vert_prealloc = 10000
        lane_prealloc = 10000
        up_alt_m = 200;
        down_alt_m = 150;
        minx = 0;
        maxx = 5000;
        maxy = 5000;
        cell_sz = 500;
        ops = []
        ops_prealloc = 10000
        cells = []
        cell_boundaries = []
    end
    
    methods
        function obj = FAA()
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.verts = zeros(obj.vert_prealloc, 3);
            obj.lanes = zeros(obj.lane_prealloc, 2);
            num_cells = obj.maxx/obj.cell_sz;
            cell.ops = zeros(1,obj.ops_prealloc);
            cell.bbox = [0 0 0 0]; % minx,miny,maxx,maxy
            cell.num_ops = 0;
            cells(num_cells) = cell;
            obj.cells = cells;
            i = 0;
            for r = 0:num_cells-1
                for c = 0:num_cells-1
                    i = i + 1;
                    obj.cells(i).bbox = [c*obj.cell_sz, r*obj.cell_sz, ...
                        (c+1)*obj.cell_sz, (r+1)*obj.cell_sz];
                end
            end
            obj.cell_boundaries = ...
                reshape([obj.cells.bbox],4,length(obj.cells))';
        end
        
        function ids = getClosestLaunchVerts(obj, q)
            % getClosestLaunchVerts Get the Vertex IDs of the closest
            % launch 
            % nodes. 
            % On Input:
            %   q - nx2 query points. Each row is [x,y]
            % On Output:
            %   ids - (nx1 string array) ids of the closest launch vertex
            % Call:
            %   ids = lbsd.getClosestLaunchVerts([0 0; 5 5; -30 -15])
            l_i = obj.vert_len+1;
            l_i_e = l_i+size(q,1)-1;
            ids = l_i:l_i_e;
            if l_i_e > size(obj.verts,1)
                obj.verts = [obj.verts; zeros(obj.vert_prealloc,3)];
            end
            if size(q,2) == 2
                obj.verts(l_i:l_i_e,1:2) = q;
            else
                obj.verts(l_i:l_i_e,:) = q;
            end
            obj.vert_len = l_i_e;
        end
        
        function ids = getClosestLandVerts(obj, q)
            % getClosestLandVerts Get the Vertex IDs of the closest land
            % nodes. 
            % On Input:
            %   q - nx2 query points. Each row is [x,y]
            % On Output:
            %   ids - (nx1 string array) ids of the closest launch vertex
            % Call:
            %   ids = lbsd.getClosestLandVerts([0 0; 5 5; -30 -15])
            l_i = obj.vert_len+1;
            l_i_e = l_i+size(q,1)-1;
            ids = l_i:l_i_e;
            if l_i_e > size(obj.verts,1)
                obj.verts = [obj.verts; zeros(obj.vert_prealloc,3)];
            end
            if size(q,2) == 2
                obj.verts(l_i:l_i_e,1:2) = q;
            else
                obj.verts(l_i:l_i_e,:) = q;
            end
            obj.vert_len = l_i_e;
        end
        
        function [minx, miny, maxx, maxy] = getEnvelope(~)
            minx = obj.minx;
            miny = obj.miny;
            maxx = obj.maxx;
            maxy = obj.maxy;
        end
        
        function h = plot(obj)
            for l_i = 1:obj.lane_len
                pos = obj.verts(obj.lanes(l_i,:),:);
                if l_i == 1
                    h=plot3(pos(:,1),pos(:,2),pos(:,3),'LineWidth',1.5);
                else
                    hold on
                    plot3(pos(:,1),pos(:,2),pos(:,3),'LineWidth',1.5);
                    hold off;
                end
            end
            xlabel("X");
            ylabel("Y");
            zlabel("Z");
        end
        
        function [lane_ids, vert_ids, dist] = getShortestPath(obj, ...
                start_vert_id, end_vert_id)
            % getShortestPath Get a shortest path between two vertexes 
            % On Input:
            %   start_vert_id - (string) the id of the start vertex
            %   end_vert_id - (string) the id of the end vertex
            % On Output:
            %   lane_ids - (nx1 string array) ids of lanes from start to
            %       end.
            %   vert_ids - (nx1 string array) ids of the vertexes from
            %       start to end
            %   distance - the distance covered by the path
            % Call:
            %   [lane_ids, vert_ids, dist] = lbsd.getShortestPath("12","22")
            
            % Create up over down
            alt_m = obj.down_alt_m + (obj.up_alt_m - obj.down_alt_m)*rand();
            start_pos = obj.verts(start_vert_id,:);
            end_pos = obj.verts(end_vert_id,:);
            
            % Create two new vertexes
            l_i = obj.vert_len+1;
            l_i_e = l_i+1;
            vert_ids = [ start_vert_id l_i:l_i_e end_vert_id ]';
            if l_i_e > size(obj.verts,1)
                obj.verts = [obj.verts; zeros(obj.vert_prealloc,3)];
            end
            obj.verts(l_i,:) = start_pos + [0 0 alt_m];
            obj.verts(l_i_e,:) = end_pos + [0 0 alt_m];
            obj.vert_len = l_i_e;
            
            if obj.lane_len+3 > size(obj.lanes,1)
                obj.lanes = [obj.lanes; zeros(obj.lane_prealloc,2)];
            end
            
            lane_ids = zeros(3,1);
            lane_1 = [start_vert_id l_i];
            obj.lanes(obj.lane_len+1,:) = lane_1;
            obj.lane_len = obj.lane_len+1;
            lane_ids(1) = obj.lane_len;
            
            lane_2 = [l_i l_i_e];
            obj.lanes(obj.lane_len+1,:) = lane_2;
            obj.lane_len = obj.lane_len+1;
            lane_ids(2) = obj.lane_len;
            
            lane_3 = [l_i_e end_vert_id];
            obj.lanes(obj.lane_len+1,:) = lane_3;
            obj.lane_len = obj.lane_len+1;
            lane_ids(3) = obj.lane_len;
            
            dist = norm(obj.verts(vert_ids(1),:)-obj.verts(vert_ids(2),:)) + ...
                norm(obj.verts(vert_ids(2),:)-obj.verts(vert_ids(3),:)) + ...
                norm(obj.verts(vert_ids(3),:)-obj.verts(vert_ids(4),:));
        end
        
        function positions = getVertPositions(obj, ids)
            % getVertPositions Get the Positions of Vertexes in the lane 
            %   system by ID
            % On Input:
            %   ids - nx1 string array or ':' for all
            % On Output:
            %   positions - nx3 positions of vertexes
%             rows = ismember(obj.node_table.Properties.RowNames, ids);
%             x = obj.node_table.XData(rows);
%             y = obj.node_table.YData(rows);
%             z = obj.node_table.ZData(rows);
            positions = obj.verts(ids,:);
%             positions = [x y z];
        end
        
        function [ok, res_ids, res_toa_s] = ...
                reserveTrajectory(obj, lane_ids, uas_id, toa_s, ...
                h_d, r_e, r_l)
            % reserveLBSDTrajectory Reseave a sequence of lanes
            %	This method takes lane ids, time-of-arrival and departure
            %	for each lane, required headway distance, earliest launch
            %	time, and latest launch time, and returns an array of
            %	reservation ids and reserved time-of-arrival and
            %	departures.
            %   On Input:
            %       lane_ids - [nx1] string of lane identifiers
            %       uas_id - string the UAS identifier
            %       toa_s - [(n+1)x1] float seconds arrival at each vertex
            %       h_d - float headway distance in meters
            %       r_e - float seconds earliest release (launch) time 
            %           desired
            %       r_l - float seconds latest release (launch) time
            %           desired
            %   On Output:
            %       res_ids - [nx1] string reservation ids for each lane
            %       res_toa_s - [(nx1)x1] float seconds reserved arrival at
            %           at each vertex in the reserved lane sequence
            %       ok - true if the flight was successfully scheduled
            %   Call:
            %       [ok, res_ids, res_toa_s] = ...
            %               lbsd.reserveLBSDTrajectory(["1"], [1,5], ...
            %                   10, 0, 10)
            lanes = obj.lanes(lane_ids,:);
            % First find the relevant cells
            cells = [];
            cell_is = [];
            for i = 1:size(lanes,1)
                v1 = lanes(i,1);
                v2 = lanes(i,2);
                n = ceil(norm(v2-v1)/100);
                lx = linspace(v1(1),v2(1), min(n,3));
                ly = linspace(v1(1),v2(1), min(n,3));
                % Find the cells
                c = (lx >= b(:,1)) & (ly >= b(:,2)) & ...
                    (lx < b(:,3)) & (ly < b(:,4));
                [cell_i,~] = ind2sub(size(c),find(c));
                cells = [cells obj.cells(cell_i)];
                cell_is = [cell_is cell_i];
            end
            
            % Deconflict
            t_total = toa_s(end)-toa_s(1);
            lane_vert_ids = unique(reshape(lanes,1,numel(lanes)));
            l_e = lane_vert_ids(2:end);
            l_s = lane_vert_ids(1:end-1);
            vs = obj.verts(l_e,:) - obj.verts(l_s);
            dists = sqrt(dot(vs,vs,2));
            speeds = dists ./ t_total;
            
            % Create candidate operation
            op.toa_s = toa_s;
            op.uas_id = uas_id;
            op.lane_ids = lane_ids;
            op.hd = hd;
            op.r_e = r_e;
            op.r_l = r_l;
            op.speeds = speeds;
            
            other_ops = [cells.ops];
            
            conflicts = true;
            op_p = op;
            op_n = op;
            step = 0.5;
            max_iter = 10000;
            disable_p = false;
            disable_n = false;
            while conflicts && (~disable_p || ~disable_n)
                for op_i = 1:length(other_ops)
                    if op_p.toa_s(1) >= r_l
                        disable_p = true;
                    else
                        conflicts_p = obj.conflicts(op_p, other_ops(op_i));
                        if ~conflicts_p
                            op = op_p;
                            conflicts = false;
                            break;
                        else
                            op_p.toa_s = op_p.toa_s + step;
                        end
                    end
                    
                    if op_n.toa_s(1) <= r_e
                        disable_n = true;
                    else
                        conflicts_n = obj.conflicts(op_n, other_ops(op_i));
                        if ~conflicts_n
                            op = op_n;
                            conflicts = false;
                            break;
                        else
                            op_n.toa_s = op_n.toa_s - step;
                        end
                    end
                end
            end
            ok = ~conflicts;
            if ok
                res_toa_s = op.toa_s;
                for i = 1:length(cells)
                    cells(i).ops = cells(i).ops
        end
        
        function conflicts = conflicts(obj, op_a, op_b)
            conflicts = false;
            for lane_a_i = op_a.lane_ids
                for lane_b_i = op_b.lane_ids
                    P = obj.verts(obj.lanes(lane_a_i,:),:);
                    v = op_a.speeds(lane_a_i)*...
                        (P(2,:)-P(1,:))/norm(P(2,:)-P(1,:));
                    t_p = op_a.toa_s(lane_a_i);
                    Q = obj.verts(obj.lanes(lane_b_i,:),:);
                    w = op_b.speeds(lane_b_i)*...
                        (Q(2,:)-Q(1,:))/norm(Q(2,:)-Q(1,:));
                    t_q = op_b.toa_s(lane_b_i); 
                    
                    wv = w-v;
                    t = -dot((wv),(Q(1,:) - P(1,:) + (t_p*v - tq*w)))/...
                        dot(wv,wv);
                    if t < t_p
                        t = t_p;
                    elseif t > op_a.toa_s(end)
                        t = op_a.toa_s(end);
                    end
                    d = norm(Q(1,:)-P(1,:)+(t_p*v - t_q*w) + t*(w-v));
                    if d < opa_a.hd
                        conflicts = true;
                        return;
                    end
                end
            end
        end
    end
end

