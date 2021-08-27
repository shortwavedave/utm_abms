function [ok, res_ids] = reserveTraj(obj, lane_ids, earliest_release, ...
            latest_release, speeds, hds)
    %RESERVETRAJ Reserve a trajectory through the lane system
    %   This method attempts to schedule multiple lanes based on a possible
    %   interval of start times and the speeds and headways through lanes.
    %   On Input:
    %       lane_ids - (nx1 string array): The lanes to reserve
    %       earliest_release - (float): earliest time in seconds to launch
    %           from the first vertex of the first lane
    %       latest_release - (float): latest time in seconds to launch
    %           from the first vertex of the first lane 
    %       speeds - (float or nx1 floats): either the ground speed in m/s 
    %           through all the lanes, or the speed through each lane.
    %       hds - (float or nx1 floats): either the headway in m though
    %           all the lanes, or the headway through each lane.
    %   On Output:
    %       ok: true if the reservations were made successfully
    %       res_id: (nx1 string array) reservation ids. empty if
    %       ok is false.
    %   Call:
    %       [ok, res_ids] = lbsd.reserveTraj(["1","2"],0,10,10,10)
    %

    % First check if all the lane ids are present in the lane system
    if ~all(ismember(lane_ids, obj.lane_graph.Edges.Properties.RowNames))
        ok = false;
        res_ids = [];
        warning(['One or more of the provided lane ids are not in the',...
            ' Lane System']);
    else
        
    end
end

