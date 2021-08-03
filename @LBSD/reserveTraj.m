function [ok, res_ids] = reserveTraj(obj, lane_ids, entry_times, ...
            exit_times, speeds, hds, )
%RESERVETRAJ Reserve a trajectory
%   This method checks that the lane_id is valid and that the
%   requested reservation does not overlap entry or exit time 
%   within headway distance.
%   On Input:
%       lane_id %(string): The lane that is being reserved
%       entry_time_s %(float): entry time from the lane in seconds 
%       exit_time_s %(float): exit time from the lane in seconds 
%       speed %(float): speed in m/s
%       hd %(float): required headway distance 
%   On Output:
%       ok: true if the reservation was made successfully
%       res_id: (string) reservation id. empty string if
%       ok is false.
%   Call:
%       res = lbsd.makeReservation("1",0,10,10,10)
%

end

