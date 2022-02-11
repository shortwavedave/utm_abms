classdef Reservations
    %RESERVATION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods (Static)   
        
        function obj = newReservations(preallocated)
            obj.preallocated = preallocated;
            obj = Reservations.clearReservations(obj);
        end
        
        function tbl = getReservationTable(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            if obj.length > 0
                id = obj.id(1:obj.length);
                lane_id = obj.lane_id(1:obj.length);
                uas_id = obj.uas_id(1:obj.length);
                entry_time_s = obj.entry_time_s(1:obj.length);
                exit_time_s = obj.exit_time_s(1:obj.length);
                speed = obj.speed(1:obj.length);
                hd = obj.hd(1:obj.length);
                tbl = table(id', lane_id', uas_id', entry_time_s', ...
                    exit_time_s', speed', hd', ...
                    'VariableNames', {'id','lane_id','uas_id', ...
                'entry_time_s', 'exit_time_s', 'speed', 'hd'});
            else
                tbl = [];
            end
        end
        
        function tbl = getRes(obj, row)
            if row >0 && row <= obj.length
                id = obj.id(row);
                lane_id = obj.lane_id(row);
                uas_id = obj.uas_id(row);
                entry_time_s = obj.entry_time_s(row);
                exit_time_s = obj.exit_time_s(row);
                speed = obj.speed(row);
                hd = obj.hd(row);
                tbl = table(id', lane_id', uas_id', entry_time_s', ...
                    exit_time_s', speed', hd', ...
                    'VariableNames', {'id','lane_id','uas_id', ...
                'entry_time_s', 'exit_time_s', 'speed', 'hd'});
            else
                tbl = [];
            end
        end
        
        function obj = clearReservations(obj)           
            p = obj.preallocated;
            t_id(p) = "";
            obj.id= t_id;
            
            t_lane_id(p) = "";
            obj.lane_id = t_lane_id;
            
            t_uas_id(p) = "";
            obj.uas_id = t_uas_id;
            
            t_entry_time_s(p) = 0;
            obj.entry_time_s = t_entry_time_s;
            
            t_exit_time_s(p) = 0;
            obj.exit_time_s = t_exit_time_s;
            
            t_speed(p) = 0;
            obj.speed = t_speed;
            
            t_hd(p) = 0;
            obj.hd = t_hd;
            
            obj.length = 0;
        end
        
        function obj = appendPreallocation(obj, n)
            old_n = obj.preallocated;
            inds = old_n+(1:n);
            
            t_id(n) = "";
            obj.id(inds) = t_id;
            
            t_lane_id(n) = "";
            obj.lane_id(inds) = t_lane_id;
            
            t_uas_id(n) = "";
            obj.uas_id(inds) = t_uas_id;
            
            t_entry_time_s(n) = 0;
            obj.entry_time_s(inds) = t_entry_time_s;
            
            t_exit_time_s(n) = 0;
            obj.exit_time_s(inds) = t_exit_time_s;
            
            t_speed(n) = 0;
            obj.speed(inds) = t_speed;
            
            t_hd(n) = 0;
            obj.hd(inds) = t_hd;
            
            obj.preallocated = old_n+n;
        end
    end
    
end

