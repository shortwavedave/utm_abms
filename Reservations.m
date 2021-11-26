classdef Reservations < handle
    %RESERVATION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        id
        lane_id
        uas_id
        entry_time_s
        exit_time_s
        speed
        hd
        length = 0
        next_tbl_row = 1
        next_res_id = 1
    end
    
    properties (Access = protected)
        preallocate = 100
        reservations
    end
    
    methods
        function obj = Reservations()
            %RESERVATION Construct an instance of this class
            %   Detailed explanation goes here
            obj.setPreallocations(obj.preallocate);
        end
        
        function varargout = subsref(obj,s)
           switch s(1).type
              case '.'
                    [varargout{1:nargout}] = builtin('subsref',obj,s);
              case '()'
                 if length(s) == 1
                    % Implement obj(indices)
                    error
%                  elseif length(s) == 2 && strcmp(s(2).type,'.')
%                     % Implement obj(ind).PropertyName
%                     ...
%                  elseif length(s) == 3 && strcmp(s(2).type,'.') && strcmp(s(3).type,'()')
%                     % Implement obj(indices).PropertyName(indices)
%                     ...
                 else
                    % Use built-in for any other expression
                    [varargout{1:nargout}] = builtin('subsref',obj,s);
                 end
              case '{}'
                 if length(s) == 1
                    % Implement obj{indices}
                    ...
                 elseif length(s) == 2 && strcmp(s(2).type,'.')
                    % Implement obj{indices}.PropertyName
                    ...
                 else
                    % Use built-in for any other expression
                    [varargout{1:nargout}] = builtin('subsref',obj,s);
                 end
              otherwise
                 error('Not a valid indexing expression')
           end
        end
        
        function obj = subsasgn(obj,s,varargin)

           % Allow subscripted assignment to uninitialized variable
           if isequal(obj,[])
              % obj = ClassName.empty;
           end

           switch s(1).type
              case '.'
                 if length(s) == 1
                    % Implement obj.PropertyName = varargin{:};
                    ...
                 elseif length(s) == 2 && strcmp(s(2).type,'()')
                    % Implement obj.PropertyName(indices) = varargin{:};
                    ...
                 else
                    % Call built-in for any other case
                    obj = builtin('subsasgn',obj,s,varargin{:});
                 end
              case '()'
                 if length(s) == 1
                    % Implement obj(indices) = varargin{:};
                 elseif length(s) == 2 && strcmp(s(2).type,'.')
                    % Implement obj(indices).PropertyName = varargin{:};
                    ...
                 elseif length(s) == 3 && strcmp(s(2).type,'.') && strcmp(s(3).type,'()')
                    % Implement obj(indices).PropertyName(indices) = varargin{:};
                    ...
                 else
                    % Use built-in for any other expression
                    obj = builtin('subsasgn',obj,s,varargin{:});
                 end       
              case '{}'
                 if length(s) == 1
                    % Implement obj{indices} = varargin{:}
                    ...
                 elseif length(s) == 2 && strcmp(s(2).type,'.')
                    % Implement obj{indices}.PropertyName = varargin{:}
                    ...
                    % Use built-in for any other expression
                    obj = builtin('subsasgn',obj,s,varargin{:});
                 end
              otherwise
                 error('Not a valid indexing expression')
           end
        end
        
        function tbl = getReservationTable(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            obj.reservations.id = obj.id(1:obj.length);
            obj.reservations.lane_id = obj.lane_id(1:obj.length);
            obj.reservations.uas_id = obj.uas_id(1:obj.length);
            obj.reservations.entry_time_s = obj.entry_time_s(1:obj.length);
            obj.reservations.exit_time_s = obj.exit_time_s(1:obj.length);
            obj.reservations.speed = obj.speed(1:obj.length);
            obj.reservations.hd = obj.hd(1:obj.length);
            tbl = obj.reservations;
        end
        
        function clearReservations(obj)
            obj.reservations = table( 'Size',[obj.preallocate 7], ...
                'VariableNames', {'id','lane_id','uas_id', ...
                'entry_time_s', 'exit_time_s', 'speed', 'hd'}, ...
                'VariableTypes',{'string','string','string','double','double', ...
                'double', 'double'} );
            
            p = obj.preallocate;
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
            
            obj.next_res_id = 1;
            obj.next_tbl_row = 1;
        end
    end
    
    methods (Access = protected)
        function setPreallocations(obj, n)
            obj.preallocate = n;
            obj.clearReservations();
        end
    end
end

