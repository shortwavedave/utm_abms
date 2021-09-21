classdef Intervals < handle
    %INTERVALS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        intervals = []
    end
    
    properties (Access = protected)
        m_intervals = []
        num_intervals = 0
        preallocate_inc = 0
    end
    
    methods
        function obj = Intervals(preallocation)
            if nargin == 0
                preallocation = 100;
            end
            obj.preallocate_inc = preallocation;
            obj.preallocate(preallocation);
        end
        
        function ints = get.intervals(obj)
            ints = obj.m_intervals(1:obj.num_intervals,:);
        end
        
        function intervals = union(obj, interval)
            a = interval(1);
            b = interval(2);
            
            for i = 1:obj.num_intervals
                c = obj.m_intervals(i,1);
                d = obj.m_intervals(i,2);
                if a < c
            end
        end
    end
    
    methods (Access = protected)
        function preallocate(obj, num)
            obj.m_intervals = [obj.m_intervals; zeros(num, 2)];
        end
        
        function allocate_append(obj, val)
            if obj.num_intervals < size(obj.m_intervals,1)
                obj.m_intervals(obj.num_intervals+1,:) = val;
            else
                preallocate(obj, obj.preallocate_inc);
                obj.m_intervals(obj.m_intervals+1,:) = val;
            end
            obj.num_intervals = obj.num_intervals + 1;
        end
        
        function allocate_prepend(obj, val)
            if obj.num_intervals >= len(obj.m_intervals)
                preallocate(obj, obj.preallocate_inc);
            end
             
            obj.m_intervals = circshift(obj.m_intervals,1);
            obj.m_intervals(1,:) = val;
            obj.num_intervals = obj.num_intervals + 1;
        end
        
        function remove_inds(obj, inds)
            obj.m_intervals(inds,:) = [];
            obj.num_intervals = obj.num_intervals - len(inds);
        end
        
        function allocate_insert(obj, ind, val)
            % allocate_insert(ind, val) Insert an interval at index
            % This method will insert an interval at the given index and
            % shift the existing member and tail down
            if obj.num_intervals >= len(obj.m_intervals)
                preallocate(obj, obj.preallocate_inc);
            end

            obj.m_intervals = [obj.m_intervals(1:ind-1,:);val; ...
                obj.m_intervals(ind:end-1,:)];
            obj.num_intervals = obj.num_intervals + 1;
        end
    end
end

