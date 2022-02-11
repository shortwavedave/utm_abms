classdef DisjointIntervals < handle
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
        function obj = DisjointIntervals(preallocation)
            if nargin == 0
                preallocation = 100;
            end
            
            obj.preallocate_inc = preallocation;
            obj.preallocate(preallocation);
        end
        
        function setIntervals(obj, intervals)
            sz = size(intervals,1);
            if sz > size(obj.m_intervals, 1)
                obj.preallocate(sz-size(obj.m_intervals, 1));
            end
            obj.m_intervals(1:sz,:) = intervals;
            obj.num_intervals = sz;
            obj.sort_ints();
        end
        
        function ints = get.intervals(obj)
            ints = obj.m_intervals(1:obj.num_intervals,:);
        end
        
        function intervals = union(obj, interval)
            a = interval(1);
            b = interval(2);
            n = obj.num_intervals;
            % TODO: Optimize, probably via binary search
            if n > 0
%                 m = obj.m_intervals(1:n,:);
                m = obj.m_intervals;
                m_n = m(1:n,:);
                inds = find(a<=m_n(:,2) & b>=m_n(:,1));
%                 inds = find(a<=obj.m_intervals(1:n,2) & b>=obj.m_intervals(1:n,1));
%                 i_1 = find(a<=obj.m_intervals(1:n,2) & b>=obj.m_intervals(1:n,1),1,'first');
%                 i_2 = find(a<=obj.m_intervals(1:n,2) & b>=obj.m_intervals(1:n,1),1,'last');
%                 inds = i_1:i_2;
                if ~isempty(inds)
                    % The input interval start is within another interval
                    if a >= m(inds(1),1)  
                        a = m(inds(1),1);
                    end

                    % The input interval end is within another interval
                    if b <= m(inds(end),2)
                        b = m(inds(end),2);
                    end

                    % Does the input interval span multiple intervals?
                    if length(inds) > 1
                        
                        m(inds,:) = [];
                        obj.num_intervals = obj.num_intervals - length(inds);
            %             obj.m_intervals = [obj.m_intervals(1:ind-1,:);val; ...
            %                 obj.m_intervals(ind:end-1,:)];
                        if obj.num_intervals >= size(m,1)
                            m = [m; zeros(obj.preallocate_inc, 2)];
                        end
                        ind = inds(1);
                        m(ind:end,:) = [a b; m(ind:end-1,:)];
                        
%                         m = [m(1:ind-1,:);[a b]; m(ind:end-1,:)];
                        obj.m_intervals = m;
                        obj.num_intervals = obj.num_intervals + 1;
                        intervals = m(1:obj.num_intervals,:);
%                         obj.remove_inds(inds);
%                         obj.allocate_insert(inds(1), [a,b]);
                    else % Either extend one or leave alone
                        obj.m_intervals(inds(1),:) = [a, b];
                        intervals = obj.m_intervals(1:obj.num_intervals,:);
                    end
                else
                    if b < obj.m_intervals(1,1)
                        obj.allocate_prepend([a,b]);
                        intervals = obj.m_intervals(1:obj.num_intervals,:);
                    elseif a > obj.m_intervals(n,2)
                        obj.allocate_append([a,b]);
                        intervals = obj.m_intervals(1:obj.num_intervals,:);
                    else
                        % In between intervals
                        inds = find(b<m(:,1),1);
                        if ~isempty(inds)
%                             obj.allocate_insert(inds(1), [a,b]);
                            if obj.num_intervals >= size(m,1)
                                m = [m; zeros(obj.preallocate_inc, 2)];
                            end
                            ind = inds(1);
                            m(ind:end,:) = [a b; m(ind:end-1,:)];
%                             m = [m(1:ind-1,:);[a b]; m(ind:end-1,:)];
                            obj.m_intervals = m;
                            obj.num_intervals = obj.num_intervals + 1;
                            intervals = m(1:obj.num_intervals,:);
                        else
                            error("Unexpected Case");
                        end
                    end
                end
            else
                obj.allocate_append([a,b]);
                intervals = obj.m_intervals(1:obj.num_intervals,:);
            end
            
        end
    end
    
    methods (Access = protected)
        function sort_ints(obj)
            [~,I] = sort(obj.intervals(:,1));
            obj.m_intervals(1:obj.num_intervals,:) = obj.m_intervals(I,:);
        end
        
        function preallocate(obj, num)
            obj.m_intervals = [obj.m_intervals; zeros(num, 2)];
        end
        
        function allocate_append(obj, val)
            if obj.num_intervals < size(obj.m_intervals,1)
                obj.m_intervals(obj.num_intervals+1,:) = val;
            else
                preallocate(obj, obj.preallocate_inc);
                obj.m_intervals(obj.num_intervals+1,:) = val;
            end
            obj.num_intervals = obj.num_intervals + 1;
        end
        
        function allocate_prepend(obj, val)
            if obj.num_intervals >= length(obj.m_intervals)
                preallocate(obj, obj.preallocate_inc);
            end
             
            obj.m_intervals = circshift(obj.m_intervals,1);
            obj.m_intervals(1,:) = val;
            obj.num_intervals = obj.num_intervals + 1;
        end
        
        function remove_inds(obj, inds)
            obj.m_intervals(inds,:) = [];
            obj.num_intervals = obj.num_intervals - length(inds);
        end
        
        function allocate_insert(obj, ind, val)
            % allocate_insert(ind, val) Insert an interval at index
            % This method will insert an interval at the given index and
            % shift the existing member and tail down
            if obj.num_intervals >= size(obj.m_intervals,1)
                preallocate(obj, obj.preallocate_inc);
            end
            m = obj.m_intervals;
%             obj.m_intervals = [obj.m_intervals(1:ind-1,:);val; ...
%                 obj.m_intervals(ind:end-1,:)];
            m(ind:end,:) = [val; ...
                m(ind:end-1,:)];
            obj.m_intervals = m;
            obj.num_intervals = obj.num_intervals + 1;
        end
    end
end

