classdef Plan < handle
    %PLAN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        name = ""
        complete = false
    end
    
    methods
        function obj = Plan()
            %PLAN Construct an instance of this class
            %   Detailed explanation goes here
        end
        
        function step(obj)
            %STEP Run the plan algorithm
            %   Detailed explanation goes here
        end
        
        function reset(obj)
            
        end
        
        function complete = isDone(obj)
            complete = obj.complete;
        end
    end
end

