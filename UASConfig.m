classdef UASConfig < handle
    %UASCONFIG Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % The number of UAS 
        num_uas = 10
    end
    
    properties (Dependent)
        speed_value
        climb_value
        hd_value
    end
    
    properties (Access = protected)
        speed_fn = []
        speed_type = "CONSTANT"
        m_speed_value = 1.0
        
        climb_fn = []
        climb_type = "CONSTANT"
        m_climb_value = 1.0
        
        hd_fn = []
        hd_type = "CONSTANT"
        m_hd_value = 10.0
    end
    
    methods
        function obj = UASConfig()
            %UASCONFIG Construct an instance of this class
        end
        
        function s = get.speed_value(obj)
            s = obj.m_speed_value;
        end
        
        function s = get.climb_value(obj)
            s = obj.m_climb_value;
        end
        
        function s = get.hd_value(obj)
            s = obj.m_hd_value;
        end
        
        function setHeadwayMix(obj, type, value)
            %setHeadwayMix Configure how this object sets uas headways
            %
            % On Input:
            %   type (string): type is either CONSTANT, UNIFORM, or
            %       GAUSSIAN
            %   value (float): for CONSTANT type, every uas has the same
            %   speed that is set to this value. For UNIFORM and GAUSSIAN
            %   type, this value represents the mean. By default the
            %   standard deviation is set to 1% of this value
            if type == "UNIFORM" || type == "GAUSSIAN"
                error("Not Implemented")
            end
            obj.m_hd_value = value;
        end
        
        function setSpeedMix(obj, type, value)
            %setSpeedMix Configure how this object sets uas speeds
            %
            % On Input:
            %   type (string): type is either CONSTANT, UNIFORM, or
            %       GAUSSIAN
            %   value (float): for CONSTANT type, every uas has the same
            %   speed that is set to this value. For UNIFORM and GAUSSIAN
            %   type, this value represents the mean. By default the
            %   standard deviation is set to 1% of this value
            if type == "UNIFORM" || type == "GAUSSIAN"
                error("Not Implemented")
            end
            obj.m_speed_value = value;
        end
        
        function setClimbRateMix(obj, type, value)
            %setClimbRateMix Configure how this object sets uas climb rates
            %
            % On Input:
            %   type (string): type is either CONSTANT, UNIFORM, or
            %       GAUSSIAN
            %   value (float): for CONSTANT type, every uas has the same
            %   speed that is set to this value. For UNIFORM and GAUSSIAN
            %   type, this value represents the mean. By default the
            %   standard deviation is set to 1% of this value
            if type == "UNIFORM" || type == "GAUSSIAN"
                error("Not Implemented")
            end
            obj.m_climb_value = value;
        end
        
        function configureUAS(obj, uas)
            if ~isa(uas, 'UAS')
                error("uas argument must be object of class UAS");
            end
            obj.setUASSpeed(uas);
            obj.setUASClimbRate(uas);
            obj.setUASHeadway(uas);
        end

    end
    
    methods (Access = protected)
                
        function setUASSpeed(obj,uas)
            %setUASSpeed Summary of this method goes here
            %   Detailed explanation goes here
            if obj.speed_type == "CONSTANT"
                uas.nominal_speed = obj.m_speed_value;
            else
                error("Not Implemented")
            end
        end
        
        function setUASClimbRate(obj,uas)
            %setUASSpeed Summary of this method goes here
            %   Detailed explanation goes here
            if obj.climb_type == "CONSTANT"
                uas.climb_rate = obj.m_climb_value;
            else
                error("Not Implemented")
            end
        end
        
        function setUASHeadway(obj,uas)
            %setUASHeadway Summary of this method goes here
            %   Detailed explanation goes here
            if obj.hd_type == "CONSTANT"
                uas.h_d = obj.m_hd_value;
            else
                error("Not Implemented")
            end
        end
    end
end

