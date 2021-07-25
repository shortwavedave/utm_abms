function [satisfiable, models] = c_sat_solve(kb, rtn_all_models)
%C_SAT_SOLVE Summary of this function goes here
%   Detailed explanation goes here
    sat_init;
    [satisfiable, model] = sat_solve(kb.m_logic_mat);
    sat_done;
    models = [model];
    if rtn_all_models
        warning('The c-sat library does not support returning all models');
    end
end

