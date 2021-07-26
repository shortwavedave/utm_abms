function [satisfiable, models] = runSATSolver(kb, rtn_all_models)
    if nargin < 2
        rtn_all_models = false;
    end
    [satisfiable, models] = kb.m_sat_solver(kb, rtn_all_models);
end

