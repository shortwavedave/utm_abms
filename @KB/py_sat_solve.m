function [satisfiable, models] = py_sat_solve(kb, rtn_all_models)
%PY_SAT_SOLVE Summary of this function goes here
%   Detailed explanation goes here
    sat_instance = py.satinstance.SATInstance();
    num_clauses = size(kb.m_logic_mat, 1);
    for i=1:num_clauses
        clause = kb.indModel2Model(kb.m_logic_mat(i,:));
        clause_ln = strjoin(clause);
        sat_instance.parse_and_add_clause(clause_ln);
    end
    assignments = py.sat.run_solver_inst(sat_instance);
    cellP = cellfun(@char,cell(assignments),'UniformOutput',false);
    satisfiable = ~isempty(cellP);
    if ~rtn_all_models
        models = kb.clause2IndClause(split(cellP{1})', true);
    else
        cellax = cellfun(@(x) kb.clause2IndClause(split(x)', true),...
            cellP, 'UniformOutput', false);
        models = cell2mat(cellax');
    end
end

