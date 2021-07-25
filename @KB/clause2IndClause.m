function [ind_clause, is_new] = clause2IndClause(kb, clause, ask_only)
    % clause2IndClause Convert a clause into the internal
    % representation.
    % Description:
    %   [ind_clause, is_new] = clause2IndClause(kb, clause, ask_only)
    % On Input:
    %   kb - an instance of KB
    %   clause - a char cell array of literals
    %   ask_only - optional, default false. if true, does not add
    %       new atoms to the KB (important for asks)
    %
    % On Output:
    %   ind_clause - the internal representation of the clause
    %
    % Example:
    %   ind_clause = kb.clause2IndClause({'uas nominal', ...
    %   'assigned'}
    if nargin < 3
        ask_only = false;
    end
    ind_clause = zeros(1, kb.m_num_atoms);
    for literal = clause
        logic_val = 1;
        atom = literal;
        if contains(literal, kb.m_negation_char)
            atom = extractAfter(literal, kb.m_negation_char);
            logic_val = -1;
        end

        [ind, is_new] = kb.getKBAtomIndex(atom, ask_only);
        if (ask_only && ~is_new) || (~ask_only)
            % New atoms increment the index, so can assume it goes on
            % the end if it's new.
            ind_clause(ind) = logic_val;
        end
    end
end
