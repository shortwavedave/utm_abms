function row = getMatchClause(kb, ind_clause)
    % getMatchClause Get the row index of a matching clause in
    % ind_clause form.
    % Description:
    %   row = getMatchClause(kb, ind_clause)
    % On Input:
    %   kb - instance of the KB
    %   ind_clause - a clause where the column index is the atom
    %   index, with 0 representing an atom not included in the
    %   clause, a 1 indicating a positive literal, and a 0
    %   indicating a negative literal.
    % On Output:
    %   row - the row index of the clause else an empty array
    %
    % Example:
    %   row = kb.getMatchClause([-1 1 0 1]);
    row = find( ismember(kb.m_logic_mat, ind_clause, 'rows'), 1);
end