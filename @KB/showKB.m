function showKB(kb)
    %SHOWKB Print the KB
    % On input:
    %     kb (instance of KB): knowledge base
    % Call:
    %    kb.showKB();
    % Author:
    %     D. Sacharny
    %     UU
    %     Spring 2021
    %
    sz = size(kb.m_logic_mat);
    num_clauses = sz(1);
    disp(' ');
    for i = 1:num_clauses
        clause = kb.indModel2Model(kb.m_logic_mat(i,:));
        disp(clause);
    end
end 

