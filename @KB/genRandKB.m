function kb = genRandKB(num_clauses, num_atoms, sat)
    %GENRANDKB Generate a random knowledge base
    % On input:
    %     num_clauses (int): number of clauses
    %     num_atoms (int): number of atoms
    %     sat (boolean): if true, generate a KB that is satisfiable
    % On output:
    %    kb (instance of KB): knowledge base
    % Call:
    %    kb = genRandKB(3, 5, false);
    % Author:
    %     D. Sacharny
    %     UU
    %     Spring 2021
    %
    is_sat = ~sat;
    kb = KB();
    max_attempts = 1000;
    attempts = 0;
    while (is_sat ~= sat && attempts < max_attempts)
        has_repeated = true;
        while has_repeated && attempts < max_attempts
            logic_mat = randi([-1,1],num_clauses,num_atoms);
            logic_mat = unique(logic_mat,'rows');
            has_repeated = size(logic_mat, 1) ~= num_clauses;
            attempts = attempts + 1;
        end
        kb.setRawKB(logic_mat);
        [is_sat, ~, ~] = kb.askClause({});
    end
    if is_sat ~= sat
        error('Unable to generate a KB with the given constraints')
    end
end

