function [atom_probs] = atomProbsFromMonteCarlo(kb, num_trials)
%ATOMPROBSFROMMONTECARLO Summary of this function goes here
%   Detailed explanation goes here
    clause_probs = kb.m_clause_probs;
    num_clauses = length(clause_probs);
    atom_probs = zeros(1, kb.m_num_atoms);
    %num_trials = 10;
    num_models = 0;
    f = waitbar(0,'Please wait...');
%     WaitMessage = parfor_wait(num_trials, 'Waitbar', true);
    tic
    for i = 1:num_trials
%     parfor i = 1:num_trials
%         WaitMessage.Send;
       if mod(i,100) == 0
           waitbar(i/num_trials,f,...
               ['Trial: ', num2str(i), '/', num2str(num_trials)]);
       end
       kbc = KB.clone(kb);
       negate_clauses = rand(1,num_clauses);
       negate_clauses = find(negate_clauses > clause_probs);
       for n = negate_clauses
           kbc.negateClause(n);
       end
%        [s, model, ~] = kbc.askClause({});
         [s, models] = kbc.py_sat_solve(kbc, true);
       % If the kb is satisfiable, count
       if s 
            num_models = num_models + size(models,1);
            atom_probs = atom_probs + sum(models > 0, 1)/size(models,1);
%             atom_probs = atom_probs + sum(models > 0, 1);
       end
    end
    toc
    close(f);
%     WaitMessage.Destroy;
    atom_probs = atom_probs / num_trials;
end

