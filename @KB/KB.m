classdef KB < handle
    % KB An instance of this class holds a knowledge base
    % Description:
    %   KB is a knowledge base for storing and reasoning over conjunctive
    %   normal form (CNF) sentences. The important methods for doing so are
    %   askClause and tellClause. To use this class, begin by constructing
    %   an object:
    %
    %   kb = KB();
    %
    %   Once you have an instance of this KB, you can store clauses in it
    %   by calling tellClause. For example, consider the case where you 
    %   want to store the clause:
    %
    %   'uas nominal if uas assigned and no errors'
    %
    %   The first step is to convert this proposition to CNF form:
    %
    %   'not uas nominal or uas assigned, and not uas nominal or no errors'
    %
    %   The next step is to convert this sentence into a cell array where
    %   each element in the array is a literal in the sentence. Negation,
    %   i.e. 'not', is represented by the tilde '~' by default - this can
    %   be changed by modifying the property m_negation_char. Since this
    %   CNF equivalent sentence is actually two sentences, we will call
    %   tellClause twice:
    %   
    %   kb.tellClause({'~uas nominal', 'assigned'});
    %   kb.tellClause({'~uas nominal', 'no errors'});
    %
    %   Now that the clauses are stored, you can view the KB by executing
    %
    %   kb.showKB();
    %
    %   You can also ask whether this KB entails clauses using askClause.
    %   askClause takes a clause to check and returns a boolean indicating
    %   whether the KB entails this sentence, a model that satisfies it,
    %   and whether this KB has the requisite knowledge to return a valid
    %   answer. If you askClause with an atom that does not exist in the
    %   KB, then has_knowledge returns false.
    %
    %   kb.tellClause({'assigned'});
    %   [s, model, h] = kb.askClause({'uas nominal'})
    %
    %   kb.tellClause({'~assigned'});
    %   [s, model, h] = kb.askClause({'uas nominal'})
    %
    % Example:
    %   kb = KB();
    %   kb.tellClause({'~uas nominal', 'assigned'});
    %   kb.tellClause({'~uas nominal', 'no errors'});
    %   kb.tellClause({'~assigned'});
    %   kb.showKB();
    %   [s, model, h] = kb.askClause({'uas nominal'})
    %
    % Author:
    %     David Sacharny
    %     University of Utah
    %     Spring 2020 
    
    properties
        % The character interpreted as a negation in a clause
        m_negation_char = '~' 
        % The internal representation of the KB. Each column corresponds to
        % an atom. Each row represents a clause. -1 represents a negative 
        % literal, 1 represents a positive literal, and 0 means an atom is 
        % not represented in a clause
        m_logic_mat = []
        % The number of atoms in this KB
        m_num_atoms = 0
        % Optional clause probabilities, index corresponds to m_logic_mat
        % row index.
        m_clause_probs = []
        % Optional atom probabilities, index corresponds to m_logic_mat
        % column index.
        m_atom_probs = []
    end
    
    properties (Access = protected)
        % A mapping between atoms and their column indexes for this KB
        m_atom_col_map
        % A mapping between column indexes and their atom for this KB
        m_col_atom_map
        % Indicates whether the internal logic matrix is sparse. This
        % property should not be modified externally
        m_is_sparse = false 
        % Handle to sat solver
        m_sat_solver = @c_sat_solve
    end
    
    methods (Access = public)
        function kb = KB()
            KB.initializeEnv(true, true, true);
            kb.initializeMaps();
        end
        
        [s, v] = runSATSolver(kb, rtn_all_models)
        
        showKB(kb)
        
        function showAtoms(kb)
            for i = 1:kb.m_num_atoms
                disp(kb.m_col_atom_map(uint64(i)));
            end
        end
        
        function setClauseProbs(kb, probs)
            %SETCLAUSEPROBS Set the clause probabilities
            % On input:
            %     kb (instance of KB): knowledge base
            %     probs (1xm): a vector of clause probabilities
            % Call:
            %    kb.setClauseProbs(probs);
            % Author:
            %     D. Sacharny
            %     UU
            %     Spring 2021
            %
            num_clauses = size(kb.m_logic_mat,1);
            if length(probs) == num_clauses
                if size(probs,1) > size(probs,2)
                    probs = probs';
                end
                kb.m_clause_probs = probs;
            else
                error(['probs must have ', num2str(num_clauses), ' elements']); 
            end
        end
        
        function negateClause(kb, clause_ind)
           if clause_ind > size(kb.m_logic_mat,1)
               error('clause_ind exceeds number of clauses');
           else
               clause = kb.m_logic_mat(clause_ind,:);
               kb.m_logic_mat(clause_ind,:) = [];
               new_clauses = diag(-clause);
               new_clauses = new_clauses(any(new_clauses,2),:);
               kb.m_logic_mat = [kb.m_logic_mat;new_clauses];
           end
        end
        
        function [c_probs, cc_probs, atom_probs] = getRandClauseProbs(kb)
            cc = 0:2^(kb.m_num_atoms)-1;
            cc_probs = rand(1,length(cc));
            cc_probs = cc_probs/sum(cc_probs);
            atom_probs = sum(de2bi(cc).*cc_probs',1);
            num_clauses = size(kb.m_logic_mat,1);
            c_probs = zeros(1,num_clauses);
            for clause_k = 1:num_clauses
                clause = kb.m_logic_mat(clause_k, :);
                inds = [];
                for li = 1:length(clause)
                   c = zeros(1,kb.m_num_atoms);
                   c(li) = 1;
                   v = bi2de(c);
                   
                   if clause(li) < 0
                       inds = [inds, find(~bitand(cc,v))];
                   elseif clause(li) > 0
                       inds = [inds, find(~~bitand(cc,v))];
                   end
                   inds = unique(inds);
                end
                c_probs(clause_k) = sum(cc_probs(inds));
            end
        end
        
        function setRawKB(kb, logic_mat)
            %SETRAWKB Set the internal logic matrix for this KB
            % The resulting knowledge base will contain arbitrary
            % characters representing each atom.
            % On input:
            %     kb (instance of KB): knowledge base
            %     logic_mat (mxn {0,-1,1}): a logic matrix where columns
            %     represent atoms, rows represent clauses, and elements
            %     represent literals: 0 if the atom is not in a clause, -1
            %     if it is negated, 1 if it is positive.
            % Call:
            %    kb.setRawKB([-1,1;0,1]);
            % Author:
            %     D. Sacharny
            %     UU
            %     Spring 2021
            %
            sz = size(logic_mat);
            kb.m_num_atoms = sz(2);
            %num_clauses = sz(1);
            kb.initializeMaps();
            kb.m_logic_mat = logic_mat;
            for i = 1:kb.m_num_atoms
                atom = ['v', num2str(i)];
                kb.m_atom_col_map(atom) = uint64(i);
                kb.m_col_atom_map(uint64(i)) = atom;
            end
        end
        
        function tellClause(kb, clause, force_append)
            % tellClause Tell the KB a CNF clause
            % Description:
            %   tellClause(kb, clause, force_append)
            %
            %   This function inserts a CNF clause into the knowledge-base.
            %   If the clause is composed of a single literal, then this
            %   clause will replace a single literal of the same atom in
            %   the KB. If the clause is exactly the same as an existing
            %   clause, then the KB is not updated.
            % On Input:
            %   clause - char cell array, e.g., {'~uas nominal','assigned'}
            %       Each element of the cell array represents a literal in
            %       a Conjunctive Normal Form (CNF) sentence. If the
            %       negation symbol (KB.m_negation_char, defaults to '~'),
            %       then the literal is split and stored internally as a
            %       negated atom.
            %
            %   force_append - (optional boolean), if true then new clause 
            %       will be
            %   appended to the KB no matter what.
            %
            % Example:
            %   my_kb = KB();
            %   my_kb.tellClause({'~uas nominal', 'assigned'})
            if nargin < 3
                force_append = false;
            end
            [ind_clause, ~] = kb.clause2IndClause(clause);
            kb.updateInsertClause(ind_clause, force_append);
        end
        
        function [entails, model, has_knowledge] = askClause(kb, ...
                clause, raw_model)
            % askClause Ask the KB if a CNF clause is satisfiable
            % Description:
            %   [s, model, has_knowledge] = askClause(kb, clause)
            %
            %   This function checks if every model satisfies
            %   a CNF clause given a knowledge-base. If the input clause
            %   does not exist in the KB, then a clone of the KB is created
            %   temporarily to test the assertion.
            % On Input:
            %   clause - char cell array, e.g., {'~uas nominal','assigned'}
            %       Each element of the cell array represents a literal in
            %       a Conjunctive Normal Form (CNF) sentence. If the
            %       negation symbol (KB.m_negation_char, defaults to '~'),
            %       then the literal is split and checked internally as a
            %       negated atom. If clause is an empty cell array, then
            %       the KB is checked for satisfiability.
            %   raw_model (boolean) - if true, then the internal clause
            %   representation is returned, otherwise the labels are
            % On Output:
            %   entails - (boolean) true if this clause is entailed
            %   model - if not entailed (entails=0) then this gives a possible
            %   reason. If the clause is entailed, then this is empty
            %   has_knowledge - boolean indicating whether all the atoms in
            %   the clause are represented in the knowledge base
            % Example:
            %   my_kb = KB();
            %   [s, model, h] = my_kb.askClause({'~uas nominal', 'assigned'})
            if nargin < 3
                raw_model = true;
            end
            has_knowledge = true;
            ask_only = true; % Don't store this clause
            v = zeros(1, kb.m_num_atoms);
            entails = 0;
            if ~isempty(clause)
                [ind_clause, is_new] = kb.clause2IndClause(clause, ask_only);
                has_knowledge = has_knowledge && ~is_new;
                if has_knowledge
                    % If this clause does not exist in the KB, then its 
                    % The negated clause must be added to check the 
                    % assertion. A temporary KB is created with the new 
                    % clause.
                    clone_kb = KB.clone(kb);
                    force_append = true;
                    % negate clause
                    n_ind_clauses = diag(-ind_clause);
                    n_ind_clauses = n_ind_clauses(any(n_ind_clauses,2),:);
                    for k_clause = 1:size(n_ind_clauses, 1)
                        clone_kb.updateInsertClause(n_ind_clauses(k_clause,:), ...
                            force_append);
                    end
                    [not_s, v] = clone_kb.runSATSolver();
                    % If the KB containing the negated clauses is
                    % satisfiable, then a model exists that does not
                    % assert the clause (stored in v). If not_s is false,
                    % and therefore every model asserts the clause, then
                    % the KB entails the clause.
                    entails = ~not_s;
                end
            else
                [entails, v] = kb.runSATSolver();
            end

            if raw_model
                model = v';
            else
                model = kb.indModel2Model(v);
            end
        end
    end
    
    methods (Access = protected)
        function initializeMaps(kb)
            %INITIALIZEMAPS clear and reinitialize map containers
            % On input:
            %     kb (instance of KB): knowledge base
            % Call:
            %    kb.initializeMaps();
            % Author:
            %     D. Sacharny
            %     UU
            %     Spring 2021
            %
            kb.m_atom_col_map = containers.Map('KeyType','char',...
                'ValueType','uint64');
            kb.m_col_atom_map = containers.Map('KeyType','uint64' ...
                ,'ValueType','char');
        end
        
        updateInsertClause(kb, ind_clause, force_append)
        
        [ind_clause, is_new] = clause2IndClause(kb, clause, ask_only)
        
        [ind, is_new] = getKBAtomIndex(kb, atom, ask_only)
        
        row = getMatchClause(kb, ind_clause)
        
        model = indModel2Model(kb, ind_model)
        
    end
    
    methods(Static)
        initializeEnv(init_c_sat, init_py_sat, compile)
        
        kb = genRandKB(num_clauses, num_atoms, sat)
        
        [satisfiable, models] = py_sat_solve(kb, rtn_all_models)
        
        function kb_clone = clone(kb)
            % clone - clone a knowledge base
            % On input:
            %    kb (KB instance): knowledge base
            % On output:
            %    kb_clone (KB instance): knowledge base clone
            % Call:
            %    kb_clone = KB.clone(kb);
            % Author:
            %     D. Sacharny
            %     UU
            %     Spring 2021
            %
            kb_clone = KB();
            kb_clone.m_negation_char = kb.m_negation_char;
            kb_clone.m_logic_mat = kb.m_logic_mat;
            kb_clone.m_num_atoms = kb.m_num_atoms;
            kb_clone.m_clause_probs = kb.m_clause_probs;
            kb_clone.m_atom_probs = kb.m_atom_probs;
            
            for key = keys(kb.m_atom_col_map)
                kb_clone.m_atom_col_map(key{:}) = kb.m_atom_col_map(key{:});
            end
            
            for key = keys(kb.m_col_atom_map)
                kb_clone.m_col_atom_map(key{:}) = kb.m_col_atom_map(key{:});
            end
            
        end
    end
end

