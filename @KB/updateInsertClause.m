function updateInsertClause(kb, ind_clause, force_append)
            % updateInsertClause Inserts a new clause or updates an
            % existing one
            % Description:
            %   updateInsertClause(kb, ind_clause, force_append)
            %   This function will insert new clauses into the KB. If
            %   m_replace_update_atom is true, then the new clause will
            %   replace an old one when it is composed of a single atom or
            %   when it is exactly the same as another clause.
            
            % (TODO: Deal with Sparse Matrix)
            if ~kb.m_is_sparse
                % First check if this clause contains new atoms. If so then it
                % will not replace any existing clause, but the logic matrix
                % must be expanded 
                curr_sz = size(kb.m_logic_mat);
                clause_sz = length(ind_clause);
                if (curr_sz(2) < clause_sz) || force_append
                    new_cols = zeros(curr_sz(1), clause_sz - curr_sz(2));
                    % Handle case in which this is the first clause
                    append_clause = curr_sz(1) > 0;
                    kb.m_logic_mat = [kb.m_logic_mat, new_cols];
                    if append_clause
                        kb.m_logic_mat = [kb.m_logic_mat; ind_clause];
                    else
                        kb.m_logic_mat = ind_clause;
                    end
                % Determine which clause to update
                else
                    % If this is a single-atom clause, see if it already
                    % exists in either positive or negative forms
                    if sum(abs(ind_clause)) == 1
                        row = find( ...
                            ismember(kb.m_logic_mat, ind_clause, 'rows') ...
                        | ismember(kb.m_logic_mat, -1* ind_clause, 'rows')...
                        , 1);
                        if ~isempty(row)
                            kb.m_logic_mat(row, :) = ind_clause;
                        else
                            kb.m_logic_mat = [kb.m_logic_mat; ind_clause];
                        end
                    % Only update the logic matrix if this doesn't match a
                    % current clause
                    else
                        row = kb.getMatchClause(ind_clause);
                        if isempty(row)
                             kb.m_logic_mat = [kb.m_logic_mat; ind_clause];
                        end
                    end
                end  
            else
                error('Sparse Matrices Are Not Supported');
            end
        end

