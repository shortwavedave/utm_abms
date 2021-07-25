function [ind, is_new] = getKBAtomIndex(kb, atom, ask_only)
            % getKBAtomIndex Returns a column index for this atom
            % Description:
            %   [ind, is_new] = getKBAtomIndex(kb, atom, ask_only)
            %   The KB stores an index for each atom present in the
            %   database so that a matrix can be formed for each clause
            %   added to the KB. For example, if the following clause were
            %   added to the KB:
            %
            %   my_kb.tellClause({'~uas nominal', 'assigned'})
            %
            %   and these were the first two atoms told to the KB, then 
            %   internally 'uas_nominal' would map to 1 and 'assigned' to
            %   2.
            %
            %   Then the clause logic matrix (m_logic_mat) would be [-1 1].
            %   New atoms are added automatically to the KB.
            % On Input:
            %   kb - A KB instance (you can alternatively use dot notation,
            %        e.g. kb.getKBAtomIndex(atom).
            %   atom - A char array or string representing an atom
            %   ask_only - optional default false. If true, does not add
            %   new atoms into the KB.
            %
            % On Output:
            %   ind - integer index of the atom or -1 if is_new && ask_only
            %   is_new - boolean indicating if this was a newly added atom
            % Example:
            %   [ind, is_new] = my_kb.getKBAtomIndex('uas_nominal')
            if nargin < 3
                ask_only = false;
            end
            is_new = false;
            % First check if the atom exists
            if isKey(kb.m_atom_col_map, atom)
                ind = kb.m_atom_col_map(atom{:});
            % Otherwise create a new id
            else
                if ~ask_only
                    kb.m_num_atoms = kb.m_num_atoms + 1;
                    ind = kb.m_num_atoms;
                    kb.m_atom_col_map(atom{:}) = uint64(ind);
                    kb.m_col_atom_map(uint64(ind)) = atom{:};
                else
                    ind = -1;
                end
                is_new = true;
            end
        end

