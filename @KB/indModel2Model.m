function model = indModel2Model(kb, ind_model)
    % indModel2Model Convert a ind_model (internal representation, 
    %   a vector of literals) into a char cell array
    % 
    % Description:
    %   model = indModel2Model(kb, ind_model)
    % On Input:
    %   kb - an instance of KB
    %   ind_model - a vector representing the state of literals,
    %   with 1 being positive, -1 being negative, and 0 meaning
    %   that atom is a don't-care.
    %
    % On Output:
    %   model - char cell array of literals
    %
    % Example:
    %   model = kb.indModel2Model([0 1 -1])
    model = cell(1,length(ind_model));
    model_i = 0;
    for i = 1:length(ind_model)
        if ind_model(i) ~= 0
            model_i = model_i + 1;
            literal = kb.m_col_atom_map(i);
            if ind_model(i) < 0
                literal = [kb.m_negation_char literal];
            end
            model{model_i} = literal;
        end
    end
    if model_i < length(ind_model)
        model(model_i + 1:end) = [];
    end
end

