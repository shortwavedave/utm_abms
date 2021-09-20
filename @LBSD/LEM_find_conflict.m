function indexes = LEM_find_conflict(obj,reservations,ht)
%

OFFSET = 0.00001;

indexes = [];

num_lanes = length(reservations);
for e = 1:num_lanes
    flights = reservations(e).flights;
    if ~isempty(flights)
        dd = flights(2:end,2) - flights(1:end-1,2);
        bad = find(dd+OFFSET<ht);
        if ~isempty(bad)
            indexes = [indexes;e, bad+1,bad];
        end
    end
end
