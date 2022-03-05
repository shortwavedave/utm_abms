[tbl_out] = add_tch_stats(tbl);
t2 = tbl_out(tbl_out.density == 1000,{'struct','density','num_failed'});
sp = [];
for s = ["FAA","g_delaunay","grid","delaunay"]
    s1.struct = s;
    s1.failures = mean(t2{t2.struct == s, 'num_failed'});
    sp = [sp;s1];
end

sp_t = struct2table(sp);
sp_t.struct(2) = "g-delaunay";
h = bar(categorical(sp_t.struct)', sp_t.failures');
h.FaceColor = 'flat';
num_bars = length(sp_t.struct);
c = jet(num_bars);
for k = 1:num_bars
    h.CData(k,:) = c(k,:);
end
ylabel("Failed Flights");
title("Unschedulable Flights v. Airspace Structure");
t = text(.4,.96,"1000 Flights/Hour",'Units','normalized')