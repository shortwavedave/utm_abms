function excluded_out = LEM_excluded(obj,excluded,t1,t2,ft1,ft2,ht)
%

if isempty(excluded) % first interval considered
    excluded_out = [ft1,ft2];
    return
else % at least one excluded interval
    [num_excluded,dummy] = size(excluded);
    if ft2<excluded(1,1) % left of all intervals
        excluded_out = [ft1,ft2;excluded];
        return
    elseif excluded(end,2)<ft1 % right of all intervals
        excluded_out = [excluded; ft1,ft2];
        return
    else % check relation to existing intervals
        e = 0;
        excluded_out = excluded;
        while e<num_excluded-1 % check all but last
            e = e + 1;
            v1 = excluded(e,1); % [v1,v2] [v3,v4] intervals
            v2 = excluded(e,2);
            v3 = excluded(e+1,1);
            v4 = excluded(e+1,2);
            if ft1<v1&ft2<=v2 %  x [ y ] []
                excluded_out(e,1) = ft1;
                return
            elseif v1<=ft1&ft2<=v2 % [ x y ] []
                return
            elseif ft1>=v1&ft1<=v2&ft2>v2&ft2<v3 % [ x ] y []
                excluded_out(e,2) = ft2;
                return
            elseif ft1>=v1&ft1<=v2&ft2>=v3&ft2<=v4 % [ x ] [ y ]
                excluded_out(e,2) = v4;
                excluded_out(e+1,:) = [];
                return
            elseif v2<ft1&ft2<v3 % [] xy []
                excluded_out = [excluded(1:e,:);ft1,ft2;...
                    excluded(e+1:end,:)];
                return
            end
        end
        v1 = excluded(end,1);
        v2 = excluded(end,2);
        if ft1<v1&ft2>=v1&ft2<=v2      %  x [ y ]
            excluded_out(end,1) = max(t1,ft1);
            return
        elseif v1<=ft1&ft2<=v2         % [ x y ]
            return
        elseif v1<=ft1&ft1<=v2&ft2>v2  % [ x ] y
            excluded_out(end,2) = min(t2,ft2);
            return
        end
    end
end

tch = 0;
