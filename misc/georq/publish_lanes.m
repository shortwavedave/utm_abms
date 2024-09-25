function [ok] = publish_lanes(airways, layerName, use_auth)
%PUBLISH_LANES Summary of this function goes here
%   Detailed explanation goes here
    headerFields = {};
    url = 'http://localhost:8080/v1/lanes';
    if use_auth
        client_id = 'matlab';
        client_secret = '3c6189bc-8d40-4b44-8c66-645580366741';
        token_url = 'https://georq.io/auth/realms/GeoRq/protocol/openid-connect/token';

        data = [...
         '&client_id=', client_id,...
         '&client_secret=', client_secret,...
         '&grant_type=', 'client_credentials',...
         '&scope=', 'write_layer'];

        response = webwrite(token_url,data);
        access_token = response.access_token;

        % save access token for future calls
        headerFields = {'Authorization', ['Bearer ', access_token]};
    end
    
    options = weboptions('HeaderFields', headerFields, ...
        'Timeout', Inf, ...
        'ContentType','json', ...
        'MediaType', 'application/json');
    
    lanes = airways.lanes;
    num_lanes = size(lanes, 1);
    lane_structs(num_lanes) = struct('id','','coordinates',zeros(1,6));
    for i = 1:num_lanes
        lane_structs(i).id = int2str(i);
        lane_structs(i).coordinates = lanes(i,:);
    end
    
    data.lanes = lane_structs;
    data.layerName = layerName;
    
    try
        response = webwrite(url, data, options);
        ok = true;
    catch ME
        display(ME.message);
        ok = false;
    end
end

