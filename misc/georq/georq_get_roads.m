function [roads] = georq_get_roads(layerName, use_auth, url)
%GEORQ_GET_ROADS Summary of this function goes here
%   Detailed explanation goes here
    if nargin < 3 
        url = 'http://localhost:8080/v1/graph';
    end
    headerFields = {};
    if use_auth
        client_id = 'matlab';
        client_secret = '3c6189bc-8d40-4b44-8c66-645580366741';
        token_url = 'https://georq.io/auth/realms/GeoRq/protocol/openid-connect/token';

    %     options = weboptions('ContentType', 'application/x-www-form-urlencoded');
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
        'ContentType','json','Timeout', 90);
    
    rdata = webread(url,'layerName', layerName, options);
    roads = netx2roads(rdata);
end

