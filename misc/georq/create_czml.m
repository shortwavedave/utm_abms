function [json_czml] = create_czml(flights, del_t)
%CREATE_CZML Summary of this function goes here
%   Detailed explanation goes here
projection = projcrs(3857);
packet = {};
doc.id = "document";
doc.name = "simulation";
doc.version = "1.0";

clock.interval = "2021-03-15T6:00:00Z/2021-03-15T23:00:00Z";
clock.currentTime = "2021-03-15T6:00:00Z";
clock.multiplier = 1;

doc.clock = clock;

packet{1} = doc;

for i = 1:length((flights)
    flight = flights(1);
    route = flight.route;
    num_edges = length(route(:,1));
    pts1 = route(:,1:3);
    pts2 = route(:,4:6);

    for e = 1:num_edges
        
        plot3([pts1(e,1),pts2(e,1)],[pts1(e,2),pts2(e,2)],...
            [pts1(e,3),pts2(e,3)],'r');
    end
   
end

json_czml = jsonencode(packet);
end

function create_czml_path(input, del_t):
    % A geodetic, WGS84 position specified as [Longitude, Latitude, Height], 
    % where Longitude and Latitude are in degrees and Height is in meters. 
    % If the array has three elements, the value is constant. If it has four 
    % or more elements, they are time-tagged samples arranged as [Time, 
    % Longitude, Latitude, Height, Time, Longitude, Latitude, Height, ...], 
    %    where Time is an ISO 8601 date and time string or seconds since 
    % epoch.
    
    % Initialize a list to store data
    output = [];
    
    % Start timer at zero
    timestep = 0;
    
    % Loop through the dataframe
    for i = 1:size(input,1)
        x1 = 
        [lat,lon] = projinv(projection,x,y)
        output = [output timestep];
        output = [output input.longitude.loc[i])
        output = [output input.latitude.loc[i])
        output = [output input.elevation.loc[i])
        
        % Keep track of time
        duration = df_input.duration.loc[(i)]
        timestep += del_t
        
    return output