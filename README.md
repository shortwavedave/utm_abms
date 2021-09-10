# utm_abms
This repository contains a matlab library for simulating advanced air mobility (AAM) traffic management systems.

## ATOC


**General Information**




ATOC Class creates an instance of an air traffic control within a simulation. It is responsible for monitoring air traffic and notify any behavior that doesn't follow that air traffic rules. 




An object of ATOC allows insights into what is happening within the simulation by looking at multiple different aspects of the simulation through tracking information about the lane based reservations, UAS telemetry data, and sensory data. 


  


**Creating ATOC Object**




To create an instance of ATOC, passing in the handle to the lane based reservation system is needed. Shown in the example code below.



```matlab:Code
% Setting up the Lane base Reservation for a given simulation.
lbsd = LBSD.genSampleLanes(10, 15);
start_time = 0;
end_time = 100;
lane_ids = ["1","2","3"];
num_res = 50;
speed = 1;
headway = 5;
lbsd.genRandReservations(start_time, end_time, num_res, lane_ids, speed, headway);

% Creates an atoc object given the handle to Lane Base Reservation Object
atoc = ATOC(lbsd);
```



This constructor will create a data structure that will store all the telemetry data that is emmitted from UAS objects. In addition, it will also create a data structure that stores all sensory data that is emmitted from the Radar objects. 


  


The next segment of code is temporary until UAS and ATOC are linked together. This is just setting up basic UAS informaiton so the graphs have information to be able to show.



```matlab:Code
% Grabing Reservations from lbsd
reservations = lbsd.getReservations;
pos = lbsd.getVertPositions(lbsd.getLaneVertexes(reservations(1, :).lane_id));

% Setting up Drone data to be used to show Visualization of the telemetry
% data
drone_time = reservations(1, :).entry_time_s;
end_time = reservations(1, :).exit_time_s;
steps = pos(1, :);
while (drone_time < end_time & (norm(steps(end, :) - pos(2, :)) > 1))
    cur_pos = steps(end, :);
    x = cur_pos(1) - rand();
    y = cur_pos(2) + rand();
    y = y - rand();
    z = cur_pos(3) + rand();
    z = z - rand();
    steps = [steps; x, y, z];
    drone_time = [drone_time; (drone_time(end, 1) + .1)];
end
uas = UAS(reservations(1, :).id);

% Updating the telemetry data
for t = 1:length(drone_time)
    atoc.time = drone_time(t); % Update time
    uas.gps.lat = steps(t, 1);
    uas.gps.lon = steps(t, 2);
    uas.gps.alt = steps(t, 3);
    atoc.updateTelemetry(uas, "1");
end

```

  


**Visualing Telemetry Data**




There are multiple different ways to visual telemetry data to track UAS behavior with respect to its planned trajectory. 




The first graph that can be formed is lane trajectory which is displaying the space diagram of the UAS projected distance from the lane itself. This is supposed to be used to get a quick glance whether an object is following the given path. However, this doesn't give a percise represented of whether an objects deviation from the planned path is due to location or speed is not relatively given from this graph. 



```matlab:Code
% Grab a random time interval
time = [0 90];

% Selection of certain lanes
lanes = ["1", "2"];
figure;
% Displays the space lane digrams
atoc.laneTrajectory(lanes, time);
```


![docs/@ATOC/AtocTutorial_images/figure_0.png
](AtocTutorial_images/docs/@ATOC/AtocTutorial_images/figure_0.png
)


![docs/@ATOC/AtocTutorial_images/figure_1.png
](AtocTutorial_images/docs/@ATOC/AtocTutorial_images/figure_1.png
)

  


Therefore, the next function is supposed to give a more specific look at the deviation between distance and speed from the telemetry data versus the planned data given a specific time interval to look at. This function takes in specific lanes that is desired to look at given a interval of time. 



```matlab:Code
% Grabs only a small time interval to specifically look at certain data
% points.
specific_time_interval = [drone_time(1), drone_time(2)];
atoc.speedvsdisGraph("1", specific_time_interval);
```


![docs/@ATOC/AtocTutorial_images/figure_2.png
](AtocTutorial_images/docs/@ATOC/AtocTutorial_images/figure_2.png
)

  


I also created another function that would invoke both of the previous graphs in one function. That would allow to look at the two different graphs at the same time interval. 



```matlab:Code
% Function that plots the previous two funciton using just a single
% function

% laneGraphs(lanes, [drone_time(1) drone_time(end)]);
```

  


The next information of telemetry data that is presented through graphs would be looking at the density of UAS through the lane system. There are two graphs that will help with the overview of this data. One looks at the number of UAS at a given time in all of the lanes, whereas the other graph will look at the density with a specific lane that was given. Both of these graphs will allow for a time interval that will allow for specfic time or no time interval to allow to see over the whole simulation. 



```matlab:Code
%% STILL NEEDS TO BE DONE!!! RADAR INFORMATION WILL BE USED FOR THIS.
```

