# Unmanned Aerial System (UAS) Traffic Management System (UTM) Simulator
This repository contains a matlab library for simulating advanced air mobility (AAM) traffic management systems. This library is currently being developed by the OmegaInfinity Reasearch Group at the University of Utah's School of Computing.

This repository is organized in a number of classes:
- LBSD: This class encapsulates the Lane-Based Strategic Deconfliction (LBSD) Supplemental Data Service Provider
- ATOC: An instance of this class represents an Air Traffic Operations Center, providing users with visualization capabilities
- UAS: Unmanned Aerial System is a mobile agent that operates within the UTM
- KB: A database for knowledge storage and access
- SIM: This class encapsulates all the simulation functions, including mocking gps and radar sensors and updating agents
- RADAR: An encapsulation of a reduced-order radar sensor model
- USS: UAS Service Supplier (or equivalently a Provider of Services for Urban Air Mobility (PSU))

- [UTM ABMS](#unmanned-aerial-system--uas--traffic-management-system--utm--simulator)
  * [LBSD](#lbsd)
    + [Creating a Lane System](#creating-a-lane-system)
    + [Tutorial](#tutorial)
  * [ATOC](#atoc)
    + [Tutorial](#tutorial-1)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>



## LBSD
### Creating a Lane System

```matlab:Code
% Instantiate an LBSD object
lbsd = LBSD();
% Initialize the LBSD object with the generated lane graph
lbsd.lane_graph = digraph(edge_table, node_table);
```
The edge table and the node table look like this:

node table:
```
           XData      YData     ZData    Launch    Land     Name
          _______    _______    _____    ______    ____    ______

    10         -5    -22.071      0        0        1      {'10'}
    11    -22.071         -5     15        0        0      {'11'}
    12    -22.071         -5      0        1        0      {'12'}
    13    -22.071          5     15        0        0      {'13'}
    14    -22.071          5      0        0        1      {'14'}
    15         -5     22.071     15        0        0      {'15'}
```
edge table:
```
              EndNodes        Weight
          ________________    ______

    2     {'1' }    {'2' }      10  
    9     {'1' }    {'9' }      10  
    3     {'2' }    {'3' }      10  
    4     {'3' }    {'4' }      10
```

The RowNames property of both tables must be strings. Here's an example for creating a node_table and an edge table:
```matlab:Code
num_verts = length(xdata);
node_table = table(xdata, ydata, zdata, is_launch, is_land, ...
        'VariableNames',  {'XData', 'YData', 'ZData', 'Launch', 'Land'},...
        'RowNames', string(1:num_verts) );
node_table.Name = node_table.Properties.RowNames;

edge_table = table([verts_a', verts_b'], lane_lengths, ...
        'VariableNames', {'EndNodes','Weight'}, ...
        'RowNames', string(1:num_verts));
```

### Tutorial
**Create a Demonstration Lane System**


```matlab:Code
lbsd = LBSD.genSampleLanes(10, 15);
clf
plot(lbsd);
```


![docs/making_reservations_images/figure_0.png
](docs/making_reservations_images/figure_0.png
)



**List the lane ids**



```matlab:Code
lane_ids = lbsd.getLaneIds
```


```text:Output
lane_ids = 24x1 string    
"2"         
"9"         
"3"         
"4"         
"13"        
"5"         
"6"         
"17"        
"7"         
"8"         

```



**Checkout the lane node table**



```matlab:Code
lbsd.lane_graph.Nodes
```

| |XData|YData|ZData|Launch|Land|Name|
|:--:|:--:|:--:|:--:|:--:|:--:|:--:|
|1 1|-5|-12.0711|15|0|0|'1'|
|2 2|-12.0711|-5|15|0|0|'2'|
|3 3|-12.0711|5|15|0|0|'3'|
|4 4|-5|12.0711|15|0|0|'4'|
|5 5|5|12.0711|15|0|0|'5'|
|6 6|12.0711|5|15|0|0|'6'|
|7 7|12.0711|-5|15|0|0|'7'|
|8 8|5|-12.0711|15|0|0|'8'|
|9 9|-5|-22.0711|15|0|0|'9'|
|10 10|-5|-22.0711|0|0|1|'10'|
|11 11|-22.0711|-5|15|0|0|'11'|
|12 12|-22.0711|-5|0|1|0|'12'|
|13 13|-22.0711|5|15|0|0|'13'|
|14 14|-22.0711|5|0|0|1|'14'|



**Checkout the lane edge table**



```matlab:Code
lbsd.lane_graph.Edges
```

| |EndNodes}|Weight| |
|:--:|:--:|:--:|:--:|
|1 2|'1'|'2'|10|
|2 9|'1'|'9'|10|
|3 3|'2'|'3'|10|
|4 4|'3'|'4'|10|
|5 13|'3'|'13'|10|
|6 5|'4'|'5'|10|
|7 6|'5'|'6'|10|
|8 17|'5'|'17'|10|
|9 7|'6'|'7'|10|
|10 8|'7'|'8'|10|
|11 21|'7'|'21'|10|
|12 1|'8'|'1'|10|
|13 10|'9'|'10'|15|
|14 12|'11'|'2'|10|



**Generate Some Random Reservations:**



```matlab:Code
start_time = 0;
end_time = 100;
lane_ids = ["1","2","3"];
num_res = 50;
speed = 1;
headway = 5;
lbsd.genRandReservations(start_time, end_time, num_res, lane_ids, speed, headway);
```



**Get the Reservation Table**



```matlab:Code
lbsd.getReservations
```

| |id|lane_id|entry_time_s|exit_time_s|speed|hd|
|:--:|:--:|:--:|:--:|:--:|:--:|:--:|
|1|"1"|"1"|98.7935|108.7935|1|5|
|2|"2"|"1"|17.0432|27.0432|1|5|
|3|"3"|"1"|25.7792|35.7792|1|5|
|4|"4"|"1"|39.6799|49.6799|1|5|
|5|"5"|"1"|7.3995|17.3995|1|5|
|6|"6"|"1"|68.4096|78.4096|1|5|
|7|"7"|"1"|62.0672|72.0672|1|5|
|8|"8"|"1"|75.8112|85.8112|1|5|
|9|"9"|"1"|87.1111|97.1111|1|5|
|10|"10"|"1"|53.0629|63.0629|1|5|
|11|"11"|"1"|33.5311|43.5311|1|5|
|12|"12"|"1"|45.2593|55.2593|1|5|
|13|"13"|"1"|93.5731|103.5731|1|5|
|14|"14"|"2"|98.7935|108.7935|1|5|



**Get the Reservation Table for Specific Lane**



```matlab:Code
lbsd.getLaneReservations("1")
```

| |id|lane_id|entry_time_s|exit_time_s|speed|hd|
|:--:|:--:|:--:|:--:|:--:|:--:|:--:|
|1|"1"|"1"|98.7935|108.7935|1|5|
|2|"2"|"1"|17.0432|27.0432|1|5|
|3|"3"|"1"|25.7792|35.7792|1|5|
|4|"4"|"1"|39.6799|49.6799|1|5|
|5|"5"|"1"|7.3995|17.3995|1|5|
|6|"6"|"1"|68.4096|78.4096|1|5|
|7|"7"|"1"|62.0672|72.0672|1|5|
|8|"8"|"1"|75.8112|85.8112|1|5|
|9|"9"|"1"|87.1111|97.1111|1|5|
|10|"10"|"1"|53.0629|63.0629|1|5|
|11|"11"|"1"|33.5311|43.5311|1|5|
|12|"12"|"1"|45.2593|55.2593|1|5|
|13|"13"|"1"|93.5731|103.5731|1|5|



**Clear Reservations**



```matlab:Code
lbsd.clearReservations()
```



**This object also publishes events each time a reservation is made for testing with the ATOC. Here''s a simple example:**




**Subscribe to events**



```matlab:Code
lbsd.subscribeToNewReservation(@(src,evt)disp("Reservation Made!"));
```



**Then if you run the same experiment as above you'll see something like this:**



```matlab:Code
start_time = 0;
end_time = 100;
lane_ids = ["1","2","3"];
num_res = 50;
speed = 1;
headway = 5;
lbsd.genRandReservations(start_time, end_time, num_res, lane_ids, speed, headway);
```


```text:Output
Reservation Made!
Reservation Made!
Reservation Made!
...
```



**View Lane Reservations in ATOC**



```matlab:Code
atoc = ATOC(lbsd, 100, zeros(3), 90);
atoc.laneGraphs(lane_ids, [start_time, end_time]);
```


![docs/making_reservations_images/figure_1.png
](docs/making_reservations_images/figure_1.png
)


![docs/making_reservations_images/figure_2.png
](docs/making_reservations_images/figure_2.png
)


![docs/making_reservations_images/figure_3.png
](docs/making_reservations_images/figure_3.png
)



## ATOC

### Tutorial
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


![docs/AtocTutorial_images/figure_0.png
](docs/AtocTutorial_images/figure_0.png
)


![docs/AtocTutorial_images/figure_1.png
](docs/AtocTutorial_images/figure_1.png
)

  


Therefore, the next function is supposed to give a more specific look at the deviation between distance and speed from the telemetry data versus the planned data given a specific time interval to look at. This function takes in specific lanes that is desired to look at given a interval of time. 



```matlab:Code
% Grabs only a small time interval to specifically look at certain data
% points.
specific_time_interval = [drone_time(1), drone_time(2)];
atoc.speedvsdisGraph("1", specific_time_interval);
```


![docs/AtocTutorial_images/figure_2.png
](docs/AtocTutorial_images/figure_2.png
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

