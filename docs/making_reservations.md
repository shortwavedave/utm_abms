

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

