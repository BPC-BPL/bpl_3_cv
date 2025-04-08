# ardupilot_mavros
pre build je potrebné mať clonute bpl_interfaces

ako prve buildnut bpl_interfaces
```
colcon build --packages-select bpl_interfaces
```
potom je potrebné sourcnut 
```
source install/setup.bash
```

## Spustenie
```
ros2 run bpl_uav_control drone_control
```

## Volanie servisov 

volanie takeoff
```
ros2 service call /takeff std_srvs/srv/Trigger "{}"
```

volanie land
```
ros2 service call /land std_srvs/srv/Trigger "{}"
```

volanie zmeny let modu na GUIDED
```
ros2 service call /set_mode std_srvs/srv/Trigger "{}"
```

volanie letu na bod
```
ros2 service call /fly_to_point bpl_interfaces/srv/Position "{latitude: -35.363261, longtitude: 149.1652373, altitude: 603.5586}"
```
