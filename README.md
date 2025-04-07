# BPC-BPL 3. cvičenie

Spustenie Ardupilot SITL (Software in the loop)
- prostredie máme už pripravané
```
cd ardu_ws
```

nakoľko sa nejedná o knižnicu, ktorá je vstavaná v základnej inštalácií ROS-u, je potrebné sourcnuť .bash z knižnice
```
source install/setup.bash
```

následne môžeme spustiť simulátor
```
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

po spustení simulátora, môžeme spustiť mavros
```
ros2 launch mavros apm.launch fcu_url:=udp://:14550@localhost:14555 gcs_url:="udp://" tgt_system:=1 tgt_component:=1
```

Vytvorenie nového prostredia
```
mkdir ros2_bpl_ws
```
Zmena aktualneho repozitara
```
cd ros2_bpl_ws
```
Vytvorenie zlozky src
```
mkdir src
```
následne si naklonujeme cv3 do zložky src, použite git clone, prípadne stiahnut cez http a nakopírovať

ako prvé je potrebné zbuildit interfaces
```
colcon build --packages-select bpl_interfaces
```
nakoľko ďalšie knižnica potrebujú funkčné interfaces, musíme package zbuildovat predtým ako ideme buildovat dalsi a nasourcovať si ho
```
source install/setup.bash
```
následne môžeme pomocou colcon build zbuilit vsetky packages
```
colcon build
```
