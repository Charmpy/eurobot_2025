# Пакет для управления актуаторами "lifter_control"

Файл с классами для работы актуаторов: `robot_lifters`

Файл-пример с простым `lifters_control`

Для работы:

1) Забилдиться 
```
colcon build --symlink-install
```

2) Засорситься
```
source install/setup.bash 
```

3) Запустить симуляцию из шеснаря
```
ros2 launch shesnar launch_sim.launch.py
```
4) Запустить пример без роса
```
python3 lifters_control.py
```
5) Радоваться прекрасной работе лифтов :-)

