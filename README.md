# eurobot_2025

Это репозиторий для робота на соревнования ЕВРОБОТ 2025

## В первую очередь:

Подготовка рабочего пространства:

1) Установка актуальной системы Ubuntu 24.04
2) Установка ROS2 Jazzy
3) Создание своей ветки в этом репозитории


## Во вторую очередь:

После подготовки рабочего пространства можно приступать к запуску симулятора, как это сделать описано вот в этих уроках:

- Про urdf и rsp https://youtu.be/BcjHyhV0kIs?si=IbDWrvdaJ2a_fD-A

- Про запуск в газебо: https://youtu.be/IjFcr5r0nMs?si=pTy2eQ2oVXIJrfi0

- Как адаптировать под новый рос: https://youtu.be/fH4gkIFZ6W8?si=Hd8WzWFhSZ4U-xpQ 


Для голономной платформы в газебо существует отдельный плагин. Его название и как с ним работать, можно найти в документации. В видеороликах выше для движения робота используются колеса и их контакт с землей, в случае голономной платформы, это не нужно, достаточно "левитирующего" base_link, который по свойствам движения повторяет голономную платформу.  Соответственно, задача на текущем этапе - запустить симулятор, этот плагин для ходовой и проверить их работу с помощью teleop_twist_keyboard

В пакете "Шеснарь" (да, он так называется) можно найти URDF с base_link и лидаром, которые повторяют форму реального робота.

## Инструкция

### Вне контейнера

Перед сборкой:
```
xhost +local:
```

### Внутри контейнера

В директории проекта `/eurobot_2025`:

```
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.sh
```

Запуск симулятора:
```
ros2 launch shesnar launch_sim.launch.py use_sim_time:=True world:=./src/shesnar/worlds/obstacle.world
```

Чтобы заработал amcl публикуем map->odom вручную (почему-то иначе не работает пока):
```
ros2 run tf2_ros static_transform_publisher 0.0 0 0.0 0 0 0 1 map odom
```
Topic map, Durability Policy - Trancient Local

Управление с клавиатуры:
``` 
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel
```