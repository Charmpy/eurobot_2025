# eurobot_2025

Это репозиторий для робота на соревнования ЕВРОБОТ 2025 123

Шпаргалка по Газебо:

Список топиков Газебо:

```
gz topic -l
```
Информация о топике Газебо (тип сообщения):

```
gz topic -i --topic <название>
```

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
ros2 launch shesnar launch_sim.launch.py use_sim_time:=True world:=./src/shesnar/worlds/obstacle.world map:=/eurobot_2025/src/shesnar/maps/euro_map.yaml is_localization:=True is_navigation:=True
```

Чтобы заработал amcl:
- Публикуем map->odom вручную (почему-то иначе не работает пока):
```
ros2 run tf2_ros static_transform_publisher 0.0 0 0.0 0 0 0 0.01 map odom
```
- В Rviz: Topic map, Durability Policy - Trancient Local
- Initial pose

Управление с клавиатуры:
``` 
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel
```