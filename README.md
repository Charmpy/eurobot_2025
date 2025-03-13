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
ros2 launch shesnar launch_sim.launch.py use_sim_time:=True 
```

Чтобы заработал amcl:
- В Rviz: Topic map, Durability Policy - Trancient Local
- Fixed frame = map
- Initial pose (2D Pose Estimate)

Управление с клавиатуры:
``` 
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel
```

Новая цель для робота через терминал:
```
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: -1.0, z: 0.0}, orie
ntation:{x: 0.0, y: 0.0, z: 0, w: 1.0000000}}}"
```