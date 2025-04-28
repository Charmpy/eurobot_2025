# Пакет depth camera
Для запуска:

Подготовка и запуск симуляции:

Билд:
```
colcon build --symlink-install
```

Запуск симуляции:
```
source install/setup.bash
ros2 launch shesnar launch_sim.py
```

В новом терминале запускаем ноду:
```
source install/setup.bash
ros2 run depth_camera depth_camera_handler
```
В рвизе должно появится изображение с камеры с обведенным контуром доски. Если изображение в каие-то моменты не приходит, значит камера не может увидеть доску

Для управления роботом:

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```

Позиционирование по левой камере:
```
ros2 topic pub positioning -1 std_msgs/msg/String "{data: 'only_left'}"
```
По правой:
```
ros2 topic pub positioning -1 std_msgs/msg/String "{data: 'only_right'}"
```
По обеоим(если одна отваливается, ориентируется по другой):
```
ros2 topic pub positioning -1 std_msgs/msg/String "{data: 'both'}"
```