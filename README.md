# eurobot_2025


Чтобы запустить: 
1) Подключиться к rpi  по ssh
2) Подключиться к контейнеру
3) В контейнере:
```
colcon build --symlink-install
source install/setup.bash
ros2 run uart_drive uart_drive --ros-args --params-file ./src/uart_drive/config/uart_params.yaml
```
