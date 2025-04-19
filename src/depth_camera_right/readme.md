# Камера глубины MaixSense A010

## Запуск
```bash
source /opt/ros/*/setup.sh
colcon build
source install/setup.sh
ros2 run sipeed_tof_ms_a010 publisher --ros-args -p device:="<порт_к_которому_подключена_камера>"
```
Для меня это
```bash
ros2 run sipeed_tof_ms_a010_right publisher --ros-args -p device:="/dev/ttyUSB1"
```
## Визуализация в rviz2
![картинка](img/Screenshot%20from%202025-02-14%2018-04-56.png)

 - топик ```image``` должен транслировать картинку без проблем
 - торик ```PointCloud2``` будет ругаться на ```could not transform [tof] to [base link]``` Против этого у нас есть костыль: в новом терминале свяжем tof и base link:

 ```bash
 ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link tof
```
