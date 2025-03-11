# Плагин захвата для Gazebo Ignition

## Описание

Плагин `gripper_plugin_gz` предназначен для использования в симуляции Gazebo Ignition и позволяет роботу захватывать и отпускать объекты с помощью захвата. Он реализует захват через создание фиксированного соединения между захватом и объектом и удаляет это соединение при отпускании объекта.

## Зависимости

Для работы плагина требуются следующие пакеты:
- **ROS 2 Jazzy**
- **Gazebo Ignition 8 (`gz-sim8`)**
- Пакеты: `rclcpp`, `std_msgs`, `gz-sim8`, `libgz-sim8-dev`, `libgz-math8-dev`, `libgz-common6-dev`, `libgz-plugin3-dev`

## Сборка

```sh
colcon build
source install/setup.bash
```

## Использование

### 1. Подключение зависимости 
В файл `CMakeLists.txt` добавляем следующую строку: 
```CMake
find_package(gripper_plugin_gz REQUIRED)
```
А в файл `package.xml` это: 
```xml
<depend>gripper_plugin_gz</depend>
```

### 2. включение в SDF(*.sdf, *.urdf, *.xacro) файл
```xml
<model name="my_bot">

  <link name="gripper_link">
    <!-- Детали ссылки захвата -->
  </link>

  <plugin name="gripper_plugin_gz::DynamicDetachableJoint" filename="libdynamic_detachable_joint.so">
    <parent_link>gripper_link</parent_link>
    <grip_link>grip_link</grip_link>
    <attach_topic>/model/my_bot/detachable_joint/attach</attach_topic>
    <detach_topic>/model/my_bot/detachable_joint/detach</detach_topic>
    <output_topic>/model/my_bot/detachable_joint/output</output_topic>      
  </plugin>

</model>
```

`parent_link` - ссылка, к которой будет прикрепляться `grip_link`  
`grip_link` - название ссылки, которая в любой(!!) `model` с выбранным именем будет прикрепляться к `parent_link`   
#### Топики, обязательные при использовании нескольких захватов в одной модели
##### В коде выше прописаны значения по умолчанию для имени модели "my_bot" 
`attach_topic` - отсюда в формате `std_msgs/msg/String` берется имя модели    
`detach_topic` - отсюда в формате `std_msgs/msg/Empty` берется команда на открепление объекта 
`output_topic` - отсюда в формате `std_msgs/msg/String` можно прочитать состояние захвата(отпущено, захвачено). Те же данные идут в Gazebo Debug терминал.
### 3. Правильный формат объекта
Чтобы было возможно захватить объект, его структура должна выглядеть следующим образом:
```xml
<model name="object1">
  <link name="grip_link">
    <!-- Детали ссылки объекта -->
  </link>
</model>
```
где `link name` должен совпадать с `grip_link` в настройках плагина
### 4. Управление захватом
Приведу пример на объекте из пункта выше с настройками плагина еще выше
Чтобы захватить объект, пише в топик каким-либо образом его `model name`. В случае использования терминала:
```bash
ros2 topic pub /model/my_bot/detachable_joint/attach std_msgs/msg/String "data: 'object1'"
``` 
```bash
ros2 topic pub /model/my_bot/detachable_joint/detach std_msgs/msg/Empty "{}"
```  
```bash
ros2 topic echo /model/my_bot/detachable_joint/state
```
для захвата, отпускания и мониторинга состояние соответственно   