# ArUco Marker Tracking для PX4/ROS2

Проект для отслеживания ArUco меток с помощью дрона на PX4 и ROS2. Включает два режима полета: зависание над меткой и полет по спирали.

## Установка

```bash
# Клонируем репозиторий
git clone https://github.com/trianmon/aruco-spiral-sandbox.git
cd aruco-spiral-sandbox

# Клонируем PX4 зависимости
cd src/
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
git clone --recursive https://github.com/Auterion/px4-ros2-interface-lib.git
cd ..

# Собираем проект
colcon build

# Настраиваем окружение
source install/setup.bash
```

## Запуск

### Режим зависания над меткой

```bash
# Запуск launch-файла
ros2 launch hover_marker_mode hover_demo.launch.py
```


### Режим полета по спирали

```bash
# Запуск launch-файла
ros2 launch spiral_marker_mode spiral_demo.launch.py
```

### Ручной запуск компонентов

```bash
# Терминал 1: Детектор ArUco меток
ros2 run aruco_detector_cpp detector_node

# Терминал 2: Режим зависания
ros2 run hover_marker_mode hover_mode

# Или режим спирали
ros2 run spiral_marker_mode spiral_mode
```

## Структура проекта

- `aruco_detector_cpp` - детектор ArUco меток из видеопотока
- `hover_marker_mode` - режим зависания над обнаруженной меткой
- `spiral_marker_mode` - режим полета по спирали вокруг метки

## Что можно улучшить

1. Учёт ориентации ArUco метки для более точного позиционирования
2. Конфигурируемые параметры камеры (топик, разрешение, калибровка)
3. Подбор параметров полёта (скорости, высота, радиус спирали)

## Лицензия

Проект создан в учебных целях.
