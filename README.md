# Виртуальный курьер

ROS 2 Humble, Webots, SLAM Toolbox и Nav2/MPPI. Реализованы исследование лабиринта,
распознавание станций, загрузка и выгрузка грузов.

## Запуск

В Docker:

```bash
xhost +local:docker
docker compose build solution
docker compose up
```

На хосте с установленным ROS 2 Humble, Nav2/MPPI и SLAM Toolbox:

```bash
source /opt/ros/humble/setup.bash
(cd src/solution && colcon build --symlink-install)
source src/solution/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/solution/config/fastdds.xml"
WEBOTS_MODE=realtime docker compose up -d scene
ros2 launch solution start.launch.py
```

`gui:=false` отключает RViz. `WEBOTS_GUI=false` отключает окно Webots.
Для нового заезда остановите решение, пересоздайте сцену и запустите решение снова.

## Навигация

- Карта разбивается на клетки 2×2 м. Переходы: открытые, закрытые, неизвестные.
- Узел выбирает непосещённые клетки, затем — точки осмотра неизвестных переходов.
- Nav2 строит маршруты к кандидатам; MPPI управляет движением и объездом.
- Цель продлевается по открытому коридору. Промежуточные клетки отмечаются на ходу.

Оценка цели:

```text
path_weight × длина пути + turn_weight × начальный поворот
− perimeter_weight × оценка периметра
```

Станции находятся у внешних стен поля 8×8. Положение поля уточняется по известным
клеткам и условию старта на периметре. Внутренние цели не запрещены.
Замыкание петель SLAM выключено. Симуляция запускается на ×1.

Код: [выбор целей](src/solution/scripts/cell_explorer_node.py),
[сетка и переходы](src/solution/scripts/maze_grid_node.py).
Параметры: [исследование](src/solution/config/cell_explorer.yaml),
[Nav2/MPPI](src/solution/config/nav2_params.yaml),
[SLAM](src/solution/config/slam_toolbox.yaml).

В RViz: зелёные переходы открыты, красные закрыты, серые неизвестны.
Посещённые клетки жёлтые; с проверенными выходами — зелёные. Красная точка — цель.

## Прогоны без GUI

Перед серией остановите обычное решение и сцену.

```bash
source /opt/ros/humble/setup.bash
python3 tools/run_exploration_batch.py \
  --output playground-vaniog/exploration-batch/new-run \
  --seeds 2026 92813 37400 52332 58603 75375 --workers 2
```

Каждый заезд использует снимок решения, отдельный ROS-домен и Xvfb-дисплей.
Лимит — 600 секунд симуляции. Результаты: `summary.csv`, `summary.json`, сценарии,
траектории и логи. Наблюдатель измеряет реальное покрытие и столкновения;
привилегированные данные сцены алгоритму не передаются.

## Соревнование

[Регламент](docs/Регламент_Виртуальный_курьер_2026.pdf),
[инструкция организаторам](docs/organizers.md).
Организаторы берут только `src/solution` и запускают `start.launch.py gui:=false`.
