# Виртуальный курьер

Соревнование по программированию автономного робота-курьера в симуляторе **Webots** на **ROS 2 Humble**.
Робот уже настроен: слушается команд и отдаёт данные всех датчиков. Ваша задача — написать алгоритм,
который проведёт его через склад-лабиринт, заберёт грузы на станциях и доставит их обратно.

Соревнование — ступень между курсом «Основы автономной робототехники» и очными соревнованиями
«Цифровой курьер»: задача та же, что вас ждёт на реальном полигоне.

## Задание

#### Дано

- **Полигон 16 × 16 м** — склад-лабиринт на сетке 8 × 8 ячеек по 2 м, неподвижные стены (h = 0.3 м) и паллеты.
- **Старт-Финиш** — угловая/краевая ячейка (зелёный пол) с **зоной выгрузки** у стены (белая рамка, ArUco №0).
- **3 станции загрузки A, B, C** у внешних стен: цветной квадрат 1.4 × 1.4 м на полу, бункер над ним,
  ArUco №1/2/3 на стене и **сигнальная лампа**: зелёная — станция активна, погасшая — груза нет,
  оранжевая — груз уже выдан.
- **Робот Courier** в габаритах робота-пылесоса (Ø 0.35 м): дифференциальный привод, лидар 360°,
  камера 640×480, IMU, энкодеры колёс, откидной грузовой лоток с весовым датчиком.

#### Задача

В полностью автономном режиме:

1. выехать из зоны Старт и пройти лабиринт;
2. заехать в зону **активной** станции и остановиться на **2 с** — груз (кубик 5 см, 0.3 кг) упадёт на лоток;
3. довезти груз, **не уронив** (масса и инерция симулируются честно), до зоны выгрузки у Старта;
4. наклоном лотка выгрузить груз в зону выгрузки.

Можно возить грузы по одному или собрать все за один рейс — решает команда.

#### Условия

- Максимальное время заезда — **10 минут** симуляционного времени. Таймер стартует с первой ненулевой команды
  решения (или автоматически через 120 с после запуска сцены).
- Заезд заканчивается, когда все активные грузы доставлены или истекло время.
- Робот стартует в центре ячейки Старт-Финиш, «лицом» вдоль одной из осей сетки (0/90/180/270°).
- Конфигурация лабиринта, число и положение паллет, набор активных станций, клетка старта и начальная
  ориентация робота **в зачётных сценариях отличаются** от публичных. Хардкод маршрута не сработает.

#### Оценивание

| Событие | Баллы |
|---|---|
| Выезд из зоны Старт | +5 |
| Активная станция достигнута (каждая, один раз) | +10 |
| Груз получен и вывезен со станции на лотке | +15 за груз |
| Груз доставлен (лежит на полу в зоне выгрузки) | +25 за груз |
| Все грузы доставлены | +20 и бонус за время `20 · (600 − T) / 600` |
| Касание стены или паллеты | −2 (суммарно не более −20) |
| Груз упал вне зоны выгрузки | −10 |

Подробно — в регламенте соревнований (публикуется в Telegram-канале https://t.me/robotics_bmstu).

## Интерфейс робота

| Топик | Тип | Направление | Описание |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | → робот | `linear.x` до 0.7 м/с, `angular.z` до 3 рад/с; без команд 0.5 с — стоп |
| `/courier/tray_cmd` | `std_msgs/Float64` | → робот | угол наклона лотка 0 … 1.05 рад (≈1.0 — выгрузка назад) |
| `/scan` | `sensor_msgs/LaserScan` | ← робот | 360 лучей, 0.12–8 м, 10 Гц, кадр `laser` |
| `/odom` | `nav_msgs/Odometry` | ← робот | одометрия по энкодерам **с шумом и дрейфом**, 50 Гц |
| `/tf`, `/tf_static` | | ← робот | `odom → base_link → laser, camera_link, camera_optical_frame, imu_link, tray_link` |
| `/imu` | `sensor_msgs/Imu` | ← робот | ориентация, угловая скорость, ускорение, 50 Гц |
| `/camera/image_raw` | `sensor_msgs/Image` | ← робот | 640×480 `bgra8`, FOV 62°, 10 Гц |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | ← робот | калибровка камеры |
| `/courier/cargo_count` | `std_msgs/UInt8` | ← робот | число грузов на лотке |
| `/courier/tray_state` | `std_msgs/Float64` | ← робот | текущий угол лотка |
| `/clock` | `rosgraph_msgs/Clock` | ← сцена | время симуляции — используйте `use_sim_time: true` |

Маркеры ArUco — словарь `DICT_4X4_50`: **0** — зона выгрузки, **1/2/3** — станции A/B/C.

> Топики `/referee/*` — служебные (табло судьи). Использовать их в решении **запрещено**,
> как и любые сведения об истинной позе робота. Нарушение — дисквалификация попытки.

## Установка

1. ОС: **Ubuntu 22.04 / 24.04** (рекомендуется) или Windows 10/11 с **WSL2** (WSLg для окон).
   Минимум: 4 ядра, 8 ГБ ОЗУ, 15 ГБ на диске. Видеокарта желательна, но не обязательна.
2. Установите [Docker Engine](https://docs.docker.com/engine/install/ubuntu/) и
   [Docker Compose Plugin](https://docs.docker.com/compose/install/linux/),
   добавьте себя в группу `docker` (`sudo usermod -aG docker $USER`, затем перелогиньтесь).
3. *(NVIDIA)* [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html).

## Запуск

1. Сделайте **форк** этого репозитория (одна копия на команду) и склонируйте его:

    ```bash
    git clone https://github.com/<ваш-аккаунт>/virtual-courier.git
    cd virtual-courier
    ```

2. Разрешите контейнерам открывать окна: `xhost +local:docker`
3. Скачайте образы и запустите сцену вместе с решением:

    ```bash
    docker compose pull
    docker compose up
    ```

    С видеокартой NVIDIA:

    ```bash
    docker compose -f docker-compose.yaml -f docker-compose.nvidia.yaml up
    ```

Откроются окно Webots с полигоном и табло судьи, RViz и окно камеры, робот поедет по демо-алгоритму.

**Пишите код в `src/solution`** — каталог примонтирован в контейнер, правки видны сразу.
После изменения кода: `docker compose restart solution` (пакет пересобирается при каждом старте).
Удобнее всего работать в VS Code: *Dev Containers → Reopen in Container* (конфиг в `.devcontainer/`).

#### Пакет solution

```
src/solution/
├── launch/start.launch.py     точка входа (организаторы запускают именно её)
├── scripts/courier_node.py    заготовка на Python: все топики подключены, демо-объезд, ArUco
├── src/example_cpp.cpp        пример узла на C++
├── config/                    slam_toolbox.yaml, nav2_params.yaml, courier.rviz
└── requirements.txt           (по желанию) доп. pip-пакеты — ставятся при старте контейнера
```

- `ros2 launch solution start.launch.py slam:=true` — включить построение карты (slam_toolbox, TF `map→odom`);
- `... nav:=true` — включить Nav2 (цели можно ставить из RViz кнопкой *2D Goal Pose*);
- `gui:=false` — без окон. **На прогоне организаторов `gui=false`** — решение не должно падать без дисплея.

#### Полезные команды

| Назначение | Команда |
|---|---|
| Другой публичный сценарий | `SCENARIO=public_2 docker compose up` |
| Без окна Webots (быстрее на слабом ПК) | `WEBOTS_GUI=false docker compose up` |
| Перезапуск решения | `docker compose restart solution` |
| Перезапуск сцены (заезд заново) | `docker compose restart scene solution` |
| Терминал в контейнере решения | `docker compose exec solution bash` |
| Посмотреть баллы в терминале | `docker compose exec solution bash -c "ros2 topic echo /referee/events"` |
| Обновить образы | `docker compose pull` |
| Собрать образы локально | `docker compose build` |

Результаты каждого заезда пишутся в `results/*.json`.

#### Свои тестовые сценарии

Сценарий — это YAML в `scene/scenarios/` (см. `public_1.yaml`). Меняйте `seed`, `loops`, `obstacles`,
станции и старт, чтобы проверить алгоритм на разных лабиринтах:

```bash
cp scene/scenarios/public_1.yaml scene/scenarios/my_test.yaml   # отредактируйте seed и т.д.
SCENARIO=my_test docker compose up
```

## Пробное исследование: explore_lite + Nav2/MPPI

Локальный запуск на хосте с ROS 2 Humble. MPPI должен быть установлен
(`ros-humble-nav2-mppi-controller`). Зависимость explore_lite собирается отдельно:

```bash
source /opt/ros/humble/setup.bash
mkdir -p playground-vaniog/ready-exploration/deps_ws/src
git clone https://github.com/robo-friends/m-explore-ros2.git playground-vaniog/ready-exploration/deps_ws/src/m-explore-ros2
git -C playground-vaniog/ready-exploration/deps_ws/src/m-explore-ros2 checkout 326cf8a0b487c34246bb8f3326afbcd69576dc60
git -C playground-vaniog/ready-exploration/deps_ws/src/m-explore-ros2 apply "$PWD/src/solution/patches/explore_lite-goal-selection.patch"
(cd playground-vaniog/ready-exploration/deps_ws && colcon build --symlink-install --packages-up-to explore_lite --executor sequential)
source playground-vaniog/ready-exploration/deps_ws/install/setup.bash
(cd src/solution && colcon build --symlink-install --executor sequential)
source src/solution/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/.vscode/fastdds.xml"
ros2 launch solution explore.launch.py
```

Перед запуском остановите прежние узлы управления. Сцену запустите отдельно через
`WEBOTS_MODE=realtime docker compose up -d scene`. Файл `.vscode/fastdds.xml` —
локальный профиль UDP для связи ROS на хосте с контейнером сцены.
В RViz отображаются `/map`, маршрут `/plan` и границы исследования `/explore/frontiers`.
Этот запуск не использует самописный Explorer. Сетка нужна только фильтру паллетов:
`/exploration_map` исключает внутренние тени из целей explore_lite, а Nav2 продолжает
читать исходную `/map`. Неизвестные участки возле границ клеток сохраняются.
Патч explore_lite учитывает угол до цели (`orientation_scale`) и удерживает текущую
цель, пока другая не выигрывает больше `goal_switch_margin`. Разворот разрешён,
если подходящих целей впереди нет; расстояние до цели оценивается по прямой.
Внешний пакет пока подключён только для локального прототипа; для сдачи его нужно
включить в допустимый способ установки зависимостей.

## Пробное исследование по Тремо

```bash
source /opt/ros/humble/setup.bash
source src/solution/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/.vscode/fastdds.xml"
ros2 launch solution tremaux.launch.py
```

Запускайте после остановки предыдущего решения. Тремо хранит отметки переходов
между клетками 2×2 м, проверяет новые проходы и возвращается по уже пройденным.
Лидар управляет реактивным объездом, скорость до 0,35 м/с. Nav2 и explore_lite
в этом варианте не запускаются. SLAM нужен для узнавания клеток и стен.
В RViz зелёные линии показывают пройденные переходы, красная точка — выбранную клетку.
Полнота обхода гарантируется для корректно распознанного конечного графа при
успешном проезде переходов. Потеря клетки или неудачный проезд вызывают остановку,
а не сообщение об успешном исследовании.

## Исследование клеток своим узлом и Nav2/MPPI

```bash
source /opt/ros/humble/setup.bash
source src/solution/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/.vscode/fastdds.xml"
ros2 launch solution cell_explore.launch.py
```

Запускайте после остановки других контроллеров. Python-узел `cell_explorer_node.py`
использует сетку 2×2 м, посещения клеток и состояния переходов по SLAM-карте.
Цели лежат в известном свободном пространстве с запасом до препятствий.
Несколько кандидатов оцениваются отдельным планировщиком Nav2; их стоимость —
`path_weight × длина пути + turn_weight × начальный поворот`.
Движение выполняет стандартный Nav2 с MPPI. Explore_lite и его патчи здесь не используются.

Параметры находятся в `src/solution/config/cell_explorer.yaml`:

| Параметр | Значение | Смысл |
|---|---:|---|
| `path_weight` | 1.0 | Вес длины настоящего маршрута в метрах |
| `turn_weight` | 1.0 | Вес начального поворота в радианах |
| `switch_margin` | 1.0 | Минимальный выигрыш для смены текущей цели |
| `candidate_count` | 5 | Число проверяемых новых целей за цикл |
| `update_period` | 1.0 | Интервал выбора целей, секунды симуляции |
| `corridor_cells` | 4 | Максимальное продление цели по прямому открытому коридору |
| `extension_distance` | 1.5 | Остаток пути для заблаговременного продления цели, м |
| `clearance` | 0.25 | Минимальное расстояние точки цели от препятствий и неизвестности, м |
| `path_timeout` | 2.0 | Ограничение ожидания одного маршрута, реальные секунды |
| `retry_delay` | 10.0 | Пауза перед повторной попыткой неудачной цели, реальные секунды |

В RViz: зелёные границы — открытые переходы, красные — стены, серые — неизвестность.
Посещённые клетки жёлтые, а клетки с проверенными выходами зелёные; красная точка — цель.
Неизвестные переходы осматриваются из доступной части клетки, когда доступных
непосещённых клеток больше нет. Если переход остаётся неизвестным или маршруты
недоступны, узел сообщает об этом и не объявляет обход завершённым.
Замыкание петель SLAM выключено; симуляцию пока следует запускать на ×1.

## Сдача решения

1. Всё решение — **только в `src/solution`**. Изменения в других файлах на прогоне игнорируются:
   сцена, compose и образы у организаторов свои.
2. Зависимости — только из образа решения и `src/solution/requirements.txt`.
3. Поставьте тег и отправьте его в свой форк:

    ```bash
    git tag final && git push origin final
    ```

4. Пришлите организаторам ссылку на форк. Если репозиторий приватный — добавьте организаторов в collaborators.

Организаторы запускают решения в прямом эфире на закрытых сценариях: `tools/run_eval.sh` (см. [docs/organizers.md](docs/organizers.md)).

## Структура репозитория

```
├── docker-compose.yaml           сцена + решение
├── docker-compose.nvidia.yaml    добавка для GPU NVIDIA
├── docker-compose.eval.yaml      добавка для зачётного прогона
├── src/solution/                 ← ваш код
├── scene/
│   ├── scenarios/                публичные сценарии (YAML)
│   ├── generator/                генератор полигона из сценария
│   └── webots/                   робот (PROTO), драйвер, судья
├── docker/                       Dockerfile'ы образов scene и solution
├── tools/                        run_eval.sh, leaderboard.py (для организаторов)
└── docs/                         регламент, памятка организаторам
```
