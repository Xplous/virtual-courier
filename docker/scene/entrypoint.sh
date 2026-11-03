#!/bin/bash
# Запуск сцены: генерация мира из сценария и старт Webots.
#
# Переменные окружения:
#   SCENARIO       путь к YAML-сценарию             (/scene/scenarios/public_1.yaml)
#   WEBOTS_GUI     true — окно Webots, false — headless (true)
#   WEBOTS_MODE    realtime | fast | pause          (realtime)
#   WEBOTS_DISPLAY номер Xvfb-дисплея для headless  (99)
#   WEBOTS_PORT    порт Webots                     (1234)
#   VC_EVAL        true — закрыть Webots после финиша (false)
#   VC_TEAM, VC_ATTEMPT, VC_RESULTS_DIR — подписи в results.json
set -e
source /opt/ros/humble/setup.bash

SCENARIO=${SCENARIO:-/scene/scenarios/public_1.yaml}
WORLD_DIR=/scene/webots/worlds
MODE=${WEBOTS_MODE:-realtime}

echo "[scene] сценарий: $SCENARIO"
python3 /scene/generator/generate_world.py "$SCENARIO" --out "$WORLD_DIR" --name current
export VC_WORLD_META="$WORLD_DIR/current.json"

ARGS=(--batch --stdout --stderr "--mode=$MODE" "--port=${WEBOTS_PORT:-1234}")
if [ "${WEBOTS_GUI:-true}" = "true" ]; then
    exec webots "${ARGS[@]}" "$WORLD_DIR/current.wbt"
else
    # xvfb-run зависает, будучи PID 1 в контейнере, поэтому поднимаем Xvfb сами.
    export DISPLAY=":${WEBOTS_DISPLAY:-99}"
    Xvfb "$DISPLAY" -screen 0 1280x1024x24 -nolisten tcp &
    sleep 1
    exec webots "${ARGS[@]}" --no-rendering --minimize "$WORLD_DIR/current.wbt"
fi
