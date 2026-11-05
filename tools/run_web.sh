#!/bin/bash
# Запуск симуляции с веб-интерфейсом noVNC и GPU RTX 4070 SUPER
# Доступ в браузере: http://localhost:8080 или http://<ВНЕШНИЙ_IP>:8080
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$DIR"

echo "==> Запуск Virtual Courier с веб-интерфейсом noVNC (порт 8080) и GPU..."
exec docker compose -f docker-compose.yaml -f docker-compose.override.yaml -f docker-compose.web.yaml up "$@"
