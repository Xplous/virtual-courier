@echo off
cd /d "%~dp0"
echo ==> Launching Virtual Courier with Web GUI (noVNC on port 8080) and GPU...
docker compose -f docker-compose.yaml -f docker-compose.override.yaml -f docker-compose.web.yaml up %*
