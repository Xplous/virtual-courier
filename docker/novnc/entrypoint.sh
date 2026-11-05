#!/bin/bash
set -e

WIDTH=${DISPLAY_WIDTH:-1920}
HEIGHT=${DISPLAY_HEIGHT:-1080}
DEPTH=${DISPLAY_DEPTH:-24}
DISPLAY_NUM=${DISPLAY_NUM:-1}

TOP_HEIGHT=$((HEIGHT * 72 / 100))
BOTTOM_HEIGHT=$((HEIGHT - TOP_HEIGHT))
HALF_WIDTH=$((WIDTH / 2))

# Очистка старых сокетов
rm -f /tmp/.X11-unix/X${DISPLAY_NUM} /tmp/.X${DISPLAY_NUM}-lock

# Настройка автоподключения в noVNC с высоким качеством картинки
if [ -f /usr/share/novnc/vnc.html ]; then
    cat <<'EOF' > /usr/share/novnc/index.html
<!DOCTYPE html>
<html>
<head>
  <meta http-equiv="refresh" content="0; url=vnc.html?autoconnect=true&resize=scale&quality=9&compression=0&reconnect=true&reconnect_delay=2000">
  <title>Virtual Courier - Web GUI</title>
</head>
<body style="background:#1a1a1a;color:#eee;font-family:sans-serif;text-align:center;padding-top:15%;">
  <h2>Подключение к симуляции Virtual Courier...</h2>
  <p>Если перенаправление не сработало, <a href="vnc.html?autoconnect=true&resize=scale&quality=9&compression=0&reconnect=true&reconnect_delay=2000" style="color:#3498db;">нажмите сюда</a>.</p>
</body>
</html>
EOF
fi

# Управляющие скрипты
cat <<'EOF' > /usr/local/bin/speed
#!/bin/bash
TARGET="${1:-}"

send_key() {
    local key="$1"
    local desc="$2"
    local found=0
    for wid in $(DISPLAY=:1 xdotool search --class Webots 2>/dev/null); do
        DISPLAY=:1 xdotool key --window "$wid" "$key" 2>/dev/null
        found=1
    done
    if [ "$found" -eq 1 ]; then
        echo "==> [speed] Режим симуляции установлен: $desc"
    else
        echo "==> [speed] Окно Webots не найдено на дисплее :1 (симуляция запущена?)"
    fi
}

case "$TARGET" in
    run|fast_gui|2|quick)
        send_key "ctrl+4" "⚡ Run (~5-10x ускоренный заезд с 3D-графикой)"
        ;;
    fast|max|turbo|3)
        send_key "ctrl+5" "🚀 Fast (~20-50x максимальная скорость без задержек)"
        ;;
    realtime|normal|1|real)
        send_key "ctrl+3" "⏱️ Real-time (1.0x нормальная скорость)"
        ;;
    pause|0|stop)
        send_key "ctrl+1" "⏸️ Pause (пауза)"
        ;;
    step)
        send_key "ctrl+2" "⏭️ Step (один физический шаг)"
        ;;
    *)
        echo "================================================================================"
        echo "  ⏩ ВЫБОР СКОРОСТИ СИМУЛЯЦИИ WEBOTS"
        echo "================================================================================"
        echo "  Использование: speed <режим>"
        echo ""
        echo "  Доступные режимы:"
        echo "    speed run       — ⚡ Ускоренный (Run, ~5-10x быстрее, графика отображается)"
        echo "    speed fast      — 🚀 Максимальный (Fast, ~20-50x, прогон без задержек)"
        echo "    speed realtime  — ⏱️ Реальное время (1.0x)"
        echo "    speed pause     — ⏸️ Пауза"
        echo ""
        echo "  💡 Переключать можно прямо на лету во время заезда робота!"
        echo "  💡 Или нажмите в окне Webots клавиши: Ctrl+4 (Run), Ctrl+5 (Fast), Ctrl+3 (1x)"
        echo "================================================================================"
        ;;
esac
EOF

cat <<'EOF' > /usr/local/bin/restart_all
#!/bin/bash
SPEED_ARG="${1:-}"
echo "==> [control] Перезапуск сцены Webots и решения..."
docker restart vc_scene
sleep 2
docker restart vc_solution
echo "==> [control] Готово! Новая симуляция запущена."

if [ -n "$SPEED_ARG" ]; then
    echo "==> [control] Установка скорости симуляции ($SPEED_ARG)..."
    for i in $(seq 1 15); do
        if DISPLAY=:1 xdotool search --class Webots &>/dev/null; then
            sleep 1
            /usr/local/bin/speed "$SPEED_ARG"
            break
        fi
        sleep 1
    done
fi
EOF

cat <<'EOF' > /usr/local/bin/restart_solution
#!/bin/bash
echo "==> [control] Перезапуск решения vc_solution..."
docker restart vc_solution
echo "==> [control] Готово! Решение перезапущено."
EOF

cat <<'EOF' > /usr/local/bin/stop_solution
#!/bin/bash
echo "==> [control] Остановка решения vc_solution..."
docker stop vc_solution
echo "==> [control] Решение остановлено."
EOF

cat <<'EOF' > /usr/local/bin/solution_shell
#!/bin/bash
echo "==> [control] Вход в интерактивный терминал vc_solution (bash)..."
exec docker exec -it vc_solution bash
EOF

cat <<'EOF' > /usr/local/bin/referee_logs
#!/bin/bash
echo "==> [control] Логи рефери (Ctrl+C для выхода):"
exec docker logs -f --tail 50 vc_scene
EOF

cat <<'EOF' > /usr/local/bin/solution_logs
#!/bin/bash
echo "==> [control] Логи решения (Ctrl+C для выхода):"
exec docker logs -f --tail 50 vc_solution
EOF

cat <<'EOF' > /usr/local/bin/control_console.sh
#!/bin/bash
clear
echo "================================================================================"
echo "  🤖 VIRTUAL COURIER — КОНСОЛЬ УПРАВЛЕНИЯ"
echo "================================================================================"
echo "  ⏩ Управление скоростью (переключение на лету):"
echo "    speed run         — ⚡ Ускорить симуляцию (~5-10x, физика + 3D-графика)"
echo "    speed fast        — 🚀 Максимальная скорость (~20-50x турбо)"
echo "    speed realtime    — ⏱️ Вернуть нормальную скорость 1.0x"
echo "    speed pause       — ⏸️ Пауза"
echo ""
echo "  🔄 Перезапуск и запуск:"
echo "    restart_all       — 🔄 Перезапуск симуляции с нуля (Webots + Решение)"
echo "    restart_all run   — ⚡ Перезапуск сразу в ускоренном режиме"
echo "    restart_all fast  — 🚀 Перезапуск сразу в турбо-режиме"
echo "    restart_solution  — 🚀 Перезапустить только решение (start.launch.py)"
echo "    stop_solution     — ⏹️  Остановить решение"
echo ""
echo "  🔍 Мониторинг и терминал:"
echo "    referee_logs      — 📊 Очки и таймер рефери в реальном времени"
echo "    solution_logs     — 📜 Логи нод решения в реальном времени"
echo "    solution_shell    — 💻 Терминал решения (bash / ROS 2)"
echo ""
echo "  💡 Подсказка: правый клик по рабочему столу также открывает меню!"
echo "================================================================================"
exec bash
EOF

chmod +x /usr/local/bin/speed \
         /usr/local/bin/restart_all \
         /usr/local/bin/restart_solution \
         /usr/local/bin/stop_solution \
         /usr/local/bin/solution_shell \
         /usr/local/bin/referee_logs \
         /usr/local/bin/solution_logs \
         /usr/local/bin/control_console.sh

ln -sf /usr/local/bin/speed /usr/local/bin/set_speed

# Настройка пароля x11vnc
X11VNC_AUTH="-nopw"
if [ -n "$VNC_PASSWORD" ]; then
    mkdir -p ~/.vnc
    x11vnc -storepasswd "$VNC_PASSWORD" ~/.vnc/passwd
    X11VNC_AUTH="-rfbauth ~/.vnc/passwd"
fi

# Запуск Xvfb
echo "[novnc] Запуск Xvfb :${DISPLAY_NUM} (${WIDTH}x${HEIGHT}x${DEPTH})..."
Xvfb :${DISPLAY_NUM} -screen 0 "${WIDTH}x${HEIGHT}x${DEPTH}" -ac +extension GLX +render -noreset &
XVFB_PID=$!

export DISPLAY=:${DISPLAY_NUM}

# Ждём появления сокета X11
for i in $(seq 1 30); do
    if [ -e "/tmp/.X11-unix/X${DISPLAY_NUM}" ]; then
        break
    fi
    sleep 0.2
done

# Настройка меню Openbox (правый клик мыши)
mkdir -p ~/.config/openbox
cat <<EOF > ~/.config/openbox/menu.xml
<?xml version="1.0" encoding="UTF-8"?>
<openbox_menu xmlns="http://openbox.org/3.4/menu">
  <menu id="root-menu" label="Virtual Courier">
    <item label="🔄 Перезапустить всё (Webots + Решение)">
      <action name="Execute"><command>xterm -title 'Рестарт симуляции' -hold -e /usr/local/bin/restart_all</command></action>
    </item>
    <item label="🚀 Перезапустить решение">
      <action name="Execute"><command>xterm -title 'Рестарт решения' -hold -e /usr/local/bin/restart_solution</command></action>
    </item>
    <item label="⏹️ Остановить решение">
      <action name="Execute"><command>xterm -title 'Остановка решения' -hold -e /usr/local/bin/stop_solution</command></action>
    </item>
    <separator label="Скорость" />
    <item label="⚡ Скорость: Run (~5–10x ускоренная)">
      <action name="Execute"><command>/usr/local/bin/speed run</command></action>
    </item>
    <item label="🚀 Скорость: Fast (~20–50x турбо)">
      <action name="Execute"><command>/usr/local/bin/speed fast</command></action>
    </item>
    <item label="⏱️ Скорость: 1.0x Real-time (нормальная)">
      <action name="Execute"><command>/usr/local/bin/speed realtime</command></action>
    </item>
    <item label="⏸️ Скорость: Пауза (Pause)">
      <action name="Execute"><command>/usr/local/bin/speed pause</command></action>
    </item>
    <separator />
    <item label="💻 Войти в терминал решения (bash)">
      <action name="Execute"><command>xterm -fa 'DejaVu Sans Mono' -fs 10 -bg '#181825' -fg '#cdd6f4' -title 'Solution Bash Shell' -e /usr/local/bin/solution_shell</command></action>
    </item>
    <item label="📊 Смотреть логи рефери (очки)">
      <action name="Execute"><command>xterm -fa 'DejaVu Sans Mono' -fs 10 -bg '#11111b' -fg '#a6e3a1' -title 'Referee Live Logs' -e /usr/local/bin/referee_logs</command></action>
    </item>
    <item label="📜 Смотреть логи решения">
      <action name="Execute"><command>xterm -fa 'DejaVu Sans Mono' -fs 10 -bg '#11111b' -fg '#89b4fa' -title 'Solution Live Logs' -e /usr/local/bin/solution_logs</command></action>
    </item>
    <separator />
    <item label="🖥️ Открыть консоль управления">
      <action name="Execute"><command>xterm -fa 'DejaVu Sans Mono' -fs 10 -bg '#181825' -fg '#cdd6f4' -title 'Control Console' -e /usr/local/bin/control_console.sh</command></action>
    </item>
  </menu>
</openbox_menu>
EOF

# Настройка расположения окон Openbox
cat <<EOF > ~/.config/openbox/rc.xml
<?xml version="1.0" encoding="UTF-8"?>
<openbox_config xmlns="http://openbox.org/3.4/rc">
  <theme>
    <name>Clearlooks</name>
    <titleLayout>NLIMC</titleLayout>
  </theme>
  <applications>
    <application name="webots-bin" class="Webots">
      <position force="yes">
        <x>0</x>
        <y>0</y>
      </position>
      <size>
        <width>${HALF_WIDTH}</width>
        <height>${TOP_HEIGHT}</height>
      </size>
      <decor>yes</decor>
      <maximized>no</maximized>
    </application>
    <application name="rviz2" class="rviz2">
      <position force="yes">
        <x>${HALF_WIDTH}</x>
        <y>0</y>
      </position>
      <size>
        <width>${HALF_WIDTH}</width>
        <height>${TOP_HEIGHT}</height>
      </size>
      <decor>yes</decor>
      <maximized>no</maximized>
    </application>
    <application name="xterm" class="XTerm">
      <position force="yes">
        <x>0</x>
        <y>${TOP_HEIGHT}</y>
      </position>
      <size>
        <width>${WIDTH}</width>
        <height>${BOTTOM_HEIGHT}</height>
      </size>
      <decor>yes</decor>
      <maximized>no</maximized>
    </application>
  </applications>
</openbox_config>
EOF

echo "[novnc] Запуск оконного менеджера Openbox..."
openbox &

echo "[novnc] Запуск нижней консоли управления (xterm)..."
xterm -fa 'DejaVu Sans Mono' -fs 10 -bg '#181825' -fg '#cdd6f4' -title 'Virtual Courier — Консоль управления' -e /usr/local/bin/control_console.sh &

echo "[novnc] Запуск x11vnc на порту 5900 (оптимизированный режим)..."
x11vnc -display :${DISPLAY_NUM} -forever -shared -rfbport 5900 $X11VNC_AUTH -threads -wait 5 -defer 5 -xdamage -noxfixes -nowf -quiet &

# Настройка прав для docker socket
if [ -e /var/run/docker.sock ]; then
    chmod 666 /var/run/docker.sock
fi

# Настройка OpenSSH Server (безопасный доступ строго по ключам)
mkdir -p /run/sshd
ssh-keygen -A 2>/dev/null || true

cat <<'EOF' > /etc/ssh/sshd_config
Port 22
PermitRootLogin prohibit-password
PubkeyAuthentication yes
PasswordAuthentication no
PermitEmptyPasswords no
KbdInteractiveAuthentication no
AllowTcpForwarding yes
PubkeyAcceptedAlgorithms +ssh-rsa
HostKeyAlgorithms +ssh-rsa
AuthorizedKeysFile .ssh/authorized_keys
EOF

AUTH_KEYS="ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAICuZgl+ZkSPY0Bdq/oW03zZ4NXVqHhYjMOiY3utCVwgj nick.filippoff2016@yandex.ru
ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIESE24gK8tVnnp/cJHn5XsIbhZwG4cgyCG3SEhsgswN2 xplous@DESKTOP-9DM8ROI
ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAABgQC1aAzVJxRgOMIoTaOcdjaaXN22wVw9kTo5uOQ9wFgkeOZGjaeFKBwPb1EPSdKGRwidgdodR6weLOdmx980kV4FVWYXh8VleeDaHCFEW7f4stN+O6AAXne8I/Ng21AbYk9yj7bPHgWxwi1XkdUjhmaCi3AQFa7KiEo8Uwt0eJVoYUPhe9ICJTHmdWr94CJ9woH5YPTet4AlhkPDxam+NSqaRNupCHnXFeeIEZATLJPCJReTl1OSyhnjKtPdJDTsJxh4XYXLNbXRrm8/I6czq7mFDQHE9rpHHZsY5wTpblK4dh/rONFIcl9PZdaVVmyB3+XIi9Wp2AKdPpT/h34vj+R52wgo7fWAH6AzKir2FDDAD1Yqj8TQYX8NMxtftaP42sxIEuBouYN5kq+ZCRs6Zoulw8iiQC3BGrW5oZWd+saDOzck4Ky3vyRxktFJwh9Rc4rSgGIreKHd6f3ZsKGmTuQxf3Tsb3Hoyi7V/pULN9orHIOZ4fsvwdRrEw7tQ4ggOf0= vaniog@i113939268"

mkdir -p /root/.ssh
echo "$AUTH_KEYS" > /root/.ssh/authorized_keys
chmod 700 /root/.ssh
chmod 600 /root/.ssh/authorized_keys

if ! id -u vaniog &>/dev/null; then
    useradd -m -s /bin/bash -u 1001 vaniog
fi
usermod -p '*' vaniog
echo "vaniog ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/vaniog
chmod 0440 /etc/sudoers.d/vaniog

mkdir -p /home/vaniog/.ssh
echo "$AUTH_KEYS" > /home/vaniog/.ssh/authorized_keys
chown -R vaniog:vaniog /home/vaniog/.ssh
chmod 700 /home/vaniog/.ssh
chmod 600 /home/vaniog/.ssh/authorized_keys

if [ -d /workspace ]; then
    ln -sfn /workspace /home/vaniog/workspace
fi

cat <<'EOF' > /etc/motd
================================================================================
  🤖 VIRTUAL COURIER — ЗАЩИЩЁННЫЙ ДОСТУП (SSH)
================================================================================
  Команды управления симуляцией:
    speed run         — ⚡ Ускорить симуляцию (~5-10x, физика + 3D-графика)
    speed fast        — 🚀 Максимальная скорость (~20-50x турбо)
    speed realtime    — ⏱️ Вернуть нормальную скорость 1.0x
    speed pause       — ⏸️ Пауза
    restart_all [spd] — 🔄 Перезапуск с нуля (можно указать: restart_all run)
    restart_solution  — 🚀 Перезапустить только решение (start.launch.py)
    stop_solution     — ⏹️  Остановить решение
    solution_shell    — 💻 Терминал решения (bash / ROS 2)
    referee_logs      — 📊 Смотреть очки и таймер рефери в реальном времени
    solution_logs     — 📜 Смотреть логи решения в реальном времени

  💡 Просмотр 3D-экрана Webots и RViz через браузер:
     Подключитесь с локальным пробросом порта 8080:
       ssh -N -L 8080:localhost:8080 -p 2222 vaniog@<IP>
     Затем откройте в браузере: http://localhost:8080
================================================================================
EOF
echo "[ -f /etc/motd ] && cat /etc/motd" >> /home/vaniog/.bashrc
echo "[ -f /etc/motd ] && cat /etc/motd" >> /root/.bashrc

echo "[novnc] Запуск OpenSSH Server на порту 22 (только авторизация по ключам)..."
/usr/sbin/sshd

echo "[novnc] Запуск websockify на порту 8080..."
websockify --web=/usr/share/novnc 8080 localhost:5900 &
WEBSOCK_PID=$!

echo "[novnc] Готово! Веб-интерфейс доступен на порту 8080"
wait $WEBSOCK_PID
