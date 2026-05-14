# ==============================================================================
# GAZEBO WEB APP - Sanal Ekran + VNC + noVNC WebSocket
# ==============================================================================
# Gazebo GUI'sini tarayıcıdan erişilebilir hale getirir.
# Masaüstü paylaşılmaz; Gazebo izole bir sanal ekranda (Xvfb :99) çalışır.
#
# KULLANIM:
#   1. Bu launch dosyasını başlat:
#        ros2 launch my_robot_cell_control gazebo_web_app.launch.py
#
#   2. Gazebo'yu sanal ekranda başlat (ayrı terminal):
#        DISPLAY=:99 ros2 launch my_robot_cell_control hil_test_whole_unified.launch.py
#
#   3. Tarayıcıda aç:
#        http://localhost:6080/vnc.html          (yerel makine)
#        http://SERVER_IP:6080/vnc.html          (ağdan erişim)
#
# BAĞIMLILIKLAR:
#   sudo apt install xvfb x11vnc   (novnc zaten kurulu)
#
# PORTLAR:
#   :99  → Xvfb sanal display (sadece Gazebo)
#   5900 → VNC (x11vnc)
#   6080 → noVNC WebSocket (tarayıcı erişimi)
# ==============================================================================

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo


def generate_launch_description():

    # 1. Xvfb: Sanal X11 ekranı (:99)
    # Gazebo'nun görüntüsü buraya yazılır; masaüstüne dokunulmaz.
    xvfb = ExecuteProcess(
        cmd=["Xvfb", ":99", "-screen", "0", "1920x1080x24", "-ac"],
        output="screen",
        name="xvfb_virtual_display",
    )

    # 2. x11vnc: :99 ekranını VNC olarak yayınla (2 sn sonra, Xvfb hazır olsun)
    x11vnc = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "x11vnc",
                    "-display", ":99",
                    "-nopw",       # Şifre yok
                    "-forever",    # İlk bağlantı kesilince çıkma
                    "-shared",     # Çoklu bağlantı izni
                    "-rfbport", "5900",
                ],
                output="screen",
                name="x11vnc_server",
            )
        ],
    )

    # 3. noVNC: VNC → WebSocket köprüsü (3 sn sonra, x11vnc hazır olsun)
    novnc = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "/usr/share/novnc/utils/launch.sh",
                    "--vnc", "localhost:5900",
                    "--listen", "6080",
                ],
                output="screen",
                name="novnc_websocket_proxy",
            )
        ],
    )

    # 4. Hazır olunca bilgi ver
    info = TimerAction(
        period=4.0,
        actions=[
            LogInfo(msg="========================================================"),
            LogInfo(msg="  noVNC hazir! Tarayicidan eris:"),
            LogInfo(msg="  http://localhost:6080/vnc.html"),
            LogInfo(msg=""),
            LogInfo(msg="  Gazebo'yu su sekilde baslat (AYRI TERMINALDE):"),
            LogInfo(msg="  DISPLAY=:99 ros2 launch my_robot_cell_control hil_test_whole_unified.launch.py"),
            LogInfo(msg="========================================================"),
        ],
    )

    return LaunchDescription([
        xvfb,
        x11vnc,
        novnc,
        info,
    ])
