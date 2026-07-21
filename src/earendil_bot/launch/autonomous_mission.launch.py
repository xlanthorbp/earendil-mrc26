import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('earendil_bot')
    params_file = os.path.join(pkg_share, 'config', 'hardware_params.yaml')

    return LaunchDescription([
        # 1. Arduino Hardware Bridge Node (Motorlar, Manyetometre, LED Mod Işıkları)
        Node(
            package='earendil_bot',
            executable='hardware_bridge',
            name='hardware_bridge',
            parameters=[params_file],
            output='screen'
        ),

        # 2. Standart Tekli GPS Alıcı Düğümü
        Node(
            package='earendil_bot',
            executable='gps_node',
            name='gps_node',
            parameters=[params_file],
            output='screen'
        ),

        # 3. ArUco Etiket Algılayıcı Düğüm (Raspberry Pi 5 Kamera)
        Node(
            package='earendil_bot',
            executable='aruco_detector',
            name='aruco_detector',
            parameters=[params_file],
            output='screen'
        ),

        # 4. Ana Otonom Navigasyon Düğümü (10 Hedef, Görsel Override, 360° Tarama)
        Node(
            package='earendil_bot',
            executable='gps_nav_main',
            name='gps_nav_main',
            parameters=[params_file],
            output='screen'
        ),

        # 5. Komut Kaydı ve Rota Tersleme ile Üsse Dönüş Düğümü
        Node(
            package='earendil_bot',
            executable='path_recorder',
            name='path_recorder',
            parameters=[params_file],
            output='screen'
        ),
    ])
