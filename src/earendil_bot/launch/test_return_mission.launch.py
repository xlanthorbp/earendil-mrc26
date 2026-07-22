import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('earendil_bot')
    hardware_params = os.path.join(pkg_share, 'config', 'hardware_params.yaml')
    test_params = os.path.join(pkg_share, 'config', 'test_params.yaml')

    # Launch Argümanları (Opsiyonel olarak komut satırından yeni hedef verilebilir)
    target_lat_arg = DeclareLaunchArgument(
        'target_lat',
        default_value='39.9017797',
        description='1. Hedef GPS Enlemi'
    )
    target_lon_arg = DeclareLaunchArgument(
        'target_lon',
        default_value='32.7704813',
        description='1. Hedef GPS Boylamı'
    )
    target2_lat_arg = DeclareLaunchArgument(
        'target2_lat',
        default_value='39.9017482',
        description='2. Hedef GPS Enlemi'
    )
    target2_lon_arg = DeclareLaunchArgument(
        'target2_lon',
        default_value='32.7704942',
        description='2. Hedef GPS Boylamı'
    )

    return LaunchDescription([
        target_lat_arg,
        target_lon_arg,
        target2_lat_arg,
        target2_lon_arg,

        # 1. Arduino Motor ve Manyetometre Haberleşme Düğümü
        Node(
            package='earendil_bot',
            executable='hardware_bridge',
            name='hardware_bridge',
            parameters=[hardware_params],
            output='screen'
        ),

        # 2. Standart Tekli GPS Alıcı Düğümü
        Node(
            package='earendil_bot',
            executable='gps_node',
            name='gps_node',
            parameters=[hardware_params],
            output='screen'
        ),

        # 3. Komut Kaydı ve Rota Tersleme ile Üsse Dönüş Düğümü
        Node(
            package='earendil_bot',
            executable='path_recorder',
            name='path_recorder',
            parameters=[test_params],
            output='screen'
        ),

        # 4. Zamanlı Test Navigasyon Düğümü (5s ileri, 2s sağ, 2s ileri, 1s sağ -> 180° Dönüş & Üsse Geri)
        Node(
            package='earendil_bot',
            executable='timed_nav_test',
            name='timed_nav_test',
            parameters=[test_params],
            output='screen'
        ),
    ])
