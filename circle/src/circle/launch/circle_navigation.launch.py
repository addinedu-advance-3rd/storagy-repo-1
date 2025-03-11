from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        # ArUco 도킹 노드 (먼저 실행)
        Node(
            package='circle',
            executable='aruco_dock',
            name='aruco_dock',
            output='screen'
        ),
        
        # 90도 회전 노드 (2초 후 실행)
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='circle',
                    executable='rotate90',
                    name='rotate90',
                    output='screen'
                )
            ]
        ),
        
        # 목표 지점 이동 노드 (5초 후 실행)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='circle',
                    executable='send_home',
                    name='send_home',
                    output='screen'
                )
            ]
        )
    ]) 