from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command


def generate_launch_description():
    return LaunchDescription([

        # 1. Start Cartographer immediately
        ExecuteProcess(
            cmd=[
                'ros2', 'launch',
                'turtlebot3_cartographer', 'cartographer.launch.py',
                'use_sim_time:=False'
            ],
            output='screen'
        ),

        # 2. Start Nav2 Bringup
        TimerAction(
            period=0.3,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'launch',
                        'nav2_bringup', 'navigation_launch.py',
                        ['map:=', Command([
                            'ros2 pkg prefix turtlebot3_navigation2'
                        ]), '/share/turtlebot3_navigation2/map/map.yaml'],
                        ['params_file:=', Command([
                            'ros2 pkg prefix turtlebot3_navigation2'
                        ]), '/share/turtlebot3_navigation2/param/humble/burger.yaml'],
                        'use_sim_time:=False'
                    ],
                    output='screen'
                )
            ]
        ),

        # 3. Start Nav2 RViz
        TimerAction(
            period=0.5,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'launch',
                        'nav2_bringup', 'rviz_launch.py',
                        'use_sim_time:=False'
                    ],
                    output='screen'
                )
            ]
        ),

        # 4. Start coordinator
        TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run',
                        'cde2310_g4_ay2526', 'coordinator'
                    ],
                    output='screen'
                )
            ]
        ),

    ])