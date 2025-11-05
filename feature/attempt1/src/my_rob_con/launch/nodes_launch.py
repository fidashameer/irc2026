from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_rob_con',
            executable='cone_detection',
            name='cone_detection_node',
            output='screen',
            parameters=[{
                'model_path': '/home/fidashameer/ros2_ws1/src/my_rob_con/rado/best.pt',
                'camera_topic': '/image_raw',
                'visualize': True
            }]
        ),
        Node(
            package='my_rob_con',
            executable='rotation_node',
            name='rotation_node',
            output='screen',
        ),
        Node(
            package='my_rob_con',
            executable='movement_node',
            name='movement_node',
            output='screen',
            parameters=[{'directions': ['north', 'south', 'east', 'west']}]
        )
    ])


##rm -rf build install log
##colcon build --cmake-clean-cache --packages-select my_rob_con --event-handlers console_direct+

##source install/setup.bash
##ros2 launch my_rob_con nodes_launch.py



##mkdir -p install/my_rob_con/lib/my_rob_con
##ln -s ../../bin/cone_detection install/my_rob_con/lib/my_rob_con/cone_detection
##ln -s ../../bin/movement_node install/my_rob_con/lib/my_rob_con/movement_node
##ln -s ../../bin/rotation_node install/my_rob_con/lib/my_rob_con/rotation_node
