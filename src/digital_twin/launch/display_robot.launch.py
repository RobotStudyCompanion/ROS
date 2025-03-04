from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os 
from launch.substitutions import Command
from launch_ros.actions import Node

robot_description = os.path.join(get_package_share_directory('digital_twin'), 'urdf/my_robot.urdf.xacro')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 'install/digital_twin/share/digital_twin/rviz/view_robot3.rviz'],
            output='screen'
        ),
        Node(
	    package='robot_state_publisher',
	    executable='robot_state_publisher',
	    name='robot_state_publisher',
	    output='screen',
	    parameters=[{'robot_description': Command(['xacro ', robot_description])}]
	),
	Node(
		package='joint_state_publisher',
		executable='joint_state_publisher',
		name='joint_state_publisher',
		output='screen'
	)

    ])

