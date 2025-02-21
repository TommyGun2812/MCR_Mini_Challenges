from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    signal_generator = Node(package='signal_processing',
                       executable='signal_generator',
                       output='screen'
                       )
    
    process = Node(package='signal_processing',
                         executable='process',
                         output='screen'
                         )
    
    rqt_node = Node(name='rqt_plot',
                    package='rqt_plot',
                    executable='rqt_plot',
                    arguments=['signal/data',
    'proc_signal/data'])
    l_d = LaunchDescription([signal_generator, process, rqt_node])
    return l_d
