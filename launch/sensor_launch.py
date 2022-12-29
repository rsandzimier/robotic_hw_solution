from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node( # service server node for sensor1
            package='robotic_hw_solution',
            executable='SensorServiceServer.py',
            name='sensor_service_server',
            parameters=[{'host': '127.0.0.3', 'port': 10000,
                                    'sampling_freq': 2000, 'delay': 0.0015}],
            namespace="sensor1"
        ),
        Node( # service server node for sensor1
            package='robotic_hw_solution',
            executable='SensorServiceServer.py',
            name='sensor_service_server',
            parameters=[{'host': '127.0.0.1', 'port': 10000,
                                    'sampling_freq': 4000, 'delay': 0.0035}],
            namespace="sensor2"
        ),
        Node( # service client node for both sensor1 and sensor 2
            package='robotic_hw_solution',
            executable='sensor_service_client',
            name='sensor_service_client',
            parameters=[{"sensor_names": ['sensor1', 'sensor2']}]
        )
    ])
