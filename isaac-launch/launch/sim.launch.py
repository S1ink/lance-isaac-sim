import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node


# def make_scan_transformer(output_frame : str, override_frame : str, sub_topic, pub_topic):
#     return Node(
#         package = 'debug_tools',
#         executable = 'scan_transformer',
#         output = 'screen',
#         parameters = [{
#             'target_frame': output_frame,
#             'override_frame': override_frame
#         }],
#         remappings = [
#             ('input_scan', sub_topic),
#             ('transformed_scan', pub_topic)
#         ]
#     )

# def make_imu_transformer(output_frame : str, override_frame : str, sub_topic, pub_topic):
#     return Node(
#         package = 'debug_tools',
#         executable = 'imu_transformer',
#         output = 'screen',
#         parameters = [{
#             'target_frame': output_frame,
#             'override_frame': override_frame
#         }],
#         remappings = [
#             ('input_imu', sub_topic),
#             ('transformed_imu', pub_topic)
#         ]
#     )

# def make_imu_visualizer(topic):
#     return Node(
#         package = 'debug_tools',
#         executable = 'imu_visualizer',
#         output = 'screen',
#         parameters = [{ 'imu_topic': topic }]
#     )

# def make_accuracy_analyzer(active_frame : str = 'base_link', origin_frame : str = 'map', validation_frame : str = 'gz_base_link', sample_window : float = 0.25, is_sim : bool = True, remap = '/accuracy_analysis'):
#     return Node(
#         package = 'debug_tools',
#         executable = 'accuracy_analyzer',
#         output = 'screen',
#         parameters = [{
#             'origin_frame_id': origin_frame,
#             'active_frame_id': active_frame,
#             'validation_frame_id': validation_frame,
#             'std_sample_window_s': sample_window,
#             'use_sim_time': is_sim
#         }],
#         remappings = [
#             ('accuracy_analysis', remap)
#         ]
#     )

def generate_launch_description():

    sim_pkg_path = get_package_share_directory('isaac_launch')

    # launch isaac sim
    isaac_sim = ExecuteProcess(
        cmd = [
            PythonExpression(["'", LaunchConfiguration('isaac-root'), "' + 'python.sh'" ]),
            os.path.join(sim_pkg_path, 'launch', 'sim.py'),
            '--gui', 'true',
            '--assets', os.path.join(sim_pkg_path, 'assets/')
        ],
        output = 'screen'
    )
    # launch xbox control
    launch_xbox_ctrl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg_path, 'launch', 'xbox_ctrl.launch.py')
        )
    )
    # tag detector
    # tag_detector = Node(
    #     name = 'tags_detector',
    #     package = 'cardinal_perception',
    #     executable = 'tag_detection_node',
    #     output = 'screen',
    #     parameters = [
    #         os.path.join(get_package_share_directory('cardinal_perception'), 'config', 'tag_detection.yaml'),
    #         { 'use_sim_time': True }
    #     ],
    #     remappings = [ ('tags_detections', '/cardinal_perception/tags_detections') ]
    # )
    # robot state publisher
    state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg_path, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments = { 'use_sim_time' : 'true' }.items()
    )
    # localization
    perception_node = Node(
        name = 'cardinal_perception',
        package = 'cardinal_perception',
        executable = 'perception_node',
        output = 'screen',
        parameters = [
            os.path.join(get_package_share_directory('cardinal_perception'), 'config', 'perception.yaml'),
            { 'use_sim_time': True }
        ],
        remappings = [
            ('filtered_scan', '/cardinal_perception/filtered_scan'),
            ('tags_detections', '/cardinal_perception/tags_detections'),
            ('map_cloud', '/cardinal_perception/map_cloud')
        ],
        # prefix=['xterm -e gdb -ex run --args']
    )
    # foxglove
    foxglove_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg_path, 'launch', 'foxglove.launch.py')
        ),
        launch_arguments = {'use_sim_time': 'true'}.items(),
        condition = IfCondition(LaunchConfiguration('foxglove', default='true'))
    )

    return LaunchDescription([
        DeclareLaunchArgument('isaac-root', default_value='~/.local/share/ov/pkg/isaac-sim-4.2.0/'),
        DeclareLaunchArgument('foxglove', default_value='true'),
        isaac_sim,
        # launch_xbox_ctrl,
        # tag_detector,
        state_publisher,
        perception_node,
        foxglove_node,
        # make_accuracy_analyzer('base_link', 'map', 'gz_base_link', 0.25, True, 'localization_acc_analysis'),
        # make_accuracy_analyzer('base_link_e0', 'map', 'gz_base_link', 0.25, True, 'tags_detection_acc_analysis')
    ])
