import colorsys
import os
from pathlib import Path
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


NODE_KEY = 'pcd_map_publisher'


def _resolve_directory(package_share, configured_path):
    path = Path(configured_path)
    return path if path.is_absolute() else package_share / path


def _pcd_files(directory):
    if not directory.is_dir():
        raise RuntimeError(f'PCD directory does not exist: {directory}')
    return sorted(
        path for path in directory.iterdir()
        if path.is_file() and path.suffix.lower() == '.pcd'
    )


def _generated_color(index):
    hue = (index * 0.61803398875) % 1.0
    red, green, blue = colorsys.hsv_to_rgb(hue, 0.8, 1.0)
    return f'{round(red * 255)}; {round(green * 255)}; {round(blue * 255)}'


def _point_cloud_display(name, topic, color, point_size):
    return {
        'Alpha': 1,
        'Class': 'rviz_default_plugins/PointCloud2',
        'Color': color,
        'Color Transformer': 'FlatColor',
        'Decay Time': 0,
        'Enabled': True,
        'Name': name,
        'Position Transformer': 'XYZ',
        'Selectable': True,
        'Size (Pixels)': point_size,
        'Size (m)': 0.01,
        'Style': 'Points',
        'Topic': {
            'Depth': 1,
            'Durability Policy': 'Transient Local',
            'Filter size': 10,
            'History Policy': 'Keep Last',
            'Reliability Policy': 'Reliable',
            'Value': topic,
        },
        'Use Fixed Frame': True,
        'Value': True,
    }


def _write_rviz_config(reference_file, eval_files, parameters):
    visualization = parameters.get('visualization', {})
    reference_color = visualization.get('reference_color', '255; 255; 255')
    configured_colors = visualization.get('eval_colors', [])
    point_size = int(visualization.get('point_size', 2))
    grid_cell_size = float(visualization.get('grid_cell_size', 5.0))

    displays = [{
        'Alpha': 0.35,
        'Cell Size': grid_cell_size,
        'Class': 'rviz_default_plugins/Grid',
        'Color': '100; 100; 100',
        'Enabled': True,
        'Name': 'Grid',
        'Plane': 'XY',
        'Plane Cell Count': 50,
        'Reference Frame': '<Fixed Frame>',
        'Value': True,
    }]
    displays.append(_point_cloud_display(
        f'[REF] {reference_file.name}', '/pcd_compare/ref/points',
        reference_color, point_size + 1))
    for index, eval_file in enumerate(eval_files):
        color = (
            configured_colors[index]
            if index < len(configured_colors)
            else _generated_color(index)
        )
        displays.append(_point_cloud_display(
            f'[EVAL] {eval_file.name}', f'/pcd_compare/eval/map_{index}/points',
            color, point_size))

    config = {
        'Panels': [{'Class': 'rviz_common/Displays', 'Name': 'Displays'}],
        'Visualization Manager': {
            'Class': '',
            'Displays': displays,
            'Enabled': True,
            'Global Options': {
                'Background Color': '32; 32; 32',
                'Fixed Frame': 'world',
                'Frame Rate': 30,
            },
            'Name': 'root',
            'Tools': [
                {'Class': 'rviz_default_plugins/Interact'},
                {'Class': 'rviz_default_plugins/MoveCamera'},
                {'Class': 'rviz_default_plugins/Select'},
            ],
            'Views': {
                'Current': {
                    'Class': 'rviz_default_plugins/Orbit',
                    'Distance': 250,
                    'Name': 'Current View',
                    'Target Frame': 'world',
                },
                'Saved': None,
            },
        },
        'Window Geometry': {'Height': 900, 'Width': 1600},
    }
    temporary = tempfile.NamedTemporaryFile(
        mode='w', prefix='pcd_compare_', suffix='.rviz', delete=False)
    with temporary:
        yaml.safe_dump(config, temporary, sort_keys=False)
    return temporary.name


def _launch_setup(context):
    package_share = Path(get_package_share_directory('pcd_compare'))
    config_path = package_share / 'config' / 'config.yaml'
    with config_path.open(encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file)
    try:
        parameters = config[NODE_KEY]['ros__parameters']
    except (KeyError, TypeError) as error:
        raise RuntimeError(f'Invalid node parameter structure in {config_path}') from error

    reference_files = _pcd_files(_resolve_directory(
        package_share, parameters['ref_directory']))
    eval_files = _pcd_files(_resolve_directory(
        package_share, parameters['eval_directory']))
    if len(reference_files) != 1:
        raise RuntimeError(
            f'Reference directory must contain exactly one PCD; found {len(reference_files)}')

    rviz_path = _write_rviz_config(reference_files[0], eval_files, parameters)

    def remove_generated_rviz(_event, _context):
        try:
            os.unlink(rviz_path)
        except FileNotFoundError:
            pass

    return [
        Node(
            package='pcd_compare',
            executable='pcd_map_publisher',
            parameters=[str(config_path)],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'world', '--child-frame-id', parameters['frame_id'],
            ],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_path],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            output='screen',
        ),
        RegisterEventHandler(OnShutdown(on_shutdown=remove_generated_rviz)),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        OpaqueFunction(function=_launch_setup),
    ])
