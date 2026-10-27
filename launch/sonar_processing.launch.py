from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableLifecycleNode


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare('sonar_processing'), 'config', 'sonar_processing.yaml'])
    
    sonar_remappings = [
        ('sonar/echo/data', 'perception/echo/data'),
        ('sonar/echo/data/processed', 'perception/echo/data/processed'),
        ('sonar/echo/data/cfar', 'perception/echo/data/cfar'),
    ]

    container = ComposableNodeContainer(
        name="sonar_processing_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableLifecycleNode(
                package="sonar_processing",
                plugin="sonar_processing::SonarProcessingNode",
                name="sonar_processing",
                namespace=LaunchConfiguration("namespace"),
                remappings=sonar_remappings,
                parameters=[LaunchConfiguration("params_file")],
                extra_arguments=[{"use_intra_process_comms": True}],
                autostart=False,
            ),
        ],
        output="screen",
    )

    def lifecycle_startup_actions(context):
        namespace = LaunchConfiguration("namespace").perform(context).strip("/")
        full_node_name = f"/{namespace}/sonar_processing" if namespace else "/sonar_processing"

        if LaunchConfiguration("autostart").perform(context).strip().lower() != "true":
            return []

        lifecycle_cmd = (
            f'until ros2 lifecycle set {full_node_name} configure; do '
            'echo "Waiting for configure service..."; '
            'sleep 1; '
            'done; '
            f'until ros2 lifecycle set {full_node_name} activate; do '
            'echo "Waiting for activate service..."; '
            'sleep 1; '
            'done'
        )

        return [
            ExecuteProcess(
                cmd=["bash", "-lc", lifecycle_cmd],
                output="screen",
            ),
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace for the sonar processing lifecycle node."
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_config,
            description="YAML file with the sonar processing parameters."
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Whether to automatically transition the node to 'active' state."
        ),
        container,
        OpaqueFunction(function=lifecycle_startup_actions),
    ])
