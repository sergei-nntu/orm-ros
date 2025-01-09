import os
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_moveit_rviz = LaunchConfiguration("use_moveit_rviz")

    declare_moveit_rviz_cmd = DeclareLaunchArgument(
        "use_moveit_rviz", default_value="False", description="Use moveit rviz"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("we_r2_arm_description"),
                "launch",
                "gazebo.launch.py"
            )
        )
    )
    
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("we_r2_arm_controller"),
                "launch",
                "controller.launch.py"
            )
        ),
        launch_arguments={"is_sim": "True"}.items()
    )
    
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("we_r2_arm_moveit"),
                "launch",
                "moveit.launch.py"
            )
        ),
        launch_arguments={"is_sim": "True"}.items(),
        condition=IfCondition(PythonExpression(['not ', use_moveit_rviz]))
    )

    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("we_r2_arm_moveit"),
                "launch",
                "moveit_rviz.launch.py"
            )
        ),
        launch_arguments={"is_sim": "True"}.items(),
        condition=IfCondition(PythonExpression([use_moveit_rviz]))
    )

    launch = LaunchDescription()

    launch.add_action(declare_moveit_rviz_cmd)

    launch.add_action(gazebo)
    launch.add_action(controller)
    launch.add_action(moveit)
    launch.add_action(moveit_rviz)

    return launch
