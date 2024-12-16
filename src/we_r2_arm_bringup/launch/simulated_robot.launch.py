import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch = LaunchDescription()

    gazebo = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("we_r2_arm_description"),
                "launch",
                "gazebo.launch.py"
            )
        )
    
    controller = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("we_r2_arm_controller"),
                "launch",
                "controller.launch.py"
            ),
            launch_arguments={"is_sim": "True"}.items()
        )
    
    moveit = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("we_r2_arm_moveit"),
                "launch",
                "moveit.launch.py"
            ),
            launch_arguments={"is_sim": "True"}.items()
        )

    launch.add_action(gazebo)
    launch.add_action(controller)
    launch.add_action(moveit)
    
    return launch