import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    moveit_config = (
        MoveItConfigsBuilder("we_r2_arm", package_name="we_r2_arm_moveit")
        .robot_description(file_path=os.path.join(
            get_package_share_directory("we_r2_arm_description"),
            "urdf",
            "we_r2_arm.urdf.xacro"
            )
        )
        .robot_description_semantic(file_path="config/we_r2_arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(file_path="config/planning_python_api.yaml")
        .to_moveit_configs()
    )

    simple_moveit_interface = Node(
        package="we_r2_arm_moveit",
        executable="arm_controller.py",
        parameters=[moveit_config.to_dict(),
                    {"use_sim_time": True}]
    )
    
    return LaunchDescription([
        simple_moveit_interface,
    ])