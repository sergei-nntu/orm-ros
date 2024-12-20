import numpy as np
from rclpy.logging import get_logger
from moveit.core.robot_state import RobotState # type: ignore

from utils import joints_state_validation


class Controller():
    def __init__(self, moveit_planner, controller_name):
        self._number_of_joints = 0 
        self._moveit_instance = moveit_planner
        self._controller_name = controller_name
        self._entity = moveit_planner.get_planning_component(controller_name)
        self._entity_state = RobotState(moveit_planner.get_robot_model())
        self._entity_plan_result = None

    def plan_and_execute(self):
        if self.plan():
            self.execute()

    def plan(self):
        self._entity_plan_result = self._entity.plan()
        return self._entity_plan_result

    def execute(self):
        if self._entity_plan_result:
            get_logger(f"{self._controller_name}_controller").info("Planner SUCCEED, moving...")
            self._moveit_instance.execute(self._entity_plan_result.trajectory, controllers=[])
        else:
            get_logger(f"{self._controller_name}_controller").error("The planner failed!")

    def set_joints_state(self, joint_positions):
        joints_state_validation(joint_positions, self._number_of_joints)

        self._entity_state.set_joint_group_positions(self._controller_name, np.array(joint_positions))

        self._entity.set_start_state_to_current_state()
        self._entity.set_goal_state(robot_state=self._entity_state)

    def set_number_of_joints(self, value):
        self._number_of_joints = value