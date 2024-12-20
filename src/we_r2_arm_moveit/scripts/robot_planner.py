from moveit.planning import MoveItPy


class RobotPlanner():
    _moveit_instance = None

    def __init__(self):
        self._moveit_instance = MoveItPy(node_name="moveit_py")
        self.setup_arm()

    def get_moveit_planner(self):
        return self._moveit_instance

    def setup_arm(self):
        pass
