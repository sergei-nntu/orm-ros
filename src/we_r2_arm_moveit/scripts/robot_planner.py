from moveit.planning import MoveItPy


class RobotPlanner():
    _we_r2_arm = None

    def __init__(self):
        self._we_r2_arm = MoveItPy(node_name="moveit_py")
        self.setup_arm()

    def get_moveit_planner(self):
        return self._we_r2_arm

    def setup_arm(self):
        pass