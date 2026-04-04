from pathlib import Path

from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from moveit.utils import create_params_file_from_dict
from moveit_configs_utils import MoveItConfigsBuilder

from .robot_arm import RobotArm

# MoveItConfigsBuilder calls sensors_3d() unconditionally; monkeypatch it out
# because the default sensors_3d.yaml format causes a KeyError when empty.
MoveItConfigsBuilder.sensors_3d = lambda self, *args, **kwargs: self


class MoveIt(RobotArm):
    """RobotArm implementation backed by MoveItPy.

    MoveItPy manages its own internal ROS node and executor — no need to add
    this object to an external executor.

    Args:
        planning_group:  MoveIt planning group name (default: "robot_arm").
        pose_link:       Link name used as the end-effector for pose goals
                         (default: "tool_link").
        robot_name:      robot_name passed to MoveItConfigsBuilder.
        package_name:    ROS package that contains the MoveIt config files.
        use_sim_time:    Set True when running in Gazebo simulation.
    """

    def __init__(
        self,
        planning_group: str = "robot_arm",
        pose_link: str = "tool_link",
        robot_name: str = "robot_arm",
        package_name: str = "robot_arm_moveit_config",
        use_sim_time: bool = True,
    ) -> None:
        self._pose_link = pose_link

        config = (
            MoveItConfigsBuilder(robot_name=robot_name, package_name=package_name)
            .robot_description_semantic(Path("config") / "robot_arm.srdf")
            .trajectory_execution(Path("config") / "ros_controllers.yaml")
            .moveit_cpp(Path("config") / "moveit_cpp.yaml")
            .to_moveit_configs()
            .to_dict()
        )
        # use_sim_time must be set on the MoveItPy node itself, not only inside
        # planning_scene_monitor_options, to keep the clock in sync with Gazebo.
        # See: https://github.com/moveit/moveit2/issues/2906#issuecomment-2259711256
        config["use_sim_time"] = use_sim_time

        params_file = create_params_file_from_dict(config, "/**")
        self._robot = MoveItPy(node_name="moveit_py", launch_params_filepaths=[params_file])
        self._arm = self._robot.get_planning_component(planning_group)
        self._robot_model = self._robot.get_robot_model()

    # ------------------------------------------------------------------
    # RobotArm interface
    # ------------------------------------------------------------------

    def move_to_pose(self, pose: PoseStamped) -> bool:
        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(pose_stamped_msg=pose, pose_link=self._pose_link)
        return self._plan_and_execute()

    def move_to_joint_positions(self, joint_positions: dict) -> bool:
        robot_state = RobotState(self._robot_model)
        current = robot_state.joint_positions
        current.update(joint_positions)
        robot_state.joint_positions = current
        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(robot_state=robot_state)
        return self._plan_and_execute()

    def move_to_configuration(self, configuration_name: str) -> bool:
        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(configuration_name=configuration_name)
        return self._plan_and_execute()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _plan_and_execute(self) -> bool:
        plan_result = self._arm.plan()
        if plan_result:
            self._robot.execute(plan_result.trajectory, controllers=[])
            return True
        return False
