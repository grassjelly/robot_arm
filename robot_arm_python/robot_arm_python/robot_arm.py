from abc import ABC, abstractmethod
from geometry_msgs.msg import PoseStamped


class RobotArm(ABC):
    """Abstract base class for robot arm motion interfaces.

    Concrete implementations (e.g. MoveIt) override the three primitive methods.
    Callers use move_to() as a single dispatch entry point or call the primitives
    directly.
    """

    @abstractmethod
    def move_to_pose(self, pose: PoseStamped) -> bool:
        """Move the end-effector to a Cartesian pose goal."""

    @abstractmethod
    def move_to_joint_positions(self, joint_positions: dict) -> bool:
        """Move to a joint-space goal.

        Args:
            joint_positions: mapping of joint_name -> target_value (radians).
        """

    @abstractmethod
    def move_to_configuration(self, configuration_name: str) -> bool:
        """Move to a named configuration defined in the SRDF."""

    def move_to(
        self,
        *,
        pose: PoseStamped = None,
        joint_positions: dict = None,
        configuration_name: str = None,
    ) -> bool:
        """Dispatch to the appropriate primitive based on which argument is set.

        Exactly one of pose, joint_positions, or configuration_name must be
        provided; raises ValueError otherwise.
        """
        provided = [x for x in (pose, joint_positions, configuration_name) if x is not None]
        if len(provided) == 0:
            raise ValueError(
                "move_to() requires exactly one of: pose, joint_positions, configuration_name"
            )
        if len(provided) > 1:
            raise ValueError(
                "move_to() accepts only one argument at a time, but multiple were provided: "
                + ", ".join(
                    name
                    for name, val in [
                        ("pose", pose),
                        ("joint_positions", joint_positions),
                        ("configuration_name", configuration_name),
                    ]
                    if val is not None
                )
            )

        if pose is not None:
            return self.move_to_pose(pose)
        if joint_positions is not None:
            return self.move_to_joint_positions(joint_positions)
        return self.move_to_configuration(configuration_name)
