"""SMORES-EP reference goals, separate from Robot Pack hardware semantics."""

from modsim.planning.models import AssemblyGoal, Face, GoalEdge, Pose2


def mobile_manipulator_goal() -> AssemblyGoal:
    """Liu et al. Task 1, seven modules; Figure 8 in the expanded paper."""
    return AssemblyGoal(
        id="smores_online_assembly",
        nodes=tuple(f"goal_{i}" for i in range(7)),
        edges=(
            _edge("goal_0", "right", "goal_1", "bottom"),
            _edge("goal_0", "pan", "goal_2", "bottom"),
            _edge("goal_0", "left", "goal_3", "bottom"),
            _edge("goal_0", "bottom", "goal_4", "pan"),
            _edge("goal_4", "bottom", "goal_5", "pan"),
            _edge("goal_5", "bottom", "goal_6", "pan"),
        ),
    )


def paper_initial_poses() -> dict[str, Pose2]:
    """Table 1 positions in metres/radians; measured by the paper's authors."""
    values = (
        (0.017, 0.357, 1.142),
        (0.0, 0.0, 0.0),
        (0.305, 0.129, 0.641),
        (-0.318, -0.132, 0.454),
        (-0.318, 0.158, 0.823),
        (0.264, -0.448, -0.763),
        (-0.172, -0.380, -2.431),
    )
    return {f"module_{i}": Pose2(x=x, y=y, yaw=yaw) for i, (x, y, yaw) in enumerate(values)}


def driver_to_snake_goal() -> AssemblyGoal:
    """Explicit physical correspondence for the existing seven-module benchmark."""
    return AssemblyGoal(
        id="smores_online_driver_to_snake",
        nodes=tuple(f"module_{i}" for i in range(1, 8)),
        edges=(
            _edge("module_3", "bottom", "module_1", "pan"),
            _edge("module_2", "bottom", "module_3", "pan"),
            _edge("module_4", "bottom", "module_2", "pan"),
            _edge("module_4", "pan", "module_5", "bottom"),
            _edge("module_5", "pan", "module_6", "bottom"),
            _edge("module_6", "pan", "module_7", "bottom"),
        ),
    )


def _edge(a: str, face_a: Face, b: str, face_b: Face) -> GoalEdge:
    return GoalEdge(a=a, face_a=face_a, b=b, face_b=face_b)
