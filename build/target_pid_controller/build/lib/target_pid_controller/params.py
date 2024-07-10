# robot_params.py

from dataclasses import dataclass

@dataclass
class RobotParams:
    target_x: float
    target_y: float
    linear_velocity: float
    angular_velocity: float
