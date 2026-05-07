"""
This script is for controlling the gait of the hexapod and 
for configuring the directionality of the leg based off of its leg instance.
"""

import numpy as np
from .iksolver import IkSolver

class GaitPattern:
    TRIPOD = 0
    WAVE = 1
    RIPPLE = 2

class GaitController:
    def __init__(self, iksolver: IkSolver, number_samples: int = 50):
        """
        Initialize GaitController using an existing IkSolver instance
        """
        if not isinstance(iksolver, IkSolver):
            raise TypeError("iksolver must be an instance of IkSolver")
        if not isinstance(number_samples, int):
            raise TypeError("number_samples must be an integer")
        self.iksolver = iksolver
        self.number_samples = number_samples
        
        # Mirror IkSolver properties
        self.x_length = self.iksolver.x_length
        self.y_length = self.iksolver.y_length
        self.z_length = self.iksolver.z_length
        
        # Gait parameters
        self.step_height = 2.0
        self.step_length = 4.0
        self.cycle_time = 1.0
        self.duty_factor = 0.5
        self.phase_offset = 0.0
        
        # Create time vector for the gait cycle
        self.time = np.linspace(0, self.cycle_time, self.number_samples)
        
        # Initialize motion vectors
        self.x_targets = None
        self.y_targets = None
        self.z_targets = None
        
        # Generate initial gait cycle
        self._generate_gait_cycle()

    def solve_leg_position_from_target_coordinates(self, x: float, y: float, z: float, verbose: bool = False) -> list:
        """
        Delegate to IkSolver's method for coordinate calculation
        """
        return self.iksolver.solve_leg_position_from_target_coordinates(x, y, z, verbose)

    def get_motion(self) -> list:
        """
        Return motion vectors in same format as IkSolver
        """
        if any(x is None for x in [self.x_targets, self.y_targets, self.z_targets]):
            self._generate_gait_cycle()
        return [self.x_targets, self.y_targets, self.z_targets]

    def set_gait_pattern(self, pattern: int) -> None:
        """Set phase offset based on leg instance and gait pattern"""
        instance_number = len(self.iksolver._instances) - 1

        if pattern == GaitPattern.TRIPOD:
            self.phase_offset = 0.5 if instance_number % 2 else 0
        elif pattern == GaitPattern.WAVE:
            self.phase_offset = (instance_number * 0.167) % 1
        elif pattern == GaitPattern.RIPPLE:
            self.phase_offset = (instance_number * 0.25) % 1
        
        self._generate_gait_cycle()

    def _generate_gait_cycle(self) -> None:
        """Generate gait cycle coordinates with phase offset"""
        vertical_motion = np.concatenate(self.lift_leg_equation())
        horizontal_motion = np.concatenate(self.carry_leg_equation())
        
        # Apply phase offset
        offset_samples = int(self.number_samples * self.phase_offset)
        self.x_targets = np.roll(horizontal_motion, offset_samples)
        self.y_targets = np.zeros_like(horizontal_motion)
        self.z_targets = np.roll(vertical_motion, offset_samples)

    def lift_leg_equation(self) -> tuple:
        """
        Calculate vertical leg movement using a sine wave pattern.
        """
        # Split the cycle into stance and swing phases
        mid_point = int(self.number_samples * self.duty_factor)
        
        # Ground contact phase (stance)
        stance_phase = np.zeros(mid_point)
        
        # Lift phase (swing)
        swing_phase = self.step_height * np.sin(
            np.linspace(0, np.pi, self.number_samples - mid_point)
        )
        
        return stance_phase, swing_phase

    def stationary_leg_equation(self) -> list:
        """
        This function is used to calculate the sample positions for the leg when the leg is to 
        be held at a constant position for the full gait cycle.
        :return: leg_stationary :: a list with the leg at a fixed position for the full gait cycle.
        :rtype: list
        """
        leg_stationary = np.zeros(self.number_samples)
        
        return leg_stationary
    
    def carry_leg_equation(self) -> tuple:
        """
        Calculate horizontal leg movement.
        """
        # Time for each phase
        stance_time = self.time[:int(self.number_samples * self.duty_factor)]
        swing_time = self.time[int(self.number_samples * self.duty_factor):]
        
        # Stance phase (moving backward)
        stance_phase = -self.step_length * (stance_time / max(stance_time))
        
        # Swing phase (moving forward)
        swing_phase = self.step_length * (swing_time / max(swing_time))
        
        return stance_phase, swing_phase

    def validate_parameters(self) -> None:
        """
        Validate gait parameters.
        """
        if self.duty_factor <= 0 or self.duty_factor >= 1:
            raise ValueError("Duty factor must be between 0 and 1")
        if self.step_height <= 0:
            raise ValueError("Step height must be positive")
        if self.step_length <= 0:
            raise ValueError("Step length must be positive")

