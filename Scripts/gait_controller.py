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

class GaitController(IkSolver):
    def __init__(self, femur_length: float, tibia_length: float, number_samples: int = 50):
        """
        Initialize GaitController with leg parameters and gait settings
        """
        super(IkSolver, self).__init__(femur_length, tibia_length)
        
        # Gait parameters
        self.number_samples = number_samples
        self.step_height = 2.0
        self.step_length = 4.0
        self.cycle_time = 1.0
        self.duty_factor = 0.5
        self.phase_offset = 0.0
        
        # Time vector for gait calculation
        self.time = np.linspace(0, self.cycle_time, number_samples)
        
        # Initialize gait cycle
        self._generate_gait_cycle()

    def set_gait_pattern(self, pattern: int) -> None:
        """Set phase offset based on leg instance and gait pattern"""
        instance_number = len(self._instances) - 1
        
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
        leg_stationary = np.zeros((self.number_samples, self.iksolver.y0))
        
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

