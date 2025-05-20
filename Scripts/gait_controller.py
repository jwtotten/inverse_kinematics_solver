"""
This script is for controlling the gait of the hexapod and 
for configuring the directionality of the leg based off of its leg instance.
"""

import numpy as np

class GaitController:

    def __init__(self, iksolver):
        self.iksolver = iksolver