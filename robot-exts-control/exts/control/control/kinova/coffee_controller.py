from omni.isaac.core.controllers import BaseController
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.franka.controllers.rmpflow_controller import RMPFlowController

from .kinova import Kinova

class CoffeeMakerController(BaseController):
    def __init__(self, name: str, kinova: Kinova) -> None: 
        BaseController.__init__(self, name=name)
        self._event = 0
        self.kinova = kinova
        self._gripper = self.kinova.gripper
        
        # TODO：find height
        self._end_effector_initial_height = 0.4 / get_stage_units()
        self._pause = False
        self._controller = RMPFlowController(name="pickplace_cspace_controller", robot_articulation=self._franka)

   