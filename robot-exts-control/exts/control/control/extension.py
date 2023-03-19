import omni.ext
import omni.ui as ui

from typing import Optional, List
import numpy as np
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.prims.rigid_prim import RigidPrim
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units
import carb
from omni.isaac.manipulators.grippers.parallel_gripper import ParallelGripper

from .kinova.kinova import Kinova

# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class ControlExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[control] control startup")

        self._count = 0

        self._window = ui.Window("Robot control", width=300, height=300)
        with self._window.frame:
            with ui.VStack():
                ui.Button("Set Robot", height = 20, clicked_fn=self.set_robot)
                ui.Button("Debug", height = 20, clicked_fn=self.debug)
                    
    
    def set_robot(self):
        print("set_robot")
        prim_path = "/World/kinova"
        self.kinova = Kinova(prim_path = prim_path, name = "kinova_robot")
        self.kinova.initialize()
        print("kinova_info", self.kinova.num_dof)
        print("kinova_gripper", self.kinova._gripper._articulation_num_dofs)
    
    def debug(self):
        print("debug")
        print("get_joint_positions: ", self.kinova.get_joint_positions())
        

    def on_shutdown(self):
        print("[control] control shutdown")
