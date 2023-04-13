import os
import json
import numpy as np
from pathlib import Path
import omni.kit.app

import omni.usd
from pxr import UsdGeom, Gf, Usd, UsdPhysics

EXTENSION_FOLDER_PATH = str(Path(
    omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
).resolve())

    
def fix_damping_and_stiffness(kinova_path = "/World/kinova_gen3_7_hand/kinova", stiffness = 1e3, damping = 1e6):
        print("fixing damping and stiffness")
        # stiffness_name = "drive:angular:physics:stiffness"
        # damping_name = "drive:angular:physics:damping"
        stage = omni.usd.get_context().get_stage()
        joint_prim_paths = [
            "/base_link/Actuator1",
            "/shoulder_link/Actuator2",
            "/half_arm_1_link/Actuator3",
            "/half_arm_2_link/Actuator4",
            "/forearm_link/Actuator5",
            "/spherical_wrist_1_link/Actuator6",
            "/spherical_wrist_2_link/Actuator7",
        ]

        for joint_prim_path in joint_prim_paths:
            joint_prim = stage.GetPrimAtPath(kinova_path + joint_prim_path)
            joint_driver = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
            joint_driver.GetStiffnessAttr().Set(stiffness)
            joint_driver.GetDampingAttr().Set(damping)

def process_policy_config(mg_config_file):
    """
    Process the policy config file to get the absolute path of the assets"""
    mp_config_dir = os.path.dirname(mg_config_file)
    with open(mg_config_file) as config_file:
        config = json.load(config_file)
    rel_assets = config.get("relative_asset_paths", {})
    for k, v in rel_assets.items():
        config[k] = os.path.join(mp_config_dir, v)
    del config["relative_asset_paths"]
    return config


def regulate_degree(degree: float, min_value: float = 0, max_value: float = 360, indegree: bool = True):
    """
    Regulate the degree to be in the range of [min_value, max_value]"""
    if not indegree:
        degree = np.rad2deg(degree)

    if degree < min_value:
        degree += 360
    elif degree > max_value:
        degree -= 360
    
    return degree


def get_transform_mat_from_pos_rot(p, q):
    """
    Get transform matrix from position and rotation
    """
    trans = Gf.Transform()
    rotation = Gf.Rotation(Gf.Quatd(float(q[0]), float(q[1]), float(q[2]), float(q[3])))
    trans.SetRotation(rotation)
    trans.SetTranslation(Gf.Vec3d(float(p[0]), float(p[1]), float(p[2])))
    return trans.GetMatrix()


def get_prim_pickup_transform(stage, prim_path: str, offset: Gf.Vec3d):
    """
    Get the pickup transform of the prim with offset"""
    prim = stage.GetPrimAtPath(prim_path)
    xformable = UsdGeom.Xformable(prim)
    mat0 = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    target_pos = mat0.ExtractTranslation()
    

    xaxis = -offset / offset.GetLength()
    yaxis = Gf.Cross(Gf.Vec3d(0, 0, 1), xaxis)
    m = Gf.Matrix4d()
    m.SetRow(0, Gf.Vec4d(xaxis[0], yaxis[0], 0, 0.0))
    m.SetRow(1, Gf.Vec4d(xaxis[1], yaxis[1], 0, 0.0))
    m.SetRow(2, Gf.Vec4d(xaxis[2], yaxis[2], 1, 0.0))
    m.SetRow(3, Gf.Vec4d(0.0, 0.0, 0.0, 1.0))

    eye_pos = target_pos + offset
    m = m * Gf.Matrix4d().SetTranslate(eye_pos)

    print("translation: ", eye_pos)
    print("rotation: ", m.ExtractRotationQuat())

    return eye_pos, m.ExtractRotationQuat()

def generate_slerp_action_sequence(ori_pos, ori_quat, rel_rot, 
                                   sub_steps = 5, sub_duration = 50, 
                                   slerp_last = True, slerp_offset = [0 ,0, 0]):
    """
    Generate slerp action sequence from relative position and rotation
    """
    slerp_action_sequence = []
    ori_pos = Gf.Vec3d(ori_pos[0], ori_pos[1], ori_pos[2])
    rel_quat = Gf.Quatd(float(rel_rot[0]), float(rel_rot[1]), float(rel_rot[2]), float(rel_rot[3])).GetNormalized()
    ori_quat = Gf.Quatd(float(ori_quat[0]), float(ori_quat[1]), float(ori_quat[2]), float(ori_quat[3])).GetNormalized()
    identity_quat = Gf.Quatd(1, 0, 0, 0)
    for i in range(1, sub_steps):
        t = (i + int(slerp_last)) / sub_steps
        
        quat_rel = Gf.Slerp(t, identity_quat, rel_quat)
        p = (quat_rel * Gf.Quatd(0, ori_pos + Gf.Vec3d(*slerp_offset) * i) * quat_rel.GetInverse()).GetImaginary()
        q = quat_rel * ori_quat
        slerp_action_sequence.append(
            {
                'action_type': 'move',
                'duration': sub_duration,
                'position': [p[0], p[1], p[2]],
                'orientation': [q.GetReal(), q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2]]
            },
        )

    return slerp_action_sequence