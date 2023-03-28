import os
import json
import numpy as np
from pathlib import Path
import omni.kit.app

from pxr import UsdGeom, Gf, Usd

EXTENSION_FOLDER_PATH = str(Path(
    omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
).resolve())

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

def look_at_matrix(eye, target, up):
    """
    Look at matrix
    ::params: 
        eye: Gf.Vec3d
        target: Gf.Vec3d
        up: Gf.Vec3d
    """
    zaxis = (target - eye)/(target - eye).Normalize()
    xaxis = Gf.Cross(up, zaxis)
    xaxis = xaxis / xaxis.Normalize()
    yaxis = Gf.Cross(zaxis, xaxis)
    yaxis = yaxis / yaxis.Normalize()
    m = Gf.Matrix4d()
    m.SetRow(0, Gf.Vec4d(xaxis[0], yaxis[0], zaxis[0], 0.0))
    m.SetRow(1, Gf.Vec4d(xaxis[1], yaxis[1], zaxis[1], 0.0))
    m.SetRow(2, Gf.Vec4d(xaxis[2], yaxis[2], zaxis[2], 0.0))
    m.SetRow(3, Gf.Vec4d(0.0, 0.0, 0.0, 1.0))
    m = m * Gf.Matrix4d().SetTranslate(-eye)
    return m