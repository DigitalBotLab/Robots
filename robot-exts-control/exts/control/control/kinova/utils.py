import os
import json
from pathlib import Path
import omni.kit.app

EXTENSION_FOLDER_PATH = str(Path(
    omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
).resolve())

def process_policy_config(mg_config_file):
    mp_config_dir = os.path.dirname(mg_config_file)
    with open(mg_config_file) as config_file:
        config = json.load(config_file)
    rel_assets = config.get("relative_asset_paths", {})
    for k, v in rel_assets.items():
        config[k] = os.path.join(mp_config_dir, v)
    del config["relative_asset_paths"]
    return config