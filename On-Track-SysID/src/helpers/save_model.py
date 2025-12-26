import yaml
import os

# Use ament_index for package path lookup (ROS2)
try:
    from ament_index_python.packages import get_package_share_directory
    USE_AMENT = True
except ImportError:
    USE_AMENT = False

def get_package_path():
    """Get package path using ament_index or fallback"""
    if USE_AMENT:
        try:
            return get_package_share_directory('on_track_sys_id')
        except Exception:
            pass
    # Fallback to relative path
    current_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.dirname(os.path.dirname(current_dir))

def save(model, overwrite_existing=True, verbose=False):

    package_path = get_package_path()
    file_path = package_path + "/models/" + model['model_name'] +"/" + model['model_name'] +"_"+ model['tire_model'] + ".txt"
    if os.path.isfile(file_path):
        if (verbose): print("Model already exists")
        if overwrite_existing:
            if (verbose): print("Overwriting...")
        else:
            if (verbose): print("Not overwriting.")
            return 0

    try:
        model = model.to_dict()
    except:
        model = model

    # Write data to the file
    with open(file_path, "w") as f:
        print(f"[INFO] MODEL IS SAVED TO: {file_path}")
        yaml.dump(model, f, default_flow_style=False)