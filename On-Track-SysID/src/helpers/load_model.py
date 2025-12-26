import yaml
import os
from .dotdict import DotDict

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

def get_dict(model_name):
    model, tire = model_name.split("_")
    package_path = get_package_path()
    with open(f'{package_path}/models/{model}/{model_name}.txt', 'rb') as f:
        params = yaml.load(f, Loader=yaml.Loader)
    
    return params

def get_dotdict(model_name):
    dict = get_dict(model_name)
    params = DotDict(dict)
    return params