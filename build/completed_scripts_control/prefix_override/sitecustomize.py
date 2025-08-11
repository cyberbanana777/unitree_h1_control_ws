import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/banana-killer/unitree_workspaces_and_repos/unitree_h1_control_ws/src/install/completed_scripts_control'
