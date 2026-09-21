import sys
if sys.prefix == '/home/morfinm3/arm/Arm-Demonstrations/.pixi/envs/default':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/morfinm3/arm/Arm-Demonstrations/install/colman_bringup'
