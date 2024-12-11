import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/michele/michele_try_ws/fishingrod_ws/install/rlg_quad_controller'
