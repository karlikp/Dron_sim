import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/karol/ws/src/dron_sim/sim_bringup/install/sim_bringup'
