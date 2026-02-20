import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/calafaker/Control-Formation/ws/install/simulation_pkg'
