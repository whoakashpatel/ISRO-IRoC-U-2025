import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/astra/IRoC_ws/odom_repub/install/odom_republisher'
