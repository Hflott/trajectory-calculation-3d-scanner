import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/subseascanning/trajectory-calculation-3d-scanner/install/subsea_ui'
