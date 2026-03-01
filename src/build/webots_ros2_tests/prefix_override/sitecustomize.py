import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/MRTP/MRTP/src/install/webots_ros2_tests'
