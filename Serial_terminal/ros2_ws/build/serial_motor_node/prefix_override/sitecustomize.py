import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sudhansu/Desktop/Arduino/Serial_terminal/ros2_ws/install/serial_motor_node'
