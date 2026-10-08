import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/asus/nav_2027/rm_nav_v2/install/my_serial_py'
