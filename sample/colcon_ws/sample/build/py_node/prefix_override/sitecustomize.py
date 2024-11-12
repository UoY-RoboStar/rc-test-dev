import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/alex/git/rc-test-dev/sample/colcon_ws/sample/install/py_node'
