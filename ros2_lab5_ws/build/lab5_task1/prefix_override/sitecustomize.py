import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bot/Algo2025/lab5/robo_ai_turku_2025/ros2_lab5_ws/install/lab5_task1'
