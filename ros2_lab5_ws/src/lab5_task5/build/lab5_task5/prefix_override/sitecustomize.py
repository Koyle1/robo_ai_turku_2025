import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bot/lab/robo_ai_turku_2025/ros2_lab5_ws/src/lab5_task5/install/lab5_task5'
