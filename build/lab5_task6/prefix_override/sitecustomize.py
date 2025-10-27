import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bot/Algo2025/robo_ai_turku_2025/install/lab5_task6'
