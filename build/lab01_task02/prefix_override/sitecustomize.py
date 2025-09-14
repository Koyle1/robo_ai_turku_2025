import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/felix/robo/robo_ai_turku_2025/install/lab01_task02'
