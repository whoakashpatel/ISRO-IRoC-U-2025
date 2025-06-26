import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/astra/IRoC_ws/survey_and_nav/install/survey_and_nav'
