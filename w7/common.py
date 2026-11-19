"""w7 공통 설정. PythonRobotics 예제를 불러다 쓰기만 하고 고치지 않는다."""
import os, sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

PR = os.path.expanduser('~/Desktop/robotics-study/PythonRobotics')
HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, 'runs')
sys.path.insert(0, PR)

# 한글 폰트
for _p in ('/System/Library/Fonts/AppleSDGothicNeo.ttc',
           '/System/Library/Fonts/Supplemental/AppleGothic.ttf'):
    if os.path.exists(_p):
        font_manager.fontManager.addfont(_p) if _p.endswith('.ttf') else None
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=_p).get_name()
        break
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['figure.facecolor'] = 'white'
plt.rcParams['savefig.facecolor'] = 'white'

# W6 스터디로그와 같은 색
BLUE, RED, GREEN, GRAY = '#2169eb', '#d62d1f', '#0a8a55', '#6b7280'
