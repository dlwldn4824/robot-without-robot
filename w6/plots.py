"""실험 결과 그림 3장."""
import json, os, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
OBS = [(1.4,0.9,0.25),(-1.3,1.7,0.22),(0.2,-1.6,0.28),(-1.9,-1.1,0.24),(2.3,-2.0,0.20),(-0.6,2.7,0.22)]
BOX = [(2.2,1.9,0.9,0.08),(-2.4,0.4,0.08,0.9)]

def draw_world(ax):
    ax.add_patch(Rectangle((-3.5,-3.5),7,7, fill=False, ec='#555', lw=2))
    for x,y,r in OBS: ax.add_patch(Circle((x,y), r, fc='#D9897F', ec='#8d4b42'))
    for x,y,w,h in BOX: ax.add_patch(Rectangle((x-w,y-h), 2*w, 2*h, fc='#777', ec='#444'))
    ax.set_xlim(-3.9,3.9); ax.set_ylim(-3.9,3.9); ax.set_aspect('equal'); ax.grid(alpha=.25)

def traj(path):
    d = json.load(open(path)); lg = d['log']
    return np.array([[e['x'],e['y']] for e in lg]), d['result']

# ---------- 그림 1: 궤적 비교
fig, axes = plt.subplots(1, 2, figsize=(12.4, 6.4))
for ax, (f, title, col) in zip(axes, [('runs/exp_0.json','BASIC  (front sensor only)','#C62828'),
                                      ('runs/exp_7.json','IMPROVED  (+side, hysteresis, hold)','#1B5E20')]):
    xy, res = traj(f)
    draw_world(ax)
    ax.plot(xy[:,0], xy[:,1], '-', color=col, lw=2.0)
    ax.plot(xy[0,0], xy[0,1], 'o', color='lime', ms=11, mec='k', label='start')
    ax.plot(xy[-1,0], xy[-1,1], 's', color='magenta', ms=9, mec='k', label='end')
    ax.set_title(f"{title}\ncollisions {res['collisions']}   travelled {res['path_len']} m   "
                 f"turn ratio {res['turn_ratio']}", fontsize=10.5)
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.legend(fontsize=8, loc='lower right')
fig.suptitle('40 s obstacle-avoidance run in MuJoCo', fontsize=13)
fig.tight_layout(rect=[0,0,1,0.94]); fig.savefig('fig_w6_traj.png', dpi=130); plt.close(fig)

# ---------- 그림 2: 실험 비교
ex = json.load(open('runs/experiments.json'))
EN = {'0':'0  basic  th=0.7 (baseline)', 'A':'A  th=0.1  too small',
      'A+':'A+ th=0.1 + speed x1.5',     'B':'B  th=3.0  too large',
      'C':'C  noise +-0.15 m',           'C+':'C+ noise +-0.25 m',
      '7':'7  IMPROVED controller',      '7a':'7a improved + noise 0.15',
      '7b':'7b improved + noise 0.25'}
labs = [EN[e['label'].split()[0]] for e in ex]
fig, axes = plt.subplots(1, 3, figsize=(16.5, 5.0))
for ax, key, ttl, col in [(axes[0],'collisions','collisions (lower better)','#C62828'),
                          (axes[1],'path_len','distance travelled [m] (higher better)','#1565C0'),
                          (axes[2],'turn_ratio','fraction of time turning','#F9A825')]:
    v = [e[key] for e in ex]
    ax.barh(range(len(ex)), v, color=col)
    ax.set_yticks(range(len(ex))); ax.set_yticklabels(labs, fontsize=8)
    ax.invert_yaxis(); ax.set_title(ttl, fontsize=10.5); ax.grid(alpha=.3, axis='x')
    for i, val in enumerate(v):
        ax.text(val, i, f' {val}', va='center', fontsize=8)
    ax.set_xlim(0, max(v)*1.25)
fig.suptitle('Threshold / noise / controller experiments  (40 s each)', fontsize=13)
fig.tight_layout(rect=[0,0,1,0.93]); fig.savefig('fig_w6_experiments.png', dpi=130); plt.close(fig)

# ---------- 그림 3: 채터링
ch = json.load(open('runs/chatter.json'))
fig, ax = plt.subplots(figsize=(8.0, 4.6))
noises = sorted({c['noise'] for c in ch})
for name, col, en in (('기본','#C62828','BASIC'), ('개선','#1B5E20','IMPROVED')):
    y = [next(c['per_sec'] for c in ch if c['controller']==name and c['noise']==n) for n in noises]
    ax.plot(noises, y, 'o-', color=col, lw=2.2, ms=8, label=en)
    for xi, yi in zip(noises, y):
        ax.annotate(f'{yi:.1f}', (xi, yi), textcoords='offset points', xytext=(0,8),
                    ha='center', fontsize=9, color=col)
ax.set_xlabel('sensor noise amplitude  ±[m]'); ax.set_ylabel('action changes per second')
ax.set_title('Chattering: true distance fixed at 0.72 m, threshold 0.70 m', fontsize=11.5)
ax.grid(alpha=.3); ax.legend(); fig.tight_layout()
fig.savefig('fig_w6_chatter.png', dpi=130)
print('saved fig_w6_traj.png, fig_w6_experiments.png, fig_w6_chatter.png')
