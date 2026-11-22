"""E1·E2 그림. 저장된 npz 만 읽고 시뮬레이션을 다시 돌리지 않는다."""
import numpy as np
from common import *

# ── FIG E1 · 무작위 주입은 밀도가 받쳐 줄 때만 ────────────
d1 = np.load(os.path.join(RUNS, 'e1.npz'), allow_pickle=True)
dd = np.load(os.path.join(RUNS, 'e1_density.npz'))

fig, ax = plt.subplots(1, 2, figsize=(12.2, 4.2))

LAB = {'none': '주입 없음', 'e1': '매 스텝 1%', 'e5': '매 스텝 5%',
       'c25': '붕괴시 25%', 'c50': '붕괴시 50%', 'c100': '붕괴시 100%'}
keys = [str(k) for k in d1['labels']]
xs = np.arange(len(keys))
cols = [GRAY if k == 'none' else (BLUE if k.startswith('e') else ORANGE) for k in keys]
ax[0].bar(xs, d1['rec'], color=cols, width=.6)
for i, k in enumerate(keys):
    ax[0].annotate(f"{d1['rec'][i]:.1f}s", (xs[i], d1['rec'][i]),
                   textcoords='offset points', xytext=(0, 4), ha='center', fontsize=9.5)
ax[0].axhline(d1['rec'][0], color=GRAY, ls=':', lw=1.2)
ax[0].set_xticks(xs); ax[0].set_xticklabels([LAB[k] for k in keys], rotation=20, ha='right', fontsize=9.5)
ax[0].set_ylabel('납치 복구까지 (s)')
ax[0].set_title('파티클 100개에서는 무작위 주입이 오히려 해롭다', fontsize=11.5, pad=8)
ax[0].grid(alpha=.25, lw=.6, axis='y')
for s in ('top', 'right'): ax[0].spines[s].set_visible(False)

ns, area = dd['ns'], float(dd['area'])
xs2 = np.arange(len(ns))
w = .36
ax[1].bar(xs2 - w/2, dd['none'], w, color=GRAY, label='주입 없음')
ax[1].bar(xs2 + w/2, dd['inj'], w, color=GREEN, label='붕괴시 100% 주입')
for i in range(len(ns)):
    ax[1].annotate(f"{dd['none'][i]:.1f}", (xs2[i]-w/2, dd['none'][i]),
                   textcoords='offset points', xytext=(0, 4), ha='center', fontsize=9.5, color=GRAY)
    ax[1].annotate(f"{dd['inj'][i]:.1f}", (xs2[i]+w/2, dd['inj'][i]),
                   textcoords='offset points', xytext=(0, 4), ha='center', fontsize=9.5, color=GREEN)
ax[1].set_xticks(xs2)
ax[1].set_xticklabels([f'{n}개\n{n/area:.2f}개/m²' for n in ns], fontsize=9.5)
ax[1].set_ylabel('납치 복구까지 (s)')
ax[1].set_title('밀도가 올라가면 뒤집힌다', fontsize=11.5, pad=8)
ax[1].legend(fontsize=9.5); ax[1].grid(alpha=.25, lw=.6, axis='y')
for s in ('top', 'right'): ax[1].spines[s].set_visible(False)
fig.tight_layout()
fig.savefig(os.path.join(RUNS, 'fig_e1.png'), dpi=150, bbox_inches='tight')
print('fig_e1.png')

# ── FIG E2 · 거울처럼 똑같이 생긴 지도 ────────────────────
d2 = np.load(os.path.join(RUNS, 'e2.npz'), allow_pickle=True)
run = d2['wrong'][0]
snaps = run['snaps']
want = [0.1, 1.0, 5.0, 40.0]
fig, ax = plt.subplots(1, 4, figsize=(16.0, 3.3))
for a, tw in zip(ax, want):
    i = min(range(len(snaps)), key=lambda j: abs(snaps[j][0] - tw))
    t, p, tp, ep, _, below = snaps[i]
    a.axhline(0, color=ORANGE, lw=1.4, alpha=.6, zorder=1)
    a.plot([0, 10, 20, 30, 40], [0]*5, '*', color=ORANGE, ms=14, zorder=5)
    a.plot(p[0], p[1], '.', color=BLUE, ms=2.6, alpha=.45, zorder=3)
    a.plot(tp[0], -tp[1], 'o', mfc='none', mec=GRAY, mew=1.8, ms=10, zorder=6)
    a.plot(*tp, 'o', color=GREEN, ms=10, zorder=7)
    a.plot(*ep, 'X', color=RED, mec='white', mew=1.2, ms=13, zorder=8)
    a.set_xlim(-6, 48); a.set_ylim(-8.5, 8.5); a.set_aspect('equal')
    a.set_xticks([]); a.set_yticks([])
    a.set_title(f'{t:.1f} 초', fontsize=11.5, pad=6)
    a.text(.5, -.06, f'거울선 아래 질량 {below*100:.0f}%', transform=a.transAxes,
           ha='center', va='top', fontsize=9.5,
           color=(RED if below > .5 else GRAY))
    for sp in a.spines.values(): sp.set_color('#d8d8d5')
ax[0].text(-5, 6.6, '● 실제   ○ 거울상(거리가 같은 곳)   ✕ 필터 추정   ★ 랜드마크',
           fontsize=9, color=GRAY)
fig.tight_layout()
fig.savefig(os.path.join(RUNS, 'fig_e2.png'), dpi=150, bbox_inches='tight')
print('fig_e2.png')
