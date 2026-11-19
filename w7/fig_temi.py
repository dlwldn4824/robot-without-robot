"""실습 C 그림 — 돌아다니며 쌓은 지도, 그리고 쌓는 위치가 틀어졌을 때."""
import numpy as np
from matplotlib.colors import ListedColormap, BoundaryNorm
from common import *

CMAP = ListedColormap(['#ffffff', '#e4e4e1', '#243244'])   # 빈 곳 / 모름 / 장애물
NORM = BoundaryNorm([-0.1, 0.25, 0.75, 1.1], CMAP.N)

d = np.load(os.path.join(RUNS, 'temi_maps.npz'))
W, H, RES = float(d['W']), float(d['H']), float(d['res'])
poses, seg = d['poses'], d['seg']


def tri(g):
    """로그오즈를 빈 곳 0 / 모름 0.5 / 장애물 1 로 바꾼다."""
    out = np.full(g.shape, 0.5)
    out[g < -0.5] = 0.0
    out[g > 0.5] = 1.0
    return out


def show(ax, g, title, sub=''):
    ax.imshow(tri(g).T, origin='lower', extent=[0, W, 0, H],
              cmap=CMAP, norm=NORM, interpolation='nearest')
    ax.set_title(title, fontsize=11, pad=6)
    if sub:
        ax.text(0.5, -0.04, sub, transform=ax.transAxes, ha='center',
                va='top', fontsize=9, color=GRAY)
    ax.set_aspect('equal'); ax.set_xticks([]); ax.set_yticks([])
    for s in ax.spines.values(): s.set_color('#d8d8d5')


def cnt(g):
    occ = int((g > 0.5).sum()); free = int((g < -0.5).sum())
    return occ, free, g.size - occ - free, g.size


def pct(g):
    o, f, u, t = cnt(g)
    return 100 * (o + f) / t


def plan(ax):
    for x1, y1, x2, y2 in seg:
        ax.plot([x1, x2], [y1, y2], color='#9aa0a6', lw=1.0, zorder=1)


# ── FIG 06 · 멈춰 선 지점이 늘어날 때 ───────────────────────
fig, ax = plt.subplots(1, 4, figsize=(16.5, 3.6))
show(ax[0], d['one'], '멈춰 선 지점 1곳', f'채운 비율 {pct(d["one"]):.1f}%')
ax[0].plot(poses[0, 0], poses[0, 1], 'o', color=RED, ms=6, zorder=5)
show(ax[1], d['corridor'], '복도만 지나간 뒤 · 12곳', f'채운 비율 {pct(d["corridor"]):.1f}%')
ax[1].plot(poses[:12, 0], poses[:12, 1], '-', color=RED, lw=1.6, zorder=5)
show(ax[2], d['full'], '방까지 다 들른 뒤 · 89곳', f'44 m 주행 · 채운 비율 {pct(d["full"]):.1f}%')
ax[2].plot(poses[:, 0], poses[:, 1], '-', color=RED, lw=1.2, zorder=5)

cov = d['cov']
ax[3].plot(np.arange(1, len(cov) + 1), cov, color=BLUE, lw=2)
_c12 = pct(d['corridor'])
ax[3].axhline(_c12, color=GRAY, ls=':', lw=1)
ax[3].annotate('복도만 지나간 시점', (12, _c12), textcoords='offset points',
               xytext=(8, -16), fontsize=9, color=GRAY)
ax[3].set_xlabel('멈춰 선 지점 수', fontsize=10)
ax[3].set_ylabel('지도에서 아는 칸의 비율 (%)', fontsize=10)
ax[3].set_title('방에 들어갈 때마다 계단처럼 오른다', fontsize=11, pad=6)
ax[3].set_ylim(0, 105); ax[3].grid(alpha=.25, lw=.6)
for s in ('top', 'right'): ax[3].spines[s].set_visible(False)
fig.tight_layout()
fig.savefig(os.path.join(RUNS, 'fig_temi_coverage.png'), dpi=150, bbox_inches='tight')
print('fig_temi_coverage.png')

# ── FIG 07 · 쌓는 위치가 틀어지면 ───────────────────────────
fig, ax = plt.subplots(1, 3, figsize=(13.2, 3.9))
_o, _f, _u, _ = cnt(d['full'])
show(ax[0], d['full'], '참 위치로 쌓은 지도', f'장애물 {_o:,}칸 · 모름 {_u:,}칸')
ax[0].plot(poses[:, 0], poses[:, 1], '-', color=GREEN, lw=1.2, zorder=5)
_o, _f, _u, _ = cnt(d['drift_mild'])
show(ax[1], d['drift_mild'], f'바퀴 추정 위치로 쌓음 · 최종 오차 {d["err_mild"][-1]:.2f} m',
     f'장애물 {_o:,}칸 · 모름 {_u:,}칸')
ax[1].plot(poses[:, 0], poses[:, 1], '-', color=GREEN, lw=1.0, alpha=.6, zorder=4)
ax[1].plot(d['od_mild'][:, 0], d['od_mild'][:, 1], '-', color=RED, lw=1.2, zorder=5)
_o, _f, _u, _ = cnt(d['drift_ekf'])
show(ax[2], d['drift_ekf'], f'오차가 더 쌓인 경우 · 최종 {d["err_ekf"][-1]:.2f} m',
     f'장애물 {_o:,}칸 · 모름 {_u:,}칸')
ax[2].plot(d['od_ekf'][:, 0], d['od_ekf'][:, 1], '-', color=RED, lw=1.2, zorder=5)
ax[2].plot(poses[:, 0], poses[:, 1], '-', color=GREEN, lw=1.0, alpha=.6, zorder=4)
fig.tight_layout()
fig.savefig(os.path.join(RUNS, 'fig_temi_drift.png'), dpi=150, bbox_inches='tight')
print('fig_temi_drift.png')
