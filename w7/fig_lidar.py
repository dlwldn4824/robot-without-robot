"""실습 A 그림 — 스캔 원본, 격자 지도, 해상도 5단계 비교."""
import numpy as np
from matplotlib.colors import ListedColormap, BoundaryNorm
from common import *
from lidar_map import ang, dist, ox, oy, build, stats

CMAP = ListedColormap(['#ffffff', '#e4e4e1', '#243244'])   # 빈 곳 / 모름 / 장애물
NORM = BoundaryNorm([-0.1, 0.25, 0.75, 1.1], CMAP.N)


def show(axm, m, ext, title, sub=''):
    mnx, mxx, mny, mxy = ext
    axm.imshow(m.T, origin='lower', extent=[mnx, mxx, mny, mxy],
               cmap=CMAP, norm=NORM, interpolation='nearest')
    axm.plot(0, 0, 'o', color=RED, ms=5, zorder=5)
    axm.set_title(title, fontsize=11, pad=6)
    if sub:
        axm.text(0.5, -0.05, sub, transform=axm.transAxes, ha='center',
                 va='top', fontsize=9, color=GRAY)
    axm.set_xlim(-1.5, 1.5); axm.set_ylim(-1.3, 1.3)
    axm.set_xticks([]); axm.set_yticks([])
    for s in axm.spines.values(): s.set_color('#d8d8d5')


# ── FIG 01 · 스캔 원본 → 격자 지도 ──────────────────────────
m2, ext2 = build(0.02)
fig, ax = plt.subplots(1, 2, figsize=(9.2, 4.3))
for x, y in zip(ox, oy):          # 지도와 같은 축(x=ox, y=oy)으로 그린다
    ax[0].plot([0, x], [0, y], color='#f0b7b2', lw=0.6, zorder=1)
ax[0].plot(ox, oy, '.', color=RED, ms=4, zorder=3)
ax[0].plot(0, 0, 'o', color=BLUE, ms=7, zorder=4)
ax[0].set_title('LiDAR가 보낸 것 — 빔 154개의 각도와 거리', fontsize=11, pad=6)
ax[0].text(0.5, -0.05, '점 하나가 빔 하나가 닿은 자리입니다', transform=ax[0].transAxes,
           ha='center', va='top', fontsize=9, color=GRAY)
ax[0].set_aspect('equal'); ax[0].set_xlim(-1.5, 1.5); ax[0].set_ylim(-1.3, 1.3)
ax[0].set_xticks([]); ax[0].set_yticks([])
for s in ax[0].spines.values(): s.set_color('#d8d8d5')
show(ax[1], m2, ext2, '격자 지도 — 한 칸 2 cm', '흰색 빈 곳 · 회색 모름 · 남색 장애물')
ax[1].set_aspect('equal')
fig.tight_layout()
fig.savefig(os.path.join(RUNS, 'fig_scan_to_map.png'), dpi=150, bbox_inches='tight')
print('fig_scan_to_map.png')

# ── FIG 02 · 해상도 5단계 ──────────────────────────────────
RES = [0.01, 0.02, 0.05, 0.10, 0.25]
fig, ax = plt.subplots(1, 5, figsize=(15.2, 3.5))
for a, res in zip(ax, RES):
    m, ext = build(res); s = stats(m, res)
    show(a, m, ext, f'한 칸 {s["cm"]:.0f} cm',
         f'{s["shape"][0]}×{s["shape"][1]}칸 · 장애물 {s["occ"]} · 빈 곳 {s["free"]}')
    a.set_aspect('equal')
fig.tight_layout()
fig.savefig(os.path.join(RUNS, 'fig_resolution.png'), dpi=150, bbox_inches='tight')
print('fig_resolution.png')

# ── FIG 03 · 빈 곳 대 장애물 비율 ───────────────────────────
fig, ax = plt.subplots(figsize=(7.2, 3.6))
xs = [f'{int(r*100)} cm' for r in RES]
ratio = []
for res in RES:
    s = stats(*(lambda t: (t[0], res))(build(res)))
    ratio.append(s['free'] / s['occ'])
bars = ax.bar(xs, ratio, color=[BLUE]*3 + [RED]*2, width=0.56)
ax.axhline(1.0, color=GRAY, ls='--', lw=1)
ax.text(4.42, 1.12, '1.0 — 장애물과 빈 곳이 같아지는 선', ha='right', fontsize=9, color=GRAY)
for b, v in zip(bars, ratio):
    ax.text(b.get_x()+b.get_width()/2, v+0.35, f'{v:.1f}', ha='center', fontsize=10)
ax.set_ylabel('빈 곳 칸 ÷ 장애물 칸'); ax.set_ylim(0, 15)
ax.set_title('격자가 커질수록 벽이 방을 잡아먹는다', fontsize=11, pad=8)
for s_ in ('top', 'right'): ax.spines[s_].set_visible(False)
fig.tight_layout()
fig.savefig(os.path.join(RUNS, 'fig_ratio.png'), dpi=150, bbox_inches='tight')
print('fig_ratio.png  비율:', [round(r, 2) for r in ratio])
