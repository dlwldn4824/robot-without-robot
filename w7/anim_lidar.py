"""REC 01 — LiDAR 빔이 하나씩 격자 지도가 되는 과정.

격자 범위만 전체 스캔으로 미리 고정하고, 칸을 채우는 규칙은
예제의 generate_ray_casting_grid_map 과 똑같다 (같은 bresenham 함수를 쓴다).
"""
import numpy as np
from PIL import Image
from matplotlib.colors import ListedColormap, BoundaryNorm
from common import *
from lidar_map import ang, dist, ox, oy
from Mapping.lidar_to_grid_map.lidar_to_grid_map import bresenham, calc_grid_map_config

RES, PER_FRAME, FPS = 0.02, 2, 10
CMAP = ListedColormap(['#ffffff', '#e4e4e1', '#243244'])
NORM = BoundaryNorm([-0.1, 0.25, 0.75, 1.1], CMAP.N)

import io, sys
_b, sys.stdout = sys.stdout, io.StringIO()
min_x, min_y, max_x, max_y, xw, yw = calc_grid_map_config(ox, oy, RES)
sys.stdout = _b
cx, cy = int(round(-min_x / RES)), int(round(-min_y / RES))
grid = np.ones((xw, yw)) / 2


def add_beam(i):
    """예제와 같은 규칙: 빔이 지나간 칸은 0, 닿은 칸과 그 옆 3칸은 1."""
    ix = int(round((ox[i] - min_x) / RES)); iy = int(round((oy[i] - min_y) / RES))
    for bx, by in bresenham((cx, cy), (ix, iy)):
        grid[bx][by] = 0.0
    for dx in (0, 1):
        for dy in (0, 1):
            grid[ix + dx][iy + dy] = 1.0


fig, ax = plt.subplots(1, 2, figsize=(7.6, 3.75), dpi=100)
fig.subplots_adjust(left=.02, right=.98, top=.80, bottom=.03, wspace=.06)
frames = []
for k in range(0, len(ox) + PER_FRAME, PER_FRAME):
    for i in range(k, min(k + PER_FRAME, len(ox))):
        add_beam(i)
    n = min(k + PER_FRAME, len(ox))
    for a in ax: a.clear()

    a0 = ax[0]
    for i in range(n):
        a0.plot([0, ox[i]], [0, oy[i]], color='#f0b7b2', lw=0.6, zorder=1)
    if n:
        a0.plot(ox[:n], oy[:n], '.', color=RED, ms=3.5, zorder=3)
        a0.plot([0, ox[n-1]], [0, oy[n-1]], color=RED, lw=1.8, zorder=4)
    a0.plot(0, 0, 'o', color=BLUE, ms=7, zorder=5)
    a0.set_title('LiDAR 빔', fontsize=10, pad=4)

    ax[1].imshow(grid.T, origin='lower', extent=[min_x, max_x, min_y, max_y],
                 cmap=CMAP, norm=NORM, interpolation='nearest')
    ax[1].plot(0, 0, 'o', color=RED, ms=5, zorder=5)
    ax[1].set_title('격자 지도 — 한 칸 2 cm', fontsize=10, pad=4)

    occ = int((grid > 0.9).sum()); free = int((grid < 0.1).sum())
    unk = grid.size - occ - free
    for a in ax:
        a.set_xlim(-1.45, 1.45); a.set_ylim(-1.15, 1.15)
        a.set_aspect('equal'); a.set_xticks([]); a.set_yticks([])
        for s in a.spines.values(): s.set_color('#d8d8d5')
    d = dist[n-1] if n else 0
    fig.suptitle(f'빔 {n:3d} / 154      이번 빔 거리 {d:4.2f} m      '
                 f'장애물 {occ:4d}칸 · 빈 곳 {free:4d}칸 · 모름 {unk:5d}칸',
                 fontsize=10.5, y=0.955, color='#243244')
    fig.canvas.draw()
    frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()))

frames += [frames[-1]] * 12                    # 마지막 지도를 잠깐 보여준다
w, h = frames[0].size
pal = frames[-1].quantize(colors=48, method=Image.MEDIANCUT)
frames = [f.quantize(palette=pal, dither=Image.NONE) for f in frames]
out = os.path.join(RUNS, 'rec01_lidar.gif')
frames[0].save(out, save_all=True, append_images=frames[1:],
               duration=int(1000/FPS), loop=0, optimize=True, disposal=2)
print(f'{out}  {len(frames)} frames  {w}x{h}')
print(f'최종: 장애물 {int((grid>0.9).sum())}칸 · 빈 곳 {int((grid<0.1).sum())}칸')
