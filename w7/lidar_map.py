"""실습 A — LiDAR 한 번 스캔한 값이 어떻게 격자 지도가 되는가.

PythonRobotics 의 Mapping/lidar_to_grid_map 예제를 그대로 호출한다.
바꾸는 것은 격자 한 칸의 크기(xy_resolution) 하나뿐이다.
"""
import io, sys
import numpy as np
from common import *
from Mapping.lidar_to_grid_map.lidar_to_grid_map import (
    file_read, generate_ray_casting_grid_map, bresenham)

CSV = os.path.join(PR, 'Mapping/lidar_to_grid_map/lidar01.csv')
ang, dist = file_read(CSV)
ox, oy = np.sin(ang) * dist, np.cos(ang) * dist


def build(res):
    buf, old = io.StringIO(), sys.stdout
    sys.stdout = buf                      # 예제가 찍는 크기 로그를 가린다
    try:
        m, mnx, mxx, mny, mxy, _ = generate_ray_casting_grid_map(ox, oy, res, True)
    finally:
        sys.stdout = old
    return m, (mnx, mxx, mny, mxy)


def stats(m, res):
    occ = int((m > 0.9).sum()); unk = int((np.abs(m - 0.5) < 1e-9).sum())
    free = int((m < 0.1).sum()); tot = m.size
    return dict(res=res, shape=m.shape, cells=tot, cm=res*100,
                occ=occ, free=free, unk=unk,
                occ_pct=100*occ/tot, unk_pct=100*unk/tot,
                area_cm2=(res*100)**2, kb=tot/1024)


if __name__ == '__main__':
    print(f"{'한 칸':>8} {'격자':>12} {'칸 수':>8} {'장애물':>7} {'빈 곳':>7} {'모름':>7} {'한 칸 넓이':>11} {'메모리':>8}")
    rows = []
    for res in (0.01, 0.02, 0.05, 0.10, 0.25):
        m, ext = build(res)
        s = stats(m, res); rows.append((s, m, ext))
        print(f"{s['cm']:>6.0f}cm {s['shape'][0]:>5}x{s['shape'][1]:<5} {s['cells']:>8} "
              f"{s['occ']:>7} {s['free']:>7} {s['unk']:>7} "
              f"{s['area_cm2']:>8.0f}cm² {s['kb']:>6.1f}KB")
    np.save(os.path.join(RUNS, 'lidar_maps.npy'),
            np.array([r[0] for r in rows], dtype=object), allow_pickle=True)
