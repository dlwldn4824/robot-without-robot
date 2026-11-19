"""실습 C — 테미가 지도를 만들 때 왜 돌아다녀야 하고, 왜 SLAM이어야 하는가.

실습 A는 한 자리에서 찍은 스캔 한 장이었다. 여기서는 같은 격자 규칙을 그대로 두고
로봇을 움직이면서 여러 장을 쌓는다. 쌓을 때 쓰는 '내 위치'를 참값으로 주는 경우와
바퀴 정보만으로 추정한 값으로 주는 경우를 비교한다.

격자 한 칸 = 5 cm (ROS 내비게이션 기본값, 테미 공식 사양의 위치 정확도와 같은 자리)
"""
import numpy as np
from common import *

RES = 0.05                 # 한 칸 5 cm
MAXR = 10.0                # LiDAR 최대 사거리 10 m
NBEAM = 1080               # 한 바퀴 1080개 (0.33도 간격)
L_OCC, L_FREE = 0.85, -0.4 # 로그오즈 증분
L_CLAMP = 5.0

W, H = 16.0, 10.0          # 바닥 크기 (m)

def floorplan():
    """사무실 한 층. 가운데 복도, 위에 방 셋, 아래에 방 둘. 문은 끊긴 구간."""
    seg = []
    def h(y, x0, x1): seg.append((x0, y, x1, y))
    def v(x, y0, y1): seg.append((x, y0, x, y1))
    # 외벽
    h(0.0, 0, W); h(H, 0, W); v(0.0, 0, H); v(W, 0, H)
    # 복도 위쪽 벽 (문 세 개)
    for x0, x1 in [(0, 2.0), (3.0, 7.5), (8.5, 13.0), (14.0, W)]:
        h(6.0, x0, x1)
    # 복도 아래쪽 벽 (문 두 개)
    for x0, x1 in [(0, 3.0), (4.0, 11.0), (12.0, W)]:
        h(4.0, x0, x1)
    # 방 칸막이
    v(5.0, 6.0, H); v(10.5, 6.0, H); v(6.5, 0, 4.0)
    return np.array(seg, dtype=float)

SEG = floorplan()

def raycast(px, py, angles):
    """각 방향으로 벽까지의 거리. 벡터화된 선분-광선 교차."""
    x1, y1, x2, y2 = SEG[:, 0], SEG[:, 1], SEG[:, 2], SEG[:, 3]
    dx, dy = np.cos(angles)[:, None], np.sin(angles)[:, None]
    sx, sy = (x2 - x1)[None, :], (y2 - y1)[None, :]
    den = dx * sy - dy * sx
    ok = np.abs(den) > 1e-12
    qx, qy = (x1 - px)[None, :], (y1 - py)[None, :]
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (qx * sy - qy * sx) / den      # 광선 쪽 거리
        u = (qx * dy - qy * dx) / den      # 선분 쪽 위치 0~1
    hit = ok & (t > 1e-6) & (u >= 0) & (u <= 1)
    t = np.where(hit, t, np.inf)
    d = t.min(axis=1)
    return np.minimum(d, MAXR), d <= MAXR

def new_grid():
    return np.zeros((int(W / RES), int(H / RES)))

def integrate(grid, px, py, th, true_px, true_py, true_th):
    """참 위치에서 재고, 추정 위치를 기준으로 지도에 적는다."""
    ang = true_th + np.linspace(-np.pi, np.pi, NBEAM, endpoint=False)
    d, got = raycast(true_px, true_py, ang)
    # 적을 때는 추정 자세를 쓴다 (참 - 추정 만큼 틀어져서 기록됨)
    wa = th + np.linspace(-np.pi, np.pi, NBEAM, endpoint=False)
    ex, ey = px + d * np.cos(wa), py + d * np.sin(wa)
    i0, j0 = int(px / RES), int(py / RES)
    nx, ny = grid.shape
    for k in range(NBEAM):
        i1, j1 = int(ex[k] / RES), int(ey[k] / RES)
        for i, j in bresenham_line(i0, j0, i1, j1):
            if 0 <= i < nx and 0 <= j < ny:
                grid[i, j] = max(-L_CLAMP, grid[i, j] + L_FREE)
        if got[k] and 0 <= i1 < nx and 0 <= j1 < ny:
            grid[i1, j1] = min(L_CLAMP, grid[i1, j1] + L_OCC)
    return grid

def bresenham_line(x0, y0, x1, y1):
    pts = []
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx, sy = (1 if x1 > x0 else -1), (1 if y1 > y0 else -1)
    err = dx - dy
    while True:
        if (x0, y0) == (x1, y1):
            break                      # 끝점은 장애물로 따로 찍는다
        pts.append((x0, y0))
        e2 = 2 * err
        if e2 > -dy: err -= dy; x0 += sx
        if e2 < dx:  err += dx; y0 += sy
    return pts

def path():
    """복도를 한 번 지나가고, 방 다섯 개를 차례로 들렀다 나온다."""
    wp = [(1.0, 5.0), (2.5, 5.0), (2.5, 8.0), (2.5, 5.0),      # 왼쪽 위 방
          (3.5, 5.0), (3.5, 2.0), (3.5, 5.0),                   # 왼쪽 아래 방
          (8.0, 5.0), (8.0, 8.0), (8.0, 5.0),                   # 가운데 위 방
          (11.5, 5.0), (11.5, 2.0), (11.5, 5.0),                # 오른쪽 아래 방
          (13.5, 5.0), (13.5, 8.0), (13.5, 5.0), (15.0, 5.0)]   # 오른쪽 위 방
    pts = []
    for a, b in zip(wp[:-1], wp[1:]):
        n = max(2, int(np.hypot(b[0]-a[0], b[1]-a[1]) / 0.5))
        for s in np.linspace(0, 1, n, endpoint=False):
            pts.append((a[0] + s*(b[0]-a[0]), a[1] + s*(b[1]-a[1])))
    pts.append(wp[-1])
    out = []
    for k, (x, y) in enumerate(pts):
        nx_, ny_ = pts[min(k+1, len(pts)-1)]
        th = np.arctan2(ny_ - y, nx_ - x) if (nx_, ny_) != (x, y) else 0.0
        out.append((x, y, th))
    return out

def counts(grid):
    occ = int((grid > 0.5).sum()); free = int((grid < -0.5).sum())
    unk = grid.size - occ - free
    return occ, free, unk, grid.size

def drifted(poses, trans_pct=0.01, head_deg_per_m=0.6, seed=7):
    """바퀴 정보만으로 추정한 자세. 오차가 지워지지 않고 쌓인다."""
    rng = np.random.default_rng(seed)
    out = [poses[0]]
    ex, ey, eth = poses[0]
    for a, b in zip(poses[:-1], poses[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        step = np.hypot(dx, dy)
        dth = np.arctan2(np.sin(b[2]-a[2]), np.cos(b[2]-a[2]))
        # 이동량과 회전량에 각각 비례하는 오차가 더해진다
        step_e = step * (1 + rng.normal(0, trans_pct))
        eth = eth + dth + np.deg2rad(head_deg_per_m) * step * rng.normal(0, 1)
        ex += step_e * np.cos(eth); ey += step_e * np.sin(eth)
        out.append((ex, ey, eth))
    return out

if __name__ == '__main__':
    poses = path()
    total_m = sum(np.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(poses[:-1], poses[1:]))
    print(f"경로 {len(poses)}개 지점 · 총 {total_m:.1f} m · 격자 {RES*100:.0f} cm "
          f"{int(W/RES)}x{int(H/RES)}")

    # 1) 멈춰 선 지점 수를 늘려 가며 지도가 얼마나 채워지는가
    stages, cov = {}, []
    g = new_grid()
    for n, (x, y, th) in enumerate(poses, 1):
        integrate(g, x, y, th, x, y, th)
        occ, free, unk, tot = counts(g)
        cov.append(100 * (occ + free) / tot)
        if n in (1, 12, len(poses)):
            stages[n] = g.copy()
    for n in sorted(stages):
        occ, free, unk, tot = counts(stages[n])
        print(f"  지점 {n:>3}개 : 장애물 {occ:>6} 빈 곳 {free:>6} 모름 {unk:>6} "
              f"→ 채운 비율 {100*(occ+free)/tot:>5.1f}%")

    # 2) 참 위치로 쌓은 지도 vs 바퀴 정보로 쌓은 지도 (드리프트 두 수준)
    out = {}
    for tag, hd in (('mild', 0.6), ('ekf', 7.0)):
        od = drifted(poses, trans_pct=0.03, head_deg_per_m=hd)
        gd = new_grid()
        for (x, y, th), (tx, ty, tth) in zip(od, poses):
            integrate(gd, x, y, th, tx, ty, tth)
        err = [np.hypot(a[0]-b[0], a[1]-b[1]) for a, b in zip(od, poses)]
        o, f, u, t = counts(gd)
        out[tag] = dict(grid=gd, od=np.array(od), err=np.array(err),
                        hd=hd, occ=o, free=f, unk=u)
        print(f"  [{tag}] 회전 오차 {hd} deg/m → 최종 {err[-1]:.2f} m "
              f"평균 {np.mean(err):.2f} m 최대 {max(err):.2f} m")
        print(f"         장애물 {o:>6} 빈 곳 {f:>6} 모름 {u:>6}")

    o1, f1, u1, t1 = counts(stages[len(poses)])
    print(f"  [참값] 장애물 {o1:>6} 빈 곳 {f1:>6} 모름 {u1:>6}")
    for tag in ('mild', 'ekf'):
        d = out[tag]
        print(f"  [{tag}] 장애물 {100*(d['occ']-o1)/o1:+5.0f}% · "
              f"빈 곳 {100*(d['free']-f1)/f1:+5.0f}%")

    np.savez(os.path.join(RUNS, 'temi_maps.npz'),
             one=stages[1], corridor=stages[12], full=stages[len(poses)],
             drift_mild=out['mild']['grid'], drift_ekf=out['ekf']['grid'],
             od_mild=out['mild']['od'], od_ekf=out['ekf']['od'],
             err_mild=out['mild']['err'], err_ekf=out['ekf']['err'],
             cov=np.array(cov),
             poses=np.array(poses), seg=SEG, res=RES, W=W, H=H)
    print("저장: runs/temi_maps.npz")
