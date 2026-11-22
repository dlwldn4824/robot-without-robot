"""E2 — 지도가 답을 하나로 정해 주지 못할 때.

랜드마크를 전부 한 직선 위에 올리면, 그 직선을 기준으로 거울처럼 뒤집은 위치가
모든 랜드마크까지 '똑같은 거리'를 갖는다. 거리만 재는 센서로는 두 곳을 구분할 수 없다.

  (a) 직진 주행 — 대칭이 깨지지 않아 모호함이 끝까지 남는다
  (b) 원 주행   — 회전이 대칭을 깨서 언젠가 풀린다. 얼마나 걸리는지 잰다
"""
import math
import numpy as np
from common import *
import pf_run
from pf_run import STEPS
pf = pf_run.pf

LINE_Y = 0.0
MARKS = np.array([[0., LINE_Y], [10., LINE_Y], [20., LINE_Y],
                  [30., LINE_Y], [40., LINE_Y]])
START_Y = 3.0                      # 거울상은 y = -3 이 된다


def two_means(x, y, iters=25):
    """파티클이 몇 덩어리인지 보려고 2-means 를 돌린다."""
    pts = np.stack([x, y], 1)
    c = np.array([[x.mean(), y.max()], [x.mean(), y.min()]], dtype=float)
    lab = np.zeros(len(pts), dtype=int)
    for _ in range(iters):
        d = ((pts[:, None, :] - c[None, :, :]) ** 2).sum(2)
        lab = d.argmin(1)
        for k in (0, 1):
            if (lab == k).any():
                c[k] = pts[lab == k].mean(0)
    share = [float((lab == k).mean()) for k in (0, 1)]
    return c, share, lab


def run(n, seed, straight, spread=12.0):
    np.random.seed(seed); pf_run.set_np(n)
    old_input = pf.calc_input
    if straight:
        pf.calc_input = lambda: np.array([[1.0, 0.0]]).T     # 직진만
    try:
        x_true = np.zeros((4, 1)); x_true[1, 0] = START_Y
        x_dr = x_true.copy()
        px = np.zeros((4, n))
        px[0, :] = np.random.uniform(0.0, 40.0, n)
        px[1, :] = np.random.uniform(-spread, spread, n)
        px[2, :] = np.random.uniform(-math.pi, math.pi, n)
        pw = np.zeros((1, n)) + 1.0 / n

        snaps, T, E, GAP = [], [], [], []
        t = 0.0
        for k in range(STEPS):
            t += pf.DT
            u = pf.calc_input()
            x_true, z, x_dr, ud = pf.observation(x_true, x_dr, u, MARKS)
            xe, P, px, pw = pf.pf_localization(px, pw, z, ud)
            if not np.all(np.isfinite(pw)):
                pw = np.zeros((1, n)) + 1.0 / n
                xe = px @ pw.T
            below = float(pw[0][px[1] < 0].sum())            # 거울선 아래에 남은 질량
            T.append(t)
            E.append(math.hypot(xe[0, 0] - x_true[0, 0], xe[1, 0] - x_true[1, 0]))
            GAP.append(below)
            if k % 5 == 0:
                snaps.append((t, px[:2].copy(), (x_true[0, 0], x_true[1, 0]),
                              (xe[0, 0], xe[1, 0]), None, below))
        return dict(t=np.array(T), e=np.array(E), gap=np.array(GAP), snaps=snaps,
                    below=np.array([s[5] for s in snaps]))
    finally:
        pf.calc_import = None
        pf.calc_input = old_input


if __name__ == '__main__':
    print('랜드마크 5개를 y = 0 직선 위에 올렸습니다. 실제 로봇은 y = +3 에 있습니다.')
    print('거울상은 y = -3 이고, 다섯 랜드마크까지의 거리가 참값과 완전히 같습니다.\n')
    for straight, name in ((True, '(a) 직진 주행'), (False, '(b) 원 주행')):
        print(name)
        for n in (500, 2000):
            rs = [run(n, s, straight) for s in range(5)]
            # 두 덩어리가 유지되는 동안의 비율과, 풀리는 시각
            gap = np.median([r['gap'] for r in rs], axis=0)
            e = np.median([r['e'] for r in rs], axis=0)
            solved = np.where(gap < 1.0)[0]
            tsolve = rs[0]['t'][solved[0]] if len(solved) else float('nan')
            print(f"   NP={n:>4}: 두 덩어리 간격 처음 {gap[0]:>5.1f} m · "
                  f"10초 뒤 {gap[99]:>5.1f} m · 50초 뒤 {gap[-1]:>5.1f} m")
            print(f"            추정 오차 10초 뒤 {e[99]:>5.2f} m · 50초 뒤 {e[-1]:>5.2f} m · "
                  f"풀린 시각 {tsolve if np.isfinite(tsolve) else '끝까지 안 풀림'}")
        print()
    np.savez(os.path.join(RUNS, 'e2.npz'),
             straight=np.array([run(2000, 0, True)], dtype=object),
             circle=np.array([run(2000, 0, False)], dtype=object), allow_pickle=True)
    print('저장: runs/e2.npz')
