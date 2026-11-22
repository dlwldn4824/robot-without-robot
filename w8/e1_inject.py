"""E1 — 매 스텝 파티클 일부를 지도에 무작위로 다시 뿌린다 (AMCL 방식).

납치 복구가 빨라지는 대신 평소 정확도를 얼마나 내주는지, 그 교환비를 잰다.
필터는 그대로 두고 재표집 뒤에 주입만 끼워 넣는다.
"""
import math, time
import numpy as np
from common import *
import pf_run
from pf_run import STEPS, RFID
pf = pf_run.pf

MAP = (-12.0, 14.0, -3.0, 23.0)      # 주행 원과 랜드마크를 모두 덮는 지도 범위


def run_inject(n, seed, ratio, kidnap_at=None, on_collapse=0.0):
    """ratio      : 매 스텝 무작위로 다시 뿌릴 비율
    on_collapse: 가중치가 전부 0 이 된 스텝에만 뿌릴 비율 (AMCL 쪽에 가깝다)"""
    np.random.seed(seed); pf_run.set_np(n)
    x_true = np.zeros((4, 1)); x_dr = np.zeros((4, 1))
    px = np.zeros((4, n)); pw = np.zeros((1, n)) + 1.0 / n
    n_inj = int(round(n * ratio))
    T, E = [], []
    t = 0.0
    for k in range(STEPS):
        t += pf.DT
        u = pf.calc_input()
        x_true, z, x_dr, ud = pf.observation(x_true, x_dr, u, RFID)
        if kidnap_at is not None and abs(t - kidnap_at) < pf.DT / 2:
            x_true[0, 0] += 6.0; x_true[1, 0] += 6.0

        xe, P, px, pw = pf.pf_localization(px, pw, z, ud)
        if not np.all(np.isfinite(pw)):
            pw = np.zeros((1, n)) + 1.0 / n          # 5절에서 넣은 안전장치
            if on_collapse:                          # 이상할 때만 넓게 다시 뿌린다
                m = int(round(n * on_collapse))
                idx = np.random.choice(n, m, replace=False)
                px[0, idx] = np.random.uniform(MAP[0], MAP[1], m)
                px[1, idx] = np.random.uniform(MAP[2], MAP[3], m)
                px[2, idx] = np.random.uniform(-math.pi, math.pi, m)
            xe = px @ pw.T

        if n_inj:                                     # 지도 전체에 무작위로 다시 뿌린다
            idx = np.random.choice(n, n_inj, replace=False)
            px[0, idx] = np.random.uniform(MAP[0], MAP[1], n_inj)
            px[1, idx] = np.random.uniform(MAP[2], MAP[3], n_inj)
            px[2, idx] = np.random.uniform(-math.pi, math.pi, n_inj)
            pw[0, idx] = pw.mean()
            pw /= pw.sum()

        T.append(t)
        E.append(math.hypot(xe[0, 0] - x_true[0, 0], xe[1, 0] - x_true[1, 0]))
    return np.array(T), np.array(E)


def recovery_time(T, E, kidnap_at=25.0, thr=1.0):
    after = (T > kidnap_at + 0.2) & (E < thr)
    return (T[after][0] - kidnap_at) if after.any() else float('nan')


if __name__ == '__main__':
    RATIOS = (0.0, 0.01, 0.05, 0.10)
    SEEDS = range(10)
    print(f"{'주입':>5} {'평상시 RMSE':>12} {'납치 복구':>11} {'복구 성공':>10}")
    out = {}
    for r in RATIOS:
        norm = [run_inject(100, s, r)[1] for s in SEEDS]
        rmse = float(np.median([np.sqrt((e ** 2).mean()) for e in norm]))
        rec = []
        for s in SEEDS:
            T, E = run_inject(100, s, r, kidnap_at=25.0)
            rec.append(recovery_time(T, E))
        ok = [x for x in rec if np.isfinite(x)]
        out[r] = dict(rmse=rmse, rec=float(np.median(ok)) if ok else float('nan'),
                      ok=len(ok), total=len(rec))
        print(f"{r*100:>4.0f}% {rmse:>11.3f}m "
              f"{(np.median(ok) if ok else float('nan')):>10.1f}s {len(ok):>7}/{len(rec)}")
    np.savez(os.path.join(RUNS, 'e1.npz'),
             ratios=np.array(RATIOS),
             rmse=np.array([out[r]['rmse'] for r in RATIOS]),
             rec=np.array([out[r]['rec'] for r in RATIOS]),
             ok=np.array([out[r]['ok'] for r in RATIOS]))
    print('\n저장: runs/e1.npz')
