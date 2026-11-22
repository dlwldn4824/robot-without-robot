"""실습 — 저장된 지도 위에서 내 위치 찾기 (Monte Carlo Localization).

PythonRobotics 의 Localization/particle_filter 예제를 그대로 호출한다.
바꾸는 것은 파티클 수와 시작 조건뿐이고, 필터 자체는 손대지 않는다.

  A. 파티클 수 20 / 100 / 500 / 2000 비교
  B. 납치된 로봇 — 주행 중에 실제 위치를 순간이동시킨다
  C. 시작 위치를 모를 때 — 파티클을 넓게 뿌리고 모이는지 본다
"""
import time, math
import numpy as np
from common import *
from Localization.particle_filter import particle_filter as pf

pf.show_animation = False
RFID = np.array([[10., 0.], [10., 10.], [0., 15.], [-5., 20.]])
STEPS = int(pf.SIM_TIME / pf.DT)


def set_np(n):
    pf.NP = n
    pf.NTh = n / 2.0


def run(n_particle, seed, kidnap_at=None, kidnap_jump=(6.0, 6.0), spread=None,
        recover=False):
    """한 판 돌리고 기록을 돌려준다."""
    np.random.seed(seed)
    set_np(n_particle)

    x_true = np.zeros((4, 1))
    x_dr = np.zeros((4, 1))                       # 바퀴 정보만 쓴 추정
    px = np.zeros((4, n_particle))
    if spread is not None:                        # 시작 위치를 모를 때
        px[0, :] = np.random.uniform(-spread, spread, n_particle)
        px[1, :] = np.random.uniform(-spread, spread, n_particle)
        px[2, :] = np.random.uniform(-math.pi, math.pi, n_particle)
    pw = np.zeros((1, n_particle)) + 1.0 / n_particle

    hist = {'t': [], 'pf': [], 'dr': [], 'true': [], 'est': [], 'spread': [], 'neff': []}
    collapsed_at = None
    t = 0.0
    compute = 0.0

    for k in range(STEPS):
        t += pf.DT
        u = pf.calc_input()
        x_true, z, x_dr, ud = pf.observation(x_true, x_dr, u, RFID)

        if kidnap_at is not None and abs(t - kidnap_at) < pf.DT / 2:
            x_true[0, 0] += kidnap_jump[0]        # 로봇을 들어서 옮긴다
            x_true[1, 0] += kidnap_jump[1]

        t0 = time.perf_counter()
        x_est, p_est, px, pw = pf.pf_localization(px, pw, z, ud)
        compute += time.perf_counter() - t0

        if not np.all(np.isfinite(pw)):
            if collapsed_at is None:
                collapsed_at = t                  # 가중치가 전부 0 이 되어 터진 시점
            if recover:                           # 개선안: 가중치를 균등하게 되돌린다
                pw = np.zeros((1, n_particle)) + 1.0 / n_particle

        e_pf = math.hypot(x_est[0, 0] - x_true[0, 0], x_est[1, 0] - x_true[1, 0])
        e_dr = math.hypot(x_dr[0, 0] - x_true[0, 0], x_dr[1, 0] - x_true[1, 0])
        hist['t'].append(t); hist['pf'].append(e_pf); hist['dr'].append(e_dr)
        hist['true'].append([x_true[0, 0], x_true[1, 0]])
        hist['est'].append([x_est[0, 0], x_est[1, 0]])
        hist['spread'].append(float(np.std(px[0, :]) + np.std(px[1, :])) / 2)
        hist['neff'].append(float(1.0 / (pw @ pw.T)[0, 0]) if np.all(np.isfinite(pw)) else 0.0)

    for k in ('t', 'pf', 'dr', 'spread', 'neff'):
        hist[k] = np.array(hist[k])
    hist['true'] = np.array(hist['true']); hist['est'] = np.array(hist['est'])
    hist['rmse'] = float(np.sqrt(np.mean(hist['pf'] ** 2)))
    hist['final'] = float(hist['pf'][-1])
    hist['max'] = float(hist['pf'].max())
    hist['dr_rmse'] = float(np.sqrt(np.mean(hist['dr'] ** 2)))
    hist['ms'] = compute / STEPS * 1000
    hist['collapsed_at'] = collapsed_at
    hist['n'] = n_particle
    return hist


if __name__ == '__main__':
    out = {}

    # ── A. 파티클 수 ─────────────────────────────────
    print(f"{'파티클':>6} {'RMSE':>8} {'최종오차':>9} {'최대오차':>9} "
          f"{'한 스텝':>9} {'한 판':>8}   (시드 10개 중앙값)")
    SEEDS = list(range(10))
    sweep = {}
    for n in (20, 100, 500, 2000):
        rs = [run(n, s) for s in SEEDS]
        med = lambda k: float(np.median([r[k] for r in rs]))
        sweep[n] = dict(rmse=med('rmse'), final=med('final'), mx=med('max'),
                        ms=med('ms'), dr=med('dr_rmse'),
                        rmse_all=[r['rmse'] for r in rs])
        print(f"{n:>6} {med('rmse'):>7.3f}m {med('final'):>8.3f}m {med('max'):>8.3f}m "
              f"{med('ms'):>7.2f}ms {med('ms')*STEPS/1000:>6.1f}s")
    print(f"\n바퀴 정보만 썼을 때 RMSE: {sweep[100]['dr']:.2f} m (파티클 수와 무관)")
    out['sweep'] = sweep
    out['base'] = run(100, 0)

    # ── B. 납치된 로봇 ───────────────────────────────
    print('\n[B] 납치된 로봇 — 25초에 (+6, +6) m 순간이동')
    kid = run(100, 0, kidnap_at=25.0)
    after = kid['pf'][kid['t'] > 25.0]
    print(f"  납치 직전 오차 {kid['pf'][kid['t'] <= 25.0][-1]:.2f} m "
          f"→ 직후 {after[0]:.2f} m → 25초 뒤 {after[-1]:.2f} m")
    print(f"  가중치가 전부 0 이 된 시점: {kid['collapsed_at']}")
    print(f"  납치 후 파티클 흩어짐 평균: {kid['spread'][kid['t'] > 25.0].mean():.2f} m")
    out['kidnap'] = kid

    # ── C. 시작 위치를 모를 때 ────────────────────────
    print('\n[C] 시작 위치를 모를 때 — 파티클을 ±20 m 에 흩뿌리고 시작')
    for n in (100, 2000):
        g = run(n, 0, spread=20.0)
        conv = np.where(g['pf'] < 2.0)[0]
        print(f"  NP={n:>4}: 시작오차 {g['pf'][0]:>5.1f} m · "
              f"2 m 안으로 들어온 시각 "
              f"{(g['t'][conv[0]] if len(conv) else float('nan')):>5.1f} s · "
              f"최종 {g['final']:.2f} m")
        out[f'global{n}'] = g

    np.savez(os.path.join(RUNS, 'pf.npz'), **{k: np.array(v, dtype=object) for k, v in out.items()}, allow_pickle=True)
    print('\n저장: runs/pf.npz')
