"""실습 B — EKF-SLAM. PythonRobotics 의 SLAM/EKFSLAM 예제를 그대로 호출한다.

세 경로를 같은 한 번의 주행에서 동시에 기록한다.
  xTrue  실제 위치        (시뮬레이터만 아는 값)
  xDR    Odometry 추정    (바퀴 명령만 적분한 것 = dead reckoning)
  xEst   SLAM 추정        (랜드마크 관측으로 보정한 것)
"""
import numpy as np
from common import *
import SLAM.EKFSLAM.ekf_slam as E

E.show_animation = False
RFID = np.array([[10.0, -2.0], [15.0, 10.0], [3.0, 15.0], [-5.0, 20.0]])


def run(sim_time=50.0, seed=1, rfid=RFID, max_range=20.0):
    np.random.seed(seed)
    E.MAX_RANGE = max_range
    xEst = np.zeros((E.STATE_SIZE, 1)); xTrue = np.zeros((E.STATE_SIZE, 1))
    xDR = np.zeros((E.STATE_SIZE, 1));  PEst = np.eye(E.STATE_SIZE)
    hT, hD, hE, hz, hP = [xTrue.copy()], [xDR.copy()], [xEst.copy()], [0], [xEst.copy()]
    t = 0.0
    import io, sys
    while sim_time >= t:
        t += E.DT
        u = E.calc_input()
        xTrue, z, xDR, ud = E.observation(xTrue, xDR, u, rfid)
        _b, sys.stdout = sys.stdout, io.StringIO()     # "New LM" 로그 숨김
        try:    xEst, PEst = E.ekf_slam(xEst, PEst, ud, z)
        finally: sys.stdout = _b
        hT.append(xTrue.copy()); hD.append(xDR.copy())
        hE.append(xEst[0:3].copy()); hz.append(len(z)); hP.append(xEst.copy())
    T = np.hstack(hT); D = np.hstack(hD); X = np.hstack(hE)
    err_dr = np.hypot(T[0] - D[0], T[1] - D[1])
    err_sl = np.hypot(T[0] - X[0], T[1] - X[1])
    return dict(T=T, D=D, X=X, nz=np.array(hz), last=xEst, P=PEst, hist=hP,
                err_dr=err_dr, err_sl=err_sl, rfid=rfid,
                t=np.arange(len(err_dr)) * E.DT)


def report(name, r):
    print(f"[{name}]")
    print(f"  최종 오차   Odometry {r['err_dr'][-1]:6.2f} m   SLAM {r['err_sl'][-1]:6.2f} m")
    print(f"  평균 오차   Odometry {r['err_dr'].mean():6.2f} m   SLAM {r['err_sl'].mean():6.2f} m")
    print(f"  최대 오차   Odometry {r['err_dr'].max():6.2f} m   SLAM {r['err_sl'].max():6.2f} m")
    n_lm = int((len(r['last']) - 3) / 2)
    print(f"  찾은 랜드마크 {n_lm}개 / 실제 {len(r['rfid'])}개", end='')
    if n_lm:
        est = r['last'][3:].reshape(-1, 2)
        d = [min(np.hypot(*(e - f)) for f in r['rfid']) for e in est]
        print(f"   위치 오차 평균 {np.mean(d):.2f} m")
    else:
        print()
    print(f"  이동 거리 {np.sum(np.hypot(np.diff(r['T'][0]), np.diff(r['T'][1]))):.1f} m")


if __name__ == '__main__':
    base = run()
    report('랜드마크 4개 — 기본', base)
    none = run(rfid=np.array([[200.0, 200.0]]))      # 사거리 밖 → 관측 0
    report('랜드마크 없음 — 보정할 것이 없을 때', none)
    np.save(os.path.join(RUNS, 'ekf_base.npy'), base, allow_pickle=True)
    np.save(os.path.join(RUNS, 'ekf_none.npy'), none, allow_pickle=True)
