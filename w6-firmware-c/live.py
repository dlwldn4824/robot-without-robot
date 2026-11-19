"""live.py — C 펌웨어가 모는 차를 실시간 창으로 본다.

실행:   mjpython live.py            (macOS 는 반드시 mjpython)
        mjpython live.py --noise 120 --ppr 20     ← 버그 재현
창 조작: 드래그 회전 / 휠 확대 / 우클릭 드래그 이동 / Esc 종료
"""
import argparse, ctypes, math, os, random, time
import numpy as np, mujoco, mujoco.viewer
from host import (FwOut, load_fw, ir_adc, beacon, yaw_of, GOAL_XY,
                  PHYS_DT, TICK_MS, MAX_OMEGA, PWM_MAX, CAR, STATES)
import host

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--noise', type=int, default=0, help='ADC 노이즈 ±LSB')
    ap.add_argument('--ppr', type=int, default=360, help='엔코더 분해능 (20 으로 주면 스톨 버그 재현)')
    ap.add_argument('--speed', type=float, default=1.0, help='재생 배속')
    a = ap.parse_args()
    host.TICKS_PER_REV = a.ppr

    fw = load_fw(); fw.fw_setup()
    m = mujoco.MjModel.from_xml_path(os.path.join(HERE, 'arena.xml'))
    d = mujoco.MjData(m)
    rng = random.Random(0)

    def reset():
        mujoco.mj_resetData(m, d)
        d.qpos[0:2] = (-2.2, 2.2)
        mujoco.mj_forward(m, d)
        fw.fw_setup()

    reset()
    spt = int(round((TICK_MS/1000)/PHYS_DT))
    prev_enc = 0; col = 0; in_col = False; trail = []; runs = 0
    t_wall = time.time()

    print('창이 뜹니다.  Esc 로 종료.')
    print(f'  ADC 노이즈 ±{a.noise} LSB   엔코더 {a.ppr} PPR')
    print(f'{"t":>6} {"state":<9} {"ADC L/F/R":>16} {"mm L/F/R":>18} {"PWM L/R":>11}  충돌')
    with mujoco.viewer.launch_passive(m, d, show_left_ui=False, show_right_ui=False) as v:
        v.cam.azimuth, v.cam.elevation, v.cam.distance = 150, -40, 3.4
        last_print = -1.0
        while v.is_running():
            step_start = time.time()
            k = int(round(d.time / PHYS_DT))
            if k % spt == 0:
                adc = tuple(ir_adc(float(d.sensordata[i]), a.noise, rng) for i in range(3))
                x, y = float(d.qpos[0]), float(d.qpos[1]); yw = yaw_of(d.qpos[3:7])
                gd = math.hypot(GOAL_XY[0]-x, GOAL_XY[1]-y)
                ga = (math.degrees(math.atan2(GOAL_XY[1]-y, GOAL_XY[0]-x) - yw) + 180) % 360 - 180
                bea = beacon(ga, gd)
                e = abs(host.enc_ticks(float(d.qpos[7]))) + abs(host.enc_ticks(float(d.qpos[8])))
                moved = abs(e - prev_enc); prev_enc = e
                out = fw.fw_loop(int(d.time*1000), *adc, *bea, moved)   # ← C 펌웨어
                d.ctrl[0] = d.ctrl[2] = out.l_dir*(out.l_duty/PWM_MAX)*MAX_OMEGA
                d.ctrl[1] = d.ctrl[3] = out.r_dir*(out.r_duty/PWM_MAX)*MAX_OMEGA
                st = STATES[out.state]
                if d.time - last_print >= 0.5:
                    last_print = d.time
                    print(f'{d.time:6.2f} {st:<9} {str(list(adc)):>16} '
                          f'{str([fw.fw_get_mm(i) for i in range(3)]):>18} '
                          f'{out.l_dir*out.l_duty:>5}{out.r_dir*out.r_duty:>6}  {col}')
                if st == 'ARRIVED':
                    runs += 1
                    print(f'★ {d.time:.2f}초에 도착.  충돌 {col}회, 스톨 {fw.fw_get_stalls()}회'
                          f'  — 3초 뒤 다시 출발합니다 ({runs}회차)')
                    time.sleep(3.0)
                    reset(); prev_enc = 0; col = 0; trail.clear(); last_print = -1.0
                    continue

            mujoco.mj_step(m, d)

            hit = False
            for i in range(d.ncon):
                c = d.contact[i]
                nm = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom1),
                      mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom2)}
                if nm & CAR and not (nm & {'floor','GOAL'}) and (nm - CAR): hit = True; break
            if hit and not in_col: col += 1
            in_col = hit

            if k % 60 == 0:
                trail.append((float(d.qpos[0]), float(d.qpos[1]))); trail[:] = trail[-26:]
                v.user_scn.ngeom = 0
                for i,(tx,ty) in enumerate(trail):
                    if v.user_scn.ngeom >= v.user_scn.maxgeom: break
                    al = 0.12 + 0.55*(i/max(len(trail)-1,1))
                    mujoco.mjv_initGeom(v.user_scn.geoms[v.user_scn.ngeom],
                        mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.033,0,0]),
                        np.array([tx,ty,0.033]), np.eye(3).flatten(),
                        np.array([0.13,0.42,0.92,al], np.float32))
                    v.user_scn.ngeom += 1
                v.sync()

            dt = PHYS_DT/max(a.speed,0.05) - (time.time() - step_start)
            if dt > 0: time.sleep(dt)
    print('창을 닫았습니다.')


if __name__ == '__main__':
    main()
