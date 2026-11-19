"""MCU + 플랜트 결합 시뮬레이터.

  MuJoCo = 하드웨어(차체·바퀴·센서),  firmware.py = MCU 안에서 도는 코드.
  둘 사이는 HAL(ADC/PWM)로만 연결된다. 펌웨어는 '미터'도 'float'도 모른다.
"""
import argparse, json, math, os, random
import numpy as np
import mujoco
from hal import ir_adc, beacon_adc, encoder_ticks, PWM_MAX
from firmware import Firmware, TICK_MS

HERE = os.path.dirname(os.path.abspath(__file__))
PHYS_DT     = 0.002
MAX_OMEGA   = 14.0          # PWM 255 일 때 바퀴 각속도 [rad/s]
CAR_GEOMS   = {'body','hood','wing','wingL','wingR','t_fl','t_fr','t_rl','t_rr'}
GOAL_XY     = (2.2, -1.9)
GOAL_R      = 0.34


def yaw_of(q):
    return math.atan2(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]**2 + q[3]**2))


def run(duration=45.0, noise_counts=0, seed=0, log_path=None, trail=None):
    m = mujoco.MjModel.from_xml_path(os.path.join(HERE, 'arena.xml'))
    d = mujoco.MjData(m)
    d.qpos[0:2] = (-2.2, 2.2)                       # 반대쪽 구석에서 출발
    mujoco.mj_forward(m, d)
    rng = random.Random(seed)

    fw = Firmware(); fw.setup()
    steps_per_tick = int(round((TICK_MS/1000.0) / PHYS_DT))
    n = int(duration / PHYS_DT)

    log, collisions, in_col = [], 0, False
    prev = d.qpos[:2].copy(); plen = 0.0
    arrived_at = None

    for k in range(n):
        if k % steps_per_tick == 0:
            # ---- 하드웨어가 MCU 에 넘기는 것: ADC 카운트뿐
            dl, df, dr = (float(d.sensordata[i]) for i in range(3))
            adc_l = ir_adc(dl, noise_counts, rng)
            adc_f = ir_adc(df, noise_counts, rng)
            adc_r = ir_adc(dr, noise_counts, rng)

            x, y = float(d.qpos[0]), float(d.qpos[1])
            yaw = yaw_of(d.qpos[3:7])
            gx, gy = GOAL_XY
            gdist = math.hypot(gx - x, gy - y)
            gang = math.degrees(math.atan2(gy - y, gx - x) - yaw)
            gang = (gang + 180) % 360 - 180
            bea_l, bea_r = beacon_adc(gang, gdist)

            # ---- MCU 한 틱
            # 엔코더: 좌/우 각 1개 (앞바퀴 관절 각도)
            enc_l = encoder_ticks(float(d.qpos[7]))      # j_fl
            enc_r = encoder_ticks(float(d.qpos[8]))      # j_fr
            cmd = fw.loop(int(d.time*1000), adc_l, adc_f, adc_r, bea_l, bea_r, enc_l, enc_r)

            # ---- MCU 가 내보내는 것: 방향 + PWM 듀티뿐
            wl = cmd.l_dir * (cmd.l_duty / PWM_MAX) * MAX_OMEGA
            wr = cmd.r_dir * (cmd.r_duty / PWM_MAX) * MAX_OMEGA
            d.ctrl[0] = d.ctrl[2] = wl        # 좌측 2륜
            d.ctrl[1] = d.ctrl[3] = wr        # 우측 2륜

            log.append(dict(t=round(d.time,2), adc=[adc_l,adc_f,adc_r],
                            mm=[fw.l_mm, fw.f_mm, fw.r_mm], bea=[bea_l,bea_r],
                            st=fw.state_name, l=cmd.l_dir*cmd.l_duty, r=cmd.r_dir*cmd.r_duty,
                            x=round(x,3), y=round(y,3), gd=round(gdist,2)))
            if fw.state == Firmware.ARRIVED and arrived_at is None:
                arrived_at = round(d.time, 2)

        mujoco.mj_step(m, d)

        hit = False
        for i in range(d.ncon):
            c = d.contact[i]
            g1 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom1)
            g2 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom2)
            nm = {g1, g2}
            if nm & CAR_GEOMS and not (nm & {'floor', 'GOAL'}) and (nm - CAR_GEOMS):
                hit = True; break
        if hit and not in_col: collisions += 1
        in_col = hit

        xy = d.qpos[:2].copy(); plen += float(np.linalg.norm(xy - prev)); prev = xy
        if trail is not None and k % 50 == 0:
            trail.append((float(xy[0]), float(xy[1])))
        if arrived_at is not None and d.time - arrived_at > 1.0:
            break

    res = dict(duration=round(d.time,2), collisions=collisions, path_len=round(plen,2),
               arrived=arrived_at, ticks=fw.ticks, state_changes=fw.state_changes, stalls=fw.stalls,
               noise_counts=noise_counts,
               final_goal_dist=round(math.hypot(GOAL_XY[0]-d.qpos[0], GOAL_XY[1]-d.qpos[1]), 2))
    if log_path:
        json.dump(dict(result=res, log=log), open(os.path.join(HERE, log_path), 'w'), ensure_ascii=False)
    return res, log


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--duration', type=float, default=45.0)
    p.add_argument('--noise', type=int, default=0, help='ADC 카운트 노이즈 ±N')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--log', default=None)
    p.add_argument('--trace', action='store_true')
    a = p.parse_args()
    res, log = run(a.duration, a.noise, a.seed, a.log)
    if a.trace:
        print(f"{'t':>6} {'ADC L/F/R':>16} {'필터 mm':>16} {'비컨 L/R':>12} {'상태':<9} {'PWM L/R':>10}")
        for e in log[::25]:
            print(f"{e['t']:6.2f} {str(e['adc']):>16} {str(e['mm']):>16} "
                  f"{str(e['bea']):>12} {e['st']:<9} {e['l']:>5}{e['r']:>5}")
    print(json.dumps(res, ensure_ascii=False))
