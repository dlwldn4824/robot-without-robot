"""host.py — 호스트(PC)가 하드웨어 역할을 한다.

  MuJoCo = 차체·바퀴·센서 (하드웨어)
  libfw.dylib = firmware.c 를 컴파일한 것 (MCU 안에서 도는 코드)

  둘 사이는 ADC 카운트와 PWM 듀티로만 오간다.
  보드에 올릴 때는 같은 firmware.c 를 arm-none-eabi-gcc 로 빌드하면 된다.
"""
import argparse, ctypes, json, math, os, random
import numpy as np, mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
PHYS_DT, TICK_MS, MAX_OMEGA, PWM_MAX = 0.002, 20, 14.0, 255
GOAL_XY, IR_K, IR_C = (2.2, -1.9), 211_000, 111
TICKS_PER_REV = 360
CAR = {'body','hood','wing','wingL','wingR','t_fl','t_fr','t_rl','t_rr'}
STATES = ('BOOT','CRUISE','SLOW','AVOID_L','AVOID_R','ESCAPE','ARRIVED','BACKUP','SPIN')


class FwOut(ctypes.Structure):
    _fields_ = [('l_duty', ctypes.c_int16), ('r_duty', ctypes.c_int16),
                ('l_dir', ctypes.c_int8),  ('r_dir', ctypes.c_int8),
                ('state', ctypes.c_uint8)]


def load_fw():
    lib = ctypes.CDLL(os.path.join(HERE, 'libfw.dylib'))
    lib.fw_setup.restype = None
    lib.fw_loop.restype = FwOut
    lib.fw_loop.argtypes = [ctypes.c_uint32] + [ctypes.c_uint16]*5 + [ctypes.c_int32]
    lib.fw_get_mm.restype = ctypes.c_int32; lib.fw_get_mm.argtypes = [ctypes.c_int]
    lib.fw_get_stalls.restype = ctypes.c_uint16
    lib.fw_get_ticks.restype = ctypes.c_uint32
    lib.fw_get_changes.restype = ctypes.c_uint16
    return lib


# ── HAL (호스트가 흉내 내는 하드웨어) ────────────────────
def ir_adc(d, nz, rng):
    if d is None or d < 0: d = 4.0
    a = IR_K // (max(int(d*1000), 1) + IR_C)
    if nz: a += rng.randint(-nz, nz)
    return max(0, min(1023, int(a)))

def beacon(ang_deg, dist):
    out = []
    for off in (30, -30):
        c = max(0.0, math.cos(math.radians(ang_deg - off))) ** 2
        out.append(max(0, min(1023, int(900 * c / max(dist, .35) ** 2))))
    return out[0], out[1]

def enc_ticks(a): return int(a * TICKS_PER_REV / (2*math.pi))
def yaw_of(q):    return math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))


def run(duration=45.0, noise=0, seed=0, ppr=None, gif=None, log=None):
    global TICKS_PER_REV
    if ppr: TICKS_PER_REV = ppr
    fw = load_fw(); fw.fw_setup()
    m = mujoco.MjModel.from_xml_path(os.path.join(HERE, 'arena.xml'))
    d = mujoco.MjData(m); d.qpos[0:2] = (-2.2, 2.2); mujoco.mj_forward(m, d)
    rng = random.Random(seed)

    ren = cam = None; frames = []; trail = []
    if gif:
        ren = mujoco.Renderer(m, 400, 480)
        cam = mujoco.MjvCamera(); mujoco.mjv_defaultCamera(cam)
        cam.azimuth, cam.elevation, cam.distance = 180, -38, 2.9
        cam_az = 180.0

    spt = int(round((TICK_MS/1000)/PHYS_DT))
    gif_every = max(1, int(round(0.1/PHYS_DT)))
    prev_enc = 0; col = 0; in_col = False; plen = 0.0
    prev = d.qpos[:2].copy(); log_rows = []; out = None; adc = (0,0,0); bea = (0,0)
    arrived = None

    for k in range(int(duration/PHYS_DT)):
        if k % spt == 0:
            dl, df, dr = (float(d.sensordata[i]) for i in range(3))
            adc = (ir_adc(dl,noise,rng), ir_adc(df,noise,rng), ir_adc(dr,noise,rng))
            x, y = float(d.qpos[0]), float(d.qpos[1]); yw = yaw_of(d.qpos[3:7])
            gd = math.hypot(GOAL_XY[0]-x, GOAL_XY[1]-y)
            ga = (math.degrees(math.atan2(GOAL_XY[1]-y, GOAL_XY[0]-x) - yw) + 180) % 360 - 180
            bea = beacon(ga, gd)
            e = abs(enc_ticks(float(d.qpos[7]))) + abs(enc_ticks(float(d.qpos[8])))
            moved = abs(e - prev_enc); prev_enc = e

            out = fw.fw_loop(int(d.time*1000), *adc, *bea, moved)   # ← C 코드 호출

            wl = out.l_dir * (out.l_duty/PWM_MAX) * MAX_OMEGA
            wr = out.r_dir * (out.r_duty/PWM_MAX) * MAX_OMEGA
            d.ctrl[0] = d.ctrl[2] = wl; d.ctrl[1] = d.ctrl[3] = wr
            log_rows.append(dict(t=round(d.time,2), adc=list(adc), bea=list(bea),
                                 mm=[fw.fw_get_mm(i) for i in range(3)],
                                 st=STATES[out.state], pwm=[out.l_dir*out.l_duty, out.r_dir*out.r_duty],
                                 x=round(x,3), y=round(y,3)))
            if STATES[out.state] == 'ARRIVED' and arrived is None: arrived = round(d.time,2)

        mujoco.mj_step(m, d)
        hit = False
        for i in range(d.ncon):
            c = d.contact[i]
            nm = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom1),
                  mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom2)}
            if nm & CAR and not (nm & {'floor','GOAL'}) and (nm - CAR): hit = True; break
        if hit and not in_col: col += 1
        in_col = hit
        xy = d.qpos[:2].copy(); plen += float(np.linalg.norm(xy-prev)); prev = xy

        if ren is not None and k % gif_every == 0:
            from PIL import Image, ImageDraw
            if len(frames) % 3 == 0:
                trail.append((float(xy[0]), float(xy[1]))); trail[:] = trail[-18:]
            want = math.degrees(yaw_of(d.qpos[3:7])) + 180.0
            cam_az += ((want - cam_az + 540) % 360 - 180) * 0.18
            cam.azimuth = cam_az; cam.lookat[:] = [xy[0], xy[1], 0.1]
            ren.update_scene(d, camera=cam)
            sc = ren.scene
            for i,(tx,ty) in enumerate(trail):
                if sc.ngeom >= sc.maxgeom: break
                a = 0.10 + 0.55*(i/max(len(trail)-1,1))
                mujoco.mjv_initGeom(sc.geoms[sc.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                                    np.array([0.033,0,0]), np.array([tx,ty,0.033]),
                                    np.eye(3).flatten(), np.array([0.13,0.42,0.92,a], np.float32))
                sc.ngeom += 1
            shot = Image.fromarray(ren.render())
            img = Image.new('RGB', (480, 400+78), (255,255,255)); img.paste(shot, (0,78))
            dr = ImageDraw.Draw(img)
            st = STATES[out.state]
            dr.text((9,5),  f"firmware.c  tick {fw.fw_get_ticks():4d}   t={d.time:5.2f}s", fill=(20,30,50))
            dr.text((9,19), f"state = {st}", fill=(20,70,200))
            dr.text((9,33), f"ADC  L={adc[0]:4d} F={adc[1]:4d} R={adc[2]:4d}   beacon {bea[0]:4d}/{bea[1]:4d}", fill=(60,70,90))
            dr.text((9,47), f"mm   L={fw.fw_get_mm(0):4d} F={fw.fw_get_mm(1):4d} R={fw.fw_get_mm(2):4d}", fill=(60,70,90))
            dr.text((9,61), f"PWM  L{out.l_dir*out.l_duty:+4d}  R{out.r_dir*out.r_duty:+4d}    collisions={col}  stalls={fw.fw_get_stalls()}",
                    fill=(20,120,60) if col==0 else (190,40,40))
            frames.append(np.asarray(img))
        if arrived is not None and d.time - arrived > 0.8: break

    res = dict(duration=round(d.time,2), arrived=arrived, collisions=col,
               path_len=round(plen,2), stalls=int(fw.fw_get_stalls()),
               ticks=int(fw.fw_get_ticks()), state_changes=int(fw.fw_get_changes()),
               noise_lsb=noise, ppr=TICKS_PER_REV,
               goal_dist=round(math.hypot(GOAL_XY[0]-d.qpos[0], GOAL_XY[1]-d.qpos[1]),2))
    if log: json.dump(dict(result=res, log=log_rows), open(os.path.join(HERE,log),'w'), ensure_ascii=False)
    if gif:
        from PIL import Image
        pil=[Image.fromarray(f) for f in frames]
        picks=sorted({0,len(pil)//3,2*len(pil)//3,len(pil)-1}); w0,h0=pil[0].size
        st=Image.new('RGB',(w0,h0*len(picks)))
        for i,kk in enumerate(picks): st.paste(pil[kk],(0,i*h0))
        mst=st.quantize(colors=64, method=Image.MEDIANCUT)
        pil=[f.quantize(palette=mst, dither=Image.NONE) for f in pil]
        p=os.path.join(HERE,gif); os.makedirs(os.path.dirname(p),exist_ok=True)
        pil[0].save(p, save_all=True, append_images=pil[1:], duration=100, loop=0,
                    optimize=True, disposal=2)
        res['gif']=gif; res['frames']=len(pil)
    return res


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--duration', type=float, default=45)
    p.add_argument('--noise', type=int, default=0)
    p.add_argument('--ppr', type=int, default=None)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--gif', default=None); p.add_argument('--log', default=None)
    a = p.parse_args()
    print(json.dumps(run(a.duration, a.noise, a.seed, a.ppr, a.gif, a.log), ensure_ascii=False))
