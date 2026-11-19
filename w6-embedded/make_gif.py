"""펌웨어 주행 GIF — 차를 따라가는 카메라 + 지나온 자리에 구슬 트레일."""
import math, os, sys, random
import numpy as np
import mujoco
from PIL import Image, ImageDraw
from hal import ir_adc, beacon_adc, encoder_ticks, PWM_MAX
from firmware import Firmware, TICK_MS
from mcu_sim import GOAL_XY, MAX_OMEGA, PHYS_DT, CAR_GEOMS, yaw_of

HERE = os.path.dirname(os.path.abspath(__file__))
DUR   = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
NOISE = int(sys.argv[2])   if len(sys.argv) > 2 else 0
OUT   = sys.argv[3]        if len(sys.argv) > 3 else 'runs/fw.gif'
FPS, W, H, HDR = 10, 420, 360, 74
TRAIL_N, TRAIL_EVERY = 16, 12      # 구슬 개수 / 몇 프레임마다 하나


def add_sphere(scn, pos, size, rgba):
    if scn.ngeom >= scn.maxgeom:
        return
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE,
                        np.array([size, 0, 0]), np.array(pos, float),
                        np.eye(3).flatten(), np.array(rgba, np.float32))
    scn.ngeom += 1


m = mujoco.MjModel.from_xml_path(os.path.join(HERE, 'arena.xml'))
d = mujoco.MjData(m)
d.qpos[0:2] = (-2.2, 2.2)
mujoco.mj_forward(m, d)
rng = random.Random(1)
fw = Firmware(); fw.setup()

r = mujoco.Renderer(m, H, W)
r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
cam = mujoco.MjvCamera(); mujoco.mjv_defaultCamera(cam)
cam.azimuth, cam.elevation, cam.distance = 180, -40, 2.7
cam_az = 180.0   # 차 뒤를 따라가는 체이스 캠 (급회전 때 어지럽지 않게 부드럽게)

spt = int(round((TICK_MS/1000.0)/PHYS_DT))
gif_every = max(1, int(round((1.0/FPS)/PHYS_DT)))
n = int(DUR/PHYS_DT)

trail, frames = [], []
cmd = None; adc = (0,0,0); bea = (0,0); col = 0; in_col = False; fcount = 0

for k in range(n):
    if k % spt == 0:
        dl, df, dr = (float(d.sensordata[i]) for i in range(3))
        adc = (ir_adc(dl, NOISE, rng), ir_adc(df, NOISE, rng), ir_adc(dr, NOISE, rng))
        x, y = float(d.qpos[0]), float(d.qpos[1]); yaw = yaw_of(d.qpos[3:7])
        gd = math.hypot(GOAL_XY[0]-x, GOAL_XY[1]-y)
        ga = (math.degrees(math.atan2(GOAL_XY[1]-y, GOAL_XY[0]-x) - yaw) + 180) % 360 - 180
        bea = beacon_adc(ga, gd)
        cmd = fw.loop(int(d.time*1000), *adc, *bea,
                      encoder_ticks(float(d.qpos[7])), encoder_ticks(float(d.qpos[8])))
        wl = cmd.l_dir*(cmd.l_duty/PWM_MAX)*MAX_OMEGA
        wr = cmd.r_dir*(cmd.r_duty/PWM_MAX)*MAX_OMEGA
        d.ctrl[0] = d.ctrl[2] = wl; d.ctrl[1] = d.ctrl[3] = wr
    mujoco.mj_step(m, d)
    hit = any((lambda nm: nm & CAR_GEOMS and not (nm & {'floor','GOAL'}) and (nm - CAR_GEOMS))(
                {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, d.contact[i].geom1),
                 mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, d.contact[i].geom2)})
              for i in range(d.ncon))
    if hit and not in_col: col += 1
    in_col = hit

    if k % gif_every == 0:
        fcount += 1
        if fcount % TRAIL_EVERY == 0:
            trail.append((float(d.qpos[0]), float(d.qpos[1])))
            trail[:] = trail[-TRAIL_N:]
        want = math.degrees(yaw_of(d.qpos[3:7])) + 180.0
        dz = (want - cam_az + 540) % 360 - 180
        cam_az += dz * 0.18
        cam.azimuth = cam_az
        cam.lookat[:] = [d.qpos[0], d.qpos[1], 0.1]
        r.update_scene(d, camera=cam)
        for i, (tx, ty) in enumerate(trail):          # 오래된 구슬일수록 흐리게
            a = 0.12 + 0.55 * (i / max(len(trail)-1, 1))
            add_sphere(r.scene, (tx, ty, 0.035), 0.035, (0.30, 0.85, 0.35, a))
        shot = Image.fromarray(r.render())
        img = Image.new('RGB', (W, H+HDR), (255,255,255)); img.paste(shot, (0, HDR))
        dr = ImageDraw.Draw(img)
        dr.text((8,4),  f"MCU tick {fw.ticks:4d}   t={d.time:5.2f}s   state={fw.state_name}",
                fill=(15,15,15))
        dr.text((8,20), f"ADC   L={adc[0]:4d}  F={adc[1]:4d}  R={adc[2]:4d}   (10-bit)", fill=(40,40,40))
        dr.text((8,34), f"filt  L={fw.l_mm:4d}  F={fw.f_mm:4d}  R={fw.r_mm:4d}  mm", fill=(40,40,40))
        dr.text((8,48), f"beacon L={bea[0]:4d} R={bea[1]:4d}    PWM  {cmd}", fill=(40,40,40))
        dr.text((8,62), f"collisions={col}   stalls={fw.stalls}", fill=(0,130,0) if col==0 else (200,0,0))
        frames.append(np.asarray(img))
    if fw.state == Firmware.ARRIVED and d.time > 1:
        for _ in range(FPS):   # 도착 후 1초 정지 화면
            frames.append(frames[-1])
        break

pil = [Image.fromarray(f) for f in frames]
picks = sorted({0, len(pil)//3, 2*len(pil)//3, len(pil)-1})
w0,h0 = pil[0].size
st = Image.new('RGB',(w0,h0*len(picks)))
for i,kk in enumerate(picks): st.paste(pil[kk],(0,i*h0))
master = st.quantize(colors=64, method=Image.MEDIANCUT)
pil = [f.quantize(palette=master, dither=Image.NONE) for f in pil]
outp = os.path.join(HERE, OUT)
os.makedirs(os.path.dirname(outp), exist_ok=True)
pil[0].save(outp, save_all=True, append_images=pil[1:], duration=int(1000/FPS), loop=0,
            optimize=True, disposal=2)
print(f'saved {OUT}  {len(pil)} frames  {w0}x{h0}  collisions={col} stalls={fw.stalls} '
      f'arrived={fw.state_name=="ARRIVED"}')
