"""기본 vs 개선 컨트롤러를 나란히 돌려 비교 GIF 를 만든다.

  python make_gif.py <초> <노이즈> <출력경로>
"""
import os, sys, random
import numpy as np
import mujoco
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import controller as C
from controller_improved import ImprovedController
from run import read_sensors, count_collisions, MAX_WHEEL_SPEED, CONTROL_HZ

DUR   = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
NOISE = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
OUT   = sys.argv[3] if len(sys.argv) > 3 else 'runs/compare.gif'
FPS, W, H, HDR = 6, 272, 212, 62


def make_cam():
    cam = mujoco.MjvCamera(); mujoco.mjv_defaultCamera(cam)
    cam.azimuth, cam.elevation, cam.distance = 90, -89, 9.6   # 아레나 전체가 들어오게
    cam.lookat[:] = [0, 0, 0]
    return cam


class Sim:
    def __init__(self, kind, label, color):
        self.m = mujoco.MjModel.from_xml_path(os.path.join(HERE, 'world.xml'))
        self.d = mujoco.MjData(self.m)
        self.kind, self.label, self.color = kind, label, color
        self.imp = ImprovedController(enter_th=0.7, exit_th=1.0, hold_s=0.45, avg_n=5)
        mujoco.mj_forward(self.m, self.d)      # run.py 와 초기 상태를 맞춘다
        self.r = mujoco.Renderer(self.m, H, W)
        # 그림자를 끄면 프레임 간 변화가 줄어 GIF 가 크게 작아진다
        self.r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
        self.r.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
        self.cam = make_cam()
        self.col, self.inc, self.act = 0, False, 'STOP'
        self.prev = self.d.qpos[:2].copy(); self.plen = 0.0
        self.L = self.F = self.R = 0.0
        self.rng = random.Random(1)

    def control(self):
        self.L, self.F, self.R = read_sensors(self.m, self.d, NOISE, self.rng)
        self.act = (self.imp.decide(self.L, self.F, self.R, self.d.time) if self.kind == 'improved'
                    else C.decide_action(self.L, self.F, self.R, 0.7))
        wl, wr = C.action_to_wheels(self.act)
        self.d.ctrl[0] = wl * MAX_WHEEL_SPEED
        self.d.ctrl[1] = wr * MAX_WHEEL_SPEED

    def step(self):
        mujoco.mj_step(self.m, self.d)
        hit = count_collisions(self.m, self.d) > 0
        if hit and not self.inc:
            self.col += 1
        self.inc = hit
        xy = self.d.qpos[:2].copy()
        self.plen += float(np.linalg.norm(xy - self.prev)); self.prev = xy

    def frame(self):
        self.r.update_scene(self.d, camera=self.cam)
        shot = Image.fromarray(self.r.render())
        # 영상 위에 글자를 덮지 않고, 머리말 칸을 따로 붙인다
        img = Image.new('RGB', (W, H + HDR), (255, 255, 255))
        img.paste(shot, (0, HDR))
        dr = ImageDraw.Draw(img)
        dr.text((8, 4), self.label, fill=self.color)
        dr.text((8, 20), f"L={self.L:4.2f}  F={self.F:4.2f}  R={self.R:4.2f}  m", fill=(20, 20, 20))
        dr.text((8, 34), f"action = {self.act}", fill=(20, 20, 20))
        dr.text((8, 48), f"collisions = {self.col}    travelled = {self.plen:4.1f} m",
                fill=(200, 0, 0) if self.col else (0, 130, 0))
        if self.inc:
            dr.rectangle([0, HDR, W - 1, H + HDR - 1], outline=(230, 0, 0), width=5)
        return np.asarray(img)


sims = [Sim('basic',    'BASIC: front sensor only', (170, 0, 0)),
        Sim('improved', 'IMPROVED: +side +hysteresis', (0, 110, 0))]

spc = max(1, int(round((1.0 / CONTROL_HZ) / sims[0].m.opt.timestep)))
gif_every = max(1, int(round((1.0 / FPS) / sims[0].m.opt.timestep)))
n = int(DUR / sims[0].m.opt.timestep)

frames = []
for k in range(n):
    for s in sims:
        if k % spc == 0:
            s.control()
        s.step()
    if k % gif_every == 0:
        a, b = sims[0].frame(), sims[1].frame()
        frames.append(np.concatenate([a, np.full((H + HDR, 6, 3), 235, np.uint8), b], axis=1))

# 프레임마다 팔레트를 새로 만들면 색이 흔들린다 -> 공통 팔레트 한 번만
pil = [Image.fromarray(f) for f in frames]
picks = sorted({0, len(pil)//3, 2*len(pil)//3, len(pil)-1})
w0, h0 = pil[0].size
stack = Image.new('RGB', (w0, h0*len(picks)))
for i, k in enumerate(picks):
    stack.paste(pil[k], (0, i*h0))
master = stack.quantize(colors=48, method=Image.MEDIANCUT)
pil = [f.quantize(palette=master, dither=Image.NONE) for f in pil]
os.makedirs(os.path.join(HERE, os.path.dirname(OUT)) or '.', exist_ok=True)
outp = OUT if os.path.isabs(OUT) else os.path.join(HERE, OUT)
pil[0].save(outp, save_all=True, append_images=pil[1:],
            duration=int(1000/FPS), loop=0, optimize=True, disposal=2)
print(f'saved {outp}  ({len(frames)} frames, {frames[0].shape[1]}x{frames[0].shape[0]}, noise={NOISE})')
for s in sims:
    print(f'  {s.label:<32} collisions={s.col:>3}  travelled={s.plen:5.2f} m')
