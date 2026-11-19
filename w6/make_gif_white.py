"""기본 vs 개선 컨트롤러 비교 GIF (흰 배경·파란 장애물).

  python3 make_gif_white.py <초> <노이즈> <출력경로>

형상·마찰·컨트롤러는 make_gif.py 와 같고 색과 화면 구성만 다르다.
"""
import os, sys, random
import numpy as np, mujoco
from PIL import Image, ImageDraw

# ── 한글 폰트 (기본 비트맵 폰트는 한글을 □ 로 그린다) ──────────
from PIL import ImageFont as _IF
def _kfont(sz=13):
    for _p in ('/System/Library/Fonts/AppleSDGothicNeo.ttc',
               '/System/Library/Fonts/Supplemental/AppleGothic.ttf',
               '/Library/Fonts/Arial Unicode.ttf'):
        try: return _IF.truetype(_p, sz)
        except Exception: pass
    return _IF.load_default()
_KF = _kfont()
_orig_text = ImageDraw.ImageDraw.text
def _text(self, xy, txt, *a, **kw):
    kw.setdefault('font', _KF); return _orig_text(self, xy, txt, *a, **kw)
ImageDraw.ImageDraw.text = _text

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import controller as C
from controller_improved import ImprovedController
from run import read_sensors, count_collisions, MAX_WHEEL_SPEED, CONTROL_HZ

DUR   = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
NOISE = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
OUT   = sys.argv[3] if len(sys.argv) > 3 else 'runs/compare_white.gif'
FPS, W, H, HDR = 10, 380, 330, 80
WORLD = os.path.join(HERE, 'world_white.xml')


class Sim:
    def __init__(self, kind, label, color, trail_rgb):
        self.m = mujoco.MjModel.from_xml_path(WORLD)
        self.d = mujoco.MjData(self.m)
        self.kind, self.label, self.color, self.trail_rgb = kind, label, color, trail_rgb
        self.imp = ImprovedController(enter_th=0.7, exit_th=1.0, hold_s=0.45, avg_n=5)
        mujoco.mj_forward(self.m, self.d)
        self.r = mujoco.Renderer(self.m, H, W)
        self.r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
        self.r.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
        self.cam = mujoco.MjvCamera(); mujoco.mjv_defaultCamera(self.cam)
        self.cam.azimuth, self.cam.elevation, self.cam.distance = 90, -89, 8.6
        self.cam.lookat[:] = [0, 0, 0]
        self.col, self.inc, self.act = 0, False, 'STOP'
        self.prev = self.d.qpos[:2].copy(); self.plen = 0.0
        self.L = self.F = self.R = 0.0
        self.rng = random.Random(1)
        self.trail = []

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

    def frame(self, t):
        self.trail.append((float(self.d.qpos[0]), float(self.d.qpos[1])))
        self.trail[:] = self.trail[-45:]
        self.r.update_scene(self.d, camera=self.cam)
        sc = self.r.scene
        for i, (tx, ty) in enumerate(self.trail):
            if sc.ngeom >= sc.maxgeom:
                break
            a = 0.08 + 0.45 * (i / max(len(self.trail) - 1, 1))
            mujoco.mjv_initGeom(
                sc.geoms[sc.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                np.array([0.030, 0, 0]), np.array([tx, ty, 0.03]), np.eye(3).flatten(),
                np.array(list(self.trail_rgb) + [a], np.float32))
            sc.ngeom += 1
        shot = Image.fromarray(self.r.render())
        img = Image.new('RGB', (W, H + HDR), (255, 255, 255))
        img.paste(shot, (0, HDR))
        dr = ImageDraw.Draw(img)
        dr.text((8, 4),  self.label, fill=self.color)
        dr.text((8, 19), f"t={t:5.2f}s   {self.act}", fill=(25, 35, 55))
        dr.text((8, 33), f"L={self.L:4.2f}  F={self.F:4.2f}  R={self.R:4.2f}  m", fill=(70, 85, 110))
        dr.text((8, 47), f"판단 전환 기준 0.70 m", fill=(70, 85, 110))
        dr.text((8, 61), f"충돌 {self.col}회   {self.plen:4.1f} m",
                fill=(200, 40, 30) if self.col else (0, 135, 85))
        if self.inc:
            dr.rectangle([0, HDR, W - 1, H + HDR - 1], outline=(220, 40, 30), width=5)
        return np.asarray(img)


sims = [Sim('basic',    'BASIC  전방 센서만',          (190, 40, 30), (0.85, 0.20, 0.15)),
        Sim('improved', 'IMPROVED  +측면 +히스테리시스', (10, 80, 230),  (0.13, 0.42, 0.92))]

dt  = sims[0].m.opt.timestep
spc = max(1, int(round((1.0 / CONTROL_HZ) / dt)))
gif_every = max(1, int(round((1.0 / FPS) / dt)))

frames = []
for k in range(int(DUR / dt)):
    t = k * dt
    for s in sims:
        if k % spc == 0:
            s.control()
        s.step()
    if k % gif_every == 0:
        a, b = sims[0].frame(t), sims[1].frame(t)
        frames.append(np.concatenate([a, np.full((H + HDR, 6, 3), 235, np.uint8), b], axis=1))

pil = [Image.fromarray(f) for f in frames]
picks = sorted({0, len(pil) // 3, 2 * len(pil) // 3, len(pil) - 1})
w0, h0 = pil[0].size
st = Image.new('RGB', (w0, h0 * len(picks)))
for i, kk in enumerate(picks):
    st.paste(pil[kk], (0, i * h0))
mst = st.quantize(colors=56, method=Image.MEDIANCUT)
pil = [f.quantize(palette=mst, dither=Image.NONE) for f in pil]
p = os.path.join(HERE, OUT); os.makedirs(os.path.dirname(p), exist_ok=True)
pil[0].save(p, save_all=True, append_images=pil[1:], duration=int(1000 / FPS), loop=0,
            optimize=True, disposal=2)
print(f'saved {OUT}  {len(pil)} frames  {w0}x{h0}')
for s in sims:
    print(f'  {s.label:<32} 충돌 {s.col:>3}회   이동 {s.plen:5.2f} m')
