"""명세판(fw_basic) vs 펌웨어판(fw_i_loop) 을 3D 에서 동시에 돌려 비교 GIF 를 만든다.
두 판단 모두 C 코드에서 나온다. 파이썬은 하드웨어 역할만 한다."""
import ctypes, math, os, random, sys
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
from host import (load_fw, ir_adc, beacon, yaw_of, GOAL_XY, PHYS_DT, TICK_MS,
                  MAX_OMEGA, PWM_MAX, CAR, STATES)
import host

HERE = os.path.dirname(os.path.abspath(__file__))
DUR  = float(sys.argv[1]) if len(sys.argv) > 1 else 26.0
OUT  = sys.argv[2] if len(sys.argv) > 2 else 'runs/fw_vs_spec.gif'
W, H, HDR, FPS = 380, 330, 80, 10
ACTS = ['FORWARD', 'TURN_LEFT', 'TURN_RIGHT']

fw = load_fw()
fw.fw_basic.restype = ctypes.c_int32
fw.fw_basic.argtypes = [ctypes.c_int32]*4
fw.fw_act_l.restype = fw.fw_act_r.restype = ctypes.c_int32
fw.fw_act_l.argtypes = fw.fw_act_r.argtypes = [ctypes.c_int32]
fw.fw_adc_to_mm.restype = ctypes.c_int32; fw.fw_adc_to_mm.argtypes = [ctypes.c_int32]
fw.fw_i_setup.argtypes = [ctypes.c_int32]
fw.fw_i_tick.argtypes = [ctypes.c_int32, ctypes.c_uint32] + [ctypes.c_uint16]*5 + [ctypes.c_int32]
for n in ('fw_i_out_l','fw_i_out_r','fw_i_state','fw_i_stalls'):
    getattr(fw, n).restype = ctypes.c_int32; getattr(fw, n).argtypes = [ctypes.c_int32]


class Sim:
    def __init__(self, kind, label, color):
        self.m = mujoco.MjModel.from_xml_path(os.path.join(HERE, 'arena.xml'))
        self.d = mujoco.MjData(self.m); self.d.qpos[0:2] = (-2.2, 2.2)
        mujoco.mj_forward(self.m, self.d)
        self.kind, self.label, self.color = kind, label, color
        self.slot = 1 if kind == 'fw' else 3
        fw.fw_i_setup(self.slot)
        self.r = mujoco.Renderer(self.m, H, W)
        self.r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
        self.cam = mujoco.MjvCamera(); mujoco.mjv_defaultCamera(self.cam)
        self.cam.azimuth, self.cam.elevation, self.cam.distance = 90, -89, 8.0
        self.cam.lookat[:] = [0, 0, 0]
        self.rng = random.Random(1)
        self.col = 0; self.inc = False; self.plen = 0.0
        self.prev = self.d.qpos[:2].copy(); self.trail = []
        self.adc = (0,0,0); self.act = '-'; self.pwm = (0,0); self.prev_enc = 0; self.arrived = None

    def control(self, t):
        dl, df, dr = (float(self.d.sensordata[i]) for i in range(3))
        self.adc = (ir_adc(dl,0,self.rng), ir_adc(df,0,self.rng), ir_adc(dr,0,self.rng))
        mm = [fw.fw_adc_to_mm(a) for a in self.adc]
        if self.kind == 'spec':
            a = fw.fw_basic(mm[0], mm[1], mm[2], 700)     # ← 명세 알고리즘 (C)
            pl, pr = fw.fw_act_l(a), fw.fw_act_r(a)
            self.act = ACTS[a]
        else:
            x, y = float(self.d.qpos[0]), float(self.d.qpos[1]); yw = yaw_of(self.d.qpos[3:7])
            gd = math.hypot(GOAL_XY[0]-x, GOAL_XY[1]-y)
            ga = (math.degrees(math.atan2(GOAL_XY[1]-y, GOAL_XY[0]-x) - yw) + 180) % 360 - 180
            bl, br = beacon(ga, gd)
            e = abs(host.enc_ticks(float(self.d.qpos[7]))) + abs(host.enc_ticks(float(self.d.qpos[8])))
            moved = abs(e - self.prev_enc); self.prev_enc = e
            fw.fw_i_tick(self.slot, int(t*1000), *self.adc, bl, br, moved)
            pl, pr = fw.fw_i_out_l(self.slot), fw.fw_i_out_r(self.slot)
            self.act = STATES[fw.fw_i_state(self.slot)]
            if self.act == 'ARRIVED' and self.arrived is None: self.arrived = t
        self.pwm = (pl, pr)
        self.d.ctrl[0] = self.d.ctrl[2] = (pl/PWM_MAX)*MAX_OMEGA
        self.d.ctrl[1] = self.d.ctrl[3] = (pr/PWM_MAX)*MAX_OMEGA

    def step(self):
        mujoco.mj_step(self.m, self.d)
        hit = False
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            nm = {mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_GEOM, c.geom1),
                  mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_GEOM, c.geom2)}
            if nm & CAR and not (nm & {'floor','GOAL'}) and (nm - CAR): hit = True; break
        if hit and not self.inc: self.col += 1
        self.inc = hit
        xy = self.d.qpos[:2].copy(); self.plen += float(np.linalg.norm(xy-self.prev)); self.prev = xy

    def frame(self, t):
        self.trail.append((float(self.d.qpos[0]), float(self.d.qpos[1]))); self.trail[:] = self.trail[-40:]
        self.r.update_scene(self.d, camera=self.cam)
        sc = self.r.scene
        for i,(tx,ty) in enumerate(self.trail):
            if sc.ngeom >= sc.maxgeom: break
            a = 0.08 + 0.45*(i/max(len(self.trail)-1,1))
            mujoco.mjv_initGeom(sc.geoms[sc.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                np.array([0.028,0,0]), np.array([tx,ty,0.03]), np.eye(3).flatten(),
                np.array([0.13,0.42,0.92,a] if self.kind=='fw' else [0.85,0.2,0.15,a], np.float32))
            sc.ngeom += 1
        shot = Image.fromarray(self.r.render())
        img = Image.new('RGB',(W,H+HDR),(255,255,255)); img.paste(shot,(0,HDR))
        dr = ImageDraw.Draw(img)
        dr.text((8,4), self.label, fill=self.color)
        dr.text((8,19), f"t={t:5.2f}s   {self.act}", fill=(25,35,55))
        dr.text((8,33), f"ADC L={self.adc[0]:4d} F={self.adc[1]:4d} R={self.adc[2]:4d}", fill=(70,85,110))
        dr.text((8,47), f"PWM L{self.pwm[0]:+4d}  R{self.pwm[1]:+4d}", fill=(70,85,110))
        msg = f"충돌 {self.col}회   {self.plen:4.1f} m"
        if self.arrived: msg += f"   ★{self.arrived:.1f}s 도착"
        dr.text((8,61), msg, fill=(200,40,30) if self.col else (0,135,85))
        if self.inc:
            dr.rectangle([0,HDR,W-1,H+HDR-1], outline=(220,40,30), width=5)
        return np.asarray(img)


A = Sim('spec','SPEC  fw_basic()  (과제 명세)',(190,40,30))
B = Sim('fw','FIRMWARE  fw_i_tick()',(10,80,230))
spt = int(round((TICK_MS/1000)/PHYS_DT)); gif_every = max(1,int(round((1/FPS)/PHYS_DT)))
frames = []
for k in range(int(DUR/PHYS_DT)):
    t = k*PHYS_DT
    if k % spt == 0:
        A.control(t); B.control(t)
    A.step(); B.step()
    if k % gif_every == 0:
        frames.append(np.concatenate([A.frame(t), np.full((H+HDR,6,3),235,np.uint8), B.frame(t)],axis=1))

pil=[Image.fromarray(f) for f in frames]
picks=sorted({0,len(pil)//3,2*len(pil)//3,len(pil)-1}); w0,h0=pil[0].size
st=Image.new('RGB',(w0,h0*len(picks)))
for i,kk in enumerate(picks): st.paste(pil[kk],(0,i*h0))
mst=st.quantize(colors=56, method=Image.MEDIANCUT)
pil=[f.quantize(palette=mst, dither=Image.NONE) for f in pil]
p=os.path.join(HERE,OUT); os.makedirs(os.path.dirname(p),exist_ok=True)
pil[0].save(p, save_all=True, append_images=pil[1:], duration=int(1000/FPS), loop=0,
            optimize=True, disposal=2)
print(f'saved {OUT}  {len(pil)} frames  {w0}x{h0}')
print(f'  SPEC     충돌 {A.col}회  {A.plen:.2f} m')
print(f'  FIRMWARE 충돌 {B.col}회  {B.plen:.2f} m  도착 {B.arrived}')
