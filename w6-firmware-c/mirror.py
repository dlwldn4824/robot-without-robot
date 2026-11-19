"""거울 앞에서 적외선 거리센서가 어떻게 되는지.

  같은 자리·같은 크기의 판을 하나는 불투명 벽, 하나는 거울로 두고
  같은 firmware.c 를 돌린다. 바뀌는 것은 '표면이 빛을 어떻게 돌려보내는가' 뿐이다.

  광학 모델 (문서화된 가정):
    - 거울은 정반사만 한다. 입사각 = 반사각.
    - 수신부가 되돌아온 빛을 잡으려면 광선이 거울 법선에서 ACCEPT_DEG 안이어야 한다.
      (실제 삼각측량 IR 은 발광부·수광부 간격이 수 mm 라 이보다 훨씬 좁다)
    - 그 안이면 광선은 반사되어 계속 가고, 센서는 '간 거리의 합'을 거리로 읽는다 → 허상.
    - 벗어나면 돌아오는 빛이 없다 → 아무것도 없는 것으로 읽는다 (CUTOFF).
  확산 반사하는 보통 벽은 어느 각도에서나 일부가 되돌아오므로 이 규칙을 적용하지 않는다.

  사용:  python3 mirror.py gif  [초]      두 판 비교 GIF
         python3 mirror.py sweep           입사각 스윕 표
"""
import ctypes, math, os, random, sys
import numpy as np, mujoco
from PIL import Image, ImageDraw, ImageFont
from host import load_fw, ir_adc, beacon, yaw_of, PHYS_DT, TICK_MS, MAX_OMEGA, PWM_MAX, CAR, STATES
import host

HERE = os.path.dirname(os.path.abspath(__file__))
W, H, HDR, FPS = 380, 330, 80, 10
CUTOFF     = 4.0      # 센서 최대 측정 거리 [m]
ACCEPT_DEG = 10.0     # 수신부가 되돌아온 빛을 잡는 최대 입사각
GOAL       = (2.1, -0.9)
START      = (-1.9, 0.0)

def kfont(sz):
    for p in ('/System/Library/Fonts/AppleSDGothicNeo.ttc',
              '/System/Library/Fonts/Supplemental/AppleGothic.ttf',
              '/Library/Fonts/Arial Unicode.ttf'):
        try: return ImageFont.truetype(p, sz)
        except Exception: pass
    return ImageFont.load_default()
FNT, FNB = kfont(13), kfont(13)

XML = """<mujoco model="fw_mirror">
  <compiler angle="degree" autolimits="true"/>
  <option timestep="0.002" integrator="implicitfast"/>
  <visual>
    <global offwidth="1920" offheight="1440"/>
    <quality shadowsize="2048"/>
    <headlight diffuse="0.80 0.80 0.82" ambient="0.52 0.53 0.56" specular="0.1 0.1 0.1"/>
    <rgba haze="1 1 1 1"/>
  </visual>
  <asset>
    <texture name="sky" type="skybox" builtin="flat" rgb1="1 1 1" rgb2="1 1 1" width="64" height="64"/>
    <material name="floor"  rgba="0.985 0.988 1.0 1" reflectance="0.05"/>
    <material name="wall"   rgba="0.88 0.90 0.94 1"/>
    <material name="panelW" rgba="0.62 0.66 0.72 1"/>
    <material name="panelM" rgba="0.78 0.86 0.95 1" reflectance="0.92" specular="1" shininess="1"/>
    <material name="goal"   rgba="0.20 0.58 1.0 0.5"/>
  </asset>
  <worldbody>
    <light pos="0 0 6" dir="0 0 -1" diffuse="0.55 0.55 0.57" castshadow="true"/>
    <light pos="-2 2 4" dir="0.4 -0.4 -1" diffuse="0.22 0.22 0.24"/>
    <geom name="floor" type="plane" size="6 6 0.1" material="floor" friction="1.1 0.005 0.0001"/>
    <geom name="wall_n" type="box" pos=" 0   1.75 0.18" size="2.58 0.08 0.18" material="wall"/>
    <geom name="wall_s" type="box" pos=" 0  -1.75 0.18" size="2.58 0.08 0.18" material="wall"/>
    <geom name="wall_e" type="box" pos=" 2.5  0   0.18" size="0.08 1.83 0.18" material="wall"/>
    <geom name="wall_w" type="box" pos="-2.5  0   0.18" size="0.08 1.83 0.18" material="wall"/>
    <geom name="PANEL" type="box" pos="1.0 -0.4 0.26" size="0.035 1.30 0.26" material="{surf}"/>
    <geom name="GOAL" type="cylinder" pos="2.1 -0.9 0.14" size="0.26 0.14" material="goal"/>
  </worldbody>
  <include file="car.xml"/>
</mujoco>"""

fw = load_fw()
fw.fw_adc_to_mm.restype = ctypes.c_int32; fw.fw_adc_to_mm.argtypes = [ctypes.c_int32]
fw.fw_i_setup.argtypes = [ctypes.c_int32]
fw.fw_i_tick.argtypes = [ctypes.c_int32, ctypes.c_uint32] + [ctypes.c_uint16]*5 + [ctypes.c_int32]
for n in ('fw_i_out_l','fw_i_out_r','fw_i_state','fw_i_stalls'):
    getattr(fw, n).restype = ctypes.c_int32; getattr(fw, n).argtypes = [ctypes.c_int32]


class Scene:
    """거울이냐 벽이냐만 다른 똑같은 방."""
    def __init__(self, mirror):
        self.mirror = mirror
        self.m = mujoco.MjModel.from_xml_string(XML.format(surf='panelM' if mirror else 'panelW'),
                                                {'car.xml': open(os.path.join(HERE,'car.xml'),'rb').read()})
        self.d = mujoco.MjData(self.m)
        self.d.qpos[0:2] = START
        mujoco.mj_forward(self.m, self.d)
        self.panel = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_GEOM, 'PANEL')
        self.chassis = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, 'chassis')
        self.sid = [mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE, s)
                    for s in ('s_left','s_front','s_right')]

    def ray(self, k):
        """센서 k 가 실제로 읽는 거리[m] 와 (입사각, 상태) 를 돌려준다.
        상태: 'ok' 보통 반사 / 'ghost' 거울 허상 / 'lost' 되돌아온 빛 없음"""
        sid = self.sid[k]
        pnt = self.d.site_xpos[sid].copy()
        vec = self.d.site_xmat[sid].reshape(3, 3)[:, 2].copy()
        gid = np.zeros(1, np.int32); nrm = np.zeros(3)
        dist = mujoco.mj_ray(self.m, self.d, pnt, vec, None, 1, self.chassis, gid, nrm)
        if dist < 0: return CUTOFF, None, 'ok'
        if not (self.mirror and gid[0] == self.panel):
            return min(dist, CUTOFF), None, 'ok'
        ang = math.degrees(math.acos(min(1.0, abs(float(np.dot(vec, nrm))))))
        if ang > ACCEPT_DEG + 1e-6:
            return CUTOFF, ang, 'lost'                    # 반사광이 수신부를 벗어난다
        hit = pnt + vec*dist
        vec2 = vec - 2*float(np.dot(vec, nrm))*nrm        # 입사각 = 반사각
        d2 = mujoco.mj_ray(self.m, self.d, hit + vec2*1e-4, vec2, None, 1, -1, gid, None)
        total = dist + (d2 if d2 >= 0 else CUTOFF)
        return min(total, CUTOFF), ang, 'ghost'           # 거울 너머에 있는 것처럼 읽힌다


class Sim(Scene):
    def __init__(self, mirror, label, color, trail, slot):
        super().__init__(mirror)
        self.label, self.color, self.trail_rgb, self.slot = label, color, trail, slot
        fw.fw_i_setup(slot)
        self.r = mujoco.Renderer(self.m, H, W)
        self.r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 1
        self.r.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 1
        self.cam = mujoco.MjvCamera(); mujoco.mjv_defaultCamera(self.cam)
        self.cam.azimuth, self.cam.elevation, self.cam.distance = 0, -55, 6.5
        self.cam.lookat[:] = [-0.10, -0.10, 0.12]
        self.rng = random.Random(1)
        self.col = 0; self.inc = False; self.plen = 0.0; self.hitp = 0
        self.free = 999; self.first = None; self.press = 0.0; self.enc_sum = 0
        self.prev = self.d.qpos[:2].copy(); self.trail = []
        self.mm = (0,0,0); self.tag = ['ok']*3; self.ang = [None]*3
        self.act = '-'; self.prev_enc = 0

    def control(self, t):
        rd = [self.ray(k) for k in range(3)]
        self.tag = [r[2] for r in rd]; self.ang = [r[1] for r in rd]
        adc = tuple(ir_adc(r[0], 0, self.rng) for r in rd)
        self.mm = tuple(fw.fw_adc_to_mm(a) for a in adc)
        x, y = float(self.d.qpos[0]), float(self.d.qpos[1]); yw = yaw_of(self.d.qpos[3:7])
        gd = math.hypot(GOAL[0]-x, GOAL[1]-y)
        ga = (math.degrees(math.atan2(GOAL[1]-y, GOAL[0]-x) - yw) + 180) % 360 - 180
        bl, br = beacon(ga, gd)
        e = abs(host.enc_ticks(float(self.d.qpos[7]))) + abs(host.enc_ticks(float(self.d.qpos[8])))
        moved = abs(e - self.prev_enc); self.prev_enc = e; self.enc_sum += moved
        fw.fw_i_tick(self.slot, int(t*1000), *adc, bl, br, moved)
        pl, pr = fw.fw_i_out_l(self.slot), fw.fw_i_out_r(self.slot)
        self.act = STATES[fw.fw_i_state(self.slot)]
        self.d.ctrl[0] = self.d.ctrl[2] = (pl/PWM_MAX)*MAX_OMEGA
        self.d.ctrl[1] = self.d.ctrl[3] = (pr/PWM_MAX)*MAX_OMEGA

    def step(self):
        mujoco.mj_step(self.m, self.d)
        hit = panel = False
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            nm = {mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_GEOM, c.geom1),
                  mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_GEOM, c.geom2)}
            if nm & CAR and not (nm & {'floor','GOAL'}) and (nm - CAR):
                hit = True
                if 'PANEL' in nm: panel = True
        # 접촉이 0.25 s 이상 끊겼다가 다시 닿을 때만 새 충돌로 센다
        if hit:
            if self.free * PHYS_DT >= 0.25:
                self.col += 1
                if panel: self.hitp += 1
                if panel and self.first is None: self.first = self.d.time
            self.free = 0
            if panel: self.press += PHYS_DT
        else:
            self.free += 1
        self.inc = hit
        xy = self.d.qpos[:2].copy(); self.plen += float(np.linalg.norm(xy-self.prev)); self.prev = xy

    def frame(self, t):
        self.trail.append((float(self.d.qpos[0]), float(self.d.qpos[1]))); self.trail[:] = self.trail[-45:]
        self.r.update_scene(self.d, camera=self.cam)
        sc = self.r.scene
        for i,(tx,ty) in enumerate(self.trail):
            if sc.ngeom >= sc.maxgeom: break
            a = 0.10 + 0.50*(i/max(len(self.trail)-1,1))
            mujoco.mjv_initGeom(sc.geoms[sc.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                np.array([0.030,0,0]), np.array([tx,ty,0.028]), np.eye(3).flatten(),
                np.array(list(self.trail_rgb)+[a], np.float32)); sc.ngeom += 1
        img = Image.new('RGB',(W,H+HDR),(255,255,255))
        img.paste(Image.fromarray(self.r.render()),(0,HDR))
        dr = ImageDraw.Draw(img)
        dr.text((8,3), self.label, fill=self.color, font=FNB)
        dr.text((8,20), f"t={t:5.2f}s   {self.act}", fill=(25,35,55), font=FNT)
        dr.text((8,35), f"읽은 거리  L{self.mm[0]:5d}  F{self.mm[1]:5d}  R{self.mm[2]:5d} mm"
                        f"   (멈춤 기준 480)", fill=(70,85,110), font=FNT)
        nm = ['L','F','R']
        s = '   '.join(f"{nm[k]} {'반사손실' if self.tag[k]=='lost' else ('허상' if self.tag[k]=='ghost' else '정상')}"
                       for k in range(3))
        dr.text((8,50), s, fill=(200,80,20) if 'ok' not in self.tag else (70,85,110), font=FNT)
        msg = f"판 충돌 {self.hitp}회   이동 {self.plen:4.1f} m"
        if self.first is not None: msg += f"   갇힌 뒤 {t-self.first:4.1f}s"
        dr.text((8,65), msg, fill=(200,40,30) if self.hitp else (0,135,85), font=FNT)
        if self.inc: dr.rectangle([0,HDR,W-1,H+HDR-1], outline=(220,40,30), width=5)
        return np.asarray(img)


def sweep():
    """거울 면에서 0.60 m 떨어진 자리에 세워두고 로봇만 돌려가며 전방 센서를 읽는다."""
    sc, wl = Scene(True), Scene(False)
    px, py = 1.0 - 0.035 - 0.60 - 0.152, -0.40
    rows = []
    print(f"{'로봇 방향':>8}  {'입사각':>5}  {'실제 거리':>8}  {'벽':>8}  {'거울':>8}  판정")
    for yaw_deg in range(-30, 31, 5):
        yaw = math.radians(yaw_deg)
        for s in (sc, wl):
            s.d.qpos[0:3] = (px, py, s.d.qpos[2])
            s.d.qpos[3:7] = (math.cos(yaw/2), 0, 0, math.sin(yaw/2))
            mujoco.mj_forward(s.m, s.d)
        dm, ang, tag = sc.ray(1)
        dw, _, _ = wl.ray(1)
        mmm = fw.fw_adc_to_mm(ir_adc(dm, 0, None))
        mmw = fw.fw_adc_to_mm(ir_adc(dw, 0, None))
        lab = {'lost': '아무것도 없음', 'ghost': '허상(너무 멀게)', 'ok': '정상'}[tag]
        print(f"{yaw_deg:>7}°  {('%.1f' % ang) if ang is not None else '   -':>5}  "
              f"{dw*1000:>6.0f}mm  {mmw:>6}mm  {mmm:>6}mm  {lab}")
        rows.append((yaw_deg, ang, mmw, mmm, tag))
    return rows


if __name__ == '__main__':
    mode = sys.argv[1] if len(sys.argv) > 1 else 'gif'
    if mode == 'sweep':
        sweep(); raise SystemExit
    DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 22.0
    OUT = sys.argv[3] if len(sys.argv) > 3 else 'runs/mirror.gif'
    A = Sim(False, '불투명 벽  (확산 반사)', (30,70,160), (0.13,0.42,0.92), 1)
    B = Sim(True,  '거울  (정반사)',         (190,40,30), (0.85,0.20,0.15), 2)
    spt = int(round((TICK_MS/1000)/PHYS_DT)); ge = max(1,int(round((1/FPS)/PHYS_DT)))
    frames = []
    for k in range(int(DUR/PHYS_DT)):
        t = k*PHYS_DT
        if k % spt == 0: A.control(t); B.control(t)
        A.step(); B.step()
        if k % ge == 0:
            frames.append(np.concatenate([A.frame(t), np.full((H+HDR,6,3),235,np.uint8), B.frame(t)],axis=1))
    pil=[Image.fromarray(f) for f in frames]
    picks=sorted({0,len(pil)//3,2*len(pil)//3,len(pil)-1}); w0,h0=pil[0].size
    st=Image.new('RGB',(w0,h0*len(picks)))
    for i,kk in enumerate(picks): st.paste(pil[kk],(0,i*h0))
    mst=st.quantize(colors=64, method=Image.MEDIANCUT)
    pil=[f.quantize(palette=mst, dither=Image.NONE) for f in pil]
    p=os.path.join(HERE,OUT); os.makedirs(os.path.dirname(p),exist_ok=True)
    pil[0].save(p, save_all=True, append_images=pil[1:], duration=int(1000/FPS), loop=0,
                optimize=True, disposal=2)
    print(f'saved {OUT}  {len(pil)} frames  {w0}x{h0}')
    for s in (A,B):
        print(f'  {s.label:<22} 판 충돌 {s.hitp}회  첫 충돌 {s.first}  밀착 {s.press:.1f}s  '
              f'이동 {s.plen:.2f} m  엔코더 누적 {s.enc_sum}  stall {fw.fw_i_stalls(s.slot)}')
