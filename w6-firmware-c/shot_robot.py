"""로봇 소개용 고해상도 스틸 렌더. 영상 프레임을 확대하지 않는다."""
import math, os, sys
import numpy as np, mujoco
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
OUT  = sys.argv[1] if len(sys.argv) > 1 else 'runs/robot.jpg'
W, H = 1760, 1180

m = mujoco.MjModel.from_xml_path(os.path.join(HERE, 'arena_hires.xml'))
d = mujoco.MjData(m)

# obs1 (0.9, 0.7) 을 비스듬히 바라보는 위치에 세운다
x, y, yaw = -1.05, -1.50, math.radians(0)
d.qpos[0:3] = (x, y, d.qpos[2])
d.qpos[3:7] = (math.cos(yaw/2), 0, 0, math.sin(yaw/2))
mujoco.mj_forward(m, d)
print('센서 거리 L/F/R =', [round(float(v), 3) for v in d.sensordata[:3]])

r = mujoco.Renderer(m, H, W)
opt = mujoco.MjvOption(); mujoco.mjv_defaultOption(opt)
opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = 1   # 거리센서 광선 표시
r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 1

cam = mujoco.MjvCamera(); mujoco.mjv_defaultCamera(cam)
cam.azimuth, cam.elevation, cam.distance = 158, -27, 1.30
cam.lookat[:] = [x + 0.16, y + 0.01, 0.045]

r.update_scene(d, camera=cam, scene_option=opt)
img = Image.fromarray(r.render()).crop((230, 215, 1745, 1175))   # 빈 여백 정리
p = os.path.join(HERE, OUT); os.makedirs(os.path.dirname(p), exist_ok=True)
img.convert('RGB').save(p, quality=94, subsampling=0)
print('saved', OUT, img.size)
