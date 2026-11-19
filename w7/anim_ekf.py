"""REC 02 — EKF-SLAM 이 도는 동안 세 경로와 오차가 어떻게 벌어지는가."""
import numpy as np
from PIL import Image
from common import *
from ekf_run import run, RFID

FPS, STEP = 10, 5
r = run()
T, D, X, H, t = r['T'], r['D'], r['X'], r['hist'], r['t']
edr, esl = r['err_dr'], r['err_sl']

fig, ax = plt.subplots(1, 2, figsize=(7.6, 3.75), dpi=100,
                       gridspec_kw={'width_ratios': [1.05, 1]})
fig.subplots_adjust(left=.085, right=.975, top=.80, bottom=.145, wspace=.30)
frames = []
for k in range(1, len(t) + STEP, STEP):
    k = min(k, len(t) - 1)
    for a in ax: a.clear()
    a0, a1 = ax

    a0.plot(RFID[:, 0], RFID[:, 1], '*', color='#111827', ms=13, zorder=7)
    a0.plot(T[0, :k], T[1, :k], '-', color=GREEN, lw=2.2, zorder=4)
    a0.plot(D[0, :k], D[1, :k], '-', color=RED, lw=1.7, zorder=3)
    a0.plot(X[0, :k], X[1, :k], '-', color=BLUE, lw=1.7, zorder=5)
    a0.plot(T[0, k], T[1, k], 'o', color=GREEN, ms=6, zorder=8)
    a0.plot(D[0, k], D[1, k], 'o', color=RED, ms=5, zorder=8)
    xe = H[k]
    if len(xe) > 3:
        lm = xe[3:].reshape(-1, 2)
        a0.plot(lm[:, 0], lm[:, 1], 'x', color=BLUE, ms=9, mew=2.1, zorder=9)
    a0.set_xlim(-13, 19); a0.set_ylim(-5, 23); a0.set_aspect('equal')
    a0.grid(alpha=.22); a0.tick_params(labelsize=8)
    a0.set_title('위에서 본 주행', fontsize=9.5, pad=3)

    a1.plot(t[:k], edr[:k], color=RED, lw=2.0)
    a1.plot(t[:k], esl[:k], color=BLUE, lw=2.0)
    a1.set_xlim(0, 50); a1.set_ylim(0, 6.6); a1.grid(alpha=.22)
    a1.tick_params(labelsize=8); a1.set_xlabel('시간 [s]', fontsize=8.5)
    a1.set_ylabel('위치 오차 [m]', fontsize=8.5)
    a1.set_title('실제 위치와 얼마나 벌어졌나', fontsize=9.5, pad=3)
    for s_ in ('top', 'right'): a1.spines[s_].set_visible(False)

    nlm = int((len(xe) - 3) / 2)
    fig.suptitle(
        f't={t[k]:4.1f}s    보이는 랜드마크 {r["nz"][k]}개 · 지도에 등록 {nlm}개      '
        f'오차  Odometry {edr[k]:4.2f} m   SLAM {esl[k]:4.2f} m',
        fontsize=10, y=0.955, color='#243244')
    fig.canvas.draw()
    frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()))

frames += [frames[-1]] * 14
w, h = frames[0].size
pal = frames[-1].quantize(colors=48, method=Image.MEDIANCUT)
frames = [f.quantize(palette=pal, dither=Image.NONE) for f in frames]
out = os.path.join(RUNS, 'rec02_ekf.gif')
frames[0].save(out, save_all=True, append_images=frames[1:],
               duration=int(1000/FPS), loop=0, optimize=True, disposal=2)
print(f'{out}  {len(frames)} frames  {w}x{h}')
