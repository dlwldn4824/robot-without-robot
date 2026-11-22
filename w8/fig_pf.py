"""W8 그림 — 파티클 수 비교, 납치된 로봇, 시작 위치를 모를 때, 그리고 GIF."""
import math, time
import numpy as np
import imageio.v2 as imageio
from common import *
import pf_run
from pf_run import run, STEPS, RFID
pf = pf_run.pf


def run_snap(n, seed, kidnap_at=None, spread=None, every=2, recover=False):
    """파티클 구름을 매 스텝 기록하면서 돌린다."""
    np.random.seed(seed); pf_run.set_np(n)
    x_true = np.zeros((4, 1)); x_dr = np.zeros((4, 1))
    px = np.zeros((4, n))
    if spread is not None:
        px[0, :] = np.random.uniform(-spread, spread, n)
        px[1, :] = np.random.uniform(-spread, spread, n)
        px[2, :] = np.random.uniform(-math.pi, math.pi, n)
    pw = np.zeros((1, n)) + 1.0 / n
    snaps, T, E, D, S = [], [], [], [], []
    t = 0.0
    for k in range(STEPS):
        t += pf.DT
        u = pf.calc_input()
        x_true, z, x_dr, ud = pf.observation(x_true, x_dr, u, RFID)
        if kidnap_at is not None and abs(t - kidnap_at) < pf.DT / 2:
            x_true[0, 0] += 6.0; x_true[1, 0] += 6.0
        xe, P, px, pw = pf.pf_localization(px, pw, z, ud)
        bad = not np.all(np.isfinite(pw))
        if bad and recover:
            pw = np.zeros((1, n)) + 1.0 / n
        if k % every == 0:
            snaps.append((t, px[:2, :].copy(),
                          (x_true[0, 0], x_true[1, 0]),
                          (xe[0, 0], xe[1, 0]),
                          (x_dr[0, 0], x_dr[1, 0]), bad))
        T.append(t)
        E.append(math.hypot(xe[0, 0] - x_true[0, 0], xe[1, 0] - x_true[1, 0]))
        D.append(math.hypot(x_dr[0, 0] - x_true[0, 0], x_dr[1, 0] - x_true[1, 0]))
        S.append((np.std(px[0, :]) + np.std(px[1, :])) / 2)
    return dict(snaps=snaps, t=np.array(T), e=np.array(E), dr=np.array(D), s=np.array(S))


def arena(ax, lim=((-12, 22), (-7, 27))):
    ax.plot(RFID[:, 0], RFID[:, 1], '*', color=ORANGE, ms=15, zorder=6,
            markeredgecolor='#9a6a16', markeredgewidth=.6)
    ax.set_xlim(*lim[0]); ax.set_ylim(*lim[1]); ax.set_aspect('equal')
    ax.set_xticks([]); ax.set_yticks([]); ax.grid(alpha=.2, lw=.5)
    for sp in ax.spines.values(): sp.set_color('#d8d8d5')


# ── FIG 01 · 파티클 수 ────────────────────────────────────
print('FIG 01 ...')
NS = (20, 100, 500, 2000)
SEEDS = range(20)
res = {}
for n in NS:
    rs = [run(n, sd) for sd in SEEDS]
    rm = np.array([r['rmse'] for r in rs])
    res[n] = dict(rmse=rm, ms=float(np.median([r['ms'] for r in rs])),
                  fail=int((rm > 1.0).sum()), total=len(rm))
    print(f"   NP={n:>4} 중앙 {np.median(rm):.3f}m  최악 {rm.max():.2f}m  "
          f"1m초과 {res[n]['fail']}/{len(rm)}  {res[n]['ms']:.2f}ms")

np.savez(os.path.join(RUNS, 'sweep.npz'),
         **{f'rmse{n}': res[n]['rmse'] for n in NS},
         ms=np.array([res[n]['ms'] for n in NS]), ns=np.array(NS))

fig, ax = plt.subplots(1, 2, figsize=(12.0, 4.2))
xs = np.arange(len(NS))
med = [float(np.median(res[n]['rmse'])) for n in NS]
p90 = [float(np.percentile(res[n]['rmse'], 90)) for n in NS]
mx = [float(res[n]['rmse'].max()) for n in NS]

for i, n in enumerate(NS):                       # 시드별 점을 다 찍는다
    ax[0].plot(np.full(len(res[n]['rmse']), xs[i]), res[n]['rmse'], '.',
               color=BLUE, ms=5, alpha=.35)
ax[0].plot(xs, med, 'o-', color=BLUE, lw=2, ms=9, zorder=5, label='중앙값')
ax[0].plot(xs, mx, 'v', color=RED, ms=7, zorder=5, label='최악')
ax[0].axhline(1.0, color=GRAY, ls=':', lw=1.2)
ax[0].text(-0.33, 1.08, '1 m', fontsize=9, color=GRAY)
for i, n in enumerate(NS):
    ax[0].annotate(f'{med[i]:.3f}', (xs[i], med[i]), textcoords='offset points',
                   xytext=(0, 13), ha='center', fontsize=9.5, color=BLUE)
    if res[n]['fail']:
        ax[0].annotate(f"{res[n]['fail']}/{res[n]['total']}회 실패",
                       (xs[i], mx[i]), textcoords='offset points',
                       xytext=(6, 2), fontsize=9.5, color=RED, ha='left')
ax[0].set_yscale('log')
# 로그 지수의 마이너스 기호가 한글 폰트에 없어 눈금을 직접 적는다
from matplotlib.ticker import FixedLocator, FixedFormatter, NullLocator
_tk = [0.1, 0.2, 0.5, 1, 2, 5, 10]
ax[0].yaxis.set_major_locator(FixedLocator(_tk))
ax[0].yaxis.set_major_formatter(FixedFormatter(['0.1', '0.2', '0.5', '1', '2', '5', '10']))
ax[0].yaxis.set_minor_locator(NullLocator())
ax[0].set_xticks(xs); ax[0].set_xticklabels([str(n) for n in NS])
ax[0].set_xlim(-.45, len(NS) - .25)
ax[0].set_ylim(0.09, 14)
ax[0].set_xlabel('파티클 수'); ax[0].set_ylabel('위치 오차 RMSE (m, 로그)')
ax[0].set_title('20개는 정확도가 아니라 신뢰성이 문제다', fontsize=11.5, pad=8)
ax[0].legend(fontsize=9, loc='lower left'); ax[0].grid(alpha=.25, lw=.6, which='both')
for sp in ('top', 'right'): ax[0].spines[sp].set_visible(False)

ms = [res[n]['ms'] for n in NS]
ax[1].bar(xs, ms, width=.5, color=RED, alpha=.85)
for i, n in enumerate(NS):
    ax[1].annotate(f'{ms[i]:.2f} ms', (xs[i], ms[i]), textcoords='offset points',
                   xytext=(0, 5), ha='center', fontsize=9.5, color=RED)
ax[1].axhline(100.0, color=GREEN, ls='--', lw=1.3)
ax[1].text(-.35, 103, '10 Hz 제어 주기 한계 (100 ms)', fontsize=9, color=GREEN)
ax[1].set_xticks(xs); ax[1].set_xticklabels([str(n) for n in NS])
ax[1].set_xlabel('파티클 수'); ax[1].set_ylabel('한 스텝 계산 시간 (ms)')
ax[1].set_title('계산량은 파티클 수에 그대로 비례한다', fontsize=11.5, pad=8)
ax[1].set_ylim(0, 115); ax[1].grid(alpha=.25, lw=.6, axis='y')
for sp in ('top', 'right'): ax[1].spines[sp].set_visible(False)
fig.tight_layout()
fig.savefig(os.path.join(RUNS, 'fig_particles.png'), dpi=150, bbox_inches='tight')

# ── FIG 02 · 납치된 로봇 ──────────────────────────────────
print('FIG 02 ...')
raw = run_snap(100, 0, kidnap_at=25.0, every=2, recover=False)
fix = run_snap(100, 0, kidnap_at=25.0, every=2, recover=True)

fig, ax = plt.subplots(1, 2, figsize=(12.4, 4.3),
                       gridspec_kw={'width_ratios': [1, 1.3]})
arena(ax[0])
tr = np.array([s_[2] for s_ in fix['snaps']]); es = np.array([s_[3] for s_ in fix['snaps']])
k0 = next(i for i, s_ in enumerate(fix['snaps']) if s_[0] > 25.0)
ax[0].plot(tr[:k0, 0], tr[:k0, 1], '-', color=GREEN, lw=2.2, label='실제 경로')
ax[0].plot(tr[k0:, 0], tr[k0:, 1], '-', color=GREEN, lw=2.2, alpha=.45)
ax[0].plot(es[:k0, 0], es[:k0, 1], '-', color=BLUE, lw=1.7, label='필터 추정')
ax[0].annotate('', xy=tr[k0], xytext=tr[k0 - 1],
               arrowprops=dict(arrowstyle='-|>', color=RED, lw=2.6))
ax[0].plot(*tr[k0], 'o', color=RED, ms=10, zorder=7)
ax[0].text(tr[k0][0] + .9, tr[k0][1] + 1.0, '25초\n여기로 들어 옮김',
           fontsize=9.5, color=RED, va='bottom')
ax[0].legend(fontsize=9, loc='lower left', framealpha=.9)
ax[0].set_title('위에서 본 주행', fontsize=11.5, pad=8)

ax[1].plot(raw['t'], raw['dr'], color=GRAY, lw=1.3, alpha=.8, label='바퀴 정보만')
ax[1].plot(raw['t'], raw['e'], color=RED, lw=2.2, label='예제 그대로')
ax[1].plot(fix['t'], fix['e'], color=BLUE, lw=2.0, label='가중치 초기화 한 줄 추가')
ax[1].axvline(25.0, color=RED, ls='--', lw=1.3)
ax[1].text(25.5, 19, '납치', color=RED, fontsize=10)
gap = raw['t'][~np.isfinite(raw['e'])]
if len(gap):
    ax[1].axvspan(gap.min(), raw['t'][-1], color=RED, alpha=.07)
    ax[1].text((gap.min() + raw['t'][-1]) / 2, 10.5,
               '예제 그대로는 여기서부터\n계산이 NaN 이 되어 끊긴다',
               fontsize=9.5, color=RED, ha='center')
ax[1].set_xlabel('시간 (s)'); ax[1].set_ylabel('위치 오차 (m)')
ax[1].set_title('납치를 당하면 스스로는 못 돌아온다', fontsize=11.5, pad=8)
ax[1].set_ylim(-1, 23)
ax[1].legend(fontsize=9, loc='upper left'); ax[1].grid(alpha=.25, lw=.6)
for sp in ('top', 'right'): ax[1].spines[sp].set_visible(False)
fig.tight_layout()
fig.savefig(os.path.join(RUNS, 'fig_kidnap.png'), dpi=150, bbox_inches='tight')

# ── FIG 03 · 시작 위치를 모를 때 ──────────────────────────
print('FIG 03 ...')
glb = run_snap(2000, 0, spread=20.0, every=1)
want = [0.0, 0.1, 0.5, 5.0]
fig, ax = plt.subplots(1, 4, figsize=(15.4, 3.9))
for a, tw in zip(ax, want):
    if tw == 0.0:
        np.random.seed(0)
        pxx = np.random.uniform(-20, 20, 2000); pyy = np.random.uniform(-20, 20, 2000)
        tp = (0.0, 0.0); sp = 20.0
    else:
        i = min(range(len(glb['snaps'])), key=lambda j: abs(glb['snaps'][j][0] - tw))
        s = glb['snaps'][i]; pxx, pyy = s[1][0], s[1][1]; tp = s[2]
        sp = glb['s'][int(round(s[0] / pf.DT)) - 1]
    arena(a, lim=((-24, 24), (-24, 24)))
    a.plot(pxx, pyy, '.', color=BLUE, ms=2.6, alpha=.5)
    a.plot(*tp, 'o', color=RED, ms=9, zorder=7)
    a.set_title(f'{tw:.1f} 초', fontsize=11.5, pad=6)
    a.text(0.5, -0.045, f'흩어짐 {sp:.2f} m', transform=a.transAxes,
           ha='center', va='top', fontsize=9.5, color=GRAY)
fig.tight_layout()
fig.savefig(os.path.join(RUNS, 'fig_global.png'), dpi=150, bbox_inches='tight')

# ── GIF · 추정 과정 ───────────────────────────────────────
print('GIF ...')
frames = []
for (t, p, tp, ep, dp, bad) in fix['snaps'][::2]:
    fig, a = plt.subplots(figsize=(5.2, 4.6))
    arena(a)
    a.plot(p[0], p[1], '.', color=BLUE, ms=2.4, alpha=.5)
    a.plot(*dp, 's', color=GRAY, ms=7, alpha=.75)
    a.plot(*ep, 'o', color=BLUE, ms=9)
    a.plot(*tp, 'o', color=RED, ms=9)
    err = math.hypot(ep[0] - tp[0], ep[1] - tp[1])
    a.set_title(f'{t:4.1f} s   오차 {err:5.2f} m', fontsize=12.5, pad=8)
    if t > 25.0:
        a.text(.03, .97, '납치됨', transform=a.transAxes, va='top',
               fontsize=12, color=RED, weight='bold')
    a.text(.03, .03, '● 실제  ● 추정  ■ 바퀴만  · 파티클  ★ 랜드마크',
           transform=a.transAxes, fontsize=8, color=GRAY)
    fig.tight_layout()
    fig.canvas.draw()
    frames.append(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy())
    plt.close(fig)
imageio.mimsave(os.path.join(RUNS, 'rec_pf.gif'), frames, fps=12, loop=0)

print('완료:', ', '.join(sorted(f for f in os.listdir(RUNS) if f.endswith(('.png', '.gif')))))
