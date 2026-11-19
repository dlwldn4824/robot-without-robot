"""실습 B 그림 — 세 경로 비교와 오차가 쌓이는 모양."""
import numpy as np
from common import *
from ekf_run import run, RFID

base = run()
none = run(rfid=np.array([[200.0, 200.0]]))

# ── FIG 04 · 세 경로 ───────────────────────────────────────
fig, ax = plt.subplots(1, 2, figsize=(10.4, 4.6))
for a, r, ttl in ((ax[0], base, '랜드마크 4개를 볼 수 있을 때'),
                  (ax[1], none, '랜드마크를 하나도 못 볼 때')):
    a.plot(r['T'][0], r['T'][1], '-', color=GREEN, lw=2.4, label='실제 경로', zorder=4)
    a.plot(r['D'][0], r['D'][1], '-', color=RED, lw=1.9, label='Odometry 추정', zorder=3)
    same = abs(r['err_sl'][-1] - r['err_dr'][-1]) < 1e-9      # 두 선이 완전히 겹치는 경우
    a.plot(r['X'][0], r['X'][1], '--' if same else '-', color=BLUE,
           lw=1.9, dashes=(5, 4) if same else (None, None), label='SLAM 추정', zorder=5)
    if same:
        a.text(.5, .035, '빨간 선과 파란 선이 완전히 겹칩니다', transform=a.transAxes,
               ha='center', fontsize=9.5, color=RED)
    if len(r['last']) > 3:
        est = r['last'][3:].reshape(-1, 2)
        a.plot(est[:, 0], est[:, 1], 'x', color=BLUE, ms=11, mew=2.4,
               label='SLAM이 찍은 랜드마크', zorder=6)
    a.plot(RFID[:, 0], RFID[:, 1], '*', color='#111827', ms=15,
           label='랜드마크 실제 위치', zorder=7)
    a.plot(0, 0, 'o', color='#111827', ms=7, zorder=8)
    a.annotate('출발', (0, 0), (-3.4, -2.6), fontsize=9, color=GRAY)
    a.set_title(f'{ttl}\n최종 오차  Odometry {r["err_dr"][-1]:.2f} m   ·   '
                f'SLAM {r["err_sl"][-1]:.2f} m', fontsize=11, pad=8)
    a.set_aspect('equal'); a.grid(alpha=.25); a.set_xlabel('x [m]'); a.set_ylabel('y [m]')
    for s in ('top', 'right'): a.spines[s].set_visible(False)
h, l = ax[0].get_legend_handles_labels()
fig.legend(h, l, fontsize=9.5, loc='lower center', ncol=5,
           frameon=False, bbox_to_anchor=(0.5, -0.045))
fig.tight_layout()
fig.savefig(os.path.join(RUNS, 'fig_paths.png'), dpi=150, bbox_inches='tight')
print('fig_paths.png')

# ── FIG 05 · 오차가 시간에 따라 어떻게 쌓이는가 ─────────────
fig, a = plt.subplots(figsize=(8.6, 4.0))
a.plot(base['t'], base['err_dr'], color=RED, lw=2.2, label='Odometry — 계속 쌓인다')
a.plot(base['t'], base['err_sl'], color=BLUE, lw=2.2, label='SLAM — 일정 수준에서 잡힌다')
a.plot(none['t'], none['err_sl'], color=GRAY, lw=1.6, ls='--',
       label='랜드마크를 못 볼 때의 SLAM — Odometry와 똑같아진다')
a.fill_between(base['t'], base['err_sl'], base['err_dr'],
               color=RED, alpha=.07)
a.annotate(f'{base["err_dr"][-1]:.2f} m', (base['t'][-1], base['err_dr'][-1]),
           (-46, 6), textcoords='offset points', color=RED, fontsize=10, weight='bold')
a.annotate(f'{base["err_sl"][-1]:.2f} m', (base['t'][-1], base['err_sl'][-1]),
           (-46, 8), textcoords='offset points', color=BLUE, fontsize=10, weight='bold')
a.set_xlabel('시간 [s]'); a.set_ylabel('실제 위치와의 거리 오차 [m]')
a.set_title('같은 주행, 같은 센서 — 보정이 있느냐 없느냐의 차이', fontsize=11, pad=8)
a.legend(fontsize=9.5, loc='upper left'); a.grid(alpha=.25); a.set_xlim(0, 50)
for s in ('top', 'right'): a.spines[s].set_visible(False)
fig.tight_layout()
fig.savefig(os.path.join(RUNS, 'fig_error.png'), dpi=150, bbox_inches='tight')
print('fig_error.png')

for nm, r in (('기본', base), ('랜드마크 없음', none)):
    print(f"{nm:<8} 최종 DR {r['err_dr'][-1]:5.2f} / SLAM {r['err_sl'][-1]:5.2f}  "
          f"평균 {r['err_dr'].mean():5.2f} / {r['err_sl'].mean():5.2f}  "
          f"관측 가능한 시간 비율 {100*(r['nz']>0).mean():.0f}%")
