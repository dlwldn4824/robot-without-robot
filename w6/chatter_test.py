"""과제 6-C 집중 실험: 경계값 근처에서 노이즈가 행동을 흔드는가?

실제 거리를 threshold 바로 근처(0.72 m)에 고정해 두고
노이즈만 섞어 컨트롤러를 반복 호출한다. 물리는 끄고 '판단'만 본다.
"""
import random, json, os
from controller import decide_action
from controller_improved import ImprovedController

TRUE_D, TH, N = 0.72, 0.70, 200
HERE = os.path.dirname(os.path.abspath(__file__))


def flips(seq):
    return sum(1 for a, b in zip(seq, seq[1:]) if a != b)


def run(noise, use_improved, seed=0):
    rng = random.Random(seed)
    imp = ImprovedController(enter_th=TH, exit_th=TH + 0.3, hold_s=0.45, avg_n=5)
    acts = []
    for k in range(N):
        t = k * 0.05
        d = TRUE_D + (rng.uniform(-noise, noise) if noise else 0.0)
        acts.append(imp.decide(2.0, d, 2.0, t) if use_improved
                    else decide_action(2.0, d, 2.0, TH))
    return acts


rows = []
print(f'실제 전방거리 {TRUE_D} m, threshold {TH} m, {N}회 판단 (20 Hz = 10초)\n')
print(f"{'컨트롤러':<10}{'노이즈':>8}{'행동 변경':>12}{'초당':>8}   첫 20회")
for noise in (0.0, 0.05, 0.10, 0.15):
    for name, imp in (('기본', False), ('개선', True)):
        acts = run(noise, imp); f = flips(acts)
        head = ' '.join('F' if a == 'FORWARD' else ('S' if a == 'SLOW' else 'T') for a in acts[:20])
        print(f'{name:<10}{noise:>8.2f}{f:>12}{f/10:>8.1f}   {head}')
        rows.append(dict(controller=name, noise=noise, flips=f, per_sec=f/10))
json.dump(rows, open(os.path.join(HERE,'runs','chatter.json'),'w'), ensure_ascii=False, indent=1)
print('\n(F=FORWARD, S=SLOW, T=TURN)')
