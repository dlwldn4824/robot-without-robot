"""과제 6 실패 실험 + 과제 7 개선 비교를 한 번에 돌린다."""
import json, subprocess, sys, os

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = [
    ('0  기본 th=0.7 (기준)',  ['--controller','basic','--threshold','0.7']),
    ('A  th=0.1 너무 작음',    ['--controller','basic','--threshold','0.1']),
    ('A+ th=0.1 + 속도 1.5배', ['--controller','basic','--threshold','0.1','--speed','1.5']),
    ('B  th=3.0 너무 큼',      ['--controller','basic','--threshold','3.0']),
    ('C  노이즈 ±0.15 m',      ['--controller','basic','--threshold','0.7','--noise','0.15']),
    ('C+ 노이즈 ±0.25 m',      ['--controller','basic','--threshold','0.7','--noise','0.25']),
    ('7  개선 컨트롤러',       ['--controller','improved','--threshold','0.7']),
    ('7a 개선 + 노이즈 ±0.15', ['--controller','improved','--threshold','0.7','--noise','0.15']),
    ('7b 개선 + 노이즈 ±0.25', ['--controller','improved','--threshold','0.7','--noise','0.25']),
]

out = []
for label, extra in RUNS:
    tag = label.split()[0]
    cmd = [sys.executable, os.path.join(HERE,'run.py'), '--duration','40','--seed','1'] + extra \
          + ['--log', os.path.join(HERE,'runs',f'exp_{tag}.json')]
    r = json.loads(subprocess.run(cmd, capture_output=True, text=True, cwd=HERE).stdout)
    r['label'] = label
    out.append(r)
    print(f"{label:<28} 충돌={r['collisions']:>3}  이동={r['path_len']:>6.2f}m  "
          f"회전비율={r['turn_ratio']:.3f}  행동변경={r['action_changes']:>3}  "
          f"최소전방={r['min_front']:.2f}m")

json.dump(out, open(os.path.join(HERE,'runs','experiments.json'),'w'), ensure_ascii=False, indent=1)
print('\nsaved runs/experiments.json')
