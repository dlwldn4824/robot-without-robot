# W6. Sensor + Control — MuJoCo 장애물 회피

```bash
pip install mujoco imageio
python run.py --duration 40                        # 기본 컨트롤러 (충돌 61회)
python run.py --controller improved --duration 40  # 개선 컨트롤러 (충돌 0회)
python experiments.py                              # 과제 6 실험 9종
python chatter_test.py                             # 채터링 분리 측정
python make_gif.py 30 0.0 runs/compare.gif         # 비교 GIF
python plots.py                                    # 결과 그림 3장
```

| 파일 | 역할 |
| --- | --- |
| `robot.xml` / `world.xml` | 로봇·월드 (수정 불필요) |
| `run.py` | 시뮬레이션 루프 |
| **`controller.py`** | **여기를 고치면 로봇 성격이 바뀐다** |
| `controller_improved.py` | 과제 7 개선본 |
| `W6-sensor-control.md` | 정리 노트 (노션용) |
