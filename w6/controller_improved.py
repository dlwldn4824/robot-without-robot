"""W6 과제 7 — 개선한 Controller.

기본 컨트롤러(controller.py)의 문제 3가지를 고쳤다.

  문제 1. 측면을 보지 않는다
     Case 1 이 "Front > th -> FORWARD" 라서, 옆 벽이 4 cm 까지 붙어도 직진한다.
     -> 측면 여유가 부족하면 반대쪽으로 호를 그리며 비켜간다 (side clearance)

  문제 2. 노이즈에 떤다 (chattering)
     경계값 근처에서 FORWARD/TURN 이 매 스텝 뒤집힌다.
     -> ① 최근 N개 평균  ② 진입/해제 기준을 다르게 (히스테리시스)

  문제 3. 회전이 너무 짧게 끊긴다
     한 스텝 돌고 바로 FORWARD 로 돌아와 같은 장애물에 다시 부딪힌다.
     -> 한 번 돌기 시작하면 최소 hold_s 초 동안 유지 (turn commitment)
"""
from collections import deque


class ImprovedController:
    def __init__(self, enter_th=0.7, exit_th=1.0, hold_s=0.45, avg_n=5,
                 side_th=0.35):
        self.enter_th = enter_th     # 이 거리 아래면 '막힘' 으로 진입
        self.exit_th = exit_th       # 이 거리 위로 올라가야 '뚫림' 으로 해제
        self.hold_s = hold_s         # 회전 최소 유지 시간
        self.avg_n = avg_n           # 이동평균 길이
        self.side_th = side_th       # 측면 최소 여유

        self.buf = deque(maxlen=avg_n)
        self.blocked = False         # 히스테리시스 상태
        self.turn_dir = None         # 'TURN_LEFT' / 'TURN_RIGHT'
        self.turn_until = -1.0

    # ---------------------------------------------------------------
    def decide(self, left, front, right, t):
        # ① 이동평균 — 노이즈를 눌러준다
        self.buf.append((left, front, right))
        n = len(self.buf)
        L = sum(b[0] for b in self.buf) / n
        F = sum(b[1] for b in self.buf) / n
        R = sum(b[2] for b in self.buf) / n

        # ③ 회전 커밋: 아직 유지 시간이 안 지났으면 계속 돈다
        if self.turn_dir is not None and t < self.turn_until:
            # 단, 앞이 충분히 뚫렸으면 일찍 빠져나온다
            if F < self.exit_th:
                return self.turn_dir
            self.turn_dir = None

        # ② 히스테리시스: 진입 기준과 해제 기준이 다르다
        if self.blocked:
            if F > self.exit_th:
                self.blocked = False
        else:
            if F < self.enter_th:
                self.blocked = True

        if self.blocked:
            self.turn_dir = 'TURN_LEFT' if L > R else 'TURN_RIGHT'
            self.turn_until = t + self.hold_s
            return self.turn_dir

        # ① 측면 여유 확인 — 기본 컨트롤러가 놓치던 부분
        if R < self.side_th and R < L:
            return 'ARC_LEFT'        # 오른쪽이 붙었다 -> 왼쪽으로 비켜 전진
        if L < self.side_th and L < R:
            return 'ARC_RIGHT'

        # 앞이 적당히 비었으면 속도를 줄여 여유를 둔다
        if F < self.exit_th:
            return 'SLOW'
        return 'FORWARD'
