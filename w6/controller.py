"""W6 Controller — 센서값을 받아 행동을 결정한다.

이 파일만 고치면 로봇의 성격이 바뀐다.
run.py 는 이 안의 decide_action() / action_to_wheels() 만 호출한다.
"""

# ────────────────────────────────── 과제 1. 바퀴 명령표
# 값은 -1.0 ~ +1.0 로 정규화한 바퀴 속도. run.py 가 MAX_WHEEL_SPEED 를 곱한다.
WHEELS = {
    'FORWARD':    ( 1.0,  1.0),   # 두 바퀴 같은 속도  -> 직진
    'SLOW':       ( 0.35, 0.35),  # 같은 속도, 느리게  -> 천천히 직진
    'TURN_LEFT':  (-0.6,  0.6),   # 좌 뒤 / 우 앞      -> 제자리 좌회전
    'TURN_RIGHT': ( 0.6, -0.6),   # 좌 앞 / 우 뒤      -> 제자리 우회전
    'ARC_LEFT':   ( 0.25, 0.8),   # 속도 차            -> 왼쪽으로 호를 그리며 전진
    'ARC_RIGHT':  ( 0.8,  0.25),
    'BACK':       (-0.5, -0.5),
    'STOP':       ( 0.0,  0.0),
}


def action_to_wheels(action):
    """행동 이름 -> (left, right) 정규화 바퀴 속도"""
    return WHEELS.get(action, (0.0, 0.0))


# ────────────────────────────────── 과제 3. Threshold 로 상태 판단
DANGER_TH = 0.50   # [m] 이보다 가까우면 위험
SLOW_TH   = 1.00   # [m] 이보다 가까우면 감속


def check_state(distance):
    """전방 거리 하나로 상태를 판단한다."""
    if distance < DANGER_TH:
        return 'DANGER'
    elif distance < SLOW_TH:
        return 'SLOW'
    else:
        return 'SAFE'


# ────────────────────────────────── 과제 5. 기본 회피 컨트롤러
def decide_action(left, front, right, th=0.7):
    """Left / Front / Right 거리만 보고 행동을 고른다.  (상태 없음 = 순수 함수)

    Case 1  앞이 비었다            -> FORWARD
    Case 2  앞이 막혔다            -> 더 넓은 쪽으로 회전
    Case 3  세 방향 모두 좁다      -> 제자리에서 돌아 빠져나온다 (U-TURN)
    """
    blocked_f = front < th
    blocked_l = left  < th
    blocked_r = right < th

    # Case 3: 삼면이 막힘 -> 한쪽으로 계속 돌아 탈출구를 찾는다 (STOP 하면 영원히 갇힌다)
    if blocked_f and blocked_l and blocked_r:
        return 'TURN_LEFT'

    # Case 1: 앞이 비었다
    if not blocked_f:
        return 'FORWARD'

    # Case 2: 앞이 막혔다 -> 넓은 쪽으로
    return 'TURN_LEFT' if left > right else 'TURN_RIGHT'
