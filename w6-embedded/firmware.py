"""firmware.py — MCU 펌웨어.

Arduino 처럼 setup() 한 번, loop() 를 20 ms 마다 호출한다.
규칙:
  * float 를 쓰지 않는다. 거리는 mm(정수), 각도는 deg(정수), 시간은 ms(정수)
  * 동적 할당 없음, 반복 횟수는 전부 상수로 묶여 있음
  * 출력은 모터 드라이버가 받는 (방향, 듀티) 뿐
"""
from hal import adc_to_mm, PWM_MAX, clamp

TICK_MS = 20                 # 50 Hz

# ---- 튜닝 상수 (전부 정수) -------------------------------------
STOP_MM      = 420           # 이보다 가까우면 '막힘'
CLEAR_MM     = 700           # 이보다 멀어야 '뚫림'  (히스테리시스)
SIDE_MM      = 380           # 측면 최소 여유
TURN_HOLD_MS = 360           # 한 번 돌기 시작하면 최소 유지 시간
ARRIVE_ADC   = 760           # 비컨이 이보다 밝으면 도착으로 본다
STALL_TICKS  = 3             # 이 수 이하로만 움직이면 '안 굴러간 것'
STALL_MS     = 500           # 이 시간 동안 계속 그러면 끼인 것으로 판정
BACK_MS      = 500           # 후진 시간
SPIN_MS      = 550           # 후진 뒤 제자리 회전 시간
CRUISE_PWM   = 210
SLOW_PWM     = 110
TURN_PWM     = 150
STEER_GAIN   = 95            # 비컨 좌우차(정규화) -> 듀티차
FILT_SHIFT   = 2             # 이동평균 대신 1/4 씩 섞는 1차 필터 (시프트 연산)


class Motors:
    """모터 드라이버에 그대로 나가는 값."""
    __slots__ = ('l_dir', 'l_duty', 'r_dir', 'r_duty')

    def __init__(self, l_dir=1, l_duty=0, r_dir=1, r_duty=0):
        self.l_dir, self.l_duty = l_dir, l_duty
        self.r_dir, self.r_duty = r_dir, r_duty

    def __repr__(self):
        sl = '+' if self.l_dir > 0 else '-'
        sr = '+' if self.r_dir > 0 else '-'
        return f'L{sl}{self.l_duty:3d} R{sr}{self.r_duty:3d}'


class Firmware:
    # 상태 기계
    BOOT, CRUISE, SLOW, AVOID_L, AVOID_R, ESCAPE, ARRIVED, BACKUP, SPIN = range(9)
    NAMES = ('BOOT','CRUISE','SLOW','AVOID_L','AVOID_R','ESCAPE','ARRIVED','BACKUP','SPIN')

    # ---------------------------------------------------------------
    def setup(self):
        self.state = self.BOOT
        self.f_mm = self.l_mm = self.r_mm = 4000   # 필터 상태
        self.turn_until = 0
        self.blocked = 0
        self.ticks = 0
        self.state_changes = 0
        self.last_state = self.BOOT
        # 스톨 감지용
        self.enc_l = self.enc_r = 0
        self.stall_ms = 0
        self.recover_until = 0
        self.spin_until = 0
        self.stalls = 0

    # ---------------------------------------------------------------
    def loop(self, t_ms, adc_l, adc_f, adc_r, bea_l, bea_r, enc_l=0, enc_r=0):
        self.ticks += 1

        # --- ① ADC -> mm, 1차 저역통과 (시프트만 사용: (3*old + new)/4)
        l = adc_to_mm(adc_l); f = adc_to_mm(adc_f); r = adc_to_mm(adc_r)
        self.l_mm += (l - self.l_mm) >> FILT_SHIFT
        self.f_mm += (f - self.f_mm) >> FILT_SHIFT
        self.r_mm += (r - self.r_mm) >> FILT_SHIFT
        L, F, R = self.l_mm, self.f_mm, self.r_mm

        # --- ①-b 스톨 감지: 바퀴를 돌리라고 했는데 엔코더가 안 변한다
        moved = abs(enc_l - self.enc_l) + abs(enc_r - self.enc_r)
        self.enc_l, self.enc_r = enc_l, enc_r
        if t_ms < self.recover_until:                      # 복구 동작 중
            if t_ms < self.spin_until:
                self.state = self.SPIN
                return self._out(Motors(-1, TURN_PWM, 1, TURN_PWM))
            self.state = self.BACKUP
            return self._out(Motors(-1, SLOW_PWM, -1, SLOW_PWM))
        if self.state not in (self.BOOT, self.ARRIVED) and moved <= STALL_TICKS:
            self.stall_ms += TICK_MS
            if self.stall_ms >= STALL_MS:                  # 끼였다 -> 후진 후 회전
                self.stalls += 1
                self.stall_ms = 0
                self.spin_until = t_ms + BACK_MS
                self.recover_until = t_ms + BACK_MS + SPIN_MS
                self.state = self.BACKUP
                return self._out(Motors(-1, SLOW_PWM, -1, SLOW_PWM))
        else:
            self.stall_ms = 0

        # --- ② 도착 판정
        if bea_l > ARRIVE_ADC or bea_r > ARRIVE_ADC:
            self.state = self.ARRIVED
            return self._out(Motors(1, 0, 1, 0))

        # --- ③ 회전 커밋 (타이머가 살아 있으면 방향 유지)
        if t_ms < self.turn_until:
            if self.state == self.AVOID_L:
                return self._out(Motors(-1, TURN_PWM, 1, TURN_PWM))
            if self.state == self.AVOID_R:
                return self._out(Motors(1, TURN_PWM, -1, TURN_PWM))
            if self.state == self.ESCAPE:
                return self._out(Motors(-1, TURN_PWM, 1, TURN_PWM))

        # --- ④ 히스테리시스로 '막힘' 판단
        if self.blocked:
            if F > CLEAR_MM:
                self.blocked = 0
        else:
            if F < STOP_MM:
                self.blocked = 1

        # --- ⑤ 막혔으면 넓은 쪽으로, 삼면이 막혔으면 탈출
        if self.blocked:
            if L < SIDE_MM and R < SIDE_MM:
                self.state = self.ESCAPE
                self.turn_until = t_ms + TURN_HOLD_MS * 2
                return self._out(Motors(-1, TURN_PWM, 1, TURN_PWM))
            self.turn_until = t_ms + TURN_HOLD_MS
            if L > R:
                self.state = self.AVOID_L
                return self._out(Motors(-1, TURN_PWM, 1, TURN_PWM))
            self.state = self.AVOID_R
            return self._out(Motors(1, TURN_PWM, -1, TURN_PWM))

        # --- ⑥ 측면 여유가 없으면 반대로 치우쳐 전진
        if R < SIDE_MM and R < L:
            self.state = self.SLOW
            return self._out(Motors(1, SLOW_PWM >> 1, 1, SLOW_PWM))
        if L < SIDE_MM and L < R:
            self.state = self.SLOW
            return self._out(Motors(1, SLOW_PWM, 1, SLOW_PWM >> 1))

        # --- ⑦ 비컨 쪽으로 조향 (좌우 듀티 차이로)
        base = CRUISE_PWM if F > CLEAR_MM else SLOW_PWM
        self.state = self.CRUISE if F > CLEAR_MM else self.SLOW
        # 비컨은 거리가 멀면 둘 다 작아진다. 절대 차이를 쓰면 멀리서는 조향이 죽는다.
        # -> 합으로 나눈 '정규화 차분' 을 쓴다 (라인트레이서와 같은 방식, 정수 연산)
        steer = (bea_l - bea_r) * STEER_GAIN // (bea_l + bea_r + 1)
        lduty = int(clamp(base - steer, 0, PWM_MAX))
        rduty = int(clamp(base + steer, 0, PWM_MAX))
        return self._out(Motors(1, lduty, 1, rduty))

    # ---------------------------------------------------------------
    def _out(self, m):
        if self.state != self.last_state:
            self.state_changes += 1
            self.last_state = self.state
        return m

    @property
    def state_name(self):
        return self.NAMES[self.state]
