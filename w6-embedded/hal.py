"""HAL — 하드웨어 추상화 계층.

실제 MCU 펌웨어가 보는 것과 같은 형태로 바꿔준다.
펌웨어는 '미터'를 모른다. ADC 카운트(0~1023)와 PWM 듀티(0~255)만 안다.
"""

ADC_MAX = 1023      # 10비트 ADC
PWM_MAX = 255       # 8비트 PWM

# --- Sharp IR 거리센서 모델:  adc = K / (d_mm + C)
#     가까울수록 값이 크고, 멀수록 급격히 둔해진다 (실제 IR 센서의 성질)
IR_K, IR_C = 211_000, 111

# --- 비컨(목표) 수광 포토다이오드
BEACON_K = 900          # 밝기 계수
BEACON_HALF_DEG = 30    # 좌우 수광부가 정면에서 벌어진 각도


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def ir_adc(dist_m, noise_counts=0, rng=None):
    """거리[m] -> ADC 카운트. 센서가 안 맞으면(-1) 아주 먼 것으로 본다."""
    if dist_m is None or dist_m < 0:
        dist_m = 4.0
    d_mm = max(int(dist_m * 1000), 1)
    adc = IR_K // (d_mm + IR_C)
    if noise_counts and rng:
        adc += rng.randint(-noise_counts, noise_counts)
    return int(clamp(adc, 0, ADC_MAX))


def adc_to_mm(adc):
    """펌웨어가 쓰는 역변환. 정수 나눗셈만 사용한다."""
    if adc <= 0:
        return 4000
    mm = IR_K // adc - IR_C
    return int(clamp(mm, 0, 4000))


def beacon_adc(angle_deg, dist_m, half_deg=BEACON_HALF_DEG):
    """목표 방향[deg, +가 왼쪽] 과 거리로 좌/우 수광부 ADC 를 만든다."""
    import math
    out = []
    for off in (+half_deg, -half_deg):          # 왼쪽, 오른쪽
        c = math.cos(math.radians(angle_deg - off))
        c = max(0.0, c) ** 2
        v = BEACON_K * c / max(dist_m, 0.35) ** 2
        out.append(int(clamp(int(v), 0, ADC_MAX)))
    return out[0], out[1]


# --- 휠 엔코더: 1회전당 360펄스 (기어모터 + 쿼드러처 수준)
#   20펄스로 하면 20 ms 제어주기에 0.4펄스밖에 안 변해서
#   정상 주행 중에도 '안 굴러간다'고 오판한다. 분해능 != 장식.
TICKS_PER_REV = 360


def encoder_ticks(angle_rad):
    """바퀴 관절 각도[rad] -> 누적 엔코더 카운트(정수)."""
    import math
    return int(angle_rad * TICKS_PER_REV / (2 * math.pi))
