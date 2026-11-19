/* firmware.c — MCU 에 그대로 올라가는 코드.
 *
 *   malloc 없음 · float 없음 · 표준 라이브러리 호출 없음
 *   상태는 전부 static (링커가 .bss 에 둔다)
 *   입력은 ADC 카운트, 출력은 PWM 듀티 + 방향 핀
 *
 *   호스트  : clang -shared          -> libfw.dylib   (MuJoCo 가 ctypes 로 호출)
 *   브라우저: clang --target=wasm32  -> firmware.wasm (스터디 노트가 호출)
 *   보드    : arm-none-eabi-gcc      -> .elf
 *   세 경우 모두 같은 파일이다.
 */
#include <stdint.h>

/* ── 튜닝 상수 ──────────────────────────────────────── */
#define TICK_MS       20u
#define IR_K       211000
#define IR_C          111
#define ADC_MAX      1023
#define PWM_MAX       255
#define STOP_MM       480
#define CLEAR_MM      700
#define SIDE_MM       380
#define TURN_HOLD_MS  360u
#define ARRIVE_ADC    760
#define STALL_TICKS     3
#define STALL_MS      500u
#define BACK_MS       500u
#define SPIN_MS       550u
#define CRUISE_PWM    210
#define SLOW_PWM      110
#define TURN_PWM      150
#define STEER_GAIN     95
#define DANGER_MM     500      /* 상태 표시용 */
#define SLOW_TH_MM   1000
#define WHEELBASE_MM  200
#define VMAX_MM_S     630      /* PWM 255 일 때 바퀴 선속도 */
#define BEACON_K      900
#define BEACON_OFF_D   30      /* 좌우 수광부가 정면에서 벌어진 각 */

enum { ST_BOOT, ST_CRUISE, ST_SLOW, ST_AVOID_L, ST_AVOID_R,
       ST_ESCAPE, ST_ARRIVED, ST_BACKUP, ST_SPIN };
enum { ACT_FORWARD, ACT_TURN_LEFT, ACT_TURN_RIGHT, ACT_STOP,
       ACT_SLOW, ACT_ARC_LEFT, ACT_ARC_RIGHT };

typedef struct {
    int16_t l_duty, r_duty;
    int8_t  l_dir,  r_dir;
    uint8_t state;
} fw_out_t;

/* ── 인스턴스 (슬롯) ─────────────────────────────────
 * 페이지에서 여러 실습이 동시에 돌 수 있어 상태를 슬롯으로 나눠 둔다.
 * 배열은 정적이다 — 여전히 malloc 은 없다.
 */
#define FW_SLOTS 4
typedef struct {
    int32_t  l_mm, f_mm, r_mm;
    uint32_t turn_until, stall_ms, rec_until, spin_until, ticks;
    uint16_t stalls, changes;
    uint8_t  state, last_state, blocked, turn_dir;
    fw_out_t out;
} fw_ctx_t;
static fw_ctx_t g_ctx[FW_SLOTS];

static fw_ctx_t *ctx_of(int32_t s)
{
    if (s < 0 || s >= FW_SLOTS) s = 0;
    return &g_ctx[s];
}

/* ── 정수 보조 ──────────────────────────────────────── */
static int32_t clamp32(int32_t v, int32_t lo, int32_t hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* cos 룩업: 0~90도, Q15.  펌웨어에 부동소수점을 안 쓰기 위한 상투적 수단. */
static const int16_t COS_Q15[91] = {
     32767,  32762,  32747,  32722,  32687,  32642,  32587,  32523,  32448,  32364,
     32269,  32165,  32051,  31927,  31794,  31650,  31498,  31335,  31163,  30982,
     30791,  30591,  30381,  30162,  29934,  29697,  29451,  29196,  28932,  28659,
     28377,  28087,  27788,  27481,  27165,  26841,  26509,  26169,  25821,  25465,
     25101,  24730,  24351,  23964,  23571,  23170,  22762,  22347,  21925,  21497,
     21062,  20621,  20173,  19720,  19260,  18794,  18323,  17846,  17364,  16876,
     16384,  15886,  15383,  14876,  14364,  13848,  13328,  12803,  12275,  11743,
     11207,  10668,  10126,   9580,   9032,   8481,   7927,   7371,   6813,   6252,
      5690,   5126,   4560,   3993,   3425,   2856,   2286,   1715,   1144,    572,
         0
};

static int32_t icos_q15(int32_t deg)
{
    if (deg < 0) deg = -deg;
    deg %= 360;
    if (deg <= 90)  return  COS_Q15[deg];
    if (deg <= 180) return -COS_Q15[180 - deg];
    if (deg <= 270) return -COS_Q15[deg - 180];
    return COS_Q15[360 - deg];
}

/* ── 공개 헬퍼 — 페이지의 각 실습이 직접 부른다 ───────── */
int32_t fw_adc_to_mm(int32_t adc)
{
    if (adc <= 0) return 4000;
    return clamp32(IR_K / adc - IR_C, 0, 4000);
}

int32_t fw_mm_to_adc(int32_t mm)
{
    if (mm < 0) mm = 0;
    return clamp32(IR_K / (mm + IR_C), 0, ADC_MAX);
}

/* 0 DANGER / 1 SLOW / 2 SAFE */
int32_t fw_check_state(int32_t mm)
{
    if (mm < DANGER_MM)  return 0;
    if (mm < SLOW_TH_MM) return 1;
    return 2;
}

/* 과제 명세 그대로의 컨트롤러. 상태가 없다 = 순수 함수. */
int32_t fw_basic(int32_t l_mm, int32_t f_mm, int32_t r_mm, int32_t th_mm)
{
    int32_t bf = f_mm < th_mm, bl = l_mm < th_mm, br = r_mm < th_mm;
    if (bf && bl && br) return ACT_TURN_LEFT;
    if (!bf)            return ACT_FORWARD;
    return (l_mm > r_mm) ? ACT_TURN_LEFT : ACT_TURN_RIGHT;
}

/* 행동 -> PWM 듀티 (부호가 방향) */
int32_t fw_act_l(int32_t a)
{
    switch (a) {
    case ACT_FORWARD:    return  PWM_MAX;
    case ACT_TURN_LEFT:  return -TURN_PWM;
    case ACT_TURN_RIGHT: return  TURN_PWM;
    case ACT_SLOW:       return  SLOW_PWM;
    case ACT_ARC_LEFT:   return  SLOW_PWM / 2;
    case ACT_ARC_RIGHT:  return  CRUISE_PWM;
    default:             return  0;
    }
}
int32_t fw_act_r(int32_t a)
{
    switch (a) {
    case ACT_FORWARD:    return  PWM_MAX;
    case ACT_TURN_LEFT:  return  TURN_PWM;
    case ACT_TURN_RIGHT: return -TURN_PWM;
    case ACT_SLOW:       return  SLOW_PWM;
    case ACT_ARC_LEFT:   return  CRUISE_PWM;
    case ACT_ARC_RIGHT:  return  SLOW_PWM / 2;
    default:             return  0;
    }
}

/* differential drive 운동학 — 정수로 */
int32_t fw_v_mm_s(int32_t pl, int32_t pr)      /* 선속도 mm/s */
{
    return (pl + pr) * VMAX_MM_S / (2 * PWM_MAX);
}
int32_t fw_w_mdeg_s(int32_t pl, int32_t pr)    /* 각속도 밀리도/s */
{
    int32_t diff = (pr - pl) * VMAX_MM_S / PWM_MAX;   /* mm/s */
    int32_t mrad = diff * 1000 / WHEELBASE_MM;        /* mrad/s */
    return mrad * 57296 / 1000;
}

/* 비컨 수광부 — 각도는 도, 거리는 mm */
int32_t fw_beacon(int32_t ang_deg, int32_t dist_mm, int32_t is_left)
{
    int32_t off = is_left ? BEACON_OFF_D : -BEACON_OFF_D;
    int32_t c   = icos_q15(ang_deg - off);
    int32_t d;
    if (c <= 0) return 0;
    c = (c * c) >> 15;                      /* cos^2, Q15 */
    d = dist_mm < 350 ? 350 : dist_mm;
    /* 900 * cos^2 / (거리[m])^2.  mm 로 받았으므로 1e6 을 곱해 맞춘다.
       한 번에 d*d 로 나누면 분자가 int32 를 넘기므로 두 번 나눈다. */
    return clamp32((BEACON_K * c / 32768) * 1000000 / d / d, 0, ADC_MAX);
}

/* ── 본체 ───────────────────────────────────────────── */
static fw_out_t mk(fw_ctx_t *c, int8_t ld, int16_t lp, int8_t rd, int16_t rp)
{
    fw_out_t o;
    o.l_dir = ld; o.l_duty = lp;
    o.r_dir = rd; o.r_duty = rp;
    if (c->state != c->last_state) { c->changes++; c->last_state = c->state; }
    o.state = c->state;
    c->out = o;
    return o;
}

void fw_i_setup(int32_t slot)
{
    fw_ctx_t *c = ctx_of(slot);
    c->l_mm = c->f_mm = c->r_mm = 4000;
    c->turn_until = c->stall_ms = c->rec_until = c->spin_until = 0u;
    c->ticks = 0u; c->stalls = 0u; c->changes = 0u;
    c->state = ST_BOOT; c->last_state = ST_BOOT;
    c->blocked = 0u; c->turn_dir = 0u;
    c->out.l_duty = c->out.r_duty = 0;
    c->out.l_dir = c->out.r_dir = 1; c->out.state = ST_BOOT;
}

fw_out_t fw_i_loop(int32_t slot, uint32_t t_ms,
                   uint16_t adc_l, uint16_t adc_f, uint16_t adc_r,
                   uint16_t bea_l, uint16_t bea_r, int32_t enc_moved)
{
    fw_ctx_t *c = ctx_of(slot);
    c->ticks++;

    /* ① ADC -> mm, 1차 저역통과. 나눗셈 대신 시프트 */
    c->l_mm += (fw_adc_to_mm(adc_l) - c->l_mm) >> 2;
    c->f_mm += (fw_adc_to_mm(adc_f) - c->f_mm) >> 2;
    c->r_mm += (fw_adc_to_mm(adc_r) - c->r_mm) >> 2;

    /* ② 복구 동작 중이면 그것부터 끝낸다 */
    if (t_ms < c->rec_until) {
        if (t_ms < c->spin_until) { c->state = ST_BACKUP; return mk(c, -1, SLOW_PWM, -1, SLOW_PWM); }
        c->state = ST_SPIN;       return mk(c, -1, TURN_PWM,  1, TURN_PWM);
    }

    /* ③ 스톨 감지: 돌리라고 했는데 엔코더가 안 변한다 */
    if (c->state != ST_BOOT && c->state != ST_ARRIVED && enc_moved <= STALL_TICKS) {
        c->stall_ms += TICK_MS;
        if (c->stall_ms >= STALL_MS) {
            c->stalls++; c->stall_ms = 0u;
            c->spin_until = t_ms + BACK_MS;
            c->rec_until  = t_ms + BACK_MS + SPIN_MS;
            c->state = ST_BACKUP;
            return mk(c, -1, SLOW_PWM, -1, SLOW_PWM);
        }
    } else {
        c->stall_ms = 0u;
    }

    /* ④ 도착 판정 */
    if (bea_l > ARRIVE_ADC || bea_r > ARRIVE_ADC) {
        c->state = ST_ARRIVED; return mk(c, 1, 0, 1, 0);
    }

    /* ⑤ 회전 커밋 */
    if (c->turn_dir != 0u && t_ms < c->turn_until) {
        c->state = c->turn_dir;
        return (c->turn_dir == ST_AVOID_L) ? mk(c, -1, TURN_PWM, 1, TURN_PWM)
                                           : mk(c,  1, TURN_PWM,-1, TURN_PWM);
    }
    c->turn_dir = 0u;

    /* ⑥ 히스테리시스 */
    if (c->blocked) { if (c->f_mm > CLEAR_MM) c->blocked = 0u; }
    else            { if (c->f_mm < STOP_MM)  c->blocked = 1u; }

    if (c->blocked) {
        if (c->l_mm < SIDE_MM && c->r_mm < SIDE_MM) {
            c->turn_dir = ST_AVOID_L; c->turn_until = t_ms + TURN_HOLD_MS * 2u;
            c->state = ST_ESCAPE;     return mk(c, -1, TURN_PWM, 1, TURN_PWM);
        }
        c->turn_until = t_ms + TURN_HOLD_MS;
        c->turn_dir = (c->l_mm > c->r_mm) ? ST_AVOID_L : ST_AVOID_R;
        c->state = c->turn_dir;
        return (c->turn_dir == ST_AVOID_L) ? mk(c, -1, TURN_PWM, 1, TURN_PWM)
                                           : mk(c,  1, TURN_PWM,-1, TURN_PWM);
    }

    /* ⑦ 측면 여유 */
    if (c->r_mm < SIDE_MM && c->r_mm < c->l_mm) { c->state = ST_SLOW; return mk(c, 1, SLOW_PWM/2, 1, SLOW_PWM); }
    if (c->l_mm < SIDE_MM && c->l_mm < c->r_mm) { c->state = ST_SLOW; return mk(c, 1, SLOW_PWM, 1, SLOW_PWM/2); }

    /* ⑧ 비컨 조향 — 정규화 차분 */
    {
        int32_t base  = (c->f_mm > CLEAR_MM) ? CRUISE_PWM : SLOW_PWM;
        int32_t steer = ((int32_t)bea_l - (int32_t)bea_r) * STEER_GAIN
                        / ((int32_t)bea_l + (int32_t)bea_r + 1);
        c->state = (c->f_mm > CLEAR_MM) ? ST_CRUISE : ST_SLOW;
        return mk(c, 1, (int16_t)clamp32(base - steer, 0, PWM_MAX),
                     1, (int16_t)clamp32(base + steer, 0, PWM_MAX));
    }
}

/* 슬롯 접근자 (wasm 은 구조체 반환이 번거로워 값으로 꺼낸다) */
void    fw_i_tick(int32_t slot, uint32_t t_ms, uint16_t al, uint16_t af, uint16_t ar,
                  uint16_t bl, uint16_t br, int32_t enc)
{ (void)fw_i_loop(slot, t_ms, al, af, ar, bl, br, enc); }
int32_t fw_i_out_l(int32_t s)  { fw_ctx_t *c = ctx_of(s); return (int32_t)c->out.l_dir * c->out.l_duty; }
int32_t fw_i_out_r(int32_t s)  { fw_ctx_t *c = ctx_of(s); return (int32_t)c->out.r_dir * c->out.r_duty; }
int32_t fw_i_state(int32_t s)  { return ctx_of(s)->state; }
int32_t fw_i_mm(int32_t s, int32_t i)
{ fw_ctx_t *c = ctx_of(s); return i == 0 ? c->l_mm : (i == 1 ? c->f_mm : c->r_mm); }
int32_t fw_i_stalls(int32_t s)  { return ctx_of(s)->stalls; }
int32_t fw_i_ticks(int32_t s)   { return (int32_t)ctx_of(s)->ticks; }
int32_t fw_i_changes(int32_t s) { return ctx_of(s)->changes; }

/* ── 기존 호스트(ctypes)용 진입점 — 슬롯 0 ────────────── */
void fw_setup(void) { fw_i_setup(0); }
fw_out_t fw_loop(uint32_t t_ms, uint16_t al, uint16_t af, uint16_t ar,
                 uint16_t bl, uint16_t br, int32_t enc)
{ return fw_i_loop(0, t_ms, al, af, ar, bl, br, enc); }
int32_t  fw_get_mm(int i)     { return fw_i_mm(0, i); }
uint16_t fw_get_stalls(void)  { return (uint16_t)fw_i_stalls(0); }
uint32_t fw_get_ticks(void)   { return (uint32_t)fw_i_ticks(0); }
uint16_t fw_get_changes(void) { return (uint16_t)fw_i_changes(0); }
