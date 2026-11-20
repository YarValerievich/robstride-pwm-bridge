/* =============================================================================
 *  robstride_pwm_bridge.c   —   ВЕРСИЯ v5
 *  Мост PWM -> CAN для сервопривода ROBSTRIDE 00 (QDD) на STM32F103C8 (Blue Pill)
 *  STM32 HAL. Конечный автомат: INIT -> SELFTEST -> OPERATION  (+ FAULT)
 *
 *  Прикладной слой. Периферию настраивает CubeMX (см. блок ниже).
 *  Вызовите app_setup() один раз после MX_*_Init(), затем app_loop() в while(1).
 *  Колбэки HAL_CAN_* / HAL_TIM_IC_CaptureCallback реализованы здесь — не дублируйте.
 *
 *  --- ИЗМЕНЕНИЯ v4 -> v5 -----------------------------------------------------
 *   [FIX] Вывод WS2812 снова защищён от разрыва прерываниями (критическая секция
 *         по умолчанию, опционально DMA). Причина бага «зелёный подмешивается ко
 *         всем цветам»: зелёный занимает ПЕРВЫЕ 3 байта посылки (порядок GRB), и
 *         прерывание в начале передачи растягивало бит именно в этом байте.
 *   [FIX] Возвращён guard по каналу в HAL_TIM_IC_CaptureCallback — колбэк снова
 *         срабатывает только на CH2 (ширина импульса), а не вдвое чаще.
 *   [FIX] Скользящее среднее канала профиля вынесено из ISR в основной цикл
 *         (ISR должен быть коротким); в ISR остаётся только сохранение значения.
 *   [NEW] Отладочный вывод в UART — под флагом DEBUG_UART (0 = полностью убран,
 *         включая extern huart1).
 *   [NEW] Пороги «мусорного» захвата ШИМ вынесены в PWM_CAPTURE_MIN/MAX.
 *   [NEW] VERIFY_MOTOR_PRESENCE — опциональная проверка наличия привода в INIT
 *         (при 0 сохраняется быстрый старт из v4 с жёстким motor_id).
 *  ---------------------------------------------------------------------------
 *  НАСТРОЙКА ПЕРИФЕРИИ В CubeMX (STM32F103C8, 72 МГц, HSE + PLL):
 *
 *   CAN (hcan): PB8=CAN_RX, PB9=CAN_TX (CAN1 remap!). 1 Mbit/s @APB1=36МГц:
 *     Prescaler=2, SJW=1TQ, BS1=15TQ, BS2=2TQ (~88.9%). Mode Normal.
 *     AutoBusOff = ENABLE (ABOM). NVIC: CAN1 RX0 + CAN1 SCE.
 *     Фильтр: все extended (IDMASK, маска 0), FIFO0.
 *   TIM2 (htim2): PWM Input на TI1 (PA0), Prescaler=71 (1 мкс), ARR=65535, NVIC.
 *                 Slave Mode = Reset Mode, Trigger = TI1FP1, Filter = 8.
 *   TIM3 (htim3): PWM Input на TI1 (PA6), Prescaler=71 (1 мкс), ARR=65535, NVIC.
 *                 Slave Mode = Reset Mode, Trigger = TI1FP1, Filter = 8.
 *   SPI2 (hspi2): только MOSI=PB15, TX-Only Master, 8 бит MSB, Prescaler=16
 *                 (=> 2.25 МГц, бит ~0.44 мкс). CPOL=Low, CPHA=1 Edge.
 *                 Для WS2812_USE_DMA=1 добавить SPI2_TX DMA:
 *                 Mode Normal, Data Width = Byte/Byte, Memory Increment = ON,
 *                 Peripheral Increment = OFF.
 *   USART1 (huart1): нужен только при DEBUG_UART=1. 115200 8N1, TX (PA9).
 *   IWDG: нужен только при USE_IWDG=1, период >= ~500 мс.
 *   SysTick: 1 мс.
 *  ---------------------------------------------------------------------------
 *  ПИТАНИЕ: привод 30В от DC-DC (просадка <28В = повреждение). Логика 5В/3.3В
 *  отдельно, земля общая. WS2812 при 5В хочет VIH~3.5В: с 3.3В — на грани,
 *  желателен буфер 74HCT или питание ленты ~4.0..4.3В.
 * ========================================================================== */

/* -----------------------------------------------------------------------------
 *  КАРТА ФАЙЛА
 *  0. ФЛАГИ СБОРКИ      DEBUG_UART / USE_IWDG / WS2812_USE_DMA /
 *                       VERIFY_MOTOR_PRESENCE / PWM_CAPTURE_MIN|MAX
 *  1. КОНФИГУРАЦИЯ      config_t cfg, profile_t profiles[3]
 *  2. ПРОТОКОЛ          RS_CMD_*, IDX_*, MOTOR_FAULT_MASK
 *  3. СОСТОЯНИЕ         fsm/fault enum, tlm, motor_fault, async_*, счётчики
 *  4. НИЗКИЙ УРОВЕНЬ CAN  rs_make_id()/rs_can_send()
 *  5. КОМАНДЫ           enable/stop/write/read/request_read/pp/csp/goto
 *  6. ПРИЁМ             RxFifo0 · ErrorCallback(bus-off) · TIM IC (только CH2)
 *  7. ИНДИКАЦИЯ         ws2812_show_n (защищённый вывод) / indication_color
 *  8. ВСПОМОГАТЕЛЬНОЕ   pwm_filtered / profile_filtered (усреднение вне ISR) /
 *                       pwm_to_angle / pwm_alive / classify_profile / go_fault
 *  9. FSM               do_init / do_selftest / do_operation / do_fault
 * 10. ТОЧКИ ВХОДА       app_setup / app_loop
 * --------------------------------------------------------------------------- */

#include "main.h"
#include <string.h>
#include <math.h>
#include <stdbool.h>
/* <stdio.h> намеренно не подключается: snprintf/printf тянут стек и код. */

extern CAN_HandleTypeDef  hcan;
extern TIM_HandleTypeDef  htim2;   /* основной канал управления (PA0) */
extern TIM_HandleTypeDef  htim3;   /* канал выбора профиля (PA6)      */
extern SPI_HandleTypeDef  hspi2;

/* =============================================================================
 *  0. ФЛАГИ СБОРКИ
 * ========================================================================== */

/* --- Отладочный вывод в UART. 0 = убирается полностью (вместе с extern). --- */
#define DEBUG_UART 0

#if DEBUG_UART
extern UART_HandleTypeDef huart1;
static void dbg(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), 10);
}
/* Печать однозначного кода результата: "<r>\r\n". */
static void dbg_res(int r)
{
    char b[4];
    b[0] = (char)('0' + (r & 0x0F)); b[1] = '\r'; b[2] = '\n'; b[3] = '\0';
    dbg(b);
}
#else
#define dbg(s)     ((void)0)
#define dbg_res(r) ((void)0)
#endif

/* --- Аппаратный сторож (IWDG). Требует конфигурации в CubeMX. --- */
#define USE_IWDG 0
#if USE_IWDG
extern IWDG_HandleTypeDef hiwdg;
#define IWDG_KICK() HAL_IWDG_Refresh(&hiwdg)
#else
#define IWDG_KICK() ((void)0)
#endif

/* --- Способ защиты потока WS2812 от прерываний ---
 * 0 = критическая секция вокруг HAL_SPI_Transmit (~32 мкс на диод; приёмный
 *     FIFO bxCAN на 3 кадра всё буферизует, потерь нет). Ничего настраивать
 *     в CubeMX не нужно — режим по умолчанию.
 * 1 = вывод по DMA: ядро и прерывания на поток не влияют вовсе.
 *     Требует SPI2_TX DMA в CubeMX (Byte/Byte, Normal, Memory Increment ON). */
#define WS2812_USE_DMA 1

/* --- Проверять наличие привода в INIT запросом OBTAIN_ID ---
 * 0 = быстрый старт (поведение v4): при заданном motor_id адрес принимается
 *     без проверки, отсутствие привода проявится как FR_COMMS при чтении.
 * 1 = как в ТЗ: адрес подтверждается ответом привода, иначе FR_NO_MOTOR. */
#define VERIFY_MOTOR_PRESENCE 0

/* --- Окно «правдоподобной» ширины импульса при захвате (мкс) ---
 * Всё вне окна считается помехой и отбрасывается в ISR. Расширьте до 500..3000,
 * если источник ШИМ выдаёт края диапазона (по ТЗ источник 500..2500 мкс). */
#define PWM_CAPTURE_MIN  800
#define PWM_CAPTURE_MAX  2200

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define DEG2RAD(d)  ((float)(d) * (M_PI / 180.0f))

#define LED_MAX     16     /* верхняя граница цепочки WS2812 (статический буфер) */

/* =============================================================================
 *  1. НАСТРАИВАЕМЫЕ ПЕРЕМЕННЫЕ
 * ========================================================================== */
typedef struct {
    float max_angle;   /* deg   — амплитуда поворота (±max_angle)  */
    float op_vel;      /* rad/s — скорость в OPERATION (limit_spd) */
    float op_acc;      /* rad/s²— ускорение в OPERATION (acc_set)  */
} profile_t;

/* max_angle держать ниже физического упора (~55°). */
profile_t profiles[3] = {
    /* profile_1 */ { .max_angle = 15.0f, .op_vel = 2.0f, .op_acc = 10.0f },
    /* profile_2 */ { .max_angle = 35.0f, .op_vel = 5.0f, .op_acc = 40.0f },
    /* profile_3 */ { .max_angle = 55.0f, .op_vel = 20.0f, .op_acc = 200.0f },
};

typedef struct {
    /* --- пределы безопасности --- */
    float    motorTemp_max;     /* °C   (80)   */
    float    VBUS_min;          /* V    (28)   */
    float    mech_pos_limit;    /* deg  — порог "упор разрушен" (60) */
    float    torque_fdb_limit;  /* N·m  (15)   */

    /* --- SELFTEST --- */
    float    selftest_angle;    /* deg  (50)   */
    float    selftest_torque;   /* N·m  (5)    */
    float    selftest_vel;      /* rad/s(5)    */
    float    selftest_acc;      /* rad/s²(50)  */
    uint16_t selftest_delay_ms; /* (1000)      */

    /* --- PWM управления --- */
    uint16_t pwm_min;           /* мкс  (1000) */
    uint16_t pwm_max;           /* мкс  (2000) */
    uint16_t pwm_dead_zone;     /* мкс  (10)   */
    uint16_t pwm_timeout_ms;    /* failsafe (60) */
    uint8_t  pwm_filter_window; /* окно среднего 1..PWM_FILTER_MAX (8) */

    /* --- PWM выбора профиля --- */
    uint16_t profile_thr_lo;    /* мкс  (1300) */
    uint16_t profile_thr_hi;    /* мкс  (1700) */
    uint16_t profile_hyst;      /* мкс  (30)   */
    uint8_t  profile_debounce;  /* (3)         */

    /* --- антидребезг защит --- */
    uint8_t  vbus_fault_debounce;/* подряд VBUS<min до FAULT (3) */

    /* --- индикация --- */
    uint8_t  led_count;         /* WS2812 в цепочке (1) */

    /* --- тайминги --- */
    uint16_t looptime_freq;     /* Гц   (100)  */

    /* --- восстановление после отказа --- */
    uint16_t restart_delay_ms;       /* мс — удержание FR_TORQUE (3000)         */
    uint8_t  comm_retry_attempts;    /* попыток связи FR_COMMS (5)              */
    uint16_t comm_retry_delay_ms;    /* интервал попыток связи (2000)           */
    float    fr_temp_retry;          /* °C — порог снятия удержания FR_TEMP (75)*/
    uint8_t  selftest_retry_attempts;/* повторов самотеста FR_SELFTEST (2)      */

    /* --- защита от «шторма» мягких отказов --- */
    uint8_t  fault_recover_max; /* мягких восстановлений подряд до защёлки (3) */
    uint32_t fault_reset_ms;    /* здоровая работа сбрасывает бюджет (60000)   */
    uint16_t tlm_lost_ms;       /* молчание телеметрии дольше -> отказ (1500)  */

    /* --- адресация CAN --- */
    uint8_t  host_id;           /* (0xFD) */
    uint8_t  motor_id;          /* 0 = автопоиск */

    /* --- допуск точки SELFTEST --- */
    float    pos_tol;           /* rad  (0.05) */
} config_t;

config_t cfg = {
    .motorTemp_max    = 80.0f,
    .VBUS_min         = 28.0f,
    .mech_pos_limit   = 56.0f,
    .torque_fdb_limit = 10.0f,
    .selftest_angle   = 55.0f,
    .selftest_torque  = 3.0f,
    .selftest_vel     = 2.0f,
    .selftest_acc     = 8.0f,
    .selftest_delay_ms= 1000,
    .pwm_min          = 1000,
    .pwm_max          = 2000,
    .pwm_dead_zone    = 5,
    .pwm_timeout_ms   = 60,
    .pwm_filter_window= 2,
    .profile_thr_lo   = 1300,
    .profile_thr_hi   = 1700,
    .profile_hyst     = 30,
    .profile_debounce = 3,
    .vbus_fault_debounce = 3,
    .led_count        = 1,
    .looptime_freq    = 100,
    .restart_delay_ms = 3000,
    .comm_retry_attempts     = 5,
    .comm_retry_delay_ms     = 2000,
    .fr_temp_retry           = 75.0f,
    .selftest_retry_attempts = 2,
    .fault_recover_max       = 3,
    .fault_reset_ms          = 60000,
    .tlm_lost_ms             = 1500,
    .host_id          = 0xFD,
    .motor_id         = 127,     /* жёстко заданный ID; 0 = автопоиск */
    .pos_tol          = 0.05f,
};

/* =============================================================================
 *  2. ПРОТОКОЛ ROBSTRIDE / CYBERGEAR
 *  Расширенный 29-бит ID: [28:24]=тип связи [23:8]=host_id [7:0]=motor_id
 * ========================================================================== */
#define RS_CMD_OBTAIN_ID   0
#define RS_CMD_CONTROL     1
#define RS_CMD_FEEDBACK    2
#define RS_CMD_ENABLE      3
#define RS_CMD_STOP        4
#define RS_CMD_SET_ZERO    6
#define RS_CMD_SET_ID      7
#define RS_CMD_READ        17
#define RS_CMD_WRITE       18
#define RS_CMD_FAULT       21
#define RS_CMD_FB_ENABLE   24

#define RUN_MODE_OPERATION  0
#define RUN_MODE_POSITION   1   /* PP  — SELFTEST  */
#define RUN_MODE_VELOCITY   2
#define RUN_MODE_CURRENT    3
#define RUN_MODE_CSP        5   /* CSP — OPERATION */

#define IDX_RUN_MODE     0x7005
#define IDX_LIMIT_TORQUE 0x700B
#define IDX_LOC_REF      0x7016
#define IDX_LIMIT_SPD    0x7017
#define IDX_LIMIT_CUR    0x7018
#define IDX_MECH_POS     0x7019
#define IDX_VBUS         0x701C
#define IDX_ACC_SET      0x7025  /* <-- Адрес проверен */
#define USE_ACC_PARAM    1       /* 1 = писать op_acc/selftest_acc */

/* Биты ошибок привода в ID кадра ОС (позиции 16..21). Точный состав фатальных
 * бит уточнить по мануалу RS00, иначе возможны ложные FR_MOTOR_INTERNAL. */
#define MOTOR_FAULT_MASK 0x3F

/* Масштаб момента в кадре ОС — ПРОВЕРИТЬ. Температура raw/10 — однозначно. */
#define RS_T_MIN  (-17.0f)
#define RS_T_MAX  ( 17.0f)

/* =============================================================================
 *  3. ГЛОБАЛЬНОЕ СОСТОЯНИЕ
 * ========================================================================== */
#define PWM_FILTER_MAX 32

typedef enum { ST_INIT, ST_SELFTEST, ST_OPERATION, ST_FAULT } fsm_state_t;

typedef enum {
    FR_NONE = 0,
    FR_NO_MOTOR,        /* HARD */
    FR_COMMS,           /* soft */
    FR_VBUS,            /* HARD */
    FR_TEMP,            /* soft */
    FR_TORQUE,          /* soft */
    FR_MECH_LIMIT,      /* HARD */
    FR_SELFTEST,        /* soft */
    FR_MOTOR_INTERNAL,  /* HARD */
    FR_PWM_LOST         /* HARD */
} fault_reason_t;

/* «Жёсткие» причины -> немедленная защёлка без попыток восстановления. */
static inline uint8_t fault_is_hard(fault_reason_t r)
{
    return (r == FR_NO_MOTOR)       || (r == FR_VBUS) || (r == FR_MECH_LIMIT) ||
           (r == FR_MOTOR_INTERNAL) || (r == FR_PWM_LOST);
}

typedef struct { uint8_t r, g, b; } rgb_t;

static volatile fsm_state_t    state          = ST_INIT;
static fsm_state_t             return_state   = ST_INIT;
static volatile fault_reason_t fault_reason   = FR_NONE;
static uint8_t                 fault_latched  = 0;
static uint8_t                 selftest_retry = 0;   /* повторов самотеста */
static uint8_t                 soft_recover_count = 0;/* бюджет мягких восстановлений */

static volatile uint8_t active_profile = 0;
static inline float cur_max_angle(void) { return profiles[active_profile].max_angle; }
static inline float cur_op_vel(void)    { return profiles[active_profile].op_vel;    }
static inline float cur_op_acc(void)    { return profiles[active_profile].op_acc;    }

/* Телеметрия из кадра ОС (пишется в ISR, читается атомарным снимком) */
typedef struct {
    volatile float    temp;
    volatile float    torque;
    volatile uint8_t  mode_status;
    volatile uint16_t fault_bits;
    volatile uint8_t  fresh;
} telemetry_t;
static telemetry_t tlm = {0};
static volatile uint8_t  motor_fault   = 0;   /* «липкий» флаг ошибки привода */
static volatile uint32_t tlm_last_tick = 0;   /* время последнего кадра ОС    */

/* Блокирующее чтение (тип 17) — для INIT/SELFTEST */
static volatile uint16_t rd_index = 0;
static volatile float    rd_value = 0;
static volatile uint8_t  rd_ready = 0;

/* Асинхронные чтения для OPERATION (ответы раскладывает ISR по индексу) */
static volatile float   async_vbus = 0, async_mpos = 0;
static volatile uint8_t async_vbus_fresh = 0, async_mpos_fresh = 0;

/* PWM-вход (ISR сохраняет только сырые значения — без вычислений) */
static volatile uint16_t pwm_pulse_us      = 1500;
static volatile uint32_t pwm_last_tick     = 0;
static volatile uint16_t profile_pulse_us  = 1000;
static volatile uint32_t profile_last_tick = 0;

/* Автопоиск адреса */
static uint8_t  found_motor_id = 0;
static volatile uint8_t scan_hit_id = 0;
static volatile uint8_t scan_got = 0;

/* =============================================================================
 *  4. НИЗКОУРОВНЕВЫЙ CAN
 * ========================================================================== */
static uint32_t rs_make_id(uint8_t cmd, uint16_t data, uint8_t motor_id)
{
    return ((uint32_t)(cmd & 0x1F) << 24) | ((uint32_t)data << 8) | ((uint32_t)motor_id);
}

/* Отправить кадр. Ждёт свободный mailbox до 5 мс, иначе HAL_TIMEOUT. */
static HAL_StatusTypeDef rs_can_send(uint32_t ext_id, const uint8_t *d, uint8_t len)
{
    CAN_TxHeaderTypeDef tx;
    uint32_t mbox, t0 = HAL_GetTick();
    tx.ExtId = ext_id; tx.IDE = CAN_ID_EXT; tx.RTR = CAN_RTR_DATA;
    tx.DLC = len; tx.TransmitGlobalTime = DISABLE;
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) {
        if (HAL_GetTick() - t0 > 5) return HAL_TIMEOUT;
    }
    return HAL_CAN_AddTxMessage(&hcan, &tx, (uint8_t *)d, &mbox);
}

/* =============================================================================
 *  5. КОМАНДЫ ПРОТОКОЛА
 * ========================================================================== */
static void rs_enable(uint8_t id)
{ uint8_t d[8] = {0}; rs_can_send(rs_make_id(RS_CMD_ENABLE, cfg.host_id, id), d, 8); }

/* OFF привода (stop, тип 4). clear_fault=1 — сбросить ошибку привода. */
static void rs_stop(uint8_t id, uint8_t clear_fault)
{ uint8_t d[8] = {0}; if (clear_fault) d[0] = 1;
  rs_can_send(rs_make_id(RS_CMD_STOP, cfg.host_id, id), d, 8); }

static void __attribute__((unused)) rs_set_zero(uint8_t id)
{ uint8_t d[8] = {0}; d[0] = 1; rs_can_send(rs_make_id(RS_CMD_SET_ZERO, cfg.host_id, id), d, 8); }

static void rs_write_f(uint8_t id, uint16_t index, float val)
{ uint8_t d[8] = {0}; d[0] = index & 0xFF; d[1] = index >> 8; memcpy(&d[4], &val, 4);
  rs_can_send(rs_make_id(RS_CMD_WRITE, cfg.host_id, id), d, 8); }

static void rs_write_u8(uint8_t id, uint16_t index, uint8_t val)
{ uint8_t d[8] = {0}; d[0] = index & 0xFF; d[1] = index >> 8; d[4] = val;
  rs_can_send(rs_make_id(RS_CMD_WRITE, cfg.host_id, id), d, 8); }

/* Включить активную телеметрию (тип 24). */
static void rs_enable_feedback(uint8_t id)
{ uint8_t d[8] = {0}; d[0] = 1; rs_can_send(rs_make_id(RS_CMD_FB_ENABLE, cfg.host_id, id), d, 8); }

/* Неблокирующий запрос чтения параметра (ответ обработает ISR). */
static void rs_request_read(uint8_t id, uint16_t index)
{ uint8_t d[8] = {0}; d[0] = index & 0xFF; d[1] = index >> 8;
  rs_can_send(rs_make_id(RS_CMD_READ, cfg.host_id, id), d, 8); }

/* Блокирующее чтение float-параметра (INIT/SELFTEST). return 1 = успех. */
static uint8_t rs_read_f(uint8_t id, uint16_t index, float *out, uint32_t to_ms)
{
    rd_index = index; rd_ready = 0;
    rs_request_read(id, index);
    uint32_t t0 = HAL_GetTick();
    while (!rd_ready) { if (HAL_GetTick() - t0 > to_ms) return 0; }
    *out = rd_value;
    return 1;
}

static void __attribute__((unused)) rs_obtain_id(uint8_t id)
{ uint8_t d[8] = {0}; rs_can_send(rs_make_id(RS_CMD_OBTAIN_ID, cfg.host_id, id), d, 8); }

/* Режим PP (для SELFTEST). */
static void rs_enter_pp(uint8_t id, float vel, float acc, float trq_limit)
{
    rs_write_u8(id, IDX_RUN_MODE, RUN_MODE_POSITION);
    rs_write_f (id, IDX_LIMIT_TORQUE, trq_limit);
    rs_write_f (id, IDX_LIMIT_SPD,    vel);
#if USE_ACC_PARAM
    rs_write_f (id, IDX_ACC_SET, acc);
#else
    (void)acc;
#endif
    rs_enable(id);
}

/* op_vel/op_acc активного профиля (вход в OPERATION и смена профиля). */
static void rs_apply_op_profile(uint8_t id)
{
    rs_write_f(id, IDX_LIMIT_SPD, cur_op_vel());
#if USE_ACC_PARAM
    rs_write_f(id, IDX_ACC_SET,   cur_op_acc());
#endif
}

/* Режим CSP (для OPERATION). */
static void rs_enter_csp(uint8_t id, float trq_limit)
{
    rs_write_u8(id, IDX_RUN_MODE, RUN_MODE_CSP);
    rs_write_f (id, IDX_LIMIT_TORQUE, trq_limit);
    rs_apply_op_profile(id);
    rs_enable(id);
}

static inline void rs_goto(uint8_t id, float angle_rad)
{ rs_write_f(id, IDX_LOC_REF, angle_rad); }

/* =============================================================================
 *  6. КОЛБЭКИ ПРИЁМА
 * ========================================================================== */
static float u16_to_f(uint16_t x, float xmin, float xmax)
{ return (float)x * (xmax - xmin) / 65535.0f + xmin; }

/* Приём CAN: ОС(2), чтение(17)->блок.+асинх кэши, анонс ID(0), ошибка(21). */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hc)
{
    CAN_RxHeaderTypeDef rx;
    uint8_t data[8];
    if (HAL_CAN_GetRxMessage(hc, CAN_RX_FIFO0, &rx, data) != HAL_OK) return;
    if (rx.IDE != CAN_ID_EXT) return;

    uint8_t cmd = (rx.ExtId >> 24) & 0x1F;

    switch (cmd) {
    case RS_CMD_FEEDBACK: {                          /* тип 2: телеметрия */
        uint16_t raw_tor = ((uint16_t)data[4] << 8) | data[5];
        uint16_t raw_tmp = ((uint16_t)data[6] << 8) | data[7];
        uint16_t fbits   = (rx.ExtId >> 16) & 0x3F;
        tlm.torque      = u16_to_f(raw_tor, RS_T_MIN, RS_T_MAX);
        tlm.temp        = (float)raw_tmp / 10.0f;
        tlm.mode_status = (rx.ExtId >> 22) & 0x03;
        tlm.fault_bits  = fbits;
        tlm.fresh       = 1;
        tlm_last_tick   = HAL_GetTick();
        if (fbits & MOTOR_FAULT_MASK) motor_fault = 1;   /* липкий, не затирается */
        break;
    }
    case RS_CMD_READ: {                              /* тип 17: ответ чтения */
        uint16_t idx = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
        float f; memcpy(&f, &data[4], 4);
        if (idx == IDX_VBUS)          { async_vbus = f; async_vbus_fresh = 1; }
        else if (idx == IDX_MECH_POS) { async_mpos = f; async_mpos_fresh = 1; }
        if (idx == rd_index)          { rd_value = f;   rd_ready = 1; }
        break;
    }
    case RS_CMD_OBTAIN_ID:                            /* тип 0: анонс ID */
        scan_hit_id = (rx.ExtId >> 8) & 0xFF; scan_got = 1;
        break;
    case RS_CMD_FAULT:                                /* тип 21: ошибка привода */
        motor_fault = 1;
        break;
    default: break;
    }
}

/* Восстановление шины CAN при bus-off (нужен ABOM=ENABLE + NVIC SCE). */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hc)
{
    uint32_t err = HAL_CAN_GetError(hc);
    if (err & HAL_CAN_ERROR_BOF) {
        HAL_CAN_ResetError(hc);   /* с ABOM bxCAN сам вернётся на шину */
    }
}

/* Захват PWM. ВАЖНО: обрабатываем ТОЛЬКО CH2 (спад = ширина импульса).
 * Без этой проверки колбэк срабатывает ещё и на CH1 (период) — частота
 * прерываний удваивается без всякой пользы.
 * В ISR сохраняем только сырое значение; любые фильтры — в основном цикле. */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_2) return;

    uint32_t w = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    if (w < PWM_CAPTURE_MIN || w > PWM_CAPTURE_MAX) return;   /* фильтр мусора */

    if (htim->Instance == TIM2) {
        pwm_pulse_us      = (uint16_t)w;
        pwm_last_tick     = HAL_GetTick();
    } else if (htim->Instance == TIM3) {
        profile_pulse_us  = (uint16_t)w;
        profile_last_tick = HAL_GetTick();
    }
}

/* =============================================================================
 *  7. ИНДИКАЦИЯ WS2812
 * ========================================================================== */
/* Поток WS2812 нельзя прерывать: у диода нет тактовой линии, и пауза между
 * байтами растягивает бит, который декодируется как «1». Зелёный занимает
 * ПЕРВЫЕ 3 байта посылки (порядок GRB), поэтому прерывание в начале передачи
 * подмешивает зелёный ко всем цветам (чёрный -> зелёный, красный -> жёлтый).
 * Защита: критическая секция (по умолчанию) либо DMA. */
static uint8_t ws_buf[LED_MAX * 9];   /* static — обязателен для режима DMA */

static void ws2812_show_n(const rgb_t *px, uint8_t n)
{
    if (n < 1) n = 1;
    if (n > LED_MAX) n = LED_MAX;

#if WS2812_USE_DMA
    if (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY) return;  /* идёт передача */
#endif

    int bitpos = 0;
    memset(ws_buf, 0, (size_t)n * 9);
    for (uint8_t L = 0; L < n; L++) {
        uint8_t grb[3] = { px[L].g, px[L].r, px[L].b };   /* порядок WS2812 */
        for (int c = 0; c < 3; c++)
            for (int i = 7; i >= 0; i--) {
                uint8_t pat = ((grb[c] >> i) & 1) ? 0b110 : 0b100;
                for (int k = 2; k >= 0; k--) {
                    if ((pat >> k) & 1) ws_buf[bitpos >> 3] |= (0x80 >> (bitpos & 7));
                    bitpos++;
                }
            }
    }

#if WS2812_USE_DMA
    HAL_SPI_Transmit_DMA(&hspi2, ws_buf, (uint16_t)(n * 9));
#else
    __disable_irq();                                     /* поток без разрывов */
    HAL_SPI_Transmit(&hspi2, ws_buf, (uint16_t)(n * 9), 10);
    __enable_irq();
#endif
}

/* Цвет активного профиля: p1 зелёный / p2 синий / p3 фиолетовый. */
static rgb_t profile_color(void)
{
    switch (active_profile) {
    case 0:  { rgb_t c = {0, 60, 0};  return c; }   /* зелёный    */
    case 1:  { rgb_t c = {0, 0, 80};  return c; }   /* синий      */
    default: { rgb_t c = {60, 0, 80}; return c; }   /* фиолетовый */
    }
}

/* INIT белый · SELFTEST жёлтый мигает · OPERATION = цвет профиля ·
 * FAULT красный (мигает при удержании/попытках, горит при защёлке). */
static rgb_t indication_color(void)
{
    uint8_t on = (HAL_GetTick() / 250) & 1;
    rgb_t off = {0, 0, 0};
    switch (state) {
    case ST_INIT:      { rgb_t c = {60, 60, 60}; return c; }
    case ST_SELFTEST:  { rgb_t c = {60, 40, 0};  return on ? c : off; }
    case ST_OPERATION: return profile_color();
    case ST_FAULT:     { rgb_t c = {80, 0, 0}; return (fault_latched || on) ? c : off; }
    }
    return off;
}

/* Rate-limit ~33 Гц: гарантирует паузу сброса WS2812 (>50 мкс) и снимает
 * нагрузку при вызове в тесных циклах. Здесь же кормится IWDG. */
static void led_update(void)
{
    static uint32_t last = 0;
    IWDG_KICK();
    uint32_t now = HAL_GetTick();
    if (now - last < 30) return;
    last = now;

    uint8_t n = cfg.led_count;
    if (n < 1) n = 1;
    if (n > LED_MAX) n = LED_MAX;
    rgb_t px[LED_MAX];
    rgb_t c = indication_color();
    for (uint8_t i = 0; i < n; i++) px[i] = c;
    ws2812_show_n(px, n);
}

/* =============================================================================
 *  8. ВСПОМОГАТЕЛЬНОЕ
 * ========================================================================== */
/* --- скользящее среднее основного канала (вызывается в цикле OPERATION) --- */
static uint16_t pf_buf[PWM_FILTER_MAX];
static uint32_t pf_sum = 0;
static uint8_t  pf_head = 0, pf_count = 0, pf_win = 0;

static void pwm_filter_reset(void) { pf_sum = 0; pf_head = 0; pf_count = 0; pf_win = 0; }

static uint16_t pwm_filtered(uint16_t raw)
{
    uint8_t w = cfg.pwm_filter_window;
    if (w < 1) w = 1;
    if (w > PWM_FILTER_MAX) w = PWM_FILTER_MAX;
    if (w != pf_win) { pf_win = w; pf_sum = 0; pf_head = 0; pf_count = 0; }
    if (pf_count < pf_win) { pf_buf[pf_head] = raw; pf_sum += raw; pf_head = (pf_head + 1) % pf_win; pf_count++; }
    else { pf_sum -= pf_buf[pf_head]; pf_buf[pf_head] = raw; pf_sum += raw; pf_head = (pf_head + 1) % pf_win; }
    return (uint16_t)(pf_sum / pf_count);
}

/* --- скользящее среднее канала профиля (вынесено ИЗ ISR) ---
 * Чтение volatile uint16_t на Cortex-M3 атомарно (одна инструкция LDRH),
 * поэтому критическая секция не нужна. */
#define PROFILE_FILTER_N 4
static uint16_t prof_buf[PROFILE_FILTER_N];
static uint8_t  prof_idx = 0, prof_cnt = 0;

static void profile_filter_reset(void) { prof_idx = 0; prof_cnt = 0; }

static uint16_t profile_filtered(void)
{
    uint16_t raw = profile_pulse_us;
    prof_buf[prof_idx] = raw;
    prof_idx = (uint8_t)((prof_idx + 1) % PROFILE_FILTER_N);
    if (prof_cnt < PROFILE_FILTER_N) prof_cnt++;
    uint32_t sum = 0;
    for (uint8_t i = 0; i < prof_cnt; i++) sum += prof_buf[i];
    return (uint16_t)(sum / prof_cnt);
}

/* PWM(мкс)->угол(рад) по активному профилю, мёртвая зона, кламп к упору. */
static float pwm_to_angle(uint16_t us)
{
    float center = (cfg.pwm_min + cfg.pwm_max) * 0.5f;
    float halfsp = (cfg.pwm_max - cfg.pwm_min) * 0.5f;
    if (us < cfg.pwm_min) us = cfg.pwm_min;
    if (us > cfg.pwm_max) us = cfg.pwm_max;
    float delta = (float)us - center;
    if (fabsf(delta) <= cfg.pwm_dead_zone) return 0.0f;
    float ang  = (delta / halfsp) * DEG2RAD(cur_max_angle());
    float safe = DEG2RAD(cfg.mech_pos_limit) - DEG2RAD(0.5f);
    if (ang >  safe) ang =  safe;
    if (ang < -safe) ang = -safe;
    return ang;
}

static uint8_t pwm_alive(void)
{ return (HAL_GetTick() - pwm_last_tick) <= cfg.pwm_timeout_ms; }

/* Ширина импульса -> номер профиля (0..2) с гистерезисом относительно текущего. */
static uint8_t classify_profile(uint16_t us, uint8_t cur)
{
    uint16_t lo = cfg.profile_thr_lo, hi = cfg.profile_thr_hi, h = cfg.profile_hyst;
    if (cur == 0) {
        if (us > hi + h) return 2;
        if (us > lo + h) return 1;
        return 0;
    }
    if (cur == 2) {
        if (us < lo - h) return 0;
        if (us < hi - h) return 1;
        return 2;
    }
    if (us < lo - h) return 0;
    if (us > hi + h) return 2;
    return 1;
}

/* Обновить активный профиль (антидребезг profile_debounce).
 * return 1 = профиль сменился (нужно переслать op_vel/op_acc). */
static uint8_t profile_update(void)
{
    static uint8_t cand = 0, cnt = 0;
    uint8_t c = classify_profile(profile_filtered(), active_profile);
    if (c == active_profile) { cand = c; cnt = 0; return 0; }
    if (c == cand) { if (++cnt >= cfg.profile_debounce) { active_profile = c; cnt = 0; return 1; } }
    else { cand = c; cnt = 1; }
    return 0;
}

static void go_fault(fsm_state_t came_from, fault_reason_t reason)
{ return_state = came_from; fault_reason = reason; state = ST_FAULT; }

/* Доехать до угла в PP. 1=достигнуто, 0=таймаут, 2=за упор. */
static int reach_point(uint8_t id, float target_rad, uint32_t to_ms)
{
    rs_goto(id, target_rad);
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < to_ms) {
        float pos;
        if (rs_read_f(id, IDX_MECH_POS, &pos, 20)) {
            if (fabsf(pos) > DEG2RAD(cfg.mech_pos_limit)) return 2;
            if (fabsf(pos - target_rad) <= cfg.pos_tol)   return 1;
        }
        led_update();
        HAL_Delay(5);
    }
    return 0;
}

/* =============================================================================
 *  9. СОСТОЯНИЯ FSM
 * ========================================================================== */
static void do_init(void)
{
    found_motor_id = 0;

    if (cfg.motor_id != 0) {
#if VERIFY_MOTOR_PRESENCE
        rs_obtain_id(cfg.motor_id); scan_got = 0;
        uint32_t t0 = HAL_GetTick();
        while (!scan_got && HAL_GetTick() - t0 < 50) { led_update(); }
        if (scan_got) found_motor_id = cfg.motor_id;
#else
        found_motor_id = cfg.motor_id;    /* быстрый старт: адрес известен */
#endif
    } else {
        for (uint8_t id = 1; id <= 127 && !found_motor_id; id++) {
            scan_got = 0;
            rs_obtain_id(id);
            uint32_t t0 = HAL_GetTick();
            while (!scan_got && HAL_GetTick() - t0 < 8) { }
            if (scan_got) found_motor_id = scan_hit_id ? scan_hit_id : id;
            led_update();
        }
    }

    if (!found_motor_id) { go_fault(ST_INIT, FR_NO_MOTOR); return; }

    rs_stop(found_motor_id, 1);
    HAL_Delay(10);
    motor_fault = 0;
    rs_enable_feedback(found_motor_id);
    HAL_Delay(50);

    float vbus = 0, mpos = 0;
    uint8_t ok = 1;
    ok &= rs_read_f(found_motor_id, IDX_VBUS,     &vbus, 100);
    ok &= rs_read_f(found_motor_id, IDX_MECH_POS, &mpos, 100);
    if (!ok) { dbg("INIT: no reply -> FR_COMMS\r\n"); go_fault(ST_INIT, FR_COMMS); return; }

    tlm.fresh = 0;
    uint32_t t0 = HAL_GetTick();
    while (!tlm.fresh && HAL_GetTick() - t0 < 200) { led_update(); }

    if (vbus < cfg.VBUS_min)                       { go_fault(ST_INIT, FR_VBUS); return; }
    if (tlm.fresh && tlm.temp > cfg.motorTemp_max) { go_fault(ST_INIT, FR_TEMP); return; }
    if (fabsf(mpos) > DEG2RAD(cfg.mech_pos_limit)) { go_fault(ST_INIT, FR_MECH_LIMIT); return; }

    dbg("INIT: OK\r\n");
    state = ST_SELFTEST;
}

static void do_selftest(void)
{
    uint8_t id = found_motor_id;
    int r;
    float a = DEG2RAD(cfg.selftest_angle);

    motor_fault = 0;
    rs_enter_pp(id, cfg.selftest_vel, cfg.selftest_acc, cfg.selftest_torque);
    HAL_Delay(50);                     /* дать приводу применить режим */

    dbg("SELFTEST: starting\r\n");

    r = reach_point(id, 0.0f, 4000);
    dbg("SELFTEST: point=0 result="); dbg_res(r);
    if (r == 2) { go_fault(ST_SELFTEST, FR_MECH_LIMIT); return; }
    if (r == 0) { go_fault(ST_SELFTEST, FR_SELFTEST);   return; }

    HAL_Delay(cfg.selftest_delay_ms);

    r = reach_point(id, a, 4000);
    dbg("SELFTEST: point=+angle result="); dbg_res(r);
    if (r == 2) { go_fault(ST_SELFTEST, FR_MECH_LIMIT); return; }
    if (r == 0) { go_fault(ST_SELFTEST, FR_SELFTEST);   return; }

    r = reach_point(id, -a, 6000);
    dbg("SELFTEST: point=-angle result="); dbg_res(r);
    if (r == 2) { go_fault(ST_SELFTEST, FR_MECH_LIMIT); return; }
    if (r == 0) { go_fault(ST_SELFTEST, FR_SELFTEST);   return; }

    reach_point(id, 0.0f, 4000);
    dbg("SELFTEST: return to zero, PASSED\r\n");

    selftest_retry = 0;
    state = ST_OPERATION;
}

/* OPERATION: неблокирующий цикл слежения. */
static void do_operation(void)
{
    dbg("--> ENTERED OPERATION\r\n");

    uint8_t  id   = found_motor_id;
    uint16_t freq = cfg.looptime_freq ? cfg.looptime_freq : 100;
    if (freq > 1000) freq = 1000;                 /* защита от period=0 */
    uint32_t period = 1000u / freq;
    uint32_t next   = HAL_GetTick();
    uint32_t read_t = HAL_GetTick();
    uint32_t op_start = HAL_GetTick();
    uint8_t  budget_reset_done = 0;
    uint8_t  vbus_low_cnt = 0;
#if DEBUG_UART
    uint32_t last_debug = HAL_GetTick();
#endif

    pwm_filter_reset();
    profile_filter_reset();
    active_profile = classify_profile(profile_filtered(), active_profile);
    motor_fault = 0;
    async_vbus_fresh = 0; async_mpos_fresh = 0;

    /* запуск CSP (run_mode+enable), в т.ч. повторный после OFF из FAULT */
    rs_enter_csp(id, cfg.torque_fdb_limit);

    /* бесступенчатый рестарт: стартовая цель = текущее положение (нет рывка) */
    { float p; if (rs_read_f(id, IDX_MECH_POS, &p, 30)) rs_goto(id, p); }

    /* грейс телеметрии: не считать связь потерянной сразу после входа */
    __disable_irq(); tlm_last_tick = HAL_GetTick(); __enable_irq();
    HAL_Delay(10);

    while (state == ST_OPERATION) {
        uint32_t now = HAL_GetTick();
        if ((int32_t)(now - next) < 0) { led_update(); continue; }
        next += period;

#if DEBUG_UART
        if (now - last_debug > 500) { last_debug = now; dbg("OPERATION\r\n"); }
#endif

        /* после здоровой работы — сбросить бюджет мягких восстановлений */
        if (!budget_reset_done && (now - op_start) > cfg.fault_reset_ms) {
            soft_recover_count = 0; budget_reset_done = 1;
        }

        /* failsafe ШИМ управления */
        if (!pwm_alive()) { go_fault(ST_OPERATION, FR_PWM_LOST); break; }

        /* смена профиля -> переслать op_vel/op_acc */
        if (profile_update()) rs_apply_op_profile(id);

        /* команда угла */
        rs_goto(id, pwm_to_angle(pwm_filtered(pwm_pulse_us)));

        /* --- атомарный снимок телеметрии --- */
        float    s_temp, s_torque; uint8_t s_fresh, s_mf; uint32_t s_last;
        __disable_irq();
        s_temp = tlm.temp; s_torque = tlm.torque; s_fresh = tlm.fresh;
        s_mf = motor_fault; s_last = tlm_last_tick;
        __enable_irq();

        if (s_fresh) {
            if (s_temp > cfg.motorTemp_max)             { go_fault(ST_OPERATION, FR_TEMP); break; }
            if (fabsf(s_torque) > cfg.torque_fdb_limit) { go_fault(ST_OPERATION, FR_TORQUE); break; }
        }
        if (s_mf)                             { go_fault(ST_OPERATION, FR_MOTOR_INTERNAL); break; }
        if ((now - s_last) > cfg.tlm_lost_ms) { go_fault(ST_OPERATION, FR_COMMS); break; }

        /* --- НЕблокирующий опрос VBUS/mechPos: запрос раз в 100 мс --- */
        if (now - read_t > 100) { read_t = now;
            rs_request_read(id, IDX_VBUS);
            rs_request_read(id, IDX_MECH_POS);
        }
        if (async_vbus_fresh) {
            __disable_irq(); float v = async_vbus; async_vbus_fresh = 0; __enable_irq();
            if (v < cfg.VBUS_min) { if (++vbus_low_cnt >= cfg.vbus_fault_debounce)
                                        { go_fault(ST_OPERATION, FR_VBUS); break; } }
            else vbus_low_cnt = 0;
        }
        if (async_mpos_fresh) {
            __disable_irq(); float m = async_mpos; async_mpos_fresh = 0; __enable_irq();
            if (fabsf(m) > DEG2RAD(cfg.mech_pos_limit)) { go_fault(ST_OPERATION, FR_MECH_LIMIT); break; }
        }
        led_update();
    }
}

/* --- FAULT: OFF + индивидуальное восстановление + защита от «шторма» --- */
static void do_fault(void)
{
    if (found_motor_id) rs_stop(found_motor_id, 0);   /* OFF */

    dbg("FAULT: entered, reason="); dbg_res((int)fault_reason);

    /* Бюджет мягких восстановлений: превышение -> защёлка. */
    if (!fault_is_hard(fault_reason)) {
        if (++soft_recover_count > cfg.fault_recover_max) {
            dbg("FAULT: soft budget exhausted -> latch\r\n");
            fault_latched = 1;
            while (1) { led_update(); HAL_Delay(10); }
        }
    }

    switch (fault_reason) {

    case FR_COMMS:
        for (uint8_t a = 0; a < cfg.comm_retry_attempts; a++) {
            uint32_t t0 = HAL_GetTick();
            while (HAL_GetTick() - t0 < cfg.comm_retry_delay_ms) { led_update(); HAL_Delay(5); }
            float v;
            if (found_motor_id && rs_read_f(found_motor_id, IDX_VBUS, &v, 30)) {
                rs_stop(found_motor_id, 1);
                dbg("FAULT: link restored -> INIT\r\n");
                state = ST_INIT;
                return;
            }
        }
        break;                                /* исчерпано -> защёлка */

    case FR_TEMP: {
        /* Удержание до остывания. Требуем СВЕЖИЕ кадры ОС; если привод замолчал
         * дольше tlm_lost_ms — безопасная защёлка вместо вечного зависания. */
        uint32_t reen = 0;
        if (found_motor_id) rs_enable_feedback(found_motor_id);
        __disable_irq(); tlm.fresh = 0; uint32_t last_fresh = HAL_GetTick(); __enable_irq();
        while (1) {
            led_update();
            HAL_Delay(20);
            if (found_motor_id && (HAL_GetTick() - reen) > 500) {
                reen = HAL_GetTick(); rs_enable_feedback(found_motor_id);
            }
            uint8_t fr; float t;
            __disable_irq(); fr = tlm.fresh; t = tlm.temp; tlm.fresh = 0; __enable_irq();
            if (fr) { last_fresh = HAL_GetTick(); if (t < cfg.fr_temp_retry) break; }
            if ((HAL_GetTick() - last_fresh) > cfg.tlm_lost_ms) {
                dbg("FAULT: telemetry lost during cooldown -> latch\r\n");
                fault_latched = 1;
                while (1) { led_update(); HAL_Delay(10); }
            }
        }
        if (found_motor_id) rs_stop(found_motor_id, 1);
        state = return_state;
        return;
    }

    case FR_TORQUE: {
        uint32_t t0 = HAL_GetTick();
        while (HAL_GetTick() - t0 < cfg.restart_delay_ms) { led_update(); HAL_Delay(5); }
        if (found_motor_id) rs_stop(found_motor_id, 1);
        selftest_retry = 0;
        state = ST_SELFTEST;
        return;
    }

    case FR_SELFTEST:
        if (selftest_retry < cfg.selftest_retry_attempts) {
            selftest_retry++;
            if (found_motor_id) rs_stop(found_motor_id, 1);
            state = ST_SELFTEST;
            return;
        }
        break;                                /* исчерпано -> защёлка */

    default:                                  /* FR_NO_MOTOR / FR_VBUS /
                                                 FR_MECH_LIMIT / FR_MOTOR_INTERNAL /
                                                 FR_PWM_LOST */
        break;
    }

    dbg("FAULT: latched\r\n");
    fault_latched = 1;
    while (1) { led_update(); HAL_Delay(10); }   /* защёлка: красный постоянно */
}

/* =============================================================================
 *  10. ТОЧКИ ВХОДА
 * ========================================================================== */
void app_setup(void)
{
    CAN_FilterTypeDef f = {0};
    f.FilterBank = 0;
    f.FilterMode = CAN_FILTERMODE_IDMASK;
    f.FilterScale = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh = 0; f.FilterIdLow = 0;
    f.FilterMaskIdHigh = 0; f.FilterMaskIdLow = 0;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation = ENABLE;
    HAL_CAN_ConfigFilter(&hcan, &f);

    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan,
        CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF);

    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

    pwm_last_tick     = HAL_GetTick();
    profile_last_tick = HAL_GetTick();

    rgb_t off[LED_MAX] = {{0}};
    ws2812_show_n(off, cfg.led_count);

    dbg("\r\n=== RobStride PWM-CAN bridge v5 ===\r\n");
    state = ST_INIT;
}

void app_loop(void)
{
    switch (state) {
    case ST_INIT:      do_init();      break;
    case ST_SELFTEST:  do_selftest();  break;
    case ST_OPERATION: do_operation(); break;
    case ST_FAULT:     do_fault();     break;
    }
    led_update();
}

/* В main.c:
 *   app_setup();
 *   while (1) { app_loop(); }
 */
