/**
 * logger.c
 * BMI270 + GPS binary data logger for STM32L476RGT6
 * Uses the official Bosch BMI270 SensorAPI (github.com/boschsensortec/BMI270_SensorAPI)
 *
 * Record format (all little-endian):
 *   [0xAA 0x55] [TYPE:u8] [TIMESTAMP_US:u64] [PAYLOAD:N bytes] [CRC8:u8]
 *
 * Record types:
 *   0x01  IMU  — int16[3] accel_raw, int16[3] gyro_raw  (12 bytes payload)
 *   0x02  GPS  — float lat, lon, alt, speed, hdop        (20 bytes payload)
 *   0xFF  META — session header string                   (variable)
 *
 * NOTE: timestamp is now 8 bytes (u64). Update parse_log.py:
 *   struct.unpack_from("<Q", data, i+3)  and adjust header size to 11+plen
 *
 * Assumptions:
 *   - BMI270 on SPI1, CS = GPIOA pin 4
 *   - BMI270 INT1 routed to a GPIO with EXTI configured as rising edge
 *   - SD card via FatFs (CubeMX-generated)
 *   - GPS UART parsed elsewhere; caller fills GpsFix and sets g_gps_ready
 *   - Bare-metal superloop (no RTOS); TIM6 drives IMU drain flag at ODR,
 *     TIM7 drives SD flush flag at 1 Hz
 *
 * Bosch SensorAPI files required (add to your project):
 *   bmi2.c / bmi2.h
 *   bmi270.c / bmi270.h
 *   bmi2_defs.h
 */

#include "main.h"
#include "fatfs.h"
#include <string.h>
#include <stdio.h>

/* Bosch SensorAPI headers */
#include "bmi270.h"

/* ------------------------------------------------------------------ */
/*  Hardware configuration — adapt to your board                       */
/* ------------------------------------------------------------------ */

#define BMI270_I2C_HANDLE       hi2c2       /* extern in main.h */

/*
 * I2C address: the BMI270 SDO/SA0 pin selects between two addresses.
 *   SDO low  -> 0x68  (BMI2_I2C_PRIM_ADDR)
 *   SDO high -> 0x69  (BMI2_I2C_SEC_ADDR)
 * The HAL wants the address left-shifted by 1 (8-bit form).
 */
#define BMI270_I2C_ADDR         (BMI2_I2C_PRIM_ADDR << 1)   /* 0xD0 */

extern I2C_HandleTypeDef        BMI270_I2C_HANDLE;

/* ------------------------------------------------------------------ */
/*  IMU configuration                                                  */
/* ------------------------------------------------------------------ */

/* Accelerometer: 400 Hz, ±8 g, performance (CIC) filter */
#define IMU_ACC_ODR             BMI2_ACC_ODR_400HZ
#define IMU_ACC_RANGE           BMI2_ACC_RANGE_8G
#define IMU_ACC_BW              BMI2_ACC_NORMAL_AVG4   /* normal mode */

/* Gyroscope: 400 Hz, ±1000 dps, performance filter */
#define IMU_GYR_ODR             BMI2_GYR_ODR_400HZ
#define IMU_GYR_RANGE           BMI2_GYR_RANGE_1000
#define IMU_GYR_BW              BMI2_GYR_NORMAL_MODE

/*
 * FIFO configuration.
 *
 * At 400 Hz with acc+gyr header frames (7 bytes each, interleaved):
 *   400 Hz x 14 B = 5600 B/s
 * The BMI270 FIFO is 6144 bytes — fits just over 1 second.
 * We drain on every watermark interrupt so the FIFO never actually fills.
 * 512 B is enough headroom to absorb interrupt latency jitter.
 */
#define FIFO_RAW_BUF_SIZE       512
#define FIFO_FRAME_COUNT        32          /* max frames to parse per drain */

/* ODR period in microseconds — must match IMU_ACC_ODR / IMU_GYR_ODR above */
#define ODR_PERIOD_US           2500U       /* 1/400 Hz = 2500 µs */

/* ------------------------------------------------------------------ */
/*  Logging configuration                                              */
/* ------------------------------------------------------------------ */

#define LOG_FILENAME            "log.bin"
#define SD_WRITE_INTERVAL_MS    1000

/*
 * Double-buffer for IMU records.
 * At 400 Hz x 20 B/record = 8000 B/s; one half holds one second comfortably.
 */
#define IMU_HALF_BUF_SIZE       8192
#define IMU_BUF_SIZE            (IMU_HALF_BUF_SIZE * 2)

/* ------------------------------------------------------------------ */
/*  Types                                                              */
/* ------------------------------------------------------------------ */

typedef struct {
    float lat, lon, alt;    /* degrees, degrees, metres */
    float speed;            /* m/s */
    float hdop;
    uint8_t fix_valid;
} GpsFix;

/* ------------------------------------------------------------------ */
/*  Globals                                                            */
/* ------------------------------------------------------------------ */

/* Bosch API device descriptor */
static struct bmi2_dev          bmi2_dev;

/* FIFO raw byte buffer + descriptor */
static uint8_t                  fifo_raw[FIFO_RAW_BUF_SIZE];
static struct bmi2_fifo_frame   fifo_desc;

/*
 * Double-buffer: the superloop writes IMU frames into the active half;
 * on each 1 Hz SD flush the halves are swapped so the write never races
 * with the SD write.  The swap itself is done inside a critical section
 * (IRQ disabled for two assignments) so it is safe even though the IMU
 * drain runs from main() context while the swap is triggered by TIM7 ISR.
 */
static uint8_t                  imu_dbl_buf[IMU_BUF_SIZE];
static volatile uint16_t        imu_buf_pos[2] = {0, 0};
static volatile uint8_t         imu_active_buf = 0;  /* 0 or 1 */

/*
 * Flags set by ISRs, cleared by the superloop.
 * Declared volatile so the compiler never caches them in a register.
 *
 * flag_imu_drain : set by BMI270 INT1 EXTI ISR — drain FIFO now
 * flag_sd_flush  : set by TIM7 update ISR     — flush buffer + write GPS
 */
volatile uint8_t                flag_imu_drain = 0;
volatile uint8_t                flag_sd_flush  = 0;

/* Set by your GPS UART callback */
volatile GpsFix                 g_gps;
volatile uint8_t                g_gps_ready = 0;

/* FatFs */
static FIL                      log_file;
static FATFS                    fat_fs;

/* ------------------------------------------------------------------ */
/*  Microsecond timestamp — 64-bit, overflow-safe                     */
/*                                                                     */
/*  Architecture:                                                      */
/*    DWT->CYCCNT  : free-running 32-bit cycle counter (hardware)     */
/*                   wraps every 2^32 / 80 MHz ≈ 53.7 s               */
/*    ts_epoch     : software 32-bit counter, incremented once per    */
/*                   DWT wraparound by TIM2's update (overflow) ISR   */
/*                                                                     */
/*  Full 64-bit timestamp = ((uint64_t)ts_epoch << 32) | CYCCNT       */
/*  Converted to µs       = timestamp_cycles / cycles_per_us          */
/*                                                                     */
/*  TIM2 on STM32L476 is a 32-bit timer clocked from PCLK1.           */
/*  We set it to count at SystemCoreClock (via the APB prescaler) and  */
/*  its ARR (auto-reload register) to 0xFFFFFFFF so it overflows at    */
/*  exactly the same period as CYCCNT — once every ~53.7 s.            */
/*  Its update interrupt bumps ts_epoch.                               */
/*                                                                     */
/*  CubeMX setup required for TIM2:                                    */
/*    - Clock source   : Internal Clock                                */
/*    - Prescaler (PSC): 0  (no division — runs at SystemCoreClock)   */
/*    - Counter period : 0xFFFFFFFF                                    */
/*    - Auto-reload    : Enable                                        */
/*    - Update interrupt: Enable in NVIC                               */
/*  Then call MX_TIM2_Init() from main() before logger_start().        */
/*  The TIM2 handle (htim2) must be declared extern in main.h.         */
/* ------------------------------------------------------------------ */

extern TIM_HandleTypeDef htim2;

/* Incremented by TIM2 update ISR on every 32-bit wraparound */
static volatile uint32_t ts_epoch = 0;

/**
 * timestamp_init()
 *
 * Enables the DWT cycle counter (the hardware source of the low 32 bits)
 * and starts TIM2 in interrupt mode (the source of the high 32 bits).
 *
 * Register-level detail:
 *
 *  CoreDebug->DEMCR  (0xE000EDFC)
 *    TRCENA [24] — master enable for the CoreSight trace subsystem.
 *    Must be set before any DWT register is written; DWT is entirely
 *    gated behind this bit.
 *
 *  DWT->CYCCNT  (0xE0001004)
 *    Free-running 32-bit up-counter, increments every CPU clock cycle.
 *    Writing zero here re-synchronises it with TIM2's counter, which we
 *    also reset to zero, so both start from the same origin.
 *
 *  DWT->CTRL  (0xE0001000)
 *    CYCCNTENA [0] — enables CYCCNT.  Without this bit the counter
 *    register is frozen regardless of TRCENA.
 *
 *  HAL_TIM_Base_Start_IT(&htim2)
 *    Writes TIM2->CR1.CEN=1 to start the counter and enables the
 *    TIM2 update interrupt in the NVIC, so TIM2_IRQHandler fires
 *    whenever TIM2->CNT rolls over from 0xFFFFFFFF to 0.
 */
static void timestamp_init(void)
{
    /* --- DWT cycle counter --- */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  /* unlock DWT */
    DWT->CYCCNT       = 0;                             /* reset      */
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;       /* start      */

    /* --- TIM2 overflow interrupt ---
     * TIM2 must already be configured by MX_TIM2_Init() with PSC=0
     * and ARR=0xFFFFFFFF before this call.                          */
    TIM2->CNT = 0;                      /* align TIM2 with CYCCNT   */
    HAL_TIM_Base_Start_IT(&htim2);      /* start + enable update IRQ */
}

/**
 * TIM2 update ISR — called once every ~53.7 s when TIM2->CNT wraps.
 *
 * Wire this up in stm32l4xx_it.c:
 *
 *   void TIM2_IRQHandler(void)
 *   {
 *       HAL_TIM_IRQHandler(&htim2);   // clears the UIF flag
 *   }
 *
 * HAL then calls HAL_TIM_PeriodElapsedCallback(), which you implement:
 *
 *   void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 *   {
 *       if (htim->Instance == TIM2)
 *           timestamp_overflow_irq();
 *   }
 */
void timestamp_overflow_irq(void)
{
    ts_epoch++;
}

/**
 * get_timestamp_us()
 *
 * Returns a 64-bit microsecond timestamp that never wraps in practice.
 *
 * The read must be atomic with respect to the overflow ISR to avoid a
 * race where CYCCNT has just wrapped to near-zero but ts_epoch has not
 * yet been incremented (or vice versa in the ISR window).
 *
 * We use the standard "double-read" technique:
 *   1. Read ts_epoch  (call it e1)
 *   2. Read CYCCNT    (call it c)
 *   3. Read ts_epoch  again (call it e2)
 *   If e1 == e2, no overflow happened between steps 1 and 2, so
 *   the pair (e1, c) is consistent.
 *   If e1 != e2, an overflow fired between the two epoch reads;
 *   we use e2 with c (c is already on the new epoch side).
 */
static inline uint64_t get_timestamp_us(void)
{
    uint32_t epoch, cyccnt;
    uint32_t epoch2;

    do {
        epoch  = ts_epoch;
        cyccnt = DWT->CYCCNT;
        epoch2 = ts_epoch;
    } while (epoch != epoch2);

    uint64_t cycles = ((uint64_t)epoch << 32) | cyccnt;
    return cycles / (SystemCoreClock / 1000000U);
}

/* ------------------------------------------------------------------ */
/*  CRC-8 (poly 0x31, init 0xFF)                                      */
/* ------------------------------------------------------------------ */

static uint8_t crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0xFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80) ? ((crc << 1) ^ 0x31) : (crc << 1);
    }
    return crc;
}

/* ------------------------------------------------------------------ */
/*  Bosch SensorAPI HAL bindings                                       */
/* ------------------------------------------------------------------ */

/**
 * bmi2_i2c_read — I2C read callback for the Bosch SensorAPI.
 *
 * I2C register reads on the BMI270 are a standard two-phase transaction:
 *   Phase 1 (write): send the register address byte.
 *   Phase 2 (read):  clock out len data bytes.
 *
 * HAL_I2C_Mem_Read() wraps both phases in one call:
 *   - DevAddress : 8-bit address (7-bit addr << 1), set by BMI270_I2C_ADDR
 *   - MemAddress : the register address (1 byte, I2C_MEMADD_SIZE_8BIT)
 *   - pData/Size : destination buffer and byte count
 *
 * No dummy byte is needed — that was SPI-specific. I2C has no such quirk.
 *
 * intf_ptr carries the I2C address; the SensorAPI passes whatever we put
 * in bmi2_dev.intf_ptr when we call bmi270_init(). We use it here so this
 * function needs no global state of its own.
 */
static BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t  reg_addr,
                                            uint8_t *reg_data,
                                            uint32_t len,
                                            void    *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t *)intf_ptr;
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(
        &BMI270_I2C_HANDLE,
        dev_addr,
        reg_addr,
        I2C_MEMADD_SIZE_8BIT,
        reg_data,
        (uint16_t)len,
        50);
    return (st == HAL_OK) ? BMI2_INTF_RET_SUCCESS : BMI2_E_COM_FAIL;
}

/**
 * bmi2_i2c_write — I2C write callback for the Bosch SensorAPI.
 *
 * HAL_I2C_Mem_Write() sends the register address followed immediately
 * by the data bytes in a single I2C transaction — exactly what the
 * BMI270 expects for register writes and config-file burst writes.
 */
static BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t        reg_addr,
                                              const uint8_t *reg_data,
                                              uint32_t       len,
                                              void          *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t *)intf_ptr;
    HAL_StatusTypeDef st = HAL_I2C_Mem_Write(
        &BMI270_I2C_HANDLE,
        dev_addr,
        reg_addr,
        I2C_MEMADD_SIZE_8BIT,
        (uint8_t *)reg_data,
        (uint16_t)len,
        50);
    return (st == HAL_OK) ? BMI2_INTF_RET_SUCCESS : BMI2_E_COM_FAIL;
}

/** Microsecond busy-wait — required by the SensorAPI during config-file load. */
static void bmi2_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = period * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks);
}

/* ------------------------------------------------------------------ */
/*  BMI270 initialisation via SensorAPI                                */
/* ------------------------------------------------------------------ */

static int8_t bmi270_setup(void)
{
    int8_t rslt;

    /*
     * Wire up the SensorAPI device descriptor for I2C.
     *
     * intf_ptr points to a variable holding the 8-bit I2C address.
     * The read/write callbacks receive this pointer and use it directly,
     * so they work correctly without any global address state.
     *
     * read_write_len caps the largest single I2C transaction. 32 bytes is
     * the safe default for I2C (some masters have shorter hardware buffers);
     * increase to 128 or 256 if your I2C peripheral and DMA support it —
     * it affects FIFO burst-read speed during the config-file load.
     */
    static uint8_t i2c_addr = BMI270_I2C_ADDR;
    bmi2_dev.intf            = BMI2_I2C_INTF;
    bmi2_dev.intf_ptr        = &i2c_addr;
    bmi2_dev.read            = bmi2_i2c_read;
    bmi2_dev.write           = bmi2_i2c_write;
    bmi2_dev.delay_us        = bmi2_delay_us;
    bmi2_dev.read_write_len  = 32;
    bmi2_dev.config_file_ptr = NULL;   /* SensorAPI loads the blob internally */

    /*
     * bmi270_init() verifies the chip ID, loads the mandatory 8 KB config
     * file via burst I2C writes, and polls the internal status bit to
     * confirm successful load — all handled by the SensorAPI.
     */
    rslt = bmi270_init(&bmi2_dev);
    if (rslt != BMI2_OK) return rslt;

    /* ---- Accelerometer config ---- */
    struct bmi2_sens_config acc_cfg = {0};
    acc_cfg.type                = BMI2_ACCEL;
    acc_cfg.cfg.acc.odr         = IMU_ACC_ODR;
    acc_cfg.cfg.acc.range       = IMU_ACC_RANGE;
    acc_cfg.cfg.acc.bwp         = IMU_ACC_BW;
    acc_cfg.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

    rslt = bmi2_set_sensor_config(&acc_cfg, 1, &bmi2_dev);
    if (rslt != BMI2_OK) return rslt;

    /* ---- Gyroscope config ---- */
    struct bmi2_sens_config gyr_cfg = {0};
    gyr_cfg.type                = BMI2_GYRO;
    gyr_cfg.cfg.gyr.odr         = IMU_GYR_ODR;
    gyr_cfg.cfg.gyr.range       = IMU_GYR_RANGE;
    gyr_cfg.cfg.gyr.bwp         = IMU_GYR_BW;
    gyr_cfg.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    gyr_cfg.cfg.gyr.noise_perf  = BMI2_PERF_OPT_MODE;

    rslt = bmi2_set_sensor_config(&gyr_cfg, 1, &bmi2_dev);
    if (rslt != BMI2_OK) return rslt;

    /* ---- Enable accel + gyro ---- */
    uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };
    rslt = bmi2_sensor_enable(sens_list, 2, &bmi2_dev);
    if (rslt != BMI2_OK) return rslt;

    /* ---- FIFO: enable acc + gyr frames with headers ---- */
    rslt = bmi2_set_fifo_config(
        BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN | BMI2_FIFO_HEADER_EN,
        BMI2_ENABLE, &bmi2_dev);
    if (rslt != BMI2_OK) return rslt;

    /*
     * Watermark at 1 frame (minimum) so INT1 fires as soon as one
     * acc+gyr pair is ready — lowest possible latency for timestamps.
     */
    rslt = bmi2_set_fifo_wm(16, &bmi2_dev);
    if (rslt != BMI2_OK) return rslt;

    /* ---- INT1: watermark interrupt, active-high, push-pull ---- */
    struct bmi2_int_pin_config int_cfg = {0};
    int_cfg.pin_type               = BMI2_INT1;
    int_cfg.int_latch              = BMI2_INT_NON_LATCH;
    int_cfg.pin_cfg[0].output_en   = BMI2_INT_OUTPUT_ENABLE;
    int_cfg.pin_cfg[0].od          = BMI2_INT_PUSH_PULL;
    int_cfg.pin_cfg[0].lvl         = BMI2_INT_ACTIVE_HIGH;
    int_cfg.pin_cfg[0].input_en    = BMI2_INT_INPUT_DISABLE;
    rslt = bmi2_set_int_pin_config(&int_cfg, &bmi2_dev);
    if (rslt != BMI2_OK) return rslt;

    rslt = bmi2_map_data_int(BMI2_FWM_INT, BMI2_INT1, &bmi2_dev);
    return rslt;
}

/* ------------------------------------------------------------------ */
/*  EXTI ISR glue                                                      */
/* ------------------------------------------------------------------ */

/* ------------------------------------------------------------------ */
/*  ISR handlers — keep these short; no I2C or SD calls inside        */
/* ------------------------------------------------------------------ */

/**
 * bmi270_drdy_irq_handler()
 *
 * Called from HAL_GPIO_EXTI_Callback in stm32l4xx_it.c when INT1 fires:
 *
 *   void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 *   {
 *       if (GPIO_Pin == BMI270_INT1_PIN)
 *           bmi270_drdy_irq_handler();
 *   }
 *
 * Just sets a flag; the superloop does the actual FIFO drain.
 * This keeps I2C transactions entirely out of interrupt context.
 */
void bmi270_drdy_irq_handler(void)
{
    flag_imu_drain = 1;
}

/**
 * logger_tim7_irq_handler()
 *
 * Called from HAL_TIM_PeriodElapsedCallback in stm32l4xx_it.c:
 *
 *   void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 *   {
 *       if (htim->Instance == TIM2)  timestamp_overflow_irq();
 *       if (htim->Instance == TIM7)  logger_tim7_irq_handler();
 *   }
 *
 * TIM7 CubeMX setup:
 *   - Clock source   : Internal Clock
 *   - Prescaler (PSC): (SystemCoreClock / 1000) - 1   e.g. 79999 at 80 MHz
 *   - Counter period : 1000 - 1  (= 999)
 *   -> overflow every 1 s exactly
 *   - Update interrupt: enabled in NVIC
 *
 * The buffer swap happens here (two volatile assignments behind
 * __disable_irq) rather than in the superloop so the swap is
 * always synchronous with the 1 Hz tick, not subject to loop jitter.
 */
void logger_tim7_irq_handler(void)
{
    /* Swap active buffer half atomically */
    __disable_irq();
    uint8_t full = imu_active_buf;
    imu_active_buf = full ^ 1u;
    __enable_irq();

    /* Signal superloop to flush 'full' half and write GPS.
     * Encode the half index in the upper nibble so the superloop
     * knows which half to drain without needing a separate variable. */
    flag_sd_flush = 0x80u | full;   /* bit7=pending, bits[0]=half index */
}

/* ------------------------------------------------------------------ */
/*  Record serialisation                                               */
/* ------------------------------------------------------------------ */

/*
 * Record layout with 64-bit timestamp:
 *   [0]    0xAA  sync high
 *   [1]    0x55  sync low
 *   [2]    type
 *   [3-10] timestamp_us (u64 LE, 8 bytes)
 *   [11..] payload (plen bytes)
 *   [11+plen] CRC8 over bytes [2]..[10+plen]
 *
 * Total record size = 12 + plen bytes.
 */
static uint16_t build_record(uint8_t *dst, uint8_t type, uint64_t ts_us,
                              const uint8_t *payload, uint8_t plen)
{
    dst[0]  = 0xAA;
    dst[1]  = 0x55;
    dst[2]  = type;
    dst[3]  = (uint8_t)(ts_us);
    dst[4]  = (uint8_t)(ts_us >>  8);
    dst[5]  = (uint8_t)(ts_us >> 16);
    dst[6]  = (uint8_t)(ts_us >> 24);
    dst[7]  = (uint8_t)(ts_us >> 32);
    dst[8]  = (uint8_t)(ts_us >> 40);
    dst[9]  = (uint8_t)(ts_us >> 48);
    dst[10] = (uint8_t)(ts_us >> 56);
    memcpy(&dst[11], payload, plen);
    dst[11 + plen] = crc8(&dst[2], 9 + plen);   /* type + 8-byte ts + payload */
    return 12 + plen;
}

/**
 * Encode one bmi2_sens_data frame into the active double-buffer half.
 * bmi2_sens_data.acc / .gyr hold int16 raw counts straight from the FIFO
 * parser — no scaling applied here; the Python parser does that offline.
 */
static void buffer_imu_frame(const struct bmi2_sens_data *d, uint64_t ts_us)
{
    uint8_t  ab  = imu_active_buf;
    uint8_t *buf = &imu_dbl_buf[ab * IMU_HALF_BUF_SIZE];
    uint16_t pos = imu_buf_pos[ab];

    if (pos + 24u > IMU_HALF_BUF_SIZE) return;  /* overrun guard (12 hdr + 12 payload) */

    uint8_t payload[12];
    payload[0]  = (uint8_t)(d->acc.x);        payload[1]  = (uint8_t)(d->acc.x >> 8);
    payload[2]  = (uint8_t)(d->acc.y);        payload[3]  = (uint8_t)(d->acc.y >> 8);
    payload[4]  = (uint8_t)(d->acc.z);        payload[5]  = (uint8_t)(d->acc.z >> 8);
    payload[6]  = (uint8_t)(d->gyr.x);        payload[7]  = (uint8_t)(d->gyr.x >> 8);
    payload[8]  = (uint8_t)(d->gyr.y);        payload[9]  = (uint8_t)(d->gyr.y >> 8);
    payload[10] = (uint8_t)(d->gyr.z);        payload[11] = (uint8_t)(d->gyr.z >> 8);

    imu_buf_pos[ab] += build_record(buf + pos, 0x01, ts_us, payload, 12);
}

static int8_t bmi2_extract_accel_gyro(
    struct bmi2_sens_data *out,
    uint16_t *frame_count,
    struct bmi2_fifo_frame *fifo,
    const struct bmi2_dev *dev)
{
    int8_t rslt;

    /* Temporary buffers */
    struct bmi2_sens_data acc_frames[FIFO_FRAME_COUNT];
    struct bmi2_sens_data gyr_frames[FIFO_FRAME_COUNT];

    uint16_t acc_count = *frame_count;
    uint16_t gyr_count = *frame_count;

    /* Make copies of FIFO descriptor because extract functions mutate it */
    struct bmi2_fifo_frame fifo_acc = *fifo;
    struct bmi2_fifo_frame fifo_gyr = *fifo;

    /* Extract accel */
    rslt = bmi2_extract_accel(acc_frames, &acc_count, &fifo_acc, dev);
    if (rslt != BMI2_OK) return rslt;

    /* Extract gyro */
    rslt = bmi2_extract_gyro(gyr_frames, &gyr_count, &fifo_gyr, dev);
    if (rslt != BMI2_OK) return rslt;

    /* Use the minimum count to keep them aligned */
    uint16_t n = (acc_count < gyr_count) ? acc_count : gyr_count;
    if (n > *frame_count) n = *frame_count;

    for (uint16_t i = 0; i < n; i++) {
        out[i].acc = acc_frames[i].acc;
        out[i].gyr = gyr_frames[i].gyr;
    }

    *frame_count = n;
    return BMI2_OK;
}

static uint16_t build_gps_record(uint8_t *dst, const GpsFix *g, uint64_t ts_us)
{
    uint8_t payload[20];
    memcpy(&payload[0],  &g->lat,   4);
    memcpy(&payload[4],  &g->lon,   4);
    memcpy(&payload[8],  &g->alt,   4);
    memcpy(&payload[12], &g->speed, 4);
    memcpy(&payload[16], &g->hdop,  4);
    return build_record(dst, 0x02, ts_us, payload, 20);
}

/* ------------------------------------------------------------------ */
/*  SD helpers                                                         */
/* ------------------------------------------------------------------ */

static uint8_t sd_open_log(void)
{
    if (f_mount(&fat_fs, "", 1) != FR_OK)                   return 1;
    if (f_open(&log_file, LOG_FILENAME,
               FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)       return 2;

    char meta[] = "BMI270+GPS logger v2.0 (Bosch SensorAPI + FIFO)";
    uint8_t rec[76];   /* 12 byte header + 48 byte string + 1 CRC = 61; 76 is safe */
    uint16_t rlen = build_record(rec, 0xFF, get_timestamp_us(),
                                 (uint8_t *)meta, (uint8_t)strlen(meta));
    UINT bw;
    f_write(&log_file, rec, rlen, &bw);
    return 0;
}

static void sd_flush_half(uint8_t half)
{
    uint16_t len = imu_buf_pos[half];
    if (len == 0) return;
    UINT bw;
    f_write(&log_file, &imu_dbl_buf[half * IMU_HALF_BUF_SIZE], len, &bw);
    imu_buf_pos[half] = 0;
}

static void sd_write_gps(const GpsFix *gps, uint64_t ts_us)
{
    uint8_t  rec[32];
    uint16_t rlen = build_gps_record(rec, gps, ts_us);
    UINT bw;
    f_write(&log_file, rec, rlen, &bw);
}

/* ------------------------------------------------------------------ */
/*  Superloop handlers (called from main loop when flags are set)     */
/* ------------------------------------------------------------------ */

/**
 * logger_drain_imu()
 *
 * Called by the superloop when flag_imu_drain is set.
 * Drains whatever is in the FIFO and encodes it into the active
 * double-buffer half.  Must not be called from ISR context because
 * it performs blocking I2C transactions.
 */
static void logger_drain_imu(void)
{
	uint16_t int_status = 0;
	bmi2_get_int_status(&int_status, &bmi2_dev);
    while (1) {
        uint16_t fifo_len = 0;

        if (bmi2_get_fifo_length(&fifo_len, &bmi2_dev) != BMI2_OK)
        	printf("failed to get length\n");
            return;
        print("fifo length: %i\n", fifo_len);
        if (fifo_len == 0)
            break;  // FIFO fully drained

        if (fifo_len > FIFO_RAW_BUF_SIZE)
            fifo_len = FIFO_RAW_BUF_SIZE;

        fifo_desc.data   = fifo_raw;
        fifo_desc.length = fifo_len;

        if (bmi2_read_fifo_data(&fifo_desc, &bmi2_dev) != BMI2_OK)
        	printf("failed to read\n");
            return;

        struct bmi2_sens_data frames[FIFO_FRAME_COUNT];
        uint16_t frame_count = FIFO_FRAME_COUNT;

        if (bmi2_extract_accel_gyro(frames, &frame_count,
                                    &fifo_desc, &bmi2_dev) != BMI2_OK)
        	printf("failed to extract data\n");
            return;

        uint64_t ts_now = get_timestamp_us();

        for (uint16_t i = 0; i < frame_count; i++) {
            uint64_t frame_ts =
                ts_now - (frame_count - 1u - i) * ODR_PERIOD_US;

            buffer_imu_frame(&frames[i], frame_ts);
        }
    }
    uint16_t fifo_len;
    bmi2_get_fifo_length(&fifo_len, &bmi2_dev);
    printf("FIFO after drain: %u\n", fifo_len);
}

/**
 * logger_do_sd_flush()
 *
 * Called by the superloop when flag_sd_flush is set (by TIM7 ISR).
 * The half index to flush is encoded in the flag value set by the ISR
 * (bit 7 = pending, bits[0] = half index) so we never need a separate
 * shared variable.
 */
static void logger_do_sd_flush(uint8_t flag_val)
{
    uint8_t full_half = flag_val & 0x01u;

    /* Drain the half the ISR already swapped out */
    sd_flush_half(full_half);

    /* Snapshot and write GPS if a fresh fix arrived */
    if (g_gps_ready) {
        GpsFix snap;
        __disable_irq();
        snap        = g_gps;
        g_gps_ready = 0;
        __enable_irq();
        sd_write_gps(&snap, get_timestamp_us());
    }
    printf("flush sd\n");

    f_sync(&log_file);
}

/* ------------------------------------------------------------------ */
/*  Entry point + superloop                                            */
/* ------------------------------------------------------------------ */

/**
 * logger_start()
 *
 * Initialises everything and then runs the superloop forever.
 * Call this at the end of main() after all MX_xxx_Init() calls.
 *
 * Superloop priority:
 *   IMU drain is checked first on every iteration so FIFO data is
 *   never stale for more than one loop pass (~few µs at most).
 *   SD flush is checked second; it takes ~10 ms but only triggers
 *   once per second so it does not meaningfully delay IMU drains —
 *   during the SD write, any INT1 pulses that arrive will set
 *   flag_imu_drain again and be serviced on the very next iteration.
 */
void logger_start(void)
{
    timestamp_init();


    if (bmi270_setup() != BMI2_OK) Error_Handler();
    printf("bmi270 setup\n");
    if (sd_open_log()  != 0)       Error_Handler();
    printf("sd card setup\n");

    /* Superloop */
    for (;;) {
    	if (HAL_GPIO_ReadPin(ACC2_INT_GPIO_Port, ACC2_INT_Pin)) {
    	    printf("INT1 HIGH\n");
    	}
        /* ---- IMU drain (fires at ~400 Hz via INT1 EXTI) ---- */
        if (flag_imu_drain) {
            flag_imu_drain = 0;     /* clear before drain so a new INT1
                                       during drain is not lost */
            logger_drain_imu();
        }
        uint16_t fifo_len;
        bmi2_get_fifo_length(&fifo_len, &bmi2_dev);
        printf("Fifo length: %i\n", fifo_len);

        /* ---- SD flush (fires at 1 Hz via TIM7) ---- */
        uint8_t sf = flag_sd_flush;
        if (sf & 0x80u) {
            flag_sd_flush = 0;      /* clear atomically before the write */
            logger_do_sd_flush(sf);
        }
    }
}
