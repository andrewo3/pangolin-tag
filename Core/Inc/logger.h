#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/*  Types (must be visible to other modules)                           */
/* ------------------------------------------------------------------ */

typedef struct {
    float lat, lon, alt;    /* degrees, degrees, metres */
    float speed;            /* m/s */
    float hdop;
    uint8_t fix_valid;
} GpsFix;

/* ------------------------------------------------------------------ */
/*  Global variables (extern, defined in logger.c)                     */
/* ------------------------------------------------------------------ */

extern volatile GpsFix g_gps;
extern volatile uint8_t g_gps_ready;

/* ------------------------------------------------------------------ */
/*  Public API                                                        */
/* ------------------------------------------------------------------ */

/* Start the logger system (init + tasks) */
void logger_start(void);

/* ISR hooks (must be callable from interrupt files) */
void bmi270_drdy_irq_handler(void);
void timestamp_overflow_irq(void);

/* FreeRTOS tasks (optional to expose, but useful for debugging/manual creation) */
void Task_IMU(void *arg);
void Task_SDFlush(void *arg);

#endif /* LOGGER_H */
