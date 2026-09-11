#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include <stdio.h>

#include "pin_config.h"

/* Shared global variables (defined here, declared extern in robot.h) */
volatile long RCOUNT = 0;
volatile long LCOUNT = 0;
long RCOUNT_1 = 0;
long LCOUNT_1 = 0;
long RCOUNT_2 = 0;
long LCOUNT_2 = 0;
unsigned int DISTANCE = 0;
unsigned int GNDV = 0;
unsigned int TRACER = 0;
unsigned int ADVOLT = 0;
float POSITION_X = 0;
float POSITION_Y = 0;
float ATTITUDE_ANGLE = 0;
float ATTITUDE_ANGLE0 = 0;
float FAI0 = 0;
float POSITION_X_1 = 0;
float POSITION_Y_1 = 0;
float POSITION_X_2 = 0;
float POSITION_Y_2 = 0;
float COSFAI_2 = 0;
float SINFAI_2 = 0;
float OMEGA_2 = 0;
float R_SPEED = 0;
float L_SPEED = 0;
float SUM_R = 0;
float SUM_L = 0;
volatile unsigned char CFLAG = 0;
unsigned char SWDATA = 0;
unsigned char AUTOFLASH = 3;
volatile unsigned char LINEDETECT_R = 0;
volatile unsigned char LINEDETECT_L = 0;
volatile unsigned char LINESENSE = 0;

/* Forward declarations */
void current_status(void);
void INITIALVALUES(void);

/* Internal variables */
static unsigned char led_state = 0;
static unsigned char flash_counter = 0;
#define FLASHCONST 50

/* Line trace clock state */
static unsigned char line_clk_high = 0;

/* ------------------------------------------------------------------ */
/*  Encoder interrupts                                                 */
/* ------------------------------------------------------------------ */
static void enc_r_callback(uint gpio, uint32_t events)
{
    (void)events;
    if (gpio_get(PIN_ENC_R_B))
        RCOUNT++;
    else
        RCOUNT--;
}

static void enc_l_callback(uint gpio, uint32_t events)
{
    (void)events;
    if (gpio_get(PIN_ENC_L_B))
        LCOUNT++;
    else
        LCOUNT--;
}

/* ------------------------------------------------------------------ */
/*  Line detect interrupts                                             */
/* ------------------------------------------------------------------ */
static void line_r_callback(uint gpio, uint32_t events)
{
    (void)gpio; (void)events;
    LINEDETECT_R = 1;
}

static void line_l_callback(uint gpio, uint32_t events)
{
    (void)gpio; (void)events;
    LINEDETECT_L = 1;
}

/* ------------------------------------------------------------------ */
/*  Line trace clock timer (1kHz toggle)                               */
/*  When LINESENSE is enabled, reads TRACER ADC on high phase          */
/* ------------------------------------------------------------------ */
static bool line_clk_callback(repeating_timer_t *rt)
{
    (void)rt;
    if (LINESENSE)
    {
        if (line_clk_high)
        {
            /* Clock was high, now going low - read tracer ADC */
            adc_select_input(2); /* ADC2 = tracer */
            TRACER = adc_read() >> 2; /* 12-bit -> 10-bit */
        }
        line_clk_high = !line_clk_high;
        gpio_put(PIN_LINE_CLOCK, line_clk_high);
    }
    else
    {
        gpio_put(PIN_LINE_CLOCK, 0);
        line_clk_high = 0;
    }
    return true;
}

/* ------------------------------------------------------------------ */
/*  Control cycle timer (10.24ms)                                      */
/*  Sets CFLAG, calls current_status(), handles LED, reads ADCs      */
/* ------------------------------------------------------------------ */
static bool control_cycle_callback(repeating_timer_t *rt)
{
    (void)rt;

    CFLAG = 1;

    /* Position and attitude estimation */
    current_status();

    /* LED auto-flash control */
    flash_counter++;
    if (flash_counter >= FLASHCONST)
    {
        flash_counter = 0;
        switch (AUTOFLASH)
        {
        case 0: /* Both off */
            gpio_put(PIN_LED0, 0);
            gpio_put(PIN_LED1, 0);
            break;
        case 1: /* Alternate */
            led_state = !led_state;
            gpio_put(PIN_LED0, led_state);
            gpio_put(PIN_LED1, !led_state);
            break;
        case 2: /* Flash both */
            led_state = !led_state;
            gpio_put(PIN_LED0, led_state);
            gpio_put(PIN_LED1, led_state);
            break;
        case 3: /* Both on */
            gpio_put(PIN_LED0, 1);
            gpio_put(PIN_LED1, 1);
            break;
        default:
            break;
        }
    }

    /* Read GNDV (line trace GND reference) */
    adc_select_input(1); /* ADC1 = GNDV */
    GNDV = adc_read() >> 2; /* 12-bit -> 10-bit */

    /* Read DISTANCE (IR range sensor) */
    adc_select_input(0); /* ADC0 = distance */
    DISTANCE = adc_read() >> 2;

    /* Read ADVOLT (user ADC) */
    adc_select_input(3); /* ADC3 = user */
    ADVOLT = adc_read() >> 2;

    return true;
}

/* ------------------------------------------------------------------ */
/*  GPIO callback dispatcher                                           */
/* ------------------------------------------------------------------ */
static void gpio_callback(uint gpio, uint32_t events)
{
    if (gpio == PIN_ENC_R_A)
        enc_r_callback(gpio, events);
    else if (gpio == PIN_ENC_L_A)
        enc_l_callback(gpio, events);
    else if (gpio == PIN_LINE_R)
        line_r_callback(gpio, events);
    else if (gpio == PIN_LINE_L)
        line_l_callback(gpio, events);
}

/* ------------------------------------------------------------------ */
/*  Hardware initialization (replaces RESET and init in core.mar)      */
/* ------------------------------------------------------------------ */
void hw_init(void)
{
    /* --- GPIO for digital I/O --- */
    /* Line trace clock (output, initially low) */
    gpio_init(PIN_LINE_CLOCK);
    gpio_set_dir(PIN_LINE_CLOCK, GPIO_OUT);
    gpio_put(PIN_LINE_CLOCK, 0);

    /* Encoder A phases (input with pullup) */
    gpio_init(PIN_ENC_R_A);
    gpio_set_dir(PIN_ENC_R_A, GPIO_IN);
    gpio_pull_up(PIN_ENC_R_A);

    gpio_init(PIN_ENC_L_A);
    gpio_set_dir(PIN_ENC_L_A, GPIO_IN);
    gpio_pull_up(PIN_ENC_L_A);

    /* Line detect sensors (input with pullup) */
    gpio_init(PIN_LINE_R);
    gpio_set_dir(PIN_LINE_R, GPIO_IN);
    gpio_pull_up(PIN_LINE_R);

    gpio_init(PIN_LINE_L);
    gpio_set_dir(PIN_LINE_L, GPIO_IN);
    gpio_pull_up(PIN_LINE_L);

    /* Dip switch (input with pullup) */
    gpio_init(PIN_DIP_SW0);
    gpio_set_dir(PIN_DIP_SW0, GPIO_IN);
    gpio_pull_up(PIN_DIP_SW0);

    gpio_init(PIN_DIP_SW1);
    gpio_set_dir(PIN_DIP_SW1, GPIO_IN);
    gpio_pull_up(PIN_DIP_SW1);

    /* LEDs (output, initially on) */
    gpio_init(PIN_LED0);
    gpio_set_dir(PIN_LED0, GPIO_OUT);
    gpio_put(PIN_LED0, 1);

    gpio_init(PIN_LED1);
    gpio_set_dir(PIN_LED1, GPIO_OUT);
    gpio_put(PIN_LED1, 1);

    /* Motor direction control (output, initially low) */
    gpio_init(PIN_MOTOR_R_FWD);
    gpio_set_dir(PIN_MOTOR_R_FWD, GPIO_OUT);
    gpio_put(PIN_MOTOR_R_FWD, 0);

    gpio_init(PIN_MOTOR_R_REV);
    gpio_set_dir(PIN_MOTOR_R_REV, GPIO_OUT);
    gpio_put(PIN_MOTOR_R_REV, 0);

    gpio_init(PIN_MOTOR_L_FWD);
    gpio_set_dir(PIN_MOTOR_L_FWD, GPIO_OUT);
    gpio_put(PIN_MOTOR_L_FWD, 0);

    gpio_init(PIN_MOTOR_L_REV);
    gpio_set_dir(PIN_MOTOR_L_REV, GPIO_OUT);
    gpio_put(PIN_MOTOR_L_REV, 0);

    /* Sensor enables (output, initially high = OFF) */
    gpio_init(PIN_LINE_EN);
    gpio_set_dir(PIN_LINE_EN, GPIO_OUT);
    gpio_put(PIN_LINE_EN, 1);

    gpio_init(PIN_RANGE_EN);
    gpio_set_dir(PIN_RANGE_EN, GPIO_OUT);
    gpio_put(PIN_RANGE_EN, 1);

    /* User digital I/O */
    gpio_init(PIN_USER_IN);
    gpio_set_dir(PIN_USER_IN, GPIO_IN);

    gpio_init(PIN_USER_OUT0);
    gpio_set_dir(PIN_USER_OUT0, GPIO_OUT);
    gpio_put(PIN_USER_OUT0, 0);

    gpio_init(PIN_USER_OUT1);
    gpio_set_dir(PIN_USER_OUT1, GPIO_OUT);
    gpio_put(PIN_USER_OUT1, 0);

    /* Encoder B phases (input) */
    gpio_init(PIN_ENC_R_B);
    gpio_set_dir(PIN_ENC_R_B, GPIO_IN);

    gpio_init(PIN_ENC_L_B);
    gpio_set_dir(PIN_ENC_L_B, GPIO_IN);

    /* --- PWM for motor control --- */
    /* Right motor PWM (slice 1, channel A) */
    gpio_set_function(PIN_MOTOR_R_PWM, GPIO_FUNC_PWM);
    /* Left motor PWM (slice 1, channel B) */
    gpio_set_function(PIN_MOTOR_L_PWM, GPIO_FUNC_PWM);
    /* Extra motor PWM (slice 2, channel A) */
    gpio_set_function(PIN_MOTOR_X_PWM, GPIO_FUNC_PWM);

    uint slice_r = pwm_gpio_to_slice_num(PIN_MOTOR_R_PWM);
    uint slice_x = pwm_gpio_to_slice_num(PIN_MOTOR_X_PWM);

    /* Configure motor PWM slice (right + left) */
    pwm_config config = pwm_get_default_config();
    /* div = 7.8125 -> 125MHz / (7.8125 * 800) = 20kHz */
    pwm_config_set_clkdiv(&config, 7.8125f);
    pwm_config_set_wrap(&config, PWM_WRAP);
    pwm_init(slice_r, &config, true);
    pwm_set_both_levels(slice_r, 1, 1); /* Initial duty = 1 (same as H8) */

    /* Configure extra motor PWM slice */
    pwm_init(slice_x, &config, true);
    pwm_set_chan_level(slice_x, PWM_CHAN_A, 1);

    /* --- ADC --- */
    adc_init();
    adc_gpio_init(PIN_ADC_DISTANCE);
    adc_gpio_init(PIN_ADC_GNDV);
    adc_gpio_init(PIN_ADC_TRACER);
    adc_gpio_init(PIN_ADC_USER);

    /* --- GPIO interrupts --- */
    gpio_set_irq_callback(&gpio_callback);
    gpio_set_irq_enabled(PIN_ENC_R_A, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(PIN_ENC_L_A, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(PIN_LINE_R, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(PIN_LINE_L, GPIO_IRQ_EDGE_RISE, true);
    irq_set_enabled(IO_IRQ_BANK0, true);

    /* --- Read dip switch --- */
    SWDATA = (gpio_get(PIN_DIP_SW0) ? 0 : 1) | ((gpio_get(PIN_DIP_SW1) ? 0 : 1) << 1);

    /* --- Initialize variables --- */
    CFLAG = 0;
    LINEDETECT_R = 0;
    LINEDETECT_L = 0;
    AUTOFLASH = 3;
    DISTANCE = 0;
    GNDV = 0;
    TRACER = 0;
    ADVOLT = 0;
    RCOUNT = 0;
    LCOUNT = 0;
    RCOUNT_1 = 0;
    LCOUNT_1 = 0;
    RCOUNT_2 = 0;
    LCOUNT_2 = 0;

    INITIALVALUES();

    /* --- Start timers --- */
    static repeating_timer_t control_timer;
    static repeating_timer_t line_clk_timer;

    add_repeating_timer_us(-CONTROL_PERIOD_US, control_cycle_callback, NULL, &control_timer);
    add_repeating_timer_us(-LINE_CLK_PERIOD_US, line_clk_callback, NULL, &line_clk_timer);
}
