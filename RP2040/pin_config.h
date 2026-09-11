#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

/*
 * RP2040 pin mapping for Robot Triathlon standard kit
 *
 * H8/3664F -> RP2040 GPIO mapping
 * Based on core.mar port assignments.
 */

/* Port 1 equivalents */
#define PIN_LINE_CLOCK    0   /* Line trace sensor clock (1kHz toggle) - was P10 */
#define PIN_ENC_R_A       1   /* Right rotary encoder A phase (IRQ falling) - was P14/IRQ0 */
#define PIN_ENC_L_A       2   /* Left rotary encoder A phase (IRQ falling) - was P15/IRQ1 */
#define PIN_LINE_R        3   /* Right line detect sensor (IRQ rising) - was P16/IRQ2 */
#define PIN_LINE_L        4   /* Left line detect sensor (IRQ rising) - was P17/IRQ3 */

/* Port 5 equivalents */
#define PIN_DIP_SW0       5   /* Dip switch bit 0 (input, pullup) - was P50 */
#define PIN_DIP_SW1       6   /* Dip switch bit 1 (input, pullup) - was P51 */
#define PIN_LED0          7   /* Display LED 0 (output) - was P52 */
#define PIN_LED1          8   /* Display LED 1 (output) - was P53 */
#define PIN_MOTOR_R_FWD   9   /* Right motor forward (output) - was P54 */
#define PIN_MOTOR_R_REV   10  /* Right motor reverse (output) - was P55 */
#define PIN_MOTOR_L_FWD   11  /* Left motor forward (output) - was P56 */
#define PIN_MOTOR_L_REV   12  /* Left motor reverse (output) - was P57 */

/* Port 7 equivalents */
#define PIN_LINE_EN       13  /* Line sensor enable (LO=ON, HI=OFF) - was P74 */
#define PIN_RANGE_EN      14  /* Range sensor enable (LO=ON, HI=OFF) - was P75 */
#define PIN_USER_IN       15  /* User digital input - was P76 */

/* Port 8 equivalents */
#define PIN_ENC_R_B       16  /* Right rotary encoder B phase (input) - was P80 */
#define PIN_ENC_L_B       17  /* Left rotary encoder B phase (input) - was P81 */
#define PIN_MOTOR_R_PWM   18  /* Right motor PWM (PWM slice 1, ch A) - was P82 */
#define PIN_MOTOR_L_PWM   19  /* Left motor PWM (PWM slice 1, ch B) - was P83 */
#define PIN_MOTOR_X_PWM   20  /* Extra motor PWM (PWM slice 2, ch A) - was P84 */
#define PIN_USER_OUT0     21  /* User digital output 0 - was P85 */
#define PIN_USER_OUT1     22  /* User digital output 1 - was P86 */
/* GPIO 23 is reserved (board VSYS on some Pico boards) */
/* GPIO 24-25 are reserved (board VBUS / output) */

/* Port B equivalents (ADC) */
#define PIN_ADC_DISTANCE  26  /* ADC0: IR range sensor - was PB0/AN0 */
#define PIN_ADC_GNDV      27  /* ADC1: Line trace GND voltage - was PB1/AN1 */
#define PIN_ADC_TRACER    28  /* ADC2: Line trace sensor signal - was PB2/AN2 */
#define PIN_ADC_USER      29  /* ADC3: User ADC input - was PB3/AN3 */

/* PWM configuration */
#define PWM_WRAP          799   /* PWM period (same as H8 GRA=800, 0-799 duty) */
#define PWM_SLICE_MOTOR   1     /* PWM slice for right (chA) and left (chB) motor */
#define PWM_SLICE_EXTRA   2     /* PWM slice for extra motor (chA) */
/* sys_clk=125MHz, div=7.8125, wrap=799 -> 20kHz PWM (matches H8) */

/* Control cycle */
#define CONTROL_PERIOD_US 10240 /* 10.24ms control cycle (matches H8 TIMERCONST=20 * 0.512ms) */

/* Line trace clock */
#define LINE_CLK_PERIOD_US 500  /* 500us toggle = 1kHz clock */

#endif /* PIN_CONFIG_H */
