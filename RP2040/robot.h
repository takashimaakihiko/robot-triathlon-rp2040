#ifndef ROBOT_H
#define ROBOT_H

/************************************************************************************
   robot.h  (RP2040 port)  2024
   Robot Triathlon standard kit library - ported from H8/3664F to RP2040
   Original by YOU-YOU ROBOTICS, Version 0.3.3
************************************************************************************/

#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "pin_config.h"

/* Robot physical parameters */
#define L_L 17.4f   /* Distance between wheels [cm] */
#define R_R 2.9f    /* Wheel radius [cm] */

#define THRESHOLD 300 /* Obstacle detection threshold */

/* PI speed control gains */
#define KP  55.0f
#define KI  3.7f

#define T_T 0.0124f    /* Control cycle [sec] */
#define PI  3.1415926535897932f
#define K_K 3.51409e-2f  /* Encoder pulse to wheel angle conversion */

/* Shared global variables (defined in hw.c) */
extern long RCOUNT;
extern long LCOUNT;
extern long RCOUNT_1;
extern long LCOUNT_1;
extern long RCOUNT_2;
extern long LCOUNT_2;
extern unsigned int DISTANCE;
extern unsigned int GNDV;
extern unsigned int TRACER;
extern unsigned int ADVOLT;
extern float POSITION_X;
extern float POSITION_Y;
extern float ATTITUDE_ANGLE;
extern float ATTITUDE_ANGLE0;
extern float FAI0;
extern float POSITION_X_1;
extern float POSITION_Y_1;
extern float POSITION_X_2;
extern float POSITION_Y_2;
extern float COSFAI_2;
extern float SINFAI_2;
extern float OMEGA_2;
extern float R_SPEED;
extern float L_SPEED;
extern float SUM_R;
extern float SUM_L;
extern volatile unsigned char CFLAG;
extern unsigned char SWDATA;
extern unsigned char AUTOFLASH;
extern volatile unsigned char LINEDETECT_R;
extern volatile unsigned char LINEDETECT_L;
extern volatile unsigned char LINESENSE;

/* Hardware init (defined in hw.c) */
extern void hw_init(void);

/* PWM slice for motor control */
static inline uint motor_pwm_slice(void)
{
    return pwm_gpio_to_slice_num(PIN_MOTOR_R_PWM);
}

/* ------------------------------------------------------------------ */
/*  Motor control helpers (replace H8 IO.PDR5 and TW.GRB/GRC access)  */
/* ------------------------------------------------------------------ */
static inline void motor_r_dir_forward(void)
{
    gpio_put(PIN_MOTOR_R_FWD, 1);
    gpio_put(PIN_MOTOR_R_REV, 0);
}

static inline void motor_r_dir_reverse(void)
{
    gpio_put(PIN_MOTOR_R_FWD, 0);
    gpio_put(PIN_MOTOR_R_REV, 1);
}

static inline void motor_l_dir_forward(void)
{
    gpio_put(PIN_MOTOR_L_FWD, 1);
    gpio_put(PIN_MOTOR_L_REV, 0);
}

static inline void motor_l_dir_reverse(void)
{
    gpio_put(PIN_MOTOR_L_FWD, 0);
    gpio_put(PIN_MOTOR_L_REV, 1);
}

static inline void motor_r_set_duty(unsigned int duty)
{
    if (duty > PWM_WRAP) duty = PWM_WRAP;
    pwm_set_chan_level(motor_pwm_slice(), PWM_CHAN_A, duty);
}

static inline void motor_l_set_duty(unsigned int duty)
{
    if (duty > PWM_WRAP) duty = PWM_WRAP;
    pwm_set_chan_level(motor_pwm_slice(), PWM_CHAN_B, duty);
}

/* Apply PI control output to right motor */
static inline void motor_r_apply(float u)
{
    if (u >= 0.0f) motor_r_dir_forward();
    else { u = -u; motor_r_dir_reverse(); }
    if (u >= (float)PWM_WRAP) u = (float)PWM_WRAP;
    motor_r_set_duty((unsigned int)u);
}

/* Apply PI control output to left motor */
static inline void motor_l_apply(float u)
{
    if (u >= 0.0f) motor_l_dir_forward();
    else { u = -u; motor_l_dir_reverse(); }
    if (u >= (float)PWM_WRAP) u = (float)PWM_WRAP;
    motor_l_set_duty((unsigned int)u);
}

/* ------------------------------------------------------------------ */
/*  Math helpers                                                      */
/* ------------------------------------------------------------------ */
static inline float sfabs(float x)
{
    return fabsf(x);
}

/* ------------------------------------------------------------------ */
/*  Position and attitude estimation (same algorithm as original)     */
/*  Reference: kinematics/estimation.html                             */
/* ------------------------------------------------------------------ */
void current_status(void)
{
    float cosfai_1, sinfai_1, fai_1, romega_1, lomega_1, omega_1, theta_r, theta_l;
    long count_r, count_l;
    count_r = RCOUNT;
    count_l = LCOUNT;
    theta_r = K_K * (float)count_r;
    theta_l = K_K * (float)count_l;
    /* Wheel speeds */
    R_SPEED = K_K * (float)RCOUNT_1;
    L_SPEED = K_K * (float)LCOUNT_1;
    R_SPEED = R_R * (theta_r - R_SPEED) / T_T;
    L_SPEED = R_R * (theta_l - L_SPEED) / T_T;
    /* Position estimation */
    romega_1 = K_K * (float)(count_r - RCOUNT_2);
    lomega_1 = K_K * (float)(count_l - LCOUNT_2);
    omega_1 = 0.5f * (romega_1 + lomega_1) / T_T;
    fai_1 = K_K * (float)(RCOUNT_1 - LCOUNT_1);
    fai_1 = R_R * fai_1 / L_L;
    cosfai_1 = cosf(fai_1 + FAI0);
    sinfai_1 = sinf(fai_1 + FAI0);
    POSITION_X_1 = 0.25f * R_R * T_T * (omega_1 * cosfai_1 + OMEGA_2 * COSFAI_2);
    POSITION_X_1 += POSITION_X_2;
    POSITION_Y_1 = 0.25f * R_R * T_T * (omega_1 * sinfai_1 + OMEGA_2 * SINFAI_2);
    POSITION_Y_1 += POSITION_Y_2;
    POSITION_X = 2.0f * POSITION_X_1 - POSITION_X_2;
    POSITION_Y = 2.0f * POSITION_Y_1 - POSITION_Y_2;
    ATTITUDE_ANGLE = 0.572958e+2f * R_R * (theta_r - theta_l) / L_L + ATTITUDE_ANGLE0;
    RCOUNT_2 = RCOUNT_1;
    LCOUNT_2 = LCOUNT_1;
    RCOUNT_1 = count_r;
    LCOUNT_1 = count_l;
    OMEGA_2 = omega_1;
    POSITION_X_2 = POSITION_X_1;
    POSITION_Y_2 = POSITION_Y_1;
    COSFAI_2 = cosfai_1;
    SINFAI_2 = sinfai_1;
}

/* ------------------------------------------------------------------ */
/*  Wait function                                                      */
/* ------------------------------------------------------------------ */
void wait(float sec)
{
    unsigned int i;
    i = (unsigned int)(sec / T_T);
    while (i)
    {
        if (CFLAG)
        {
            CFLAG = 0;
            i--;
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Digital I/O (user ports)                                          */
/* ------------------------------------------------------------------ */
unsigned char input0(void)
{
    return (unsigned char)gpio_get(PIN_USER_IN);
}

unsigned char input1(void)
{
    /* Second user input - use GPIO 23 if available, or adapt as needed */
    return 0;
}

void output0(unsigned char revel)
{
    gpio_put(PIN_USER_OUT0, revel ? 1 : 0);
}

void output1(unsigned char revel)
{
    gpio_put(PIN_USER_OUT1, revel ? 1 : 0);
}

/* ------------------------------------------------------------------ */
/*  Extra PWM output (user port)                                       */
/* ------------------------------------------------------------------ */
void pwm(unsigned int duty)
{
    if (duty > PWM_WRAP) duty = PWM_WRAP;
    pwm_set_chan_level(pwm_gpio_to_slice_num(PIN_MOTOR_X_PWM), PWM_CHAN_A, duty);
}

/* ------------------------------------------------------------------ */
/*  Sensor enable/disable                                              */
/* ------------------------------------------------------------------ */
void linesensor_on(char on)
{
    if (on != 0) on = 1;
    gpio_put(PIN_LINE_EN, on ? 0 : 1); /* LO = ON */
}

void rangesensor_on(char on)
{
    if (on != 0) on = 1;
    gpio_put(PIN_RANGE_EN, on ? 0 : 1); /* LO = ON */
}

/* ------------------------------------------------------------------ */
/*  Stop                                                               */
/* ------------------------------------------------------------------ */
void stop(void)
{
    gpio_put(PIN_MOTOR_R_FWD, 0);
    gpio_put(PIN_MOTOR_R_REV, 0);
    gpio_put(PIN_MOTOR_L_FWD, 0);
    gpio_put(PIN_MOTOR_L_REV, 0);
    motor_r_set_duty(1);
    motor_l_set_duty(1);
    SUM_R = 0.0f;
    SUM_L = 0.0f;
}

/* ------------------------------------------------------------------ */
/*  Move forward / backward (no PI control, direct duty)              */
/* ------------------------------------------------------------------ */
void move_forward(unsigned int lduty, unsigned int rduty)
{
    if (lduty > PWM_WRAP) lduty = PWM_WRAP;
    if (rduty > PWM_WRAP) rduty = PWM_WRAP;
    motor_l_set_duty(lduty);
    motor_r_set_duty(rduty);
    motor_l_dir_forward();
    motor_r_dir_forward();
}

void move_backward(unsigned int lduty, unsigned int rduty)
{
    if (lduty > PWM_WRAP) lduty = PWM_WRAP;
    if (rduty > PWM_WRAP) rduty = PWM_WRAP;
    motor_l_set_duty(lduty);
    motor_r_set_duty(rduty);
    motor_l_dir_reverse();
    motor_r_dir_reverse();
}

/* ------------------------------------------------------------------ */
/*  Move specified distance with PI speed control                     */
/* ------------------------------------------------------------------ */
void move_s(float lspeed, float rspeed, float distance)
{
    float error_r, error_l, u_r, u_l, x0, y0, s, dx, dy;
    x0 = POSITION_X_1;
    y0 = POSITION_Y_1;
    s = 0.0f;
    while (s < distance)
    {
        if (CFLAG)
        {
            CFLAG = 0;
            error_r = rspeed - R_SPEED;
            error_l = lspeed - L_SPEED;
            SUM_R += error_r;
            SUM_L += error_l;
            u_r = KP * error_r + KI * SUM_R;
            u_l = KP * error_l + KI * SUM_L;
            motor_r_apply(u_r);
            motor_l_apply(u_l);
            dx = POSITION_X_1 - x0;
            dy = POSITION_Y_1 - y0;
            s += sqrtf(dx * dx + dy * dy);
            x0 = POSITION_X_1;
            y0 = POSITION_Y_1;
        }
    }
    stop();
}

/* ------------------------------------------------------------------ */
/*  Move for specified time with PI speed control                     */
/* ------------------------------------------------------------------ */
void move_t(float lspeed, float rspeed, float sec)
{
    float error_r, error_l, u_r, u_l;
    unsigned int i;
    i = (unsigned int)(sec / T_T);
    while (i)
    {
        if (CFLAG)
        {
            CFLAG = 0;
            i--;
            error_r = rspeed - R_SPEED;
            error_l = lspeed - L_SPEED;
            SUM_R += error_r;
            SUM_L += error_l;
            u_r = KP * error_r + KI * SUM_R;
            u_l = KP * error_l + KI * SUM_L;
            motor_r_apply(u_r);
            motor_l_apply(u_l);
        }
    }
    stop();
}

/* ------------------------------------------------------------------ */
/*  Rotate (turn in place) specified angle                             */
/* ------------------------------------------------------------------ */
void rotate(float speed, float angle)
{
    float rspeed, lspeed, error_r, error_l, u_r, u_l, angle0;
    angle0 = ATTITUDE_ANGLE;
    speed = sfabs(speed);
    if (angle > 0.0f)
    {
        rspeed = 8.7266462599716478e-3f * speed * L_L;
        lspeed = -rspeed;
    }
    else
    {
        lspeed = 8.7266462599716478e-3f * speed * L_L;
        rspeed = -lspeed;
    }
    angle = sfabs(angle);
    while (sfabs(ATTITUDE_ANGLE - angle0) < angle)
    {
        if (CFLAG)
        {
            CFLAG = 0;
            error_r = rspeed - R_SPEED;
            error_l = lspeed - L_SPEED;
            SUM_R += error_r;
            SUM_L += error_l;
            u_r = KP * error_r + KI * SUM_R;
            u_l = KP * error_l + KI * SUM_L;
            motor_r_apply(u_r);
            motor_l_apply(u_l);
        }
    }
    stop();
}

/* ------------------------------------------------------------------ */
/*  Line trace for specified time                                     */
/*  Returns: 0=timeout, 1=right line detected, 2=left line detected   */
/* ------------------------------------------------------------------ */
unsigned char linetrace(float speed, float sec)
{
    unsigned int i;
    float adjust, error_r, error_l, u_r, u_l, deviation, shift, kp, ki;
    unsigned char flash, result;
    kp = 0.05f;
    ki = 0.0001f;
    shift = 0.0f;
    LINESENSE = 1;
    wait(0.1f);
    LINEDETECT_R = 0;
    LINEDETECT_L = 0;
    flash = AUTOFLASH;
    AUTOFLASH = 5;
    i = (unsigned int)(sec / 0.01f);
    while (i)
    {
        if (CFLAG)
        {
            CFLAG = 0;
            i--;
            deviation = (float)TRACER - (float)GNDV;
            if (deviation >= 0)
            {
                gpio_put(PIN_LED0, 1);
                gpio_put(PIN_LED1, 0);
            }
            else
            {
                gpio_put(PIN_LED0, 0);
                gpio_put(PIN_LED1, 1);
            }
            shift += deviation;
            adjust = kp * deviation + ki * shift;
            error_r = speed - adjust - R_SPEED;
            error_l = speed + adjust - L_SPEED;
            SUM_R += error_r;
            SUM_L += error_l;
            u_r = KP * error_r + KI * SUM_R;
            u_l = KP * error_l + KI * SUM_L;
            motor_r_apply(u_r);
            motor_l_apply(u_l);
        }
        if ((LINEDETECT_R == 1) || (LINEDETECT_L == 1)) break;
    }
    stop();
    if (LINEDETECT_R == 1)
    {
        result = 1;
        gpio_put(PIN_LED0, 1);
        gpio_put(PIN_LED1, 0);
    }
    else if (LINEDETECT_L == 1)
    {
        result = 2;
        gpio_put(PIN_LED0, 0);
        gpio_put(PIN_LED1, 1);
    }
    else
    {
        result = 0;
        AUTOFLASH = flash;
    }
    LINESENSE = 0;
    return result;
}

/* ------------------------------------------------------------------ */
/*  Find obstacle while rotating                                      */
/*  Returns distance [cm], 100.0 if none, negative if not converged   */
/* ------------------------------------------------------------------ */
float find_obstacle(float speed, float angle)
{
    float rspeed, lspeed, error_r, error_l, u_r, u_l, angle0, ang[5], av[3], d, v;
    unsigned int i, dis[5];
    angle0 = ATTITUDE_ANGLE;
    speed = sfabs(speed);
    if (angle > 0.0f)
    {
        rspeed = 8.7266462599716478e-3f * speed * L_L;
        lspeed = -rspeed;
    }
    else
    {
        lspeed = 8.7266462599716478e-3f * speed * L_L;
        rspeed = -lspeed;
    }
    angle = sfabs(angle);
    for (i = 0; i < 5; i++)
    {
        dis[i] = THRESHOLD;
        ang[i] = 0.0f;
    }
    av[0] = (float)THRESHOLD;
    av[1] = (float)THRESHOLD;
    d = 1.0f;
    while (sfabs(ATTITUDE_ANGLE - angle0) < angle)
    {
        if (CFLAG)
        {
            CFLAG = 0;
            error_r = rspeed - R_SPEED;
            error_l = lspeed - L_SPEED;
            SUM_R += error_r;
            SUM_L += error_l;
            u_r = KP * error_r + KI * SUM_R;
            u_l = KP * error_l + KI * SUM_L;
            motor_r_apply(u_r);
            motor_l_apply(u_l);

            /* Shift distance data */
            dis[4] = dis[3]; dis[3] = dis[2]; dis[2] = dis[1]; dis[1] = dis[0];
            dis[0] = DISTANCE;
            if (dis[0] <= THRESHOLD) dis[0] = THRESHOLD;
            ang[4] = ang[3]; ang[3] = ang[2]; ang[2] = ang[1]; ang[1] = ang[0];
            ang[0] = ATTITUDE_ANGLE;
            /* Moving average */
            av[2] = av[1];
            av[1] = av[0];
            av[0] = (float)(dis[0] + dis[1] + dis[2]) * 3.3333333333333333e-1f;
            /* Check if past minimum */
            if (av[0] - av[1] < 0.0f)
            {
                d = -1.0f;
                v = 4.8875855327468230e-3f * (float)dis[1];
            }
        }
        if (d < 0.0f) break;
    }
    stop();
    if (sfabs(ATTITUDE_ANGLE - angle0) - angle >= 0.0f)
    {
        if ((dis[0] == THRESHOLD) && (dis[3] == THRESHOLD))
        {
            d = 100.0f;
        }
        else
        {
            v = 4.8875855327468230e-3f * (float)dis[0];
            d = 0.5508f;
            d = d * v - 8.5941f;
            d = d * v + 50.26f;
            d = d * v - 136.51f;
            d = d * v + 166.2f;
            d = -d;
        }
    }
    else
    {
        d = 0.5508f;
        d = d * v - 8.5941f;
        d = d * v + 50.26f;
        d = d * v - 136.51f;
        d = d * v + 166.2f;
    }
    return d;
}

/* ------------------------------------------------------------------ */
/*  Find line while moving                                             */
/*  Returns: 0=timeout, 1=right detected, 2=left detected             */
/* ------------------------------------------------------------------ */
unsigned char find_line(float lspeed, float rspeed, float sec)
{
    float error_r, error_l, u_r, u_l;
    unsigned int i;
    unsigned char result;
    LINEDETECT_R = 0;
    LINEDETECT_L = 0;
    i = (unsigned int)(sec / T_T);
    while (i)
    {
        if (CFLAG)
        {
            CFLAG = 0;
            i--;
            error_r = rspeed - R_SPEED;
            error_l = lspeed - L_SPEED;
            SUM_R += error_r;
            SUM_L += error_l;
            u_r = KP * error_r + KI * SUM_R;
            u_l = KP * error_l + KI * SUM_L;
            motor_r_apply(u_r);
            motor_l_apply(u_l);
        }
        if ((LINEDETECT_R == 1) || (LINEDETECT_L == 1)) break;
    }
    stop();
    if (LINEDETECT_R == 1)
    {
        result = 1;
        AUTOFLASH = 4;
        gpio_put(PIN_LED0, 1);
        gpio_put(PIN_LED1, 0);
    }
    else if (LINEDETECT_L == 1)
    {
        result = 2;
        AUTOFLASH = 4;
        gpio_put(PIN_LED0, 0);
        gpio_put(PIN_LED1, 1);
    }
    else
    {
        result = 0;
    }
    return result;
}

/* ------------------------------------------------------------------ */
/*  Initial values                                                     */
/* ------------------------------------------------------------------ */
void INITIALVALUES(void)
{
    AUTOFLASH = 3;
    POSITION_X_2 = 0.0f;
    POSITION_Y_2 = 0.0f;
    ATTITUDE_ANGLE0 = 90.0f;
    ATTITUDE_ANGLE = ATTITUDE_ANGLE0;
    POSITION_X = POSITION_X_2;
    POSITION_X_1 = POSITION_X_2;
    POSITION_Y = POSITION_Y_2;
    POSITION_Y_1 = POSITION_Y_2;
    FAI0 = PI * ATTITUDE_ANGLE0 / 180.0f;
    SINFAI_2 = sinf(FAI0);
    COSFAI_2 = cosf(FAI0);
    OMEGA_2 = 0.0f;
    SUM_R = 0.0f;
    SUM_L = 0.0f;
}

#endif /* ROBOT_H */
