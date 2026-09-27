#include "pid.h"

void pid_init(pid_ctrl_t *p, float kp, float ki, float kd, float integ_max)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->integ_max = integ_max;
    pid_reset(p, 0.0f);
}

void pid_reset(pid_ctrl_t *p, float meas)
{
    p->integ = 0.0f;
    p->d_filt = 0.0f;
    p->prev_meas = meas;
}

float pid_update(pid_ctrl_t *p, float err, float meas, float dt, float v_limit)
{
    /* Dérivée sur la MESURE (pas sur l'erreur) : évite les pics quand la consigne change par paliers */
    float d_raw = (meas - p->prev_meas) / dt;
    p->prev_meas = meas;
    p->d_filt += 0.2f * (d_raw - p->d_filt);

    float out = p->kp * err + p->ki * p->integ - p->kd * p->d_filt;

    if (out > v_limit) {
        out = v_limit;
    } else if (out < -v_limit) {
        out = -v_limit;
    } else {
        p->integ += err * dt;                /* on n'intègre que si la sortie n'est pas saturée */
        if (p->integ > p->integ_max) {
            p->integ = p->integ_max;
        } else if (p->integ < -p->integ_max) {
            p->integ = -p->integ_max;
        }
    }
    return out;
}
