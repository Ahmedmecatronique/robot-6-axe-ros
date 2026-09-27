/* PID de position : erreur (rad) -> correction de vitesse articulaire (rad/s).
 * Avec des moteurs pas-à-pas, la « commande » est une vitesse (fréquence de pas) : le système est un
 * intégrateur, donc un P seul suffit à annuler l'erreur ; I et D servent aux réglages fins. */
#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;
    float integ;        /* intégrale de l'erreur (rad·s) */
    float integ_max;    /* borne anti-windup (rad·s) */
    float d_filt;       /* dérivée de la mesure filtrée (rad/s) */
    float prev_meas;
} pid_ctrl_t;

void pid_init(pid_ctrl_t *p, float kp, float ki, float kd, float integ_max);
void pid_reset(pid_ctrl_t *p, float meas);

/* err = consigne - mesure ; retourne la correction de vitesse, saturée à ±v_limit.
 * L'intégrale ne s'accumule pas quand la sortie est saturée (anti-windup). */
float pid_update(pid_ctrl_t *p, float err, float meas, float dt, float v_limit);

#endif /* PID_H */
