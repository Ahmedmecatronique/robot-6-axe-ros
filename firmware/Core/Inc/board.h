/* Couche matérielle : GPIO, encodeurs (timers en mode encodeur), générateur de pas, PWM servo.
 * Accès direct aux registres (aucune dépendance à un fichier généré par CubeMX). */
#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

/* Initialise GPIO, 6 timers encodeurs, timer de pas (TIM7, 20 kHz) et PWM du servo. Drivers COUPÉS. */
void board_init(void);

/* /EN commun des A4988 : 1 = moteurs alimentés, 0 = coupés. */
void board_drivers_enable(int enable);

/* Variation du compteur encodeur depuis le dernier appel (signe matériel, sans CFG_ENC_SIGN).
 * Doit être appelée à > 100 Hz par UN SEUL lecteur (la tâche de contrôle). */
int32_t board_encoder_delta(int axis);

/* Fréquence de pas (>= 0) et sens (1 = rotation articulaire positive). Plafonnée à CFG_MAX_STEP_RATE_HZ.
 * Écriture atomique : appelable depuis la tâche de contrôle pendant que l'ISR tourne. */
void board_set_step_rate(int axis, float steps_per_s, int positive);

/* 1 si le fin de course de l'axe est actif (état brut, sans anti-rebond). */
int board_limit_raw(int axis);

/* Ouverture de la pince : 0.0 = fermée ... 1.0 = ouverte. */
void board_gripper_set(float opening);

#endif /* BOARD_H */
