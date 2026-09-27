/* Contrôle des 6 axes : machine d'états, PID 1 kHz, homing, watchdog, arrêt d'urgence.
 * Les fonctions control_set_* / control_command peuvent être appelées depuis la tâche micro-ROS. */
#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include "app_types.h"

void control_init(void);
void control_task(void *arg);                        /* tâche FreeRTOS, priorité maximale, période 1 ms */

/* --- entrées (appelées par les callbacks micro-ROS) ---------------------------------------------- */
void control_set_command(const float q[6], const float v[6]);   /* /joint_commands : position + vitesse de préaction */
void control_estop(int active);                                  /* /emergency_stop : true = arrêt verrouillé */
void control_heartbeat(void);                                    /* tout message /emergency_stop = signe de vie */
void control_command(int32_t cmd);                               /* /stm32/command (robot_cmd_t) */
void control_gripper(float opening);                             /* /gripper_command */
void control_link_lost(void);                                    /* l'agent micro-ROS ne répond plus */

/* --- sortie -------------------------------------------------------------------------------------- */
void control_get_snapshot(control_snapshot_t *out);              /* copie atomique */

#endif /* CONTROL_H */
