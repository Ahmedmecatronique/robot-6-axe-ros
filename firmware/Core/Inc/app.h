#ifndef APP_H
#define APP_H

/* Crée les tâches FreeRTOS : contrôle 1 kHz (priorité max) et micro-ROS. Appeler après board_init(),
 * avant le démarrage de l'ordonnanceur. */
void app_start(void);

#endif /* APP_H */
