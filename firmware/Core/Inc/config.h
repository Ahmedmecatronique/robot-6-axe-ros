/* Configuration du firmware.
 * Les constantes du robot (réducteurs, limites, PID...) viennent de robot.yaml :
 *   python tools/gen_firmware_config.py   -> robot_config_generated.h
 *
 * Câblage (Nucleo-F446RE) — à respecter ou à modifier dans board.c :
 *
 *   Axe | STEP  | DIR   | Fin de course | Encodeur (timer : A / B)
 *   ----+-------+-------+---------------+---------------------------
 *    1  | PC0   | PB12  | PC9           | TIM1 : PA8  / PA9
 *    2  | PC1   | PB13  | PC10          | TIM2 : PA15 / PB3
 *    3  | PC2   | PB14  | PC11          | TIM3 : PA6  / PA7
 *    4  | PC3   | PB15  | PC12          | TIM4 : PB6  / PB7
 *    5  | PC4   | PB4   | PD2           | TIM5 : PA0  / PA1
 *    6  | PC5   | PB5   | PA4           | TIM8 : PC6  / PC7
 *
 *   /EN commun des 6 A4988 : PC8 (actif à l'état bas)
 *   Servo de pince (PWM 50 Hz) : PB8 (TIM10_CH1)
 *   Liaison micro-ROS : USART2 (PA2/PA3 = port USB du ST-LINK, /dev/ttyACM0 côté Pi)
 *   Fins de course : contact vers GND, pull-up interne (actif à l'état bas)
 */
#ifndef CONFIG_H
#define CONFIG_H

#include "robot_config_generated.h"

/* --- Temps ----------------------------------------------------------------------------------- */
#define CONTROL_PERIOD_MS      1u          /* boucle de contrôle : 1 kHz (tick FreeRTOS = 1 ms) */
#define CONTROL_DT             0.001f
#define STEP_ISR_HZ            20000u      /* générateur de pas : impulsion max 10 kHz par axe */

/* --- Sécurité -------------------------------------------------------------------------------- */
/* 0 : l'arrêt d'urgence logiciel garde les bobines alimentées (le bras ne tombe pas).
 * 1 : coupe les drivers (le bras peut tomber sous son poids !). Le vrai arrêt de sécurité doit
 *     être un bouton physique qui coupe l'alimentation moteurs. */
#define ESTOP_DISABLE_DRIVERS  0
#define FERR_TIME_MS           200u        /* écart de poursuite > seuil pendant ce temps -> défaut */
#define SOFT_LIMIT_MARGIN_RAD  0.035f      /* 2° au-delà des limites -> défaut */
#define EXTRAPOLATION_MAX_MS   50u         /* durée max d'extrapolation de la consigne entre 2 messages */
#define LIMIT_DEBOUNCE_MS      3u

/* --- Contrôle -------------------------------------------------------------------------------- */
#define VEL_FILTER_ALPHA       0.10f       /* filtre passe-bas de la vitesse mesurée */
#define ACCEL_MARGIN           2.0f        /* le PID peut accélérer jusqu'à 2x l'accélération de trajectoire */

/* --- Homing ---------------------------------------------------------------------------------- */
#define HOME_BACKOFF_SPEED_DIV 3.0f        /* recul plus lent que l'approche */
#define HOME_TIMEOUT_MARGIN_S  5.0f
#define HOME_PROGRESS_CHECK_MS 1000u       /* contrôle de progression (moteur bloqué / encodeur inversé) */
#define HOME_PROGRESS_MIN      0.25f       /* fraction du déplacement théorique exigée */

/* --- Pince (servo) ---------------------------------------------------------------------------- */
#define GRIPPER_US_CLOSED      1000u
#define GRIPPER_US_OPEN        2000u

#endif /* CONFIG_H */
