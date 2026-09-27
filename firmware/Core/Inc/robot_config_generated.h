/* FICHIER GÉNÉRÉ par tools/gen_firmware_config.py à partir de robot.yaml — NE PAS MODIFIER À LA MAIN. */
#ifndef ROBOT_CONFIG_GENERATED_H
#define ROBOT_CONFIG_GENERATED_H

#include <stdint.h>

#define NUM_AXES 6
#define CFG_MAX_STEP_RATE_HZ 8000.0f
#define CFG_WATCHDOG_MS 300
#define CFG_DEADBAND_RAD 0.000872664626f
#define CFG_FERR_MAX_RAD 0.104719755f

/* pas moteur (avec micro-pas) par radian ARTICULAIRE, réducteur inclus */
static const float CFG_STEPS_PER_RAD[NUM_AXES] = { 15278.8745f, 15278.8745f, 15278.8745f, 10185.9164f, 10185.9164f, 5092.95818f };
/* incréments encodeur (x4) par radian ARTICULAIRE ; l'encodeur est sur l'arbre moteur */
static const float CFG_ENC_COUNTS_PER_RAD[NUM_AXES] = { 19098.5932f, 19098.5932f, 19098.5932f, 12732.3954f, 12732.3954f, 6366.19772f };
static const int8_t CFG_ENC_SIGN[NUM_AXES] = { 1, 1, 1, 1, 1, 1 };
static const int8_t CFG_DIR_SIGN[NUM_AXES] = { 1, 1, 1, 1, 1, 1 };

/* limites articulaires (rad), vitesses (rad/s), accélérations (rad/s^2) */
static const float CFG_Q_MIN[NUM_AXES] = { -2.96705973f, -1.57079633f, -2.61799388f, -2.96705973f, -2.0943951f, -2.96705973f };
static const float CFG_Q_MAX[NUM_AXES] = { 2.96705973f, 1.57079633f, 2.61799388f, 2.96705973f, 2.0943951f, 2.96705973f };
static const float CFG_V_MAX[NUM_AXES] = { 0.523598776f, 0.523598776f, 0.523598776f, 0.785398163f, 0.785398163f, 1.57079633f };
static const float CFG_A_MAX[NUM_AXES] = { 1.04719755f, 1.04719755f, 1.04719755f, 1.57079633f, 1.57079633f, 3.14159265f };

/* homing : sens de recherche, vitesse, angle au fin de course, ordre */
static const int8_t CFG_HOME_DIR[NUM_AXES] = { -1, -1, 1, -1, -1, -1 };
static const float CFG_HOME_SPEED[NUM_AXES] = { 0.174532925f, 0.174532925f, 0.174532925f, 0.261799388f, 0.261799388f, 0.523598776f };
static const float CFG_HOME_POS[NUM_AXES] = { -2.96705973f, -1.57079633f, 2.61799388f, -2.96705973f, -2.0943951f, -2.96705973f };
static const uint8_t CFG_HOME_ORDER[NUM_AXES] = { 3, 4, 5, 2, 1, 0 };

/* PID de position (rad -> rad/s) */
static const float CFG_KP[NUM_AXES] = { 6f, 6f, 6f, 8f, 8f, 8f };
static const float CFG_KI[NUM_AXES] = { 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f };
static const float CFG_KD[NUM_AXES] = { 0f, 0f, 0f, 0f, 0f, 0f };

#endif /* ROBOT_CONFIG_GENERATED_H */
