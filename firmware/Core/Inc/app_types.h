/* Types et codes partagés entre le firmware et ROS 2.
 * DOIT rester identique à ros2_ws/src/robot_kinematics/robot_kinematics/status_codes.py */
#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdint.h>

/* --- États de la machine d'états ------------------------------------------------------------ */
typedef enum {
    STATE_BOOT = 0,      /* démarrage */
    STATE_DISABLED = 1,  /* drivers coupés */
    STATE_IDLE = 2,      /* drivers actifs, non référencé */
    STATE_HOMING = 3,    /* recherche des fins de course */
    STATE_READY = 4,     /* référencé, suit /joint_commands */
    STATE_ESTOP = 5,     /* arrêt d'urgence verrouillé (efface : CMD_ENABLE) */
    STATE_FAULT = 6      /* défaut verrouillé (watchdog, écart de poursuite, limite, homing) */
} robot_state_t;

/* --- Bits de défaut (bits 8..13 = axe concerné) ---------------------------------------------- */
#define FAULT_COMM        (1u << 0)
#define FAULT_FOLLOWING   (1u << 1)
#define FAULT_LIMIT       (1u << 2)
#define FAULT_HOMING      (1u << 3)
#define FAULT_ESTOP       (1u << 4)
#define FAULT_AXIS_SHIFT  8u

/* --- Commandes reçues sur /stm32/command (std_msgs/Int32) ------------------------------------ */
typedef enum {
    CMD_ENABLE = 1,      /* active les drivers, efface arrêt d'urgence / défaut */
    CMD_DISABLE = 2,     /* coupe les drivers (oublie le référencement) */
    CMD_HOME = 3,        /* lance le homing */
    CMD_ZERO_HERE = 4    /* mise en service : position actuelle = pose zéro (bras vertical) */
} robot_cmd_t;

/* --- Instantané de l'état, copié atomiquement pour micro-ROS ---------------------------------- */
typedef struct {
    float q[6];          /* positions mesurées (rad), issues des encodeurs */
    float v[6];          /* vitesses mesurées (rad/s), filtrées */
    float effort[6];     /* commande normalisée -1..1 (pas de mesure de courant avec l'A4988) */
    float q_target[6];   /* consigne effective (rad) */
    uint8_t state;
    uint8_t homed_mask;  /* bit i = axe i référencé */
    uint8_t limit_mask;  /* bit i = fin de course i actif */
    uint32_t fault;
    float max_ferr;      /* plus grand écart consigne/mesure (rad) */
    uint32_t loop_hz;    /* fréquence réelle de la boucle de contrôle */
    uint32_t cmd_age_ms; /* âge de la dernière consigne reçue */
    uint32_t uptime_s;
} control_snapshot_t;

#endif /* APP_TYPES_H */
