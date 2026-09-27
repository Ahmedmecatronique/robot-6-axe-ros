"""Codes partagés entre le firmware STM32, le simulateur, safety_node, trajectory_node et l'IHM.

Le STM32 publie /stm32/status (std_msgs/Int32MultiArray, 10 Hz, 8 entiers) :

    data[ST_STATE]        état de la machine d'états (STATE_*)
    data[ST_HOMED_MASK]   bit i = axe i référencé (homing fait)
    data[ST_LIMIT_MASK]   bit i = fin de course i actuellement actif
    data[ST_FAULT_CODE]   bits FAULT_* ; bits 8..13 = axes concernés
    data[ST_MAX_FERR]     plus grand écart consigne/mesure, en milli-radians
    data[ST_LOOP_HZ]      fréquence réelle de la boucle de contrôle (Hz)
    data[ST_CMD_AGE_MS]   âge de la dernière consigne reçue (ms)
    data[ST_UPTIME_S]     temps depuis le démarrage (s)

Ces valeurs doivent rester identiques à firmware/Core/Inc/app_types.h.
"""

# --- États --------------------------------------------------------------------------
STATE_BOOT = 0       # démarrage
STATE_DISABLED = 1   # drivers coupés
STATE_IDLE = 2       # drivers actifs, non référencé (homing requis)
STATE_HOMING = 3     # recherche des fins de course
STATE_READY = 4      # référencé, prêt à suivre /joint_commands
STATE_ESTOP = 5      # arrêt d'urgence verrouillé (efface : commande ENABLE)
STATE_FAULT = 6      # défaut verrouillé (watchdog, écart de poursuite, homing...)

STATE_NAMES = {
    STATE_BOOT: 'DÉMARRAGE',
    STATE_DISABLED: 'DÉSACTIVÉ',
    STATE_IDLE: 'NON RÉFÉRENCÉ',
    STATE_HOMING: 'HOMING',
    STATE_READY: 'PRÊT',
    STATE_ESTOP: 'ARRÊT D\'URGENCE',
    STATE_FAULT: 'DÉFAUT',
}

# --- Indices du tableau /stm32/status ---------------------------------------------------
ST_STATE = 0
ST_HOMED_MASK = 1
ST_LIMIT_MASK = 2
ST_FAULT_CODE = 3
ST_MAX_FERR = 4
ST_LOOP_HZ = 5
ST_CMD_AGE_MS = 6
ST_UPTIME_S = 7
ST_LEN = 8

# --- Bits de défaut -------------------------------------------------------------------
FAULT_COMM = 1 << 0        # watchdog : plus de consigne / heartbeat
FAULT_FOLLOWING = 1 << 1   # écart de poursuite trop grand (moteur décroché ?)
FAULT_LIMIT = 1 << 2       # limite articulaire dépassée
FAULT_HOMING = 1 << 3      # homing échoué (timeout / fin de course absent)
FAULT_ESTOP = 1 << 4       # arrêt d'urgence reçu
FAULT_AXIS_SHIFT = 8       # bits 8..13 : axe concerné

FAULT_NAMES = {
    FAULT_COMM: 'perte de communication (watchdog)',
    FAULT_FOLLOWING: 'écart de poursuite excessif',
    FAULT_LIMIT: 'limite articulaire dépassée',
    FAULT_HOMING: 'échec du homing',
    FAULT_ESTOP: 'arrêt d\'urgence',
}

# --- Commandes /stm32/command (std_msgs/Int32) ----------------------------------------------
CMD_ENABLE = 1      # active les drivers, efface arrêt d'urgence / défaut
CMD_DISABLE = 2     # coupe les drivers
CMD_HOME = 3        # lance le homing
CMD_ZERO_HERE = 4   # (mise en service) prend la position actuelle comme référence, sans fin de course


def decode_faults(code: int) -> str:
    """Texte lisible d'un code de défaut (bits FAULT_* + axes concernés)."""
    parts = [name for bit, name in FAULT_NAMES.items() if code & bit]
    axes = [str(i + 1) for i in range(6) if code & (1 << (FAULT_AXIS_SHIFT + i))]
    text = ', '.join(parts) if parts else 'aucun'
    if axes:
        text += ' (axe ' + ', '.join(axes) + ')'
    return text
