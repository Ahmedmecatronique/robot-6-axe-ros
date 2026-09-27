/* Contrôle des 6 axes (tâche FreeRTOS 1 kHz).
 *
 * Chaque milliseconde :
 *   1. lit les encodeurs -> positions/vitesses articulaires ;
 *   2. anti-rebond des fins de course ;
 *   3. traite les commandes de supervision, l'arrêt d'urgence et le watchdog ;
 *   4. selon l'état : PID de position (READY) ou séquence de homing (HOMING) ;
 *   5. convertit les vitesses articulaires en fréquences de pas (générateur TIM7) ;
 *   6. publie un instantané pour micro-ROS.
 * micro-ROS n'intervient JAMAIS dans cette boucle : s'il plante, les moteurs sont arrêtés par le watchdog. */
#include "control.h"

#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "board.h"
#include "config.h"
#include "pid.h"

typedef enum { HOME_APPROACH = 0, HOME_BACKOFF } home_phase_t;

static struct {
    /* mesures */
    int32_t enc_count[NUM_AXES];
    int32_t enc_zero[NUM_AXES];
    float   q_ref[NUM_AXES];            /* angle articulaire à l'instant du zéro encodeur */
    float   q[NUM_AXES], q_prev[NUM_AXES], v[NUM_AXES];
    /* sortie */
    float   v_out[NUM_AXES];            /* vitesse articulaire commandée après limitation d'accélération */
    float   q_target[NUM_AXES];
    float   hold_q[NUM_AXES];           /* position tenue tant qu'aucune consigne n'est reçue */
    pid_ctrl_t pid[NUM_AXES];
    /* consigne reçue de ROS */
    float   cmd_q[NUM_AXES], cmd_v[NUM_AXES];
    uint32_t cmd_tick;
    uint8_t have_cmd;
    /* état */
    uint8_t  state;
    uint8_t  homed_mask;
    uint32_t fault;
    uint32_t last_hb_tick;
    uint8_t  hb_seen;
    volatile uint8_t estop_req;
    volatile uint8_t link_lost_req;
    /* fins de course */
    uint8_t lim_cnt[NUM_AXES];
    uint8_t lim_state[NUM_AXES];
    /* écart de poursuite */
    uint32_t ferr_since[NUM_AXES];
    float    max_ferr;
    /* homing */
    uint8_t  home_idx;
    home_phase_t home_phase;
    uint32_t home_t0, home_check_t;
    float    home_q_check;
    /* statistiques */
    uint32_t loops, loop_hz, hz_tick;
    uint32_t boot_tick;
    control_snapshot_t snap;
} G;

static QueueHandle_t s_cmd_queue;

/* ------------------------------------------------------------------------------------------------
 *  Utilitaires
 * ---------------------------------------------------------------------------------------------- */
static float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

static void stop_all(void)
{
    for (int i = 0; i < NUM_AXES; i++) {
        G.v_out[i] = 0.0f;
        board_set_step_rate(i, 0.0f, 1);
    }
}

static void reset_pids(void)
{
    for (int i = 0; i < NUM_AXES; i++) {
        pid_reset(&G.pid[i], G.q[i]);
        G.ferr_since[i] = 0u;
        G.hold_q[i] = G.q[i];
    }
    G.have_cmd = 0u;
}

static void enter_estop(void)
{
    stop_all();
    G.state = STATE_ESTOP;
    G.fault |= FAULT_ESTOP;
#if ESTOP_DISABLE_DRIVERS
    board_drivers_enable(0);
#endif
}

static void enter_fault(uint32_t code, int axis)
{
    stop_all();
    G.state = STATE_FAULT;
    G.fault = code | ((axis >= 0) ? (1u << (FAULT_AXIS_SHIFT + (uint32_t)axis)) : 0u);
}

/* Applique une vitesse articulaire (rad/s) à un axe avec limitation d'accélération. */
static void drive_axis(int i, float v_des, float a_max, float dt)
{
    float dv_max = a_max * dt;
    G.v_out[i] += clampf(v_des - G.v_out[i], -dv_max, dv_max);
    board_set_step_rate(i, fabsf(G.v_out[i]) * CFG_STEPS_PER_RAD[i], G.v_out[i] >= 0.0f);
}

/* ------------------------------------------------------------------------------------------------
 *  API appelée depuis micro-ROS
 * ---------------------------------------------------------------------------------------------- */
void control_set_command(const float q[6], const float v[6])
{
    taskENTER_CRITICAL();
    memcpy(G.cmd_q, q, sizeof(G.cmd_q));
    memcpy(G.cmd_v, v, sizeof(G.cmd_v));
    G.cmd_tick = xTaskGetTickCount();
    G.have_cmd = 1u;
    taskEXIT_CRITICAL();
}

void control_estop(int active)
{
    if (active) {
        G.estop_req = 1u;            /* verrouillé : seul CMD_ENABLE l'efface */
    }
}

void control_heartbeat(void)
{
    G.last_hb_tick = xTaskGetTickCount();
}

void control_command(int32_t cmd)
{
    if (s_cmd_queue != NULL) {
        (void)xQueueSend(s_cmd_queue, &cmd, 0);
    }
}

void control_gripper(float opening)
{
    board_gripper_set(opening);
}

void control_link_lost(void)
{
    G.link_lost_req = 1u;
}

void control_get_snapshot(control_snapshot_t *out)
{
    taskENTER_CRITICAL();
    *out = G.snap;
    taskEXIT_CRITICAL();
}

/* ------------------------------------------------------------------------------------------------
 *  Commandes de supervision
 * ---------------------------------------------------------------------------------------------- */
static void process_command(int32_t cmd, uint32_t now)
{
    switch (cmd) {
    case CMD_ENABLE:
        if (G.state == STATE_HOMING) {
            break;                                   /* on n'interrompt pas un homing (utiliser DISABLE) */
        }
        board_drivers_enable(1);
        G.fault = 0u;
        G.estop_req = 0u;
        G.link_lost_req = 0u;
        reset_pids();
        G.hb_seen = 1u;
        G.last_hb_tick = now;                        /* le heartbeat doit arriver dans CFG_WATCHDOG_MS */
        G.state = (G.homed_mask == 0x3Fu) ? STATE_READY : STATE_IDLE;
        break;
    case CMD_DISABLE:
        stop_all();
        board_drivers_enable(0);
        G.homed_mask = 0u;                           /* sans couple, la position est perdue */
        G.state = STATE_DISABLED;
        break;
    case CMD_HOME:
        if (G.state == STATE_IDLE || G.state == STATE_READY) {
            stop_all();
            G.homed_mask = 0u;
            G.home_idx = 0u;
            G.home_phase = HOME_APPROACH;
            G.home_t0 = G.home_check_t = now;
            G.home_q_check = G.q[CFG_HOME_ORDER[0]];
            G.state = STATE_HOMING;
        }
        break;
    case CMD_ZERO_HERE:
        if (G.state == STATE_IDLE || G.state == STATE_READY) {
            for (int i = 0; i < NUM_AXES; i++) {
                G.enc_zero[i] = G.enc_count[i];
                G.q_ref[i] = 0.0f;
                G.q[i] = G.q_prev[i] = 0.0f;
            }
            G.homed_mask = 0x3Fu;
            reset_pids();
            G.state = STATE_READY;
        }
        break;
    default:
        break;
    }
}

/* ------------------------------------------------------------------------------------------------
 *  État PRÊT : suivi de la consigne par PID
 * ---------------------------------------------------------------------------------------------- */
static void run_ready(uint32_t now, float dt)
{
    float cq[NUM_AXES], cv[NUM_AXES];
    uint8_t have;
    uint32_t tick;

    taskENTER_CRITICAL();
    memcpy(cq, G.cmd_q, sizeof(cq));
    memcpy(cv, G.cmd_v, sizeof(cv));
    have = G.have_cmd;
    tick = G.cmd_tick;
    taskEXIT_CRITICAL();

    float age = 0.0f;
    if (have) {
        uint32_t age_ms = now - tick;
        age = (float)((age_ms > EXTRAPOLATION_MAX_MS) ? EXTRAPOLATION_MAX_MS : age_ms) * 0.001f;
    }

    float max_err = 0.0f;
    for (int i = 0; i < NUM_AXES; i++) {
        float qt, ff;
        if (have) {
            qt = cq[i] + cv[i] * age;                /* extrapolation linéaire entre deux messages à 100 Hz */
            ff = cv[i];
        } else {
            qt = G.hold_q[i];
            ff = 0.0f;
        }
        qt = clampf(qt, CFG_Q_MIN[i], CFG_Q_MAX[i]);
        G.q_target[i] = qt;

        float e = qt - G.q[i];
        if (fabsf(e) > max_err) {
            max_err = fabsf(e);
        }

        /* Sécurités : limites articulaires et écart de poursuite (moteur bloqué / décroché) */
        if (G.q[i] < CFG_Q_MIN[i] - SOFT_LIMIT_MARGIN_RAD || G.q[i] > CFG_Q_MAX[i] + SOFT_LIMIT_MARGIN_RAD) {
            enter_fault(FAULT_LIMIT, i);
            return;
        }
        if (fabsf(e) > CFG_FERR_MAX_RAD) {
            if (G.ferr_since[i] == 0u) {
                G.ferr_since[i] = now;
            } else if ((now - G.ferr_since[i]) > FERR_TIME_MS) {
                enter_fault(FAULT_FOLLOWING, i);
                return;
            }
        } else {
            G.ferr_since[i] = 0u;
        }

        float v_des;
        if (fabsf(e) < CFG_DEADBAND_RAD && fabsf(ff) < 1e-3f) {
            v_des = 0.0f;                             /* zone morte : évite le « sifflement » du moteur à l'arrêt */
            G.pid[i].integ *= 0.99f;
        } else {
            v_des = ff + pid_update(&G.pid[i], e, G.q[i], dt, CFG_V_MAX[i]);
        }
        v_des = clampf(v_des, -CFG_V_MAX[i], CFG_V_MAX[i]);
        /* jamais de mouvement vers l'extérieur des limites */
        if ((G.q[i] >= CFG_Q_MAX[i] && v_des > 0.0f) || (G.q[i] <= CFG_Q_MIN[i] && v_des < 0.0f)) {
            v_des = 0.0f;
        }
        drive_axis(i, v_des, CFG_A_MAX[i] * ACCEL_MARGIN, dt);
    }
    G.max_ferr = max_err;
}

/* ------------------------------------------------------------------------------------------------
 *  État HOMING : approche du fin de course, recul lent, référence au front de relâchement
 * ---------------------------------------------------------------------------------------------- */
static void run_homing(uint32_t now, float dt)
{
    const int ax = CFG_HOME_ORDER[G.home_idx];
    const float speed = CFG_HOME_SPEED[ax];
    const int dir = CFG_HOME_DIR[ax];                /* sens du fin de course */

    for (int i = 0; i < NUM_AXES; i++) {
        if (i != ax) {
            G.v_out[i] = 0.0f;
            board_set_step_rate(i, 0.0f, 1);
        }
    }

    /* Délai maximal pour cet axe */
    float travel = CFG_Q_MAX[ax] - CFG_Q_MIN[ax];
    uint32_t timeout_ms = (uint32_t)((travel * 1.3f / speed + HOME_TIMEOUT_MARGIN_S) * 1000.0f);
    if ((now - G.home_t0) > timeout_ms) {
        enter_fault(FAULT_HOMING, ax);
        return;
    }

    /* Contrôle de progression : détecte moteur bloqué, encodeur débranché ou POLARITÉ INVERSÉE */
    float v_target = (G.home_phase == HOME_APPROACH) ? (float)dir * speed
                                                     : -(float)dir * speed / HOME_BACKOFF_SPEED_DIV;
    if ((now - G.home_check_t) >= HOME_PROGRESS_CHECK_MS) {
        float progress = (G.q[ax] - G.home_q_check) * (v_target >= 0.0f ? 1.0f : -1.0f);
        float expected = fabsf(v_target) * ((float)HOME_PROGRESS_CHECK_MS * 0.001f);
        if (progress < HOME_PROGRESS_MIN * expected) {
            enter_fault(FAULT_HOMING, ax);
            return;
        }
        G.home_check_t = now;
        G.home_q_check = G.q[ax];
    }

    if (G.home_phase == HOME_APPROACH) {
        if (G.lim_state[ax]) {                       /* fin de course atteint : on repart lentement en arrière */
            G.home_phase = HOME_BACKOFF;
            G.home_check_t = now;
            G.home_q_check = G.q[ax];
        }
        drive_axis(ax, v_target, CFG_A_MAX[ax], dt);
    } else {
        if (!G.lim_state[ax]) {                      /* front de relâchement = référence de l'axe */
            stop_all();
            G.enc_zero[ax] = G.enc_count[ax];
            G.q_ref[ax] = CFG_HOME_POS[ax];
            G.q[ax] = G.q_prev[ax] = CFG_HOME_POS[ax];
            G.homed_mask |= (uint8_t)(1u << ax);
            G.home_idx++;
            G.home_phase = HOME_APPROACH;
            G.home_t0 = G.home_check_t = now;
            if (G.home_idx >= NUM_AXES) {            /* tous les axes référencés */
                reset_pids();
                G.state = STATE_READY;
            } else {
                G.home_q_check = G.q[CFG_HOME_ORDER[G.home_idx]];
            }
            return;
        }
        drive_axis(ax, v_target, CFG_A_MAX[ax], dt);
    }
}

/* ------------------------------------------------------------------------------------------------
 *  Watchdog
 * ---------------------------------------------------------------------------------------------- */
static void check_watchdog(uint32_t now)
{
    if (G.link_lost_req) {
        G.link_lost_req = 0u;
        if (G.state == STATE_READY || G.state == STATE_HOMING) {
            enter_fault(FAULT_COMM, -1);
            return;
        }
    }
    if (G.state != STATE_READY && G.state != STATE_HOMING) {
        return;
    }
    int hb_stale = G.hb_seen && ((now - G.last_hb_tick) > CFG_WATCHDOG_MS);
    int cmd_stale = (G.state == STATE_READY) && G.have_cmd && ((now - G.cmd_tick) > CFG_WATCHDOG_MS);
    if (hb_stale || cmd_stale) {
        enter_fault(FAULT_COMM, -1);                 /* arrêt immédiat : plus de consigne ou de heartbeat */
    }
}

/* ------------------------------------------------------------------------------------------------
 *  Boucle 1 kHz
 * ---------------------------------------------------------------------------------------------- */
static void control_step(void)
{
    const float dt = CONTROL_DT;
    const uint32_t now = xTaskGetTickCount();

    /* 1) encodeurs -> position / vitesse articulaires */
    for (int i = 0; i < NUM_AXES; i++) {
        G.enc_count[i] += board_encoder_delta(i) * (int32_t)CFG_ENC_SIGN[i];
        G.q[i] = (float)(G.enc_count[i] - G.enc_zero[i]) / CFG_ENC_COUNTS_PER_RAD[i] + G.q_ref[i];
        float v_raw = (G.q[i] - G.q_prev[i]) / dt;
        G.v[i] += VEL_FILTER_ALPHA * (v_raw - G.v[i]);
        G.q_prev[i] = G.q[i];
    }

    /* 2) fins de course : anti-rebond */
    for (int i = 0; i < NUM_AXES; i++) {
        int raw = board_limit_raw(i);
        if (raw != (int)G.lim_state[i]) {
            if (++G.lim_cnt[i] >= LIMIT_DEBOUNCE_MS) {
                G.lim_state[i] = (uint8_t)raw;
                G.lim_cnt[i] = 0u;
            }
        } else {
            G.lim_cnt[i] = 0u;
        }
    }

    /* 3) supervision : commandes, arrêt d'urgence, watchdog */
    int32_t cmd;
    while (xQueueReceive(s_cmd_queue, &cmd, 0) == pdTRUE) {
        process_command(cmd, now);
    }
    if (G.estop_req) {
        G.estop_req = 0u;
        if (G.state != STATE_DISABLED && G.state != STATE_BOOT) {
            enter_estop();
        }
    }
    check_watchdog(now);

    /* 4) selon l'état */
    G.max_ferr = 0.0f;
    switch (G.state) {
    case STATE_READY:
        run_ready(now, dt);
        break;
    case STATE_HOMING:
        run_homing(now, dt);
        break;
    default:
        stop_all();
        for (int i = 0; i < NUM_AXES; i++) {
            G.q_target[i] = G.q[i];
        }
        break;
    }

    /* 5) statistiques et instantané pour micro-ROS */
    G.loops++;
    if ((now - G.hz_tick) >= 1000u) {
        G.loop_hz = G.loops;
        G.loops = 0u;
        G.hz_tick = now;
    }

    taskENTER_CRITICAL();
    for (int i = 0; i < NUM_AXES; i++) {
        G.snap.q[i] = G.q[i];
        G.snap.v[i] = G.v[i];
        G.snap.effort[i] = G.v_out[i] / CFG_V_MAX[i];
        G.snap.q_target[i] = G.q_target[i];
    }
    uint8_t lim = 0u;
    for (int i = 0; i < NUM_AXES; i++) {
        lim |= (uint8_t)(G.lim_state[i] << i);
    }
    G.snap.state = G.state;
    G.snap.homed_mask = G.homed_mask;
    G.snap.limit_mask = lim;
    G.snap.fault = G.fault;
    G.snap.max_ferr = G.max_ferr;
    G.snap.loop_hz = G.loop_hz;
    G.snap.cmd_age_ms = G.have_cmd ? (now - G.cmd_tick) : 0u;
    G.snap.uptime_s = (now - G.boot_tick) / 1000u;
    taskEXIT_CRITICAL();
}

void control_init(void)
{
    memset(&G, 0, sizeof(G));
    for (int i = 0; i < NUM_AXES; i++) {
        /* intégrale bornée pour ne jamais dépasser 20 % de la vitesse max */
        float integ_max = (CFG_KI[i] > 1e-6f) ? (0.2f * CFG_V_MAX[i] / CFG_KI[i]) : 0.0f;
        pid_init(&G.pid[i], CFG_KP[i], CFG_KI[i], CFG_KD[i], integ_max);
    }
    s_cmd_queue = xQueueCreate(8, sizeof(int32_t));
    G.boot_tick = xTaskGetTickCount();
    G.hz_tick = G.boot_tick;
    G.state = STATE_DISABLED;                        /* démarrage : drivers coupés, en attente de CMD_ENABLE */
    G.snap.state = STATE_DISABLED;
    board_drivers_enable(0);
}

void control_task(void *arg)
{
    (void)arg;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));   /* période exacte : 1 kHz */
        control_step();
    }
}
