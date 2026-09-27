/* Tâche micro-ROS (Humble). Basée sur micro_ros_stm32cubemx_utils : transport UART + DMA de CubeMX. */
#include "microros_app.h"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"                     /* huart2 (généré par CubeMX) */

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int32_multi_array.h>

#include "config.h"
#include "control.h"

/* Fournis par micro_ros_stm32cubemx_utils (dma_transport.c / microros_allocators.c) */
bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);
void *microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void *microros_reallocate(void *pointer, size_t size, void *state);
void *microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void *state);

#define RC_CHECK(x)  do { if ((x) != RCL_RET_OK) { return false; } } while (0)

/* ------------------------------------------------------------------------------------------------
 *  Entités micro-ROS
 * ---------------------------------------------------------------------------------------------- */
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_publisher_t pub_js, pub_status;
static rcl_subscription_t sub_cmd, sub_estop, sub_command, sub_gripper;
static rcl_timer_t timer_js, timer_status;

/* ---- Messages : mémoire STATIQUE (pas d'allocation dynamique après le démarrage) ----------------- */
static sensor_msgs__msg__JointState js_pub_msg;
static rosidl_runtime_c__String js_pub_names[NUM_AXES];
static char js_pub_name_buf[NUM_AXES][16];
static char js_pub_frame_buf[16];
static double js_pub_pos[NUM_AXES], js_pub_vel[NUM_AXES], js_pub_eff[NUM_AXES];

static sensor_msgs__msg__JointState js_sub_msg;
static rosidl_runtime_c__String js_sub_names[NUM_AXES];
static char js_sub_name_buf[NUM_AXES][16];
static char js_sub_frame_buf[16];
static double js_sub_pos[NUM_AXES], js_sub_vel[NUM_AXES], js_sub_eff[NUM_AXES];

static std_msgs__msg__Int32MultiArray status_msg;
static int32_t status_buf[8];

static std_msgs__msg__Bool estop_msg;
static std_msgs__msg__Int32 command_msg;
static std_msgs__msg__Float32 gripper_msg;

static const char *const JOINT_NAMES[NUM_AXES] = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };

static void init_messages(void)
{
    /* /joint_states publié : 6 noms, 6 positions/vitesses/efforts */
    sensor_msgs__msg__JointState__init(&js_pub_msg);
    js_pub_msg.header.frame_id.data = js_pub_frame_buf;
    js_pub_msg.header.frame_id.size = 0;
    js_pub_msg.header.frame_id.capacity = sizeof(js_pub_frame_buf);
    js_pub_msg.name.data = js_pub_names;
    js_pub_msg.name.size = js_pub_msg.name.capacity = NUM_AXES;
    for (int i = 0; i < NUM_AXES; i++) {
        strcpy(js_pub_name_buf[i], JOINT_NAMES[i]);
        js_pub_names[i].data = js_pub_name_buf[i];
        js_pub_names[i].size = strlen(JOINT_NAMES[i]);
        js_pub_names[i].capacity = sizeof(js_pub_name_buf[i]);
    }
    js_pub_msg.position.data = js_pub_pos;
    js_pub_msg.position.size = js_pub_msg.position.capacity = NUM_AXES;
    js_pub_msg.velocity.data = js_pub_vel;
    js_pub_msg.velocity.size = js_pub_msg.velocity.capacity = NUM_AXES;
    js_pub_msg.effort.data = js_pub_eff;
    js_pub_msg.effort.size = js_pub_msg.effort.capacity = NUM_AXES;

    /* /joint_commands reçu : les capacités doivent couvrir le plus grand message attendu */
    sensor_msgs__msg__JointState__init(&js_sub_msg);
    js_sub_msg.header.frame_id.data = js_sub_frame_buf;
    js_sub_msg.header.frame_id.size = 0;
    js_sub_msg.header.frame_id.capacity = sizeof(js_sub_frame_buf);
    js_sub_msg.name.data = js_sub_names;
    js_sub_msg.name.size = 0;
    js_sub_msg.name.capacity = NUM_AXES;
    for (int i = 0; i < NUM_AXES; i++) {
        js_sub_names[i].data = js_sub_name_buf[i];
        js_sub_names[i].size = 0;
        js_sub_names[i].capacity = sizeof(js_sub_name_buf[i]);
    }
    js_sub_msg.position.data = js_sub_pos;
    js_sub_msg.position.size = 0;
    js_sub_msg.position.capacity = NUM_AXES;
    js_sub_msg.velocity.data = js_sub_vel;
    js_sub_msg.velocity.size = 0;
    js_sub_msg.velocity.capacity = NUM_AXES;
    js_sub_msg.effort.data = js_sub_eff;
    js_sub_msg.effort.size = 0;
    js_sub_msg.effort.capacity = NUM_AXES;

    std_msgs__msg__Int32MultiArray__init(&status_msg);
    status_msg.data.data = status_buf;
    status_msg.data.size = status_msg.data.capacity = 8;

    std_msgs__msg__Bool__init(&estop_msg);
    std_msgs__msg__Int32__init(&command_msg);
    std_msgs__msg__Float32__init(&gripper_msg);
}

/* ------------------------------------------------------------------------------------------------
 *  Callbacks
 * ---------------------------------------------------------------------------------------------- */
static void cmd_callback(const void *msgin)
{
    const sensor_msgs__msg__JointState *m = (const sensor_msgs__msg__JointState *)msgin;
    if (m->position.size < NUM_AXES) {
        return;                                          /* message incomplet : ignoré */
    }
    float q[NUM_AXES], v[NUM_AXES];
    for (int i = 0; i < NUM_AXES; i++) {
        q[i] = (float)m->position.data[i];
        v[i] = (m->velocity.size >= NUM_AXES) ? (float)m->velocity.data[i] : 0.0f;
    }
    control_set_command(q, v);                           /* l'ordre des articulations = joint_1..joint_6 */
}

static void estop_callback(const void *msgin)
{
    const std_msgs__msg__Bool *m = (const std_msgs__msg__Bool *)msgin;
    control_heartbeat();                                 /* tout message = signe de vie de safety_node */
    if (m->data) {
        control_estop(1);
    }
}

static void command_callback(const void *msgin)
{
    control_command(((const std_msgs__msg__Int32 *)msgin)->data);
}

static void gripper_callback(const void *msgin)
{
    control_gripper(((const std_msgs__msg__Float32 *)msgin)->data);
}

static void js_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) {
        return;
    }
    control_snapshot_t s;
    control_get_snapshot(&s);
    for (int i = 0; i < NUM_AXES; i++) {
        js_pub_pos[i] = (double)s.q[i];
        js_pub_vel[i] = (double)s.v[i];
        js_pub_eff[i] = (double)s.effort[i];
    }
    int64_t ns = rmw_uros_epoch_nanos();                 /* horloge synchronisée avec l'agent (Pi) */
    js_pub_msg.header.stamp.sec = (int32_t)(ns / 1000000000LL);
    js_pub_msg.header.stamp.nanosec = (uint32_t)(ns % 1000000000LL);
    (void)rcl_publish(&pub_js, &js_pub_msg, NULL);
}

static void status_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) {
        return;
    }
    control_snapshot_t s;
    control_get_snapshot(&s);
    status_buf[0] = (int32_t)s.state;
    status_buf[1] = (int32_t)s.homed_mask;
    status_buf[2] = (int32_t)s.limit_mask;
    status_buf[3] = (int32_t)s.fault;
    status_buf[4] = (int32_t)(s.max_ferr * 1000.0f);     /* milli-radians */
    status_buf[5] = (int32_t)s.loop_hz;
    status_buf[6] = (int32_t)s.cmd_age_ms;
    status_buf[7] = (int32_t)s.uptime_s;
    (void)rcl_publish(&pub_status, &status_msg, NULL);
}

/* ------------------------------------------------------------------------------------------------
 *  Création / destruction des entités
 * ---------------------------------------------------------------------------------------------- */
static bool create_entities(void)
{
    allocator = rcl_get_default_allocator();
    RC_CHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RC_CHECK(rclc_node_init_default(&node, "stm32_micro_ros", "", &support));

    /* Mesures : best effort (une valeur perdue est remplacée 10 ms plus tard) */
    RC_CHECK(rclc_publisher_init_best_effort(&pub_js, &node,
             ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "joint_states"));
    RC_CHECK(rclc_publisher_init_best_effort(&pub_status, &node,
             ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "stm32/status"));

    /* Commandes et arrêt d'urgence : FIABLES */
    RC_CHECK(rclc_subscription_init_default(&sub_cmd, &node,
             ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "joint_commands"));
    RC_CHECK(rclc_subscription_init_default(&sub_estop, &node,
             ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "emergency_stop"));
    RC_CHECK(rclc_subscription_init_default(&sub_command, &node,
             ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "stm32/command"));
    RC_CHECK(rclc_subscription_init_default(&sub_gripper, &node,
             ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "gripper_command"));

    RC_CHECK(rclc_timer_init_default(&timer_js, &support, RCL_MS_TO_NS(10), js_timer_callback));       /* 100 Hz */
    RC_CHECK(rclc_timer_init_default(&timer_status, &support, RCL_MS_TO_NS(100), status_timer_callback)); /* 10 Hz */

    RC_CHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
    RC_CHECK(rclc_executor_add_subscription(&executor, &sub_cmd, &js_sub_msg, &cmd_callback, ON_NEW_DATA));
    RC_CHECK(rclc_executor_add_subscription(&executor, &sub_estop, &estop_msg, &estop_callback, ON_NEW_DATA));
    RC_CHECK(rclc_executor_add_subscription(&executor, &sub_command, &command_msg, &command_callback, ON_NEW_DATA));
    RC_CHECK(rclc_executor_add_subscription(&executor, &sub_gripper, &gripper_msg, &gripper_callback, ON_NEW_DATA));
    RC_CHECK(rclc_executor_add_timer(&executor, &timer_js));
    RC_CHECK(rclc_executor_add_timer(&executor, &timer_status));
    return true;
}

static void destroy_entities(void)
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    (void)rclc_executor_fini(&executor);
    (void)rcl_timer_fini(&timer_js);
    (void)rcl_timer_fini(&timer_status);
    (void)rcl_subscription_fini(&sub_cmd, &node);
    (void)rcl_subscription_fini(&sub_estop, &node);
    (void)rcl_subscription_fini(&sub_command, &node);
    (void)rcl_subscription_fini(&sub_gripper, &node);
    (void)rcl_publisher_fini(&pub_js, &node);
    (void)rcl_publisher_fini(&pub_status, &node);
    (void)rcl_node_fini(&node);
    (void)rclc_support_fini(&support);
}

/* ------------------------------------------------------------------------------------------------
 *  Tâche : attend l'agent, crée les entités, traite les messages, détecte la perte de l'agent
 * ---------------------------------------------------------------------------------------------- */
void microros_task(void *arg)
{
    (void)arg;

    rmw_uros_set_custom_transport(true, (void *)&huart2,
                                  cubemx_transport_open, cubemx_transport_close,
                                  cubemx_transport_write, cubemx_transport_read);

    rcl_allocator_t freertos_allocator = rcutils_get_zero_initialized_allocator();
    freertos_allocator.allocate = microros_allocate;
    freertos_allocator.deallocate = microros_deallocate;
    freertos_allocator.reallocate = microros_reallocate;
    freertos_allocator.zero_allocate = microros_zero_allocate;
    if (!rcutils_set_default_allocator(&freertos_allocator)) {
        for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }    /* erreur fatale : allocateur non installé */
    }

    init_messages();

    for (;;) {
        /* 1) attendre l'agent micro-ROS (démarré sur le Raspberry Pi) */
        while (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) {
            control_link_lost();
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        /* 2) créer les entités */
        if (!create_entities()) {
            destroy_entities();
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        (void)rmw_uros_sync_session(500);                /* synchronise l'horloge pour les horodatages */

        /* 3) boucle de service ; on vérifie l'agent une fois par seconde */
        TickType_t last_ping = xTaskGetTickCount();
        TickType_t last_sync = last_ping;
        for (;;) {
            (void)rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));
            TickType_t now = xTaskGetTickCount();
            if ((now - last_ping) > pdMS_TO_TICKS(1000)) {
                last_ping = now;
                if (rmw_uros_ping_agent(50, 2) != RMW_RET_OK) {
                    break;                               /* agent perdu */
                }
            }
            if ((now - last_sync) > pdMS_TO_TICKS(60000)) {
                last_sync = now;
                (void)rmw_uros_sync_session(200);
            }
            vTaskDelay(1);
        }

        /* 4) liaison perdue : le contrôle arrête les moteurs (watchdog), on recommence */
        control_link_lost();
        destroy_entities();
    }
}
