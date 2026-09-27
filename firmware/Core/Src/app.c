#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "control.h"
#include "microros_app.h"

/* Priorités : le contrôle (1 kHz) est ABSOLUMENT prioritaire sur micro-ROS. */
#define CONTROL_PRIORITY   (configMAX_PRIORITIES - 1)
#define MICROROS_PRIORITY  (tskIDLE_PRIORITY + 3)

/* Piles en mots de 4 octets */
#define CONTROL_STACK_WORDS   768
#define MICROROS_STACK_WORDS  3000       /* ~12 Ko : rclc + sérialisation XRCE */

void app_start(void)
{
    control_init();
    (void)xTaskCreate(control_task, "control", CONTROL_STACK_WORDS, NULL, CONTROL_PRIORITY, NULL);
    (void)xTaskCreate(microros_task, "microros", MICROROS_STACK_WORDS, NULL, MICROROS_PRIORITY, NULL);
}
