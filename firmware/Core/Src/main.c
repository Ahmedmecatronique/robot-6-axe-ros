/* Firmware du bras 6 axes — STM32F446RE (Nucleo-64), FreeRTOS + micro-ROS.
 *
 * Ce main.c remplace celui généré par STM32CubeIDE. Il suppose que CubeMX a généré (voir firmware/README.md) :
 *   - gpio.c, dma.c, usart.c avec USART2 + DMA RX/TX (MX_GPIO_Init, MX_DMA_Init, MX_USART2_UART_Init, huart2),
 *   - FreeRTOS (CMSIS_V2) avec la base de temps HAL sur TIM6,
 *   - stm32f4xx_hal_timebase_tim.c.
 * Les timers encodeurs, le générateur de pas (TIM7) et le servo (TIM10) sont configurés par board.c
 * (accès direct aux registres) : ne PAS les activer dans CubeMX.
 */
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "gpio.h"
#include "usart.h"

#include "app.h"
#include "board.h"

void SystemClock_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART2_UART_Init();          /* liaison micro-ROS : 921600 bauds, DMA (configurée dans CubeMX) */

    board_init();                   /* GPIO STEP/DIR/EN, encodeurs, TIM7 (pas), TIM10 (servo) : drivers COUPÉS */

    osKernelInitialize();
    app_start();                    /* tâche contrôle 1 kHz + tâche micro-ROS */
    osKernelStart();

    while (1) {
    }
}

/* Horloge 180 MHz : HSE 8 MHz (MCO du ST-LINK, mode bypass) -> PLL /4 x180 /2 ; APB1 = 45 MHz, APB2 = 90 MHz */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_BYPASS;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 4;
    osc.PLL.PLLN = 180;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 2;
    osc.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        Error_Handler();
    }

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/* Base de temps HAL sur TIM6 (le SysTick est réservé à FreeRTOS) */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        HAL_IncTick();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    /* Défaut fatal : arrêt des impulsions STEP et coupure des drivers */
    board_drivers_enable(0);
    while (1) {
    }
}
