/* Couche matérielle — voir board.h et config.h (câblage). */
#include "board.h"
#include "config.h"
#include "stm32f4xx.h"

/* ------------------------------------------------------------------------------------------------
 *  Tables de câblage
 * ---------------------------------------------------------------------------------------------- */
#define STEP_PORT   GPIOC                       /* STEP1..6 = PC0..PC5 : une seule écriture BSRR par tick */
#define STEP_MASK   0x3Fu
#define EN_PORT     GPIOC
#define EN_PIN      8u

static GPIO_TypeDef * const DIR_PORT[NUM_AXES] = { GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB };
static const uint8_t DIR_PIN[NUM_AXES] = { 12u, 13u, 14u, 15u, 4u, 5u };

static GPIO_TypeDef * const LIM_PORT[NUM_AXES] = { GPIOC, GPIOC, GPIOC, GPIOC, GPIOD, GPIOA };
static const uint8_t LIM_PIN[NUM_AXES] = { 9u, 10u, 11u, 12u, 2u, 4u };

static TIM_TypeDef * const ENC_TIM[NUM_AXES] = { TIM1, TIM2, TIM3, TIM4, TIM5, TIM8 };
static uint16_t enc_last[NUM_AXES];

/* ------------------------------------------------------------------------------------------------
 *  Utilitaires GPIO
 * ---------------------------------------------------------------------------------------------- */
#define MODE_INPUT   0u
#define MODE_OUTPUT  1u
#define MODE_AF      2u

static void pin_mode(GPIO_TypeDef *g, uint32_t pin, uint32_t mode)
{
    g->MODER = (g->MODER & ~(3u << (pin * 2u))) | (mode << (pin * 2u));
}

static void pin_pull_up(GPIO_TypeDef *g, uint32_t pin)
{
    g->PUPDR = (g->PUPDR & ~(3u << (pin * 2u))) | (1u << (pin * 2u));
}

static void pin_fast(GPIO_TypeDef *g, uint32_t pin)
{
    g->OSPEEDR |= (3u << (pin * 2u));
}

static void pin_af(GPIO_TypeDef *g, uint32_t pin, uint32_t af)
{
    g->AFR[pin >> 3u] = (g->AFR[pin >> 3u] & ~(0xFu << ((pin & 7u) * 4u))) | (af << ((pin & 7u) * 4u));
    pin_mode(g, pin, MODE_AF);
}

/* ------------------------------------------------------------------------------------------------
 *  Générateur de pas : accumulateur de phase 32 bits, ISR à 20 kHz.
 *  À chaque tick : phase += incrément ; un dépassement (retenue) = une impulsion STEP.
 *  Incrément = f_pas / 20000 * 2^32 ; f_pas max 10 kHz (0.5 * 2^32), 8 kHz par défaut.
 * ---------------------------------------------------------------------------------------------- */
static volatile uint32_t g_inc[NUM_AXES];       /* incrément de phase (0 = axe arrêté) */
static volatile uint8_t  g_dir[NUM_AXES];       /* niveau demandé sur la broche DIR */
static uint8_t  g_dir_applied[NUM_AXES];
static uint32_t g_phase[NUM_AXES];

void TIM7_IRQHandler(void)
{
    TIM7->SR = 0u;                                   /* efface UIF */
    STEP_PORT->BSRR = (uint32_t)STEP_MASK << 16u;    /* fin des impulsions du tick précédent */

    uint32_t rising = 0u;
    for (uint32_t i = 0u; i < NUM_AXES; i++) {
        uint8_t d = g_dir[i];
        if (d != g_dir_applied[i]) {                 /* changement de sens : DIR d'abord, impulsion au tick suivant */
            g_dir_applied[i] = d;
            DIR_PORT[i]->BSRR = d ? (1u << DIR_PIN[i]) : (1u << (DIR_PIN[i] + 16u));
            continue;
        }
        uint32_t inc = g_inc[i];
        if (inc != 0u) {
            uint32_t p = g_phase[i];
            uint32_t n = p + inc;
            g_phase[i] = n;
            if (n < p) {                             /* retenue = impulsion */
                rising |= (1u << i);
            }
        }
    }
    if (rising != 0u) {
        STEP_PORT->BSRR = rising;
    }
}

/* ------------------------------------------------------------------------------------------------
 *  Initialisation
 * ---------------------------------------------------------------------------------------------- */
static void enc_init(TIM_TypeDef *t)
{
    t->PSC = 0u;
    t->ARR = 0xFFFFu;                                    /* on lit toujours 16 bits (même TIM2/TIM5) */
    t->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;           /* SMS = 011 : mode encodeur x4 (TI1 et TI2) */
    t->CCMR1 = (1u << TIM_CCMR1_CC1S_Pos) | (1u << TIM_CCMR1_CC2S_Pos)   /* CH1/CH2 = entrées TI1/TI2 */
             | (0x3u << TIM_CCMR1_IC1F_Pos) | (0x3u << TIM_CCMR1_IC2F_Pos); /* filtre anti-parasites */
    t->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;             /* polarité normale */
    t->CNT = 0u;
    t->CR1 = TIM_CR1_CEN;
}

void board_init(void)
{
    /* Horloges */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM5EN
                 |  RCC_APB1ENR_TIM7EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM8EN | RCC_APB2ENR_TIM10EN;
    (void)RCC->APB2ENR;                                  /* délai de propagation de l'horloge */

    /* Sorties : niveaux sûrs AVANT de passer en sortie (drivers coupés, STEP/DIR bas) */
    EN_PORT->BSRR = (1u << EN_PIN);                      /* /EN haut = drivers coupés */
    STEP_PORT->BSRR = (uint32_t)STEP_MASK << 16u;
    for (uint32_t i = 0u; i < NUM_AXES; i++) {
        DIR_PORT[i]->BSRR = (1u << (DIR_PIN[i] + 16u));
        pin_mode(STEP_PORT, i, MODE_OUTPUT);  pin_fast(STEP_PORT, i);
        pin_mode(DIR_PORT[i], DIR_PIN[i], MODE_OUTPUT);  pin_fast(DIR_PORT[i], DIR_PIN[i]);
        g_dir[i] = g_dir_applied[i] = 0u;
        g_inc[i] = 0u;
    }
    pin_mode(EN_PORT, EN_PIN, MODE_OUTPUT);

    /* Fins de course : entrées avec pull-up */
    for (uint32_t i = 0u; i < NUM_AXES; i++) {
        pin_mode(LIM_PORT[i], LIM_PIN[i], MODE_INPUT);
        pin_pull_up(LIM_PORT[i], LIM_PIN[i]);
    }

    /* Encodeurs : broches en alternate function + pull-up (encodeurs à collecteur ouvert) */
    pin_af(GPIOA, 8u, 1u);   pin_pull_up(GPIOA, 8u);     /* TIM1_CH1 */
    pin_af(GPIOA, 9u, 1u);   pin_pull_up(GPIOA, 9u);     /* TIM1_CH2 */
    pin_af(GPIOA, 15u, 1u);  pin_pull_up(GPIOA, 15u);    /* TIM2_CH1 (JTDI libéré, SWD conservé) */
    pin_af(GPIOB, 3u, 1u);   pin_pull_up(GPIOB, 3u);     /* TIM2_CH2 (SWO libéré) */
    pin_af(GPIOA, 6u, 2u);   pin_pull_up(GPIOA, 6u);     /* TIM3_CH1 */
    pin_af(GPIOA, 7u, 2u);   pin_pull_up(GPIOA, 7u);     /* TIM3_CH2 */
    pin_af(GPIOB, 6u, 2u);   pin_pull_up(GPIOB, 6u);     /* TIM4_CH1 */
    pin_af(GPIOB, 7u, 2u);   pin_pull_up(GPIOB, 7u);     /* TIM4_CH2 */
    pin_af(GPIOA, 0u, 2u);   pin_pull_up(GPIOA, 0u);     /* TIM5_CH1 */
    pin_af(GPIOA, 1u, 2u);   pin_pull_up(GPIOA, 1u);     /* TIM5_CH2 */
    pin_af(GPIOC, 6u, 3u);   pin_pull_up(GPIOC, 6u);     /* TIM8_CH1 */
    pin_af(GPIOC, 7u, 3u);   pin_pull_up(GPIOC, 7u);     /* TIM8_CH2 */
    for (uint32_t i = 0u; i < NUM_AXES; i++) {
        enc_init(ENC_TIM[i]);
        enc_last[i] = (uint16_t)ENC_TIM[i]->CNT;
    }

    /* Servo de pince : TIM10_CH1 sur PB8, 50 Hz, résolution 1 µs (horloge timer APB2 = 180 MHz) */
    pin_af(GPIOB, 8u, 3u);
    TIM10->PSC = 179u;
    TIM10->ARR = 19999u;
    TIM10->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;   /* PWM mode 1 */
    TIM10->CCER = TIM_CCER_CC1E;
    TIM10->CCR1 = GRIPPER_US_OPEN;
    TIM10->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
    TIM10->EGR = TIM_EGR_UG;

    /* Générateur de pas : TIM7 (basic), horloge 90 MHz -> /90 = 1 MHz -> ARR = 49 -> 20 kHz */
    TIM7->PSC = 89u;
    TIM7->ARR = (1000000u / STEP_ISR_HZ) - 1u;
    TIM7->DIER = TIM_DIER_UIE;
    NVIC_SetPriority(TIM7_IRQn, 1u);                     /* plus prioritaire que FreeRTOS (n'appelle aucune API RTOS) */
    NVIC_EnableIRQ(TIM7_IRQn);
    TIM7->CR1 = TIM_CR1_CEN;
}

/* ------------------------------------------------------------------------------------------------
 *  API
 * ---------------------------------------------------------------------------------------------- */
void board_drivers_enable(int enable)
{
    EN_PORT->BSRR = enable ? (1u << (EN_PIN + 16u)) : (1u << EN_PIN);   /* /EN actif bas */
}

int32_t board_encoder_delta(int axis)
{
    uint16_t now = (uint16_t)ENC_TIM[axis]->CNT;
    int16_t d = (int16_t)(uint16_t)(now - enc_last[axis]);   /* différence modulo 2^16 : gère le débordement */
    enc_last[axis] = now;
    return (int32_t)d;
}

void board_set_step_rate(int axis, float steps_per_s, int positive)
{
    if (steps_per_s < 0.0f) {
        steps_per_s = 0.0f;
    }
    if (steps_per_s > CFG_MAX_STEP_RATE_HZ) {
        steps_per_s = CFG_MAX_STEP_RATE_HZ;
    }
    uint8_t level = (uint8_t)((positive ? 1 : 0) ^ ((CFG_DIR_SIGN[axis] < 0) ? 1 : 0));
    uint32_t inc = (uint32_t)(steps_per_s * (4294967296.0f / (float)STEP_ISR_HZ));
    g_dir[axis] = level;
    g_inc[axis] = inc;
}

int board_limit_raw(int axis)
{
    return ((LIM_PORT[axis]->IDR >> LIM_PIN[axis]) & 1u) == 0u;   /* contact vers GND = actif */
}

void board_gripper_set(float opening)
{
    if (opening < 0.0f) {
        opening = 0.0f;
    }
    if (opening > 1.0f) {
        opening = 1.0f;
    }
    TIM10->CCR1 = (uint32_t)((float)GRIPPER_US_CLOSED + opening * (float)(GRIPPER_US_OPEN - GRIPPER_US_CLOSED));
}
