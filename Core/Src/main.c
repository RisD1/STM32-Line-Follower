
/* USER CODE BEGIN Header */
/*
 * main.c
 * STM32F411CEU6 - Line Following Robot
 *
 * HOW TO USE:
 * Set exactly ONE RUN_* option to 1.
 * Set all other RUN_* options to 0.
 *
 * Start with sensor testing. Raise the wheels for motor tests.
 */
/* USER CODE END Header */

#include "main.h"
#include "gpio.h"
#include "dma.h"
#include "adc.h"
#include "tim.h"

#include "switch_input.h"
#include "ir_sensors.h"
#include "line_position.h"
#include "motor_control.h"
#include "line_following.h"

/* =========================================================
   TEST SELECTION
   ========================================================= */

#define RUN_SENSOR_TEST       1
#define RUN_LINE_POSITION     0
#define RUN_DIP_SWITCH_TEST   0

#define RUN_FORWARD_TEST      0
#define RUN_BACKWARD_TEST     0

#define RUN_LEFT_TURN_TEST    0
#define RUN_RIGHT_TURN_TEST   0

#define RUN_LINE_FOLLOWING    0
#define RUN_STOP_TEST         0

/* =========================================================
   FUNCTION DECLARATIONS
   ========================================================= */

void SystemClock_Config(void);
void Error_Handler(void);

/* =========================================================
   MAIN
   ========================================================= */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    /* Initialize STM32 peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();

    MX_TIM1_Init();  /* Encoder timer */
    MX_TIM2_Init();  /* Encoder timer */
    MX_TIM3_Init();  /* Motor PWM timer */

    /* Initialize software modules */
    MotorControl_Init();
    IR_Sensors_Init();
    LinePosition_Init();
    LineFollowing_Init();

#if RUN_DIP_SWITCH_TEST
    SwitchInput_Init();
#endif

    /* Start safely */
    MotorControl_Stop();

    /* Start a selected timed turn once */
#if RUN_LEFT_TURN_TEST
    MotorControl_TurnLeft90_Start();
#elif RUN_RIGHT_TURN_TEST
    MotorControl_TurnRight90_Start();
#endif

    while (1)
    {
#if RUN_SENSOR_TEST
        /* Inspect sensor readings in the debugger */
        IR_Sensors_Update();
#endif

#if RUN_LINE_POSITION
        /* Refresh sensors, then calculate line position */
        IR_Sensors_Update();
        LinePosition_Update();
#endif

#if RUN_DIP_SWITCH_TEST
        /* Read the four DIP switches */
        SwitchInput_Update();
#endif

#if RUN_FORWARD_TEST
        /* Continuous forward motion at 15% */
        MotorControl_Forward(15);
#endif

#if RUN_BACKWARD_TEST
        /* Continuous backward motion at 15% */
        MotorControl_Backward(15);
#endif

#if RUN_LEFT_TURN_TEST || RUN_RIGHT_TURN_TEST
        /* Stop the timed turn when its deadline expires */
        MotorControl_TurnUpdate();
#endif

#if RUN_LINE_FOLLOWING
        /* Line-following controller */
        LineFollowing_Update();
#endif

#if RUN_STOP_TEST
        /* Keep motors stopped */
        MotorControl_Stop();
#endif

        HAL_Delay(10);
    }
}

/* =========================================================
   SYSTEM CLOCK CONFIGURATION
   HSI 16 MHz -> PLL -> SYSCLK 100 MHz
   ========================================================= */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;

    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 200;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    {
        Error_Handler();
    }
}

/* =========================================================
   ERROR HANDLER
   ========================================================= */

void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
        /* Stop here if initialization fails */
    }
}
