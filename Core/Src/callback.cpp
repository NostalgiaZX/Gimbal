#include "imu.h"
#include "bmi088.h"
#include "gpio.h"
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "iwdg.h"
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {
        HAL_IWDG_Refresh(&hiwdg);
    }

}

