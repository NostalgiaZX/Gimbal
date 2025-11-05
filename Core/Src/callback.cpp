#include "imu.h"
#include "bmi088.h"
#include "gpio.h"
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "iwdg.h"
extern imu bmi088_imu;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM6)
    {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */
    else if (htim->Instance == TIM7)
    {
        HAL_IWDG_Refresh(&hiwdg);
    }
    else if (htim->Instance == TIM5)
    {
        bmi088_imu.acc_calculate();
        bmi088_imu.gyro_calculate();
        bmi088_imu.filter(0.4f);
    }
    /* USER CODE END Callback 1 */
}
