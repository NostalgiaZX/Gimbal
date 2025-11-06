#include "imu.h"
#include "bmi088.h"
#include "gpio.h"
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "iwdg.h"
#include "Usertask.h"
#include "usart.h"
#include "Remcon.h"
uint8_t rxbuffer[8]={0};
extern rem rem1;
extern imu bmi088_imu;
extern osSemaphoreId_t imudatarhandle;;
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
        osSemaphoreRelease(imudatarhandle);
    }
    /* USER CODE END Callback 1 */
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart==&huart3) {
        if (Size>0) {
            for (int i=0;i<Size;i++) {
                rem1.rxbuffer[i]=rxbuffer[i];
                rem1.rxdata[rem1.len+i]=rem1.rxbuffer[i];
                rem1.len++;
                rem1.len=rem1.len==18?0:rem1.len;

            }
            HAL_UARTEx_ReceiveToIdle_DMA(&huart3,rxbuffer,1);
        }
    }
}
