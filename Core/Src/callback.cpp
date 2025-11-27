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
#include <math.h>

#include "can.h"
#include "motor.h"
//can read and send
extern Motor yawmotor;
extern Motor pitchmotor;
uint32_t ptr;
extern uint8_t rxdata[8];
extern uint8_t txdata[8];
extern CAN_TxHeaderTypeDef txheader;
extern CAN_RxHeaderTypeDef rxheader;
extern uint8_t stop_flag;
float targetspeed=25;
float targetangle=20;
float forwardspeed=0;
float forwardinten=0.7;
float degree=90.0f;
//remcon read buffer
uint8_t rxbuffer[8]={0};
extern rem rem1;
extern imu bmi088_imu;
extern osSemaphoreId_t imudatarhandle;
extern osSemaphoreId_t remconrhandle;
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
    //imu data update
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
                if (rem1.len==18) {
                    rem1.len=0;
                    osSemaphoreRelease(remconrhandle);
                }

            }

            HAL_UARTEx_ReceiveToIdle_DMA(&huart3,rxbuffer,1);
        }
    }
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance==CAN1)
    {
        HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&rxheader,rxdata);
        //0x208 is pitch
        if (rxheader.StdId==0x208)
        {
            pitchmotor.canrxmsgcallback(rxdata);
        }
        //0x205 is yaw
        else if (rxheader.StdId==0x205)
        {
            yawmotor.canrxmsgcallback(rxdata);
        }
    }
}