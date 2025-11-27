
#include "../Inc/Usertask.h"

#include "imu.h"
#include "Remcon.h"
#include "motor.h"
#include "main.h"
#include "can.h"
uint8_t stop_flag=1;
extern CAN_TxHeaderTypeDef txheader;
extern Motor yawmotor;
extern Motor pitchmotor;
extern rem rem1;
extern imu bmi088_imu;
uint32_t ptrnull;
uint32_t count;
//forward intensity
float yawinten=0.0;
float pitchinten=-0.55;
//forward speed
float yawspeed=0.0;
float pitchspeed=0.0;
//forward inensity canshu
float ksin=-0.25f;
float constant=-0.4f;

int16_t pitchcurrent;
int16_t yawcurrent;
float yawangleset=0;
float pitchangleset=80;
osMessageQueueId_t imu_msgque;
osMessageQueueAttr_t imu_msgque_attributes = {.name = "imu_msgque"};
osMessageQueueId_t remcon_msgque;
osMessageQueueAttr_t remcon_msgque_attributes = {.name = "remcon_msgque"};
osSemaphoreAttr_t imudatar_attributes = {.name = "imudatar"};
osSemaphoreId_t imudatarhandle;
osSemaphoreAttr_t remconr_attributes = {.name = "remconr"};
osSemaphoreId_t remconrhandle;
constexpr auto remcondata=1u<<0;
constexpr auto imudata=1u<<1;
osEventFlagsAttr_t eventFlags={.name="gimbal_flag"};
osEventFlagsId_t eventFlagId;
osThreadId_t imu_datacalHandle;
constexpr osThreadAttr_t imu_datacal_attributes = {
    .name = "imu_datacal",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };

[[noreturn]] void imu_datacal(void *argument) {
    while (1) {
        osSemaphoreAcquire(imudatarhandle,osWaitForever);
        bmi088_imu.filter(0.4f);

        osEventFlagsSet(eventFlagId, imudata);
    }
}
osThreadId_t remconHandle;
constexpr osThreadAttr_t remcon_attributes = {
    .name = "remcon",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };

[[noreturn]] void remcon(void *argument) {
    while (1) {
        osSemaphoreAcquire(remconrhandle,osWaitForever);
        rem1.Handle();
        osEventFlagsSet(eventFlagId, remcondata);
    }
}


osThreadId_t gimbalHandle;
constexpr osThreadAttr_t gimbal_attributes = {
    .name = "gimbal",
    .stack_size = 256 * 8,
    .priority = (osPriority_t) osPriorityNormal,
  };
[[noreturn]] void gimbal(void *argument) {
    while (1)
    {
        osEventFlagsWait(eventFlagId,remcondata|imudata,osFlagsWaitAll,osWaitForever);
        osEventFlagsClear(eventFlagId, remcondata|imudata);
        float yawangle=linermap(rem1.yaw,-1.0f,1.0f,-180.0f,180.0f);
        float pitchangle=linermap(rem1.pitch,-1.0f,1.0f,66.0f,96.0f);
        yawmotor.SetPosition(yawangle,0,yawinten);
        pitchmotor.SetPosition(pitchangle,0,pitchforwardinten(pitchmotor.fdb_angle_));
        //yawmotor.SetSpeed(yawspeed,yawinten);
        //pitchmotor.SetSpeed(pitchspeed,pitchinten);
        //pitchmotor.SetIntensity(-0.5);
        yawmotor.Handle();
        pitchmotor.Handle();
        yawcurrent=(int16_t)linermap(yawmotor.output_intensity_,-3.0f,3.0f,-16384.0f,16384.0f);
        pitchcurrent=(int16_t)linermap(pitchmotor.output_intensity_,-3.0f,3.0f,-16384.0f,16384.0f);
        uint8_t txdata[8]={0};
        if (stop_flag==1)
        {
            uint8_t txdata_stop[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
            HAL_CAN_AddTxMessage(&hcan1,&txheader,txdata,&ptrnull);
        }
        else
        {
            txdata[0]=((yawcurrent>>8)&0xFF);
            txdata[1]=(yawcurrent&0xFF);
            txdata[6]=((pitchcurrent>>8)&0xFF);
            txdata[7]=(pitchcurrent&0x00FF);
            HAL_CAN_AddTxMessage(&hcan1,&txheader,txdata,&ptrnull);
        }
        //osMessageQueueGet(MessageQueue1,&receive, nullptr,osWaitForever);
        //osSemaphoreAcquire(Semaphorehandle,osWaitForever);
    }
}
void user_task_init() {
    imu_datacalHandle = osThreadNew(imu_datacal, nullptr, &imu_datacal_attributes);
    remconHandle = osThreadNew(remcon, nullptr, &remcon_attributes);
    gimbalHandle = osThreadNew(gimbal, nullptr, &gimbal_attributes);
    imu_msgque = osMessageQueueNew(16, sizeof(uint32_t), &imu_msgque_attributes);
    remcon_msgque = osMessageQueueNew(16, sizeof(uint32_t), &remcon_msgque_attributes);
    imudatarhandle = osSemaphoreNew(1, 0, &imudatar_attributes);
    eventFlagId= osEventFlagsNew(&eventFlags);
}
