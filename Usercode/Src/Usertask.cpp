
#include "../Inc/Usertask.h"

#include "imu.h"
#include "Remcon.h"

uint8_t stop_flag;
extern rem rem1;
extern imu bmi088_imu;
uint32_t count;
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
        //mahony算法（目前是线性滤波）
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
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };
[[noreturn]] void gimbal(void *argument) {
    while (1)
    {
        osEventFlagsWait(eventFlagId,remcondata|imudata,osFlagsWaitAll,osWaitForever);
        osEventFlagsClear(eventFlagId, remcondata|imudata);
        //osMessageQueueGet(MessageQueue1,&receive,nullptr,osWaitForever);
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
