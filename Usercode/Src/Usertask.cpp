
#include "../Inc/Usertask.h"

#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "imu.h"
#include "bmi088.h"
#include "Remcon.h"

extern imu bmi088_imu;
osMessageQueueId_t MessageQueue1;
osMessageQueueAttr_t MessageQueue_attributes = {.name = "test_queue"};
osSemaphoreAttr_t Semaphore_attributes = {.name = "test_sem"};
osSemaphoreId_t Semaphorehandle;
constexpr auto flag1=1u<<0;
constexpr auto flag2=1u<<1;
osEventFlagsAttr_t eventFlags={.name="eventtest"};
osEventFlagsId_t eventFlagId;
osThreadId_t imu_datacalHandle;
constexpr osThreadAttr_t imu_datacal_attributes = {
    .name = "imu_datacal",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };

[[noreturn]] void imu_datacal(void *argument) {
    while (1) {
        //osMessageQueuePut(MessageQueue1,&send,0,0);
        /*
        if (send%10==0)
        {
            osSemaphoreRelease(Semaphorehandle);
        }
        */

        osEventFlagsSet(eventFlagId, flag2);
        osDelay(1000); // Delay for 1000 ms
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
        osEventFlagsSet(eventFlagId, flag1);
        osDelay(7000); // Delay for 1000 ms
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
        osEventFlagsWait(eventFlagId,flag1|flag2,osFlagsWaitAll,osWaitForever);
        osEventFlagsClear(eventFlagId, flag1|flag2);
        //osMessageQueueGet(MessageQueue1,&receive,nullptr,osWaitForever);
        //osSemaphoreAcquire(Semaphorehandle,osWaitForever);
    }
}
void user_task_init() {
    imu_datacalHandle = osThreadNew(imu_datacal, nullptr, &imu_datacal_attributes);
    remconHandle = osThreadNew(remcon, nullptr, &remcon_attributes);
    gimbalHandle = osThreadNew(gimbal, nullptr, &gimbal_attributes);
    MessageQueue1 = osMessageQueueNew(16, sizeof(uint32_t), &MessageQueue_attributes);
    Semaphorehandle = osSemaphoreNew(1, 0, &Semaphore_attributes);
    eventFlagId= osEventFlagsNew(&eventFlags);
}
