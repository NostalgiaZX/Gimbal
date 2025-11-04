
#include "../Inc/Usertask.h"

#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "imu.h"
#include "bmi088.h"
#include "Remcon.h"

uint32_t send=0;
uint32_t receive=0;
osMessageQueueId_t MessageQueue1;
osMessageQueueAttr_t MessageQueue_attributes = {.name = "test_queue"};
osSemaphoreAttr_t Semaphore_attributes = {.name = "test_sem"};
osSemaphoreId_t Semaphorehandle;
constexpr auto flag1=1u<<0;
constexpr auto flag2=1u<<1;
osEventFlagsAttr_t eventFlags={.name="eventtest"};
osEventFlagsId_t eventFlagId;
uint32_t count=0;
osThreadId_t UserTask1Handle;
constexpr osThreadAttr_t UserTask1_attributes = {
    .name = "UserTask1",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };

[[noreturn]] void test_task1(void *argument) {
    while (1) {
        const auto tick = osKernelGetTickCount();
        send++;
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
osThreadId_t UserTask3Handle;
constexpr osThreadAttr_t UserTask3_attributes = {
    .name = "UserTask3",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };

[[noreturn]] void test_task3(void *argument) {
    while (1) {
        const auto tick = osKernelGetTickCount();
        osEventFlagsSet(eventFlagId, flag1);
        osDelay(7000); // Delay for 1000 ms
    }
}


osThreadId_t UserTask2Handle;
constexpr osThreadAttr_t UserTask2_attributes = {
    .name = "UserTask2",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };
[[noreturn]] void test_task2(void *argument) {
    while (1)
    {
        osEventFlagsWait(eventFlagId,flag1|flag2,osFlagsWaitAll,osWaitForever);
        osEventFlagsClear(eventFlagId, flag1|flag2);
        //osMessageQueueGet(MessageQueue1,&receive,nullptr,osWaitForever);
        //osSemaphoreAcquire(Semaphorehandle,osWaitForever);
        receive++;
    }
}
void user_task_init() {
    UserTask1Handle = osThreadNew(test_task1, nullptr, &UserTask1_attributes);
    UserTask2Handle = osThreadNew(test_task2, nullptr, &UserTask2_attributes);
    UserTask3Handle = osThreadNew(test_task3, nullptr, &UserTask3_attributes);
    MessageQueue1 = osMessageQueueNew(16, sizeof(uint32_t), &MessageQueue_attributes);
    Semaphorehandle = osSemaphoreNew(1, 0, &Semaphore_attributes);
    eventFlagId= osEventFlagsNew(&eventFlags);
}
