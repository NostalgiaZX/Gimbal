
#include "../Inc/Usertask.h"

#include "cmsis_os.h"
#include "cmsis_os2.h"
/*
uint32_t send=0;
uint32_t receive=0;
osMessageQueueId_t MessageQueue1;
osMessageQueueAttr_t MessageQueue_attributes = {.name = "test_queue"};
osSemaphoreAttr_t Semaphore_attributes = {.name = "test_sem"};
osSemaphoreId_t Semaphorehandle;
constexpr auto flag=1u<<0;
constexpr auto flag=1u<<1;
osEventFlagsAttr_t eventFlags={.name="eventtest"};
osEventFlagsId_t eventFlagId;
*/
uint32_t count=0;
osThreadId_t UserTask1Handle;
constexpr osThreadAttr_t UserTask1_attributes = {
    .name = "UserTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };

void test_task1(void *argument) {
    while (1) {
        const auto tick = osKernelGetTickCount();
        count++;
        //osMessageQueuePut(MessageQueue1,&send,0,0);
        //osSemaphoreRelease(Semaphorehandle);
        osDelayUntil(tick+1000); // Delay for 1000 ms
    }
}


/*
osThreadId_t UserTask2Handle;
constexpr osThreadAttr_t UserTask2_attributes = {
    .name = "UserTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };
void test_task2(void *argument) {
    while (1)
    {
        osMessageQueueGet(MessageQueue1,&receive,nullptr,osWaitForever);
        osSemaphoreAcquire(Semaphorehandle,osWaitForever);
    }
}*/
void user_task_init() {
    UserTask1Handle = osThreadNew(test_task1, nullptr, &UserTask1_attributes);
    //UserTask2Handle = osThreadNew(test_task2, nullptr, &UserTask2_attributes);
    //MessageQueue1 = osMessageQueueNew(16, sizeof(uint32_t), &MessageQueue_attributes);
    //Semaphorehandle = osSemaphoreNew(1, 0, &Semaphore_attributes);
}
