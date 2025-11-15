
#ifndef SPITEST_REMCON_H
#define SPITEST_REMCON_H
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdint.h>
#define MID_VALUE    0x0400  // 中位值
#define MIN_VALUE    0x016C  // 最小值
#define MAX_VALUE    0x0694  // 最大值

float trans(uint16_t input);
class data {
  public:
  uint32_t ch1;
  uint32_t ch2;
  uint32_t ch3;
  uint32_t ch4;
  float chan1;
  float chan2;
  float chan3;
  float chan4;
  uint8_t s1;
        uint8_t s2;
  void init();
};
class rem {
public:
    float pitch;
    float yaw;
  uint16_t len;
  uint32_t tick;
  uint8_t rxbuffer[8]={0};
  uint8_t rxdata[20]={0};
  data data1;
  void init();
  rem();
  void Handle();

};


#endif // SPITEST_REMCON_H
