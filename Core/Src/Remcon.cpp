

#include "../Inc/Remcon.h"
float trans(uint16_t input) {
  float result;

  // 限制输入值在有效范围内
  if (input <= MIN_VALUE) {
    return -1.0f;
  } else if (input >= MAX_VALUE) {
    return 1.0f;
  }

  // 计算相对于中位值的偏移
  if (input == MID_VALUE) {
    result = 0.0f;  // 中位值对应0
  } else if (input < MID_VALUE) {
    // 小于中位值：映射到-1 ~ 0
    uint32_t range = MID_VALUE - MIN_VALUE;  // 低位范围（中位到最小）
    uint32_t offset = MID_VALUE - input;     // 距离中位值的偏移
    result = -((float)offset / range);       // 负向映射
  } else {
    // 大于中位值：映射到0 ~ 1
    uint32_t range = MAX_VALUE - MID_VALUE;  // 高位范围（中位到最大）
    uint32_t offset = input - MID_VALUE;     // 距离中位值的偏移
    result = (float)offset / range;          // 正向映射
  }

  return result;
}
void data::init() {
    ch1=0;
    ch2=0;
    ch3=0;
    ch4=0;
    s1=0;
    s2=0;
  }
void rem::init() {
  data1.init();
  tick=0;

}
rem::rem(){
  init();
}
void rem::Handle() {

  data1.ch1 = ((int16_t)rxdata[0] | ((int16_t)rxdata[1] << 8)) & 0x07FF;
  data1.ch2 = (((int16_t)rxdata[1] >> 3) | ((int16_t)rxdata[2] << 5)) & 0x07FF;
  data1.ch3 = (((int16_t)rxdata[2] >> 6) | ((int16_t)rxdata[3] << 2) | ((int16_t)rxdata[4] << 10)) & 0x07FF;
  data1.ch4 = (((int16_t)rxdata[4] >> 1) | ((int16_t)rxdata[5] << 7)) & 0x07FF;
  data1.chan1=trans(data1.ch1);
        data1.chan2=trans(data1.ch2);
  data1.chan3=trans(data1.ch3);
        data1.chan4=trans(data1.ch4);
  data1.s1=((rxdata[5]>>4)&0x0C)>>2;
  data1.s2=(rxdata[5]>>4)&0x03;
}
rem rem1;

void dmareceive() {
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3,rem1.rxbuffer,8);
}