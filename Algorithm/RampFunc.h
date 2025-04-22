#include "stdint.h"
#include "struct_typedef.h"
#define _RAMPFUNC_H_
#ifdef  _RAMPFUNC_H_


#pragma anon_unions

// 定义一个用于表示斜坡发生器状态的结构体
 typedef struct RampGenerator
{
    float currentValue; // 当前值
    float targetValue;  // 目标值
    float step;         // 每个控制周期应当改变的数值大小
    int  isBusy;        // 指示斜坡发生器是否正在调整中
} RampGenerator;

// 一个周期内对斜坡发生器状态的更新
void rampIterate(RampGenerator *ramp);
// 初始化斜坡发生器
void rampCal(RampGenerator *ramp, float startValue, float targetValue, float time, float cycleTime);

#endif

// end of file
