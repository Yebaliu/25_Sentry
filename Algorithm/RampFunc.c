#include "RampFunc.h"
// 一个周期内对斜坡发生器状态的更新
void rampIterate(RampGenerator *ramp)
{
	if (ramp->isBusy)
    {	
        if (ramp->currentValue < ramp->targetValue)
        {                                     // 如果当前值小于目标值
            ramp->currentValue += ramp->step; // 增大当前值
            if (ramp->currentValue > ramp->targetValue)
            { // 避免超调
                ramp->currentValue = ramp->targetValue;
            }
        }

        else if (ramp->currentValue > ramp->targetValue)
        {   
			ramp->currentValue -= ramp->step;  //step等于正数时
			if (ramp->currentValue < ramp->targetValue)// 避免超调
				{ 
					ramp->currentValue = ramp->targetValue;
				}			
        }
        // 判断是否达到目标
        if (ramp->currentValue == ramp->targetValue)
        {
            ramp->isBusy =0 ; // 达到目标，标记为不忙碌
        }
    }	
}
// 初始化斜坡发生器
/**
  * @brief          斜坡函数计算
  * @param[in]      startValue
  * @param[in]     	targetValue
  * @param[in]     	time  每次叠加的时间
  * @param[in]     	cycleTime   达到目标值所需要的时间
  * @retval         none
  */
//这个函数本身是基于正向的方向，例如4-》10，但是如果是让-4-》-10那就出问题了
void rampCal(RampGenerator *ramp, float startValue, float targetValue, float time, float cycleTime)
{
	ramp->currentValue = startValue;
    ramp->targetValue = targetValue;
    // 计算步进值，这里需要注意的是，确保斜坡时间和周期时间都不为零来避免除以零的错误
    if (time != 0 && cycleTime != 0)
    {
		if(targetValue - startValue >0)
			ramp->step = (targetValue - startValue) *(cycleTime/time);
		else
			ramp->step = -(targetValue - startValue) *(cycleTime/time);//比fabs好用（
    }
    else
    {
        ramp->step = 0; // 出错情况下设置为0，避免非法操作
    }
    ramp->isBusy = 1; // 标记为忙碌
}
