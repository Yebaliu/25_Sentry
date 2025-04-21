#define __CAP_H__
#ifdef __CAP_H__

#include "stdint.h"

typedef struct // μ?èY×′ì??á11ì?
{
	uint8_t set_max_power; // 最大功率
	uint8_t cache_energy;  // 缓冲电量
  uint16_t cacheEnergylimit;
  uint8_t state;

	float remain_vol;	   // 剩余电压
	float prediect_energy; // 预测容量 0 - 100%
} cap_t;

extern cap_t cap;

void cap_handle_message(uint8_t data[8]);
void cap_update(void);
int cap_set_power(uint8_t set);
float cap_get_remain_vol(void);
float cap_get_predict_energy(void);
#endif
