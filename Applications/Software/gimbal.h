/**
 * @file gimbal.h
 * @author sethome
 * @brief
 * @version 0.1
 * @date 2022-11-20
 *
 * @copyright Copyright (c) 2022
 *
 */
#define __GIMBAL_H__
#ifdef __GIMBAL_H__

#include "pid.h"
#include "stdint.h"
#include "dm_driver.h"

#define PI 3.1415926f

enum gimbal_status_e
{
    LOCATION = 0,
    SPEED,
    ABSOLUTE,
    zero_force,
};

struct gimbal_status
{
    // �趨��̨����ģʽ
		enum gimbal_status_e big_yaw_status;

		struct
    {
        float set, now, last, absoulte_offset, location_offset;
        float stable;
    } big_yaw;
    float big_yaw_speed;
    float set_big_yaw_speed;
};

extern struct gimbal_status gimbal;

// �ⲿ����
void gimbal_init(void);    // ��ʼ����̨

float slope_calculation(float IMU_pitch, float LOCATION_pitch); // ������������ĽǶ�

//big_yaw
void big_yaw_init();
void big_yaw_cal();
void big_yaw_update();
void relative_angle_big_yaw_send();
void float_to_bytes(float f, uint8_t *bytes) ;
float iir_low_pass_filter(float input, float alpha);
float big_yaw_feedforward(float in);//��yawǰ��
// end of file

#endif
