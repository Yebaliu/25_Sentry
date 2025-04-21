/**
 * @file chassis_move.h
 * @author sethome
 * @brief 底盘控制文件
 * @version 0.1
 * @date 2022-11-20
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "struct_typedef.h"
#include "referee_handle_pack.h"

#define __chassis_move_H_
#ifdef __chassis_move_H_

/* motorID 1 %++++++% 0
				++++
				++++
			2 %++++++% 3 */
#define chassis_move_FR CAN_2_1
#define chassis_move_FL CAN_1_2
#define chassis_move_BL CAN_1_1
#define chassis_move_BR CAN_2_2

#define chassis_turn_FR CAN_2_5
#define chassis_turn_FL CAN_1_5
#define chassis_turn_BL CAN_1_6
#define chassis_turn_BR CAN_2_6

struct chassis_status
{
	struct
    {
        float set, now, last, offset;
        float stable;
			  float imu_set, imu_now, imu_last, imu_offect;			
				float turn_FR_speed;
				float set_turn_FR_speed;
    } turn_FR;
		struct
    {
        float set, now, last, offset;
        float stable;
			  float imu_set, imu_now, imu_last, imu_offect;		
				float turn_FL_speed;
				float set_turn_FL_speed;
    } turn_FL;
		struct
    {
        float set, now, last, offset;
        float stable;
			  float imu_set, imu_now, imu_last, imu_offect;			
				float turn_BL_speed;
				float set_turn_BL_speed;
    } turn_BL;
		struct
    {
        float set, now, last, offset;
        float stable;
			  float imu_set, imu_now, imu_last, imu_offect;			
				float turn_BR_speed;
				float set_turn_BR_speed;
    } turn_BR;
	struct
	{
		float x, y, r,yaw;
		float now_x, now_y, now_r;
		float last_x, last_y, last_r;
		float max_x, max_y, max_r; // m/s
	} speed;

	struct
	{
		float x,y,r,big_yaw;
	} speed_RC;
	
	struct
	{
		float now_x, now_y, now_r;
		float max_x, max_y, max_r; // m/s^2
	} acc;
	enum chassis_status_e
	{
		park = 0,
		move,
	} status;

	int32_t wheel_current[4]; // PID输出的电调电流
	double wheel_speed[4];
	double angle_set[4];
	double angle_now[4];

	uint8_t is_open_cap;
};
extern struct chassis_status chassis;

struct cap
{
	float cap_vol;
	float cap_now_vol;
	float cap_last_vol;
	float cap_error_vol;
};
extern struct cap cap_chassis;

extern ext_power_heat_data_t power_heat_data;

extern float Power;
extern float wheel_rpm[4];

void chassis_move_init(void);				// 底盘移动初始化
void val_limit(float *val, float MAX);

void Chassic_course_solving(float x,float y,float w);
void Chassis_course_updata();
void Chassis_course_pid_cal();
void Chassis_velocity_calc(float vx, float vy, float vw);
void chassis_receive_from_gimbal_1(uint8_t data[8]);
void chassis_receive_from_gimbal_2(uint8_t data[8]);
float Angle_Limit (float angle ,float max);
fp32 Find_min_angle(fp32 angle1,fp32 angle2);
//妙妙工具
float bytes_to_float(uint8_t *bytes);
float math_modulus_normalization(float x, float modulus);

#endif

// end of file
