/**
 * @file gimbal.c
 * @author sethome
 * @brief
 * @version 0.1
 * @date 2022-11-20
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "gimbal.h"
#include "dm_driver.h"
#include "chassis_move.h"
#include "Global_status.h"

#include "IMU_updata.h"
#include "CAN_receive&send.h"

#include "Stm32_time.h"
#include "stm32f4xx_hal.h"

#include "math.h"
struct gimbal_status gimbal;

/* 大yaw pid 陀螺仪*/
//mit
pid_t imu_big_yaw_control_speed;
pid_t imu_big_yaw_control_tor;

//位置速度
pid_t big_yaw_control_location;
pid_t big_yaw_control_speed;

/*前馈参数*/
float KF = 50.0f;
// 目标值
float Target = 0.0f;
float Pre_Target = 0.0f;
/*停在坡上参数*/
float slope;
/* 大yaw 相关参数*/
float relative_angle_big_yaw;
float imu_bigyaw_dif;
float dm4310_vel_set;
float Time_delay_trs;

// 定义一个静态变量来保存上一次的输出值
static float prev_output = 0.0;
float filtered_tor;

// 云台初始化
void gimbal_init()
{
	//mit模式
//pid_set(&imu_big_yaw_control_speed,1.9f, 0.0, 0.0f, 3.0f, 1.0f);
//pid_set(&imu_big_yaw_control_tor, 1.4f, 0.0, 0.0f, 3.0f, 1.0f);
  pid_set(&imu_big_yaw_control_speed,6.0f, 0.0, 0.03f, 10.0f, 1.0f);
  pid_set(&imu_big_yaw_control_tor, 1.4f, 0.0, 0.01f, 10.0f, 1.0f);
  
  //位置速度
  pid_set(&big_yaw_control_location,1.0f, 0.0, 0.0f, 2.0f, 1.0f);
  pid_set(&big_yaw_control_speed, 1.0f, 0.0, 0.0f, 2.0f, 1.0f);
  
	/*云台相关参数初始化*/

	/* 云台模式*/

}

float slope_calculation(float IMU_pitch, float LOCATION_pitch)
{
	slope = rad2degree(IMU_pitch) - LOCATION_pitch - 48.5;
	return slope;
}

/*****************************big行为控制********************************/
void big_yaw_init()
{
	dm4310_init();											//ID和模式初始化	
	dm4310.ctrl.tor_set = 0.0 ;
}
float ttt=0;
void big_yaw_cal()
{

	//MIT模式
/*************************************************************************************************************/
//双环 陀螺仪
	gimbal.big_yaw_speed = cos(IMU_data.AHRS.pitch) * IMU_data.gyro[2] - sin(IMU_data.AHRS.pitch) * IMU_data.gyro[0];
	gimbal.set_big_yaw_speed =  pid_cal(&imu_big_yaw_control_speed,IMU_data.AHRS.yaw_rad_cnt,chassis.speed_RC.big_yaw);
	dm4310.ctrl.tor_set = pid_cal(&imu_big_yaw_control_tor,gimbal.big_yaw_speed,gimbal.set_big_yaw_speed);
  
//  //双环 电机角度 滤波
//  gimbal.set_big_yaw_speed =  pid_cal(&big_yaw_control_location,dm4310.para.pos,chassis.speed_RC.big_yaw);
//	dm4310.ctrl.tor_set = pid_cal(&imu_big_yaw_control_tor,gimbal.big_yaw_speed,gimbal.set_big_yaw_speed);
  
/*************************************************************************************************************/
//  //位置速度模式
//  gimbal.set_big_yaw_speed = pid_cal(&big_yaw_control_location,IMU_data.AHRS.yaw_rad_cnt,chassis.speed_RC.big_yaw);
//  dm4310.ctrl.pos_set = chassis.speed_RC.big_yaw ;
//  dm4310.ctrl.vel_set = pid_cal(&big_yaw_control_speed,IMU_data.AHRS.yaw_rad_cnt,chassis.speed_RC.big_yaw);

//  //速度模式
//  gimbal.big_yaw_speed = cos(IMU_data.AHRS.pitch) * IMU_data.gyro[2] - sin(IMU_data.AHRS.pitch) * IMU_data.gyro[0];
//	gimbal.set_big_yaw_speed =  pid_cal(&imu_big_yaw_control_speed,IMU_data.AHRS.yaw_rad_cnt,chassis.speed_RC.big_yaw);
//	dm4310.ctrl.vel_set = pid_cal(&imu_big_yaw_control_tor,gimbal.big_yaw_speed,gimbal.set_big_yaw_speed);
  
		dm4310_ctrl_send(&hcan2,&dm4310);		
  
    //帧与帧延迟
//  	if(Get_sys_time_ms() - Time_delay_trs >2)
//    {
//      dm4310_ctrl_send(&hcan2,&dm4310);		
//      Time_delay_trs = Get_sys_time_ms();
//    }
   
}

void big_yaw_update()
{
	dm4310.ctrl.pos_set = IMU_data.AHRS.yaw_rad_cnt;
}

void set_big_yaw_pos_vel(float pos, float vel,motor_t *motor)
{
	dm4310.ctrl.pos_set = pos ;
	dm4310.ctrl.vel_set = vel ;
}

void relative_angle_big_yaw_send()
{
	relative_angle_big_yaw = dm4310.para.pos/PI*180 + 2.0 ;
	
	uint8_t can_send_data [8];
	static CAN_TxHeaderTypeDef tx_message;
	uint32_t send_mail_box;

	tx_message.StdId = 0x05;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	
	float_to_bytes(relative_angle_big_yaw, &can_send_data[0]);  // 将x的字节存入can_data[0]~can_data[3]
	float chassis_imu_data_yaw = IMU_data.AHRS.yaw_rad_cnt;
	float_to_bytes(chassis_imu_data_yaw, &can_send_data[4]);
		
	HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);
}

// 将float按字节拆分
void float_to_bytes(float f, uint8_t *bytes) 
	{
    uint32_t *p = (uint32_t *)&f;  // 将float指针强制转换为uint32_t指针
    uint32_t temp = *p;            // 获取float的二进制表示
    bytes[0] = (temp >> 0) & 0xFF; // 提取最低字节
    bytes[1] = (temp >> 8) & 0xFF; // 提取第二个字节
    bytes[2] = (temp >> 16) & 0xFF; // 提取第三个字节
    bytes[3] = (temp >> 24) & 0xFF; // 提取最高字节
  }
/*****************************big行为控制********************************/
// 一阶 IIR 低通滤波函数
float iir_low_pass_filter(float input, float alpha) 
{
    float output;
    // 根据差分方程计算当前输出
    output = alpha * input + (1 - alpha) * prev_output;
    // 更新上一次的输出值
    prev_output = output;
    return output;
}
//大yaw前馈
float last_in = 0.0;
float T = 0.001;
float big_yaw_feedforward(float in)
{
  float out;
//  out = (in - last_in)/T + in;
  out = (in - last_in)/T ;
  last_in = in;
  return out;
}
