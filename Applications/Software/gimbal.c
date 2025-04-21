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

/* ��yaw pid ������*/
//mit
pid_t imu_big_yaw_control_speed;
pid_t imu_big_yaw_control_tor;

//λ���ٶ�
pid_t big_yaw_control_location;
pid_t big_yaw_control_speed;

/*ǰ������*/
float KF = 50.0f;
// Ŀ��ֵ
float Target = 0.0f;
float Pre_Target = 0.0f;
/*ͣ�����ϲ���*/
float slope;
/* ��yaw ��ز���*/
float relative_angle_big_yaw;
float imu_bigyaw_dif;
float dm4310_vel_set;
float Time_delay_trs;

// ����һ����̬������������һ�ε����ֵ
static float prev_output = 0.0;
float filtered_tor;

// ��̨��ʼ��
void gimbal_init()
{
	//mitģʽ
//pid_set(&imu_big_yaw_control_speed,1.9f, 0.0, 0.0f, 3.0f, 1.0f);
//pid_set(&imu_big_yaw_control_tor, 1.4f, 0.0, 0.0f, 3.0f, 1.0f);
  pid_set(&imu_big_yaw_control_speed,6.0f, 0.0, 0.03f, 10.0f, 1.0f);
  pid_set(&imu_big_yaw_control_tor, 1.4f, 0.0, 0.01f, 10.0f, 1.0f);
  
  //λ���ٶ�
  pid_set(&big_yaw_control_location,1.0f, 0.0, 0.0f, 2.0f, 1.0f);
  pid_set(&big_yaw_control_speed, 1.0f, 0.0, 0.0f, 2.0f, 1.0f);
  
	/*��̨��ز�����ʼ��*/

	/* ��̨ģʽ*/

}

float slope_calculation(float IMU_pitch, float LOCATION_pitch)
{
	slope = rad2degree(IMU_pitch) - LOCATION_pitch - 48.5;
	return slope;
}

/*****************************big��Ϊ����********************************/
void big_yaw_init()
{
	dm4310_init();											//ID��ģʽ��ʼ��	
	dm4310.ctrl.tor_set = 0.0 ;
}
float ttt=0;
void big_yaw_cal()
{

	//MITģʽ
/*************************************************************************************************************/
//˫�� ������
	gimbal.big_yaw_speed = cos(IMU_data.AHRS.pitch) * IMU_data.gyro[2] - sin(IMU_data.AHRS.pitch) * IMU_data.gyro[0];
	gimbal.set_big_yaw_speed =  pid_cal(&imu_big_yaw_control_speed,IMU_data.AHRS.yaw_rad_cnt,chassis.speed_RC.big_yaw);
	dm4310.ctrl.tor_set = pid_cal(&imu_big_yaw_control_tor,gimbal.big_yaw_speed,gimbal.set_big_yaw_speed);
  
//  //˫�� ����Ƕ� �˲�
//  gimbal.set_big_yaw_speed =  pid_cal(&big_yaw_control_location,dm4310.para.pos,chassis.speed_RC.big_yaw);
//	dm4310.ctrl.tor_set = pid_cal(&imu_big_yaw_control_tor,gimbal.big_yaw_speed,gimbal.set_big_yaw_speed);
  
/*************************************************************************************************************/
//  //λ���ٶ�ģʽ
//  gimbal.set_big_yaw_speed = pid_cal(&big_yaw_control_location,IMU_data.AHRS.yaw_rad_cnt,chassis.speed_RC.big_yaw);
//  dm4310.ctrl.pos_set = chassis.speed_RC.big_yaw ;
//  dm4310.ctrl.vel_set = pid_cal(&big_yaw_control_speed,IMU_data.AHRS.yaw_rad_cnt,chassis.speed_RC.big_yaw);

//  //�ٶ�ģʽ
//  gimbal.big_yaw_speed = cos(IMU_data.AHRS.pitch) * IMU_data.gyro[2] - sin(IMU_data.AHRS.pitch) * IMU_data.gyro[0];
//	gimbal.set_big_yaw_speed =  pid_cal(&imu_big_yaw_control_speed,IMU_data.AHRS.yaw_rad_cnt,chassis.speed_RC.big_yaw);
//	dm4310.ctrl.vel_set = pid_cal(&imu_big_yaw_control_tor,gimbal.big_yaw_speed,gimbal.set_big_yaw_speed);
  
		dm4310_ctrl_send(&hcan2,&dm4310);		
  
    //֡��֡�ӳ�
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
	
	float_to_bytes(relative_angle_big_yaw, &can_send_data[0]);  // ��x���ֽڴ���can_data[0]~can_data[3]
	float chassis_imu_data_yaw = IMU_data.AHRS.yaw_rad_cnt;
	float_to_bytes(chassis_imu_data_yaw, &can_send_data[4]);
		
	HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);
}

// ��float���ֽڲ��
void float_to_bytes(float f, uint8_t *bytes) 
	{
    uint32_t *p = (uint32_t *)&f;  // ��floatָ��ǿ��ת��Ϊuint32_tָ��
    uint32_t temp = *p;            // ��ȡfloat�Ķ����Ʊ�ʾ
    bytes[0] = (temp >> 0) & 0xFF; // ��ȡ����ֽ�
    bytes[1] = (temp >> 8) & 0xFF; // ��ȡ�ڶ����ֽ�
    bytes[2] = (temp >> 16) & 0xFF; // ��ȡ�������ֽ�
    bytes[3] = (temp >> 24) & 0xFF; // ��ȡ����ֽ�
  }
/*****************************big��Ϊ����********************************/
// һ�� IIR ��ͨ�˲�����
float iir_low_pass_filter(float input, float alpha) 
{
    float output;
    // ���ݲ�ַ��̼��㵱ǰ���
    output = alpha * input + (1 - alpha) * prev_output;
    // ������һ�ε����ֵ
    prev_output = output;
    return output;
}
//��yawǰ��
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
