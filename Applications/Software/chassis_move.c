
#include "stdlib.h"
#include "stdio.h"
#include "stdint.h"

#include "CAN_receive&send.h"
#include "IMU_updata.h"
#include "cap_ctl.h"

#include "chassis_move.h"
#include "gimbal.h"
#include "referee_handle_pack.h"
#include "Global_status.h"
#include "arm_math.h"

#include "Stm32_time.h"

#include "math.h"
#include "RampFunc.h" // 引入斜坡函数。

// wheel conf
#define WHEEL_RADIUS 0.26282f // m
#define Rotation_radius  0.37169f  //m
#define PI 3.1415926f
// car conf
#define ROLLER_DISTANCE 100 // mm  轴距
#define WHEELS_DISTANCE 100 // mm  轮距
// mm/s
#define FR 0
#define FL 1
#define BL 2
#define BR 3
//零点定义
#define TURN_FR_ANGLE 7560   //7560
#define TURN_FL_ANGLE 4009   //
#define TURN_BL_ANGLE 2766   //122.18
#define TURN_BR_ANGLE 6181		//6181
//判断用于检查
float judgement = 1 ;
float encoder_difference;
float shortest_encoder;

//计算转向时用到的角度
static fp32 deta[4]={45.0f,135.0f,225.0f,315.0f};
//轮子向前的零点
fp32 X_AXIS_ECD[4]= {7560,4009,2766,6181};  //7560
//静止时候保护变速器的零点
static fp32 still[4] = {7560,4009,2766,6181};
//求出的角度
fp32 set_angle[4] = {0,0,0,0};
//角度转化成的编码器值
fp32 set_mangle[4] = {0,0,0,0};
fp32 last_set_mangle[4] = {0,0,0,0};
//上一次的角度
fp32 last_angle[4] = {0,0,0,0};
//标志位
int fllg[4] = {0,0,0,0};
static int flag_course=1;
//延时时候所用的计数
int time = 0;
//电机转向设置
static int dirt[4]={1,1,1,-1};

struct chassis_status chassis;
struct cap cap_chassis; // 电容组

float wheel_rpm[4]; // 底盘速度数组

// 行进电机速度环PID
pid_t motor_speed_3508[4];
// 转向电机双环PID
pid_t motor_location_6020[4];
pid_t motor_speed_6020[4];
//--目标速度
float target_velocity[4]={0,0,0,0};
// 底盘跟随PID
pid_t chassis_follow;
// 轮组方位角
const float Wheel_Azimuth[4] = {PI / 4.0f,
                                    3.0f * PI / 4.0f,
                                    5.0f * PI / 4.0f,
                                    7.0f * PI / 4.0f,};
const float Wheel_Radius = 0.058f;
// 轮向电机角速度目标值
float Target_Wheel_Omega[4];
// 舵向电机角度目标值
float Target_Steer_Angle[4];

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

// 初始化底盘
void chassis_move_init()
{
	chassis.speed.max_x = 8.0f; // m/s
	chassis.speed.max_y = 8.0f; // m/s
	chassis.speed.max_r = 5.0f; //

	chassis.acc.max_x = 2.5f; // 1m/^2
	chassis.acc.max_y = 2.5f; // m/^2
	chassis.acc.max_r = 2.5f; //

	/***********************************行进电机单速度环控制*******************************************/
	pid_set(&motor_speed_3508[FR], 1000, 0, 0.01, MAX_CURRENT, 0);
	pid_set(&motor_speed_3508[FL], 1000, 0, 0.01, MAX_CURRENT, 0);
	pid_set(&motor_speed_3508[BL], 1000, 0, 0.01, MAX_CURRENT, 0);
	pid_set(&motor_speed_3508[BR], 1000, 0, 0.01, MAX_CURRENT, 0);
	/***********************************转向电机双环控制************************************************/
	pid_set(&motor_location_6020[FR], 20.0, 0, 0.0  ,9000, 0);
	pid_set(&motor_location_6020[FL], 20.0, 0, 0.0  ,9000, 0);
	pid_set(&motor_location_6020[BL], 20.0, 0, 0.0  ,9000, 0);
	pid_set(&motor_location_6020[BR], 20.0, 0, 0.0  ,9000, 0);

	pid_set(&motor_speed_6020[FR], 40, 0, 0, 10000, 0);
	pid_set(&motor_speed_6020[FL], 40, 0, 0, 10000, 0);
	pid_set(&motor_speed_6020[BL], 40, 0, 0, 10000, 0);
	pid_set(&motor_speed_6020[BR], 40, 0, 0, 10000, 0);

	chassis.turn_FR.now = 0;
	chassis.turn_FR.set = TURN_FR_ANGLE/8192*360-90;
	chassis.turn_FR.offset = 0;
	chassis.turn_FR.stable = 0;
	chassis.turn_FR.set_turn_FR_speed = 0;
	
	chassis.turn_FL.now = 0;
	chassis.turn_FL.set = TURN_FL_ANGLE/8192*360-90;
	chassis.turn_FL.offset = 0;
	chassis.turn_FL.stable = 0;
	chassis.turn_FL.set_turn_FL_speed = 0;
	
	chassis.turn_BL.now = 0;
	chassis.turn_BL.set = TURN_BL_ANGLE/8192*360-90;
	chassis.turn_BL.offset = 0;
	chassis.turn_BL.stable = 0;
	chassis.turn_BL.set_turn_BL_speed = 0;
	
	chassis.turn_BR.now = 0;
	chassis.turn_BR.set = TURN_BR_ANGLE/8192*360-90;
	chassis.turn_BR.offset = 0;
	chassis.turn_BR.stable = 0;
	chassis.turn_BR.set_turn_BR_speed = 0;
	
	//旋转速度分解的角度deta
	  for(int i=0;i<4;i++)
	     deta[i]=deta[i]*PI/180.0f;//角度转弧度
			 
	// srand(2); // 初始化一个随机数种子，为了之后变速小陀螺使用
}
//舵向角度解算
void Chassic_course_solving(float x,float y,float w)
{
	static fp32 set_angle_last[4]={TURN_FR_ANGLE,TURN_FL_ANGLE,TURN_BL_ANGLE,TURN_BR_ANGLE};
	//线速度
	 w=w*Rotation_radius;
	 int16_t angle_temp[4];

	//旋转运动
	if(x==0&&y==0&&w==0)
	{
				judgement = 1 ;

        //RC无输入,不进行最短路径判断,零点位置
        set_mangle[0] = X_AXIS_ECD[0]/8192.0*360.0;
        set_mangle[1] = X_AXIS_ECD[1]/8192.0*360.0;
        set_mangle[2] = X_AXIS_ECD[2]/8192.0*360.0;
        set_mangle[3] = X_AXIS_ECD[3]/8192.0*360.0;

	}
	else
	{
		judgement = 2 ;
    
		set_angle[3] =   atan2((y-w*0.707107f),(x+w*0.707107f))*180.0f/PI;    
		set_angle[2] =	 atan2((y-w*0.707107f),(x-w*0.707107f))*180.0f/PI;
		set_angle[1] =	 atan2((y+w*0.707107f),(x-w*0.707107f))*180.0f/PI;
		set_angle[0] =	 atan2((y+w*0.707107f),(x+w*0.707107f))*180.0f/PI;    
        
    last_angle[0] = set_mangle[0];
		last_angle[1] = set_mangle[1];
		last_angle[2] = set_mangle[2];
		last_angle[3] = set_mangle[3];
	}

    set_mangle[0] =	X_AXIS_ECD[0]/8192*360+set_angle[0];
		set_mangle[1] =	X_AXIS_ECD[1]/8192*360+set_angle[1];
		set_mangle[2] =	X_AXIS_ECD[2]/8192*360+set_angle[2];
		set_mangle[3] =	X_AXIS_ECD[3]/8192*360+set_angle[3];
  
  //角度赋值
  //--在这里设置的需要的角度对应在编码器上的位置
	chassis.turn_FR.set = (int32_t)set_mangle[0];
	chassis.turn_FL.set = (int32_t)set_mangle[1];
	chassis.turn_BL.set = (int32_t)set_mangle[2];
	chassis.turn_BR.set = (int32_t)set_mangle[3];

	set_angle_last[0]=set_mangle[0];
	set_angle_last[1]=set_mangle[1];
	set_angle_last[2]=set_mangle[2];
	set_angle_last[3]=set_mangle[3];
}	
//航向电机数据更新
void Chassis_course_updata()
{
//--用原始数据计算出一些数据
	decode_as_6020(chassis_turn_FR);
	decode_as_6020(chassis_turn_FL);
	decode_as_6020(chassis_turn_BL);
	decode_as_6020(chassis_turn_BR);
//--编码器赋值
	chassis.turn_FR.now = get_motor_data(chassis_turn_FR).ecd/8192.0*360.0;
	chassis.turn_FL.now = get_motor_data(chassis_turn_FL).ecd/8192.0*360.0;
	chassis.turn_BL.now = get_motor_data(chassis_turn_BL).ecd/8192.0*360.0;
	chassis.turn_BR.now = get_motor_data(chassis_turn_BR).ecd/8192.0*360.0;
//--转速赋值
	chassis.turn_FR.turn_FR_speed = get_motor_data(chassis_turn_FR).speed_rpm*6;
	chassis.turn_FL.turn_FL_speed = get_motor_data(chassis_turn_FL).speed_rpm*6;
	chassis.turn_BL.turn_BL_speed = get_motor_data(chassis_turn_BL).speed_rpm*6;
	chassis.turn_BR.turn_BR_speed = get_motor_data(chassis_turn_BR).speed_rpm*6;

}
//航向PID计算
void Chassis_course_pid_cal()
{
  //FR
  if((chassis.turn_FR.set-chassis.turn_FR.now)>=90.0)
    {
      dirt[0] = -1;
      chassis.turn_FR.set-=180.0;
    }
    else if((chassis.turn_FR.set-chassis.turn_FR.now)<=-90.0)
    {
      dirt[0] = -1;
      chassis.turn_FR.set+=180.0;
    }
    else if((chassis.turn_FR.set-chassis.turn_FR.now)>=-90.0&&(chassis.turn_FR.set-chassis.turn_FR.now<=90.0))
    {
      dirt[0] = 1;
      chassis.turn_FR.set=chassis.turn_FR.set;
    }
  //FL
  if((chassis.turn_FL.set-chassis.turn_FL.now)>=90.0)
    {
      dirt[1] = -1;
      chassis.turn_FL.set-=180.0;
    }
    else if((chassis.turn_FL.set-chassis.turn_FL.now)<=-90.0)
    {
      dirt[1] = -1;
      chassis.turn_FL.set+=180.0;
    }
    else if((chassis.turn_FL.set-chassis.turn_FL.now)>=-90.0&&(chassis.turn_FL.set-chassis.turn_FL.now<=90.0))
    {
      dirt[1] = 1;
      chassis.turn_FL.set=chassis.turn_FL.set;
    }
    //BL
  if((chassis.turn_BL.set-chassis.turn_BL.now)>=90.0)
    {
      dirt[2] = -1;
      chassis.turn_BL.set-=180.0;
    }
    else if((chassis.turn_BL.set-chassis.turn_BL.now)<=-90.0)
    {
      dirt[2] = -1;
      chassis.turn_BL.set+=180.0;
    }
    else if((chassis.turn_BL.set-chassis.turn_BL.now)>=-90.0&&(chassis.turn_BL.set-chassis.turn_BL.now<=90.0))
    {
      dirt[2] = 1;
      chassis.turn_BL.set=chassis.turn_BL.set;
    }
    //BR
  if((chassis.turn_BR.set-chassis.turn_BR.now)>=90.0)
    {
      dirt[3] = 1;
      chassis.turn_BR.set-=180.0;
    }
    else if((chassis.turn_BR.set-chassis.turn_BR.now)<=-90.0)
    {
      dirt[3] = 1;
      chassis.turn_BR.set+=180.0;
    }
    else if((chassis.turn_BR.set-chassis.turn_BR.now)>=-90.0&&(chassis.turn_BR.set-chassis.turn_BR.now<=90.0))
    {
      dirt[3] = -1;
      chassis.turn_BR.set=chassis.turn_BR.set;
    }
  /******************************************************************/
//  //back_up
////FR
//	if(chassis.turn_FR.set-chassis.turn_FR.now>=180.0)
//	{
//	chassis.turn_FR.turn_FR_speed = pid_cal(&motor_location_6020[FR], chassis.turn_FR.now, chassis.turn_FR.set-360.0);
//	}
//	else if(chassis.turn_FR.set-chassis.turn_FR.now<=-180.0)
//	{
//	chassis.turn_FR.turn_FR_speed = pid_cal(&motor_location_6020[FR], chassis.turn_FR.now, chassis.turn_FR.set+360.0);
//	}
//	else
//	{
//	chassis.turn_FR.turn_FR_speed = pid_cal(&motor_location_6020[FR], chassis.turn_FR.now, chassis.turn_FR.set);
//	}
//  //FL
//	if(chassis.turn_FL.set-chassis.turn_FL.now>=180.0)
//	{
//	chassis.turn_FL.turn_FL_speed = pid_cal(&motor_location_6020[FL], chassis.turn_FL.now, chassis.turn_FL.set-360.0);
//	}
//	else if(chassis.turn_FL.set-chassis.turn_FL.now<=-180.0)
//	{
//	chassis.turn_FL.turn_FL_speed = pid_cal(&motor_location_6020[FL], chassis.turn_FL.now, chassis.turn_FL.set+360.0);
//	}
//	else
//	{
//	chassis.turn_FL.turn_FL_speed = pid_cal(&motor_location_6020[FL], chassis.turn_FL.now, chassis.turn_FL.set);
//	}
//	//BL
//	if(chassis.turn_BL.set-chassis.turn_BL.now>=180.0)
//	{
//	chassis.turn_BL.turn_BL_speed = pid_cal(&motor_location_6020[BL], chassis.turn_BL.now, chassis.turn_BL.set-360.0);
//	}
//	else if(chassis.turn_BL.set-chassis.turn_BL.now<=-180.0)
//	{
//	chassis.turn_BL.turn_BL_speed = pid_cal(&motor_location_6020[BL], chassis.turn_BL.now, chassis.turn_BL.set+360.0);
//	}
//	else
//	{
//		chassis.turn_BL.turn_BL_speed = pid_cal(&motor_location_6020[BL], chassis.turn_BL.now, chassis.turn_BL.set);
//	}
//	//BR
//	if(chassis.turn_BR.set-chassis.turn_BR.now>=180.0)
//	{
//	chassis.turn_BR.turn_BR_speed = pid_cal(&motor_location_6020[BR], chassis.turn_BR.now, chassis.turn_BR.set-360.0);
//	}
//	else if(chassis.turn_BR.set-chassis.turn_BR.now<=-180.0)
//	{
//	chassis.turn_BR.turn_BR_speed = pid_cal(&motor_location_6020[BR], chassis.turn_BR.now, chassis.turn_BR.set+360.0);
//	}
//	else
//	{
//	chassis.turn_BR.turn_BR_speed = pid_cal(&motor_location_6020[BR], chassis.turn_BR.now, chassis.turn_BR.set);
//	}
  /******************************************************************/
//--更新下电机数据
	set_motor(pid_cal(&motor_speed_6020[FR], get_motor_data(chassis_turn_FR).speed_rpm, chassis.turn_FR.turn_FR_speed), chassis_turn_FR);
	set_motor(pid_cal(&motor_speed_6020[FL], get_motor_data(chassis_turn_FL).speed_rpm, chassis.turn_FL.turn_FL_speed), chassis_turn_FL);
	set_motor(pid_cal(&motor_speed_6020[BL], get_motor_data(chassis_turn_BL).speed_rpm, chassis.turn_BL.turn_BL_speed), chassis_turn_BL);
	set_motor(pid_cal(&motor_speed_6020[BR], get_motor_data(chassis_turn_BR).speed_rpm, chassis.turn_BR.turn_BR_speed), chassis_turn_BR);
//	set_motor(0, chassis_turn_FR);
//	set_motor(0, chassis_turn_FL);
//	set_motor(0, chassis_turn_BL);
//	set_motor(0, chassis_turn_BR);
}
//计算行进电机速度及电流
void Chassis_velocity_calc(float vx, float vy, float vw)
{
  //
  //计算和速度
	vw=vw*Rotation_radius;
	float  V[4]={0,0,0,0};
	//分别计算四个轮子的速度大小
	V[3]=sqrt((vx+vw*sinf(deta[0]))*(vx+vw*sinf(deta[0]))+(vy-vw*cosf(deta[0]))*(vy-vw*cosf(deta[0])));
	V[2]=sqrt((vx-vw*sinf(deta[1]))*(vx-vw*sinf(deta[1]))+(vy-vw*cosf(deta[1]))*(vy-vw*cosf(deta[1])));
	V[1]=sqrt((vx-vw*sinf(deta[2]))*(vx-vw*sinf(deta[2]))+(vy+vw*cosf(deta[2]))*(vy+vw*cosf(deta[2])));
	V[0]=-sqrt((vx+vw*sinf(deta[3]))*(vx+vw*sinf(deta[3]))+(vy+vw*cosf(deta[3]))*(vy+vw*cosf(deta[3])));

	if(vw != 0)
	{
	V[0] = V[0];
	V[1] = V[1];
	V[2] = V[2];
	V[3] = V[3];

	}
	
    for(int i =0;i<4;i++)
	{
		if(fllg[i] == 1)
		{
			V[i] = -V[i];
			fllg[i] = 0;
		}
	}
	//最大速度限制
	val_limit(&vx, chassis.speed.max_x);
	val_limit(&vy, chassis.speed.max_y);
	val_limit(&vw, chassis.speed.max_r);
   //--电机数据更新
	decode_as_3508(chassis_move_FR);
	decode_as_3508(chassis_move_FL);
	decode_as_3508(chassis_move_BL);
	decode_as_3508(chassis_move_BR);
   //--确定目标速度的正负
   target_velocity[FR] =     V[0]*dirt[0];
   target_velocity[FL] =     V[1]*dirt[1];
   target_velocity[BL] =     V[2]*dirt[2];
	 target_velocity[BR] =     V[3]*dirt[3];

	//计算马达电流
	chassis.wheel_current[FR] = pid_cal(&motor_speed_3508[FR], get_motor_data(chassis_move_FR).round_speed * WHEEL_RADIUS * PI, target_velocity[FR]);
	chassis.wheel_current[FL] = pid_cal(&motor_speed_3508[FL], get_motor_data(chassis_move_FL).round_speed * WHEEL_RADIUS * PI, target_velocity[FL]);
	chassis.wheel_current[BL] = pid_cal(&motor_speed_3508[BL], get_motor_data(chassis_move_BL).round_speed * WHEEL_RADIUS * PI, target_velocity[BL]);
	chassis.wheel_current[BR] = pid_cal(&motor_speed_3508[BR], get_motor_data(chassis_move_BR).round_speed * WHEEL_RADIUS * PI, target_velocity[BR]);

	//发送马达电流 
	set_motor(chassis.wheel_current[FR], chassis_move_FR);
	set_motor(chassis.wheel_current[FL], chassis_move_FL);
	set_motor(chassis.wheel_current[BL], chassis_move_BL);
	set_motor(chassis.wheel_current[BR], chassis_move_BR);
//		set_motor(0, chassis_move_FR);
//		set_motor(0, chassis_move_FL);
//		set_motor(0, chassis_move_BL);
//		set_motor(0, chassis_move_BR);
}

/*******************************************************/
////运动学解算
//void kinematic_solution(float vx, float vy, float vw)
//{
//  float turn_angle[4];
//  turn_angle[0] = get_motor_data(chassis_turn_FR).angle;
//  turn_angle[1] = get_motor_data(chassis_turn_FL).angle;
//  turn_angle[2] = get_motor_data(chassis_turn_BL).angle;
//  turn_angle[3] = get_motor_data(chassis_turn_BR).angle;
//  
//  for(int i=0;i<4;i++)
//  {
//    float tmp_velocity_x, tmp_velocity_y, tmp_velocity_modulus;

//    // 解算到每个轮组的具体线速度
//    tmp_velocity_x = vx - vw * Rotation_radius * arm_sin_f32(Wheel_Azimuth[i]);
//    tmp_velocity_y = vy + vw * Rotation_radius * arm_cos_f32(Wheel_Azimuth[i]);
//    arm_sqrt_f32(tmp_velocity_x * tmp_velocity_x + tmp_velocity_y * tmp_velocity_y, &tmp_velocity_modulus);

//    // 根据线速度决定轮向电机角速度
//    Target_Wheel_Omega[i] = tmp_velocity_modulus / Wheel_Radius;

//    // 根据速度的xy分量分别决定舵向电机角度
//    if (tmp_velocity_modulus == 0.0f)
//      {
//        // 排除除零问题
//        Target_Steer_Angle[0] = get_motor_data(chassis_turn_FR).angle;
//        Target_Steer_Angle[1] = get_motor_data(chassis_turn_FL).angle;
//        Target_Steer_Angle[2] = get_motor_data(chassis_turn_BL).angle;
//        Target_Steer_Angle[3] = get_motor_data(chassis_turn_BR).angle;
//      }
//    else
//      {
//        // 没有除零问题
//        Target_Steer_Angle[i] = atan2f(tmp_velocity_y, tmp_velocity_x);
//      }
//  }
//  steer_motor_kinematics_nearest_transposition();
//}
////舵向电机依照轮向电机目标角速度就近转位
//void steer_motor_kinematics_nearest_transposition()
//{
//  for (int i = 0; i < 4; i++)
//    {
//        float tmp_delta_angle = math_modulus_normalization(Target_Steer_Angle[i] - Motor_Steer[i].Get_Now_Angle(), 2.0f * PI);

//        // 根据转动角度范围决定是否需要就近转位
//        if (-PI / 2.0f <= tmp_delta_angle && tmp_delta_angle <= PI / 2.0f)
//        {
//            // ±PI / 2之间无需反向就近转位
//            Target_Steer_Angle[i] = tmp_delta_angle + Motor_Steer[i].Get_Now_Angle();
//        }
//        else
//        {
//            // 需要反转扣圈情况
//            Target_Steer_Angle[i] = math_modulus_normalization(tmp_delta_angle + PI, 2.0f * PI) + Motor_Steer[i].Get_Now_Angle();
//            Target_Wheel_Omega[i] *= -1.0f;
//        }
//    }
//}
/*******************************************************/

	void chassis_receive_from_gimbal_1(uint8_t data[8])
{

		float speed_x = bytes_to_float(&data[0]);  
    float speed_y = bytes_to_float(&data[4]);  
		chassis.speed_RC.x = -speed_x;
		chassis.speed_RC.y = speed_y;
}
void chassis_receive_from_gimbal_2(uint8_t data[8])
{

		float speed_w = bytes_to_float(&data[0]);  
		float speed_yaw = bytes_to_float(&data[4]);  
		chassis.speed_RC.r = speed_w;
		chassis.speed_RC.big_yaw = speed_yaw*2*PI/360.f;		//单位 rad
}

// 限制值
inline void val_limit(float *val, float MAX)
{
	if (fabs(*val) > MAX)
	{
		if (*val > 0)
			*val = MAX;
		else
			*val = -MAX;
	}
}
// 限制变化量
inline void change_limit(float last, float *now, float limit)
{
	float change = *now - last;
	if (fabs(change) > limit)
	{
		if (change > 0)
			*now = last + limit;
		else
			*now = last - limit;
	}
}

//妙妙小工具
float Angle_Limit (float angle ,float max)
{
		if(angle > max)
			angle -= max;
		if(angle < 0)
			angle += max; 
		return angle;
}
// 将字节数组转换为float
float bytes_to_float(uint8_t *bytes) 
	{
    uint32_t temp = 0;
    temp |= (bytes[0] << 0);  // 最低字节
    temp |= (bytes[1] << 8);  // 第二个字节
    temp |= (bytes[2] << 16); // 第三个字节
    temp |= (bytes[3] << 24); // 最高字节
    return *(float *)&temp;   // 将uint32_t指针强制转换为float指针
	}
  
 float math_modulus_normalization(float x, float modulus)
{
    float tmp;

    tmp = fmod(x + modulus / 2.0f, modulus);

    if (tmp < 0.0f)
    {
        tmp += modulus;
    }

    return (tmp - modulus / 2.0f);
}
/*****************************************************/
	//电机转向内侧，修正方向
	fp32 Find_min_angle(fp32 angle1,fp32 angle2)
	{
		fp32 err;
		err = angle1 - angle2;
		if(fabs(err)>4096)
		{
			err = 8192 - fabs(err) ;
		}
		return err;
	}
  
 

/******************************************************/