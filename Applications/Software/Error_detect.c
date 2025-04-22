/**
 * @file Error_detect.c
 * @author sethome
 * @brief ´íÎó¼ì²é
 * @version 0.1
 * @date 2022-11-19
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "global_status.h" 
#include "Error_detect.h"
#include "CAN_receive&send.h"
#include "chassis_move.h"
#include "gimbal.h"
#include "LED_control.h"
#include "Stm32_time.h"


struct Error_detect_t Error_detect;

void Error_detect_init(void)
{
	
}

void Error_detect_flush(void)
{
    // Ò£¿ØÆ÷¼ì²â
    static uint32_t last_remote = 0;
    if (last_remote == Error_detect.remote.last_time)
    {
        Global.err[REMOTE_ERR] = 1;
        Error_detect.remote.flag = 1;
    }
    else
    {
        Global.err[REMOTE_ERR] = 0;
        Error_detect.remote.flag = 0;
    }
    last_remote = Error_detect.remote.last_time;

    // µç»ú¼ì²â
    Error_detect_motor(chassis_move_FR);
    Error_detect_motor(chassis_move_FL);
    Error_detect_motor(chassis_move_BL);
    Error_detect_motor(chassis_move_BR);
    if (Error_detect.motor.flag[chassis_turn_FR] ||
        Error_detect.motor.flag[chassis_turn_FL] ||
        Error_detect.motor.flag[chassis_turn_BL] ||
        Error_detect.motor.flag[chassis_turn_BR])
        Global.err[CHASSIS_ERR] = 1;
    else
        Global.err[CHASSIS_ERR] = 0;

//    Error_detect_motor(YAW_MOTOR);
//    Error_detect_motor(PITCH_MOTOR);
//    if (Error_detect.motor.flag[YAW_MOTOR] ||
//        Error_detect.motor.flag[PITCH_MOTOR])
//        Global.err[GIMBAL_ERR] = 1;
//    else
//        Global.err[GIMBAL_ERR] = 0;

#ifdef USE_3508_AS_SHOOT_MOTOR
    Error_detect_motor(SHOOT_MOTOR1);
    Error_detect_motor(SHOOT_MOTOR2);
#endif

//    Error_detect_motor(TRIGGER_MOTOR);
//    if (Error_detect.motor.flag[TRIGGER_MOTOR] ||
//        Error_detect.motor.flag[SHOOT_MOTOR1] ||
//        Error_detect.motor.flag[SHOOT_MOTOR2])
//        Global.err[SHOOT_ERR] = 1;
//    else
//        Global.err[SHOOT_ERR] = 0;
}

/**
 * @brief µç»ú´íÎó¼ì²â£¬ÔÚFreeRTOSÖÐµ÷ÓÃ
 *
 * @param ID
 */
void Error_detect_motor(can_id ID)
{
    uint16_t tmp1 = get_motor_data(ID).given_current;
    uint16_t tmp2 = get_motor_data(ID).ecd;
    if (Error_detect.motor.last_given_current[ID] == tmp1 &&
        Error_detect.motor.last_ecd[ID] == tmp2)
        Error_detect.motor.err_cnt[ID]++;
    else
    {
        Error_detect.motor.err_cnt[ID] = 0;
        Error_detect.motor.flag[ID] = 0;
    }

    Error_detect.motor.last_given_current[ID] = tmp1;
    Error_detect.motor.last_ecd[ID] = tmp2;

    if (Error_detect.motor.err_cnt[ID] > 3)
    {
        Error_detect.motor.err_cnt[ID] = 0;
        Error_detect.motor.flag[ID] = 1;
    }
}

void Error_detect_remote(void)
{
    Error_detect.remote.last_time++;
}

