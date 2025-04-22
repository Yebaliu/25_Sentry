/**
 * @file USB_VirCom.c
 * @author sethome
 * @brief 虚拟串口数据发送
 * @version 0.1
 * @date 2022-11-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "usbd_cdc_if.h"
#include "USB_VirCom.h"
#include "Global_status.h"
#include "crc8_crc16.h"
#include "Stm32_time.h"

bool if_vision_start =0;
void VirCom_send(uint8_t data[], uint16_t len)
{
  CDC_Transmit_FS(data, len);
}


void VirCom_rev(uint8_t data[], uint16_t len)
{
	uint16_t sum;
  if (data[0] == (unsigned)'s')
  {		
	  if_vision_start =1;//主要是为了开机时刻对其施加戳
//		if(Verify_CRC16_Check_Sum(data,sizeof(NUC_data_t)))
////		decodeNUC(&fromNUC,data,len);
  }
}


