/*
 * RD_Flash.h
 *
 *  Created on: Dec 2, 2020
 *      Author: dangv
 */

#ifndef RD_FLASH_H_
#define RD_FLASH_H_

#include "proj/common/types.h"
#include "proj/tl_common.h"
#include "proj/mcu/watchdog_i.h"
#include "vendor/common/user_config.h"
#include "proj_lib/rf_drv.h"
#include "proj_lib/pm.h"
#include "proj_lib/ble/blt_config.h"
#include "proj_lib/ble/ll/ll.h"
#include "proj_lib/sig_mesh/app_mesh.h"

//#include "../Button/Button.h"
//#include "../RD_Remote/RD_Remote.h"

#define FLASH_ADDR              0x22000//0x022000
#define FLASH_BUFF_LEN           64
extern unsigned char  Buff_Flash_Read[FLASH_BUFF_LEN];
extern unsigned char  Buff_Flash_Write[FLASH_BUFF_LEN] ;
typedef struct
{
	uint8_t BID;
	uint8_t MODE_ID;
	uint8_t SENCE_ID[2];
}Button_Sence;
extern  Button_Sence   button_sence_data;

uint16_t RD_Flash_Get_Sence_ID(uint8_t buttonID, uint8_t modeID );
void RD_Flash_SaveChangeSenceID2Flash(uint8_t buttonID,uint8_t modeID, uint8_t SenceID[2]);
void RD_Flash_Init();
void RD_Flash_CleanSenceFlash();
//volatile unsigned char Flash_Read_Buff[FLASH_BUFF_LEN]={0};

#endif /* RD_FLASH_H_ */
