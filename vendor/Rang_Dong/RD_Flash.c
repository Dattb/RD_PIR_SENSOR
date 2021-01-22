/*
 * RD_Flash.c
 *
 *  Created on: Dec 2, 2020
 *      Author: dangv
 */

#include "RD_Flash.h"


Button_Sence   button_sence_data;
unsigned char  Buff_Flash_Read[FLASH_BUFF_LEN];
unsigned char  Buff_Flash_Write[FLASH_BUFF_LEN] = {
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

};

/*
 *  Check da ghi Flash lan nao chua. neu chua thi ghi 0x00 len 64 byte dau tien
 * 	Doc 64 byte Flash tu 0x210000 vao Buff_Flash de su dung khi can.
 */
void RD_Flash_Init()
{
	flash_read_page(FLASH_ADDR, FLASH_BUFF_LEN, Buff_Flash_Read);
	if(Buff_Flash_Read[63] == 0xFF)
	{
		RD_Flash_CleanSenceFlash();
	}
	else
	{
		char UART_TempSend[128];
		sprintf(UART_TempSend,"Flash data 0x210000:0x%x 0x%x \n",Buff_Flash_Read[0],Buff_Flash_Read[1] );
		uart_CSend(UART_TempSend);
//		for(int i=0; i<36;i=i+2)
//		{
//			sprintf(UART_TempSend," 0x%x 0x%x \n",Buff_Flash_Read[i+1],Buff_Flash_Read[i] );
//			uart_CSend(UART_TempSend);
//		}
	}
}

void RD_Flash_CleanSenceFlash()
{
	uint8_t Buff_Flash_Null[FLASH_BUFF_LEN]={0};
	uart_CSend("Clean Sence Flash");
   	flash_erase_sector(FLASH_ADDR);
	flash_write_page(FLASH_ADDR, FLASH_BUFF_LEN,Buff_Flash_Null );
	flash_read_page(FLASH_ADDR, FLASH_BUFF_LEN, Buff_Flash_Read);
}
//flash_read_page(FLASH_ADDR,FLASH_BUFF_LEN,(unsigned char *)Flash_Read_Buff);

