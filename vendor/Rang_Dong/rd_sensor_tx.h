/*
 * rd_sensor_tx.h
 *
 *  Created on: Sep 5, 2020
 *      Author: Smart5
 */

#include "proj/tl_common.h"
#include "vendor/mesh/app.h"
#include "vendor/mesh_lpn/app.h"
#include "vendor/mesh_provision/app.h"
#include "vendor/mesh_switch/app.h"
#include "vendor/common/sensors_model.h"
#include "proj_lib/mesh_crypto/sha256_telink.h"
#include "vendor/common/app_heartbeat.h"





#ifndef RD_SENSOR_TX_H_
#define RD_SENSOR_TX_H_


#define     BUFF_DATA_LEN    2
typedef struct{
	u16  reset_cnt;
	u16  check;
	u16  button_press;
} FactoryResetDetect;

typedef enum {
	ON  =0,
	OF  =1,
}LedDebugStatus;







#define     TX_BUFF_DATA_LEN                    2
#define     RX_BUFF_DATA_LEN                    2
#define     GW_TX_BUFF_DATA_LEN                 8
#define     SLAVE_DEVICE_ADDR                   0x88
#define     SLAVE_DEVICE_ADDR_LEN               1
#define     I2C_CLK_SPEED                       200000

//volatile unsigned char i2c_tx_buff[TX_BUFF_DATA_LEN] = {0};


#define OPT3001_RESULT_REGISTER                 0x00
#define OPT3001_RESULT_REGISTER_LEN             1
#define OPT3001_CONFIG_REGISTER                 0x01
#define OPT3001_CONFIG_REGISTER_LEN             1
#define RSP_MAX                                 2
#define GATEWAY_ADDR                            0x0001
#define OPT3001_CONFIG_REG_HIGH                 0xC4   // xem trong datasheet phan configure register
//#define OPT3001_CONFIG_REG_HIGH                 0xC2   // xem trong datasheet phan configure register
#define OPT3001_CONFIG_REG_LOW                  0x10   // xem trong datasheet phan configure register
//#define OPT3001_CONFIG_REG_LOW                  0x14   // xem trong datasheet phan configure register


#define SENSOR_POWER_TYPE                      0x0001
#define REMOTE_DC_MODULE_TYPE                  0x0002
#define REMOTE_AC_MODULE_TYPE                  0x0003
#define LIGHT_SENSOR_MODULE_TYPE               0x0004
#define PIR_SENSOR_MODULE_TYPE                 0x0005







void RD_Poll_SenData(void);
void RD_I2C_SenData(void);
void RD_PirReadHightLow();
void RD_PirWakeup ();
u32 RD_Check_GpioWakeup();
void RD_SetGpio(u32 gpio,bool mode,GPIO_PullTypeDef pull_mode);
void RD_led_to_debug (LedDebugStatus gpio_level);



void RD_light_sensor_tx (u16 loop);
void RD_Led_init(void);
unsigned int RD_power_read();
void RD_ADC_init (u32 gpio);
void  Led_toggle(void);
unsigned int CalculateLux(unsigned int rsp_lux);
void delay_init(u16 time);
void time_poll_change (u8*par,mesh_cb_fun_par_t *cb_par);
void RD_Send_LightSensorPower();
void RD_Send_Lux();
void RD_Send_Pir_motion();
void RD_Facory_RESET(unsigned char cnt);
#endif /* RD_SENSOR_TX_H_ */
