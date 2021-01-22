/*
 * rd_sensor_tx.c
 *
 *  Created on: Sep 5, 2020
 *      Author: Smart5
 */

//#include "../../proj_lib/sig_mesh/app_mesh.h"
#include "rd_sensor_tx.h"
u8 kiem_tra_factory_reset_khi_ngoai_mang_o_day;
u8 factory_reset_check ;
u16 provision_time = 60000;
u16 friend_poll = 6000;
u16 LPN_WorKing_Timeout = 8000;
u8 Light_Control_Timeout=0;
bool lux_changle;

FactoryResetDetect reset_detect;


u8 check_done=0;
u8 check_poll=1;
u16 lux_val_old,lux_val_new;

u8 Light_sensor_to_gw_tx_buff[GW_TX_BUFF_DATA_LEN]={0};
u8 Pir_sensor_to_gw_tx_buff[GW_TX_BUFF_DATA_LEN]={0};
u8 power_to_gw_tx_buff[GW_TX_BUFF_DATA_LEN]={0};
u8 i2c_tx_buff[TX_BUFF_DATA_LEN] = {OPT3001_CONFIG_REG_HIGH,OPT3001_CONFIG_REG_LOW};
u8 i2c_rx_buff[RX_BUFF_DATA_LEN] = {0};

u16 LIGHT_SENSOR_TRANS_CNT = 12*60;    // 12*60*5000 = 3600000 ms = 1h


u8 dem_reset=0;
void RD_Facory_RESET(unsigned char cnt)
{
//	while(!gpio_read(SW1_GPIO));
	dem_reset++;
	if(cnt>=4){
		RD_led_to_debug(ON);
		dem_reset=0;
	}
}
void RD_Poll_SenData(void){

//	static u8 test_count = 0;
//	u8 SendDataCount[16];
//	sprintf(SendDataCount, "IS: %4d", test_count++);
//	mesh_tx_cmd2normal_primary(RD_OPCODE_RSP, SendDataCount, 8, 0x0001, 2);
#define     I2C_CLK_SPEED               200000
	unsigned char i2c_rx_buff[BUFF_DATA_LEN] = {0xC6, 0x01};
	i2c_gpio_set(I2C_GPIO_GROUP_C0C1);
	i2c_master_init(0x44<<1, (unsigned char)(CLOCK_SYS_CLOCK_HZ/(4*I2C_CLK_SPEED)));
	i2c_write_series(0x01, 1, (unsigned char *)(i2c_rx_buff), 2);
	i2c_read_series(0x00, 1, (unsigned char *)(i2c_rx_buff), 2);

	float LuxConv;
	uint16_t Mantissa;
	uint16_t Exponent;
	Mantissa = ((uint16_t)(i2c_rx_buff[0]&0x0f)<<8) | (uint16_t)i2c_rx_buff[1];
	Exponent = (i2c_rx_buff[0]>>4)&0x0F;
	LuxConv = (float)(1<<Exponent) * (float)(Mantissa/100.0);
	u8 SendDataCount[10];
	my_sprintf(SendDataCount, "%d", (unsigned int)LuxConv);

	//mesh_tx_cmd2normal_primary(RD_OPCODE_RSP, SendDataCount, 6, 0x0001, 2);

}

void RD_I2C_SenData(void){

	#define ON  1
	#define OFF 0
	#define GROUP_ADDR 0xC001
	#define LIGHT_ADDR 0x0090
	RD_led_to_debug(ON);
	unsigned char i2c_rx_buff[BUFF_DATA_LEN] = {0xC6, 0x01};
	i2c_gpio_set(I2C_GPIO_GROUP_C0C1);
	i2c_master_init(0x44<<1, (unsigned char)(CLOCK_SYS_CLOCK_HZ/(4*I2C_CLK_SPEED)));
	i2c_write_series(0x01, 1, (unsigned char *)(i2c_rx_buff), 2);
	i2c_read_series(0x00, 1, (unsigned char *)(i2c_rx_buff), 2);
	unsigned char data_tx[4] = {0x00, 0x03, i2c_rx_buff[0], i2c_rx_buff[1]};

	mesh_tx_cmd2normal_primary(SENSOR_STATUS, data_tx, 4, 0x0001, 2);
	Light_Control_Timeout++;
	u16 lux_data;
	lux_data = (i2c_rx_buff[0]<<8) | i2c_rx_buff[1];
}


void RD_PirReadHightLow()
{

	gpio_set_func(PIR_LOW, AS_GPIO);
	gpio_set_output_en(PIR_LOW,0);
	gpio_set_input_en(PIR_LOW, 1);
	gpio_setup_up_down_resistor(PIR_LOW,PM_PIN_UP_DOWN_FLOAT);

	if(!gpio_read(PIR_LOW)&&!gpio_read(PIR_HI))
	{
		RD_I2C_SenData();
	}

	while(gpio_read(PIR_HI));
	sleep_ms(1);
	u16 pir_cnt;
	while(!gpio_read(PIR_LOW))
	{
		pir_cnt++;
		if(pir_cnt>=60000) break;
	}
	if(gpio_read(PIR_LOW)&&!gpio_read(PIR_HI))
	{
		RD_I2C_SenData();

	}

}
void RD_PirWakeup ()
{
    gpio_set_wakeup (GPIO_PB4,1, 1);         // level : 1 (high); 0 (low)
    gpio_setup_up_down_resistor(GPIO_PB4, PM_PIN_PULLDOWN_100K);
    cpu_set_gpio_wakeup (GPIO_PB4,1, 1);     // level : 1 (high); 0 (low)

    //cpu_sleep_wakeup(DEEPSLEEP_MODE, PM_WAKEUP_PAD,100);
    //cpu_sleep_wakeup(DEEPSLEEP_MODE, PM_WAKEUP_TIMER, clock_time() + 3000*CLOCK_SYS_CLOCK_1MS);
    cpu_sleep_wakeup(DEEPSLEEP_MODE, PM_WAKEUP_PAD | PM_WAKEUP_TIMER, clock_time() + 10000*CLOCK_SYS_CLOCK_1MS);
}

u32 RD_Check_GpioWakeup()
{
	extern u8 key_not_release;
    #define WAKEUP_TRIGGER_LEVEL  1
	if(WAKEUP_TRIGGER_LEVEL == gpio_read (SW1_GPIO)){
		key_not_release = 1;
		return SW1_GPIO;
	}
	else if(WAKEUP_TRIGGER_LEVEL == gpio_read (PIR_HI)){
		key_not_release = 1;
		return PIR_HI;
	}
}

void RD_SetGpio(u32 gpio,bool mode,GPIO_PullTypeDef pull_mode)
{
	gpio_set_func(gpio, AS_GPIO);
	gpio_set_output_en(gpio,mode);
	gpio_set_input_en(gpio, !mode);
	gpio_setup_up_down_resistor(gpio,pull_mode);
}

void RD_led_to_debug (LedDebugStatus gpio_level)
{
	//while(1){
	gpio_set_func(GPIO_PB5, AS_GPIO);
	gpio_set_output_en(GPIO_PB5,1);
	gpio_set_input_en(GPIO_PB5, 0);
	gpio_write(GPIO_PB5,gpio_level);
	//}
}
void time_poll_change (u8 *par,mesh_cb_fun_par_t *cb_par)
{
	if(cb_par->op==0x3082)
	{
		unsigned int data;
		data = (par[0]<<8) | par[1];
		friend_poll = data;
//		sleep_us(2000); // delay de gui uart
	}
}
unsigned int CalculateLux(unsigned int rsp_lux)
{
	unsigned int lux_LSB = 0;
	unsigned char lux_MSB = 0;
	unsigned int lux_Value = 0;
	unsigned int pow = 1;
	unsigned char i;
	lux_LSB = rsp_lux & 0x0FFF;
	lux_MSB = ((rsp_lux>>12) & 0x0F);
	//Lux_Value = 0.01 * pow(2,Lux_MSB) * Lux_LSB; //don't use
	for(i=0;i<lux_MSB;i++){
		pow=pow*2;
	}
	lux_Value=0.01 * pow * lux_LSB;
	return lux_Value;
}


void RD_ADC_init (u32 gpio)
{
	adc_init();
	adc_base_init(gpio);
	adc_power_on_sar_adc(1);

}

unsigned int RD_power_read()
{
    RD_ADC_init (GPIO_PC4);
	unsigned int power_read;
	float power_persent=0;
    //sleep_us(2000);
	power_read = adc_sample_and_get_result();
	if(power_read<=1070)
	{
		power_persent=  power_read*0.0513-45.82;   // %pin = vol*0.0513-45.82
	}
	else if (power_read<=1174)
	{
		power_persent=  power_read*0.2449-254.62; //y = 0.2449x - 254.62
	}
	else if (power_read<=1404)
	{
		power_persent=  power_read*0.3084-333.94;          //y = 0.3084x - 333.94
	}
	else if(power_read<1480) {
		power_persent=  power_read*0.1273-89.77;  	//y = 0.1273x - 89.77
	}
	else power_persent=100;
	power_persent = (unsigned int) power_persent;
	return power_persent;
	//return power_read;
}

void RD_light_sensor_tx (u16 loop)
{

	if(!check_done)
	{
		*Light_sensor_to_gw_tx_buff = LIGHT_SENSOR_MODULE_TYPE;
		*(Light_sensor_to_gw_tx_buff+1) = (u8)(LIGHT_SENSOR_MODULE_TYPE << 8);

		*Pir_sensor_to_gw_tx_buff = PIR_SENSOR_MODULE_TYPE;
    	*(Pir_sensor_to_gw_tx_buff+1) =(u8)(PIR_SENSOR_MODULE_TYPE << 8);

		*power_to_gw_tx_buff = SENSOR_POWER_TYPE;
		*(power_to_gw_tx_buff+1) = (u8)(SENSOR_POWER_TYPE << 8);
	    check_done=1;
	}
	i2c_gpio_set(I2C_GPIO_GROUP_C0C1);
	i2c_master_init(SLAVE_DEVICE_ADDR,(unsigned char)(CLOCK_SYS_CLOCK_HZ/(4*I2C_CLK_SPEED)));
	i2c_write_series(OPT3001_CONFIG_REGISTER,OPT3001_CONFIG_REGISTER_LEN,(u8 *)i2c_tx_buff, TX_BUFF_DATA_LEN);
	i2c_read_series(OPT3001_RESULT_REGISTER,OPT3001_RESULT_REGISTER_LEN, (u8 *)i2c_rx_buff, RX_BUFF_DATA_LEN);

	lux_val_new = (i2c_rx_buff[0]<<8) | i2c_rx_buff[1];
	if(lux_val_new!=lux_val_old)
	{
		if(CalculateLux(lux_val_new)>=CalculateLux(lux_val_old))
		{
			if(CalculateLux(lux_val_new)-CalculateLux(lux_val_old)>=100)
			{
				lux_val_old=lux_val_new;
				lux_changle=TRUE;
			}
		}
		else if(CalculateLux(lux_val_new)<=CalculateLux(lux_val_old))
		{
			if(CalculateLux(lux_val_old)-CalculateLux(lux_val_new)>=100)
			{
				lux_val_old=lux_val_new;
				lux_changle=TRUE;
			}
		}
	}
	if(check_poll>=loop||!gpio_read(GPIO_PD4)||lux_changle==TRUE)
	{
		RD_Send_Lux();
		if(check_poll>=loop||!gpio_read(GPIO_PD4)) RD_Send_LightSensorPower();
		check_poll=0;
		lux_changle=FALSE;
		/*
		 * TODO: truong data khi gui di duoc du nguyen ma khong bi dao bit
		 */
	}
	if(gpio_read(GPIO_PB4))RD_Send_Pir_motion();
	 check_poll++;
}

void RD_Send_Pir_motion()
{
	//adc_set_ref_voltage()
	*(Pir_sensor_to_gw_tx_buff+2) = * i2c_rx_buff;
	*(Pir_sensor_to_gw_tx_buff+3) = *(i2c_rx_buff+1);
	mesh_tx_cmd2normal_primary(SENSOR_STATUS, (u8 *)Pir_sensor_to_gw_tx_buff, GW_TX_BUFF_DATA_LEN, GATEWAY_ADDR, RSP_MAX);
}

void RD_Send_Lux()
{
	//adc_set_ref_voltage()
	*(Light_sensor_to_gw_tx_buff+2) = * i2c_rx_buff;
	*(Light_sensor_to_gw_tx_buff+3) = *(i2c_rx_buff+1);
	mesh_tx_cmd2normal_primary(SENSOR_STATUS, (u8 *)Light_sensor_to_gw_tx_buff, GW_TX_BUFF_DATA_LEN, GATEWAY_ADDR, RSP_MAX);
}

void RD_Send_LightSensorPower()
{
	u16 Power_Data=RD_power_read();
	*(power_to_gw_tx_buff+2) = (u8) (Power_Data>>8);
	*(power_to_gw_tx_buff+3) = (u8)(Power_Data);
	mesh_tx_cmd2normal_primary(SENSOR_STATUS, (u8 *)power_to_gw_tx_buff, GW_TX_BUFF_DATA_LEN, GATEWAY_ADDR, RSP_MAX);
}
