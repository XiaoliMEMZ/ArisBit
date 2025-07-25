#include "bsp.h"
#include "app.h"
#include "config.h"

#include "app_flash.h"
#include "app_oled.h"

#include "icm20948.h"


static uint8_t fac_us = 0;  //us��ʱ������
static uint16_t fac_ms = 0; //ms��ʱ������

uint8_t g_test_mode = MODE_STANDARD;
uint8_t g_imu_type = IMU_TYPE_ICM20948;


// ϵͳʱ��Ϊ72M��1/8Ϊ9M����ÿ������9000000�Σ�1s=1000ms=1000000us =�� 1us��9��
void SysTick_Init(void)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);// ѡ���ⲿʱ��  HCLK/8
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;   	//����SYSTICK�ж�
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;   	//����SYSTICK
}

/**********************************************************
** ������: delay_init	��ʼ���ӳٺ���
** ��������: ��ʼ���ӳٺ���,SYSTICK��ʱ�ӹ̶�ΪHCLKʱ�ӵ�1/8
** �������??: SYSCLK����λMHz)
** �������??: ��
** ���÷��������ϵͳʱ�ӱ����?72MHz,�����delay_init(72)
** ϵͳʱ��Ϊ72M��1/8Ϊ9M����ÿ������9000000�Σ�1s=1000ms=1000000us =�� 1us��9��
***********************************************************/
void delay_init(void)
{
	uint8_t SYSCLK = 72;
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); // ѡ���ⲿʱ��  HCLK/8
	fac_us = SYSCLK / 8;
	fac_ms = (uint16_t)fac_us * 1000;
}

/**********************************************************
** ������: delay_ms
** ��������: ��ʱnms
** �������??: nms=[0, 1500]
** �������??: ��
** ˵����SysTick->LOADΪ24λ�Ĵ���,����,�����ʱ�?:
		nms<=0xffffff*8*1000/SYSCLK
		SYSCLK��λΪHz,nms��λΪms
		��72M������,nms<=1864 
***********************************************************/
void delay_ms(uint16_t nms)
{
	uint32_t temp;
	if (nms > 1500) nms = 1500;
	SysTick->LOAD = (uint32_t)nms * fac_ms; //ʱ�����??(SysTick->LOADΪ24bit)
	SysTick->VAL = 0x00;			   //��ռ�����??
	SysTick->CTRL = 0x01;			   //��ʼ����
	do
	{
		temp = SysTick->CTRL;
	} while (temp & 0x01 && !(temp & (1 << 16))); //�ȴ�ʱ�䵽��
	SysTick->CTRL = 0x00;						  //�رռ�����
	SysTick->VAL = 0X00;						  //��ռ�����??
}

/**********************************************************
** ������: delay_us
** ��������: ��ʱnus��nusΪҪ��ʱ��us��.
** �������??: nus
** �������??: ��
***********************************************************/
void delay_us(uint32_t nus)
{
	uint32_t temp;
	SysTick->LOAD = nus * fac_us; //ʱ�����??
	SysTick->VAL = 0x00;		  //��ռ�����??
	SysTick->CTRL = 0x01;		  //��ʼ����
	do
	{
		temp = SysTick->CTRL;
	} while (temp & 0x01 && !(temp & (1 << 16))); //�ȴ�ʱ�䵽��
	SysTick->CTRL = 0x00;						  //�رռ�����
	SysTick->VAL = 0X00;						  //��ռ�����??
}


void LED_GPIO_Init(void)
{
	/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
	GPIO_InitTypeDef GPIO_InitStructure;
	/*��������ʱ��*/
	RCC_APB2PeriphClockCmd(LED_GPIO_CLK, ENABLE);
	/*ѡ��Ҫ���Ƶ�����*/
	GPIO_InitStructure.GPIO_Pin = LED_GPIO_PIN;
	/*��������ģʽΪͨ���������??*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	/*������������Ϊ50MHz */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	/*���ÿ⺯������ʼ��PORT*/
	GPIO_Init(LED_GPIO_PORT, &GPIO_InitStructure);

	LED_ON();
}

void LED_SW_GPIO_Init(void)
{
	/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
	GPIO_InitTypeDef GPIO_InitStructure;
	/*��������ʱ��*/
	RCC_APB2PeriphClockCmd(LED_SW_GPIO_CLK, ENABLE);
	/*ѡ��Ҫ���Ƶ�����*/
	GPIO_InitStructure.GPIO_Pin = LED_SW_GPIO_PIN;
	/*��������ģʽΪͨ���������??*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	/*������������Ϊ50MHz */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	/*���ÿ⺯������ʼ��PORT*/
	GPIO_Init(LED_SW_GPIO_PORT, &GPIO_InitStructure);

	LED_SW_ON();
}

//JTAGģʽ����,��������JTAG��ģʽ
//mode:jtag,swdģʽ����;00,ȫʹ��;01,ʹ��SWD;10,ȫ�ر�;
//#define JTAG_SWD_DISABLE   0X02
//#define SWD_ENABLE         0X01
//#define JTAG_SWD_ENABLE    0X00
void Bsp_JTAG_Set(uint8_t mode)
{
	uint32_t temp;
	temp = mode;
	temp <<= 25;
	RCC->APB2ENR |= 1 << 0;	  //��������ʱ��
	AFIO->MAPR &= 0XF8FFFFFF; //���MAPR��[26:24]
	AFIO->MAPR |= temp;		  //����jtagģʽ
}

// ���͵�ǰ�汾�ŵ�������
void Bsp_Send_Version(void)
{
	#define LEN       7
	uint8_t data[LEN] = {0};
	uint8_t checknum = 0;
	data[0] = PTO_HEAD;
	data[1] = PTO_DEVICE_ID - 1;
	data[2] = LEN - 2;           // ����
	data[3] = FUNC_VERSION;   // ������
	data[4] = VERSION_MAJOR;      // ��汾��??, ��ɽ����1.2
	data[5] = VERSION_MINOR;      // С�汾��

	for (uint8_t i = 2; i < LEN - 1; i++)
	{
		checknum += data[i];
	}
	data[LEN - 1] = checknum;
	USART1_Send_ArrayU8(data, LEN);
}

// LEDָʾ��Ƭ���ײ㣬ÿ100�������һ��??,Ч����LEDÿ3����2�Ρ�
void Bsp_Led_Show_State(void)
{
	static uint8_t led_flash = 0;
	led_flash++;
	if (led_flash >= 30) led_flash = 0;

	if (led_flash >= 1 && led_flash < 4)
	{
		LED_ON();
	}
	else if (led_flash >= 4 && led_flash < 7)
	{
		LED_OFF();
	}
	else if (led_flash >= 7 && led_flash < 10)
	{
		LED_ON();
	}
	else if (led_flash >= 10 && led_flash < 13)
	{
		LED_OFF();
	}
}

// ��Ƭ��ָʾ����ʾ�͵���״̬��LED��˸�ͷ�����BB
void Bsp_Led_Show_Low_Battery(uint8_t enable_beep)
{
	static uint8_t led_flash_1 = 0;
	if (led_flash_1)
	{
		if (enable_beep) BEEP_ON();
		LED_ON();
		LED_SW_ON();
		led_flash_1 = 0;
	}
	else
	{
		BEEP_OFF();
		LED_OFF();
		LED_SW_OFF();
		led_flash_1 = 1;
	}
}

// ��Ƭ��ָʾ����ʾ��ѹ����״̬��LED��˸�ͷ�����������B~B~
void Bsp_Led_Show_Overvoltage_Battery(uint8_t enable_beep)
{
	static uint8_t time_count = 0;
	static uint8_t state = 0;
	time_count++;
	if (time_count > 5)
	{
		if (state == 0)
		{
			if (enable_beep) BEEP_ON();
			LED_ON();
			LED_SW_ON();
			state = 1;
		}
		else
		{
			BEEP_OFF();
			LED_OFF();
			LED_SW_OFF();
			state = 0;
		}
		time_count = 0;
	}
}

// ���ù�������ģʽ
void Bsp_Set_TestMode(uint16_t mode)
{
	g_test_mode = mode & 0xFF;
	if (g_test_mode > 2) g_test_mode = 0;
}

// ��ȡ��������ģʽ״̬
uint8_t Bsp_Get_TestMode(void)
{
	return g_test_mode;
}

// ��ȡIMU�ͺţ�0=icm20948,1=mpu9250,0xFF=��
uint8_t Bsp_Get_Imu_Type(void)
{
	return g_imu_type;
}

// ����IMU����Ϊ��
void Bsp_Imu_Type_None(void)
{
	g_imu_type = IMU_TYPE_MAX;
}

// IMU��ʼ��ʧ��ʱ������������һ������ʾ�û�
void Bsp_Long_Beep_Alarm(void)
{
	BEEP_ON();
	delay_ms(1000);
	BEEP_OFF();
}

// ɨ���Ƿ���MPU9250�豸������IMU����ֵ��
uint8_t Bsp_MPU_Scanf(void)
{
	uint8_t res;
	MPU_IIC_Init();
	MPU_ADDR_CTRL();
	delay_ms(10);
	for (int i = 0; i < 3; i++)
	{
		MPU_Write_Byte(MPU9250_ADDR, MPU_PWR_MGMT1_REG, 0X80);
		delay_ms(100);
		MPU_Write_Byte(MPU9250_ADDR, MPU_PWR_MGMT1_REG, 0X00);
		res = MPU_Read_Byte(MPU9250_ADDR, MPU_DEVICE_ID_REG);
		DEBUG("MPU Scanf ID=0x%02X\n", res);
		if (res == MPU9250_ID)
		{
			return IMU_TYPE_MPU9250;
		}
		delay_ms(10);
	}
	return IMU_TYPE_ICM20948;
}

static void Bsp_imu_init(void)
{
	#if ENABLE_MPU9250 && ENABLE_ICM20948
	g_imu_type = Bsp_MPU_Scanf();
	#elif ENABLE_MPU9250
	g_imu_type = IMU_TYPE_MPU9250;
	#elif ENABLE_ICM20948
	g_imu_type = IMU_TYPE_ICM20948;
	#else
	g_imu_type = IMU_TYPE_MAX;
	#endif

	printf("IMU Type:%d\n", g_imu_type);

	if(Bsp_Get_Imu_Type() == IMU_TYPE_MPU9250)
	{
		DEBUG("Start MPU9250 Init\r\n");
		if (MPU9250_Init())
		{
			printf("MPU_INIT ERROR\n");
			OLED_Show_Error();
			#if ENABLE_IMU_ERROR_PASS
			Bsp_Imu_Type_None();
			#else
			Bsp_Long_Beep_Alarm();
			while(1);
			#endif
		}
	}

	if (Bsp_Get_Imu_Type() == IMU_TYPE_ICM20948)
	{
		SPI2_Init();
	}
}


void Bsp_Init(void)
{
	delay_init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����Ϊ���ȼ���2

	USART1_Init(USART1_BAUDRATE);
	#if !APP_RELEASE
	printf("-----Develop-----\n");
	#endif
	printf("\nFirmware Version: V%d.%d.%d\n", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
	printf("Firmware Compiled: %s, %s\r\n\n", __DATE__, __TIME__);

	
	#if ENABLE_USART2
	USART2_Init(USART2_BAUDRATE);
	#endif

	#if ENABLE_USART3
	USART3_Init(USART3_BAUDRATE);
	#endif

	#if ENABLE_UART4
	UART4_Init(UART4_BAUDRATE);
	#endif

	Bsp_JTAG_Set(SWD_ENABLE);
	LED_GPIO_Init();
	LED_SW_GPIO_Init();
	Beep_GPIO_Init();
	Key_GPIO_Init();
	Adc_Init();

	delay_ms(50);
	BEEP_OFF();

#if ENABLE_OLED
	IIC_Init();
	i2c_scanf_addr();
	SSD1306_Init();
	OLED_Show_Waiting();
#endif

	Bsp_imu_init();

	RGB_Init();

	MOTOR_GPIO_Init();
	Motor_PWM_Init(MOTOR_MAX_PULSE, MOTOR_FREQ_DIVIDE);

	Encoder_Init();
	CAN_Config_Init(CAN_BAUD_1000Kbps);

	TIM7_Init();
	// PwmServo_Init();

	IOInit();

	

#if ENABLE_IWDG
	IWDG_Init();
#endif
}

// ������Ƭ��
void Bsp_Reset_MCU(void)
{
	printf("\r\nReset MCU\r\n");
	__set_FAULTMASK(1);
    NVIC_SystemReset();
}
