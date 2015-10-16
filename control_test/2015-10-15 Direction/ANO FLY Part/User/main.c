#include "config.h"

extern unsigned char ID;
int count = 0;
extern int16_t motor0, motor1, motor2, motor3;
extern float surRoll,surPitch;
extern int expThro;
extern PID_ PID_ROLL,PID_PITCH;
extern float expRoll;
extern float expPitch;
int main(void)
{
	float q[4];
	float ypr[3]; // yaw pitch roll
	u8 tmp_buf[32] = "";
	float Receive_Data = 0;
	int throttle = 0;//油门 999为满油门
	char status[5] = "start";
	float RC_get_Roll = 0;
	float RC_get_Pitch = 0;
	RCC_HSE_Configuration();
	SysTick_Init();
	USART1_Config(115200);
	LED_Init();
	IIC_Init();
	Initial_Timer3();
	TIM2_Init(999,0);
	Mpu6050_Init_offset();
	Mpu6050init();
	MOT_GPIO_init();
	MOT_PWM_init();
	Set_PWM(0,0,0,0);
	PID_Init();
	NRF24L01_Init();    	//³õÊ¼»¯NRF24L01

	while (NRF24L01_Check())	//¼ì²éNRF24L01ÊÇ·ñÔÚÎ».
	{
		printf("no");
	}

	NRF24L01_RX_Mode();

	while (1)
	{
		Read_Mpu6050();
		IMU_getQ(q);
		IMU_getYawPitchRoll(ypr);
		if (NRF24L01_RxPacket(tmp_buf) == 0)
		{
			//Receive_Data = (float)(tmp_buf[1] << 8 | tmp_buf[0]) / 1000.0;
			//throttle = (int)(Receive_Data * 999.0);
			//printf("a:%d\t\n", throttle);
			RC_get_Roll =((tmp_buf[3] << 8 | tmp_buf[2])-1970)/70.0;//回中值是1970  如果设置成遥杆最左和最右都为30°，则量程为70
			RC_get_Pitch= ((tmp_buf[5] << 8 | tmp_buf[4])-2120)/70.0;;
			//printf("Rd_get:%f",RC_get_Roll);
			printf("Rd_get:%d\r\n",RC_get_Pitch);
		}
		expRoll = RC_get_Roll;
		expPitch = RC_get_Pitch;		
		surRoll = ypr[2];
		surPitch = ypr[1];
		//printf("%f", surPitch);
		  PID_Set();
			Set_PWM(motor0,motor1,motor2,motor3);
	}
}
