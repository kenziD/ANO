#include "config.h"
void Delay_ms_led(u16 nms)
{	
	uint16_t i,j;
	for(i=0;i<nms;i++)
		for(j=0;j<8500;j++);
} 

void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_Structure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	//JTAG pin(PA13\PA14\PA15\PB3\PB4) is defalut function pin when reset the cpu.
	//If you want to use it as normal GPIO pin(LED2 here[PB3]),you need to open AFIO clock.
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	//Because all JTAG pin is functioning at first.if you want to just use SWD.(SWD and JTAP function pin are repeat in two pin)
  //1.disable all JTAG pin.
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	//2.open SWD pin.(PA13 -->JTMS/SWDIO,PA14 --> JTCK/SWCLK)
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
	
	GPIO_Structure.GPIO_Pin =  GPIO_Pin_3;	//Ö¸Ê¾µÆ
	GPIO_Structure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Structure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(GPIOB, &GPIO_Structure);
	
	GPIO_Structure.GPIO_Pin =  GPIO_Pin_13;	//Ò¹¼äµÆ
	GPIO_Structure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Structure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(GPIOC, &GPIO_Structure);
}

void LED2_Flash(u8 times, u32 time)
{
    while(times--)
    {
        LED2_ON;
        Delay_ms_led(time);
        LED2_OFF;
        Delay_ms_led(time);
    }
}
void LED3_Flash(u8 times, u32 time)
{
    while(times--)
    {
        
				LED3_ON;
        Delay_ms_led(time);
        
				LED3_OFF;
        Delay_ms_led(time);
    }
}
