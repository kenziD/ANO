#include "config.h"
#include "stdint.h"

int main(void)
{
	SysTick_Init();

	LED_Init();

	while(1)
  {
		LED1_ON;//green led flash
		delay_ms(1000);
		LED1_OFF;
		delay_ms(1000);
		LED2_ON;//red led flash
		delay_ms(100);
		LED2_OFF;
		delay_ms(100);
  }
}
