#include "config.h"
#include "stdint.h"

int main(void)
{
	int key = 0;
	SysTick_Init();

	LED_Init();

	USART1_Config(9600);

	KEY_Init();
		
	while(1)
	{
		key = KEY_scan();

		if(key>0)
		 {
			 if(key==MODE_KEY_DOWN)
			 {
				 printf("MODE_KEY\n");
				 key=0;
			 }
			 else if (key==FUN_KEY_DOWN)
			 {
				 printf("FUN_KEY\n");
				 key=0;
			 }
			
			}
	}
}
