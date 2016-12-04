SET PATH=C:\Keil_v5\ARM\ARMCC\Bin;C:\ProgramData\Oracle\Java\javapath;E:\Python27\;E:\Python27\Scripts;C:\Program Files\AMD APP\bin\x86;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Program Files\ATI Technologies\ATI.ACE\Core-Static;e:\Program Files\MiKTeX 2.9\miktex\bin\;E:\Program Files\SumatraPDF;E:\Program Files\nodejs\;C:\Program Files\SourceGear\Common\DiffMerge\;E:\Program Files\phantomjs-2.0.0-windows\bin\;E:\Program Files\cmder\;d:\Program Files\CMake\bin;E:\HashiCorp\Vagrant\bin;C:\Program Files\MongoDB\Server\3.2\bin;E:\Program Files\Anaconda2;E:\Program Files\Anaconda2\Scripts;E:\Program Files\Anaconda2\Library\bin;e:\Program Files\MATLAB\R2013a\runtime\win32;e:\Program Files\MATLAB\R2013a\bin;e:\Program Files\ATMEL Corporation\AT91-ISP v1.13\Library\;e:\Program Files\ATMEL Corporation\AT91-ISP v1.13\sam-ba 2.9;C:\Users\kenziD\AppData\Roaming\npm
SET ARMCC5_ASMOPT=--diag_suppress=9931
SET ARMCC5_CCOPT=--diag_suppress=9931
SET ARMCC5_LINKOPT=--diag_suppress=9931
SET CPU_TYPE=STM32F103C8
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=STM32_Project
SET CPU_CLOCK=0x00F42400
"C:\Keil_v5\ARM\ARMCC\Bin\ArmAsm" --Via "..\output\startup_stm32f10x_md._ia"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\main.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f10x_it.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\misc.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f10x_gpio.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f10x_rcc.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f10x_usart.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f10x_i2c.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f10x_tim.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f10x_spi.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f10x_adc.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f10x_dma.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\core_cm3.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\system_stm32f10x.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\config.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\delay.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\hseconf.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\led.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\sys.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\usart.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\iic.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\imu.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\mpu6050.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\motor.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\24l01.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\spi.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\wave.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\pid.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\timer.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\rc.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\adc.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmLink" --Via "..\Output\Project.lnp"
"C:\Keil_v5\ARM\ARMCC\Bin\fromelf.exe" "..\Output\Project.axf" --i32combined --output "..\Output\Project.hex"
