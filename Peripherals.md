
PINS:
- green led: PA5
- user blue button: PC13 (GPIO_EXTI13)
- microphone: PA8 *(Connected as ON/OFF)*
- ADC:
  - potentiometer: PA1
  - light sensor: PA0

- speaker: PA9 (TIM1_CH2)

- encoder: PC7 (TIM3_CH2), PC6 (TIM3_CH1)
- keyboard: PC8:11 OUTPUT (column driver), PC12,13,2,3 INPUT (row readout)
  No hardware debouncing
- LCD:
  - PB12:15	(GPIO_Output)
  - PB1:2 (GPIO_Output)
  - PA4 (GPIO_Output)
  PMDB16_LCD.h and PMDB16_LCD.c have to be included

- USART2:
	- rx: PA3
	- tx: PA2

- I2C: PB9->SDA (Serial Data), PB8->SCL (Serial Clock)
	- thermometer addr: 0b1001000<<1 
	- accelerometer addr: 0b0101000<<1 

- SPI: (1,4 up to 42Mbps; 2,3 up to 21Mbps)
	- led matrix (daisy-chain configuration, MSB first): PA5 -> SCK (Serial Clock), PA6(MISO), PA7(MOSI), PB6 (RCLK of SNx4HC595 (shift register): enables parallel output on rising edge)
- IR: PB10 (Led attached to TIM2_CH3) and PA10 (receiver USART1RX)

- RGB: PWM R=TIM3_CH1, G=TIM3_CH2, B=TIM3_CH3

