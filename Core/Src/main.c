/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN */
void checkoffOne_Init(void);
void init_Gyro(void);
short init_xread(void);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	//Enable GPIOB and GPIOC
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
	//Initialize (Clear) the GPIOB/C registers
	GPIOB->OTYPER = 0;
	GPIOB->MODER = 0;
	
	//SDA Setup --------
	//Initialize PB11
	GPIOB->MODER |= (1 << 23); //Enable AF Mode
	GPIOB->OTYPER |= (1 << 11); //Open-drain
	
	//PB11 Alternate Function AF1 select: I2C2_SDA -> AF1 of PB11
	//Bits [15:12] correspond to PB11 in the register -> AF1 is 0001.
	GPIOB->AFR[1] |= (1 << 12); 
	
	//SCL Setup --------
	// Initialize PB13
	GPIOB->MODER |= (1 << 27); //Enable AF Mode
	GPIOB->OTYPER |= (1 << 13); //Open-drain
	
	//PB13 Alternate Function AF5 select: I2C2_SCL -> AF5 of PB13
	//Bits [23:20] correspond to PB13 in the register, -> AF5 is 0101.
	GPIOB->AFR[1] |= (5 << 20);
	
	//Initialize PC0 and PB14
	GPIOB->MODER |= (1 << 28); //General-Purpose Output
	GPIOC->MODER |= (1 << 0);
	GPIOC->OTYPER = 0; //Push-pull mode.
	GPIOB->ODR |= (1 << 14); //PB14 set High
	GPIOC->ODR |= (1 << 0);  //PC0 set High
	
	//I2C2 Peripheral RCC Enable (I2C)
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN; //Clock enable
	
	//Setting up the I2C2 Peripheral timing for SDA and SCL
	//Set to 100kHz operation
	I2C2->TIMINGR |= (1 << 28); //PSC = 1
	I2C2->TIMINGR |= (0x4 << 20); //[23:20] SCLDEL (Data Setup Time) NOTE: may need to add 1 to 0x4
	I2C2->TIMINGR |=  (0x2 << 16); //[19:16] SDADEL (Data Hold Time)
	I2C2->TIMINGR |= (0xF << 8); //[15:8] SCLH (SCL High Period)
	I2C2->TIMINGR |= (0x13 << 0); //[7:0] SCLL (SCL Low Period)
	
	//Enable the I2C2 Peripheral
	I2C2->CR1 |= (1 << 0); //Enable (PE Bit in CR1 Register)
	
	//My Key: 
	//--------//Positive Y-Axis = RED LED
	//--------//Negative Y-Axis = BLUE LED
	//--------//Positive X-Axis = GREEN LED
	//--------//Negative X-Axis = ORANGE LED
	
	//LED Setup
	GPIOC->MODER |= (1 << 12);
	GPIOC->MODER |= (1 << 14); 
	GPIOC->MODER |= (1 << 16);
	GPIOC->MODER |= (1 << 18);
	
	checkoffOne_Init();
	//init_Gyro();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//First Checkoff "who am i register"
void checkoffOne_Init(void) {
			//Transmit corresponding X-Axis address
		I2C2->CR2 = 0; //Clear to start.
		
		//Set the Gyroscope Slave Address in SADD: 0x69
		//Set the number of bytes to transmit = 1
		I2C2->CR2 |= (0x69 << 1); //Slave address is [7:1]
		I2C2->CR2 |= (1 << 16);
		
		//Set the RD_WRN bit to 'WRITE' operation.
		//Set the START bit
		I2C2->CR2 &= ~(1 << 10); //"WRITE" in RD_WRN is 0 in bit 10
		I2C2->CR2 |= (1 << 13);

		//Wait until either of the TXIS (Transmit Register Empty/Ready) 
		//or NACKF (Slave NotAcknowledge) flags are set.
		//----//If the NACKF flag is set, the slave did not respond to the address frame.
		//----//Continue if the TXIS flag is set
		while(((I2C2->ISR & 0x2) >> 1) != 1){
			if((I2C2->ISR & 0x10) >> 4){ //Check NACFK flag
				//LED Error Indicator Setup
				GPIOC->ODR |= (1 << 6); //Red LED
			}
		}
		
		//WHO_AM_I register ------- 0x0F
		I2C2->TXDR = 0x0F; 
		
		//Wait: Transfer Complete 'TC' flag
		while(((I2C2->ISR & 0x40) >> 6) != 1){ //is bit 6 a 1? 
			//this is great way to isolate the bit with a mask, 
			//since we don't know what the other bits might be doing 
			//in the ISR register
		}
		
		//Reset parameter: RD_WRN bit to 'READ' operation
		//Set the Gyroscope Slave Address in SADD: 0x69
		//Set number of bytes to 1
		//Master setup: READ
		I2C2->CR2 = 0; //Clear to start.
		I2C2->CR2 |= (0x69 << 1); //Slave address is [7:1]
		I2C2->CR2 |= (1 << 16);
		I2C2->CR2 |= (1 << 10);
		
		//Start bit: 'Restart Condition'
		I2C2->CR2 |= (1 << 13);
		
		//Wait until either of the RXNE (Receive Register Not Empty) 
		//or NACKF (Slave NotAcknowledge) flags are set.
		//----//Continue if the RXNE flag is set.
		while(((I2C2->ISR & 0x4) >> 2) != 1){
			if((I2C2->ISR & 0x10) >> 4){ //NACKF error
				//LED Error Indicator Setup
				GPIOC->ODR |= (1 << 6); //red LED
				
			}
		}
				
		////Wait: Transfer Complete 'TC' flag
		while(((I2C2->ISR & 0x40) >> 6) != 1){
			
		}
		
		//Check RXDR Register
		short who = I2C2->RXDR;
		if (who == 0xD3) {
			GPIOC->ODR |= (1 << 7);	//Blue LED on
		}
		else {
			GPIOC->ODR |= (1 << 6);	//RED LED on: ERROR
		}
		
		//STOP bit: Release the I2C bus
		I2C2->CR2 |= (1 << 14); 
		
}

void init_Gyro(void){
		I2C2->CR2 = 0; //Clear to start.
		
		//Set the Gyroscope Slave Address in SADD: 0x69
		I2C2->CR2 |= (0x6B << 1); //Slave address is [7:1]
		
		//Set number of bytes to 2 (Address + new reg value).
		I2C2->CR2 |= (2 << 16);
		
		//Set the RD_WRN bit to 'WRITE' operation.
		//Set the START bit
		I2C2->CR2 &= ~(1 << 10); //Ensure 0 for write
		I2C2->CR2 |= (1 << 13);

		//Wait until either of the TXIS (Transmit Register Empty/Ready) 
		//or NACKF (Slave NotAcknowledge) flags are set.
		//----//If the NACKF flag is set, the slave did not respond to the address frame.
		//----//Continue if the TXIS flag is set
		while(((I2C2->ISR & 0x2) >> 1) != 1){
			if((I2C2->ISR & 0x10) >> 4){ //NACFK flag check
				//Turn on LED to indicate config error
				GPIOC->ODR |= (1 << 6);
			}
		}
		
		//Set address of CTRL_REG1 register to be sent on bus
		I2C2->TXDR = 0x20;
		
		//GPIOC->ODR |= (1 << 6);
		
		//Wait for TXIS flag
		while(((I2C2->ISR & 0x2) >> 1) != 1){
			if((I2C2->ISR & 0x10) >> 4){ //NACFK flag check
				//Turn on LED to indicate config error
				GPIOC->ODR |= (1 << 6);
			}
		}
		
		//Set value to enable X, Y axes and normal/sleep mode. (00001011)
		I2C2->TXDR = 11;
		
		//Wait for TC flag
		while(((I2C2->ISR & 0x40) >> 6) != 1){
			
		}
		
		//Set the stop bit to complete transaction.
		I2C2->CR2 |= (1 << 14); 
		

}

short init_xread(void){ 
		//Transmit corresponding X-Axis address
		I2C2->CR2 = 0; //Clear to start.
		
		//Set the Gyroscope Slave Address in SADD: 0x69
		//Set the number of bytes to transmit = 1
		I2C2->CR2 |= (0x69 << 1); //Slave address is [7:1]
		I2C2->CR2 |= (1 << 16);
		
		//Set the RD_WRN bit to 'WRITE' operation.
		//Set the START bit
		I2C2->CR2 &= ~(1 << 10); //Ensure 0 for write
		I2C2->CR2 |= (1 << 13);

		//Wait until either of the TXIS (Transmit Register Empty/Ready) 
		//or NACKF (Slave NotAcknowledge) flags are set.
		//----//If the NACKF flag is set, the slave did not respond to the address frame.
		//----//Continue if the TXIS flag is set
		while(((I2C2->ISR & 0x2) >> 1) != 1){
			if((I2C2->ISR & 0x10) >> 4){ //Check NACFK flag
				//LED Error Indicator Setup
				GPIOC->ODR |= (1 << 6);
			}
		}
		
		//WHO_AM_I -------
		//Bus send: X-Axis register address setup
		I2C2->TXDR = 0xA8; //might be 0x0F
		
		//Wait: Transfer Complete 'TC' flag
		while(((I2C2->ISR & 0x40) >> 6) != 1){
			
		}
		
		//Reset parameter: RD_WRN bit to 'READ' operation
		//Set the Gyroscope Slave Address in SADD: 0x69
		//Set number of bytes to 2
		//Master setup: READ
		I2C2->CR2 = 0; //Clear to start.
		I2C2->CR2 |= (0x69 << 1); //Slave address is [7:1]
		I2C2->CR2 |= (2 << 16);
		I2C2->CR2 |= (1 << 10);
		
		//Start bit: 'Restart Condition'
		I2C2->CR2 |= (1 << 13);
		
		//Wait until either of the RXNE (Receive Register Not Empty) 
		//or NACKF (Slave NotAcknowledge) flags are set.
		//----//Continue if the RXNE flag is set.
		while(((I2C2->ISR & 0x4) >> 2) != 1){
			if((I2C2->ISR & 0x10) >> 4){ //NACKF error
				//LED Error Indicator Setup
				GPIOC->ODR |= (1 << 6);
				
			}
		}
		
		//Check RXDR Register
		short low = I2C2->RXDR;
		
		//Wait for RXNE Flag
		while(((I2C2->ISR & 0x4) >> 2) != 1){
			if((I2C2->ISR & 0x10) >> 4){ //NACKF error
				//LED Error Indicator (config error)
				GPIOC->ODR |= (1 << 6);	
			}
		}
		
		//Check RXDR Register
		short high = I2C2->RXDR;
		
				
		////Wait: Transfer Complete 'TC' flag
		while(((I2C2->ISR & 0x40) >> 6) != 1){
			
		}
		
		//Set the stop bit
		I2C2->CR2 |= (1 << 14); 
		
		high = high << 8;
		high |= low;
		
		return high;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
