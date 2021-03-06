/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#include "stdio.h"
#include "stdlib.h"
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
  #define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */
	PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,1);
	HAL_UART_Transmit(&huart1, (uint8_t*)&ch,1,100);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,0);
  /* Loop until the end of transmission */


  return ch;
}
#define Sampling_time		1	 //thoi gian lay mau (ms)
#define inv_Sampling_time	100 	 // 1/Sampling_time 
#define PWM_Period			14400 //8000 cycles = 1ms, f=8MHz
int32_t total_pluse_left=0,Pulse=0,new_pluse_left=0,old_pluse_left=0,i;
uint8_t Rx_data,Tx_data;
uint8_t Rx_buffer[50];
uint32_t pwm=0,old_time,new_time,_timer,old_time_1,new_time_1;
int32_t sp_sub,start=0,count_timer;
uint8_t step,flag_timer;
typedef struct 
{
	int32_t Setpoint;
	int32_t Old_setpoint;
	int32_t Delta;
	int32_t accelerator;
	int32_t speed;
	int32_t pre_speed;
	int32_t Kp;
	int32_t Ki;
	int32_t Kd;
	int32_t pre_Err;
	int32_t Err;
	int32_t pPart;
	int32_t iPart;
	int32_t dPart;
	int32_t Output;
}motor_parameter;
typedef struct 
{
	int32_t now_position;
	int32_t old_position;
	int32_t run_pluse;
	int32_t dir;
	uint32_t now_time;
	uint32_t old_time;
	uint32_t off_set;
}step_motor;
step_motor my_step;
motor_parameter my_motor;
void process_pluse()
{
	new_pluse_left=TIM3->CNT;
	
	if(new_pluse_left-old_pluse_left!=0)
	{
		i=new_pluse_left-old_pluse_left;
		if((TIM3->CR1)&0x10) // true = giam
		{
		total_pluse_left -= 1;
		}
		else
		{
		total_pluse_left += 1; 
		}
		old_pluse_left=new_pluse_left;
	}
}

void Motor_Position(int32_t SP,int32_t PV)
{  
					my_motor.Err=SP-PV;  
					my_motor.pPart=my_motor.Kp*my_motor.Err;
					my_motor.dPart=my_motor.Kd*(my_motor.Err-my_motor.pre_Err);	
					my_motor.iPart+=my_motor.Ki*my_motor.Err;
				  if(my_motor.iPart<0){my_motor.iPart=-my_motor.iPart;}
					if(my_motor.iPart>PWM_Period){my_motor.iPart=PWM_Period;}
					my_motor.Output =my_motor.pPart+my_motor.dPart+my_motor.iPart;     
					my_motor.pre_Err=my_motor.Err;
					
					if (my_motor.Output <0)
						{
							my_motor.Output =-my_motor.Output;
							HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,1);
						}
						else
						{
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,0);
						}
					if (my_motor.Output >=PWM_Period) my_motor.Output=PWM_Period;
						TIM2->CCR3=my_motor.Output; 			
					my_motor.pre_speed=my_motor.speed;
			
}
void HAL_SYSTICK_Callback(void)
{
_timer++;
}
uint32_t millis()
{
	return _timer;
}
void Run_step_motor(uint8_t pluse,uint8_t dir)
{
	if(dir==1)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,1);
		my_step.run_pluse=-my_step.run_pluse;
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,0);
	}
	if(pluse)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,1);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,0);
		pluse=0;
	}
	else
	{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,0);
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		if(htim->Instance == TIM4)
	{
		if(start)
		{
			flag_timer=1;
		}
		//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_1); // Thuc hien chop tat led PB12
	}
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
		HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
		HAL_TIMEx_PWMN_Start(&htim2,TIM_CHANNEL_3);
		my_motor.Kp=5;
		my_motor.Ki=3;
		my_motor.Kd=1;
		my_motor.pre_speed=0;
		my_motor.accelerator=1;
		printf("Hello\r \n");
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,0);
		
		my_motor.Setpoint=20000;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		HAL_UART_Receive_IT(&huart1, &Rx_data, 1); //Kich hoat ngat uart khi nhan duoc 1 byte
		HAL_TIM_Base_Start_IT(&htim4);
		new_time=millis();
		my_step.now_time=millis();
		new_time_1=millis();
			process_pluse();
		if(new_time_1-old_time_1>5000)
		{
			my_motor.Setpoint=-my_motor.Setpoint;
			old_time_1=new_time_1;
		}
		
		if(flag_timer)
		{
			//count_timer++;
			sp_sub+=5;
			if(sp_sub>=my_motor.Setpoint)
			{
				sp_sub=my_motor.Setpoint;
			}
			flag_timer=0;
		}
		if(new_time-old_time>1)
		{
			Motor_Position(sp_sub,total_pluse_left);
		
			if(my_step.now_position>0)
			{
				Run_step_motor(1,my_step.dir);
				my_step.now_position--;
			}
			old_time=new_time;
		}
				if(Rx_data=='a')
		{
			start=1;
			Tx_data=Rx_data;
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,1);
			HAL_UART_Transmit(&huart1, &Tx_data,1, 1000);	//Truyen buffer voi chieu dai qua uart3
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,0);
			Tx_data=0x00;
			Rx_data=0x00;
		}
			 if(Rx_data=='b')
		{
			start=0;
			Tx_data=Rx_data;
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,1);
			HAL_UART_Transmit(&huart1, &Tx_data,1, 1000);	//Truyen buffer voi chieu dai qua uart3
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,0);
			Tx_data=0x00;
			Rx_data=0x00;
		}
			 if(Rx_data=='c')
		{
			my_step.now_position=36000;
			my_step.dir=1;
			Tx_data=Rx_data;
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,1);
			HAL_UART_Transmit(&huart1, &Tx_data,1, 1000);	//Truyen buffer voi chieu dai qua uart3
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,0);
			Tx_data=0x00;
			Rx_data=0x00;
		}
			 if(Rx_data=='d')
		{
			
			my_step.now_position=36000;
			my_step.dir=0;
			Tx_data=Rx_data;
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,1);
			HAL_UART_Transmit(&huart1, &Tx_data,1, 1000);	//Truyen buffer voi chieu dai qua uart3
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,0);
			Tx_data=0x00;
			Rx_data=0x00;
		}
		 if(Rx_data=='e')
		{
			my_step.now_position=0;
			my_step.dir=0;
			Tx_data=Rx_data;
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,1);
			HAL_UART_Transmit(&huart1, &Tx_data,1, 1000);	//Truyen buffer voi chieu dai qua uart3
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,0);
			Tx_data=0x00;
			Rx_data=0x00;
		}
		
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
