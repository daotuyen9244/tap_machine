/**
  ******************************************************************************
    File Name          : main.c
    Description        : Main program body
  ******************************************************************************

    COPYRIGHT(c) 2016 STMicroelectronics

    Redistribution and use in source and binary forms, with or without modification,
    are permitted provided that the following conditions are met:
      1. Redistributions of source code must retain the above copyright notice,
         this list of conditions and the following disclaimer.
      2. Redistributions in binary form must reproduce the above copyright notice,
         this list of conditions and the following disclaimer in the documentation
         and/or other materials provided with the distribution.
      3. Neither the name of STMicroelectronics nor the names of its contributors
         may be used to endorse or promote products derived from this software
         without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
#include <stdio.h>
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  /* Loop until the end of transmission */
  return ch;
}
#define PWM_Period      7200    // gia tri PWM toi da
#define PPR             1600    // so xung tren 1 vong
#define DPR             10      // buoc cua visme -> dung cho ham chuyen doi
#define T1              180// 2560
#define T2              90//1280
#define T3              10
// Bien dung de dieu khien dong co
uint16_t  acc = 3, f_run, f_max; // truyen setpoint trung gian
int32_t counter_over_distance = 0, reset_counter_over_distance = 0, rx_count = 0;
// Bien nha truyen nhan du lieu
uint8_t  Rx_data, Rx_indx,  transfer_cplt, x_counter, y_counter, f_counter, g_counter, t_counter; //Khai bao cac bien de nhan du lieu
char x_buff[10], f_buff[10], t_buff[10], Rx_Buffer[20], y_buff[10];
uint8_t len; // do dai chuoi nhan dc
// Bien dieu khien tgian
uint32_t new_time_calculator_PID, old_time_calculator_PID, _timer;
uint32_t old_time_update_speed, new_time_update_speed, new_test_time, old_test_time, new_time_check_distance, old_time_check_distance;
uint32_t new_time_check_sensor, old_time_check_sensor, new_time_sethome, old_time_sethome, new_time_requset, old_time_request;
uint32_t new_time_check_error_frame, old_time_check_error_frame, count_error_frame, old_frame_error, new_frame_error, true_frame;
// thong so cai dat
float x = 0.0; // x,y vi tri se chay den
uint32_t f = 200, y, time_request, count_frame, count_frame_error; // f van toc, time_request tgian tra loi cac thong so cho may tinh
int32_t  plus_position; // sp_arr mang chia nho cac setpoint ra,
int32_t temp_dc_position = 0, temp_step_positon = 0, time_requset_offset = 233;
uint8_t g_command, m_command, r_command, reciver_cmd = 0, allow_run = 0, update_setpoint_flag = 0, old_cmd = 0, step_set_home = 0, flag_request = 1, flag_over_distance = 0;
enum machine_state {NONE = 0, STOP, START, SET_HOME, RUN, OVER_DISTANCE};
uint8_t step_program = SET_HOME, set_home_cplt = 0, reset_system_ok = 1, run_ok = 0, count_reset = 0,reset_step;
uint8_t flag_reset_home = 0, start_check_error_encoder = 0, flag_send_error_frame = 0;
int32_t calculate_error_encoder, reset = 0;
typedef struct
{
  int32_t Setpoint;
  int32_t Old_Setpoint;
  int32_t sub_Setpoint;
  int32_t total_pluse;
  int32_t abs_pluse;
  int32_t new_pluse;
  int32_t old_pluse;
  int32_t round;
  uint8_t new_change;
  uint8_t old_change;
  int32_t Kp;
  int32_t Ki;
  int32_t Kd;
  int32_t pre_Err;
  int32_t Err;
  int32_t pPart;
  int32_t iPart;
  int32_t dPart;
  int32_t Output;
  uint8_t  Dir;
  int32_t counter;
  int32_t check_zero;
  int32_t time_check_zero;
} motor_parameter;
typedef struct
{
  int32_t now_position;
  int32_t old_position;
	int32_t pre_reset;
  int32_t pluse;
  int32_t allow;
  int32_t run_pluse;
  int32_t dir;
  uint32_t now_time;
  uint32_t old_time;
  uint32_t off_set;
} step_motor;

step_motor my_step;
motor_parameter my_motor;
void process_pluse(void);// do va xu ly tin hieu encoder
void Motor_Position(motor_parameter *data);// tinh toan PID vi tri
void HAL_SYSTICK_Callback(void);// chuong trinh ngat system stick
uint32_t millis(void);// chuong trinh tao millis tu system stick
void Run_step_motor(uint8_t pluse, uint8_t dir);// chuong trinh dieu khien step motor
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);// chuong trinh ngat timer 4
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);// chuong trinh ngat uart
void update_new_setpoint_new_speed(void);// cap nhat gia tri setpoint moi nhan tu may tinh
void process_data_recive(void);// xu ly du lieu nhan dc tu may tinh
uint8_t check_limit_switch(void);// kiem tra Limit Switch
void control_motor_program(void);// chua ham tinh PID va ham dieu khien step
int32_t conver_distance_to_pluse(float decimail_distance);// ham chuyen doi vi tri nhan dc sang xung
void machine_state(void);// chuong trinh chua trang thai hien tai cua may
void requset_machine_state(void);
void reset_encoder(motor_parameter *data, TIM_HandleTypeDef *htim);
void reset_system(void);
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
  MX_TIM2_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim2, TIM_CHANNEL_3);
  my_motor.Kp = 30;//50
  my_motor.Ki = 0;//4
  my_motor.Kd = 0;//50
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    HAL_UART_Receive_IT(&huart1, &Rx_data, 1); //Kich hoat ngat uart khi nhan duoc 1 byte
    HAL_TIM_Base_Start_IT(&htim4);
    if (reset_system_ok == 0)
    {
      if (run_ok)
      {
        reset_system();
				reset_system_ok = 1;
      }
    }
		if(reset_step)
		{
			new_time_sethome = millis();
         if (new_time_sethome - old_time_sethome > 1)
          {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 1)
             {
								my_step.now_position=T3;
								my_step.old_position=T3;
								my_step.pluse=0;
								my_step.now_position=my_step.pre_reset;
                
              }
             else
             {
									my_step.pluse++;
                  my_step.dir = 0;
             }
             old_time_sethome = new_time_sethome;
         }
			reset_step=0;
		}
    process_data_recive();
    machine_state();
    update_new_setpoint_new_speed();
    control_motor_program();
    requset_machine_state();
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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void reset_system(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_UART_MspDeInit(&huart1);
  HAL_UART_MspInit(&huart1);
  MX_USART1_UART_Init();
  HAL_UART_Receive_IT(&huart1, &Rx_data, 1);
}
void reset_encoder(motor_parameter *data, TIM_HandleTypeDef *htim)
{
  __HAL_TIM_SetCounter(htim, 0);
  data->old_change = 0;
  data->new_change = 0;
  data->round = 0;
  htim->Instance->CNT = 0;
  data->old_pluse = 0;
  data->new_pluse = 0;
  data->total_pluse = 0;
  data->sub_Setpoint = 0;
  data->Setpoint = 0;
  data->abs_pluse = 0;
}
void process_pluse(void)
{
  my_motor.new_pluse = TIM3->CNT;
  if (my_motor.new_pluse - my_motor.old_pluse != 0)
  {

    if ((TIM3->CR1) & 0x10) // true = giam
    {
      my_motor.total_pluse -= 1;
    }
    else
    {
      my_motor.total_pluse += 1;
    }
    my_motor.old_pluse = my_motor.new_pluse;
  }
}
void Motor_Position(motor_parameter *data)
{
  my_motor.Err = my_motor.sub_Setpoint - my_motor.total_pluse;
  my_motor.pPart = my_motor.Kp * my_motor.Err;
  my_motor.dPart = my_motor.Kd * (my_motor.Err - my_motor.pre_Err);
  my_motor.iPart += my_motor.Ki * my_motor.Err / 1000;
  my_motor.pre_Err = my_motor.Err;
  my_motor.Output = my_motor.pPart + my_motor.dPart + my_motor.iPart;
  if (data->Output < 0)
  {
    data->Output = -(data->Output);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  }
  if (data->Output > 7200) {
    data->Output = 7200;
  }

  if (flag_over_distance == 0)
  {
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, my_motor.Output);
  }
  else
  {
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 0);
  }
}
void HAL_SYSTICK_Callback(void)
{
  _timer++;
  if (_timer > 300000)
  {
    reset_system_ok = 0;
		reset_step=1;
    reset++;
    _timer = 0;
  }
}
uint32_t millis(void)
{
  return _timer;
}
void Run_step_motor(uint8_t pluse, uint8_t dir)
{
  if (dir == 1)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    my_step.run_pluse = -my_step.run_pluse;
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  }
  if (pluse)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    pluse = 0;
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  }

  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5))
  {
    if (my_step.allow)
    {

      my_step.pluse = 0;
      my_step.allow = 0;
    }
  }



}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  process_pluse();
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  if (huart->Instance == USART1)  //UART hien tai la uart3
  {
    if (Rx_data != '\n') //Neu du lieu nhan duoc khac voi 13 (13 la ma cua phim enter)
    {
      Rx_Buffer[Rx_indx++] = Rx_data; //them du lieu vao Rx_Buffer
      if (Rx_indx == 1)
      {
        count_frame++;
      }
    }
    else      //Neu phim enter duoc an
    {
      Rx_indx = 0; //Dua index ve 0
      transfer_cplt = 1; //Qua trinh truyen hoan tat, du lieu da san sang de doc
    }
  }
}
void update_new_setpoint_new_speed(void)
{
  if (update_setpoint_flag)
  {
    new_time_update_speed = millis();

    if (new_time_update_speed - old_time_update_speed > 2)
    {
      f_max = f / 2;
      if (my_motor.counter < my_motor.abs_pluse * 10)
      {
        f_run = f_run + acc;
      }
      else if (abs(my_motor.counter) > my_motor.abs_pluse * 90)
      {
        f_run = f_run - acc;
      }
      else
      {
        f_run = f_run;
      }

      if (f_run < 30)f_run = 30;
      if (f_run > f_max)f_run = f_max;
      if (my_motor.Setpoint - my_motor.total_pluse > 0)
      {
        if (my_motor.sub_Setpoint > my_motor.Setpoint)
        {
          my_motor.sub_Setpoint = my_motor.Setpoint;
          run_ok = 1;
          start_check_error_encoder = 1;
          update_setpoint_flag = 0;
          my_motor.counter = 0;
        }
        else
        {
          my_motor.sub_Setpoint += f_run;
          my_motor.counter += f_run;
        }
      }
      else
      {
        if (my_motor.sub_Setpoint < my_motor.Setpoint)
        {
          my_motor.sub_Setpoint = my_motor.Setpoint;
          run_ok = 1;
          start_check_error_encoder = 1;
          update_setpoint_flag = 0;
          my_motor.counter = 0;
        }
        else
        {
          my_motor.sub_Setpoint -= f_run;
          my_motor.counter += f_run;

        }
      }
      old_time_update_speed = new_time_update_speed;
    }
  }
}
void process_data_recive(void)
{
  new_time_check_error_frame = millis();
  if (new_time_check_error_frame - old_time_check_error_frame > 200)
  {
    if (count_frame != rx_count)
    {
      old_frame_error = new_frame_error;
      new_frame_error = count_frame;
      count_frame = rx_count;
      flag_send_error_frame = 1;
      true_frame = 0;
    }
    else
    {
      true_frame = 1;
    }
    old_time_check_error_frame = new_time_check_error_frame;
  }
  if (transfer_cplt)
  {
    rx_count++;
    if (true_frame)
    {
      len = strlen(Rx_Buffer); //Lay chieu dai cua Rx_Buffer
      for (uint8_t i = 0; i < len; i++)
      {
        if (Rx_Buffer[i] == 'G')
        {

          reciver_cmd = 1;
        }
        if (Rx_Buffer[i] == 'R')
        {

          reciver_cmd = 2;
          r_command = Rx_Buffer[i + 1] - 48;
          t_counter = i + 3;
        }
        if (Rx_Buffer[i] == 'H')
        {

          reciver_cmd = 3;
        }
      }
      if (reciver_cmd == 1)
      {

        for (uint8_t i = 0; i < len; i++)
        {
          if (Rx_Buffer[i] == 'X')
          {
            g_counter = i + 1;

          }
          if (Rx_Buffer[i] == 'Y')
          {
            x_counter = i + 1;

          }
          if (Rx_Buffer[i] == 'F')
          {
            y_counter = i + 1;
          }
        }
        f_counter = len;

        for (uint8_t i = g_counter; i < x_counter - 1; i++)
        {
          x_buff[i - g_counter] = Rx_Buffer[i];
        }
        for (uint8_t i = x_counter; i < y_counter - 1; i++)
        {
          y_buff[i - x_counter] = Rx_Buffer[i];
        }
        //y=Rx_Buffer[x_counter]-48;
        for (uint8_t i = y_counter; i < f_counter - 1; i++)
        {
          f_buff[i - y_counter] = Rx_Buffer[i];
        }
        x = atof(x_buff);
        y = atoi(y_buff);
        f = atoi(f_buff);
        if (y == 1)
        {

          my_step.now_position = T1;
					my_step.pre_reset=my_step.now_position;
          my_step.allow = 1;
        }
        else if (y == 2)
        {

          my_step.now_position = T2;
					my_step.pre_reset=my_step.now_position;
          my_step.allow = 1;
        }
        else if (y == 3)
        {

          my_step.now_position = T3;
					my_step.pre_reset=my_step.now_position;
          my_step.allow = 1;
        }
        else
        {

          my_step.now_position = my_step.old_position;
					my_step.pre_reset=my_step.now_position;
          my_step.allow = 1;
        }


        if (f > 300)
        {
          f = 300;
        }
        if (x > 0.4)
        {
          x = 0.0;
        }
        if (x < -285.0)
        {
          x = -285.0;
        }

        if (flag_over_distance == 0)
        {
          if (set_home_cplt)
          {
            my_motor.Setpoint = conver_distance_to_pluse(x);
            my_motor.abs_pluse = abs(my_motor.Setpoint - my_motor.Old_Setpoint) / 100;
            my_motor.Old_Setpoint = my_motor.Setpoint;
            update_setpoint_flag = 1;
            run_ok = 0;
            step_program = RUN;
          }
        }
        for (uint8_t i = 0; i < 10; i++)
        {
          x_buff[i] = 0x00;
          y_buff[i] = 0x00;
          f_buff[i] = 0x00;
        }
        reciver_cmd = 0;
      }
      else if (reciver_cmd == 2)
      {

        if (r_command)
        {
          for (uint8_t i = t_counter; i < len - 1; i++)
          {
            t_buff[i - t_counter] = Rx_Buffer[i];
          }
          flag_request = 1;
          time_request = atoi(t_buff);
        }
        else
        {
          flag_request = 0;
          time_request = 233;
        }
        for (uint8_t i = 0; i < 20; i++)
        {
          Rx_Buffer[i] = 0x00;
        }

        reciver_cmd = 0;
      }
      else if (reciver_cmd == 3)
      {
        NVIC_SystemReset();
        reciver_cmd = 0;
      }
      else
      {
        for (uint8_t i = 0; i < 20; i++)
        {
          Rx_Buffer[i] = 0x00;
        }
      }
      for (uint8_t i = 0; i < 20; i++)
      {
        Rx_Buffer[i] = 0x00;
      }
    }
    //reset_system();
    transfer_cplt = 0; //Reset lai bien tranfer_complete
  }
}
uint8_t check_limit_switch(void)
{
  uint8_t data = 0x00;
  data |= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
  data |= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) << 1;
  data |= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) << 2;
  data |= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) << 3;
  return data;
}
void control_motor_program(void)
{
  new_time_calculator_PID = millis();
  if (new_time_calculator_PID - old_time_calculator_PID > 1)
  {

    Motor_Position(&my_motor);
    //if (!my_motor.counter)
    //{
      if (my_step.allow)
      {
        if (my_step.now_position != my_step.old_position)
        {
          my_step.pluse = my_step.now_position - my_step.old_position;
          if (my_step.pluse >= 0)
          {
            my_step.pluse = my_step.pluse;
            my_step.dir = 1;
          }
          else
          {
            my_step.pluse = -my_step.pluse;
            my_step.dir = 0;
          }
          my_step.old_position = my_step.now_position;
        }
        my_step.allow = 0;
      }
      if (my_step.pluse > 0)
      {
				my_step.pluse--;
        Run_step_motor(1, my_step.dir);
        
      }
    //}
    old_time_calculator_PID = new_time_calculator_PID;
  }
}
int32_t conver_distance_to_pluse(float decimail_distance)
{
  float data = 0.0;
  data = PPR * decimail_distance / DPR;
  return (int32_t)data;
}
void machine_state(void)
{
  switch (step_program)
  {

    case SET_HOME:
      {

        switch (step_set_home)
        {
          case 0:
            {

              if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 1)
              {
                step_set_home = 2;
              }
              reset_encoder(&my_motor, &htim3);
              my_motor.Setpoint = 0;
              my_step.allow = 0;
              my_step.now_position = 0;
              my_step.old_position = 0;
              my_step.pluse = 0;
              step_set_home = 1;


              break;
            }
          case 1:
            {
              new_time_sethome = millis();

              if (new_time_sethome - old_time_sethome > 1)
              {
                my_motor.sub_Setpoint += 20;

                if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 1)
                {
                  my_step.allow = 0;
                  my_step.now_position = T3;
                  my_step.old_position = 0;
                  //my_step.pluse = 0;
                  my_motor.sub_Setpoint = my_motor.total_pluse;
                  reset_encoder(&my_motor, &htim3);
                  step_set_home = 2;
                }
                old_time_sethome = new_time_sethome;
              }
              break;
            }
          case 2:
            {
              flag_over_distance = 0;
              update_setpoint_flag = 0;
              x = 0;
              y = 0;
              f = 0;
              reset_encoder(&my_motor, &htim3);
              step_set_home = 4;
              new_time_sethome = millis();
              old_time_sethome = new_time_sethome;

              break;
            }
          case 3:
            {
              break;
            }
          case 4:
            {
              step_set_home = 5;
              update_setpoint_flag = 0; x = 0; y = 0; f = 0;
              reset_encoder(&my_motor, &htim3);
              new_time_sethome = millis();
              old_time_sethome = new_time_sethome;
              break;
            }
          case 5:
            {
              new_time_sethome = millis();
              if (new_time_sethome - old_time_sethome > 1)
              {
                if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 1)
                {
                  my_step.allow = 0;
                  my_step.now_position = 0;
                  my_step.old_position = 0;
                  my_step.pluse = 0;
                  my_step.old_position = 0; //flag_reset_home = 1;
                  reset_encoder(&my_motor, &htim3);
                  flag_over_distance = 0;
                  reset_encoder(&my_motor, &htim3);
                  count_reset++;
                  if (count_reset < 10)
                  {
                    step_set_home = 1;
                  }
                  else
                  {
                    step_set_home = 6;
                  }
                }
                else
                {
                  my_step.pluse++;
                  my_step.dir = 0;
                }
                old_time_sethome = new_time_sethome;
              }
              break;
            }
          case 6:
            {
              count_reset = 0;
              set_home_cplt = 1;
              step_set_home = 0;
              my_step.now_position = T3;
							my_step.pre_reset=T3;
              reset_system_ok = 0;
              run_ok = 1;
              step_program = RUN;
              break;
            }
          default: break;
        }
        break;
      }
    case RUN:
      {
        my_motor.Setpoint = conver_distance_to_pluse(x);
        break;
      }
    case OVER_DISTANCE:
      {
        flag_over_distance = 1;
        my_step.allow = 0;
        my_step.now_position = 0;
        my_step.old_position = 0;
        my_step.pluse = 0;
        break;
      }
    default: break;
  }
}
void requset_machine_state(void)
{
  float temp_x = 0.0;

  new_time_requset = millis();
  new_test_time = millis();
  if (new_test_time - old_test_time > 1)
  {
    reset_counter_over_distance++;
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 1)
    {
      counter_over_distance++;
    }
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 1)
    {
      counter_over_distance++;
    }
    old_test_time = new_test_time;
  }
  if (reset_counter_over_distance > 50)
  {
    counter_over_distance = 0;
    reset_counter_over_distance = 0;
  }
  if (counter_over_distance > 10)
  {
    counter_over_distance = 0;
    flag_over_distance = 1;
    step_program = OVER_DISTANCE;
  }
  if (new_time_requset - old_time_request > time_requset_offset)
  {
    if (set_home_cplt)
    {
      temp_x = (float)DPR * my_motor.total_pluse / PPR;
      if (temp_x > 1.0)
      {
        flag_over_distance = 1;
        step_program = OVER_DISTANCE;
      }
      else if (temp_x < -290.0)
      {
        flag_over_distance = 1;
        step_program = OVER_DISTANCE;
      }
    }
    if (flag_over_distance == 1)
    {
      printf("M3X%3.3fF%d\r\n", temp_x, f);
    }
    else
    {
      if (flag_request)
      {
        if (!flag_send_error_frame)
        {
          if (step_program == RUN)
          {
            printf("X%3.3fF%d\r\n", temp_x, f);
          }
          else if (step_program == SET_HOME)
          {
            printf("H0F200\r\n");
          }
        }
        else
        {
          printf("M4F%d\r\n", new_frame_error);
          count_error_frame++;
          flag_send_error_frame = 0;
        }

      }
    }
    old_time_request = new_time_requset;
  }

}
/* USER CODE END 4 */

/**
    @brief  This function is executed in case of error occurrence.
    @param  None
    @retval None
*/
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while (1)
  {
  }
  /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
     @brief Reports the name of the source file and the source line number
     where the assert_param error has occurred.
     @param file: pointer to the source file name
     @param line: assert_param error line source number
     @retval None
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
    @}
*/

/**
    @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
