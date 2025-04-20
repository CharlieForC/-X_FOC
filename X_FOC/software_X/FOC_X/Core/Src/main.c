/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "define.h"
#include "DRV8353.H"
#include "AS5047P.H"
#include "FOC.H"
#include "math.h"
#include "stdio.h"
#include <string.h>
#include "arm_math.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define abs(x) ((x)>0?(x):-(x))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void SVPWM_SET_OUT(float angle_el,float Uq,float Ud);
void lib_position_control(float rad);
void lib_speed_control(float speed);
void lib_torque_control(float torque_norm_q);
void lib_speed_torque_control(float speed_rad, float max_torque_norm);
void lib_position_torque_control(float position_rad, float max_torque_norm);
void lib_position_speed_torque_control(float position_rad, float max_speed_rad, float max_torque_norm);
float FOC_speed_pid(float speed,float _motor_speed);
float V_Q_pid=0;

float motor_i_u,motor_i_v,motor_i_d,motor_i_q;
float u_1, u_2;

////中值滤波////
#define lenth 3
float data_mon[lenth];
float lvbo_data[lenth];

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float angle;
uint16_t as5047_rx_data;
float angle_add,angle_Multi,angle_mon;////多圈角度变量
float motor_speed=0;
float D_U,D_V,D_W;//占空比
///****串口****//////////////
uint8_t	 RX_Buf[128]={0};//DMA接收数组 
uint8_t	 RX_RCBuf[128]={'0'};//DMA接收数组存储

int fputc(int ch,FILE *f)
{

	HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,HAL_MAX_DELAY);  
	return ch;
}
float angle_mon_last=0;
float angle_diff;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float speed_diff,Uq_set,tor_set;
float tor_diff,speed_set;
float postion_diff;

unsigned int ADC_tim_count=0;
float vote_BUS,tempture=0;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static arm_pid_instance_f32 pid_position;
static arm_pid_instance_f32 pid_speed;
static arm_pid_instance_f32 pid_torque_d;
static arm_pid_instance_f32 pid_torque_q;
void set_motor_pid(  //////PID参数
float position_p, float position_i, float position_d,
float speed_p, float speed_i, float speed_d,
float torque_d_p, float torque_d_i, float torque_d_d,
float torque_q_p, float torque_q_i, float torque_q_d)
{
    pid_position.Kp = position_p;
    pid_position.Ki = position_i;
    pid_position.Kd = position_d;

    pid_speed.Kp = speed_p;
    pid_speed.Ki = speed_i;
    pid_speed.Kd = speed_d;

    pid_torque_d.Kp = torque_d_p;
    pid_torque_d.Ki = torque_d_i;
    pid_torque_d.Kd = torque_d_d;

    pid_torque_q.Kp = torque_q_p;
    pid_torque_q.Ki = torque_q_i;
    pid_torque_q.Kd = torque_q_d;
    arm_pid_init_f32(&pid_position, false);//false代表清空内部增量数据
    arm_pid_init_f32(&pid_speed, false);
    arm_pid_init_f32(&pid_torque_d, false);
    arm_pid_init_f32(&pid_torque_q, false);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM15_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	Run_ON();//// 
	Fault_ON();             
  Set_DRV8353();	
	/////PWM init//////
	HAL_TIM_Base_Start (&htim1);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_PWM_Start(&htim1 ,TIM_CHANNEL_1 );
	HAL_TIM_PWM_Start(&htim1 ,TIM_CHANNEL_2 );
	HAL_TIM_PWM_Start(&htim1 ,TIM_CHANNEL_3 );
	///串口
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);	//使能IDLE(空闲)中断
  HAL_UART_Receive_DMA(&huart1,RX_Buf,128);		//启动DMA接收
	/////tim15//////////
	HAL_TIM_Base_Start_IT(&htim15);
  HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
  HAL_ADCEx_InjectedStart_IT(&hadc1);
	HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)0xffff, (uint8_t *)&as5047_rx_data, 2);

	AS5407P_CS_L
		/////零点////////
	__HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_1 ,(int)(2125*0.05));
	__HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_2 ,(int)(2125*0));
	__HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_3 ,(int)(2125*0));
	angle_mon_flag=1;
	HAL_Delay(400);
	__HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_1 ,(int)(2125*0));
	__HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_2 ,(int)(2125*0));
	__HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_3 ,(int)(2125*0));

//set_motor_pid(//位置-力矩Pi
//     30.000000,0.00951 , 0,///位置环
//      0.0, 0.0, 0,///速度环
//      0, 0, 0,///力矩环d
//      0.0810100, 0.0000101, 0);////力矩环q

//		set_motor_pid( ///速度-力矩环
//      0.001 , 0,0,///位置环
//      0.01, 0.00013, 0,///速度环
//      0, 0, 0,///力矩环d
//      0.0110115, 0.00011, 0);////力矩环q

	set_motor_pid(          ///独立模式
      2.990,0.0000301 , 0,///位置环
      0.001001, 0.0001001, 0,///速度环
      0, 0, 0,///力矩环d
      0.0054, 0.001, 0);////力矩环q
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(motor_mode==1)
		{
			printf("%.2f,%.2f\n",motor_tor_set*MAX_CURRENT,motor_i_q);
		}
		if(motor_mode==2)
		{
			printf("%.2f,%.2f\n",motor_speed_set,motor_speed);
		}
		if(motor_mode==3)
		{
			printf("%.2f,%.2f\n",deg2rad(motor_pos_set),angle_Multi);
		}
		if(motor_mode==4)
		{
			printf("%.2f,%.2f,%.2f,%.2f\n",motor_spe_tor_set,motor_speed,motor_tor_spe_set*MAX_CURRENT,motor_i_q);
		}
		if(motor_mode==5)
		{
			printf("%.2f,%.2f,%.2f,%.2f\n",motor_tor_pos_set*MAX_CURRENT,motor_i_q,deg2rad(motor_pos_tor_set),angle_Multi);
		}
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//*********Callback*********////////////

////******//callback//******/////
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1)
  {
    AS5407P_CS_H
		as5047_rx_data=as5047_rx_data&0x03fff;
		angle=_2PI*as5047_rx_data/0x03fff;
	  if(angle_mon_flag==1)////记录初始位置
		{
			angle_start_mon=angle;
			angle_mon_flag=0;
		}
		float angle_deff=(float)as5047_rx_data-angle_mon;///记录多圈角度
		if(abs(angle_deff)>(0.8*16383))
		{
			angle_add+=(angle_deff>0?-_2PI:_2PI);
		}angle_mon=as5047_rx_data;
		angle_Multi=(angle_add+angle);

		AS5407P_CS_L
		HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)0xffff, (uint8_t *)&as5047_rx_data, 2);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM15)
	{
		static float angle_mon_last=0;
		static char once=1;
		if(once==1)
		{
			angle_mon_last=angle;
			once=0;
		}
		angle_diff = cycle_diff(angle - angle_mon_last, PI*2);
		angle_mon_last=angle;
		motor_speed=angle_diff/0.001f;	
		for(int i=1;i<lenth;i++)
		{
			data_mon[i-1]=data_mon[i];
		}data_mon[lenth-1]=motor_speed;
		for(int i=0; i<lenth; i++) 
		{
			lvbo_data[i] = data_mon[i];
		}
		// 使用冒泡排序
		for(int i=0; i<lenth-1; i++) {
				for(int j=0; j<lenth-i-1; j++) {
						if(lvbo_data[j] > lvbo_data[j+1]) {
								float swap = lvbo_data[j];
								lvbo_data[j] = lvbo_data[j+1];
								lvbo_data[j+1] = swap;
						}
				}
		}
		motor_speed=lvbo_data[lenth/2];
		motor_speed=Low_pass_filter(motor_speed,0.07);		
	}
}
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    u_1 = 3.3f * ((float)hadc->Instance->JDR1/ ((1 << 12) - 1) - 0.5);
    u_2 = 3.3f * ((float)hadc->Instance->JDR2 / ((1 << 12) - 1) - 0.5);
    float i_1= u_1 / 0.001f / 20;
    float i_2 = u_2 / 0.001f / 20;
	  motor_i_v= i_1;
    motor_i_u = i_2;
		float i_alpha = 0;
    float i_beta = 0;
    arm_clarke_f32(motor_i_u, motor_i_v, &i_alpha, &i_beta);
    float sin_value = arm_sin_f32(angle_Multi);
    float cos_value = arm_cos_f32(angle_Multi);
    arm_park_f32(i_alpha, i_beta, &motor_i_d, &motor_i_q, sin_value, cos_value);

    motor_i_d = Low_pass_filter(motor_i_d,0.1); 
    motor_i_q = Low_pass_filter(motor_i_q,0.1); 
		if(motor_mode==1)////力矩模式
		{
			lib_torque_control(motor_tor_set);
		}
		if(motor_mode==2)///速度模式
		{
			lib_speed_control(motor_speed_set);
		}
		if(motor_mode==3)///位置模式
		{
			lib_position_control(deg2rad(motor_pos_set));
		}
		if(motor_mode==4)///速度-力矩模式
		{
			lib_speed_torque_control(motor_spe_tor_set,motor_tor_spe_set);
		} 
		if(motor_mode==5)///位置-力矩模式
		{  
			lib_position_torque_control(deg2rad(motor_pos_tor_set),motor_tor_pos_set);
		}  
  }    
}      
       
/////串口-DMA//////////////////////////////////////////////////////////
void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)
	{
		for(unsigned char m=0;m<128;m++)
		{
			 RX_RCBuf[m]=RX_Buf[m];
			 if(RX_RCBuf[0]=='t'&&RX_RCBuf[1]=='o'&&RX_RCBuf[2]=='r')
			 {
				 motor_mode=1;
				 motor_tor_set=(RX_RCBuf[4]-'0')/10.0f+(RX_RCBuf[5]-'0')/100.f+(RX_RCBuf[6]-'0')/1000.0f;
			 }
			 if(RX_RCBuf[0]=='s'&&RX_RCBuf[1]=='p'&&RX_RCBuf[2]=='e')
			 {
				 motor_mode=2;
				 motor_speed_set=(RX_RCBuf[4]-'0')*100+(RX_RCBuf[5]-'0')*10+(RX_RCBuf[6]-'0');
			 }
			 if(RX_RCBuf[0]=='p'&&RX_RCBuf[1]=='o'&&RX_RCBuf[2]=='s')
			 {
				 motor_mode=3;
				 motor_pos_set=(RX_RCBuf[4]-'0')*100+(RX_RCBuf[5]-'0')*10+(RX_RCBuf[6]-'0');

			 }
			 if(RX_RCBuf[0]=='t'&&RX_RCBuf[1]=='a'&&RX_RCBuf[2]=='s')
			 {
				 motor_mode=4;
				 motor_spe_tor_set=(RX_RCBuf[4]-'0')*100+(RX_RCBuf[5]-'0')*10+(RX_RCBuf[6]-'0');
				 motor_tor_spe_set=(RX_RCBuf[8]-'0')/10.0f+(RX_RCBuf[9]-'0')/100.0f+(RX_RCBuf[10]-'0')/1000.0f;
			 }
			 if(RX_RCBuf[0]=='t'&&RX_RCBuf[1]=='a'&&RX_RCBuf[2]=='p')
			 {
				 motor_mode=5;
				 motor_pos_tor_set=(RX_RCBuf[4]-'0')*100+(RX_RCBuf[5]-'0')*10+(RX_RCBuf[6]-'0');
				 motor_tor_pos_set=(RX_RCBuf[8]-'0')/10.0f+(RX_RCBuf[9]-'0')/100.0f+(RX_RCBuf[10]-'0')/1000.0f;

			 }
		}
		memset(RX_Buf,0,sizeof(RX_Buf));//清空接收数组
		HAL_UART_Receive_DMA(&huart1,RX_Buf,128);		//启动DMA接收	
	}
}
void SVPWM_SET_OUT(float angle_el,float Uq,float Ud)
{
	SVPWM(angle_el, Ud, Uq, &D_U, &D_V, &D_W);
	limit_max(D_U,0.90);
	limit_max(D_V,0.90);
	limit_max(D_W,0.90);
	__HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_1 ,(int)(2125*D_U));
	__HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_2 ,(int)(2125*D_V));
	__HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_3 ,(int)(2125*D_W));
}
/////////位置环/////////////
static float position_loop(float rad)
{
    float diff = cycle_diff(rad -angle_Multi, position_cycle);
    return arm_pid_f32(&pid_position, diff);
}

void lib_position_control(float rad)
{
    float d = 0;
    float q = position_loop(rad);
    SVPWM_SET_OUT(angle_Multi*10,0,q);
}

/////////速度环////////////////

static float speed_loop(float speed_rad)
{
    float diff = speed_rad-motor_speed;
    return arm_pid_f32(&pid_speed, diff);
}
void lib_speed_control(float speed)
{
    float d = 0;
    float q = speed_loop(speed);
    SVPWM_SET_OUT(angle_Multi*10,0,q);
}
/////////力矩环////////////////
static float torque_q_loop(float q)
{
    float diff = q - motor_i_q/ MAX_CURRENT;
    return arm_pid_f32(&pid_torque_q, diff);
}

void lib_torque_control(float torque_norm_q)
{
    float q = torque_q_loop(torque_norm_q);
    SVPWM_SET_OUT(angle_Multi*10,0,q);
}
//串级PID 力矩-速度控制
void lib_speed_torque_control(float speed_rad, float max_torque_norm)
{
    float torque_norm = speed_loop(speed_rad);
    torque_norm = min(fabs(torque_norm), max_torque_norm) * (torque_norm > 0 ? 1 : -1);
    lib_torque_control(torque_norm);
}

//串级PID 力矩-位置控制
void lib_position_torque_control(float position_rad, float max_torque_norm)
{
    float torque_norm = position_loop(position_rad);
    torque_norm = min(fabs(torque_norm), max_torque_norm) * (torque_norm > 0 ? 1 : -1);
    lib_torque_control(torque_norm);
}






void lib_position_speed_torque_control(float position_rad, float max_speed_rad, float max_torque_norm)
{
    float speed_rad = position_loop(position_rad);
    speed_rad = min(fabs(speed_rad), max_speed_rad) * (speed_rad > 0 ? 1 : -1);
    lib_speed_torque_control(speed_rad, max_torque_norm);
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
