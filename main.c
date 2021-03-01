/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "VL53L0X.h"
#include "stdlib.h"
#include "robot.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef struct{
	int pin;
	uint8_t ON_L;
	uint8_t num;
	int pwm_goal;
	int pwm_adj;
	int clip_state;
//	int mid_state;
	int place_state;
	int open_arm;
	int close_arm;
//	int close_to_top_state;
//	uint32_t distance;
}servo_data;

typedef struct{
	I2C_HandleTypeDef *hi2c;
	uint8_t address;
	uint8_t led_off[64];
}pca;

typedef enum{
	pin_0 = 0x06,
	pin_1 = 0x0a,
	pin_2 = 0x0e,
	pin_3 = 0x12,
	pin_4 = 0x16,
	pin_5 = 0x1a,
	pin_6 = 0x1e,
	pin_7 = 0x22,
	pin_8 = 0x26,
	pin_9 = 0x2a,
	pin_10 = 0x2e,
	pin_11 = 0x32,
	pin_12 = 0x36,
	pin_13 = 0x3a,
	pin_14 = 0x3e,
	pin_15 = 0x42
}pin_address;

servo_data servo[] =
{
//	{.pin = 14, .ON_L = pin_14, .num = pin_14-6, .pwm_goal = 1500, .pwm_adj =  5, .max_state = 2070, .mid_state = 2000, .min_state = 1550, .close_to_top_state = 1605, .distance = 40},       // servo[0] --> raise : top(min), horizontal(mid), down(max)
	{.pin = 13, .ON_L = pin_13, .num = pin_13-6, .pwm_goal = 2210, .pwm_adj = 5, .open_arm = 1900, .close_arm = 2200},    // servo[0]   --> release : open, close
	{.pin = 6, .ON_L = pin_6, .num = pin_6-6, .pwm_goal = 2510, .pwm_adj = 5, .clip_state = 1550, .place_state = 2500},   // servo[1~5] --> catch : clip, place
	{.pin = 7, .ON_L = pin_7, .num = pin_7-6, .pwm_goal = 810, .pwm_adj = 5, .clip_state = 2000, .place_state = 800},
	{.pin = 8, .ON_L = pin_8, .num = pin_8-6,.pwm_goal = 810, .pwm_adj = 5, .clip_state = 1900, .place_state = 800},
	{.pin = 9, .ON_L = pin_9, .num = pin_9-6, .pwm_goal = 810, .pwm_adj = 5, .clip_state = 1800, .place_state = 800},
	{.pin = 10, .ON_L = pin_10, .num = pin_10-6, .pwm_goal = 2510, .pwm_adj = 5, .clip_state = 1600, .place_state = 2500},
//	{.pin = 4, .ON_L = pin_4, .num = pin_4-6, .pwm_goal = 2100, .pwm_adj = 10, .max_state = 2100, .min_state = 1150},  // servo[6~7] wind
//	{.pin = 4, .ON_L = pin_4, .num = pin_4-6, .pwm_goal = 2100, .pwm_adj = 10, .max_state = 2100, .min_state = 1150}
};

pca my_pca = {.hi2c = &hi2c1, .address = 0x80, .led_off = {0}};

struct ROBOT eurobot2021;

//int which = 0;
//VL53 vl53[] = { { .targetAddress = 0x61, .I2cLine = &hi2c2, .port = GPB, .mcpWhichPin = 7, .finalRangeSignalRate = 5 },
//			  { .targetAddress = 0x62, .I2cLine = &hi2c2, .port = GPB, .mcpWhichPin = 6, .finalRangeSignalRate = 5 },
//			  { .targetAddress = 0x63, .I2cLine = &hi2c2, .port = GPB, .mcpWhichPin = 5, .finalRangeSignalRate = 5 },
//			  { .targetAddress = 0x64, .I2cLine = &hi2c2, .port = GPB, .mcpWhichPin = 4, .finalRangeSignalRate = 5 }};
//
//mcp mcp23017[] = { { .address = 0b01000000, .IODIR[0] = 0, .IODIR[1] = 0, .hi2c = &hi2c2 }};
//
//int AmountOfVL53 = sizeof(vl53) / sizeof(VL53);
//int AmountOfMCP23017 = sizeof(mcp23017) / sizeof(mcp);

//int32_t originData[7] = {1,1,1,1,1,1};
int32_t receive[9] = {1,1,1,1,1,1,1,1,0};
//int32_t done[7] = {6,6,6,6,6,6,6};
//int32_t get_cup[5] = {0}; // 0 --> not get ; 1 --> get
int32_t transmit[9] = {0};  // 0 --> raise+flag ; 1 --> release ; 2 --> catch ; 3 --> vl53 distance value
int32_t stepper = 0;
int count = 1500;
int count_stepper = 0;
int temp = 0;

int countR;
int countT;
int a;
int p = 0;
int v = 0;
int reset_catch;
int stop = 0;
uint8_t pull_up = 255;
int switchread_up;
int switchread_down;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void sleep_mode_on(){
	uint8_t mode1_sleep_on = 17;
	HAL_I2C_Mem_Write(&hi2c1, 0x80, 0x00, I2C_MEMADD_SIZE_8BIT, &(mode1_sleep_on), 1, 1);
}

void sleep_mode_off_plus_ai(){
	uint8_t mode1_sleep_off_ai = 33;
	HAL_I2C_Mem_Write(&hi2c1, 0x80, 0x00, I2C_MEMADD_SIZE_8BIT, &(mode1_sleep_off_ai), 1, 1);
}

void setPrescale(){
	#define osc_clock 25990000
	#define updateRate 50
	uint8_t prescale_on;
	prescale_on = round(osc_clock/(4096*updateRate))-1;
	HAL_I2C_Mem_Write(&hi2c1, 0x80, 0xFE, I2C_MEMADD_SIZE_8BIT, &(prescale_on), 1, 1);
}

void pwmCal(servo_data *servo, pca *pca_1){
	double on_time;
	on_time = (double)((4096*(servo->pwm_goal))/20000);
	pca_1->led_off[servo->num+3] = (int)on_time/256;
	pca_1->led_off[servo->num+2] = (int)on_time%256;
	pca_1->led_off[servo->num+1] = 0;
	pca_1->led_off[servo->num] = 0;
}

int ouo = 600;
void pca_reset(pca *pca_1){
	HAL_I2C_Mem_Write(pca_1->hi2c, pca_1->address, 0x06, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&ouo, 64, 100);
}

void servo_turn(servo_data *servo, pca *pca_1){
	for(int i = 0; i < 8; i++){
		pwmCal(&servo[i], &my_pca);
	}
	HAL_I2C_Mem_Write_DMA(pca_1->hi2c, pca_1->address, 0x06, I2C_MEMADD_SIZE_8BIT, pca_1->led_off, 64);
}

//void originData_to_receive(int32_t *receive, int32_t *originData){
//	receive[0] = originData[0];
//	receive[1] = originData[1];
//	receive[2] = originData[2];
//	receive[3] = originData[3];
//	receive[4] = originData[4];
//	receive[5] = originData[5];
//	receive[6] = originData[6];
////	for(int i = 2; i < 7; i++){
////		receive[i] = (originData[2] >> (i*2-4)) % 4;
////	}
//}

//void done_to_transmit(int32_t *done, int32_t *transmit){
//	if(originData[0] == 0){
//		transmit[2] = reset_catch;
//	}
//	else{
//		transmit[2] = pow(4,4)*done[6]+pow(4,3)*done[5]+pow(4,2)*done[4]+pow(4,1)*done[3]+pow(4,0)*done[2];
//	}
//}

//void reset_mode(servo_data *servo, pca *pca_1){
//	if(receive[0] == 0){
//		if(servo[0].pwm_goal < servo[0].place_state){
//			transmit[0] = 5;
//			servo[0].pwm_goal += servo[0].pwm_adj;
//			servo_turn(servo, &my_pca);
//		}
//		else if(servo[0].pwm_goal > servo[0].place_state){
//			transmit[0] = 5;
//			servo[0].pwm_goal -= servo[0].pwm_adj;
//			servo_turn(servo, &my_pca);
//		}
//		else if(servo[0].pwm_goal == servo[0].place_state){
//			transmit[0] = 1;
//		}
//
//		if(servo[1].pwm_goal < servo[1].clip_state){
//			transmit[1] = 3;
//			servo[1].pwm_goal += servo[1].pwm_adj;
//			servo_turn(servo, &my_pca);
//		}
//		else if(servo[1].pwm_goal > servo[1].clip_state){
//			transmit[1] = 3;
//			servo[1].pwm_goal -= servo[1].pwm_adj;
//			servo_turn(servo, &my_pca);
//		}
//		else if(servo[1].pwm_goal == servo[1].clip_state){
//			transmit[1] = 1;
//		}
//
//		servo[2].pwm_goal = servo[2].min_state;
//		servo[3].pwm_goal = servo[3].min_state;
//		servo[4].pwm_goal = servo[4].min_state;
//		servo[5].pwm_goal = servo[5].min_state;
//		servo[6].pwm_goal = servo[6].min_state;
//		servo_turn(servo, &my_pca);
//		reset_catch = 341;
//	}
//}

//void raise(int32_t *receive, servo_data *servo, pca *pca_1){
//	/* raise: top --> 1, mid --> 2, down --> 3 */
//	if(receive[0] == 1){
//		if(servo[0].pwm_goal < servo[0].min_state){
//			transmit[0] = 5;
//			servo[0].pwm_goal += servo[0].pwm_adj;
//			servo_turn(servo, &my_pca);
//		}
//		else if(servo[0].pwm_goal > servo[0].min_state){
//			transmit[0] = 5;
//			servo[0].pwm_goal -= servo[0].pwm_adj;
//			servo_turn(servo, &my_pca);
//		}
//		else if(servo[0].pwm_goal == servo[0].min_state){
//			transmit[0] = receive[0];
//		}
//	}
//	else if(receive[0] == 2){
//		if(servo[0].pwm_goal < servo[0].mid_state){
//			transmit[0] = 5;
//			servo[0].pwm_goal += servo[0].pwm_adj;
//			servo_turn(servo, &my_pca);
//		}
//		else if(servo[0].pwm_goal > servo[0].mid_state){
//			transmit[0] = 5;
//			servo[0].pwm_goal -= servo[0].pwm_adj;
//			servo_turn(servo, &my_pca);
//		}
//		else if(servo[0].pwm_goal == servo[0].mid_state){
//			transmit[0] = receive[0];
//		}
//	}
//	else if(receive[0] == 3){
//		if(servo[0].pwm_goal < servo[0].max_state){
//			transmit[0] = 5;
//			servo[0].pwm_goal += servo[0].pwm_adj;
//			servo_turn(servo, &my_pca);
//		}
//		else if(servo[0].pwm_goal > servo[0].max_state){
//			transmit[0] = 5;
//			servo[0].pwm_goal -= servo[0].pwm_adj;
//			servo_turn(servo, &my_pca);
//		}
//		else if(servo[0].pwm_goal == servo[0].max_state){
//			transmit[0] = receive[0];
//		}
//	}
//	else if(receive[0] == 4){
//		if(servo[0].pwm_goal < servo[0].close_to_top_state){
//			transmit[0] = 5;
//			servo[0].pwm_goal += servo[0].pwm_adj;
//			servo_turn(servo, &my_pca);
//		}
//		else if(servo[0].pwm_goal > servo[0].close_to_top_state){
//			transmit[0] = 5;
//			servo[0].pwm_goal -= servo[0].pwm_adj;
//			servo_turn(servo, &my_pca);
//		}
//		else if(servo[0].pwm_goal == servo[0].close_to_top_state){
//			transmit[0] = receive[0];
//		}
//	}
//}

void release(int32_t *receive, servo_data *servo, pca *pca_0){
	/* reduce --> 1, extend --> 2, windsock up --> 3, windsock down --> 4 */
	if(receive[0] == 1){
		if(servo[0].pwm_goal < servo[0].close_arm){
			transmit[0] = 5;
			servo[0].pwm_goal += servo[0].pwm_adj;
			servo_turn(servo, &my_pca);
		}
		else if(servo[0].pwm_goal > servo[0].close_arm){
			transmit[0] = 5;
			servo[0].pwm_goal -= servo[0].pwm_adj;
			servo_turn(servo, &my_pca);
		}
		else if(servo[0].pwm_goal == servo[0].close_arm){
			transmit[0] = receive[0];
			HAL_I2C_Mem_Write(pca_0->hi2c, pca_0->address, 0x3a, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&ouo, 4, 100);
		}
	}
	else if(receive[0] == 2){
		if(servo[0].pwm_goal < servo[0].open_arm){
			transmit[0] = 5;
			servo[0].pwm_goal += servo[0].pwm_adj;
			servo_turn(servo, &my_pca);
		}
		else if(servo[0].pwm_goal > servo[0].open_arm){
			transmit[0] = 5;
			servo[0].pwm_goal -= servo[0].pwm_adj;
			servo_turn(servo, &my_pca);
		}
		else if(servo[0].pwm_goal == servo[0].open_arm){
			transmit[0] = receive[0];
//			servo_turn(servo, &my_pca);
			HAL_I2C_Mem_Write(pca_0->hi2c, pca_0->address, 0x3a, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&ouo, 4, 100);
		}
	}
}

void catch(int32_t *receive, servo_data *servo, pca *pca_1){
	/* catch: grab --> 0, put --> 1 */
	for(int i = 1; i < 6; i++){
		if(receive[i] == 0){
			if(servo[i].pwm_goal < servo[i].clip_state){
				transmit[i] = 3;
				servo[i].pwm_goal += servo[i].pwm_adj;
				servo_turn(servo, &my_pca);
			}
			else if(servo[i].pwm_goal > servo[i].clip_state){
				transmit[i] = 3;
				servo[i].pwm_goal -= servo[i].pwm_adj;
				servo_turn(servo, &my_pca);
			}
			else if(servo[i].pwm_goal == servo[i].clip_state){
				transmit[i] = receive[i];
//				servo_turn(servo, &my_pca);
				HAL_I2C_Mem_Write(pca_1->hi2c, pca_1->address, servo[i].ON_L, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&ouo, 4, 100);
			}
		}
		else if(receive[i] == 1){
			if(servo[i].pwm_goal < servo[i].place_state){
				transmit[i] = 3;
				servo[i].pwm_goal += servo[i].pwm_adj;
				servo_turn(servo, &my_pca);
			}
			else if(servo[i].pwm_goal > servo[i].place_state){
				transmit[i] = 3;
				servo[i].pwm_goal -= servo[i].pwm_adj;
				servo_turn(servo, &my_pca);
			}
			else if(servo[i].pwm_goal == servo[i].place_state){
				transmit[i] = receive[i];
//				servo_turn(servo, &my_pca);
				HAL_I2C_Mem_Write(pca_1->hi2c, pca_1->address, servo[i].ON_L, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&ouo, 4, 100);
			}
		}
	}
}

//void vl53_state(VL53 *vl53){
//	for(int i = 0; i < 4; i++){
//		if(vl53[i].result.VL53RangingResult <= 600){
//			stop++;
//		}
//		else if(vl53[i].result.VL53RangingResult > 600){
//			stop = 0;
//		}
//	}
//	if(stop > 0){
//		transmit[3] = 1;
//	}
//	else if(stop == 0){
//		transmit[3] = 0;
//	}
//}
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim14){
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
			temp++;
		}
	}
}

int qaq = 0;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim6) // 200Hz
	{
		Set_Transmit_Uart(&eurobot2021.hardware.uartRPI, transmit);
		Uart_Transmit(&eurobot2021.hardware.uartRPI);
//		Uart_Rate_Count(&eurobot2019.hardware.uartRPI);
	}

    if(htim->Instance==TIM2){
    	qaq++;
//		if (which >= AmountOfVL53) {
//			which = 0;
//			//when you finish a cycle turn back to first VL53
//		}
//		VL53Ranging_DMA(&(vl53[which]));
////		//this function will get a unprocessed data
////		// get your final data by  VL53L0X_Ranging_Result_Decipher
//		vl53_state(vl53);
    	release(receive, servo, &my_pca);
    	catch(receive, servo, &my_pca);
    }

    if(htim->Instance == TIM3){
    	switchread_up = HAL_GPIO_ReadPin(UP_GPIO_Port , UP_Pin);
    	switchread_down = HAL_GPIO_ReadPin(DOWN_GPIO_Port , DOWN_Pin);

    	if(stepper == 1){
    		count_stepper ++;
    		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
			HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
			__HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,htim14.Instance->ARR/2);
			if(switchread_down == 0){
				__HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,0);
				HAL_TIM_PWM_Stop(&htim14,TIM_CHANNEL_1);
				transmit[8] = stepper;
			}
			if(count_stepper >= 215){
    			__HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,0);
    			HAL_TIM_PWM_Stop(&htim14,TIM_CHANNEL_1);
    			transmit[8] = stepper;
    		}
    	}
    	else if(stepper == 2){
    		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    		HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
    		__HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,htim14.Instance->ARR/2);
    		if(switchread_up == 0){
    			__HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,0);
    			HAL_TIM_PWM_Stop(&htim14,TIM_CHANNEL_1);
    			transmit[8] = stepper;
    			count_stepper = 0;
    		}
    	}
    	else if(stepper == 3){
			HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
			HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
			__HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,htim14.Instance->ARR/2);
			if(switchread_down == 0){
				__HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,0);
				HAL_TIM_PWM_Stop(&htim14,TIM_CHANNEL_1);
				transmit[8] = stepper;
			}
		}
    }
    if(htim->Instance == TIM4){
    	if(v < p){
			v = p;
		}
		else if(v == p){
			eurobot2021.hardware.uartRPI.stuck_count ++;
		}
		if(eurobot2021.hardware.uartRPI.stuck_count > 10){
			HAL_UART_AbortReceive(eurobot2021.hardware.uartRPI.huart);
			eurobot2021.hardware.uartRPI.stuck_count = 0;
			eurobot2021.hardware.uartRPI.one_or_block = 0;
			HAL_UART_Receive_DMA(eurobot2021.hardware.uartRPI.huart, (uint8_t *)eurobot2021.hardware.uartRPI.rx_single, 4);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1){
		Uart_RxCplt(huart, &eurobot2021.hardware.uartRPI);
		Set_Command_Uart(receive, eurobot2021.hardware.uartRPI);
	}
}

void Set_Transmit_Uart(struct UART *uart, int32_t *transmit)
{
	uart->tx[0] = 0x32; //ST1 = 0x31 (1), ST2 = 0x32 (2)
	for(int len = 1; len < uart->tx_length; len++){
		uart->tx[len] = transmit[len-1];
	}
	if(uart->tx[uart->tx_length + 1] == 255){
		HAL_UART_Receive_DMA(uart->huart, (uint8_t *)uart->rx_single, 4);
		uart->tx[uart->tx_length + 1] = 0;
	}
}

void Set_Command_Uart(int32_t *receive, struct UART uart)
{
	for(int len = 0; len < uart.rx_length; len++){
		receive[len] = uart.checked_rx[len];
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
	if(hi2c->Instance == I2C1){
//		vl53[which].VL53_RxCplt++;
//		vl53[which].result.VL53RangingResult = VL53L0X_Ranging_Result_Decipher(
//				&(vl53[which]));
////		this function will tell you the final result of the ranging process
//		which++;
////		which VL53L0X are you calling since you have a lot of VL53
		countR++;
	}
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c){
	if(hi2c->Instance==I2C1){
		countT++;
	}
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_CRC_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  sleep_mode_on();
  setPrescale();
  sleep_mode_off_plus_ai();

//  HAL_TIM_PWM_Start(&htim13,TIM_CHANNEL_1);
//  HAL_TIM_PWM_Stop(&htim13,TIM_CHANNEL_1);
//  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
//  __HAL_TIM_SetCompare(&htim13,TIM_CHANNEL_1,5000);

//  pca_reset(&my_pca);
//  HAL_Delay(1000);
//  servo[0].pwm_goal = 200;
//  servo_turn(servo, &my_pca);

//  Hardware_Init(&eurobot2019.hardware);
//
//  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
//  HAL_Delay(10);
//  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);

//  reset mcp
//  for (int count = 0; count < AmountOfMCP23017; count++)
//	  mcp_init(&(mcp23017[count]));
//  VL53VarInit( vl53, mcp23017);

//  HAL_I2C_Mem_Write(&hi2c2, 0x40, 0x0D, I2C_MEMADD_SIZE_8BIT, &(pull_up), 1, 1); //pull up register for switch
//  HAL_I2C_Mem_Write(&hi2c2, 0x40, 0x13, I2C_MEMADD_SIZE_8BIT, &(pull_up), 1, 1); //read switch
//  HAL_Delay(10);


//  for(int i = 0; i <= 1000; i++){
//	  transmit[0] += i;
//	  transmit[1] += i;
//	  transmit[2] += i;
//	  transmit[3] += i;
//  }
//  	  transmit[0] = 8;
  Hardware_Init(&eurobot2021.hardware);


  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//	  if(count < 10000){
//		  servo[1].pwm_goal = count;
//		  servo_turn(servo, &my_pca);
//		  servo[3].pwm_goal = count;
//		  servo_turn(servo, &my_pca);
//		  servo[4].pwm_goal = count;
//		  servo_turn(servo, &my_pca);
//		  servo[5].pwm_goal = count;
//		  servo_turn(servo, &my_pca);
//		  servo[6].pwm_goal = count;
//		  servo_turn(servo, &my_pca);
//		  HAL_Delay(50);
//		  count++;
//	  }

//	  servo_turn(servo, &my_pca);
//	  HAL_I2C_Mem_Write(&hi2c2, 0x40, 0x12, I2C_MEMADD_SIZE_8BIT, &(pull_up), 1, 1); //read switch
//	  HAL_Delay(10);
//	  pull_up = 0;
//	  HAL_Delay(10000);
//	  HAL_I2C_Mem_Write(&hi2c2, 0x40, 0x12, I2C_MEMADD_SIZE_8BIT, &(pull_up), 1, 1); //read switch
//	  HAL_Delay(10);
//	  pull_up = 255;

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
