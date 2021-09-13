/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#define CENTRO_JOYSTICK       210 //este valor es independiente a la funcion, es para iniciar correctamente la silla y detener movimiento
#define CENTRO_JOYSTICK_FUNCTION  194
#define MAX_SPEED_DIGITAL_VALUE  167
#define MIN_SPEED_DIGITAL_VALUE  97  //97
#define MAX_SPEED_DIGITAL_VALUE_X  255
#define MIN_SPEED_DIGITAL_VALUE_X  160  //179
#define DELAY_MS_BOTONES      150
#define UP_BUTTON_PIN     GPIO_PIN_7
#define DOWN_BUTTON_PIN   GPIO_PIN_6
#define DELAY_SILLA_STOP  2

#define UART_DATA_SIZE 13

#define UART_HEADER_LENGHT 4

#define VALID_DATA_START 4
#define VALID_DATA_LENGTH 5

#define UART_COUNTER_POS 8

#define HEADER_VALID_VALUE 160

#define DELAY_COUNTDOWN 5 //delay de si no recibo el comando cerrar app

#define ERROR_COUNTDOWN_TO_SHUTDOWN 4

#define INIT_SPEED 3

#define Y_AXIS_CORRECTION_THRESHOLD 135

#define X_AXIS_CORRECTION_MAGNITUD 3
enum{
	POWER_ON,
	POWER_OFF,
	POWER_PULSE,
	DOWN_BUTTON,
	UP_BUTTON,
};

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void ResetLeds(void);
void LightLeds(void);
void ToggleLeds(void);
void TurnOnLeds(void);
void centerJoystick(void);
void clean_data_uart(void);
void lower_speed(void);
void initial_speed(void);
void init_conditions(void);
void turn_on_off_pulse(void);
void lower_pulse(void);
void rise_pulse(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint8_t data_uart[UART_DATA_SIZE];
uint8_t xcoord=CENTRO_JOYSTICK;
uint8_t ycoord=CENTRO_JOYSTICK;
uint8_t xmax=0;
uint8_t ymax=0;
uint8_t received_done=0;
uint8_t send_done=0;
uint8_t keep_moving=0;
uint8_t timer_counter=0;
uint8_t timer_countdown = DELAY_COUNTDOWN; //delay de si no recibo el comando cerrar app
uint8_t timer_countdown_activate=0;
uint8_t running_program=0;
uint8_t chair_speed=0;
uint8_t send_data[UART_DATA_SIZE];
uint8_t resta_velocidad_recibida_velocidad_actual=0;
uint8_t timer_movement_activate=0;
uint8_t is_on = 0;
uint8_t command_check =0;
uint8_t command_check_count=0;
uint8_t ack_recived =0;
uint32_t wait_ack_counter =0;
uint8_t uart_counter = 255;
uint8_t error_counter_to_shutdown = 0;
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	float m, n;
	
	uint8_t desviacion = 255 - CENTRO_JOYSTICK_FUNCTION ;
	
  clean_data_uart();
  xcoord=CENTRO_JOYSTICK;
  ycoord=CENTRO_JOYSTICK;
	xmax=0;
  ymax=0;
  received_done=0;
	
	
	
  m = (float)(desviacion)/(MAX_SPEED_DIGITAL_VALUE - 127);
	
	n = CENTRO_JOYSTICK_FUNCTION - (127 * m); 
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
	//HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
	
	//HAL_Delay(1000);
	//TurnOnLeds();
	
			//pines de botones en 1
	
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
	//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,DOWN_BUTTON_PIN,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,UP_BUTTON_PIN,GPIO_PIN_SET);
  
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	
	centerJoystick();
	
	HAL_Delay(500);
	
	turn_on_off_pulse();
	
	HAL_TIM_Base_Start_IT(&htim2); //timer por IT a 2seg
	
		//HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
		

	//HAL_UART_Receive_DMA(&huart2, &data_uart[0], UART_DATA_SIZE);  //dma no funciona no se porque
	
//	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		//start:
		
    HAL_UART_Receive_IT(&huart2, &data_uart[0], UART_DATA_SIZE);
		//HAL_UART_Receive_DMA(&huart2, &data_uart[0], UART_DATA_SIZE); //dma no funciona no se porque
		
		if(running_program==1 &&  is_on == 1 && timer_countdown==0){  // Cuenta atras de comunicacion si no se recibe el comando de comunicacion en este tiempo se apaga la silla
				
				running_program=0;
				timer_countdown_activate=0;
			  //timer_countdown=0;
				
				turn_on_off_pulse();
				
			  init_conditions();
			  //NVIC_SystemReset();
			  continue;
			
		}
		
		if (received_done==1){
			
			received_done=0;
			
			if(data_uart[VALID_DATA_START]==170 && data_uart[VALID_DATA_START+1]==170 && data_uart[VALID_DATA_START+2]==170 && data_uart[VALID_DATA_START+3]==170 && running_program==0){  //aplicacion corriendo en Raspi y no recibio mensaje de inicio de aplicacion
				
					uint16_t i=0;
		      for(i=0; i< UART_HEADER_LENGHT; i++){
			 
		        data_uart[i]=HEADER_VALID_VALUE;

		      }
					data_uart[VALID_DATA_START]=0xDD;
				  data_uart[VALID_DATA_START+1]=0xDD;
				  data_uart[VALID_DATA_START+2]=0xDD;
				  data_uart[VALID_DATA_START+3]=0xDD;
					data_uart[UART_COUNTER_POS]= 0xDD;
				
				  for(i=VALID_DATA_START+VALID_DATA_LENGTH; i< UART_DATA_SIZE; i++){
			 
		        data_uart[VALID_DATA_START+VALID_DATA_LENGTH + i]=HEADER_VALID_VALUE;

		      }

				
				HAL_UART_Transmit(&huart2, &data_uart[0], UART_DATA_SIZE,50);
				
				init_conditions();
				
				continue;
				//HAL_Delay(1000);
				
				//NVIC_SystemReset();
			}
			
			if(running_program==0 && data_uart[VALID_DATA_START]==204 && data_uart[VALID_DATA_START+1]==204 && data_uart[VALID_DATA_START+2]==204 && data_uart[VALID_DATA_START+3]==204){  //pregunto si recibio el comando de inicio de programa
			
			   running_program = 1;
				 timer_countdown_activate=1;
				 timer_countdown = DELAY_COUNTDOWN;
			}
		  if(running_program==0){  // si no lo recibo termino paso del ciclo y repite hasta que se reciba
			
			  continue;
				
			}
			
			
			if(data_uart[VALID_DATA_START]==77 && data_uart[VALID_DATA_START+1]==77){  // aqui recibo señal de soltado el joystick detengo movimiento apago leds
			 
				centerJoystick();
				keep_moving=0;
			  ResetLeds();
				//HAL_TIM_Base_Stop_IT(&htim2);
				timer_movement_activate = 0;
				
			  uint16_t i=0;
		    for(i=0; i< UART_HEADER_LENGHT; i++){
			 
		      data_uart[i]=HEADER_VALID_VALUE;

		    }
				data_uart[VALID_DATA_START]=83;
		    data_uart[VALID_DATA_START+1]=83;
		    data_uart[VALID_DATA_START+2]=83;
		    data_uart[VALID_DATA_START+3]=83;
				data_uart[UART_COUNTER_POS]= 0;
				
				for(i=VALID_DATA_START+VALID_DATA_LENGTH; i< UART_DATA_SIZE; i++){
			 
		      data_uart[VALID_DATA_START+VALID_DATA_LENGTH + i]=HEADER_VALID_VALUE;

		    }
				//}	
		    HAL_UART_Transmit(&huart2, &data_uart[0], UART_DATA_SIZE,50);
				//clean_data_uart();	
				
	    }
			
	    if(data_uart[VALID_DATA_START]==1){
				
//				  while(ack_recived !=1){
//					
//						wait_ack_counter++;
//						
//						if(wait_ack_counter >= 500){
//							wait_ack_counter = 0;
//						  goto start;
//						}
//					}
//					wait_ack_counter = 0;
//					ack_recived=0;
					
				  if(data_uart[VALID_DATA_START+1] < MIN_SPEED_DIGITAL_VALUE){
					  xcoord = MIN_SPEED_DIGITAL_VALUE_X;
					}
				  else if(data_uart[VALID_DATA_START+1] > MAX_SPEED_DIGITAL_VALUE){ //Aseguroque xcoord no sobrepase los 255
					  xcoord = MAX_SPEED_DIGITAL_VALUE_X;
				  }			
				  else{
	          //xcoord = (uint8_t)(data_uart[VALID_DATA_START+1]*0.9486 + 96.0516);//sobrepasa el 255 para altos valores//de esta forma cubre el rango reversa //original
						//xcoord = (uint8_t)(data_uart[VALID_DATA_START+1]*0.5028 + 171.03);//sobrepasa el 255 para altos valores
						//xcoord = (uint8_t)(data_uart[VALID_DATA_START+1]*0.4791 +  174.9903);//sobrepasa el 255 para altos valores// aqui la velocidad hacia la derecha max es la misma que izquierda max
						if(data_uart[VALID_DATA_START+3] > Y_AXIS_CORRECTION_THRESHOLD){
						
							xcoord -= (uint8_t)((float)(((uint8_t)(xcoord - 127))/X_AXIS_CORRECTION_MAGNITUD));
						}
						xcoord = (uint8_t)(data_uart[VALID_DATA_START+1]*m + n);
						
				  }
				  //xcoord = (uint8_t)(data_uart[1]*0.6245 + 96.38);
				  ////sobrepasa el 255 para altos valores
					if(data_uart[VALID_DATA_START+3] < 85){
					  ycoord = 166;
					}
				  else if(data_uart[VALID_DATA_START+3] > MAX_SPEED_DIGITAL_VALUE){  //Aseguroque ycoord no sobrepase los 255
				  	ycoord = 255;
				  }
				  else{
				    //ycoord = (uint8_t)(data_uart[3]*0.6269 + 96.38);
				  	//ycoord = (uint8_t)(data_uart[VALID_DATA_START+3]*0.9522 + 96.0516);//sobrepasa el 255 para altos valores//de esta forma cubre el rango reversa  //original
						//ycoord = (uint8_t)(data_uart[VALID_DATA_START+3]*0.5028 + 171.03);//sobrepasa el 255 para altos valores // aqui la velocidad hacia atras max es la misma que adelante max
						//ycoord = (uint8_t)(data_uart[VALID_DATA_START+3]*0.4791 + 174.9903);//sobrepasa el 255 para altos valores // aqui la velocidad hacia atras max es la misma que adelante max
						//ycoord = (uint8_t)(data_uart[VALID_DATA_START+3]*0.9522 + 96.0516); //217 medio
						ycoord = (uint8_t)(data_uart[VALID_DATA_START+3]*m + n);//213 medio
				  }
				
				  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, xcoord);
				  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_8B_R, ycoord);
				  
				  timer_countdown=DELAY_COUNTDOWN;
			   				
	    }

			if(data_uart[VALID_DATA_START]==170 && data_uart[VALID_DATA_START+1]==170 && data_uart[VALID_DATA_START+2]==170 && data_uart[VALID_DATA_START+3]==170){  // aqui recibo cada segundo comando de comunicacion y reinicio contador
			 
				timer_countdown=DELAY_COUNTDOWN;	
				
	    }
	 
			if(data_uart[VALID_DATA_START]==69 && data_uart[VALID_DATA_START+1]==69 && data_uart[VALID_DATA_START+2]==69 && data_uart[VALID_DATA_START+3]==69){  // aqui recibo cada segundo comando de comunicacion y reinicio contador
			 
				if(is_on == 1){
				  turn_on_off_pulse();
				}
				
				//NVIC_SystemReset();				
	    }
			
		  if(data_uart[VALID_DATA_START]==72 && data_uart[VALID_DATA_START+2]==72 && data_uart[VALID_DATA_START+3]==72){ //Aumentar************************************************************************
						
				  chair_speed = data_uart[VALID_DATA_START+1];
				
				  rise_pulse();	 
				
	    }
		 
		  if(data_uart[VALID_DATA_START]==76 && data_uart[VALID_DATA_START+2]==76 && data_uart[VALID_DATA_START+3]==76){ //disminuir*******************************************************************
					
				   chair_speed = data_uart[VALID_DATA_START+1];
				
				   lower_pulse();		
	    }
		 
		  if(data_uart[VALID_DATA_START]==84){ //encendido
				
				//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_3);
				
				turn_on_off_pulse();
				 
				if(data_uart[VALID_DATA_START+1]==79 && data_uart[VALID_DATA_START+2]==70 && data_uart[VALID_DATA_START+3]==70){ //aqui apaga
				  is_on=0;
				}
				if(data_uart[VALID_DATA_START+1]==0 && data_uart[VALID_DATA_START+2]==79 && data_uart[VALID_DATA_START+3]==78){ // aqui enciende
				  is_on=1;
					lower_speed(); // baja la velocidad al minimo al inicio del programa
					initial_speed(); //sube la velocidad hasta la velocidad inicial seleccionada
				}
				//clean_data_uart();	
	    }
		 
//		  if(data_uart[VALID_DATA_START]==67){ //claxon
//			 
//				if(data_uart[VALID_DATA_START+1]==100){
//					
//				  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
//					//clean_data_uart();	
//				}
//				
//			  if(data_uart[VALID_DATA_START+1]==97){
//					
//			    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET); 
//					//clean_data_uart();	
//				}
//	    }
			
			if(data_uart[VALID_DATA_START]==75){ // keep moving  # K   //aqui recibo señal de mantener movimiento y reinicio timer
			 
				if(data_uart[VALID_DATA_START+1]==69 && data_uart[VALID_DATA_START+2]==69 && data_uart[VALID_DATA_START+3]==80){  // keep moving  # EEP
					
				  keep_moving=1; 
					timer_counter=0;
					//clean_data_uart();	
				}
	    }
			if(data_uart[VALID_DATA_START]==77){ // MOVE # M  //inicio de movimiento de joystick// aqui inicia el timer
			 
				if(data_uart[VALID_DATA_START+1]==79 && data_uart[VALID_DATA_START+2]==86 && data_uart[VALID_DATA_START+3]==69){  // MOVE  # OVE
					
				  //HAL_TIM_Base_Start_IT(&htim2);
					timer_counter=0;
					timer_movement_activate = 1;
					
					//HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
          //clean_data_uart();					
				}
	    }
			
			//clean_data_uart();
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

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

    /**DAC channel OUT2 config 
    */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16800;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	
	
	 if(data_uart[0]==HEADER_VALID_VALUE && data_uart[1]==HEADER_VALID_VALUE && data_uart[2]==HEADER_VALID_VALUE && data_uart[3]==HEADER_VALID_VALUE){  
			 
			if(data_uart[VALID_DATA_START+VALID_DATA_LENGTH]==HEADER_VALID_VALUE && data_uart[VALID_DATA_START+VALID_DATA_LENGTH+1]==HEADER_VALID_VALUE && data_uart[VALID_DATA_START+VALID_DATA_LENGTH+2]==HEADER_VALID_VALUE && data_uart[VALID_DATA_START+VALID_DATA_LENGTH+3]==HEADER_VALID_VALUE){
			
				received_done=1;
				
				if(data_uart[UART_COUNTER_POS] == uart_counter){
				
						centerJoystick();
	          error_counter_to_shutdown++;
					
					  if(error_counter_to_shutdown >= ERROR_COUNTDOWN_TO_SHUTDOWN){
						
							Error_Handler();
						}
				}
				uart_counter = data_uart[UART_COUNTER_POS];	
        data_uart[UART_COUNTER_POS] = 0;				
			}
		
	 }
	
   HAL_UART_Transmit(&huart2, &data_uart[0], UART_DATA_SIZE, 50);
}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	
//}
	

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){  // se desborda 2seg y mando a parar la silla y apago leds

  ToggleLeds();
	
	if(timer_movement_activate == 1){
		
	  timer_counter++;
	}
	
	if(timer_countdown_activate == 1){
		
	  timer_countdown--;
	}
	
//	if(timer_counter >= DELAY_SILLA_STOP){
//		
//	   keep_moving=0;
//	   centerJoystick();
//	   ResetLeds();
//		 timer_counter = 0;
//		 timer_movement_activate = 0;
//		
//		 uint16_t i=0;
//		 for(i=0; i< UART_HEADER_LENGHT; i++){
//			 
//		   data_uart[i]=HEADER_VALID_VALUE;

//		 }

//		 data_uart[VALID_DATA_START]=83;
//		 data_uart[VALID_DATA_START+1]=83;
//		 data_uart[VALID_DATA_START+2]=83;
//		 data_uart[VALID_DATA_START+3]=83;
//		 data_uart[UART_COUNTER_POS]=0;
//		
//		 
//		 for(i=VALID_DATA_START+VALID_DATA_LENGTH; i< UART_DATA_SIZE; i++){
//			 
//		   data_uart[VALID_DATA_START+VALID_DATA_LENGTH + i]=HEADER_VALID_VALUE;

//		 }
//				//}	
//		 HAL_UART_Transmit(&huart2, &data_uart[0], UART_DATA_SIZE,50);
//		
//	}

}

void init_conditions(){
	
	clean_data_uart();
  xcoord=CENTRO_JOYSTICK;
  ycoord=CENTRO_JOYSTICK;
	xmax=0;
  ymax=0;
  received_done=0;
  send_done=0;
  keep_moving=0;
  timer_counter=0;
  timer_countdown=DELAY_COUNTDOWN;
  timer_countdown_activate=0;
  running_program=0;
  timer_movement_activate=0;
  is_on = 0;
	command_check=0;
	command_check_count=0;
	ack_recived = 0;

}

void centerJoystick(){

	xcoord = CENTRO_JOYSTICK;
	ycoord = CENTRO_JOYSTICK;
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, CENTRO_JOYSTICK); //centro 2.5V,2.5V
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_8B_R, CENTRO_JOYSTICK);
	
}


void initial_speed(){
	
  for(uint8_t i=0; i< INIT_SPEED -1; i++){
		
    HAL_GPIO_WritePin(GPIOA, UP_BUTTON_PIN ,GPIO_PIN_RESET);
		HAL_Delay(DELAY_MS_BOTONES);
		HAL_GPIO_WritePin(GPIOA, UP_BUTTON_PIN,GPIO_PIN_SET);
		HAL_Delay(DELAY_MS_BOTONES);
	}
	timer_countdown=DELAY_COUNTDOWN;
}

void lower_speed(){
	
  for(uint8_t i=0; i< 5; i++){
		
    HAL_GPIO_WritePin(GPIOA, DOWN_BUTTON_PIN ,GPIO_PIN_RESET);
		HAL_Delay(DELAY_MS_BOTONES);
		HAL_GPIO_WritePin(GPIOA, DOWN_BUTTON_PIN,GPIO_PIN_SET);
		HAL_Delay(DELAY_MS_BOTONES);
	}
	timer_countdown=DELAY_COUNTDOWN;
}


void ResetLeds(){
 
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);

}

void ToggleLeds(){

  HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);
	HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);
	HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);
	HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14);
	
}

void LightLeds(){

	if(xcoord > CENTRO_JOYSTICK){   //debe ser 217 - 2.5V al la salida del DAC medio del joystick
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);
	}
  else{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
	}

  if(ycoord > CENTRO_JOYSTICK){
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
	}
  else{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
	}
	
}
 
void TurnOnLeds(){

	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
	
}

void clean_data_uart(){

	uint16_t i=0;
  for(i=0;i< UART_DATA_SIZE; i++){
	
		data_uart[i]=0;
	}

}

void lower_pulse(void){

            HAL_GPIO_WritePin(GPIOA, DOWN_BUTTON_PIN, GPIO_PIN_RESET);
			      HAL_Delay(DELAY_MS_BOTONES);
			      HAL_GPIO_WritePin(GPIOA, DOWN_BUTTON_PIN, GPIO_PIN_SET);
					  HAL_Delay(DELAY_MS_BOTONES);
}

void rise_pulse(void){

            HAL_GPIO_WritePin(GPIOA, UP_BUTTON_PIN ,GPIO_PIN_RESET);
			      HAL_Delay(DELAY_MS_BOTONES);
			      HAL_GPIO_WritePin(GPIOA, UP_BUTTON_PIN ,GPIO_PIN_SET);
					  HAL_Delay(DELAY_MS_BOTONES);
}

void turn_on_off_pulse(void){

        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
			  HAL_Delay(DELAY_MS_BOTONES);
			  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
	      HAL_Delay(DELAY_MS_BOTONES);
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  centerJoystick();
}
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
	
	centerJoystick();
	
	data_uart[VALID_DATA_START]=111;
	data_uart[VALID_DATA_START+1]=111;
	data_uart[VALID_DATA_START+2]=111;
	data_uart[VALID_DATA_START+3]=111;
	data_uart[UART_COUNTER_POS]=111;
	
	HAL_UART_Transmit(&huart2, &data_uart[0], UART_DATA_SIZE, 50);
	
	NVIC_SystemReset();
	
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
