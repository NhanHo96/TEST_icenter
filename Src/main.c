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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#define DS3231_ADD 0x68
#define Len_rtc 21
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t Tx_data[5];
uint8_t cnt;
uint16_t timer;
typedef struct UARTRS232
{
	char Rx_indx;
	char Rx_data[1];
	char	Rx_Buffer[100];
	char	Transfer_cplt;
	char rtc_relay_uart[59];
} Uart2rs232;
Uart2rs232 uart2rs232;

typedef struct RTC_1
{
	uint8_t send_rtc[7];
	uint8_t read_rtc[7];
	uint8_t second,minute,hour,day,date,month,year;
	char rtc1_uart[22];
} RTC1;
RTC1 rtc1;

typedef struct RTC_2
{
	uint8_t giay,phut,gio,thu,ngay,thang,nam;
	RTC_TimeTypeDef sTime;
  RTC_DateTypeDef DateToUpdate;
	char rtc2_uart[21];
}RTC2;
RTC2 rtc2;

typedef struct set_time
{
	uint8_t H1_bat;
	uint8_t H1_tat;
	uint8_t M1_bat;
	uint8_t M1_tat;
	
	uint8_t H2_bat;
	uint8_t H2_tat;
	uint8_t M2_bat;
	uint8_t M2_tat;
	
	uint8_t set_ngay;
	uint8_t set_thang;
	uint8_t set_nam;
	uint8_t set_gio;
	uint8_t set_phut;
	uint8_t set_giay;
	uint8_t tam_set_time;
	uint8_t send_data_rtc[7];
}TIME;
TIME time;

typedef struct check_relay
{
	uint8_t check_1;
	uint8_t check_2;
	uint8_t check_3;
	uint8_t cnt_relay1;
	uint8_t cnt_relay2;
	uint8_t cnt_relay3;
	uint8_t relay_echo1;
	uint8_t relay_echo2;
	uint8_t relay_echo3;
	uint8_t relay_echo4;
	uint8_t count_relay1;
	uint8_t count_relay2;
	uint8_t count_relay3;
	uint8_t status_relay1;
	uint8_t status_relay2;
	uint8_t status_relay3;
	uint8_t status_relay4;
	uint8_t tam_relay1;
	uint8_t tam_relay2;
	uint8_t tam_relay3;
	char relay_echo[11];
}RELAY;
RELAY relay;
typedef struct RS485
{
		uint8_t trans_1;
		uint8_t trans_2;
}RS485;
RS485 rs485;
uint8_t a;
uint8_t cnt_relay1;
uint8_t cnt_relay2;
uint8_t cnt_relay3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void ONOFF_relay(void);
uint8_t BCD2DEC(uint8_t data);
uint8_t DEC2BCD(uint8_t data);
void read_rtc1(void);
void read_rtc2(void);
void transmit_RTC_relayecho(void);
void ONOFF_relay_time(void);
void check_setting_Time(void);
void count_relay(void);
void relay_10_times(void);
void relay_echo(void);
void set_time(void);
void set_rtc1(void);
void status_relay(void);
void renew_tam_relay(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

//-------------Receive UART----------------------------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  
{
    if (huart->Instance == huart1.Instance)                                                               //current UART
		{
				if (uart2rs232.Rx_indx==0) 
				{
						for (uint8_t i=0;i<100;i++) uart2rs232.Rx_Buffer[i]=0;                                        //clear Rx_Buffer before receiving new data 
				}   

				if (uart2rs232.Rx_data[0]!='\n')                                                                  //if received data different from ascii 13 (enter)
				{
						uart2rs232.Rx_Buffer[uart2rs232.Rx_indx++]=uart2rs232.Rx_data[0];                            //add data to Rx_Buffer
				}
				else                                                                                             //if received data = 13
				{
						uart2rs232.Rx_indx=0;
						HAL_UART_Transmit_IT(&huart1,(uint8_t *)uart2rs232.Rx_Buffer,sizeof(uart2rs232.Rx_Buffer));
				}   
				HAL_UART_Receive_IT(&huart1,(uint8_t *)uart2rs232.Rx_data, 1);                                    //activate UART receive interrupt every time		
		}
		
		if(huart->Instance==huart4.Instance)
		{
				if (uart2rs232.Rx_indx==0) 
				{
						for (uint8_t i=0;i<100;i++) uart2rs232.Rx_Buffer[i]=0;                                        //clear Rx_Buffer before receiving new data 
				}   

				if (uart2rs232.Rx_data[0]!='\n')                                                                  //if received data different from ascii 13 (enter)
				{
						uart2rs232.Rx_Buffer[uart2rs232.Rx_indx++]=uart2rs232.Rx_data[0];                             //add data to Rx_Buffer
				}
				else                                                                                              //if received data = 13
				{
						uart2rs232.Rx_indx=0;
						HAL_UART_Transmit_IT(&huart4,(uint8_t *)uart2rs232.Rx_Buffer,sizeof(uart2rs232.Rx_Buffer));
				}   
				HAL_UART_Receive_IT(&huart4,(uint8_t *)uart2rs232.Rx_data, 1);                                    //activate UART receive interrupt every time
		}
		
		if(huart->Instance==huart2.Instance)
		{
			
				if (uart2rs232.Rx_indx==0) 
				{
						for (uint8_t i=0;i<100;i++) uart2rs232.Rx_Buffer[i]=0;                                        //clear Rx_Buffer before receiving new data 
				}   

				if (uart2rs232.Rx_data[0]!='\n')                                                                  //if received data different from ascii 13 (enter)
				{
						uart2rs232.Rx_Buffer[uart2rs232.Rx_indx++]=uart2rs232.Rx_data[0];                             //add data to Rx_Buffer
				}
				else                                                                                              //if received data = 13
				{
						uart2rs232.Rx_indx=0;
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);
						HAL_UART_Transmit_IT(&huart2,(uint8_t *)uart2rs232.Rx_Buffer,sizeof(uart2rs232.Rx_Buffer));
				}   
				HAL_UART_Receive_IT(&huart2,(uint8_t *)uart2rs232.Rx_data, 1);                                    //activate UART receive interrupt every time
		}
		
		if(huart->Instance==huart3.Instance)
		{
				if (uart2rs232.Rx_indx==0) 
				{
						for (uint8_t i=0;i<100;i++) uart2rs232.Rx_Buffer[i]=0;                                        //clear Rx_Buffer before receiving new data 
				}   

				if (uart2rs232.Rx_data[0]!='\n')                                                                  //if received data different from ascii 13 (enter)
				{
						uart2rs232.Rx_Buffer[uart2rs232.Rx_indx++]=uart2rs232.Rx_data[0];                             //add data to Rx_Buffer
				}
				else                                                                                              //if received data = 13
				{
						uart2rs232.Rx_indx=0;
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET);
						HAL_UART_Transmit_IT(&huart3,(uint8_t *)uart2rs232.Rx_Buffer,sizeof(uart2rs232.Rx_Buffer));
				}
				HAL_UART_Receive_IT(&huart3,(uint8_t *)uart2rs232.Rx_data, 1);                                    //activate UART receive interrupt every time
		}
}
//--------TX RS485------------------------------------------------------------------------
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
		if(huart->Instance==huart2.Instance)
		{
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);
		}
		if(huart->Instance==huart3.Instance)
		{
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_RESET);
		}
}
//--------------check relay-------------------------------------------------------------------
void ONOFF_relay(void)
{
		if(uart2rs232.Rx_Buffer[1]=='\r')
		{
				switch(uart2rs232.Rx_Buffer[0])
				{
						case '<':
								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET);
							break;
						case '>':
								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_RESET);
							break;
						case '?':
								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
							break;
						case '/':
								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET);
							break;
						case ';':
								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
							break;
						case ':':
								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);
							break;
						case '~':
								relay.check_1++;
							for (uint8_t i=0;i<100;i++) uart2rs232.Rx_Buffer[i]=0;    
						break;
						case '!':
								relay.check_2++;
								for (uint8_t i=0;i<100;i++) uart2rs232.Rx_Buffer[i]=0;    
						break;
						case '@':
								relay.check_3++;
								for (uint8_t i=0;i<100;i++) uart2rs232.Rx_Buffer[i]=0;    
						break;
						default: break;
				}
		}
}

void relay_10_times(void)
{
		if(relay.check_1!=0)
		{
				HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_7);
				relay.cnt_relay1++;
				if(relay.cnt_relay1==10)
				{
						relay.cnt_relay1=0;
						relay.check_1--;
				}
		}
		
		if(relay.check_2!=0)
		{
				HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_8);
				relay.cnt_relay2++;
				if(relay.cnt_relay2==10)
				{
						relay.cnt_relay2=0;
						relay.check_2--;
				}
		}
		
		if(relay.check_3!=0)
		{
				HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_9);
				relay.cnt_relay3++;
				if(relay.cnt_relay3==10)
				{
						relay.cnt_relay3=0;
						relay.check_3--;
				}
		}
		cnt++;
		HAL_Delay(500);
}
void ONOFF_relay_time(void)
{
		check_setting_Time();
		if(time.H1_bat!=time.H1_tat || time.M1_bat!=time.M1_tat)
		{
				if(rtc1.hour==time.H1_bat && rtc1.minute==time.M1_bat)
				{
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET);
				}
				else if(rtc1.hour==time.H1_tat && rtc1.minute == time.M1_tat)
				{
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_RESET);
				}
		}
		
		if(time.H2_bat!=time.H2_tat || time.M2_bat!=time.M2_tat)
		{
				if(rtc2.gio==time.H2_bat && rtc2.phut==time.M2_bat)
				{
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
				}
				else if(rtc2.gio==time.H2_tat && rtc2.phut == time.M2_tat)
				{
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET);
				}
		}
}

void relay_echo(void)
{
		if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_13)==0)
		{
				relay.relay_echo1=1;
		}
		else relay.relay_echo1=0;
		
		if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_14)==0)
		{
				relay.relay_echo2=1;
		}
		else relay.relay_echo2=0;
		
		if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_15)==0)
		{
				relay.relay_echo3=1;
		}
		else relay.relay_echo3=0;
		
		if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_0)==0)
		{
				relay.relay_echo4=1;
		}
		else relay.relay_echo4=0;
}

void status_relay(void)
{
		if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_14)==1)
		{	
						relay.status_relay1=0;		
		}
		if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_14)==0)
		{
				relay.status_relay1=1;
		}
		
		if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_15)==1)
		{
				relay.status_relay2=0;
		}
		if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_15)==0)
		{
				relay.status_relay2=1;
		}
		
		if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_0)==1)
		{
				relay.status_relay3=0;
		}
		if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_0)==0)
		{
				relay.status_relay3=1;
		}
		if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_13)==1)
		{
				relay.status_relay4=0;
		}
		if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_13)==0)
		{
				relay.status_relay4=1;
		}
}
void count_relay(void)
{
		if(relay.status_relay1!=relay.tam_relay1)
		{
				relay.count_relay1++;
		}
		if(relay.status_relay2!=relay.tam_relay2)
		{
				relay.count_relay2++;
		}
		if(relay.status_relay3!=relay.tam_relay3)
		{
				relay.count_relay3++;
		}
}
void renew_tam_relay(void)
{
		relay.tam_relay1=relay.status_relay1;
		relay.tam_relay2=relay.status_relay2;
		relay.tam_relay3=relay.status_relay3;
}
//--------TIMER 1-----------------------------------------------------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		if(htim->Instance == htim2.Instance)
		{
				transmit_RTC_relayecho();
		}
		if(htim->Instance == htim1.Instance)
		{
				cnt++;
		}
}
//----------RTC------------------------------------------------------------
uint8_t BCD2DEC(uint8_t data)
{
		return (data>>4)*10 + (data&0x0f);
}
uint8_t DEC2BCD(uint8_t data)
{
		return (data/10)<<4|(data%10);
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
		if(hi2c->Instance==hi2c1.Instance)
		{
				rtc1.second=BCD2DEC(rtc1.read_rtc[0]);
				rtc1.minute=BCD2DEC(rtc1.read_rtc[1]);
				rtc1.hour=BCD2DEC(rtc1.read_rtc[2]);
				rtc1.day=BCD2DEC(rtc1.read_rtc[3]);
				rtc1.date=BCD2DEC(rtc1.read_rtc[4]);
				rtc1.month=BCD2DEC(rtc1.read_rtc[5]);
				rtc1.year=BCD2DEC(rtc1.read_rtc[6]);
		}
}
void read_rtc1(void)
{
		HAL_I2C_Mem_Read_DMA(&hi2c1,DS3231_ADD<<1,0,I2C_MEMADD_SIZE_8BIT,rtc1.read_rtc,7);
}

void read_rtc2(void)
{
		HAL_RTC_GetTime(&hrtc,&rtc2.sTime,RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc,&rtc2.DateToUpdate,RTC_FORMAT_BIN);
		rtc2.giay=rtc2.sTime.Seconds;
		rtc2.phut=rtc2.sTime.Minutes;
		rtc2.gio=rtc2.sTime.Hours;
		rtc2.thu=rtc2.DateToUpdate.WeekDay;
		rtc2.ngay=rtc2.DateToUpdate.Date;
		rtc2.thang=rtc2.DateToUpdate.Month;
		rtc2.nam=rtc2.DateToUpdate.Year;
}
void transmit_RTC_relayecho(void)
{
		relay_echo();
		sprintf(uart2rs232.rtc_relay_uart,"T,%02d,%02d,%02d,%02d,%02d,%02d,%02d,%02d,%02d,%02d,%02d,%02d,%d,%d,%d,%d,%03d,%03d,%03d,*",
						rtc1.date,rtc1.month,rtc1.year,rtc1.hour,rtc1.minute,rtc1.second,rtc2.ngay,rtc2.thang,rtc2.nam,rtc2.gio,rtc2.phut,rtc2.giay,
						relay.relay_echo1,relay.relay_echo2,relay.relay_echo3,relay.relay_echo4,
						relay.count_relay1,relay.count_relay2,relay.count_relay3);
		HAL_UART_Transmit_IT(&huart1,(uint8_t *)uart2rs232.rtc_relay_uart,59);
}
//--------SETTING TIME---------------------------------------------------------------
void check_setting_Time(void)
{
		if(uart2rs232.Rx_Buffer[0]=='S' && uart2rs232.Rx_Buffer[1]=='1' && uart2rs232.Rx_Buffer[14]=='\r')
		{
				time.H1_bat	=(uart2rs232.Rx_Buffer[3]-48)*10 + (uart2rs232.Rx_Buffer[4]-48);
				time.M1_bat	=(uart2rs232.Rx_Buffer[6]-48)*10 + (uart2rs232.Rx_Buffer[7]-48);
				time.H1_tat	=(uart2rs232.Rx_Buffer[9]-48)*10 + (uart2rs232.Rx_Buffer[10]-48);
				time.M1_tat	=(uart2rs232.Rx_Buffer[12]-48)*10 + (uart2rs232.Rx_Buffer[13]-48);
		}
		if(uart2rs232.Rx_Buffer[0]=='S' && uart2rs232.Rx_Buffer[1]=='2' && uart2rs232.Rx_Buffer[14]=='\r')
		{
				time.H2_bat	=(uart2rs232.Rx_Buffer[3]-48)*10 + (uart2rs232.Rx_Buffer[4]-48);
				time.M2_bat	=(uart2rs232.Rx_Buffer[6]-48)*10 + (uart2rs232.Rx_Buffer[7]-48);
				time.H2_tat	=(uart2rs232.Rx_Buffer[9]-48)*10 + (uart2rs232.Rx_Buffer[10]-48);
				time.M2_tat	=(uart2rs232.Rx_Buffer[12]-48)*10 + (uart2rs232.Rx_Buffer[13]-48);
		}
}
void set_rtc1(void)
{
		time.send_data_rtc[0]=DEC2BCD(time.set_giay);
		time.send_data_rtc[1]=DEC2BCD(time.set_phut);
		time.send_data_rtc[2]=DEC2BCD(time.set_gio);
		time.send_data_rtc[3]=DEC2BCD(2);
		time.send_data_rtc[4]=DEC2BCD(time.set_ngay);
		time.send_data_rtc[5]=DEC2BCD(time.set_thang);
		time.send_data_rtc[6]=DEC2BCD(time.set_nam);
		HAL_I2C_Mem_Write_IT(&hi2c1,DS3231_ADD<<1,0,I2C_MEMADD_SIZE_8BIT,time.send_data_rtc,7);
}
void set_time(void)
{
		if(uart2rs232.Rx_Buffer[0]=='S' && uart2rs232.Rx_Buffer[1]=='T' && uart2rs232.Rx_Buffer[20]=='\r')
		{
				time.set_ngay		=(uart2rs232.Rx_Buffer[3]-48)*10 + (uart2rs232.Rx_Buffer[4]-48);
				time.set_thang	=(uart2rs232.Rx_Buffer[6]-48)*10 + (uart2rs232.Rx_Buffer[7]-48);
				time.set_nam		=(uart2rs232.Rx_Buffer[9]-48)*10 + (uart2rs232.Rx_Buffer[10]-48);
				time.set_gio		=(uart2rs232.Rx_Buffer[12]-48)*10 + (uart2rs232.Rx_Buffer[13]-48);
				time.set_phut		=(uart2rs232.Rx_Buffer[15]-48)*10 + (uart2rs232.Rx_Buffer[16]-48);
				time.set_giay		=(uart2rs232.Rx_Buffer[18]-48)*10 + (uart2rs232.Rx_Buffer[19]-48);
				time.tam_set_time=1;
				for (uint8_t i=0;i<100;i++) uart2rs232.Rx_Buffer[i]=0;
		}
		if(time.tam_set_time==1)
		{
				MX_RTC_Init();
				set_rtc1();
				time.tam_set_time=0;
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_TC);
	__HAL_UART_ENABLE_IT(&huart4,UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart4,UART_IT_TC);
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_TC);
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_TC);
	HAL_UART_Receive_IT(&huart1,(uint8_t *)uart2rs232.Rx_data, 1);
	HAL_UART_Transmit_IT(&huart1,(uint8_t *)uart2rs232.Rx_Buffer,sizeof(uart2rs232.Rx_Buffer));
	HAL_UART_Receive_IT(&huart4,(uint8_t *)uart2rs232.Rx_data, 1);
	HAL_UART_Transmit_IT(&huart4,(uint8_t *)uart2rs232.Rx_Buffer,sizeof(uart2rs232.Rx_Buffer));
	HAL_UART_Receive_IT(&huart2,(uint8_t *)uart2rs232.Rx_data, 1);
	HAL_UART_Transmit_IT(&huart2,(uint8_t *)uart2rs232.Rx_Buffer,sizeof(uart2rs232.Rx_Buffer));
	HAL_UART_Receive_IT(&huart3,(uint8_t *)uart2rs232.Rx_data, 1);
	HAL_UART_Transmit_IT(&huart3,(uint8_t *)uart2rs232.Rx_Buffer,sizeof(uart2rs232.Rx_Buffer));
	HAL_TIM_Base_Start_IT(&htim2);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
			read_rtc1();
			read_rtc2();
			set_time();
		
			ONOFF_relay();
			relay_10_times();
			ONOFF_relay_time();
			status_relay();
			count_relay();		
			renew_tam_relay();
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);

}

/* RTC init function */
void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef DateToUpdate;
  RTC_AlarmTypeDef sAlarm;

    /**Initialize RTC and set the Time and Date 
    */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  HAL_RTC_Init(&hrtc);

  sTime.Hours = time.set_gio;
  sTime.Minutes = time.set_phut;
  sTime.Seconds = time.set_giay;

  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

  DateToUpdate.WeekDay = RTC_WEEKDAY_THURSDAY;
  DateToUpdate.Month = time.set_thang;
  DateToUpdate.Date = time.set_ngay;
  DateToUpdate.Year = time.set_nam;

  HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);

    /**Enable the Alarm A 
    */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.Alarm = RTC_ALARM_A;
  HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BIN);

}

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 36000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim1);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 36000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

}

/* UART4 init function */
void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart4);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}

/* USART3 init function */
void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart3);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE4 PE5 PE7 PE8 
                           PE9 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PF13 PF14 PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PG0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
