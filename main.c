/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bldcv2.h" 
typedef enum FSMSTATE {A , B , C , D , E , F , Idle,Nop} statetype;
statetype state = Idle;
statetype buf_state = Idle;
typedef enum FSMSTATE1 {Openloop,Closeloop,Init,Fault,Idle1}statustype;
statustype Status = Init;
unsigned int Timer=0;
unsigned int FlagFrequencyinOpenloop=0,add,add1;
unsigned int Timer1ms;
unsigned int SetCom=1000;//870
unsigned int ControlSetCom=1000;//870
unsigned int Cm_time=0;
float DifTime,SetTimer=0;
int IdleCount=0;
int cnt=0;
uint8_t TEST=0;
uint16_t ADCValue;
float Voltage;
uint8_t Start=1;
uint8_t FirstTime=0;
int Ic_valHB1=0;
int Ic_valHB2=0;
uint8_t SpeedB;
uint8_t IsFirstCaptureB=0;
int DifferenceB=0,Speed_B,Speed_BB,Speed_BBB;
int DifferenceA=0;
int DifferenceC=0;
int DifferenceSpeed=0;
int FrequencyB=0;
int Ic_valHA1=0;
int Ic_valHA2=0;
uint8_t IsFirstCaptureA=0;
int FrequencyA=0,Speed_A,Speed_AA,Speed_AAA;
int Ic_valHC1=0;
int Ic_valHC2=0;
uint8_t IsFirstCaptureC=0;
int FrequencyC=0,Speed_C,Speed_CC,Speed_CCC;
 uint8_t Firstinit=1 ;
int Ic_valSpeed1=0;
int Ic_valSpeed2=0;
uint8_t IsFirstCaptureSpeed=0;
uint32_t FrequencySpeed=0;
uint8_t A1=0,B1=0,C1=0; 
uint8_t StartCm=0;
uint8_t Guest=0;
uint8_t Guest1=0;
uint8_t  PhaseState=0;
uint32_t CaptureTimer=0;
uint32_t ticks=0;
uint32_t CaptureTimerSpeed=0;
uint32_t TickSpeed=0;
uint8_t FDelay=0;
uint8_t FirstSwitch=1;
uint8_t Flag=0,A10=0,B10=0,C10=0;
unsigned int timStart=0;
uint32_t Rpm,cntt,f,cnttt,BuferFrequencA,firstTimeSetTimer14;
int DutyCycle=0; 
uint8_t OpenloopCase=0;
uint8_t  BuferHA=0;
uint8_t BuferHB=0;
uint8_t BuferHC=0;
uint8_t  BuferHA1=0;
uint8_t BuferHB1=0;
uint8_t BuferHC1=0;
uint32_t i=0;
uint8_t ReadPin[36]={0x00,0x0,0x0,0x0};
uint8_t ReadFlag=0;
uint8_t PhaseStateBuffer=0;
uint8_t Forward=0;
uint8_t Reverse=0;
uint8_t countChooseState=0;
uint8_t CounterSamestet1=0;
uint8_t CounterSamestet2=0;
uint8_t CounterSamestet3=0;
uint8_t CounterSamestet4=0;
uint8_t CounterSamestet5=0;
uint8_t CounterSamestet6=0;
uint8_t countStopState=0;
uint8_t CountR =0;
uint8_t CountF =0;
extern unsigned char CounterInitial;
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
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */
void StateA(unsigned int Duty);
void StateB(unsigned int Duty);
void StateC(unsigned int Duty);
void StateD(unsigned int Duty);
void StateE(unsigned int Duty);
void StateF(unsigned int Duty);
void PwmDuty( int SignalDuty);
void SwitchPhase(uint8_t PhaseStateCloseLoop );


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//READ INPUTCAPTURE FROM TIMER 1 & TIMER3
{	
if(htim -> Instance == TIM1 )
	{
////////////////////READ FREQUENCY PHASE B////////////////////////////////////////////////////////////////////////////////////
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
			{
				Ic_valHB1 = __HAL_TIM_GET_COMPARE(&htim1,TIM_CHANNEL_2);
				DifferenceB=Ic_valHB1-Ic_valHB2;
				Ic_valHB2=Ic_valHB1;
				if (DifferenceB<0)
					DifferenceB+=35000;
				if(DifferenceB>=90 && DifferenceB<=15001)
					{
						FrequencyB=15000/DifferenceB;
					}
			}
///////////////////////READ FREQUENCY PHASE A/////////////////////////////////////////////////////////////////////////////////
			 if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
			{
				Ic_valHA1 = __HAL_TIM_GET_COMPARE(&htim1,TIM_CHANNEL_3);
				DifferenceA=Ic_valHA1-Ic_valHA2;
				Ic_valHA2=Ic_valHA1;
				if (DifferenceA<0)
					DifferenceA+=35000;
				if(DifferenceA>=90 && DifferenceA<=15001)
					{
						FrequencyA=15000/DifferenceA;
					}
			}
		}
		if(htim -> Instance == TIM3 )
			{
////////////////////////READ SPEED SIGNALS FOR CHANGE DUTY CYCLE PWM 25KHZ///////////////////////////////////////////////////
				if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
					{
						if(IsFirstCaptureSpeed==0 && SpeedSignal==1)
							{
								Ic_valSpeed1 = TIM3->CCR2;// __HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_2);
								IsFirstCaptureSpeed=1;
							}
							else if(IsFirstCaptureSpeed==1)
								{
									Ic_valSpeed2  =TIM3->CCR2;// __HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_2);
									IsFirstCaptureSpeed=0;
								}
						if(Ic_valSpeed2>Ic_valSpeed1 )
							DifferenceSpeed=Ic_valSpeed2-Ic_valSpeed1;
						if(DifferenceSpeed<0)
							{
								DifferenceSpeed+=36000;
							}
							DutyCycle=(DifferenceSpeed/3)+1;
					}
///////////////////////READ FREQUENCY PHASE C/////////////////////////////////////////////////////////////////////////////////
					if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
						{
							Ic_valHC1 = __HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_1);
							DifferenceC=Ic_valHC1-Ic_valHC2;
							Ic_valHC2=Ic_valHC1;
							if (DifferenceC<0)
								DifferenceC+=35000;
							if(DifferenceC>=90 && DifferenceC<=15001)
								{
									FrequencyC=15000/DifferenceC;
								}
						}
			}
}	

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim -> Instance == TIM14)
	{
	if (state!=Idle && state!=buf_state)
		{
			buf_state=state;
			state = Idle;
		}
				switch(state)
		{
			case Idle:
			SwitchT1_off();
			SwitchT4_off();
			SwitchT2_off();
		  SwitchT3_off();
		  SwitchT5_off();
		  SwitchT6_off();
		break;
			case A:
				if(FirstSwitch==0)
				{
					SwitchT1_off();
					SwitchT4_off();
					FirstSwitch=1;
				}
		break;
			case B:
			SwitchT1_off();
		  SwitchT6_off();
		break;
			case C:
		  SwitchT3_off();
		  SwitchT6_off();
		break;
			case D:
			SwitchT2_off();
		  SwitchT3_off();
		break;
			case E:
			SwitchT2_off();
		  SwitchT5_off();
		break;
			case F:
			SwitchT4_off();
		  SwitchT5_off();
		break;
			case Nop:
		break;
		}
}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	
	if(htim -> Instance == TIM14)
	{
			
			if (state!=Idle && state!=buf_state)
	{
		buf_state=state;
		state = Idle;
	}
		switch(state)
		{
			case Idle:
			SwitchT1_off();
			SwitchT4_off();
			SwitchT2_off();
		  SwitchT3_off();
		  SwitchT5_off();
		  SwitchT6_off();
			IdleCount++;
			if (IdleCount>=1)
			{
				state=buf_state;
				IdleCount=0;
			}
		break;
			case A:
			if(FirstSwitch==1)	
				{
				FirstSwitch=0;
				SwitchT1_on();
				SwitchT4_on();
				}
		break;
			case B:
			SwitchT1_on();
		  SwitchT6_on();
		break;
			case C:
		  SwitchT3_on();
		  SwitchT6_on();
		break;
			case D:
		  SwitchT3_on();
		  SwitchT2_on();
		break;
			case E:
			SwitchT2_on();
		  SwitchT5_on();
		break;
			case F:
			SwitchT4_on();
		  SwitchT5_on();
		break;
			case Nop:
		break;
		}

	}
		if(htim -> Instance == TIM17)
		{

//			Cm_time=SetTimer-1;
			Cm_time=SetTimer;
			Timer++;
			switch(state)
				{
				case A:
					Led_on();
				if(Timer>Cm_time)
					{
						Timer=0;
						state=B;
					}
					break;
				case B:
					Led_off();
				if(Timer>Cm_time)
					{
						Timer=0;
						state=C;
					}	
					break;
				case C:
					Led_on();
				if(Timer>Cm_time)
					{
						Timer=0;
						state=D;
					}
					break;
				case D:
					Led_off();
				if(Timer>Cm_time)
					{
						Timer=0;
						state=E;			
					}
					break;
				case E:
					Led_on();
				if(Timer>Cm_time)
					{
						Timer=0;
						state=F;
					}
					break;
				case F:
					Led_off();
				if(Timer>Cm_time)
					{
						Timer=0;
						state=A;
					}
					break;
				case Idle:
					break;
				case Nop:
					break;
				}
	}
		if(htim -> Instance == TIM16)
		{	
if(FlagFrequencyinOpenloop==0)			
{	
	Timer1ms++;
					if(Timer1ms<1000)
						DifTime=0;
					else
						DifTime=((SetTimer*SetTimer)*0.0000015);
					
					SetTimer=SetTimer-DifTime;
					SetCom=1050+10000/SetTimer+350000/(SetTimer*SetTimer);// 100 hz 1400 20000 150000 70%
					__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1,SetCom);
					if(SetTimer<832)//69// 83 50hz ok//138 30hz not ok//76 ok 55 hz//73  57hz//208
						{
							SetTimer=832;//832
							Timer1ms=0;
							OpenloopCase=1;
							FlagFrequencyinOpenloop=0;
						}
		}
	}
		if(htim -> Instance == TIM1)
		{
			CaptureTimer++;
		}
		if(htim -> Instance == TIM3)
		{
			 CaptureTimerSpeed++;
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
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
	SetTimer=50000;
	//HAL_Delay(3000);
	//HAL_ADC_Start(&hadc);
//HAL_ADC_Start_IT(&hadc);

//HAL_ADC_Start(&hadc);

HAL_TIM_Base_Start_IT(&htim1);
HAL_TIM_Base_Start_IT(&htim3);
HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_2);
HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_3);
HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);
 
	Start=0;
	FirstTime=0;
	FirstSwitch=1;
	timStart=0;
	IsFirstCaptureSpeed=0;
	Status = Init;	
  BuferHA=0;
	BuferHB=0;
	BuferHC=0;
	i=1;
			HAL_Delay(200);
			Forward=0;
			Reverse=0;
		 CounterInitial=0;
//HAL_TIM_Base_Start_IT(&htim14);
//HAL_TIM_PWM_Start_IT(&htim14, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//	while(1)
//	{
//		   	BuferHA=HA;
//		    BuferHB=HB;
//		    BuferHC=HC;

//	}
  while (1)
  {	
		switch(Status)
			{
			case Openloop:
				switch(OpenloopCase)
				{
					case 0:		
						if(Firstinit==1)
							{		
								//add1=10;
								HAL_TIM_Base_Start_IT(&htim14);
								HAL_TIM_PWM_Start_IT(&htim14, TIM_CHANNEL_1);
								state=A;
								HAL_TIM_Base_Start_IT(&htim16);
						    HAL_TIM_Base_Start_IT(&htim17);							
						    Firstinit=2;
					    	Timer1ms=0;
							}
						break;
					case 1:
						HAL_TIM_Base_Stop_IT(&htim16); 
						HAL_TIM_Base_Stop_IT(&htim17);
					  buf_state=Idle;
					  state=Idle;	
					  Status=Closeloop;
						Firstinit=1;
						break;
				}	
					break;
			case Closeloop:	
			if(firstTimeSetTimer14==0)
				{		
					 firstTimeSetTimer14=1;
			     HAL_TIM_Base_Start_IT(&htim14);
					 HAL_TIM_PWM_Start_IT(&htim14, TIM_CHANNEL_1);
				 __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1,SetCom);
				}
				///////	Speed_A=(Speed_A*90+ FrequencyA*10)/100;  LPF
				PwmDuty(DutyCycle);
				PhaseState=(HA<<2|HB<<1|HC<<0);
	      SwitchPhase(PhaseState);
				break;
			case Init:
				if(ReadFlag==0)
				{
					BuferHA=(HA*4);
					BuferHB=(HB*2);
					BuferHC=(HC*1);
					PhaseState=(BuferHA|BuferHB|BuferHC);
					ReadFlag=1;
					countChooseState++;
				}
				
				switch(PhaseState)
					{
					case 0X03:
						if(ReadFlag==1)
						{
							BuferHA=(HA*4);
							BuferHB=(HB*2);
				    	BuferHC=(HC*1);
				    	PhaseStateBuffer=(BuferHA|BuferHB|BuferHC);
							ReadFlag=2;
						}
						if(PhaseStateBuffer==PhaseState)
						{
							ReadFlag=1;
							CounterSamestet1++;
						}
						else if(PhaseStateBuffer==0x02)
							{
								Reverse++;
								ReadFlag=1;
								PhaseState=PhaseStateBuffer;
								
							}
							else if(PhaseStateBuffer==0x01)
							{
								Forward++;
								ReadFlag=1;
								PhaseState=PhaseStateBuffer;
								
							}
							else
							{
								ReadFlag=0;
							}											
					break;
		    	case 0x01:
						if(ReadFlag==1)
						{
							BuferHA=(HA*4);
							BuferHB=(HB*2);
				    	BuferHC=(HC*1);
				    	PhaseStateBuffer=(BuferHA|BuferHB|BuferHC);
							ReadFlag=2;
						}
						if(PhaseStateBuffer==PhaseState)
						{
							ReadFlag=1;
							CounterSamestet2++;
						}
							else if(PhaseStateBuffer==0x03)
							{
								Reverse++;
								ReadFlag=1;
								PhaseState=PhaseStateBuffer;
								
							}
							else if(PhaseStateBuffer==0x05)
							{
								Forward++;
								ReadFlag=1;
								PhaseState=PhaseStateBuffer;
							}
							else
							{
								ReadFlag=0;
							}		
					    	
				  break;
			    case 0x05:
						if(ReadFlag==1)
						{
							BuferHA=(HA*4);
							BuferHB=(HB*2);
				    	BuferHC=(HC*1);
				    	PhaseStateBuffer=(BuferHA|BuferHB|BuferHC);
							ReadFlag=2;
						}
						if(PhaseStateBuffer==PhaseState)
						{
							ReadFlag=1;
							CounterSamestet3++;
						}
							else if(PhaseStateBuffer==0x01)
							{
								Reverse++;
								ReadFlag=1;
								PhaseState=PhaseStateBuffer;
								
							}
							else if(PhaseStateBuffer==0x04)
							{
								Forward++;
								ReadFlag=1;
								PhaseState=PhaseStateBuffer;
							}
							else
							{
								ReadFlag=0;
							}
				  break;
			    case 0x04:
							if(ReadFlag==1)
						{
							BuferHA=(HA*4);
							BuferHB=(HB*2);
				    	BuferHC=(HC*1);
				    	PhaseStateBuffer=(BuferHA|BuferHB|BuferHC);
							ReadFlag=2;
						}
						if(PhaseStateBuffer==PhaseState)
						{
							ReadFlag=1;
							CounterSamestet4++;
						}
							else if(PhaseStateBuffer==0x05)
							{
								Reverse++;
								ReadFlag=1;
								PhaseState=PhaseStateBuffer;
								
							}
							else if(PhaseStateBuffer==0x06)
							{
								
								Forward++;
								ReadFlag=1;
								PhaseState=PhaseStateBuffer;
							}
							else
							{
								ReadFlag=0;
							}					      
				  break;
		   	  case 0x06:
						if(ReadFlag==1)
						{
							BuferHA=(HA*4);
							BuferHB=(HB*2);
				    	BuferHC=(HC*1);
				    	PhaseStateBuffer=(BuferHA|BuferHB|BuferHC);
							ReadFlag=2;
						}
						if(PhaseStateBuffer==PhaseState)
						{
							ReadFlag=1;
							CounterSamestet5++;
						}
							else if(PhaseStateBuffer==0x04)
							{
								Reverse++;	
								ReadFlag=1;
								PhaseState=PhaseStateBuffer;
								
							}
							else if(PhaseStateBuffer==0x02)
							{
								Forward++;
								ReadFlag=1;
								PhaseState=PhaseStateBuffer;
								
							}
							else
							{
								ReadFlag=0;
							}								
			 	  break;
			    case 0x02:
						if(ReadFlag==1)
						{
							BuferHA=(HA*4);
							BuferHB=(HB*2);
				    	BuferHC=(HC*1);
				    	PhaseStateBuffer=(BuferHA|BuferHB|BuferHC);
							ReadFlag=2;
						}
						if(PhaseStateBuffer==PhaseState)
						{
							ReadFlag=1;
							CounterSamestet6++;
						}
							else if(PhaseStateBuffer==0x06)
							{
								Reverse++;	
								ReadFlag=1;
								PhaseState=PhaseStateBuffer;
								
							}
							else if(PhaseStateBuffer==0x03)
							{
								Forward++;
								ReadFlag=1;
								PhaseState=PhaseStateBuffer;
							}
							else
							{
								ReadFlag=0;
							}
		   
						break;
					case 0x00:
						if(ReadFlag==1)
						{
							BuferHA=(HA*4);
							BuferHB=(HB*2);
				    	BuferHC=(HC*1);
				    	PhaseStateBuffer=(BuferHA|BuferHB|BuferHC);
							ReadFlag=2;
						}
						if(PhaseStateBuffer==PhaseState)
						{
							ReadFlag=1;
							countStopState++;
						}	
						else
							ReadFlag=0;
						if(countStopState>10)
							ReadFlag=0;					
						break;
						case 0x07:
							ReadFlag=0;
						break;
					}
					if(Forward>=200|| Reverse>=200)
					{
						if(Forward>Reverse)
						{
							Status=Closeloop;
								Status=Closeloop;
								firstTimeSetTimer14	=0;
						}
						else if(Forward<Reverse)
						{
							Forward=0;
							Reverse=0;
						}
					}
					else if(CounterInitial>100 && Forward==Reverse)
					{
						Status=Openloop;
					  OpenloopCase=0;
						Firstinit=1;
					}
						
					
//			//	}
//				HAL_Delay(200);
//			for(i=1;i<36;i++)
//			{
//				HAL_Delay(500);
//				ReadPin[i]=(HA<<2|HB<<1|HC<<0);
//			}
//			//	HAL_TIM_Base_Stop_IT(&htim14);
//				

//				BuferHA=HA;///READ PIN HA
//			  BuferHB=HB;///READ PIN HB
//			  BuferHC=HC;///READ PIN HC
//			if(BuferHA==0&& BuferHB==0&& BuferHC==0 )
//				{
//					add++;
//					Status=Openloop;
//					OpenloopCase=0;
//						Firstinit=1;	
//	    					
//				}
//				else 
//					{
//						add1++;

//						if(FrequencyA>70 && FrequencyA <10)
//							{
//								FrequencyA=60;
//								BuferFrequencA =FrequencyA;
//							}
//							else
//								{
//									BuferFrequencA =FrequencyA;
//								}
//								SetCom=BuferFrequencA*5.1+899;
//								Status=Closeloop;
//								firstTimeSetTimer14	=0;		
//					}
			break;			
			case Fault:
				break;
			case Idle1:
				break;
		}
			////////////////////////////End Of SwitchCase////////////////////////////////////////////////
	
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }/////////////////////////////////End Of While(1)//////////////////////////////////////////////////////////
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1599;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 34999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1599;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 34999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1920;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 47999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 1920;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PF0 PF1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void SwitchPhase(uint8_t PhaseStateCloseLoop )
{
				switch(PhaseStateCloseLoop)
					{
					case 0X03:
						StateE(SetCom);
					break;
		    	case 0x01:
			    	StateF(SetCom);
				  break;
			    case 0x05:
			      StateA(SetCom);
				  break;
			    case 0x04:
			      StateB(SetCom);
				  break;
		   	  case 0x06:
				    StateC(SetCom);
			 	  break;
			    case 0x02:
				    StateD(SetCom);
				  break;
					}
}
void PwmDuty( int SignalDuty)
{
	//	Speed_A=(Speed_A*90+ FrequencyA*10)/100;
	
			if (SetCom>1000 && SetCom <1850)//2533
				{
					if(SignalDuty>5 && SignalDuty<95)
					{
						SetCom=8.5*SignalDuty+1000;
						ControlSetCom=(ControlSetCom*10+SetCom*90)/100;//LPF
						SetCom =ControlSetCom;
						
					}
				  __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1,ControlSetCom);

				}	
}
void StateA(unsigned int Duty)
{
	if (Duty >=100 && Duty <1850)
	{
		state=A;
	//	__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1,Duty);
	}
	else if (Duty <100)
	{

		state=Nop;
		SwitchT1_off();
		SwitchT4_off();
	}
	else if (Duty >=1850)
	{
		state=Nop;
		SwitchT2_off();
		SwitchT3_off();
		SwitchT5_off();
		SwitchT6_off();
		SwitchT1_on();
		SwitchT4_on();
	}
}
void StateB(unsigned int Duty)
{
	if (Duty >=100 && Duty <1850)
	{
		state=B;
		//__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1,Duty);
	}
	else if (Duty <100)
	{

		state=Nop;
		SwitchT1_off();
	  SwitchT6_off();
	}
	else if (Duty >=1850)
	{
		state=Nop;
		SwitchT4_off();
		SwitchT2_off();
		SwitchT3_off();
		SwitchT5_off();
		SwitchT1_on();
		SwitchT6_on();
	}
	
}
void StateC(unsigned int Duty)
{
	if (Duty >=100 && Duty <1850)
	{
	
		state=C;
	//	__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1,Duty);
	}
	else if (Duty <100)
	{

		state=Nop;
		SwitchT3_off();
		SwitchT6_off();
	
	}
	else if (Duty >=1850)
	{
		SwitchT1_off();
		SwitchT4_off();
		SwitchT2_off();
		SwitchT5_off();
		state=Nop;
		SwitchT3_on();
		SwitchT6_on();

	}
}
void StateD(unsigned int Duty)
{
	if (Duty >=100 && Duty <1850)
	{
		state=D;
	//	__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1,Duty);
	}
	else if (Duty <100)
	{

		state=Nop;
		SwitchT2_off();
		SwitchT3_off();

	}
	else if (Duty >=1850)
	{
		SwitchT1_off();
		SwitchT4_off();
		SwitchT5_off();
		SwitchT6_off();
		state=Nop;
		SwitchT2_on();
		SwitchT3_on();
	}
}
void StateE(unsigned int Duty)
{
	if (Duty >=100 && Duty <1850)
	{
		state=E;
		//__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1,Duty);
	}
	else if (Duty <100)
	{
	  state=Nop;
		SwitchT2_off();
		SwitchT5_off();
	}
	else if (Duty >=1850)
	{
		SwitchT1_off();
		SwitchT4_off();
		SwitchT3_off();
		SwitchT6_off();
		state=Nop;
		SwitchT2_on();
		SwitchT5_on();
	}
}
void StateF(unsigned int Duty)
{
	if (Duty >=100 && Duty <1850)
	{
		state=F;
		//__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1,Duty);
	}
	else if (Duty <100)
	{
		state=Nop;
		SwitchT4_off();
		SwitchT5_off();
	}
	else if (Duty >=1850)
	{
		SwitchT1_off();
		SwitchT2_off();
		SwitchT3_off();
		SwitchT6_off();
		state=Nop;
		SwitchT4_on();
		SwitchT5_on();
	}
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
