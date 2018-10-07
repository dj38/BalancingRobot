/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2017 STMicroelectronics
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
#include <IOSerialStream.h>
#include "main.h"
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "Motors.h"
#include "MPU6050.h"
#include "Timer.h"
#include "BTCom.h"
#include "Encoder.h"
#include "Tests.h"
#include "Regulation.h"
#include "EEvar.h"

#include <string>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
IOSerialStream serialUSB(&huart2) ;
BTCom serialHC06(&huart1) ;

/* USER CODE END 0 */

int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

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
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_TIM3_Init();
	MX_TIM5_Init();
	MX_TIM2_Init();
	MX_TIM1_Init();
	MX_TIM4_Init();

	/* USER CODE BEGIN 2 */
	// creation of alias on HAL handles to make code more readable
	// Timers
	TIM_HandleTypeDef &htim3_PWM=htim3;
	TIM_HandleTypeDef &htim1_timebaseLSB=htim1;
	TIM_HandleTypeDef &htim4_timebaseMSB=htim4;
	TIM_HandleTypeDef &htim2_encoderRight=htim2;
	TIM_HandleTypeDef &htim5_encoderLeft=htim5;
	TIM_initPrescaler(&htim1_timebaseLSB,1000000);
	HAL_TIM_Base_Start_IT(&htim4_timebaseMSB);
	HAL_TIM_Base_Start_IT(&htim1_timebaseLSB);
	Timer::initHandlers(&htim4_timebaseMSB,&htim1_timebaseLSB);

	// UART
	UART_HandleTypeDef &huart1_hc06=huart1;
	UART_HandleTypeDef &huart2_usb=huart2;
	serialHC06.startRX(); // TODO : clean once IOSerialStream constructor issue will be fixed
	serialUSB.startRX();

	IOStreamList serials;
	serials.attachStream(serialUSB).attachStream(serialHC06);
	serials << "Hello World" << __ENDL;


    // EEPROM emulation initialization
	HAL_FLASH_Unlock();    /* Unlock the Flash Program Erase controller */
	//if(EE_Format()!=HAL_OK) Error_Handler();  /* To format EEPROM uncomment this line*/
	if(EE_Init()!=HAL_OK) Error_Handler(); 	  /* EEPROM Init */
	HAL_FLASH_Lock();     /* RElock once EEPROM initialization is finalized */

	//Motors and associated encoders initialization
	Motor motR(&htim3_PWM,TIM_CHANNEL_PWM_MOTR,MOTR_DIR_GPIO_Port,MOTR_DIR_Pin);
	Motor motL(&htim3_PWM,TIM_CHANNEL_PWM_MOTL,MOTL_DIR_GPIO_Port,MOTL_DIR_Pin);
	Encoder motREnc(&htim2_encoderRight);
	Encoder motLEnc(&htim5_encoderLeft);
	Motors motors(&motL,&motR,&motLEnc,&motREnc);

	//Gyro/Accel initialization
	MPU6050 mpu6050(&hi2c1,MPU_INT_GPIO_Port,MPU_INT_Pin,400000);
	mpu6050.attachStream(serialHC06).attachStream(serialUSB);
	MPU6050::initStatus mpuStatus;
	do {
		HAL_GPIO_WritePin(MPU_POWER_GPIO_Port,MPU_POWER_Pin,GPIO_PIN_RESET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(MPU_POWER_GPIO_Port,MPU_POWER_Pin,GPIO_PIN_SET);
		HAL_Delay(1000);
		mpuStatus=mpu6050.fullInitMPU6050();
	} while (mpuStatus!=MPU6050::OK);

	/* Control
	Angle PID : P=3.97, I=0.211, D=0.021
	AngleOffset = -2.2°
	Speed PID : P=0.42, I=0.189, D=1e-7
	=> P=0.84 apres nbre pulse/rev *2 (la vitesse est au passage divisée par 2) ?
	Regulation mode : speed
	AngleLimits=+- 15°

	*/
	HAL_FLASH_Unlock();
	EEvar<float> anglePID_P(3.97,0);
	EEvar<float> anglePID_I(0.211,1);
	EEvar<float> anglePID_D(0.021,2);
	EEvar<float> angleOffset(1.1,3);
	EEvar<float> speedPID_P(0.42,4);
	EEvar<float> speedPID_I(0.189,5);
	EEvar<float> speedPID_D(1e-7,6);
	EEvar<float> controlAngleLimit(35,7);
	// TODO use an EEVar to store Regulation_mode ???
	HAL_FLASH_Lock();

	//Regulation regul(3.97, 0.211, 0.021, 0.3, 0.7, 0.005, 1.1 , Regulation::speed, 35);
	Regulation regul(anglePID_P.getVal(),anglePID_I.getVal(),anglePID_D.getVal(),
			speedPID_P.getVal(),speedPID_I.getVal(),speedPID_D.getVal(),
			angleOffset.getVal(), Regulation::speed, controlAngleLimit.getVal() );
	//regul.setJoystickAngleGain(1);
	regul.setJoystickSpeedGain(0.25);
	regul.setJoystickAngleGain(0.35);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	/* Tests section BEGIN */
	/*
  Tests::setObjectsRefs(&motR,&motL,&motREnc,&motLEnc,&motors,&mpu6050,&serialUSB,&serialHC06);
  Tests::procTestMotorLinearity();
	 */
	/* Tests section END */

	Timer tim;
	tim.start();
	TimeOut timeout;
	timeout.setTimeOut(1000000*2);
	regul.setMotorsState(true);
	int displayIteration=0;

	//float myLog [4][1000];
	//uint16_t logIndex=0;

	while (1)
	{
		if(mpu6050.update()) // if new data are available from mpu6050, update regul
		{
			motors.computeSpeed();
			regul.update(tim.read_us(),mpu6050.pitch(),motors.getSpeed());
			motors.setPWM(regul.getLeftMotorPWM(),regul.getRightMotorPWM());
			/*
			if((tim.read()>10) && (logIndex<1000)) {
				myLog[0][logIndex]=tim.read();
				myLog[1][logIndex]=regul.getMeasuredAngle();
				myLog[2][logIndex]=regul.getMeasuredSpeed();
				myLog[3][logIndex]=regul.getRightMotorPWM();
				//myLog[3][logIndex]=(regul.getLeftMotorPWM()+regul.getRightMotorPWM())/2;
				logIndex++;
			}*/
		};
/*
		if((999<logIndex)&&(logIndex<2000)){
			if(serialHC06.getTxBufferSize()==0){
				uint16_t tmpIndex=logIndex-1000;
				serialHC06 << tmpIndex << ";";
				serialHC06 << myLog[0][tmpIndex]-myLog[0][0] << ";";
				serialHC06 << myLog[1][tmpIndex] << ";";
				serialHC06 << myLog[2][tmpIndex] << ";";
				serialHC06 << myLog[3][tmpIndex] << "\n";
				logIndex++;
			}
		}
*/
		// check if new command have been received from bluetooth link
		SerialCommand command=serialHC06.getSerialCommand();
		std::vector<float> values=command.getValues();
		switch(command.getCommand()) {
		case SerialCommand::NOP:
			break;
		case SerialCommand::saveDataToEEPROM:
			HAL_FLASH_Unlock(); // necessary to allow writing
			anglePID_P.write(); // Todo operation on a list of EEVar?
			anglePID_I.write();
			anglePID_D.write();
			angleOffset.write();
			speedPID_P.write();
			speedPID_I.write();
			speedPID_D.write();
			controlAngleLimit.write();
			HAL_FLASH_Lock();
			break;
		case SerialCommand::setAnglePIDValueP:
			anglePID_P.setRamVal(values.at(0));
			regul.getPIDAngle()->setPTuning(anglePID_P.getVal());
			break;
		case SerialCommand::setAnglePIDValueI:
			anglePID_I.setRamVal(values.at(0));
			regul.getPIDAngle()->setITuning(anglePID_I.getVal());
			break;
		case SerialCommand::setAnglePIDValueD:
			anglePID_D.setRamVal(values.at(0));
			regul.getPIDAngle()->setDTuning(anglePID_D.getVal());
			break;
		case SerialCommand::setJoystickXY:
			regul.setJoyStickValue(values.at(0),values.at(1)); break;
		case SerialCommand::setJoystickY:
			regul.setJoyStickYValue(values.at(0)); break;
		case SerialCommand::setJoystickX:
			regul.setJoyStickXValue(values.at(0)); break;
		case SerialCommand::setAngleOffset:
			angleOffset.setRamVal(values.at(0));
			regul.setAngleOffset(angleOffset.getVal());
			break;
		case SerialCommand::setSpeedRegulMode:
			regul.setRegulationMode(Regulation::speed); break;
		case SerialCommand::setAngleRegulMode:
			regul.setRegulationMode(Regulation::angle); break;
		case SerialCommand::setSpeedPIDValueP:
			speedPID_P.setRamVal(values.at(0));
			regul.getPIDSpeed()->setPTuning(speedPID_P.getVal());
			break;
		case SerialCommand::setSpeedPIDValueI:
			speedPID_I.setRamVal(values.at(0));
			regul.getPIDSpeed()->setITuning(speedPID_I.getVal());
			break;
		case SerialCommand::setSpeedPIDValueD:
			speedPID_D.setRamVal(values.at(0));
			regul.getPIDSpeed()->setDTuning(speedPID_D.getVal());
			break;
		case SerialCommand::setControlAngleLimit:
			controlAngleLimit.setRamVal(values.at(0));
			regul.setControlAngleLimit(controlAngleLimit.getVal());
			break;
		case SerialCommand::formatEEPROM:
			HAL_FLASH_Unlock();
			if(EE_Format()!=HAL_OK) Error_Handler();
			if(EE_Init()!=HAL_OK) Error_Handler();
			HAL_FLASH_Lock();
			break;
		case SerialCommand::sendPIDValues:
			serialHC06 << "AnglePID settings :" << "\n";
			serialHC06 << "P : " << regul.getPIDAngle()->getPParam() << "\n";
			serialHC06 << "I : " << regul.getPIDAngle()->getIParam() << "\n";
			serialHC06 << "D : " << regul.getPIDAngle()->getDParam() << "\n";
			serialHC06 << "Target : " << regul.getAngleOffset() << "\n";
			serialHC06 << "SpeedPID settings :" << "\n";
			serialHC06 << "P : " << regul.getPIDSpeed()->getPParam() << "\n";
			serialHC06 << "I : " << regul.getPIDSpeed()->getIParam() << "\n";
			serialHC06 << "D : " << regul.getPIDSpeed()->getDParam() << "\n";
			serialHC06 << "Angle Limit : " << regul.getControlAngleLimit() << "\n";
			serialHC06 << "Regulation mode : " ;
			switch(regul.getRegulMode()) {
			case Regulation::angle: serialHC06 << "angle\n" ; break;
			case Regulation::speed: serialHC06 << "speed\n" ; break;
			case Regulation::pos: serialHC06 << "position\n" ; break;
			}
			serialHC06 << "\n";
			break;
		default:
			serialUSB << "unknown : " << command.getStringCommand() << __ENDL;
			serialHC06 << "unknown : " << command.getStringCommand() << "\n";
			break;
		}
		if(command.getCommand()!=SerialCommand::NOP){
			serialUSB << command.getStringCommand() << "(";
			serialHC06 << command.getStringCommand() << "(";
			for (uint16_t i = 0; i < values.size(); ++i) {
				serialUSB << values.at(i);
				serialHC06 << values.at(i);
				if(i<(values.size()-1)) {
					serialUSB << "," ;
					serialHC06 << "," ;
				}
			}
			serialUSB << ")\n\r";
			serialHC06 << ")\n";
		}

		if(timeout.repeatedTimeoutCheck()) { // display values periodically
			displayIteration++;
			serialHC06  << "runtime : " << tim.read() << "\n";
			serialHC06 << "Regul : " ;
			switch(regul.getRegulMode()) {
				case Regulation::angle: serialHC06 << "angle\n" ; break;
				case Regulation::speed: serialHC06 << "speed\n" ; break;
				case Regulation::pos: serialHC06 << "position\n" ; break;
			}
			serialHC06 << "target/meas angle : " << regul.getTargetAngle() << "/" << regul.getMeasuredAngle() << "\n";
			serialHC06 << "target/meas speed : " << regul.getTargetSpeed() << "/" << regul.getMeasuredSpeed() << "\n";
			Motors::position_t position=motors.getPosition();
			serialHC06 << "Position (x/y/theta) : " << position.x << "/" << position.y << "/" << 180.0/3.141592654*position.theta << "\n";

			/*
            mpu.m_StatLogList.print(&pc,true);
            mpu.m_StatLogList.resetAllStatLog();
			  */
		};

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

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		__Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		__Error_Handler(__FILE__, __LINE__);
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

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//	if (htim->Instance == TIM1) {TIM1UpdateItFlag=1;};
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1) // USART2=USB, USART1=HC06
		serialHC06.RxCpltCallback();
	else if(huart->Instance==USART2) // USART2=USB, USART1=HC06
		serialUSB.RxCpltCallback();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	__NOP();
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1) // USART2=USB, USART1=HC06
		serialHC06.TxCpltCallback();
	else if(huart->Instance==USART2) // USART2=USB, USART1=HC06
		serialUSB.TxCpltCallback();
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void __Error_Handler(char * file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
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
