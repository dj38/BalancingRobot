/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define ENCL_CH1_Pin GPIO_PIN_0
#define ENCL_CH1_GPIO_Port GPIOA
#define ENCL_CH2_Pin GPIO_PIN_1
#define ENCL_CH2_GPIO_Port GPIOA
#define MOTL_DIR_Pin GPIO_PIN_5
#define MOTL_DIR_GPIO_Port GPIOA
#define MOTR_DIR_Pin GPIO_PIN_6
#define MOTR_DIR_GPIO_Port GPIOA
#define PWM_MOTR_Pin GPIO_PIN_7
#define PWM_MOTR_GPIO_Port GPIOA
#define MPU_POWER_Pin GPIO_PIN_9
#define MPU_POWER_GPIO_Port GPIOC
#define BLUETOOTH_TX_Pin GPIO_PIN_9
#define BLUETOOTH_TX_GPIO_Port GPIOA
#define BLUETOOTH_RX_Pin GPIO_PIN_10
#define BLUETOOTH_RX_GPIO_Port GPIOA
#define ENCR_CH1_Pin GPIO_PIN_15
#define ENCR_CH1_GPIO_Port GPIOA
#define ENCR_CH2_Pin GPIO_PIN_3
#define ENCR_CH2_GPIO_Port GPIOB
#define PWM_MOTL_Pin GPIO_PIN_4
#define PWM_MOTL_GPIO_Port GPIOB
#define MPU_INT_Pin GPIO_PIN_7
#define MPU_INT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define TIM_CHANNEL_PWM_MOTR TIM_CHANNEL_1
#define TIM_CHANNEL_PWM_MOTL TIM_CHANNEL_2

/* USER CODE END Private defines */

void __Error_Handler(char *, int);

#define Error_Handler() __Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
