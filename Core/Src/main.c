/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include <stdio.h>
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif
PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}
GETCHAR_PROTOTYPE {
	uint8_t ch = 0;
	__HAL_UART_CLEAR_OREFLAG(&huart2);
	HAL_UART_Receive(&huart2, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define n 53 				//Broj ledica u krugu
#define UseBrightness 1			//Koristimo li svijetlinu svake ledice
#define Pi 3.14159265
#define SLAVE_ADDRESS_LCD 0x4E 	// 0b01001110 Adresa I2C LCD displaya

uint8_t Tim2State = 0;		//Varijabla za provjeru pritisnutosti interupta 2
uint8_t Tim3State = 0;		//Varijabla za provjeru pritisnutosti interupta 3
uint8_t Tim4State = 0;		//Varijabla za provjeru pritisnutosti interupta 4
uint8_t Tim5State = 0;		//Varijabla za provjeru pritisnutosti interupta 5
int e1 = -1; //e11 = -1;//Pomoćne varijable za provjeru promjene stanja tipkala1 na PC0 pinu
int e2 = -1; // e21 = -1;//Pomoćne varijable za provjeru promjene stanja tipkala2 na PC1 pinu
int e3 = -1; // e31 = -1;//Pomoćne varijable za provjeru promjene stanja tipkala3 na PC2 pinu
int e4 = -1; // e41 = -1;//Pomoćne varijable za provjeru promjene stanja tipkala4 na PC3 pinu
int s1 = -1, s11 = -1;
//uint8_t RedFlag = 0;
//uint8_t v = 255 / 32;
uint8_t r = 255;
uint8_t g = 255;
uint8_t b = 255;
uint8_t k, l;
uint8_t p1 = 1; //broj igraca
uint8_t pScore[4]; //bodovi igraca
//uint8_t oneShot[4] = { 0, 0, 0, 0 };
static uint16_t global_gpio_pin = 0; //Spremanje pina kod kojeg se dogodio interupt
uint8_t LedData[n][4];				//Spremanje podataka za svaku ledicu
//uint8_t LedMod[LedMax][4];
uint16_t pwmData[(24 * n) + 50];   //
uint8_t DataSentFlag = 0;
char numbers[11] = "0123456789";
/*			RS	R/W	DB7	DB6	DB5 DB4 DB3 DB2 DB1 DB0
 * CLEAR 	0	0	0	0	0	0	0	0	0	1
 * HOME		0	0	0	0	0	0	0	0	1	-
 * ON/OFF	0	0	0	0	0	0	1	D	C	B
 */

void LcdSendCmd(char cmd) { //7-bitni podatci
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd & 0xf0); //gornja vrijednost 8 bitnog podatka
	data_l = ((cmd << 4) & 0xf0); //doljnja vrijednost 8 bitnog podatka
	data_t[0] = data_u | 0x0C;  //0b00001100 en=1, rs=0
	data_t[1] = data_u | 0x08;  //0b00001000 en=0, rs=0
	data_t[2] = data_l | 0x0C;  //0b00001100 en=1, rs=0
	data_t[3] = data_l | 0x08;  //0b00001000 en=0, rs=0
	HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, (uint8_t*) data_t, 4,
			100);
	HAL_Delay(5);
}
void LcdSendData(char data) { //7-bitni podatci
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data & 0xf0);
	data_l = ((data << 4) & 0xf0);
	data_t[0] = data_u | 0x0D;  //0b00001101 en=1,rw=0, rs=1
	data_t[1] = data_u | 0x09;  //0b00001001 en=0, rs=1
	data_t[2] = data_l | 0x0D;  //0b00001101 en=1, rs=1
	data_t[3] = data_l | 0x09;  //0b00001001 en=0, rs=1
	HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, (uint8_t*) data_t, 4,
			100);
	HAL_Delay(5);
}
void LcdInit(void) {
	// 4 bit initialization
	//	D7	D6	D5	D4
	HAL_Delay(50);  // Cekaj >40ms
	LcdSendCmd(0x30); 	//0b0	0	1	1
	HAL_Delay(5);  //  Cekaj >4.1ms
	LcdSendCmd(0x30);	//0b0	0	1	1
	HAL_Delay(1);  //  Cekaj >100us
	LcdSendCmd(0x30);	//0b0	0	1	1
	HAL_Delay(10);
	LcdSendCmd(0x20);   //0B0	0	1	0 4bit mode
	HAL_Delay(10);

	// display initialization
	LcdSendCmd(0x28); //0b00111000 Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	LcdSendCmd(0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	LcdSendCmd(0x01);  // clear display
	HAL_Delay(1);
	HAL_Delay(1);
	LcdSendCmd(0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	LcdSendCmd(0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}
void LcdSendString(char *str) {
	while (*str)
		LcdSendData(*str++);
}
void LcdPointer(int row, int col) {
	switch (row) {
	case 0:
		col |= 0x80; //1000 0000
		break;
	case 1:
		col |= 0xC0;
		break;
	case 2:
		col |= 0xC0;
		break;
	case 3:
		col |= 0x80;
		break;
	}
	LcdSendCmd(col);
}
void SetLed(int LedNum, int Red, int Green, int Blue, int brightness) {
	LedData[LedNum][0] = LedNum;	//Podatak za koju ledicu vrijedi
	LedData[LedNum][1] = Green;		//Podatak za zelenu boju
	LedData[LedNum][2] = Red;		//Podatak za crvenu boju
	LedData[LedNum][3] = Blue;		//Podatak za plavu boju

#if UseBrightness

	if (brightness > 45)
		brightness = 45;

	for (int i = 1; i < 4; i++) {
		float angle = 90 - brightness;  // in degrees
		angle = angle * Pi / 180;  // in rad
		//LedData[LedNum][i] = (LedData[LedNum][i]) / (tan(angle));
		LedData[LedNum][i] /= tan(angle);
	}

#endif
}

/*void Set_Brightness(int led, int brightness)  // 0-45
 {

 }
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {//povratna funkcija PWM timera
	HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1); //Zaustavljanje timera za slanje PWM podataka na Ledice
	DataSentFlag = 1; //Postavljanje zastavice u visoko stanje nakon poslanih podataka
}

void WS2812b_Send(void) {
	uint32_t indx = 0;	//Praćenje bitova PWM signala
	uint32_t color;		//redoslijedno spremanje vrijednosti boja za slanje

	for (int i = 0; i < n; i++) {
//#if UseBrightness
		/*color = ((LedMod[i][1] << 16) | (LedMod[i][2] << 8) | (LedMod[i][3]));
		 #else*/
		color =
				((LedData[i][1] << 16) | (LedData[i][2] << 8) | (LedData[i][3])); //Zapisujemo redoslijedno zapisivanje RGB vrijednosti
//#endif
		for (int i = 23; i >= 0; i--) {
			if (color & (1 << i)) { //Pomoću AND naredbe prolazimo kroz sve bitove i zapisujemo njihovu PWM vrijednost
				pwmData[indx] = 150;  	// Zapisujemo 2/3 of 225 za logičko "1"
			} else
				pwmData[indx] = 75;  	// Zapisujemo 1/3 of 225 za logičko "0"
			indx++; 					// slijedeći podatak
		}
	}
	for (int i = 0; i < 50; i++) { 	//Zahtijeva se 50 "0" na kraju komunikacije
		pwmData[indx] = 0;
		indx++;
	}
	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*) pwmData, indx); //Slanje pripremljenih podataka
	while (!DataSentFlag) {
	};			//Čekaj dok se ne primimo povratan signal(DataSentFlag = 1)
	DataSentFlag = 0; 					//resetiraj povratan signal
}

void LedCheck(int x1, uint8_t r1, uint8_t g1, uint8_t b1, uint8_t brightness1) {
	if (x1 != -1) {
		SetLed(x1, r1, g1, b1, brightness1);
	}
}
void LcdSendDataNum(uint8_t x) { // dijeljenje brojeva na decimale
	uint8_t j, d;
	j = x % 10;
	d = x / 10;
	numbers[j];
	if (d > 0)
		LcdSendData(numbers[d]);
	LcdSendData(numbers[j]);
}
void OneStep(uint8_t NLed, uint8_t Brightness) {
	CheckpointLedSet(25);

	SetLed(NLed, 0, g, 0, Brightness);

	SetLed(NLed - 1, 0, g, 0, 0);

	LedCheck(e1, r, 0, 0, 45);
	if (p1 >= 2)
		LedCheck(e2, r, g, 0, 45);
	if (p1 >= 3)
		LedCheck(e3, r, 0, b, 45);
	if (p1 >= 4)
		LedCheck(e4, 0, g, b, 45);
}
void OneLap(uint8_t speed) {
	for (int i = 0; i < n; i++) {
		k = i;
		for (int j = 0; j <= 45; j += speed) {
			OneStep(i, j);

			WS2812b_Send();
		}
		for (int j = 40; j > 0; j -= (speed * 3)) {
			OneStep(i, j);

			WS2812b_Send();
		}
	}
	if (k == n - 1) {
		SetLed(n - 1, 0, g, 0, 0);
		WS2812b_Send();
	}

}
void CheckpointLedSet(uint8_t brightness) {
	//printf("Checkpoint p1=%d Oneshot[0]==%d\n",p1, oneShot[0]);
	if (p1 >= 1) {
		SetLed(41, 255, 255, 255, brightness);
	}
	if (p1 >= 2) {
		SetLed(14, 255, 255, 255, brightness);
	}
	if (p1 >= 3) {
		SetLed(28, 255, 255, 255, brightness);
	}
	if (p1 >= 4) {
		SetLed(53, 255, 255, 255, brightness);
	}
}

void Start() {
	Buzzer(100);
	LcdSendCmd(0x01); 					//Clear display
	LcdSendString(
			"Dobrodosli,         Za pocetak klikni                       Tipku");
	HAL_Delay(50);
	//printf("Value of s1 == -1 = %d\n", s1);
	s1 = -1;

	while (s1 > 4 || s1 < 1) {
		//printf("Value of loop s1 = %d\n", s1);
		HAL_Delay(10);
	}
	LcdSendCmd(0x01); 					//Clear display
	HAL_Delay(500);
	s1 = 1;
	s11 = s1;
	uint8_t t1 = 5;
	do {
		LcdSendCmd(0x01); 					//Clear display
		LcdSendString("Broj igraca:");
		p1 = s1;
		//LcdSendData(numbers[p1]);
		LcdSendDataNum(p1);
		LcdSendString("       Igra zapocinje: ");
		//LcdSendData(numbers[t1]);
		LcdSendDataNum(t1);
		LcdSendString("s");
		HAL_Delay(1000);
		t1--;
	} while (t1 > 0);
	LcdSendCmd(0x01); 					//Clear display
	LcdSendString("Zapocnimo!!         Broj igraca je: ");
	//LcdSendData(numbers[p1]);
	LcdSendDataNum(p1);
	HAL_Delay(3000);
}

void Glavna(uint8_t rmax) {
	uint8_t r1 = 1, t1 = 7;
	for (int i = 0; i < 4; i++)
		pScore[i] = 0;
	do {

		CheckpointLedSet(25);
		do {
			LcdSendCmd(0x01); 					//Clear display
			LcdSendString("Brzina igre:x");
			//LcdSendData(numbers[r1]);
			LcdSendDataNum(r1);

			LcdSendString("      P1:");
			//LcdSendData(numbers[pScore[0]]);
			LcdSendDataNum(pScore[0]);
			if (p1 >= 2) {
				LcdSendString(" P2:");
				//LcdSendData(numbers[pScore[1]]);
				LcdSendDataNum(pScore[1]);
			} else
				LcdSendString("     ");
			if (p1 >= 3) {
				LcdSendString(" P3:");
				//LcdSendData(numbers[pScore[2]]);
				LcdSendDataNum(pScore[2]);
			} else
				LcdSendString("     ");
			if (p1 >= 4) {
				LcdSendString(" P4:");
				//LcdSendData(numbers[pScore[3]]);
				LcdSendDataNum(pScore[3]);
			} else
				LcdSendString("     ");
			LcdSendString("                     Spreman?  ");
			//LcdSendData(numbers[t1]);
			LcdSendDataNum(t1);
			if (t1 < 3)
				Buzzer(100);
			HAL_Delay(1000);
			t1--;
		} while (t1 > 0);
		for (int j = 0; j < n; j++) //reset ledica
			SetLed(j, 0, 0, 0, 0);
		WS2812b_Send();
		CheckpointLedSet(25);
		e1 = -1, e2 = -1, e3 = -1, e4 = -1;
		int m = 0;
		while (m < 3
				&& (e1 == -1 || (p1 >= 2 && e2 == -1) || (p1 >= 3 && e3 == -1)
						|| (p1 >= 4 && e4 == -1))) {
			OneLap(r1);
			m++;
		}
		/*for (int i = 0; i < 3; i++) {
		 OneLap(r1);
		 }*/
		r1 += 2;
		t1 = 5;
		if (e1 != -1)
			pScore[0] += 5 - (abs(41 - e1));
		if (e2 != -1)
			pScore[1] += 5 - (abs(14 - e2));
		if (e3 != -1)
			pScore[2] += 5 - (abs(28 - e3));
		if (e4 != -1)
			pScore[3] += 5 - (abs(53 - e4));
		for (int j = 0; j < 4; j++) {
			if (pScore[j] <= 0)
				pScore[j] = 0;
		}
	} while (r1 < rmax * 2 + 1);
	LcdSendCmd(0x01); 					//Clear display
	LcdSendString("Kraj igre           ");
	LcdSendString("P1:");
	//LcdSendData(numbers[pScore[0]]);
	LcdSendDataNum(pScore[0]);
	if (p1 >= 2) {
		LcdSendString(" P2:");
		//LcdSendData(numbers[pScore[1]]);
		LcdSendDataNum(pScore[1]);
	} else
		LcdSendString("     ");
	if (p1 >= 3) {
		LcdSendString(" P3:");
		//LcdSendData(numbers[pScore[2]]);
		LcdSendDataNum(pScore[2]);
	} else
		LcdSendString("     ");
	if (p1 >= 4) {
		LcdSendString(" P4:");
		//LcdSendData(numbers[pScore[3]]);
		LcdSendDataNum(pScore[3]);
	} else
		LcdSendString("     ");
	LcdSendString("                     Hvala na igranju");
	HAL_Delay(5000);
}
void Buzzer(uint16_t duration) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
	HAL_Delay(duration);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_TIM1_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM5_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	LcdInit();
	HAL_Delay(1000);
	/*LcdSendString("MAAAAA BRAVOOOOOO");
	 HAL_Delay(1000);
	 LcdSendString("   NITKO U GENERACIJI:)RASTURAS KAO        CESTITAMMM!!!!");
	 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		Start();
		Glavna(5);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//printf("Pocetak interupta\n");
	if (GPIO_Pin == GPIO_PIN_0 && Tim2State == 0) {
		Tim2State = 1;
		global_gpio_pin = GPIO_Pin;
		//printf("	Pritisnut gumb0: %d\n", global_gpio_pin);
		//__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
		//printf("	Pokretanje timera2\n");
		HAL_TIM_Base_Start_IT(&htim2);
	}
	if (GPIO_Pin == GPIO_PIN_1 && Tim3State == 0) {
		Tim3State = 1;
		global_gpio_pin = GPIO_Pin;
		//printf("	Pritisnut gumb1: %d\n", global_gpio_pin);
		//__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
		//printf("	Pokretanje timera3\n");
		HAL_TIM_Base_Start_IT(&htim3);
	}
	if (GPIO_Pin == GPIO_PIN_2 && Tim4State == 0) {
		Tim4State = 1;
		global_gpio_pin = GPIO_Pin;
		//printf("	Pritisnut gumb2: %d\n", global_gpio_pin);
		//__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
		//printf("	Pokretanje timera4\n");
		HAL_TIM_Base_Start_IT(&htim4);
	}
	if (GPIO_Pin == GPIO_PIN_3 && Tim5State == 0) {
		Tim5State = 1;
		global_gpio_pin = GPIO_Pin;
		//printf("	Pritisnut gumb3: %d\n", global_gpio_pin);
		//__HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);
		//printf("	Pokretanje timera5\n");
		HAL_TIM_Base_Start_IT(&htim5);
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		HAL_TIM_Base_Stop_IT(&htim2);
		Tim2State = 0;
		//printf("		Timer2 zavrsio\n");
		if (HAL_GPIO_ReadPin(GPIOC, global_gpio_pin) == GPIO_PIN_RESET) {
			if (global_gpio_pin == GPIO_PIN_0) {
				if (e1 == -1)
					e1 = k;
				s1 = 1;
				printf("Value of s1 = %d\n", s1);
				//printf("Value of e1:e11 = %d:%d\n", e1, e11);
			}
		}
	}
	if (htim->Instance == TIM3) {
		HAL_TIM_Base_Stop_IT(&htim3);
		Tim3State = 0;
		//printf("		Timer3 zavrsio\n");
		if (HAL_GPIO_ReadPin(GPIOC, global_gpio_pin) == GPIO_PIN_RESET) {
			if (global_gpio_pin == GPIO_PIN_1) {
				if (e2 == -1)
					e2 = k;
				s1 = 2;
				printf("Value of s1 = %d\n", s1);
				//printf("Value of e2:e21 = %d:%d\n", e2, e21);
			}
		}
	}
	if (htim->Instance == TIM4) {
		HAL_TIM_Base_Stop_IT(&htim4);
		Tim4State = 0;
		//printf("		Timer4 zavrsio\n");
		if (HAL_GPIO_ReadPin(GPIOC, global_gpio_pin) == GPIO_PIN_RESET) {
			if (global_gpio_pin == GPIO_PIN_2) {
				if (e3 == -1)
					e3 = k;
				s1 = 3;
				printf("Value of s1 = %d\n", s1);

			}
		}
	}
	if (htim->Instance == TIM5) {
		HAL_TIM_Base_Stop_IT(&htim5);
		Tim5State = 0;
		//printf("		Timer5 zavrsio\n");
		if (HAL_GPIO_ReadPin(GPIOC, global_gpio_pin) == GPIO_PIN_RESET) {
			if (global_gpio_pin == GPIO_PIN_3) {
				if (e4 == -1)
					e4 = k;
				s1 = 4;
				printf("Value of s1 = %d\n", s1);
				//printf("Value of e4:e41 = %d:%d\n", e4, e41);
			}
		}
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
