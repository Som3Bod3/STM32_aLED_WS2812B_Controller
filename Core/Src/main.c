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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdarg.h"
#include "string.h"
#include "math.h"
#include "ws2812b.h"
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
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

//\\\\\\\\\\\\\\\\\\\\\\\\\\Protokół Komunikacyjny/////////////////////////////
/*========================Receive==============================*/
#define buf_RX_length 255 //dlugość bufora
volatile uint8_t buf_RX[buf_RX_length];
volatile uint16_t RX_EMPTY = 0; //wskaźnik zapisu bufora
volatile uint16_t RX_BUSY = 0; //wskaźnik odbioru bufora
/*========================Transmit=============================*/
#define buf_TX_length 255 //długość bufora
volatile uint8_t buf_TX[buf_TX_length];
volatile uint16_t TX_EMPTY = 0; //wskaźnik zapisu bufora
volatile uint16_t TX_BUSY = 0; //wskaźnik odbioru bufora
/*========================Inne=================================*/
uint8_t errorAS; //Flaga już wysłanego błędu
uint8_t commandSlot = 0; //Indeks tablicy command
char command[100]; //Tablica odczytująca komende
char returnFrame[9]; //Tablica zawierająca ramke zwrotną

////////////////////////////Protokół Komunikacyjny\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//\\\\\\\\\\\\\\\\\\\\\\\\\\WS2812B/////////////////////////////

#define n_led 8 //ilość obsługiwanych ledów
//#define n_led 256 //ilość obsługiwanych ledów
#define reset_bit 60 //ilośc bitów resetujących
#define n_bit reset_bit+24*n_led //połączona ilość bitów do wysłania

uint32_t reset = 0; //bit resrtu
uint32_t high = 64; //bit logiczne 1
uint32_t low = 32; //bit logiczne 0

uint32_t control_buffer[n_bit]; //Cały bufor przechowujący dane do LED'ów, podawany do PWM'a

////////////////////////////WS2812B\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

uint8_t switch_color = 0; //Indeks do siwtch'a
int execute = 1;

//\\\\\\\\\\\\\\\\\\\\\\\\\\LED ANIMACJE/////////////////////////////
//Współrzędne macierzy do testowania
int x = 1, x2 = 1, y = 1, y2 = 1;

//zmienne kontroli animacji z defaultowymi wartościami
float brightness = 0.6; //jasność (od 0 do 1 - mnoży kolory)
float speed = 0.6; //prędkość animacji (od 0 do 1 - mnoży kolory)
uint8_t colourID = 2; //ID koloru
uint8_t modeID = 1; //ID trybu wyświetlania (0 to tryb testowania)

//KOLORY
uint8_t pink[3] = {255,0,255};
uint8_t folly[3] = {255,0,85};
uint8_t red[3] = {255,0,0}; //RED
uint8_t orange[3] = {255,128,0};

uint8_t lime[3] = {212,255,0};
uint8_t brightGreen[3] = {85,255,0};
uint8_t green[3] = {0,255,0};//GREEN
uint8_t malachite[3] = {0,255,85};

uint8_t capri[3] = {0,170,255};
uint8_t blue[3] = {0,0,255};//BLUE
uint8_t indigo[3] = {128,0,255};

uint8_t white[3] = {255,255,255};
uint8_t turn_off[3] = {0,0,0};
uint8_t inputColour[3] = {0, 0, 0};

uint8_t noOfColours = 11; //liczba używanych kolorów podczas normalnej pracy
//tablica wskaźników na kolory
uint8_t *colourPtr[13] = {inputColour, turn_off, red, orange, lime, brightGreen, green, malachite, capri, blue, indigo, pink, folly}; //noOfColours+2

int Tick = 0; //tick zegara co 1 ms
uint8_t somethingChanged = 1; //Flaga wprowadzonej zmiany (koloru, prędkości, jasności, trybu)

//NEW
int colorSwitchStatus = 0; //zmienna statusu funkcji colorSwitch

////////////////////////////LED ANIMACJE\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//EXTI
uint8_t interrupted = 0; //zmienna sprawdza czy przerwanei zostało właśnie wywołane
uint32_t interruptTick = 0; //Tick opóźnienia przerwania

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//\\\\\\\\\\\\\\\\\\\\\\\\\\Protokół Komunikacyjny/////////////////////////////
void communicationProtocol();
void readFrame();
int controlSumRX();
void readCommand();
void returnFrameAppend(char str[]);
int compareCommand(char commandStr[], int hasArgs, char str[]);
uint32_t hex2int(char *hex);
int numOfDigits(long long num);

void Send(char* msg_to_send, ...);
////////////////////////////Protokół Komunikacyjny\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\


//\\\\\\\\\\\\\\\\\\\\\\\\\\WS2812B/////////////////////////////

uint8_t switchBitOrder(uint8_t var); //Funkcja zmienia endian
uint32_t getColorInBit(uint8_t red,uint8_t green,uint8_t blue); //Funkcja zmienia dec na binarny
uint32_t mapBitInNZR(uint32_t color, uint8_t c); //Funkcja zmienia binarny na NZR
void get_aLight(uint8_t * color_vector, uint16_t ledID, uint16_t ledAmount);
void innit_aLight();
void launchLED(); //TODO jakaś implementacja
////////////////////////////WS2812B\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//\\\\\\\\\\\\\\\\\\\\\\\\\\LED ANIMACJE/////////////////////////////

//NEW
void checkArgs(); //sprawdza poprawność zakresu argumentów i je ewentualnie zapętla
void exeLED(); //wybiera funkcje trybu
void save(); //zapisuje obecny stan do zmiennych temp lub do pamiecy flash
void init(); //uruchamia od nowa wszystko i resetuje pamiec flash oraz zmienne temp
void bringback(); //przywraca stan z pamięci flash lub zmiennych temp
void testInit(); //zachowuje zmienne save() i przechodzi w tryb testowania

//Funkcje zmieniające parametry
void CMN(); //Zmiana trybu wyświetlania na następny.
void CMP(); //Zmiana trybu wyświetlania na poprzedni.
void CCN(); //Zmiana wyświetlanego koloru na następny.
void CCP(); //Zmiana wyświetlanego koloru na poprzedni
void CBU(); //Zmiana jasności na wyższy stopień.
void CBD(); //Zmiana jasności na niższy stopień.
void CSU(); //Zmiana prędkości wyświetlania na wyższy stopień.
void CSD(); //Zmiana prędkości wyświetlania na niższy stopień.

//Animacje
uint8_t * colorSwitch(uint8_t * color_vector1, uint8_t * color_vector2, uint8_t jump); //Funkcja przechodzi między dwoma kolorami
int noOfAnimations = 5;
void testMode(); //Tryb testowy
void oneColour(); //wyświetla jeden stały kolor
void colourPulse(); //wyświetla pulsujący kolor
void colourFade(); //Przechodzi między odcieniami jednego koloru
void rainbow(); //przechodzi przez wszystkie kolory
void iconMode(); //Wyświetla animacje ikon

////////////////////////////LED ANIMACJE\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//\\\\\\\\\\\\\\\\\\\\\\\\\\FLASH/////////////////////////////

//TODO test
void eraseSector5(); //Funkcja czyści sektor piąty
void getNextFreeFlashWord(); //Funkcja szuka pierwszej wolnej pary słów w piątym sektorze
void SaveVarsToFlash(); //Funkcja zapisuje zmienne operacyjne do flash
void getVarsFromFlash(); //Funkcja odczytuje ostatnie zmienne operacyjne z pamięci flash
uint32_t * nextFreeFlashWord = (uint32_t*)0x08020000; //Wskaźnik na pierwsze wolne słowo pamięci

////////////////////////////FLASH\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == B1_Pin)
	{
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		Send("test\r\n");
	}
	else if(GPIO_Pin == BTN1_Pin && !interrupted) //Przycisk 1
	{
		interrupted = 1;
		CCN();
	}
	else if(GPIO_Pin == BTN2_Pin && !interrupted) //Przycisk 2
	{
		interrupted = 1;
		CMN();
	}
	else if(GPIO_Pin == BTN3_Pin && !interrupted) //Przycisk 3
	{
		interrupted = 1;
		CSU();
	}
	else if(GPIO_Pin == BTN4_Pin && !interrupted) //Przycisk 4
	{
		interrupted = 1;
		CBU();
	}
	else if(GPIO_Pin == BTN5_Pin && !interrupted) //Przycisk 5
	{
		interrupted = 1;
		CMP();
	}
	else if(GPIO_Pin == BTN6_Pin && !interrupted) //Przycisk 6
	{
		interrupted = 1;
		CCP();
	}
}

void Sys_1ms_Tick() //Funkcja wywoływana co 1ms
{
	Tick++;

	if(interrupted) //Sprawdzenie czy przerwanie zostało wywołane
	{
		interruptTick++;
		if(interruptTick > 300) //Sprawdzenie czy minął czas od ostatniego wciśnięcia przycisku
		{
			interruptTick = 0; //Reset zmiennej Tick
			interrupted = 0; //reset zmiennej interrupted
		}
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
  MX_USART2_UART_Init();
  MX_CRC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart2, &buf_RX[RX_EMPTY], 1); //Uruchomienie oczekiwania na znak

  getNextFreeFlashWord(); //Znalezienie ostatnich zapisanych zmiennych w FLASH
  bringback(); //Odczytanie ostatnich zapisanych zmiennych z FLASH

  innit_aLight(); //Inicjalizacja bufora LED z kolorem czarnym
  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, control_buffer, n_bit); //Uruchomienie PWM z DMA

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  communicationProtocol(); //Sprawdzenie protokołu komunikacyjnego

	  //Sprawdzenie czy zmienne operacyjne uległy zmianie lub minął ustalony czas
	  if(somethingChanged || Tick > 50*(1.2-speed))
	  {
		  exeLED();
		  Tick = 0; //Reset czasu
		  somethingChanged = 0; //Reset flagi zmiany
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//================================================WS2812B====================================================================

void launchLED()
{
	innit_aLight();
	switch_color++;
	switch(switch_color)
	{
		case 1:
			get_aLight(colourPtr[3],0,4);
			break;
		case 2:
			get_aLight(turn_off,0,8);
			break;
		case 3:
			get_aLight(colourPtr[3],0,4);
			break;
		case 4:
			get_aLight(turn_off,0,8);
			break;
		case 5:
			get_aLight(colourPtr[3],0,4);
			break;
		case 6:
			get_aLight(turn_off,0,8);
			break;
		case 7:
			get_aLight(turn_off,0,8);
			break;
		case 8:
			get_aLight(turn_off,0,8);
			break;
		case 9:
			get_aLight(colourPtr[3],4,4);
			break;
		case 10:
			get_aLight(turn_off,0,8);
			break;
		case 11:
			get_aLight(colourPtr[3],4,4);
			break;
		case 12:
			get_aLight(turn_off,0,8);
			break;
		case 13:
			get_aLight(colourPtr[3],4,4);
			break;
		case 14:
			get_aLight(turn_off,0,8);
			break;
		case 15:
			get_aLight(turn_off,0,8);
			break;
		case 16:
			get_aLight(turn_off,0,8);
			switch_color = 0;
			break;
		  }
}

void get_aLight(uint8_t * color_vector, uint16_t ledID, uint16_t ledAmount)
{
	//Funkcja konwertuje kolor na NZR i edytuje bufor dla dokładnych adresów

/*	uint8_t red = color_vector[0] * brightness * 0.02;
	uint8_t green = color_vector[1] * brightness * 0.02;
	uint8_t blue = color_vector[2] * brightness * 0.02;*/

	//rozdzielenie kolorów i przemnożenie wartości przez zmienną jasności

	uint8_t red = color_vector[0] * brightness;
	uint8_t green = color_vector[1] * brightness;
	uint8_t blue = color_vector[2] * brightness;


	uint32_t k = 0; //Zmienna pomocnicza
	uint8_t c = 0; //Zmienna pomocnicza
	uint32_t color = getColorInBit(red,green,blue); //Przetworzenie kolorów na postać bitową
	for (int i = reset_bit + (24*ledID); i < reset_bit + (24*ledID) + (24*ledAmount); i++)
	{
		if (i >= reset_bit) //Uzupełnienie bitów z kolorami
		{
			 c = k % 24; //zmienna C wyznacza miejsce bit'u
			 control_buffer[i] = mapBitInNZR(color,c); //konwersja na NZR
			 k++;
		}
	}
}

void innit_aLight() //Funkcja inicjalizuje bufor z kolorem czarnym
{
	uint8_t red = 0; //rozdzielenie kolorów
	uint8_t green = 0;
	uint8_t blue = 0;
	uint32_t k = 0;
	uint8_t c = 0;
	uint32_t color = getColorInBit(red,green,blue); //Przetworzenie kolorów na postać bitową
	for (int i = 0; i < n_bit; i++)
	{
		  if (i < reset_bit) //Uzupełnienie bitów reset
			  control_buffer[i] = reset;
		  else
		  { //Uzupełnienie bitów z kolorami
			  c = k % 24; //zmienna C wyznacza miejsce bit'u
			  control_buffer[i] = mapBitInNZR(color,c); //konwersja na NZR
			  k++;
		  }
	}
}

uint32_t mapBitInNZR(uint32_t color, uint8_t c) //Funkcja koduje bit w NZR odpowiednią kombinacją stanów wysokich i niskich
{
	if (((color & (1 << c)) >> c) == 1) //Sprawdzenie bitu c czy jest logicznym 1 lub 0
		return high;
	else
		return low;
}

uint32_t getColorInBit(uint8_t red,uint8_t green,uint8_t blue) //Funkcja zmienia kolejnośc kolorów z RGB na GRB
{
	uint8_t red_s = switchBitOrder(red); //Zamiana kolejności bitów
	uint8_t green_s = switchBitOrder(green);
	uint8_t blue_s = switchBitOrder(blue);
	return (green_s + (red_s << 8) + (blue_s << 16)); //Zwrócenie odpowiednio ułożonych barw
}

uint8_t switchBitOrder(uint8_t var) //Funkcja odwraca kolejność bitów
{
    uint8_t rav = 0; //Zmienna zwracana
    for(int i=0;i<8;i++) //Przejście przez każde 8 bitów
    {
    	rav |= ((var>>i) & 0b1)<<(7-i); //Przeniesienie bitu na drugą stronę
    }
	return rav; //Zwrócenie odbitej zmiennej
}

//================================================WS2812B====================================================================

//================================================LED ANIMACJE====================================================================

void save() //Funkcja zapisuje obecne zmienne
{
	SaveVarsToFlash();
}

void bringback() //Funkcja wczytuje ostatnie zachowane zmienne
{
	getVarsFromFlash();
}

void init() //Funkcja ustawia wartości zmiennych operacyjnych na nowe i je zapisuje
{
	brightness = 0.6; //jasność (od 0 do 1 - mnoży kolory)
	speed = 0.6; //prędkość animacji (od 0 do 1 - mnoży kolory)
	colourID = 2; //ID koloru
	modeID = 1; //ID trybu wyświetlania (0 to tryb testowania)
	save(); //Zapisanie wartości zmiennych operacyjnych
	somethingChanged = 1;  //Zmiana flagi zmiany wartości operacyjnych na true
}

void testInit() //Funkcja przygotowująca matryce do trybu testowego
{
	bringback(); //Odczytanie wartości poprzednich zmiennych operacyjnych
	save(); //Zapisanie wartości zmiennych operacyjnych

	modeID = 0; //Ustawienie wartości pod tryb testowy
	colourID = 0;
	brightness = 1;
	speed = 0.1;

	inputColour[0] = 0; //Reset wprowadzanych kolorów i pozycji
	inputColour[1] = 0;
	inputColour[2] = 0;
	x = 1;
	y = 1;
	x2 = 1;
	y2 = 1;

	innit_aLight(); //Inicjalizacja bufora kolorów od nowa
	somethingChanged = 1; //Zmiana flagi zmiany wartości operacyjnych na true
}

void checkArgs() //Funkcja sprawdza poprawność zakresu zmiennych operacyjnych
{
	//Warunki sprawdzające zakresy jasności, prędkości, trybu, koloru i zapętlanie ich
	//Kolor
	if(colourID-2 < 0)
	{
		colourID = noOfColours + 1;
	}
	else if(colourID-2 > noOfColours-1)
	{
		colourID = 2;
	}

	//Tryb
	if(modeID < 1)
	{
		modeID = noOfAnimations-1;
	}
	else if(modeID > noOfAnimations-1)
	{
		modeID = 1;
	}

	//Jasność
	if(brightness <= 0)
	{
		brightness = 1;
	}
	else if(brightness > 1)
	{
		brightness = 0.2;
	}

	//Prędkość
	if(speed <= 0)
	{
		speed = 1;
	}
	else if(speed > 1)
	{
		speed = 0.2;
	}
}

void exeLED() //Funkcja wybiera odpowiedni tryb
{
	//Swicth, który wybiera odpowiedni tryb na bazie zmiennej operacyjnej modeID
	switch(modeID)
	{
	case 0:
		testMode(); //Tryb testowy metrycy
		break;
	case 1:
		oneColour(); //animacja 1
		break;
	case 2:
		colourPulse(); //animacja 2
		break;
	case 3:
		colourFade(); //animacja 3
		break;
	case 4:
		rainbow(); //animacja 4
		break;
	case 5:
		iconMode(); //animacja 5 - animacja ikon
		break;
	default:
		break;
	}
}

//Animacje

uint8_t * colorSwitch(uint8_t * color_vector1, uint8_t * color_vector2, uint8_t jump) //Funkcja przechodzi płynnie między dwoma kolorami
{
	/* Funkcja odpowiada za płynne przejście między kolorem, który jest za wskaźnikiem
	 * color_vector1 do koloru color_vector2. W momencie osiągnięcia koloru zmienna colorSwitchStatus będzie równa 1
	 * */

	static uint8_t returnColor[3]; //Zwracany kolor
	static float newColor[3]; //Kolor pomocniczy do dokładnego obliczenia wartości
	static float dif[3]; //Różnica

	if(somethingChanged) //Sprawdzenie czy zmienne operacyjnie się zmieniły
	{
		for(int e = 0; e<3; e++)
		{
			newColor[e] = color_vector1[e]; //Przepisanie wartości koloru pierwszego
			dif[e] = (float)abs(color_vector1[e] - color_vector2[e])/jump; //Ustawienie różnicy animacji
		}
	}


	for(int e = 0; e<3; e++)
	{
		//Warunek wybiera odpowiedni kierunek zmiany koloru na podstawie różnicy argumentów funkcji
		if(color_vector1[e] > color_vector2[e])
		{
			//Zmniejszenie wartości barwy e
			newColor[e] -= dif[e];
			//Sprawdzenie czy wartość z argumentu drugiego została osiągnięta
			if(newColor[e] < color_vector2[e])
			{
				//Oznaczenie flagi końca zmiany koloru na 1
				colorSwitchStatus = 1;
			}
		}
		else
		{
			//Zwiększenie wartości barwy e
			newColor[e] += dif[e];
			//Sprawdzenie czy wartość z argumentu drugiego została osiągnięta
			if(newColor[e] > color_vector2[e])
			{
				//Oznaczenie flagi końca zmiany koloru na 1
				colorSwitchStatus = 1;
			}
		}
		if(!colorSwitchStatus) //Sprawdzenie czy kolor 2 został osiągnięty
		{
			returnColor[e] = newColor[e]; //Przepisanie zmienionej wartości koloru
		}
	}
	return returnColor; //Zwrócenie uzyskanego koloru
}

void testMode() //Funkcja zapala na matrycy ledy na bazie wprowadzonych współrzędnych macierzy
{
	if(somethingChanged) //Sprawdzenie czy obliczenia są konieczne
	{
		for(int i = 1; i<=16; i++) //Przejście po kolumnach matrycy
		{
			if(i >= x && i <= x2) //Sprawdzenie czy w kolumnie znajduje się nowy kolor
			{
				uint16_t startLed; //Deklaracja ledu startującego
				if(i%2!=0) //Sprawdzenie czy kolumna jest nieparzysta
				{
					startLed = 16*(i-1) + (y-1); //przypisanie wartości ledu startującego
				}
				else
				{
					startLed = 16*(i-1) + (16-y2); //przypisanie wartości ledu startującego
				}
				uint16_t ledNo = y2 - y + 1; //przypisanie ilości ledów do zapalenia w danej kolumnie
				get_aLight(colourPtr[colourID],startLed,ledNo); //Zapalenie ledów
			}
		}
	}
}

void oneColour() //funkcja wyświetla jeden kolor
{
	get_aLight(colourPtr[colourID],0,n_led);
}

void colourPulse() //Funkcja wyswietla pulsujący kolor
{
	static uint8_t animationToggle = 0; //Przejście do kolejnego etapu animacji
	static uint8_t nextColor[3]; //kolejny kolor

	if(somethingChanged) //Sprawdzenie czy zmienne operacyjne uległy zmianie
	{
		uint8_t *color_vector = colourPtr[colourID]; //Przepisanie nowego koloru
		nextColor[0] = color_vector[0] * 0.15;
		nextColor[1] = color_vector[1] * 0.15;
		nextColor[2] = color_vector[2] * 0.15;
		colorSwitchStatus=0; //Ustawienie flagi zakończenia zmiany koloru na 0
		animationToggle = 0; //Ustawienie zmiennej toggle na 0
	}

	if(colorSwitchStatus == 1) //Sprawdzenie czy zmiana koloru została zakończona
	{
		//Zmiana wartości zmiennej toggle na przeciwną
		animationToggle = (animationToggle == 1) ? 0 : 1;
		colorSwitchStatus = 0; //Ustawienie flagi zakończenia zmiany koloru na 0
		somethingChanged = 1; //Ustawienie flagi zmiany zmiennych operacyjnych na 1
	}

	//Wybranie odpowiedniego wywołania funkcji colorSwitch() na bazie zmiennej Toggle
	if(animationToggle == 0)
	{
		//Przejście z koloru bazowego na przyciemniony
		get_aLight(colorSwitch(colourPtr[colourID], nextColor, 40),0,n_led);
	}
	else
	{
		//Przejście z koloru przyciemnionego na bazowy
		get_aLight(colorSwitch(nextColor, colourPtr[colourID], 40),0,n_led);
	}
}

void colourFade() //Funkcja przechodzi płynnie między obecnym kolorem a następnym
{
	static uint8_t animationToggle = 0; //Przejście do kolejnego etapu animacji
	static uint8_t color1[3]; //kolor pierwszy

	if(somethingChanged) //Sprawdzenie czy zmienne operacyjne uległy zmianie
	{
		//uzyskanie ID następnego koloru
		uint8_t colourIDtmp = (colourID + 1 > noOfColours + 1) ? 2 : colourID + 1;
		uint8_t *color_vector = colourPtr[colourIDtmp]; //Przepisanie koloru 1
		color1[0] = color_vector[0];
		color1[1] = color_vector[1];
		color1[2] = color_vector[2];
		colorSwitchStatus=0; //Ustawienie flagi zakończenia zmiany koloru na 0
		animationToggle = 0; //Ustawienie zmiennej toggle na 0
	}

	if(colorSwitchStatus == 1) //Sprawdzenie czy zmiana koloru została zakończona
	{
		//Zmiana wartości zmiennej toggle na kolejną
		animationToggle = (animationToggle == 1) ? 0 : 1;
		colorSwitchStatus = 0; //Ustawienie flagi zakończenia zmiany koloru na 0
		somethingChanged = 1; //Ustawienie flagi zmiany zmiennych operacyjnych na 1
	}

	//Wybranie odpowiedniego wywołania funkcji colorSwitch() na bazie zmiennej Toggle
	if(animationToggle == 0)
	{
		//Przejście z color 1 na bazowy
		get_aLight(colorSwitch(colourPtr[colourID], color1, 40),0,n_led);
	}
	else
	{
		//Przejście z bazowego na color 1
		get_aLight(colorSwitch(color1, colourPtr[colourID], 40),0,n_led);
	}
}

void rainbow() //Funkcja przechodzi płynnie między wszystkimi kolorami
{
	static uint8_t colourIDR = 2; //ID koloru 1
	static uint8_t colourIDRN = 3; //ID koloru 2

	if(somethingChanged) //Sprawdzenie czy zmienne operacyjne uległy zmianie
	{
		colorSwitchStatus=0; //Ustawienie flagi zakończenia zmiany koloru na 0
	}

	if(colorSwitchStatus == 1) //Sprawdzenie czy zmiana koloru została zakończona
	{
		colourIDR = (colourIDR + 1 > noOfColours + 1) ? 2 : colourIDR + 1; //Przejście na kolejny kolor
		colourIDRN = (colourIDRN + 1 > noOfColours + 1) ? 2 : colourIDRN + 1; //Przejście na kolejny kolor
		colorSwitchStatus = 0; //Ustawienie flagi zakończenia zmiany koloru na 0
		somethingChanged = 1; //Ustawienie flagi zmiany zmiennych operacyjnych na 1
	}
	//Wywołanie przejścia między kolorami
	get_aLight(colorSwitch(colourPtr[colourIDR], colourPtr[colourIDRN], 40),0,n_led);
}

void iconGenerator(uint8_t x_1, uint8_t y_1, uint8_t x_2, uint8_t y_2, uint8_t r, uint8_t g, uint8_t b)
{
	/* Funkcja odpowiada za wyświetlenie koloru podanego w argumentach na polu matrycy wyznaczonym przez
	 * argumenty, które są współrzędnymi dwóch punktów matrycy
	 * */
	uint8_t iconColor[3] = {r,g,b}; //Deklaracja tablicy koloru na bazie argumentów
	for(int i = 1; i<=16; i++) //Przejście po kolumnach matrycy
	{
		if(i >= x_1 && i <= x_2) //Sprawdzenie czy w kolumnie znajduje się nowy kolor
		{
			uint16_t startLed; //Deklaracja ledu startującego
			if(i%2!=0) //Sprawdzenie czy kolumna jest nieparzysta
			{
				startLed = 16*(i-1) + (y_1-1); //przypisanie wartości ledu startującego
			}
			else
			{
				startLed = 16*(i-1) + (16-y_2); //przypisanie wartości ledu startującego
			}
			uint16_t ledNo = y_2 - y_1 + 1; //przypisanie ilości ledów do zapalenia w danej kolumnie
			get_aLight(iconColor,startLed,ledNo); //Zapalenie ledów
		}
	}
}

void iconMode() //Funkcja wyswietla animowaną ikonę
{
	if(somethingChanged) //Sprawdzenie czy zmienne operacyjne uległy zmianie
	{
		iconGenerator(1,1,16,16,0,0,0); //reset matrycy
	}
	static uint8_t iconTick = 10; //Zmienna odpowiadająca za długość wyświetlania jednej ikony
	static uint8_t iconToggle = 0; //Zmienna zmiany ikony
	iconTick ++;
	if(iconTick > 10) //Sprawdzenei czy nastąpiła pora zmiany ikony
	{
		iconTick = 0; //reset zmiennej Tick
		iconToggle = (iconToggle == 1) ? 0 : 1; //Zmiana zmiennej Toggle
		if(iconToggle == 0) //Wybór odpowiedniej ikony
		{
			iconGenerator(4,2,6,2,255,0,0); //Instrukcje odpowiadające ikonie
			iconGenerator(11,2,13,2,255,0,0);
			iconGenerator(3,3,7,3,255,0,0);
			iconGenerator(10,3,14,3,255,0,0);
			iconGenerator(2,4,15,7,255,0,0);
			iconGenerator(3,8,14,9,255,0,0);
			iconGenerator(4,10,13,11,255,0,0);
			iconGenerator(5,12,12,12,255,0,0);
			iconGenerator(6,13,11,13,255,0,0);
			iconGenerator(7,14,10,14,255,0,0);
			iconGenerator(8,15,9,15,255,0,0);
		}
		else
		{
			iconGenerator(1,1,16,16,0,0,0); //reset
			iconGenerator(5,3,6,3,255,0,0); //Instrukcje odpowiadające ikonie
			iconGenerator(11,3,12,3,255,0,0);
			iconGenerator(4,4,7,4,255,0,0);
			iconGenerator(10,4,13,4,255,0,0);
			iconGenerator(3,5,14,7,255,0,0);
			iconGenerator(4,8,13,9,255,0,0);
			iconGenerator(5,10,12,11,255,0,0);
			iconGenerator(6,12,11,12,255,0,0);
			iconGenerator(7,13,10,13,255,0,0);
			iconGenerator(8,14,9,14,255,0,0);
		}
	}
}

//Funkcje zmieniające parametry

void CMN() //Przechodzi na kolejny tryb
{
	bringback(); //Przywrócenie wcześniejszych ustawień
	modeID++; //zmiana
	checkArgs(); //sprawdzenie poprawności zakresu
	save(); //Zapisanie zmiany
	somethingChanged = 1; //Zmiana flagi zmiany na true
}

void CMP() //Przechodzi na poprzedni tryb
{
	bringback(); //Przywrócenie wcześniejszych ustawień
	modeID--; //zmiana
	checkArgs(); //sprawdzenie poprawności zakresu
	save(); //Zapisanie zmiany
	somethingChanged = 1; //Ustawienie flagi zmiany na prawde
}

void CCN() //Zmienia kolor na kolejny
{
	if(modeID!=4 && modeID!=5) //Sprawdzenie czy obecny tryb obsługuje zmianę koloru
	{
		bringback(); //Przywrócenie wcześniejszych ustawień
		colourID++; //zmiana
		checkArgs(); //sprawdzenie poprawności zakresu
		save(); //Zapisanie zmiany
		somethingChanged = 1; //Ustawienie flagi zmiany na prawde
	}
}

void CCP() //Zmienia kolor na poprzedni
{
	if(modeID!=4 && modeID!=5) //Sprawdzenie czy obecny tryb obsługuje zmianę koloru
	{
		bringback(); //Przywrócenie wcześniejszych ustawień
		colourID--; //zmiana
		checkArgs(); //sprawdzenie poprawności zakresu
		save(); //Zapisanie zmiany
		somethingChanged = 1; //Ustawienie flagi zmiany na prawde
	}
}

void CBU() //Zmienia jasność na stopień wyżej
{
	bringback(); //Przywrócenie wcześniejszych ustawień
	brightness += 0.2; //zmiana
	brightness = roundf(brightness * 100)/100; //zaokrąglenie do 2 miejsca po przecinku
	checkArgs(); //sprawdzenie poprawności zakresu
	save(); //Zapisanie zmiany
}

void CBD() //Zmienia jasność na stopień nieżej
{
	bringback(); //Przywrócenie wcześniejszych ustawień
	brightness -= 0.2; //zmiana
	brightness = roundf(brightness * 100)/100; //zaokrąglenie do 2 miejsca po przecinku
	checkArgs(); //sprawdzenie poprawności zakresu
	save(); //Zapisanie zmiany
}

void CSU() //Zmienia prędkość na stopień wyżej
{
	bringback(); //Przywrócenie wcześniejszych ustawień
	speed += 0.2; //zmiana
	speed = roundf(speed * 100)/100; //zaokrąglenie do 2 miejsca po przecinku
	checkArgs(); //sprawdzenie poprawności zakresu
	save(); //Zapisanie zmiany
}

void CSD() //Zmienia prędkość na stopień niżej
{
	bringback(); //Przywrócenie wcześniejszych ustawień
	speed -= 0.2; //zmiana
	speed = roundf(speed * 100)/100; //zaokrąglenie do 2 miejsca po przecinku
	checkArgs(); //sprawdzenie poprawności zakresu
	save(); //Zapisanie zmiany
}


//================================================LED ANIMACJE====================================================================

//\\\\\\\\\\\\\\\\\\\\\\\\\\FLASH/////////////////////////////

void eraseSector5() //Funkcja czyści sektor 5 FLASH
{
	HAL_FLASH_Unlock(); //Odblokowanie pamięci

	FLASH_EraseInitTypeDef EraseInitStruct; //Ustawienie parametrów czyszczenia
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.Sector = FLASH_SECTOR_5;
	EraseInitStruct.NbSectors = 1;

	uint32_t SectorError;
	volatile HAL_StatusTypeDef status;
	status = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError); //Czyszczenie pamięci sektora 5
	if(status == HAL_OK) //Sprawdzenie czy czyszczenie powiodło się
	{
		Send("Flash Complete"); //Poinformowanie o wyczyszczeniu
	}

	HAL_FLASH_Lock(); //Zablokowanie pamięci
}

void getNextFreeFlashWord() //Funkcja szuka pierwszej wolnej pary słów sektora 5 FLASH
{
	//Przeszukanie całego sektora parami słów (0x08040000 - 8 eliminuje możliwośc przypadkowego przekroczenia sektora)
	while(nextFreeFlashWord < (uint32_t*)(0x08040000 - 8) && *nextFreeFlashWord != 0xFFFFFFFF)
	{
		nextFreeFlashWord += 2; //Przejście na kolejną parę
	}

	//Sprawdzenie czy wyszedłem poza sektor, jeśli tak to wracam na początek przepisując ostatnie zmienne
	if(nextFreeFlashWord >= (uint32_t*)0x08040000 - 8)
	{
		getVarsFromFlash();
		eraseSector5();
		nextFreeFlashWord = (uint32_t*)0x08020000;
		SaveVarsToFlash();
		Send("ERASED!\r\n");
	}
}

void SaveVarsToFlash() //Funkcja zapisuje zmienne operacyjne do FLASH
{
	uint8_t brightnessT = brightness*100; //Przepisanie zmiennej brightness do uint8_t
	uint8_t speedT = speed*100; //Przepisanie zmiennej brightness do uint8_t

	uint32_t rar1 = 0; //Deklaracja zmiennych uint32_t do zapisu
	uint32_t rar2 = 0;

	//Przepisanie zmiennych operacyjnych uint8_t do zmiennej uint32_t
	rar1 = brightnessT | speedT << 8 | modeID << 16 | colourID << 24;

	HAL_FLASH_Unlock(); //Odblokowanie FLASH
	//Zapis zmiennych na odpowiednich miejscach
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)nextFreeFlashWord, rar1);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)(nextFreeFlashWord+1), rar2);
	HAL_FLASH_Lock(); //Zablokowanie pamięci

	nextFreeFlashWord += 2; //Przypisanie adresu kolejnego pustego miejsca

	//Sprawdzenie czy wyszedłem poza sektor, jeśli tak to wracam na początek przepisując ostatnie zmienne na początek
	if(nextFreeFlashWord >= (uint32_t*)(0x08040000 - 8))
	{
		eraseSector5();
		nextFreeFlashWord = (uint32_t*)0x08020000;
		SaveVarsToFlash();
		Send("ERASED!\r\n"); //Poinformowanie o wyczyszczeniu pamięci
	}
}

void getVarsFromFlash() //Funkcja odczytuje zmienne operacyjne z FLASH
{
	uint32_t rar1; //Deklaracja zmiennych uint32_t do odczytu
	//uint32_t rar2;

	uint8_t brightnessT; //Deklaracja tymczasowych zmiennych uint8_t
	uint8_t speedT;

	rar1 = *(nextFreeFlashWord-2); //Odczytanie zmiennych uint32_t z flash
	//rar2 = *(nextFreeFlashWord-1);

	//rodzielenie zmiennej uint32_t na 4 i przypisanie wartości do odpowiedników uint8_t
	colourID = rar1 >> 24;
	modeID = rar1 >> 16;
	speedT = rar1 >> 8;
	brightnessT = rar1;

	//konwersja uint8_t na float
	speed = (float)speedT/100;
	brightness = (float)brightnessT/100;
}

////////////////////////////FLASH\\\\\\\\\\\\\\\\\\\\\\\\\\\\\



//================================================Protokół komunikacyjny====================================================================

void communicationProtocol() //Funkcja odpowiedzialna za odnalezienie ramki
{
	uint8_t singleChar; //Zmienna pobierająca znaki z bufora
	static uint8_t errorTL = 0; //flaga kontroli wyjścia poza tablice
	static uint8_t gotFrame = 0; //Flaga stanu wykrycia ramki

    if(RX_BUSY != RX_EMPTY) //Sprawdzenie czy w buforze znajdują się elementy gotowe do odbioru
    {
        singleChar = buf_RX[RX_BUSY]; //Zapisanie do char'a obecnego elementu z bufora
        RX_BUSY++; //przejście na kolejne miejsce w buforze
        if(RX_BUSY>=buf_RX_length) //Sprawdzenie zakresu i ewentualne cofnięcie
        {
            RX_BUSY = 0;
        }

        if(gotFrame) //Sprawdzenie czy ramka została wykryta
        {
        	if(singleChar == ';') //Sprawdzenie czy char to nie ; (koniec ramki)
        	{
        		if(commandSlot >= 16) //Sprawdzenie czy polecenie jest wystarczająco długie
        		{
        			readFrame(); //Wywołanie funkcji readFrame()
        		}
        		commandSlot = 0; //Reset indeksu tablicy
        		errorTL = 0; //Ustawienie flagi zbyt długiej komendy na 0
        		gotFrame = 0; //Ustawienie flagi ramki na 0
        	}
        	else if(!errorTL) //Sprawdzenie czy komenda nie jest zbyt długa
        	{
                command[commandSlot] = singleChar; //Przepisanie odczytanego chara do tablicy
                commandSlot++; //Przejście do kolejnego miejsca tablicy
                if(commandSlot > 46) //Kontrola wyjścia poza zakres tablicy
                {
                	errorTL = 1; //Ustawienie flagi zbyt długiej komendy na 1
                	//gotFrame = 0;
                }
        	}
        }

        if(singleChar == '/') //Sprawdzenie czy znak to początek ramki
        {
        	commandSlot = 0; //Wyzerowanie indeksu
        	errorTL = 0; //Ustawienie flagi zbyt długiej komendy na 0
        	gotFrame = 1; //Ustawienie flagi wykrycia ramki na 1
        }
    }
}

void readFrame() //Funkcja sprawdzająca poprawność ramki
{
	//===========================\CRC/===============================

	if(!controlSumRX()) //Sprawdzenie sumy kontrolnej CRC32
	{
		return;
	}

	for(int e = 0; e <= commandSlot - 10; e++) //Usunięcie sumy kontrolnej z tablicy
	{
		command[e] = command[e+10];
	}
	commandSlot -= 10; //Skrócenie długości polecenia

	//===========================/CRC\===============================

	for(int e = 0; e < 3; e++) //Odczytanie nadawcy
	{
		returnFrame[e+3] = command[e];
	}
	for(int e = 0; e <= commandSlot - 3; e++) //Usunięcie nadawcy z tablicy
	{
		command[e] = command[e+3];
	}
	commandSlot -= 3; //Skrócenie długości polecenia

	char receiverStr[4]; //Deklaracja stringa z nazwą odbiorcy i przepisanie go
	for(int e=0; e<3; e++)
	{
		receiverStr[e]=command[e];
	}
	receiverStr[3]='\0';

	if(!strncmp(receiverStr, "STM", 3)) //Sprawdzenie czy odbiorca jest poprawny
	{
		for(int e = 0; e <= commandSlot - 3; e++) //Usunięcie odbiorcy z tablicy
		{
			command[e] = command[e+3];
		}
		commandSlot -= 3; //Skrócenie długości komendy

		readCommand(); // Wywołanie funkcji sprawdzającej polecenie

		returnFrame[0] = 'S'; //Ustawienie nadawcy w ramce zwrotnej
		returnFrame[1] = 'T';
		returnFrame[2] = 'M';

		//===========================\CRC/===============================


		uint32_t returnFramE[9]; //nowa tablica returnFrame 32 bitowa do wywołania funkcji crc calculate
		for(int e = 0; e<9; e++) //przepisanie wartości do nowej tablicy
		{
			returnFramE[e]=returnFrame[e];
		}
		uint32_t crcValue = HAL_CRC_Calculate(&hcrc, returnFramE, 9); //Obliczenie CRC32 MPEG-2 z tablicy pomocniczej
		Send("/0x%X%s;\r\n",crcValue,returnFrame); //Wysłanie ramki zwrotnej z CRC32


		//===========================/CRC\===============================
		//Send("/%s;\r\n",returnFrame); //Wysłanie ramki zwrotnej
	}
	else
	{
		return; //Opuszczenie funkcji w wypadku błędnego odbiorcy
	}

}

int controlSumRX() //Funkcja sprawdza poprawnośc przysłanego crc
{
	uint32_t crcValue; //Wartość CRC do obliczenia
	char crcRXArray[10]; //Tablica do przechowania sumy kontrolnej

	for(int e = 0; e < 10; e++) //Przepisanie sumy kontrolnej
	{
		crcRXArray[e] = command[e];
	}

	uint32_t crcRXValue = hex2int(crcRXArray); //Przepisanie wartości odczytanej sumy kontrolnej do zmiennej uint32_t

	uint32_t cmdArray[commandSlot-10]; //Deklaracja tablicy pomocnicznej do przechowania komendy
	for(int e = 0; e <=commandSlot-10 ; e++) //Przepisanie komendy do tablicy pomocniczej
	{
		cmdArray[e] = (uint32_t)command[e+10];
	}

	crcValue = HAL_CRC_Calculate(&hcrc, cmdArray, commandSlot-10); //Obliczenie CRC32 MPEG-2 dla komendy
	if(crcRXValue == crcValue) //Sprawdzenie czy suma odczytana i obliczona są zgodne
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void readCommand() //Funkcja sprawdzająca polecenie
{
	//zmienne wejścia
	int brightnessIn = 0; //od 1 do 100
	int speedIn = 0; // od 1 do 100

	static uint8_t exeMode = 0; //Flaga testowego trybu
	errorAS = 0; //Reset flagi wysłanego errora
	char lastChar; //Char do sprawdzania ostatniego znaku w scanf
	char nawias; //Char do sprawdzania czy argumenty kończą się poprawnie nawiasem

	char commandStr[commandSlot+1]; //String zawierający w sobie odebraną komende
	for(int e = 0; e<commandSlot; e++) //Uzupełnienie stringa commandStr o komendę
	{
		commandStr[e] = command[e];
	}
	commandStr[commandSlot] = '\0';

	if(!exeMode) //Sprawdzenie trybu pracy
	{
		if(compareCommand(commandStr, 0,"CMN")) //Sprawdzenie czy polecenie to CMN(następny tryb)
		{
			returnFrameAppend("EXE");
			CMN();
		}
		else if(compareCommand(commandStr, 0,"CMP")) //Sprawdzenie czy polecenie to CMP(poprzedni tryb)
		{
			returnFrameAppend("EXE");
			CMP();
		}
		else if(compareCommand(commandStr, 0,"CCN")) //Sprawdzenie czy polecenie to CCN(następny kolor)
		{
			returnFrameAppend("EXE");
			CCN();
		}
		else if(compareCommand(commandStr, 0,"CCP")) //Sprawdzenie czy polecenie to CCP(poprzedni kolor)
		{
			returnFrameAppend("EXE");
			CCP();
		}
		else if(compareCommand(commandStr, 0,"CBU")) //Sprawdzenie czy polecenie to CBU(jaśniej)
		{
			returnFrameAppend("EXE");
			CBU();
		}
		else if(compareCommand(commandStr, 0,"CBD")) //Sprawdzenie czy polecenie to CBD(ciemniej)
		{
			returnFrameAppend("EXE");
			CBD();
		}
		else if(compareCommand(commandStr, 0,"CSU")) //Sprawdzenie czy polecenie to CSU(szybciej)
		{
			returnFrameAppend("EXE");
			CSU();
		}
		else if(compareCommand(commandStr, 0,"CSD")) //Sprawdzenie czy polecenei to CSD(wolniej)
		{
			returnFrameAppend("EXE");
			CSD();
		}
		else if(compareCommand(commandStr, 0,"RES")) //Sprawdzenie czy polecenie to RES(reset do pierwotnych ustawień)
		{
			returnFrameAppend("EXE");
			init();
		}
		else if(compareCommand(commandStr, 0,"TEN")) //Sprawdzenie czy polecenie to TEN(exeMode = 1)
		{
			returnFrameAppend("EXE");
			exeMode = 1; //Zmiana trybu na testowy
			testInit(); //Przejście w tryb testowy
		}
		else if(compareCommand(commandStr, 1,"SEC(")) //Sprawdzenie czy polecenie to SEC(ustawienie koloru)
		{
			//Sprawdzenie poprawności argumentów i odczytanie ich
			if(sscanf(commandStr, "%d,%d,%d%c%c", &inputColour[0], &inputColour[1], &inputColour[2], &nawias, &lastChar)==4)
			{
				if(nawias == ')' ) //Sprawdzenie czy argumenty zostały poprawnie zamknięte
				{
					//Sprawdzenie zakresu argumentów
					if(inputColour[0] > 255 || inputColour[0] < 0 || inputColour[1] > 255 || inputColour[1] < 0 || inputColour[2] > 255 || inputColour[2] < 0)
					{
						returnFrameAppend("WRA"); //Poinformowanie o błędzie i wyzerowanie zmiennych
						inputColour[0] = 0, inputColour[1] = 0, inputColour[2] = 0;
					}
					else
					{
						returnFrameAppend("EXE");
						if(modeID!=4 && modeID!=5) //Sprawdzenie czy obecny tryb obsługuje zmiany koloru
						{
							colourID=0; //Zmianak colourID na 0 (wskaxnik na inputColour)
							somethingChanged = 1; //Ustawienie flagi zmiany na 1
						}
					}
				}
				else
				{
					returnFrameAppend("WRC");
					inputColour[0] = 0, inputColour[1] = 0, inputColour[2] = 0;
				}
			}
			else if(nawias == ')') //Sprawdzenie czy błąd wystąpił w argumentach
			{
				returnFrameAppend("WRC");
			}
			else
			{
				returnFrameAppend("WRA");
			}
		}
		else if(compareCommand(commandStr, 1,"SEB(")) //Sprawdzenie czy polecenie to SEB(ustawienie jasności)
		{
			//Sprawdzenie poprawności argumentów i odczytanie ich
			if(sscanf(commandStr, "%d%c%c", &brightnessIn, &nawias, &lastChar)==2)
			{
				if(nawias == ')' ) //Sprawdzenie czy argumenty zostały poprawnie zamknięte
				{
					//Sprawdzenie zakresu argumentów
					if(brightnessIn < 1 || brightnessIn > 100)
					{
						returnFrameAppend("WRA"); //Poinformowanie o błędzie i wyzerowanie zmiennych
					}
					else
					{
						returnFrameAppend("EXE");
						brightness = (float)brightnessIn/100;
					}
				}
				else
				{
					returnFrameAppend("WRC");
				}
			}
			else if(nawias == ')') //Sprawdzenie czy błąd wystąpił w argumentach
			{
				returnFrameAppend("WRC");
			}
			else
			{
				returnFrameAppend("WRA");
			}
		}
		else if(compareCommand(commandStr, 1,"SES(")) //Sprawdzenie czy polecenie to SES(ustawienie prędkości)
		{
			//Sprawdzenie poprawności argumentów i odczytanie ich
			if(sscanf(commandStr, "%d%c%c", &speedIn, &nawias, &lastChar)==2)
			{
				if(nawias == ')' ) //Sprawdzenie czy argumenty zostały poprawnie zamknięte
				{
					//Sprawdzenie zakresu argumentów
					if(speedIn < 1 || speedIn > 100)
					{
						returnFrameAppend("WRA"); //Poinformowanie o błędzie i wyzerowanie zmiennych
					}
					else
					{
						returnFrameAppend("EXE");
						speed = (float)speedIn / 100;
					}
				}
				else
				{
					returnFrameAppend("WRC");
				}
			}
			else if(nawias == ')') //Sprawdzenie czy błąd wystąpił w argumentach
			{
				returnFrameAppend("WRC");
			}
			else
			{
				returnFrameAppend("WRA");
			}
		}
		else if(!errorAS) //Sprawdzenie czy error został już wysłany
		{
			returnFrameAppend("WRC");
		}
	}
	else
	{
		if(compareCommand(commandStr, 0,"TEX")) //Sprawdzenie czy polecenie to TEX(exeMode = 0)
		{
			returnFrameAppend("EXE");
			exeMode = 0; //Zmiana trybu na normalny
			bringback(); //Odczytanie ostatnio zapisanych zmiennych operacyjnych
			somethingChanged = 1; //Ustawienie flagi zmiany na 1
		}
		else if(compareCommand(commandStr, 0,"TER")) //Sprawdzenie czy polecenie to TER(exeMode = 0 i reset)
		{
			returnFrameAppend("EXE");
			exeMode = 0; //Zmiana trybu na normalny
			init(); //Ustawienie defaultowych zmiennych operacyjnych
		}
		else if(compareCommand(commandStr, 1,"SEM[")) //Sprawdzenie czy polecenie to SEM(zmiana obszaru matrycy)
		{
			//Sprawdzenie poprawności argumentów i odczytanie ich
			if(sscanf(commandStr, "%d,%d][%d,%d](%d,%d,%d%c%c", &x, &y, &x2, &y2, &inputColour[0], &inputColour[1], &inputColour[2], &nawias, &lastChar)==8)
			{
				if(nawias == ')' ) //Sprawdzenie czy argumenty zostały poprawnie zamknięte
				{
					//Sprawdzenie zakresu argumentów
					if(x < 1 || x > 16 || x2 < 1 || x2 > 16 || y < 1 || y > 16 || y2 < 1 || y2 > 16 || inputColour[0] < 0 || inputColour[0] > 255 || inputColour[1] < 0 || inputColour[1] > 255 || inputColour[2] < 0 || inputColour[2] > 255)
					{
						returnFrameAppend("WRA");//Poinformowanie o błędzie i wyzerowanie zmiennych
						x = 0, x2 = 0, y = 0, y2 = 0, inputColour[0] = 0, inputColour[1] = 0, inputColour[2] = 0;
					}
					else
					{
						returnFrameAppend("EXE");
						somethingChanged = 1; //Ustawienie flagi zmiany na 1
					}
				}
				else
				{
					returnFrameAppend("WRC");
					x = 0, x2 = 0, y = 0, y2 = 0, inputColour[0] = 0, inputColour[1] = 0, inputColour[2] = 0;
				}
			}
			else if(nawias == ')') //Sprawdzenie czy błąd wystąpił w argumentach
			{
				returnFrameAppend("WRC");
			}
			else
			{
				returnFrameAppend("WRA");
			}
		}
		else if(!errorAS) //Sprawdzenie czy error został już wysłany
		{
			returnFrameAppend("WRC");
		}
	}
}

void returnFrameAppend(char str[]) //Funkcja uzupełnia tablice zwrotną o podane znaki
{
	returnFrame[6]=str[0];
	returnFrame[7]=str[1];
	returnFrame[8]=str[2];
}

int compareCommand(char commandStr[],int hasArgs, char str[]) //Funkcja sprawdzająca komende
{
	int strLen = strlen(str); //Długość szukanej komendy
	if(!strncmp(commandStr, str, strLen)) //Sprawdzenie czy polecenie to str
	{
		if(commandSlot != strLen && !hasArgs) //Sprawdzenie czy polecenie nie ma błędnej składni
		{
			returnFrameAppend("WRC"); //Powiadomienie o błędzie
			errorAS = 1; //Ustawienie flagi wysłanego errora na 1
			return 0;
		}

		for(int e=0; e<=commandSlot - strLen; e++) //Usunięcie odczytanego polecenia z tablicy w celu otrzymania samych argumentów
		{
			commandStr[e] = commandStr[e+strLen];
		}
		for(int e=commandSlot-strLen; e<commandSlot; e++) //reset reszty stringa
		{
			commandStr[e] = '\0';
		}
		commandSlot -= strLen; //skrócenie zmiennej długości polecenia
		return 1;
	}
	return 0;
}

uint32_t hex2int(char *hex) //Funkcja zwraca sumowaną wartość uint32_t dla znaków hex z argumentu
{
    uint32_t val = 0; //zwracana wartość uint32_t
    while (*hex) { //petla wywoływana dla każdego znaku z argumentu
        uint8_t byte = *hex++; //przejście wskaźnikiem na kolejny znak
        if (byte >= '0' && byte <= '9') byte = byte - '0'; // przeksztalcenie znaku ascii na jej odpowiednik wartości
        else if (byte >= 'a' && byte <='f') byte = byte - 'a' + 10;
        else if (byte >= 'A' && byte <='F') byte = byte - 'A' + 10;
        val = (val << 4) | (byte & 0xF); // dodanie wartości do zmiennej val
    }
    return val; //zwrócenie int 32 bitowego
}

int numOfDigits(long long num) //Funkcja obliczająca ilość cyfr w liczbie
{
    int count = 0;
    do
    {
        count++;
        num /= 10;
    } while(num != 0);
    return count;
}

//-------------------------UART-RX-CALLBACK-------------------------

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) //Callback wywoływany po odebraniu znaku
{
	if (huart -> Instance == USART2) //Sprawdzenie czy dane pochodzą z uart2
	{
		RX_EMPTY++; //Przejscie na kolejne miejsce w buforze RX
		if (RX_EMPTY >= buf_RX_length) //Sprawdzenie czy miejsce wychodzi poza limit bufora
		{
			RX_EMPTY = 0; //Ewentualne cofnięcie na poczatek bufora
		}
		HAL_UART_Receive_IT(&huart2, &buf_RX[RX_EMPTY], 1); //Wznowienie funkcji Receive_IT
	}
}

//-------------------------UART-TX-CALLBACK-------------------------

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) //Callback wywoływany po zakończeniu wysłania jednego znaku
{
	if (TX_BUSY != TX_EMPTY) //sprawdzenie czy są dostępne wolne dane do nadania
	{
		uint8_t tmp = buf_TX[TX_BUSY]; //przypisanie zmiennej tmp obecnej wartości z bufora
		TX_BUSY++; // przejście na kolejny element bufora

		if (TX_BUSY >= buf_TX_length) //Sprawdzenie czy miejsce wychodzi poza limit bufora
		{
			TX_BUSY = 0; //Ewentualne cofnięcie na początek bufora
		}
		HAL_UART_Transmit_IT(&huart2, &tmp, 1); //Wznowienie funkcji Transmit_IT
	}
}

//------------------SEND------------------

void Send(char* msg_to_send, ...) //Funkcja pobiera adresy do zmiennych typu char i uzupełnia bufor
{
	char data_to_send[100]; //Deklaracja tablicy pomocniczej oraz wskaźnika pomocniczego
	int indeks; //Deklaracja zmiennej wskazującej na TX_EMPTY

	//Skopoiowanie wszystkich argumentów do data_to_send
	va_list arguments;
	va_start(arguments, msg_to_send);
	vsprintf(data_to_send, msg_to_send, arguments);
	va_end(arguments); //Wyczyszczenie pamięci po va_list

	indeks = TX_EMPTY; //Przepisanei wartości TX_EMPTY

	for (int i = 0; i < strlen(data_to_send); i++) //Przepisanie danych do bufora
	{
		buf_TX[indeks] = data_to_send[i]; //Przepisanie znaku i do bufora na miejscu indeks
		indeks++; //Zwiększenie indeks o 1
		if (indeks >= buf_TX_length) //Sprawdzenie zakresu
		{
			indeks = 0;
		}
	}
	__disable_irq(); //Wyłączenie przerwań

	//Sprawdzenie czy wszystkie poprzednie znaki zostały wysłane i czy flaga TXE uart2 jest równa SET
	if ((TX_BUSY == TX_EMPTY) && (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == SET))
	{
		TX_EMPTY = indeks; //Przepisanie wartości indeks do EMPTY
		HAL_UART_Transmit_IT(&huart2, &buf_TX[TX_BUSY], 1); //Wznowienie funkcji Transmit_IT

		TX_BUSY++; //Zwiększenie BUSY o 1
		if (TX_BUSY >= buf_TX_length) //Sprawdzenie zakresu
		{
			TX_BUSY = 0;
		}
	}
	else
	{
		TX_EMPTY = indeks; //Przepisanie wartości indeks do EMPTY
	}
	__enable_irq(); //Włączenie przerwań
}


//================================================Protokół komunikacyjny====================================================================


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
