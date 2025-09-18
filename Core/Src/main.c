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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal_flash.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "si5351.h"
#include "stm32f1xx_hal_i2c.h"
#include "socket.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint32_t magic;          // Метка валидности (0xDEADBEEF)
    int32_t  data[3]; // Полезные данные
} FlashData;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FLASH_USER_START_ADDR  0x0801FC00  // Начало Page 127
#define ARRAY_SIZE 3           // Размер массива
#define FLASH_USER_START_ADDR  0x0801FC00  // Page 127
#define FLASH_MAGIC_NUMBER     0xDEADBEEF  // Метка валидности
//ethernet код
#define HTTP_SOCKET     0
#define PORT_TCPS        5000
#define DATA_BUF_SIZE 2048
uint8_t gDATABUF[DATA_BUF_SIZE];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

int32_t freq[3];//массив частот
int prev_encoder;//предыдущее значение энкодера
char choice;//переменная выбора, 0-энкодер не был нажат или был нажат чётное кол-во раз, 1- нечётное
char num_string [3][7];//массив частот в строках
uint8_t choiced_num;//порядковый номер выбранной цифры в значении частоты
uint8_t choiced_channel;//номер выбранного канала
char interface_mode;//0-интерфейс выбора канал, 1-интерфейс выбора частот
char is_ethernet_work;
uint8_t socket_status;
//ethernet код
wiz_NetInfo gWIZNETINFO={ .mac={0x00, 0x08, 0xDC, 0xAB, 0xCD, 0xEF},
	    .ip = {192, 168, 0, 100},                     // IP платы
	    .sn = {255, 255, 255, 0},                     // Маска подсети
	    .gw = {192, 168, 0, 1},                       // Шлюз (роутер)
	    .dns = {0, 0, 0, 0},                          // DNS (можно 0)
	    .dhcp = NETINFO_STATIC                        // Статический IP


};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
err_t si5351_set_frequency(uint8_t output, uint32_t frequency) {
    // Проверка допустимости выхода и частоты
    if (output > 2) return ERROR_INVALIDPARAMETER;
    if (frequency < 8000 || frequency > 150000000) return ERROR_INVALIDPARAMETER;

    // Определение R-делителя для частот < 500 кГц
    si5351RDiv_t r_div = SI5351_R_DIV_1;
    uint32_t r_div_value = 1;
    if (frequency < 500000) {
        uint32_t min_freq = frequency;
        while (min_freq < 500000 && r_div_value < 128) {
            r_div_value *= 2;
            min_freq = frequency * r_div_value;
        }
        switch (r_div_value) {
            case 2:   r_div = SI5351_R_DIV_2;   break;
            case 4:   r_div = SI5351_R_DIV_4;   break;
            case 8:   r_div = SI5351_R_DIV_8;   break;
            case 16:  r_div = SI5351_R_DIV_16;  break;
            case 32:  r_div = SI5351_R_DIV_32;  break;
            case 64:  r_div = SI5351_R_DIV_64;  break;
            case 128: r_div = SI5351_R_DIV_128; break;
            default:  r_div = SI5351_R_DIV_1;   break;
        }
    }

    // Расчет частоты для мультисинта (до R-делителя)
    double f_ms = (double)frequency * r_div_value;

    // Подбор делителя мультисинта (8-900)
    uint32_t div = (uint32_t)(800000000.0 / f_ms); // Целевой делитель для ~800 МГц
    if (div < 8) div = 8;
    if (div > 900) div = 900;

    // Расчет частоты PLL
    double f_pll = f_ms * div;
    if (f_pll < 600000000 || f_pll > 900000000) {
        // Корректировка при выходе за пределы 600-900 МГц
        div = (f_pll < 600000000) ? (uint32_t)ceil(600000000.0 / f_ms) : 900;
        f_pll = f_ms * div;
    }

    // Настройка PLL
    double f_xtal = (double)m_si5351Config.crystalFreq;
    uint32_t mult = (uint32_t)(f_pll / f_xtal);
    double fraction = (f_pll / f_xtal) - mult;
    uint32_t num = (uint32_t)round(fraction * 1048575.0); // 20-битный числитель
    uint32_t denom = 1048575; // 20-битный знаменатель

    // Выбор PLL: выход 2 → PLL_B, остальные → PLL_A
    si5351PLL_t pll = (output == 2) ? SI5351_PLL_B : SI5351_PLL_A;

    // Применение настроек
    ASSERT_STATUS(si5351_setupPLL(pll, mult, num, denom));
    ASSERT_STATUS(si5351_setupMultisynth(output, pll, div, 0, 1)); // Целочисленный режим
    ASSERT_STATUS(si5351_setupRdiv(output, r_div));

    return ERROR_NONE;
}

void int_to_str(int num, char *str) {
    char tmp[12]; // Временный буфер
    int i = 0;

    // Обрабатываем 0 отдельно
    if (num == 0) {
        str[0] = '0';
        str[1] = '\0';
        return;
    }

    // Записываем цифры в обратном порядке
    while (num > 0) {
        tmp[i++] = '0' + (num % 10);
        num /= 10;
    }

    // Разворачиваем строку
    int j = 0;
    while (i-- > 0) {
        str[j++] = tmp[i];
    }
    str[j] = '\0';
}

char Is_Flash_Valid() {
    // Чтение метки из Flash
    uint32_t magic = *(__IO uint32_t*)FLASH_USER_START_ADDR;
    if(magic == FLASH_MAGIC_NUMBER){
    	return 1;
    }else{
    	return 0;
    }
    //return (magic == FLASH_MAGIC_NUMBER);
}

void Read_Flash_Array(int32_t *output) {
    if (!Is_Flash_Valid()) {
        // Данные не валидны (первый запуск)
        memset(output, 0, ARRAY_SIZE * sizeof(int32_t));
        return;
    }

    // Чтение данных (пропускаем метку)
    FlashData *flash_data = (FlashData*)FLASH_USER_START_ADDR;
    memcpy(output, flash_data->data, ARRAY_SIZE * sizeof(int32_t));
}

void Write_Flash_Array(int32_t *data) {
    HAL_FLASH_Unlock();

    // Стирание страницы
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = FLASH_USER_START_ADDR;
    erase.NbPages = 1;

    uint32_t page_error;
    HAL_FLASHEx_Erase(&erase, &page_error);

    // Запись структуры (метка + данные)
    FlashData flash_data;
    flash_data.magic = FLASH_MAGIC_NUMBER;
    memcpy(flash_data.data, data, ARRAY_SIZE * sizeof(int32_t));

    // Запись по 16-битным полусловам
    uint32_t addr = FLASH_USER_START_ADDR;
    uint32_t *ptr = (uint32_t*)&flash_data;
    uint32_t size = sizeof(FlashData) / 2; // Количество 16-битных слов

    for (uint32_t i = 0; i < size; i++) {
        uint32_t value = ptr[i];
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, value & 0xFFFF);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + 2, (value >> 16) & 0xFFFF);
        addr += 4;
    }

    HAL_FLASH_Lock();
}


int get_encoder(){
	return (int)TIM1->CNT/4;//для энкодера использующегося в проекте
}

void set_encoder(int e){
	TIM1->CNT=e*4;//для энкодера использующегося в проекте
}


void print_interface_mode0(){
	//далее 3 строки обычного вывода на дисплей строки
	ssd1306_SetCursor(1, 1);//установка курсора
	ssd1306_Fill(Black);//заполнение экрана чёрным(типа стирание)
	ssd1306_WriteString(num_string[0], Font_7x10, White);//отправка строки
	//далее выводятся строки, чтобы частоты каналов отображались корректно
	ssd1306_WriteString("   ", Font_7x10, White);
	ssd1306_WriteString(num_string[1], Font_7x10, White);
	ssd1306_SetCursor(1, 10);//для переноса на следующую строку
	ssd1306_WriteString(num_string[2], Font_7x10, White);
	ssd1306_WriteString("   ", Font_7x10, White);
	if(choiced_channel==0){
		ssd1306_WriteString("ch0", Font_7x10, White);
	}else if(choiced_channel==1){
		ssd1306_WriteString("ch1", Font_7x10, White);
	}else{
		ssd1306_WriteString("ch2", Font_7x10, White);
	}
	ssd1306_UpdateScreen();//самая важная функция, без которой что было отправлено на дисплей не отобразится
}

void print_interface_mode1(){
	//смысл такой-же, как и в mode0, но здесь цифра большего размера показывает, что она выбрана, а ^ указывает на изменение этой цифры
	ssd1306_SetCursor(1, 1);
	ssd1306_Fill(Black);
	if(choiced_num==0){
		uint16_t size=strlen(num_string[choiced_channel]);
		for(int i=size;i<6;++i){
			ssd1306_WriteString("0", Font_7x10, White);
		}
		ssd1306_WriteString(num_string[choiced_channel], Font_7x10, White);


		ssd1306_WriteString("   ", Font_7x10, White);
		ssd1306_WriteString("send", Font_11x18, White);
	}else{
		uint16_t size=strlen(num_string[choiced_channel]);
		for(int i=5;i>size-1;--i){
			if(i==choiced_num-1){
				ssd1306_WriteString("0", Font_11x18, White);
			}else{
				ssd1306_WriteString("0", Font_7x10, White);
			}
		}
		for(int i=0;i<size;++i){
			if(i==size-choiced_num){

				char str[2] = {num_string[choiced_channel][i], '\0'};
				ssd1306_WriteString(str, Font_11x18, White);
			}else{

				char str[2] = {num_string[choiced_channel][i], '\0'};
				ssd1306_WriteString(str, Font_7x10, White);
			}
		}
		ssd1306_WriteString("   ", Font_7x10, White);
		ssd1306_WriteString("send", Font_7x10, White);
		if(choice==1){
			ssd1306_SetCursor(1+((6-choiced_num)*7), 18);
			ssd1306_WriteString("^", Font_7x10, White);
		}


	}
	//проверка пределов значений частот и соответствуещие напоминания об этом
	if(freq[choiced_channel]==160000){
		ssd1306_SetCursor(77, 18);
		ssd1306_WriteString("MAX", Font_7x10, White);
	}else if(freq[choiced_channel]==8){
		ssd1306_SetCursor(77, 18);
		ssd1306_WriteString("MIN", Font_7x10, White);
	}
	ssd1306_UpdateScreen();
}

void int_mode_0(){

	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {  // Если кнопка нажата (подтяжка к VCC)
		choice=1;
	}
	while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET);  // Ждём отпускания
	if(choice==0){
		//кнопка не нажата, смотрим значение энкодера и его остаток от деления даёт выбранный канал
		choiced_channel=get_encoder()%3;
		print_interface_mode0();
	}else if(choice==1){
		//был выбран канал и нажата кнопка, преключаем интерфейс на редактирование частоты
		set_encoder(1);
		choiced_num=1;//потому что есть send который будем считать за 0 положение
		interface_mode=1;
		choice=0;
		print_interface_mode1();
	}

}


//функция выставления минимальной частоты
void min_freq(){
	num_string[choiced_channel][0]='8';
	num_string[choiced_channel][1]='\0';
}
//функция выставления максимальной частоты
void max_freq(){
	num_string[choiced_channel][0]='1';
	num_string[choiced_channel][1]='6';
	num_string[choiced_channel][2]='0';
	num_string[choiced_channel][3]='0';
	num_string[choiced_channel][4]='0';
	num_string[choiced_channel][5]='0';
	num_string[choiced_channel][6]='\0';

}

void int_mode_1(){
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {  // Если кнопка нажата (подтяжка к VCC)
		//поменялось с 0 на 1 значит начали редактирование цифры, а если был выбран send(0), то происходит отправка частоты и изменение интерфейса на mode0
		if(choice==0){

			choice=1;
			if(choiced_num!=0){
				prev_encoder=1000;
				set_encoder(1000);

			}
		}else{
			//закончили редактирование цифры, устанавливаем энкодер на выбранную цифру(номер)
			choice=0;
			set_encoder(choiced_num);
		}

	}
	while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET);  // Ждём отпускания
	if(choice==0){
		choiced_num=get_encoder()%7;
		print_interface_mode1();
	}else if(choice==1){
		if(choiced_num==0){
			//отправка частоты
			choice=0;
			interface_mode=0;

			si5351_set_frequency(choiced_channel, freq[choiced_channel]*1000);
			si5351_enableOutputs(0xFF);

			Write_Flash_Array(freq);//обновляем значение в памяти
			set_encoder(choiced_channel);
			print_interface_mode0();
		}else{
			//изменение цифры в значении частоты
			int delta = get_encoder();//минимизируем вызов функции

			freq[choiced_channel]+=(delta-prev_encoder)*pow(10,choiced_num-1);//сразу меняем частоту при изменении энкодера
			if(freq[choiced_channel]<0){
				freq[choiced_channel]=160000+freq[choiced_channel];//если частота очень мала делаем её почти максимальной
			}else if(freq[choiced_channel]>999999){
				freq[choiced_channel]=(7+(delta-prev_encoder)*pow(10,choiced_num-1));//если частота очень большая, то делаем её почти минимальной
			}
			prev_encoder=delta;
			//проверяем пределы
			if(freq[choiced_channel]<8){
				freq[choiced_channel]=8;
				min_freq();
			}else if(freq[choiced_channel]>160000){
				freq[choiced_channel]=160000;
				max_freq();
			}
			int_to_str(freq[choiced_channel],num_string[choiced_channel]);//обновляем строку с выбранной частотой
			print_interface_mode1();

		}
	}
}
uint32_t str_to_int(char* str){
    uint32_t ans = 0;
    uint8_t i = 0;

    // Обрабатываем цифры
    while(str[i] >= '0' && str[i] <= '9'){
        ans = ans * 10 + (str[i] - '0');
        i++;
    }

    return ans;
}
//ethernet код
void W5500_Select(void){
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
}

void W5500_Unselect(void){
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

void W5500_ReadBuff(uint8_t* buff, uint16_t len){
	HAL_SPI_Receive(&hspi1,buff,len,HAL_MAX_DELAY);
}

void W5500_WriteBuff(uint8_t* buff,uint16_t len){
	HAL_SPI_Transmit(&hspi1,buff,len,HAL_MAX_DELAY);
}

uint8_t W5500_ReadByte(void){
	uint8_t byte;
	W5500_ReadBuff(&byte, sizeof(byte));
	return byte;
}

void W5500_WriteByte(uint8_t byte){
	W5500_WriteBuff(&byte, sizeof(byte));
}

uint8_t stat;
uint8_t reqnr;
char Message[128];
// Функция обработки клиентского подключения
void process_client_connection(uint8_t sn)
{
    int32_t received_len;
    uint8_t received_data[1024];



    // Главный цикл обработки соединения
    while(getSn_SR(sn) == SOCK_ESTABLISHED)
    {
        // Проверяем есть ли данные для чтения
        uint16_t available = getSn_RX_RSR(sn);

        if(available > 0)
        {
            // Читаем данные
            received_len = recv(sn, received_data, sizeof(received_data)-1);

            if(received_len > 0)
            {
            	received_data[received_len] = '\0';
                if(strstr((char*)received_data, "EXIT") != NULL)
                {
                    send(sn, (uint8_t*)"Goodbye!\r\n", 10);

                    break;
                }

                uint32_t r=str_to_int((char*)received_data);
                if(r>7 && r<=160000){
                	freq[choiced_channel]=r;
                }
                int_to_str(freq[choiced_channel], num_string[choiced_channel]);
                si5351_set_frequency(choiced_channel, freq[choiced_channel]*1000);//устанвливаем частоту введённую через ethernet
                si5351_enableOutputs(0xFF);//включаем все выходы
                Write_Flash_Array(freq);
                print_interface_mode0();
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                send(sn, (uint8_t*)"OK\r\n", 4);


            }
        }
  	  uint32_t t=HAL_GetTick();

  	  while(t>HAL_GetTick()-10);
        //HAL_Delay(10); // Небольшая задержка
    }

}
// Функция инициализации сервера
uint8_t init_server(uint8_t sn, uint16_t port)
{
    // Закрываем сокет если был открыт
    if(getSn_SR(sn) != SOCK_CLOSED) {
        close(sn);
        //HAL_Delay(100);
    }

    // Создаем сокет
    if((stat = socket(sn, Sn_MR_TCP, port, 0)) != sn) {

        return 0;
    }

    // Слушаем порт
    if((stat = listen(sn)) != SOCK_OK) {

        close(sn);
        return 0;
    }

    return 1;
}
void ethernet_work(){
	// Мигание светодиодом в режиме ожидания
	          static uint32_t led_timer = 0;
	          if(HAL_GetTick() - led_timer > 500) {
	              HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	              led_timer = HAL_GetTick();
	          }

	          // Проверяем статус сокета
	          socket_status = getSn_SR(HTTP_SOCKET);

	          switch(socket_status)
	          {
	              case SOCK_LISTEN:
	                  // Ожидаем подключения
	            	  uint32_t t=HAL_GetTick();
	            	  while(t>HAL_GetTick()-100);
	                  //HAL_Delay(100);
	                  break;

	              case SOCK_ESTABLISHED:
	                  // Клиент подключен - обрабатываем
	            	  //is_ethernet_work=1;
	            	  if(interface_mode==0){
	                  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // LED ON

	                  process_client_connection(HTTP_SOCKET);
	                  disconnect(HTTP_SOCKET);
	                  close(HTTP_SOCKET);
	                  // Переинициализируем сервер
	                  init_server(HTTP_SOCKET, 80);
	            	  }
	                  break;


	              case SOCK_CLOSED:
	                  // Сервер не запущен - пытаемся перезапустить
	                  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // LED OFF
	                  init_server(HTTP_SOCKET, 80);
	                  //HAL_Delay(1000);
	                  break;

	              default:
	                  // Неизвестный статус - перезапускаем

	                  close(HTTP_SOCKET);
	                  init_server(HTTP_SOCKET, 80);
	                  //HAL_Delay(1000);
	                  break;
	          }

	          //HAL_Delay(10);
}
//функция для работы программы во время прерываний
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
    	//uint8_t socket_status = getSn_SR(HTTP_SOCKET);
    	if(interface_mode==0 && socket_status!=SOCK_ESTABLISHED){
    		//ethernet_work();
    		int_mode_0();
    	}else if(interface_mode==1){
    		int_mode_1();
    	}
    }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */






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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  ssd1306_Init();
  si5351_Init();

  set_encoder(0);//выставление энкодера в 0
      /*freq[0]=8;//начальная минимальная частота канала 0
      freq[1]=8;//начальная минимальная частота канала 1
      freq[2]=8;//начальная минимальная частота канала 2
      */
      if (Is_Flash_Valid()==0) {
          Write_Flash_Array(freq);
      }else{
    	  Read_Flash_Array(freq);
      }
      choice=0;//переменная для считывания был ли нажат энкодер
      //записываем частоты в строки для них
      int_to_str(freq[2],num_string[2]);
      int_to_str(freq[1],num_string[1]);
      int_to_str(freq[0],num_string[0]);
      choiced_num=0;//переменная для определения выбранной цифры в массиве частоты
      choiced_channel=0;// номер выбранного канала
      interface_mode=0;//переменная для определения что должно показыватиься на экране(0-значения частот, 1-редактирование частоты)
      prev_encoder=8;
      print_interface_mode0();
      HAL_TIM_Base_Start_IT(&htim2);  // Запуск таймера с прерыванием
      // Инициализация W5500
            reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);
            reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);
            reg_wizchip_spiburst_cbfunc(W5500_ReadBuff, W5500_WriteBuff);
            //ssd1306_Init();
            uint8_t rx_tx_buff_sizes[] = {2, 2, 2, 2, 2, 2, 2, 2};
            wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes);
            wizchip_setnetinfo(&gWIZNETINFO);
            ctlnetwork(CN_SET_NETINFO, (void*)&gWIZNETINFO);
            //is_ethernet_work=0;
            //HAL_Delay(1000);



  si5351_set_frequency(0, freq[0]*1000);//устанвливаем частоту в минимальную
  si5351_set_frequency(1, freq[1]*1000);//устанвливаем частоту в минимальную
  si5351_set_frequency(2, freq[2]*1000);//устанвливаем частоту в минимальную
  si5351_enableOutputs(0xFF);//включаем все выходы



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  ethernet_work();

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
