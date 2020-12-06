/* LEDC (LED Controller) fade example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/ledc.h"

#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"


#define TXD_PIN     (GPIO_NUM_27)
#define RXD_PIN     (GPIO_NUM_33)
#define FAN_PWM_OUT (GPIO_NUM_26)


#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (18)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0

#define MAX_FAN_DUTY         (4095)
#define HIGH_FAN_DUTY        (3060)
#define MID_FAN_DUTY         (2000)
#define MIN_FAN_DUTY         (1024)
#define LEDC_FADE_TIME       (1000)


static const int RX_BUF_SIZE = 1024;

static SemaphoreHandle_t xPMsem;
volatile unsigned long s_PM1,s_PM25,s_PM10;


ledc_channel_config_t ledc_channel;
ledc_timer_config_t ledc_timer;




static char Send_data[5] = {0x11,0x02,0x0B,0x07,0xDB}; // 농도읽는명령
unsigned char recv_cnt = 0;
 

void init() {
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);


    //ledc config
    ledc_timer.duty_resolution = LEDC_TIMER_12_BIT, // resolution of PWM duty
    ledc_timer.freq_hz = 5000,                      // frequency of PWM signal
    ledc_timer.speed_mode = LEDC_HS_MODE,           // timer mode
    ledc_timer.timer_num = LEDC_HS_TIMER,            // timer index
    ledc_timer.clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);
    ledc_channel.channel    = LEDC_HS_CH0_CHANNEL;
    ledc_channel.duty       = 0;
    ledc_channel.gpio_num   = LEDC_HS_CH0_GPIO;
    ledc_channel.speed_mode = LEDC_HS_MODE;
    ledc_channel.hpoint     = 0;
    ledc_channel.timer_sel  = LEDC_HS_TIMER;
    // Set LED Controller with previously prepared configuration
    ledc_channel_config(&ledc_channel);
    
    // Initialize fade service.
    ledc_fade_func_install(0);
    xPMsem = xSemaphoreCreateBinary();
    xSemaphoreGive(xPMsem);
}

static void tx_task()
{

    unsigned char i;
    
    while (1) {
        for(i=0;i<5;i++)
        {
            uart_write_bytes(UART_NUM_1,Send_data,5);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        printf("tx cmd send to sensor\r\n");
        vTaskDelay(950 / portTICK_PERIOD_MS);
    }
}
static void rx_task()
{   
    unsigned char count, SUM=0;
    
    unsigned long PM1, PM25, PM10;                                     // 농도저장변수 : 각 32bit(8bit*4 = 32)
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* Receive_Buff = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, Receive_Buff, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        if (rxBytes > 55) {
            for(count=0; count<55; count++)
            {
                SUM += Receive_Buff[count];
            }
            if(256-SUM != Receive_Buff[55])
                ESP_LOGE(RX_TASK_TAG,"checksum not matching!");
            
            

                PM1 = (unsigned long)Receive_Buff[3]<<24 | (unsigned long)Receive_Buff[4]<<16 | (unsigned long)Receive_Buff[5]<<8| (unsigned long)Receive_Buff[6];  // 농도계산(시프트)
                PM25 = (unsigned long)Receive_Buff[7]<<24 | (unsigned long)Receive_Buff[8]<<16 | (unsigned long)Receive_Buff[9]<<8| (unsigned long)Receive_Buff[10];  // 농도계산(시프트)
                PM10 = (unsigned long)Receive_Buff[11]<<24 | (unsigned long)Receive_Buff[12]<<16 | (unsigned long)Receive_Buff[13]<<8| (unsigned long)Receive_Buff[14];  // 농도계산(시프트)
                

                xSemaphoreTake(xPMsem,portMAX_DELAY);
                s_PM1 = PM1;
                s_PM25 = PM25;
                s_PM10 = PM10;
                xSemaphoreGive(xPMsem);

                printf("result :\tPM1.0 : %ld\tPM2.5 : %ld\tPM10 : %ld \r\n",PM1,PM25,PM10);

                SUM = 0;
            
            
               
            
        }
    }
    free(Receive_Buff);

}



static void ledc_task()
{
    unsigned int current_duty = 1024;
    unsigned long PM1, PM25, PM10;  
    while(1)
    {
        xSemaphoreTake(xPMsem,portMAX_DELAY);
            PM1  = s_PM1;
            PM25 = s_PM25;
            PM10 = s_PM10;
        xSemaphoreGive(xPMsem);


        if(PM10 >50 || PM25>35 || PM1>20)
        {
            printf("4. LEDC set duty = 100%% without fade\n");
            ledc_set_fade_time_and_start(ledc_channel.speed_mode,ledc_channel.channel,MAX_FAN_DUTY,LEDC_FADE_TIME,LEDC_FADE_NO_WAIT);
            current_duty = MAX_FAN_DUTY;
        }
        else if(PM10 >35 || PM25>25 || PM1>15)
        {
            printf("4. LEDC set duty = 75%% without fade\n");
            ledc_set_fade_time_and_start(ledc_channel.speed_mode,ledc_channel.channel,HIGH_FAN_DUTY,LEDC_FADE_TIME,LEDC_FADE_NO_WAIT);
            current_duty = HIGH_FAN_DUTY;
        }
        else if(PM10 >20 || PM25>15 || PM1>10)
        {
            printf("4. LEDC set duty = 50%% without fade\n");
            ledc_set_fade_time_and_start(ledc_channel.speed_mode,ledc_channel.channel,MID_FAN_DUTY,LEDC_FADE_TIME,LEDC_FADE_NO_WAIT);
            current_duty = MID_FAN_DUTY;
        }
        else
        {
            printf("4. LEDC set duty = 25%% without fade\n");
            ledc_set_fade_time_and_start(ledc_channel.speed_mode,ledc_channel.channel,MIN_FAN_DUTY,LEDC_FADE_TIME,LEDC_FADE_NO_WAIT);
            current_duty = MIN_FAN_DUTY;
        }
        printf("current fan duty is %d\r\n", current_duty);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    init();
    xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-2, NULL);
    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(ledc_task, "ledc_control_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    
}
