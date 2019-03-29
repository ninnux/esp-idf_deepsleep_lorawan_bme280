/*******************************************************************************
 * 
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x - BMP280
 * 
 * Copyright (c) 2018 Manuel Bleichenbacher
 * 
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * Sample program showing how to send and receive messages.
 *******************************************************************************/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "stdio.h"
#include "lmic/lmic.h"

#include "bmp280.h"

#include "TheThingsNetwork.h"

// NOTE:
// The LoRaWAN frequency and the radio chip must be configured by running 'make menuconfig'.
// Go to Components / The Things Network, select the appropriate values and save.

// Copy the below hex string from the "Device EUI" field
// on your device's overview page in the TTN console.
const char *devEui = CONFIG_devEui;

// Copy the below two lines from bottom of the same page
const char *appEui = CONFIG_appEui;
const char *appKey = CONFIG_appKey;
#define LMIC_DEBUG_LEVEL  

// Pins and other resources
#define TTN_SPI_HOST      HSPI_HOST
#define TTN_SPI_DMA_CHAN  1
#define TTN_PIN_SPI_SCLK  CONFIG_TTN_PIN_SPI_SCLK
#define TTN_PIN_SPI_MOSI  CONFIG_TTN_PIN_SPI_MOSI
#define TTN_PIN_SPI_MISO  CONFIG_TTN_PIN_SPI_MISO
#define TTN_PIN_NSS       CONFIG_TTN_PIN_NSS
#define TTN_PIN_RXTX      TTN_NOT_CONNECTED
#define TTN_PIN_RST       CONFIG_TTN_PIN_RST
#define TTN_PIN_DIO0      CONFIG_TTN_PIN_DIO0
#define TTN_PIN_DIO1      CONFIG_TTN_PIN_DIO1

#define SDA_GPIO (gpio_num_t)CONFIG_SCA_PIN 
#define SCL_GPIO (gpio_num_t)CONFIG_SCL_PIN 

static TheThingsNetwork ttn;

const unsigned TX_INTERVAL = 20;

static RTC_DATA_ATTR struct timeval sleep_enter_time;

SemaphoreHandle_t xSemaphore = NULL;
static uint8_t msgData[30] = "Hello, world";


void bmp280_status(void *pvParamters)
{
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    bmp280_t dev;
    float psum=0;
    float tsum=0;
    esp_err_t res;

    if( xSemaphore != NULL )
    {
       if( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
       {
	    while (i2cdev_init() != ESP_OK)
	    {
	        printf("Could not init I2Cdev library\n");
	        vTaskDelay(250 / portTICK_PERIOD_MS);
	    }

	    while (bmp280_init_desc(&dev, BMP280_I2C_ADDRESS_1, I2C_NUM_0 , SDA_GPIO, SCL_GPIO) != ESP_OK)
	    {
	        printf("Could not init device descriptor\n");
	        vTaskDelay(250 / portTICK_PERIOD_MS);
	    }

	    while ((res = bmp280_init(&dev, &params)) != ESP_OK)
	    {
	        printf("Could not init BMP280, err: %d\n", res);
	        vTaskDelay(250 / portTICK_PERIOD_MS);
	    }

	    bool bme280p = dev.id == BME280_CHIP_ID;
	    printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");

	    float pressure, temperature, humidity;
		int i=0;
	    while (i<10)
	    {	
		i++;
	        vTaskDelay(500 / portTICK_PERIOD_MS);
	        if (bmp280_read_float(&dev, &temperature, &pressure, &humidity) != ESP_OK)
	        {
	            printf("Temperature/pressure reading failed\n");
	            continue;
	        }

		psum+=pressure;
		tsum+=temperature;

	        printf("Pressure: %.2f Pa, Temperature: %.2f C", pressure, temperature);
	        if (bme280p)
	            printf(", Humidity: %.2f\n", humidity);
	        else
	            printf("\n");
	    }
	    int p=(psum/i*10)/100;
	    int t=(tsum/i*10);
	    sprintf((char*)msgData,"pres:%d,temp:%d",p,t);
	    xSemaphoreGive( xSemaphore );
	}
    }
    vTaskDelete( NULL );
}

void sleeppa(int sec)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_EXT1: {
            uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
            if (wakeup_pin_mask != 0) {
                int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
                printf("Wake up from GPIO %d\n", pin);
            } else {
                printf("Wake up from GPIO\n");
            }
            break;
        }
        case ESP_SLEEP_WAKEUP_TIMER: {
            printf("Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
            break;
        }
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            printf("Not a deep sleep reset\n");
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    const int wakeup_time_sec = sec;
    printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
    esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);

    const int ext_wakeup_pin_1 = 25;
    const uint64_t ext_wakeup_pin_1_mask = 1ULL << ext_wakeup_pin_1;
    const int ext_wakeup_pin_2 = 26;
    const uint64_t ext_wakeup_pin_2_mask = 1ULL << ext_wakeup_pin_2;

    printf("Enabling EXT1 wakeup on pins GPIO%d, GPIO%d\n", ext_wakeup_pin_1, ext_wakeup_pin_2);
    esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask | ext_wakeup_pin_2_mask, ESP_EXT1_WAKEUP_ANY_HIGH);

    // Isolate GPIO12 pin from external circuits. This is needed for modules
    // which have an external pull-up resistor on GPIO12 (such as ESP32-WROVER)
    // to minimize current consumption.
    //rtc_gpio_isolate(GPIO_NUM_12);

    printf("Entering deep sleep\n");
    gettimeofday(&sleep_enter_time, NULL);

    esp_deep_sleep_start();
}

void sendMessages(void* pvParameter)
{
    while (1) {
  	if( xSemaphore != NULL )
   	{
    	    if( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE ) {
	        printf("Sending message...\n");
	        TTNResponseCode res = ttn.transmitMessage(msgData, sizeof(msgData) - 1);
	        printf(res == kTTNSuccessfulTransmission ? "Message sent.\n" : "Transmission failed.\n");
		sleeppa(120);
                //vTaskDelay(TX_INTERVAL * 1000 / portTICK_PERIOD_MS);
		}
	}
    }
}

void messageReceived(const uint8_t* message, size_t length, port_t port)
{
    printf("Message of %d bytes received on port %d:", length, port);
    for (int i = 0; i < length; i++)
        printf(" %02x", message[i]);
    printf("\n");
}

extern "C" void app_main(void)
{
    esp_err_t err;
    // Initialize the GPIO ISR handler service
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    ESP_ERROR_CHECK(err);
	    
    vSemaphoreCreateBinary( xSemaphore );
    // Initialize the NVS (non-volatile storage) for saving and restoring the keys
    err = nvs_flash_init();
    ESP_ERROR_CHECK(err);

    // Initialize SPI bus
    spi_bus_config_t spi_bus_config;
    spi_bus_config.miso_io_num = TTN_PIN_SPI_MISO;
    spi_bus_config.mosi_io_num = TTN_PIN_SPI_MOSI;
    spi_bus_config.sclk_io_num = TTN_PIN_SPI_SCLK;
    spi_bus_config.quadwp_io_num = -1;
    spi_bus_config.quadhd_io_num = -1;
    spi_bus_config.max_transfer_sz = 0;
    err = spi_bus_initialize(TTN_SPI_HOST, &spi_bus_config, TTN_SPI_DMA_CHAN);
    ESP_ERROR_CHECK(err);
    // Configure the SX127x pins
    ttn.configurePins(TTN_SPI_HOST, TTN_PIN_NSS, TTN_PIN_RXTX, TTN_PIN_RST, TTN_PIN_DIO0, TTN_PIN_DIO1);

    // The below line can be commented after the first run as the data is saved in NVS
    ttn.provision(devEui, appEui, appKey);

    ttn.onMessage(messageReceived);
    printf("Joining...\n");
    LMIC.adrTxPow = 14;
    LMIC_setDrTxpow(DR_SF10, 14);
    LMIC.datarate = DR_SF10;
    if (ttn.join())
    {
        printf("Joined.\n");
        xTaskCreatePinnedToCore(bmp280_status, "bmp280_status", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
	
        xTaskCreate(sendMessages, "send_messages", 1024 * 4, (void* )0, 3, NULL);
    }
    else
    {
        printf("Join failed. Goodbye\n");
    }
}
