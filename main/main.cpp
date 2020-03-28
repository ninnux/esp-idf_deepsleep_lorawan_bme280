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
#include "driver/gpio.h"
#include "driver/i2c.h"
extern "C" {
//#include "bmp280.h"
#include "bme280.h"
}

#include "TheThingsNetwork.h"
#include "../src/lmic/oslmic.h"
#include "../src/lmic/lmic.h"
#include "../src/hal/hal_esp32.h"
#include "../src/lmic/lmic_eu_like.h"

#include "cayenne_lpp.h"
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
//#define SDA_GPIO (gpio_num_t) 16
//#define SCL_GPIO (gpio_num_t) 17

#define TAG_BME280 "BME280"

#define I2C_MASTER_ACK (i2c_ack_type_t) 0
#define I2C_MASTER_NACK (i2c_ack_type_t) 1

#define TIMESLOT 5 
#define SLEEP_INTERVAL 300
static TheThingsNetwork ttn;

const unsigned TX_INTERVAL = 20;

static RTC_DATA_ATTR struct timeval sleep_enter_time;
RTC_DATA_ATTR u4_t RTCnetid;
RTC_DATA_ATTR u4_t RTCdevaddr;
RTC_DATA_ATTR u1_t RTCnwkKey[16];
RTC_DATA_ATTR u1_t RTCartKey[16];
RTC_DATA_ATTR int RTCseqnoUp;
RTC_DATA_ATTR int RTCseqnoDn;
RTC_DATA_ATTR int deepsleep=0;
RTC_DATA_ATTR int counter=0;

float t;
float p;
float h;

extern "C"{
static void _print_buffer(cayenne_lpp_t *lpp)
{
    printf("buffer di %d bytes:",lpp->cursor);
    uint8_t i=0;
    for (i = 0; i < lpp->cursor; ++i) {
	printf("%d,",lpp->buffer[i]);
    }
    printf("\n");

}

}

SemaphoreHandle_t xSemaphore = NULL;
SemaphoreHandle_t txSemaphore = NULL;
SemaphoreHandle_t tSemaphore = NULL;
SemaphoreHandle_t hSemaphore = NULL;
SemaphoreHandle_t pSemaphore = NULL;
static uint8_t msgData[50] = "Hello, world";
//static uint8_t msgData[1024] = "Hello, world";
 
//RTC_DATA_ATTR uint8_t rtc_buffer[128];
//RTC_DATA_ATTR char rtc_buffer[1024];
//RTC_DATA_ATTR int rtc_buffer_len=0;

RTC_DATA_ATTR cayenne_lpp_t tlpp = { 0 };
RTC_DATA_ATTR cayenne_lpp_t hlpp = { 0 };
RTC_DATA_ATTR cayenne_lpp_t plpp = { 0 };

//void bmp280_status(void *pvParamters)
//{
//    bmp280_params_t params;
//    bmp280_init_default_params(&params);
//    bmp280_t dev;
//    float psum=0;
//    float tsum=0;
//    float hsum=0;
//    esp_err_t res;
//
//    if( xSemaphore != NULL )
//    {
//       if( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
//       {
//	    while (i2cdev_init() != ESP_OK)
//	    {
//	        printf("Could not init I2Cdev library\n");
//	        vTaskDelay(250 / portTICK_PERIOD_MS);
//	    }
//
//	    while (bmp280_init_desc(&dev, BMP280_I2C_ADDRESS_0, I2C_NUM_0 , SDA_GPIO, SCL_GPIO) != ESP_OK)
//	    {
//	        printf("Could not init device descriptor\n");
//	        vTaskDelay(250 / portTICK_PERIOD_MS);
//	    }
//
//	    while ((res = bmp280_init(&dev, &params)) != ESP_OK)
//	    {
//	        printf("Could not init BMP280, err: %d\n", res);
//	        vTaskDelay(250 / portTICK_PERIOD_MS);
//	    }
//
//	    bool bme280p = dev.id == BME280_CHIP_ID;
//	    printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");
//
//	    float pressure, temperature, humidity;
//	    int i=0;
//	    bool busy;
//	    while (i<10) // scaldo il sensore con 10 letture a vuoto
//	    {	
//	        i++;
//	        vTaskDelay(500 / portTICK_PERIOD_MS);
//		// force mode
//		bmp280_force_measurement(&dev);
//		do { bmp280_is_measuring(&dev, &busy); } while(busy);	
//		//
//	        if (bmp280_read_float(&dev, &temperature, &pressure, &humidity) != ESP_OK)
//	        {
//	            printf("Temperature/pressure reading failed\n");
//	            continue;
//	        }
//
//	        psum+=pressure;
//	        tsum+=temperature;
//
//	        printf("Pressure: %.2f Pa, Temperature: %.2f C", pressure, temperature);
//	        if (bme280p){
//	            printf(", Humidity: %.2f\n", humidity);
//	            hsum+=humidity;
//	        }
//	        else{
//	            printf("\n");
//	        }
//	    }
//	    i=0;
//	    pressure=0;
//            temperature=0;
//            humidity=0;
//	    psum=0;
//            tsum=0;
//            hsum=0;
//	    while (i<10)
//	    {	
//		i++;
//	        vTaskDelay(500 / portTICK_PERIOD_MS);
//	        if (bmp280_read_float(&dev, &temperature, &pressure, &humidity) != ESP_OK)
//	        {
//	            printf("Temperature/pressure reading failed\n");
//	            continue;
//	        }
//
//		psum+=pressure;
//		tsum+=temperature;
//
//	        printf("Pressure: %.2f Pa, Temperature: %.2f C", pressure, temperature);
//	        if (bme280p){
//	            printf(", Humidity: %.2f\n", humidity);
//		    hsum+=humidity;
//		}
//	        else{
//	            printf("\n");
//		}
//	    }
//	    //int p=(psum/i*10)/100;
//	    //int t=(tsum/i*10);
//	    if (bme280p){
//	    	//int h=(hsum/i*10);
//	      p=psum/i/100;
//	      t=tsum/i;
//	      h=hsum/i;
//	    }else{
//	      p=psum/i;
//	      t=tsum/i;
//	      h=hsum/i;
//	    }
//	    xSemaphoreGive( xSemaphore );
//	}
//    }
//    vTaskDelete( NULL );
//}



extern "C" void i2c_master_init()
{
	//i2c_config_t i2c_config = {
	//	.mode = I2C_MODE_MASTER,
	//	.sda_io_num = SDA_GPIO,
	//	.sda_pullup_en = GPIO_PULLUP_ENABLE,
	//	.scl_io_num = SCL_GPIO,
	//	.scl_pullup_en = GPIO_PULLUP_ENABLE,
	//	.master.clk_speed = 1000000
	//};
	i2c_config_t i2c_config;
	i2c_config.mode = I2C_MODE_MASTER;
	i2c_config.sda_io_num = SDA_GPIO;
	i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
	i2c_config.scl_io_num = SCL_GPIO;
	i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
	i2c_config.master.clk_speed = 1000000;
	i2c_param_config(I2C_NUM_0, &i2c_config);
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

extern "C" s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BME280_INIT_VALUE;

	esp_err_t espRc;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

	i2c_master_write_byte(cmd, reg_addr, true);
	i2c_master_write(cmd, reg_data, cnt, true);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		iError = SUCCESS;
	} else {
		iError = FAIL;
	}
	i2c_cmd_link_delete(cmd);

	return (s8)iError;
}

extern "C" s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BME280_INIT_VALUE;
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg_addr, true);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

	if (cnt > 1) {
		i2c_master_read(cmd, reg_data, cnt-1, I2C_MASTER_ACK);
	}
	i2c_master_read_byte(cmd, reg_data+cnt-1, I2C_MASTER_NACK);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		iError = SUCCESS;
	} else {
		iError = FAIL;
	}

	i2c_cmd_link_delete(cmd);

	return (s8)iError;
}

void BME280_delay_msek(u32 msek)
{
	vTaskDelay(msek/portTICK_PERIOD_MS);
}

extern "C" void task_bme280_normal_mode(void *ignore)
{
   //struct bme280_t bme280 = {
   //	.bus_write = BME280_I2C_bus_write,
   //	.bus_read = BME280_I2C_bus_read,
   //	.dev_addr = CONFIG_BME280_ADDRESS,
   //	.delay_msec = BME280_delay_msek
   //};
   struct bme280_t bme280;
   bme280.bus_write = BME280_I2C_bus_write;
   bme280.bus_read = BME280_I2C_bus_read;
   bme280.dev_addr = CONFIG_BME280_ADDRESS;
   bme280.delay_msec = BME280_delay_msek;

   int i=0;
   //int h=0;
   //int t=0;
   //int p=0;
   float hsum=0;
   float tsum=0;
   float psum=0;

   s32 com_rslt;
   s32 v_uncomp_pressure_s32;
   s32 v_uncomp_temperature_s32;
   s32 v_uncomp_humidity_s32;
   if( xSemaphore != NULL )
   {
       if( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
       {

	com_rslt = bme280_init(&bme280);

	com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_16X);
	com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_2X);
	com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);

	com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);
	com_rslt += bme280_set_filter(BME280_FILTER_COEFF_16);

	//com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE);
	com_rslt += bme280_set_power_mode(BME280_FORCED_MODE);
	if (com_rslt == SUCCESS) {
	 i=0;
	 for(i=0;i<10;i++){
	   vTaskDelay(40/portTICK_PERIOD_MS);
	   com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
	   	&v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);
	   if (com_rslt == SUCCESS) {
	     hsum=bme280_compensate_humidity_double(v_uncomp_humidity_s32);
	     psum=bme280_compensate_pressure_double(v_uncomp_pressure_s32);
	     tsum=bme280_compensate_temperature_double(v_uncomp_temperature_s32);
	   } else {
	     printf("measure error. code: %d", com_rslt);
	   }
	 }
	 hsum=0;
         psum=0;
         tsum=0;
	 for(i=0;i<10;i++){
	   vTaskDelay(40/portTICK_PERIOD_MS);
	   com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
	   	&v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

	   if (com_rslt == SUCCESS) {
	     hsum+=bme280_compensate_humidity_double(v_uncomp_humidity_s32);
	     psum+=bme280_compensate_pressure_double(v_uncomp_pressure_s32)/100;
	     tsum+=bme280_compensate_temperature_double(v_uncomp_temperature_s32);
	     //ESP_LOGI(TAG_BME280, "%.2f degC / %.3f hPa / %.3f %%",
	     //bme280_compensate_temperature_double(v_uncomp_temperature_s32),
	     //bme280_compensate_pressure_double(v_uncomp_pressure_s32)/100, // Pa -> hPa
	     //bme280_compensate_humidity_double(v_uncomp_humidity_s32));
	   } else {
	     printf("measure error. code: %d", com_rslt);
	   }
	 }
	 h=hsum/i;
	 p=psum/i;
	 t=tsum/i;
	 //printf("hum:%d,temp:%d,pres:%d\n",h,t,p);
	 //sprintf((char*)msgData,"{\"hum\":%d,\"temp\":%d,\"pres\":%d}",h,t,p);
	 //printf("%s",msgData);
   	//if(counter%TIMESLOT!=0){
  	// char* keys[]={"pres","temp","hum"}; 
  	// int values[]={p,t,h};
  	// sensordata_insert_values2((unsigned char **) &rtc_buffer,timeref*SLEEPTIME,keys,values,3,&rtc_buffer_len);
	//} 
	
	} else {
		printf("init or setting error. code: %d", com_rslt);
	}

	xSemaphoreGive( xSemaphore );
       }
    
   }
	
   	//if(counter%TIMESLOT!=0){
	//	counter+=1;
      	//	timeref+=1;
	//	sleeppa(SLEEPTIME);
        //};
	vTaskDelete(NULL);
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
	    //deepsleep=1;
            break;
        }
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:{
	    //deepsleep=0;
            printf("Not a deep sleep reset\n");
	    cayenne_lpp_reset(&tlpp);
	    cayenne_lpp_reset(&hlpp);
	    cayenne_lpp_reset(&plpp);
            //cayenne_lpp_add_analog_input(&tlpp,0,SLEEP_INTERVAL);
            //cayenne_lpp_add_analog_input(&hlpp,0,SLEEP_INTERVAL);
            //cayenne_lpp_add_analog_input(&plpp,0,SLEEP_INTERVAL);
  	    //sensordata_init2((unsigned char **) &rtc_buffer, &rtc_buffer_len);
        }
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    const int wakeup_time_sec = sec;
    printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
    esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);

    //const int ext_wakeup_pin_1 = 25;
    //const uint64_t ext_wakeup_pin_1_mask = 1ULL << ext_wakeup_pin_1;
    //const int ext_wakeup_pin_2 = 26;
    //const uint64_t ext_wakeup_pin_2_mask = 1ULL << ext_wakeup_pin_2;

    //printf("Enabling EXT1 wakeup on pins GPIO%d, GPIO%d\n", ext_wakeup_pin_1, ext_wakeup_pin_2);
    //esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask | ext_wakeup_pin_2_mask, ESP_EXT1_WAKEUP_ANY_HIGH);

    // Isolate GPIO12 pin from external circuits. This is needed for modules
    // which have an external pull-up resistor on GPIO12 (such as ESP32-WROVER)
    // to minimize current consumption.
    //rtc_gpio_isolate(GPIO_NUM_12);

    printf("Entering deep sleep\n");
    gettimeofday(&sleep_enter_time, NULL);
    deepsleep=1;
    esp_deep_sleep_start();
}

void sendTMessage2()
{
	printf("Sending message...\n");

        cayenne_lpp_add_analog_input(&tlpp,0,SLEEP_INTERVAL);
	_print_buffer(&tlpp);
	TTNResponseCode res = ttn.transmitMessage((unsigned char*) tlpp.buffer, tlpp.cursor);
	if(res == kTTNSuccessfulTransmission){
		 printf("Message sent.\n");
		cayenne_lpp_reset(&tlpp);
	}else{
		printf("Transmission failed.\n");
	}
	RTCseqnoUp=LMIC.seqnoUp;	
	RTCseqnoDn=LMIC.seqnoDn;	
}
void sendHMessage2()
{
	printf("Sending message...\n");

        cayenne_lpp_add_analog_input(&hlpp,0,SLEEP_INTERVAL);
	_print_buffer(&hlpp);
	TTNResponseCode res = ttn.transmitMessage((unsigned char*) hlpp.buffer, hlpp.cursor);
	if(res == kTTNSuccessfulTransmission){
		 printf("Message sent.\n");
		cayenne_lpp_reset(&hlpp);
	}else{
		printf("Transmission failed.\n");
	}
	RTCseqnoUp=LMIC.seqnoUp;	
	RTCseqnoDn=LMIC.seqnoDn;	
}

void sendPMessage2()
{
	printf("Sending message...\n");

        cayenne_lpp_add_analog_input(&plpp,0,SLEEP_INTERVAL);
	_print_buffer(&plpp);
	TTNResponseCode res = ttn.transmitMessage((unsigned char*) plpp.buffer, plpp.cursor);
	if(res == kTTNSuccessfulTransmission){
		 printf("Message sent.\n");
		cayenne_lpp_reset(&plpp);
	}else{
		printf("Transmission failed.\n");
	}
	RTCseqnoUp=LMIC.seqnoUp;	
	RTCseqnoDn=LMIC.seqnoDn;	
}

void sendTMessage(void* pvParameter)
{
    while (1) {
  	if( txSemaphore != NULL )
   	{
    	    if( xSemaphoreTake( txSemaphore, ( TickType_t ) 10 ) == pdTRUE ) {
			xSemaphoreTake( tSemaphore, ( TickType_t ) 10 );
	        	printf("Sending message...\n");

               cayenne_lpp_add_analog_input(&tlpp,0,SLEEP_INTERVAL);
	      		_print_buffer(&tlpp);
	        	TTNResponseCode res = ttn.transmitMessage((unsigned char*) tlpp.buffer, tlpp.cursor);
	        	if(res == kTTNSuccessfulTransmission){
				 printf("Message sent.\n");
	    			cayenne_lpp_reset(&tlpp);
			}else{
				printf("Transmission failed.\n");
			}
			RTCseqnoUp=LMIC.seqnoUp;	
			RTCseqnoDn=LMIC.seqnoDn;	
			//counter=0;
			//sleeppa(SLEEP_INTERVAL);
	    		xSemaphoreGive( tSemaphore );

	   }
	}
    }
}
void sendHMessage(void* pvParameter)
{
    while (1) {
  	if( txSemaphore != NULL )
   	{
    	    if( xSemaphoreTake( txSemaphore, ( TickType_t ) 10 ) == pdTRUE ) {
			xSemaphoreTake( hSemaphore, ( TickType_t ) 10 );
	        	printf("Sending message...\n");
               cayenne_lpp_add_analog_input(&hlpp,0,SLEEP_INTERVAL);
	      		_print_buffer(&hlpp);
	        	TTNResponseCode res = ttn.transmitMessage((unsigned char*) hlpp.buffer, hlpp.cursor);
	        	if(res == kTTNSuccessfulTransmission){
				 printf("Message sent.\n");
	    			cayenne_lpp_reset(&hlpp);
			}else{
				printf("Transmission failed.\n");
			}
                	vTaskDelay(TX_INTERVAL * 1000 / portTICK_PERIOD_MS);
			RTCseqnoUp=LMIC.seqnoUp;	
			RTCseqnoDn=LMIC.seqnoDn;	
			//counter=0;
			//sleeppa(SLEEP_INTERVAL);
	    		xSemaphoreGive( hSemaphore );

	   }
	}
    }
}
void sendPMessage(void* pvParameter)
{
    while (1) {
  	if( txSemaphore != NULL )
   	{
    	    if( xSemaphoreTake( txSemaphore, ( TickType_t ) 10 ) == pdTRUE ) {
			xSemaphoreTake( pSemaphore, ( TickType_t ) 10 );
	        	printf("Sending message...\n");

               cayenne_lpp_add_analog_input(&plpp,0,SLEEP_INTERVAL);
	      		_print_buffer(&plpp);
	        	TTNResponseCode	res = ttn.transmitMessage((unsigned char*) plpp.buffer, plpp.cursor);
	        	if(res == kTTNSuccessfulTransmission){
				 printf("Message sent.\n");
	    			cayenne_lpp_reset(&plpp);
			}else{
				printf("Transmission failed.\n");
			}
			RTCseqnoUp=LMIC.seqnoUp;	
			RTCseqnoDn=LMIC.seqnoDn;	
			//counter=0;
			//sleeppa(SLEEP_INTERVAL);
	    		xSemaphoreGive( pSemaphore );

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
    vSemaphoreCreateBinary( txSemaphore );
    vSemaphoreCreateBinary( tSemaphore );
    vSemaphoreCreateBinary( hSemaphore );
    vSemaphoreCreateBinary( pSemaphore );

    err = nvs_flash_init();


    vTaskDelay( 1000 / portTICK_RATE_MS );


    ESP_ERROR_CHECK(err);


   if(counter==TIMESLOT or !deepsleep){
    	//xTaskCreate( &bmp280_status, "bmp280_status", 2048, NULL, 5, NULL );
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
    }
    // The below line can be commented after the first run as the data is saved in NVS
    if(!deepsleep){
    	ttn.provision(devEui, appEui, appKey);

    	ttn.onMessage(messageReceived);
    	printf("Joining...\n");
    	//LMIC.adrTxPow = 14;
    	//LMIC_setDrTxpow(DR_SF10, 14);
    	//LMIC.datarate = DR_SF10;
    	if (ttn.join())
    	{
    	    printf("Joined.\n");
	    LMIC_getSessionKeys (&RTCnetid, &RTCdevaddr,RTCnwkKey,RTCartKey);	    
            printf("netid:%x\n",RTCnetid);
            printf("devaddr:%x\n",RTCdevaddr);
            int i=0;
            int k=0;
            for(i=0;i<16;i++){
            	printf("%.2x",RTCnwkKey[i]);
            }
            printf("\n");
            for(k=0;k<16;k++){
            	printf("%.2x",RTCartKey[k]);
            }
            printf("\n");


	    sleeppa(SLEEP_INTERVAL);
    	    //xTaskCreate(sendMessages, "send_messages", 1024 * 4, (void* )0, 3, NULL);
    	}
    	else
    	{
    	    printf("Join failed. Goodbye\n");
    	        sleeppa(600);
    	}
    }else{
       if (counter==TIMESLOT){
               printf("counter=%d\n",counter);
    	       ttn_hal.initCriticalSection();
               LMIC_reset();
               //hal_enterCriticalSection();
               LMIC_setSession (RTCnetid, RTCdevaddr, RTCnwkKey, RTCartKey);
               //hal_leaveCriticalSection();
               LMIC.seqnoUp=RTCseqnoUp;
               LMIC.seqnoDn=RTCseqnoDn;
               ttn_hal.leaveCriticalSection();


               printf("mando il messaggio in ABP con numeri di sequenza Up:%d Dn:%d\n",LMIC.seqnoUp,LMIC.seqnoDn);
	       LMIC.bands[BAND_MILLI].avail = os_getTime();
	       LMIC.bands[BAND_CENTI].avail = os_getTime();
	       LMIC.bands[BAND_DECI].avail = os_getTime();
	       sendTMessage2();
	       LMIC.bands[BAND_MILLI].avail = os_getTime();
	       LMIC.bands[BAND_CENTI].avail = os_getTime();
	       LMIC.bands[BAND_DECI].avail = os_getTime();
	       sendHMessage2();
	       LMIC.bands[BAND_MILLI].avail = os_getTime();
	       LMIC.bands[BAND_CENTI].avail = os_getTime();
	       LMIC.bands[BAND_DECI].avail = os_getTime();
	       sendPMessage2();
	       counter=0;
               sleeppa(SLEEP_INTERVAL);
               counter=0;
       }
	else{
	       i2c_master_init();
    	       xTaskCreate(&task_bme280_normal_mode, "bme280_normal_mode",  2048, NULL, 6, NULL);
               counter+=1;
               while (1) {

                  vTaskDelay( 1000 / portTICK_RATE_MS );
                   printf("aspetto che legga il sensore...\n");
                  if( xSemaphore != NULL )
                    {
                     if( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE ) {
                       printf("scrivo nelle strutture...\n");
	       	       printf("p:%f h:%f t:%f\n",p,h,t);
               	       cayenne_lpp_add_temperature(&tlpp, counter-1, t);
               	       cayenne_lpp_add_relative_humidity(&hlpp, counter-1, h);
               	       cayenne_lpp_add_barometric_pressure(&plpp, counter-1, p);
              	       _print_buffer(&tlpp);
              	       _print_buffer(&hlpp);
              	       _print_buffer(&plpp);
		     	 
                      sleeppa(SLEEP_INTERVAL);
		     
                     }
                    }
               }
       }
    }
}
