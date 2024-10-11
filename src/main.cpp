#include "devconfig.h"
#include "heltec.h"
#include "Lora\loracom.h"
#include <Wire.h>
#include "OLED\SSD1306.h"
#include <RadioLib.h>
#if DISPLAY_ENABLE
#include "display.h"
#endif
#include "esp_log.h"
 
#define LOG_LOCAL_LEVEL CONFIG_LOG_MAXIMUM_LEVEL
 
#define PABOOST 1
#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 30 // Define the payload size here

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
char frame1[21]= "12345678901234567890";
int count1=0;

uint16_t count = 0;
int sensorValue;

void setup()
{
   bool displayEnable;

#if DISPLAY_ENABLE 
   displayEnable = true;
#else
   displayEnable = false;
#endif

//Heltec.begin(DisplayEnable,LoRaEnable, SerialEnable,PABOOST=true, long BAND=470E6
#if defined( WIFI_LoRa_32_V3 ) 
Heltec.begin(displayEnable, true, true,PABOOST, LORA_FREQUENCY_V3);
#else //WIFI_LoRa_32_V2
Heltec.begin(displayEnable, true, true,PABOOST, LORA_FREQUENCY_V2);
#endif
}

void loop() {
  char buf[30];
  uint8_t ret=0;   

#if ROUTER
    sprintf(txpacket,"%s %d",frame1,count1);
    count1++;
  
    LoRa.SendFrame(txpacket,1);
    log_i("%s",txpacket);
#else
   ret = LoRa.ReceiveFrame(rxpacket);
    if (ret>0){
      sprintf(buf,"Rx=%s",rxpacket);
      log_i("ED buf=%s",buf);
    }
#endif
  
#if DISPLAY_ENABLE 
  Heltec.DisplayShow(buf);
#endif
  delay(500);
}

