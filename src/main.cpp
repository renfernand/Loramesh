#include "devconfig.h"
#include "heltec.h"
#include <Wire.h>
#include <RadioLib.h>
#if DISPLAY_ENABLE
#include "OLED\SSD1306.h"
#include "display.h"
#endif
#include "esp_log.h"
 
#define LOG_LOCAL_LEVEL CONFIG_LOG_MAXIMUM_LEVEL
 
#define PABOOST 1
#define RX_TIMEOUT_VALUE                            1000

#define TEST_FRAME_40

#define BUFFER_SIZE                                 50 // Define the payload size here
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

#if defined TEST_FRAME_10
char frame1[11]= "123456789";
#elif defined TEST_FRAME_20
char frame1[21]= "12345678901234567890";
#elif defined TEST_FRAME_30
char frame1[31]= "123456789012345678901234567890";
#elif defined TEST_FRAME_40
char frame1[41]= "1234567890123456789012345678901234567890";
#endif

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
  char buf[BUFFER_SIZE];
  uint8_t ret=0;   

#if ROUTER
    sprintf(txpacket,"%s%d",frame1,count1);
    count1++;
  
    LoRa.SendFrame(txpacket,1);
    log_i("%s",txpacket);
    delay(2000);
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
}

