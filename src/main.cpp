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
 
#define PABOOST 0


//Alterei alguma coisa no codigo 
//Alterei de novo



int count = 0;
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

char txframe[10];
size_t Buildframe(char *txframe) {
   
   size_t pos=0;
   char *pucAux=(char *) &count; 

   //txframe[pos++] = MY_ADDRESS;
   //txframe[pos++] = BROADCAST;
   //txframe[pos++] = CMD_DATA;
   txframe[pos++] = *pucAux++;
   txframe[pos++] = *pucAux++;
   return pos;
}

void loop(){
  char buf[30];
  size_t len=0;

  #if ROUTER
    // Send the message
    len = Buildframe(txframe);

    sprintf(buf,"Tx= %d",count);
    log_i("%s",buf);
    LoRa.SendFrame(txframe,len);
    //LoRa.SendFrame(String(count),1);
    count++;
  #else
    uint8_t ret=0;
    char  newframe[50];

    ret = LoRa.ReceiveFrame(newframe);
    if (ret>0){
      sprintf(buf,"Rx=%s",newframe);
      log_i("ED buf=%s",buf);
    }
  #endif 

  #if DISPLAY_ENABLE 
    Heltec.DisplayShow(buf);
  #endif
  delay(500); 
}



