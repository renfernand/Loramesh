#include "devconfig.h"
#include "heltec.h"
#include <Wire.h>
#include <RadioLib.h>
#if DISPLAY_ENABLE
#include "OLED\SSD1306.h"
#include "display.h"
#endif
#include "esp_log.h"
 #include "driver/board.h"

#define LOG_LOCAL_LEVEL CONFIG_LOG_MAXIMUM_LEVEL
 
#define PABOOST 1
#define RX_TIMEOUT_VALUE                            1000

#define TEST_FRAME_10
#define MINIMUM_DELAY 900 

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

#define timewindow 20000
static TimerEvent_t twin;

int count1=0;
  
//attachInterrupt(GPIO_Pin, zcisr, RISING);
uint32_t interval;
uint16_t count = 0;
int sensorValue;
uint32_t delayMs;


void OnWindow()
{
#if ROUTER
  sprintf(txpacket,"%s%d",frame1,count1);
  count1++;
  
  LoRa.SendFrame(txpacket,1);
  log_i("%s",txpacket);
#endif

  TimerSetValue( &twin, timewindow );
  TimerStart( &twin );
}

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

  // And then pick it or our MINIMUM_DELAY, whichever is greater
  interval = 0;
  delayMs = max(interval, (uint32_t)MINIMUM_DELAY * 1000);

  // and off to bed we go
  //heltec_deep_sleep(delayMs/1000);
#if ROUTER
  TimerInit( &twin, OnWindow );
  OnWindow();
#endif


}

void loop() {
  char buf[BUFFER_SIZE];
  uint8_t ret=0;   

#if ROUTER == 0
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

