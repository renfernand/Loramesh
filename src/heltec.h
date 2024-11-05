

#ifndef _HELTEC_H_
#define _HELTEC_H_

#if defined(ESP32)

#include <Arduino.h>
#if defined( WIFI_Kit_32 ) || defined( WIFI_LoRa_32 ) || defined( WIFI_LoRa_32_V2 ) || defined( WIFI_LoRa_32_V3 ) || defined( Wireless_Stick ) 
#include <Wire.h>
#include "oled/SSD1306Wire.h"
#endif

#if defined( WIFI_LoRa_32 ) || defined( WIFI_LoRa_32_V2 ) || defined( WIFI_LoRa_32_V3 ) || defined( Wireless_Stick ) || defined( Wireless_Stick_Lite ) || defined( Wireless_Bridge )
#include <SPI.h>
#include "lora\LoRa.h"
#endif


class Heltec_ESP32 {

 public:
    Heltec_ESP32();
	~Heltec_ESP32();

//  void begin(bool DisplayEnable=true, bool LoRaEnable=true, bool SerialEnable=true, bool PABOOST=true, long BAND=470E6);
    void begin(bool DisplayEnable, bool LoRaEnable, bool SerialEnable, bool PABOOST, long BAND);

    void DisplayShow(char *pframe);

#if defined( WIFI_LoRa_32 ) || defined( WIFI_LoRa_32_V2 ) || defined( Wireless_Stick ) || defined( Wireless_Stick_Lite ) || defined( Wireless_Bridge ) 
    LoRaClass LoRa;
#endif

#if defined( WIFI_Kit_32 ) || defined( WIFI_LoRa_32 ) || defined( WIFI_LoRa_32_V2 ) || defined( Wireless_Stick ) || defined( WIFI_LoRa_32_V3 )
    SSD1306Wire *display;
#endif

/*wifi kit 32 and WiFi LoRa 32(V1) do not have vext*/
    void VextON(void);
    void VextOFF(void);
    
    uint32_t freq;
    uint32_t bw;
    uint8_t  sf;
};

extern Heltec_ESP32 Heltec;

#else
#error "This library only supports boards with ESP32 processor."
#endif


#endif
