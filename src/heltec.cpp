// Copyright (c) Heltec Automation. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.
#include "devconfig.h"
#include "heltec.h"

#define LOG_LOCAL_LEVEL CONFIG_LOG_MAXIMUM_LEVEL

Heltec_ESP32::Heltec_ESP32()
{

#if defined( WIFI_Kit_32 ) || defined( WIFI_LoRa_32 ) || defined( WIFI_LoRa_32_V2 ) || defined ( WIFI_LoRa_32_V3 )
      display = new SSD1306Wire(0x3c, SDA_OLED, SCL_OLED, RST_OLED, GEOMETRY_128_64);
#elif defined( Wireless_Stick )
	  display = new SSD1306Wire(0x3c, SDA_OLED, SCL_OLED, RST_OLED, GEOMETRY_64_32);
#endif
 
}

Heltec_ESP32::~Heltec_ESP32()
{
#if defined( WIFI_Kit_32 ) || defined( WIFI_LoRa_32 ) || defined( WIFI_LoRa_32_V2 ) || defined( Wireless_Stick ) || defined( WIFI_LoRa_32_V3 )
	delete display;
#endif
}


void Heltec_ESP32::begin(bool DisplayEnable, bool LoRaEnable, bool SerialEnable, bool PABOOST, long BAND) 
{

#if defined( WIFI_LoRa_32_V2 ) || defined( Wireless_Stick ) || defined( Wireless_Stick_Lite ) || defined(WIFI_Kit_32) || defined( Wireless_Bridge ) || defined( WIFI_LoRa_32_V3 ) 
	VextON();
#endif

	// UART
	if (SerialEnable) {
		Serial.begin(9600);
		Serial.flush();
		delay(50);
		Serial.print("Serial initial done\r\n");
	}

    #if ROUTER
	log_i("DEVICE IS A ROUTER!!!! \r\n");
    #else
	log_i("DEVICE IS A END DEVICE!!!! \r\n");
    #endif

	// OLED
	if (DisplayEnable)
	{
#if defined( Wireless_Stick_Lite ) || defined( Wireless_Bridge )
		if(SerialEnable)		{
			Serial.print("Wireless Stick Lite and Wireless Bridge don't have an on board display, Display option must be FALSE!!!\r\n");
		}
#endif

#if defined( WIFI_LoRa_32_V3 ) 
/* O estado do GPIO36 é utilizado para controlar o display OLED. 
   Para o OLED permanecer ligado, o GPIO36 deve permanecer HIGH e deve estar neste estado quando chamar display Init.
   NAO INVERTER esta ORDEM.
*/
      pinMode(36,OUTPUT);
	  digitalWrite(36,LOW);
	  delay(50);
	  digitalWrite(36,HIGH);
#endif

#if defined( WIFI_Kit_32 ) || defined( WIFI_LoRa_32 ) || defined( WIFI_LoRa_32_V2 ) || defined( Wireless_Stick ) || defined( WIFI_LoRa_32_V3 ) 
		display->init();
		display->flipScreenVertically();
		display->setFont(ArialMT_Plain_10);
		display->drawString(0, 0, "OLED initial done!");
		display->display();

		if (SerialEnable){
			Serial.print("you can see OLED printed OLED initial done!\r\n");
		}
#endif
	}

	// LoRa INIT
	if (LoRaEnable)
	{
#if defined(WIFI_Kit_32)
		if(SerialEnable && WIFI_Kit_32){
			Serial.print("The WiFi Kit 32 not have LoRa function, LoRa option must be FALSE!!!\r\n");
		}
#endif

#if defined( WIFI_LoRa_32 ) || defined( WIFI_LoRa_32_V2 ) || defined( Wireless_Stick ) || defined( Wireless_Stick_Lite ) || defined( Wireless_Bridge )  ||  defined( WIFI_LoRa_32_V3 )

		LoRa.setPins(SS,RST_LoRa,DIO0);
		if (!LoRa.begin(BAND,PABOOST))
		{
			if (SerialEnable){
				log_e("Starting LoRa failed!\r\n");
			}
#if defined( WIFI_Kit_32 ) || defined( WIFI_LoRa_32 ) || defined( WIFI_LoRa_32_V2 ) || defined( Wireless_Stick ) 
			if(DisplayEnable){
				display->clear();
				display->drawString(0, 0, "Starting LoRa failed!");
				display->display();
				delay(300);
			}
#endif
			while (1);
		}
		if (SerialEnable){
			//Serial.print("LoRa Initial success!\r\n");
			freq = LORA_FREQUENCY_V2/1E6;
			bw = (uint32_t) LORA_BW_V3;
			sf =  LORA_SF;
			log_i("Radio Lora Freq=%d Bw=%d Sf=%d",freq,bw,sf);
		}
#if defined( WIFI_Kit_32 ) || defined( WIFI_LoRa_32 ) || defined( WIFI_LoRa_32_V2 ) || defined( Wireless_Stick ) 
		if(DisplayEnable){
			display->clear();
			display->drawString(0, 0, "LoRa Initial success!");
			display->display();
			delay(300);
		}
#endif

#endif
	}
#if defined( WIFI_Kit_32 ) || defined( WIFI_LoRa_32 ) || defined( WIFI_LoRa_32_V2 ) || defined( Wireless_Stick ) 	
	pinMode(LED,OUTPUT);
#endif

#if DISPLAY_ENABLE
		//display->clear();
		display->drawString(0, 10, "Radio Lora Ok!!!!");
		log_i("Radio Lora Ok!!!!");
#if ROUTER		
		display->drawString(0, 20, "Device is a router!!!!");
		log_i("Device is a router!!!!");
#else		
		display->drawString(0, 20, "Device is a end device!!!!");
		log_i("Device is a end device!!!!");
#endif
		display->display();
		delay(300);
#endif


}

void Heltec_ESP32::VextON(void)
{
	pinMode(Vext,OUTPUT);
	digitalWrite(Vext, LOW);
}

void Heltec_ESP32::VextOFF(void) //Vext default OFF
{
	pinMode(Vext,OUTPUT);
	digitalWrite(Vext, HIGH);
}

void Heltec_ESP32::DisplayShow(char *pframe) 
{
	char buf[20];
	uint32_t rssi=0;

#if DISPLAY_ENABLE

	Heltec.display->clear();
	Heltec.display->setFont(ArialMT_Plain_10);

	#if ROUTER == 1
		sprintf(buf,"RT - FREQ=%3d SF=%d ",freq,sf);
		//log_i("buf=%s",buf);
		Heltec.display->drawString(0, 0, buf);
		memset(buf,0x20,sizeof(buf));
		sprintf(buf, "%s", pframe);  
		Heltec.display->drawString(0, 20, buf);
		memset(buf,0x20,sizeof(buf));
		//Heltec.display->drawString(0, 40, buf);
		Heltec.display->display();
	#else

		sprintf(buf,"ED - FREQ=%3s SF=%1s ",String(freq),String(sf));
		Heltec.display->drawString(0, 0, buf);
		memset(buf,0x20,sizeof(buf));
		//sprintf(buf, "RX [%d] = %s", value,pframe);  
		sprintf(buf, "%s", pframe);  
		Heltec.display->drawString(0, 20, buf);
		memset(buf,0x20,sizeof(buf));

		rssi = getRssi();
		sprintf(buf, "RSSI = %d", rssi);  
		Heltec.display->drawString(0, 40, buf);
		Heltec.display->display();
		memset(buf,0x20,sizeof(buf));

	#endif


#endif
}


Heltec_ESP32 Heltec;
