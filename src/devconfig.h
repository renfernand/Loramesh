#ifndef _DEVCONFIG_H_
#define _DEVCONFIG_H_
//Suport Heltec V2 and V3 
// heltec_wifi_lora_32_V2 -  radio chipset SX1276 OLED chip SSD1306)
// heltec_wifi_lora_32_V3 -  radio chipset SX1262 OLED chip SSD1306)

//define se o nó sensor será um ED (ROUTER=0) ou RT (ROUTER=1) 
#define ROUTER 0

//define se vai usar ou nao o Display Oled da placa 
#define DISPLAY_ENABLE 0

#ifndef LED_BUILTIN
//#define LED_BUILTIN 13
#endif

#define MY_ADDRESS 0
#define BROADCAST 255

/*  LoRa spreading factor. Defaults to 9.
   goes from 7 to 12 where SF7 is the shortest and SF12 the longest */
#define LORA_SF 9

/*Frequency for lora
   434.0 MHz (default) 
   470E6
   868E6 
   915E6 */
#define LORA_FREQUENCY_V3 868.0 
#define LORA_FREQUENCY_V2 868E6 

/* LoRa bandwidth in kHz.  Defaults to 125.0 kHz.
125.0
250.0 
500.0
1625.0  (para freq 2.4Ghz)
*/
#define LORA_BW_V3 125.0
#define LORA_BW_V2 125E3

/*Output power in dBm. Defaults to 10 dBm.
  range from 10 to 20 dBm*/
#define LORA_POWER 10

/* coding rate */
#define LORA_CR 7



#endif
