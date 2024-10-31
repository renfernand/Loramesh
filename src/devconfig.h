#ifndef _DEVCONFIG_H_
#define _DEVCONFIG_H_
//Suport Heltec V2 and V3 
// heltec_wifi_lora_32_V2 -  radio chipset SX1276 OLED chip SSD1306)
// heltec_wifi_lora_32_V3 -  radio chipset SX1262 OLED chip SSD1306)

//define se o nó sensor será um ED (ROUTER=0) ou RT (ROUTER=1) 
#define ROUTER 0

//define se vai usar ou nao o Display Oled da placa 
#define DISPLAY_ENABLE 0


#define ENABLE_RX_INTERRUPT 0

#ifndef LED_BUILTIN
//#define LED_BUILTIN 13
#endif

/*  LoRa spreading factor. Defaults to 9.
   goes from 7 to 12 where SF7 is the shortest and SF12 the longest */
   // Number from 5 to 12. Higher means slower but higher "processor gain",
// meaning (in nutshell) longer range and more robust against interference. 
#define LORA_SF 7

/*Frequency for lora
   434.0 MHz (default) 
   470E6
   868E6 
   915E6 */
#define LORA_FREQUENCY_V3 915.0 
#define LORA_FREQUENCY_V2 915E6 

/* LoRa bandwidth in kHz.  Defaults to 125.0 kHz.
125.0
250.0 
500.0
1625.0  (para freq 2.4Ghz)
*/
// LoRa bandwidth. Keep the decimal point to designate float.
// Allowed values are 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0 and 500.0 kHz.
#define LORA_BW_V3 125.0
#define LORA_BW_V2 125E3

/*Output power in dBm. Defaults to 10 dBm.
  range from 10 to 20 dBm*/
#define LORA_POWER 10


// Transmit power in dBm. 0 dBm = 1 mW, enough for tabletop-testing. This value can be
// set anywhere between -9 dBm (0.125 mW) to 22 dBm (158 mW). Note that the maximum ERP
// (which is what your antenna maximally radiates) on the EU ISM band is 25 mW, and that
// transmissting without an antenna can damage your hardware.
#define TRANSMIT_POWER      0

/* cr */
#define LORA_CR 5



#endif
