#ifndef _DEVCONFIG_H_
#define _DEVCONFIG_H_
//Suport Heltec V2 and V3 
// heltec_wifi_lora_32_V2 -  radio chipset SX1276 OLED chip SSD1306)
// heltec_wifi_lora_32_V3 -  radio chipset SX1262 OLED chip SSD1306)

//define se o nó sensor será um ED (ROUTER=0) ou RT (ROUTER=1) 
#define ROUTER 1


//define se vai usar ou nao o Display Oled da placa 
#define DISPLAY_ENABLE 1

#define ENABLE_RX_INTERRUPT 0
// Packet types
#define NEED_ACK_P 0b00000011
#define DATA_P     0b00000010
#define HELLO_P    0b00000100
#define ACK_P      0b00001010
#define XL_DATA_P  0b00010010
#define LOST_P     0b00100010
#define SYNC_P     0b01000010
#define LOG_LOCAL_LEVEL CONFIG_LOG_MAXIMUM_LEVEL
 
#define PABOOST 1
#define RX_TIMEOUT_VALUE   1000

#define MINIMUM_DELAY 900 

#define MAX_ADDR 5
#define BROADCAST_ADDR 0
#define BYTE_CRC 0x66
//#define SOURCE_ADDRESS 1

#define MAX_PACKET_SIZE  30
#define MAX_SLOTS 2
#define BEACON_SLOT 0 
#define DATA_SLOT   2 

//Intervalo entre os envios
#define SLOT_INTERVAL 1000

#define BUFFER_SIZE           50 // Define the payload size here

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
#define LORA_FREQUENCY_V3 868.0 
#define LORA_FREQUENCY_V2 868E0 
#define LORA_FREQUENCY 868.0 

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
#define LORA_BW 125.0

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

typedef enum {
   DEV_TYPE_ROUTER=1,
   DEV_TYPE_ENDDEV
} devicetype;
typedef enum {
   FCT_BEACON=1,
   FCT_JOIN,
   FCT_DATA
} functioncode;

typedef enum  {
    ST_TXBEACON,
    ST_RXWAIT,
    ST_RXDONE,
    ST_TXDATA,
    ST_STANDBY
}statemac;


#endif
