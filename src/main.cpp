#include "devconfig.h"
#include "heltec.h"
#include <Wire.h>
#include <RadioLib.h>
#if DISPLAY_ENABLE
#include "OLED\SSD1306.h"
#endif
#include "esp_log.h"
 
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

typedef enum  {
    ST_RXWAIT,
    ST_RXBEFORE,
    ST_RXAFTER,
    ST_TXACK
}statemac;

uint8_t state;
uint8_t countRxAfter=0;

int count1=0;
char frameAck[11]= "123456789";

//attachInterrupt(GPIO_Pin, zcisr, RISING);
uint32_t interval;
uint16_t count = 0;

int sensorValue;

TaskHandle_t Rx1Handle = NULL;

//#define ARDUINO_RUNNING_CORE 1

// Função para limpar o buffer
void clearBuffer(char *buffer, int size)
{
    for (int i = 0; i < size; i++)
    {
        buffer[i] = '\0';
    }
}

bool receivePacket()
{
    int packetSize = 0;
    bool ret=0;
    
    packetSize = loramesh.parsePacket(0);
    if (packetSize)
    {
        int len = 0;
        clearBuffer(rxpacket, BUFFER_SIZE);

        while (loramesh.available() && len < BUFFER_SIZE - 1)
        {
            rxpacket[len++] = (char)loramesh.read(); // Lê o pacote byte a byte
        }

        rxpacket[len] = '\0'; // Termina string

        // Confirmação de recepção
        Serial.println(rxpacket);
        ret = 1;
    }
    
    return ret;
}


void sendAck()
{
    clearBuffer(txpacket, BUFFER_SIZE);

    sprintf(txpacket, "%s", frameAck);

    loramesh.SendFrame(txpacket, 1);

    //log_i("Tx Ack=%s", txpacket);
}


void receivetask(void * parameter){

    for( ;; )
    {
#if 0        
        switch (state) {
            case ST_RXWAIT:
                log_i("state =%d",state);
                LoRa.EnableReception();
                state = ST_RXAFTER;
              break;
            case ST_RXAFTER:
                log_i("state =%d",state);
                if  (receivePacket()) {
                    log_i("ST_RXWAIT...recebeu!!!");
                    state = ST_RXWAIT;
                }        
                break;
            default:
               break;    
        }
#else
                if  (receivePacket()) {
                    log_i("ST_RXWAIT...recebeu!!!");
                    //sendAck();
                }
                //else {
                //    loramesh.EnableReception();
                //}
#endif
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

}

void sendPacket()
{
    sprintf(txpacket, "%s %d", frame1, count1);
    count1++;

    loramesh.SendFrame(txpacket, 1);
    log_i("%s", txpacket);
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

 // xTaskCreatePinnedToCore( receivetask,"Task1",1024, NULL, 2, NULL, ARDUINO_RUNNING_CORE);

  state = ST_RXWAIT;
}

void loop() {
  char buf[BUFFER_SIZE];
  uint8_t ret=0;   

#if ROUTER
    sendPacket();
    delay(2000);
#else
    //ret = LoRa.ReceiveFrame(rxpacket);

    if (receivePacket()){
        log_i("Pacote recebido: ");
        //Heltec.display->clear();
        // Heltec.display->drawString(0, 0, "Aguardando pacote...");
        // Heltec.display->display();
    }
    delay(10);

#endif
    //if (Rx1Handle != NULL) {
    //   vTaskDelete(Rx1Handle);
   // }
#if DISPLAY_ENABLE 
  Heltec.DisplayShow(buf);
#endif
}

