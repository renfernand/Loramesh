#include "devconfig.h"
#include "heltec.h"
#include "Lora\loramesh.h"
#include <Wire.h>
#include <RadioLib.h>
#include "esp_log.h"

#if DISPLAY_ENABLE
#include "OLED\SSD1306.h"
#endif

TaskHandle_t Router_TaskHandle = nullptr;
TaskHandle_t EndDev_TaskHandle = nullptr;

LoRaClass &LoRa1 = LoRaClass::getInstance();
extern LoRaClass loramesh;
#define LOG_LOCAL_LEVEL CONFIG_LOG_MAXIMUM_LEVEL
 
#define PABOOST 1
#define RX_TIMEOUT_VALUE                            1000

#define MINIMUM_DELAY 900 

#define BUFFER_SIZE                                 50 // Define the payload size here
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
uint8_t srcaddress;
uint8_t dstaddress;
uint16_t invokeid=0;
uint16_t resInvokeid=0;
uint8_t fct;
uint32_t timestamp;
uint16_t data1 = 0x3344;

//Tempo do último envio
long lastSendTime = 0;

uint8_t wait_res = 0;
uint8_t send_req = 0;

//Intervalo entre os envios
#define INTERVAL 2000

uint8_t state;
uint8_t countRxAfter=0;

int count1=0;

//attachInterrupt(GPIO_Pin, zcisr, RISING);
uint32_t interval;
uint16_t count = 0;

int sensorValue;

TaskHandle_t Rx1Handle = NULL;

//#define ARDUINO_RUNNING_CORE 1

void standby();
void start();

// Função para limpar o buffer
void clearBuffer(char *buffer, int size)
{
    for (int i = 0; i < size; i++)
    {
        buffer[i] = '\0';
    }
}

uint16_t getinvid(uint8_t *packet,uint8_t len){
    uint16_t aux;
    uint8_t *pucaux = (uint8_t *) &aux;
    if (len > 4){
        *pucaux++ = packet[4];
        *pucaux = packet[3];
        //log_i ("Invokeid=%d",invokeid);
        return aux;
    }
    else
        return 0;

}

uint8_t getfunction(uint8_t *packet,uint8_t len){
    uint8_t function;
    if (len > 3){
        function = packet[2];
        //log_i ("function=%d",function);
        return function;
    }
    else
        return 0;

}

uint16_t gettimestamp(uint8_t *packet,uint8_t len){
    uint8_t *pucaux = (uint8_t *) &timestamp;

    if (len > 5){
        *pucaux++ = packet[8];
        *pucaux++ = packet[7];
        *pucaux++ = packet[6];
        *pucaux = packet[5];
        //log_i ("timestamp=%4x",timestamp);
        return timestamp;
    }
    else
        return 0;

}
uint8_t getaddress(uint8_t *packet,uint8_t len){
  
  srcaddress = packet[0];
  dstaddress = packet[1];

  if ((srcaddress < MAX_ADDR) && (dstaddress < MAX_ADDR)) 
    return dstaddress;
  else
    return 0;

}

//Todo!!! implementar um CRC
//checa somente o ultimo byte do frame é igual ao definido
uint8_t checkcrc (uint8_t *packet, uint8_t len){
   
   if (packet[len-1] == BYTE_CRC)
    return 1;
  else
    return 0;
}

bool receivePacket()
{
    bool retcrc=0;
    int packetSize = 0;

    packetSize = loramesh.parsePacket(0);
    if (packetSize)
    {
        uint8_t address;
        int len = 0;
        clearBuffer(rxpacket, BUFFER_SIZE);

        while (loramesh.available() && len < BUFFER_SIZE - 1) {
            rxpacket[len++] = (char)loramesh.read(); // Lê o pacote byte a byte
        }

        rxpacket[len] = '\0'; // Termina string

        // Confirmação de recepção
        address = getaddress((uint8_t *)rxpacket,packetSize);
        fct     = getfunction((uint8_t *)rxpacket,packetSize);
        resInvokeid = getinvid((uint8_t *)rxpacket,packetSize);
        timestamp = gettimestamp((uint8_t *)rxpacket,packetSize);
        retcrc = checkcrc((uint8_t *)rxpacket,packetSize);

        log_i("Rx len=%d src=%d dest=%d fct=%d invid=%2x crc=%d ",packetSize, srcaddress,dstaddress,fct, invokeid, retcrc);

        if ((address == MY_ADDR) && (retcrc == 1)) {
           //log_i("Receive len[%d]=%2x invokeid=%d",packetSize, invokeid);
            return 1;
        }
        else
            return 0;
    }
    else
        return 0;

}

uint8_t sendPacketRes(uint8_t dstaddr, uint16_t dtvalue)
{
    uint8_t ret=0;
    uint8_t pos=0;
    uint8_t buf[BUFFER_SIZE];
    uint8_t *pucaux = (uint8_t *) &resInvokeid; 

    srcaddress = MY_ADDR;
    dstaddress = dstaddr;

    buf[pos++] =  srcaddress;
    buf[pos++] =  dstaddress;
    buf[pos++] =  FCT_DATA;
    buf[pos++] =  *(pucaux+1);
    buf[pos++] =  *(pucaux+0);
    pucaux = (uint8_t *) &dtvalue; 
    buf[pos++] =  *(pucaux+1);
    buf[pos++] =  *(pucaux+0);
    buf[pos++] =  BYTE_CRC;

#if 1
    ret = loramesh.sendPacket(buf,pos);
    if (ret){
        log_i("RES[%d]=%2x %2x %2x %2x %2x %2x %2x %2x", pos, buf[0], buf[1],buf[2],buf[3],
        buf[4], buf[5],buf[6],buf[7]);
       return pos;
    }
    else
       return 0;   
#else
  loramesh.beginPacket();
  //print: adiciona os dados no pacote
  for (int i = 0; i < sizeof(frame1); i++) {
      loramesh.write((uint8_t)txpacket[i]);
  }
  loramesh.endPacket(); //retorno= 1:sucesso | 0: falha

#endif    
}

uint8_t sendPacketReq(long timestamp)
{
    uint8_t ret=0;
    uint8_t pos=0;
    uint8_t buf[BUFFER_SIZE];
    uint8_t *pucaux = (uint8_t *) &invokeid;

    srcaddress = MY_ADDR;
    dstaddress = ED1_ADDR;

    buf[pos++] =  srcaddress;
    buf[pos++] =  dstaddress;
    buf[pos++] =  FCT_BEACON;
    buf[pos++] =  *(pucaux+1);
    buf[pos++] =  *(pucaux+0);
    pucaux = (uint8_t *) &timestamp;
    buf[pos++] =  *(pucaux+3);
    buf[pos++] =  *(pucaux+2);
    buf[pos++] =  *(pucaux+1);
    buf[pos++] =  *(pucaux+0);
    buf[pos++] =  BYTE_CRC;

    invokeid++;

#if 1
    ret = loramesh.sendPacket(buf,pos);
    if (ret)
        //log_i("%2x %2x %2x %2x %2x %2x %2x %2x", buf[0], buf[1],buf[2],buf[3],
        //buf[4], buf[5],buf[6],buf[7]);
       return pos;
    else
       return 0;   
#else
  loramesh.beginPacket();
  //print: adiciona os dados no pacote
  for (int i = 0; i < sizeof(frame1); i++) {
      loramesh.write((uint8_t)txpacket[i]);
  }
  loramesh.endPacket(); //retorno= 1:sucesso | 0: falha

#endif   

}

#if 0
void RouterTask(void * parameter) {

    bool ret=0;
    
    for(;;){
        log_i("RouterTask");
        
        //Se passou o tempo definido em INTERVAL desde o último envio
        if (millis() - lastSendTime > INTERVAL){

            //Marcamos o tempo que ocorreu o último envio
            lastSendTime = millis();
            log_i("SendPacket %d",lastSendTime);
            //Envia o pacote para informar ao Slave que queremos receber os dados
            //send();
            //sendPacket();
            //loramesh.SendFrame(frame1,1);
            loramesh.createPacketAndSend(BROADCAST_ADDR, helloPacket, 1);          
        }

        //if  (receivePacket()) {
        //    log_i("Receive=%2x %2x %2x",rxpacket[0],rxpacket[1],rxpacket[2]);
        //}

        vTaskDelay(10 / portTICK_PERIOD_MS);


    } 
}

void EndDevTask(void * parameter) {

    for( ;; )
    {

        if  (receivePacket()) {
            
            vTaskDelay(10 / portTICK_PERIOD_MS);

            if (0) {  
            //if(memcmp(received, FrameMaster, sizeof(FrameMaster)) == 0){
                //Cria o pacote para envio
                loramesh.beginPacket();

                for (int i = 0; i < sizeof(FrameSlave); i++) {
                loramesh.write((uint8_t)FrameSlave[i]);
                }
                //Finaliza e envia o pacote
                loramesh.endPacket();
            
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

}
#endif

void standby() {
    //Get actual priority
    UBaseType_t prevPriority = uxTaskPriorityGet(NULL);

    //Set max priority
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

    //int16_t res = loramesh.standby();
    //if (res != 0)
    //    log_e("Standby gave error: %d", res);

    //Clear Dio Actions
    //loramesh.clearDioActions();

    //Suspend all tasks
    vTaskSuspend(Router_TaskHandle);
    //vTaskSuspend(EndDev_TaskHandle);

    //Set previous priority
    vTaskPrioritySet(NULL, prevPriority);

}

void start() {
    // Get actual priority
    UBaseType_t prevPriority = uxTaskPriorityGet(NULL);

    // Set max priority
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

    // Resume all tasks
    vTaskResume(Router_TaskHandle);
    //vTaskResume(EndDev_TaskHandle);

    // Start Receiving
#if ROUTER == 0    
    loramesh.startReceiving();
#endif
    // Set previous priority
    vTaskPrioritySet(NULL, prevPriority);
}

void setup()
{
    Serial.begin(9600);

    state = ST_RXWAIT;

    Heltec.begin();

    loramesh.begin();


#if ROUTER
   // xTaskCreatePinnedToCore( RouterTask,"Router_TaskHandle",1024, NULL, 2, NULL, ARDUINO_RUNNING_CORE);
    wait_res = 0;
    send_req = 1;
#else
   //xTaskCreatePinnedToCore( EndDevTask,"EndDev_TaskHandle",1024, NULL, 2, NULL, ARDUINO_RUNNING_CORE);
#endif

   // start();

}

void loop() {
bool ret=0;
int packetSize = 0;
uint8_t address;
uint8_t framesize;
int len = 0;

#if ROUTER
    if (millis() - lastSendTime > INTERVAL){
        lastSendTime = millis();
        wait_res = 0;
        send_req = 1;
    }

    if (wait_res){
        ret = receivePacket();
    }

    if (send_req){
        //Marcamos o tempo que ocorreu o último envio
        wait_res = 1;
        send_req = 0;
        //Envia o pacote para informar ao Slave que queremos receber os dados
        framesize = sendPacketReq(lastSendTime);
        log_i("SendPacket len=%d src=%d dst=%d invid=%d time=%d",framesize,srcaddress,dstaddress,invokeid, lastSendTime);
    }



#else
    
    ret = receivePacket();
    if (ret){
        delay(10);
        sendPacketRes(srcaddress,data1);            
    }    

#if 0
    packetSize = loramesh.parsePacket(0);
    if (packetSize)
    {
        clearBuffer(rxpacket, BUFFER_SIZE);

        while (loramesh.available() && len < BUFFER_SIZE - 1) {
            rxpacket[len++] = (char)loramesh.read(); // Lê o pacote byte a byte
        }

        rxpacket[len] = '\0'; // Termina string

        // Confirmação de recepção
        address = getaddress((uint8_t *)rxpacket);
        ret = checkcrc((uint8_t *)rxpacket,packetSize);

        if ((address == MY_ADDR) && (ret == 1)){
           log_i("Receive OK");
           delay(10);
           sendPacketRes();            
        }
        else
           log_i("Receive Error len[%d] addr=%d crc=%d",packetSize, address,retcrc);

     }

#endif
#endif
    delay(10);

    //if (Rx1Handle != NULL) {
    //   vTaskDelete(Rx1Handle);
   // }

}

