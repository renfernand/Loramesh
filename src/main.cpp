#include "devconfig.h"
#include "heltec.h"
#include "Lora\loramesh.h"
#include <Wire.h>
#include <RadioLib.h>
#include "esp_log.h"

#if DISPLAY_ENABLE
#include "OLED\SSD1306.h"
#endif
//#define ARDUINO_RUNNING_CORE 1

TaskHandle_t App_TaskHandle = nullptr;
TaskHandle_t EndDev_TaskHandle = nullptr;
statemac nextstate;
uint16_t idx_response=0;

extern LoRaClass loramesh;

uint8_t actualslot=0;
long lastSendTime = 0;
uint8_t send_pct = 0;
extern volatile bool messageReceived;



void AppTask(void * parameter) {

    bool ret=0;
    uint8_t framesize=0;

    for(;;){

        switch (nextstate) {
            case ST_TXBEACON:
              if (loramesh.mydd.devtype == DEV_TYPE_ROUTER) {
                send_pct = 1;
                nextstate = ST_RXWAIT; 
              }
            case ST_RXWAIT:
              if (loramesh.receivePacket()){
                if (loramesh.mydd.devtype == DEV_TYPE_ROUTER){
                    //o router nao faz nada com o dado
                    //somente calcula indices de desempenho
                    //log_i("Pacote eh para mim e eu sou Router!!!!");

                    //proximo estado eh enviar o beacon novamente
                    nextstate = ST_STANDBY;
                }
                else{
                    //o end device deve tratar o pacote e retransmitir se for beacon
                    //sincronizar o relogio atual
                    //log_i("RX PCT SEND RESP seqnum = %d",lastpkt.seqnum );
                    //send_pct = 1;
                    nextstate = ST_STANDBY; 
                }
              }
              break;
            case ST_TXDATA: 
               if ((loramesh.mydd.devtype == DEV_TYPE_ENDDEV) && (actualslot == DATA_SLOT)){
                   //aqui eh minha janela de dados...devo enviar o pacote de dados...
                    //log_i("Aqui eh hora de enviar os dados!!!!");
                    nextstate = ST_RXWAIT; 
               }    
            default:
              break;
        } 

        //controle de slots
        if ((millis() - lastSendTime) > SLOT_INTERVAL){
            lastSendTime = millis();
            actualslot++;

            if (actualslot > MAX_SLOTS){
               actualslot=0;
               nextstate = ST_TXBEACON;
            }
            //log_i("ActualSlot=%d",actualslot);
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    } 
}


//calcula a quantidade de polls e erros
void setindpolls(){
    uint16_t lastseqnum = loramesh.getLastSeqNum();
    if (lastseqnum == loramesh.lastpkt.seqnum){
        idx_response++;
    }
    log_i("Polls=%d Resp=%d",lastseqnum, idx_response);

}


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
    vTaskSuspend(App_TaskHandle);
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
    vTaskResume(App_TaskHandle);
    //vTaskResume(EndDev_TaskHandle);

    // Start Receiving
    loramesh.startReceiving();

    // Set previous priority
    vTaskPrioritySet(NULL, prevPriority);
}


void setup()
{
    uint8_t ret=0;

    Serial.begin(9600);

    Heltec.begin();

    loramesh.begin();

   // xTaskCreatePinnedToCore( AppTask,"App_TaskHandle",1024, NULL, 2, NULL, ARDUINO_RUNNING_CORE);
#if 0
    xTaskCreatePinnedToCore( EndDevTask,"EndDev_TaskHandle",1024, NULL, 2, NULL, ARDUINO_RUNNING_CORE);
#endif
   // start();

    //device function could be ROUTER (=1) or END DEVICE = 2
    if (loramesh.mydd.devtype == DEV_TYPE_ROUTER){
        actualslot = 0;
        nextstate = ST_TXBEACON;
    }
    else{
        loramesh.startReceiving();
        nextstate = ST_RXWAIT;
    }


  //print mydd
#if DISPLAY_ENABLE  
    {
        char buf[20];

        Heltec.DisplayClear();

        sprintf(buf,"SN=%x ",loramesh.mydd.devserialnumber);
        Heltec.DisplayShow1(buf);
        
        if (loramesh.mydd.devtype == DEV_TYPE_ROUTER)
            sprintf(buf,"DevType = RT ");
        else
            sprintf(buf,"DevType = ED ");
        Heltec.DisplayShow2(buf);

        sprintf(buf,"Addr=%x ", loramesh.mydd.devaddr);
        Heltec.DisplayShow3(buf);
    }
#endif

}

void loop() {

    uint8_t framesize;
    uint8_t rtaddr=0;
    int len = 0;
    bool ret=0;

    switch (nextstate) {
        case ST_TXBEACON:
            if (loramesh.mydd.devtype == DEV_TYPE_ROUTER) {
            send_pct = 1;
            nextstate = ST_RXWAIT; 
            }
            break;
        case ST_RXWAIT:
          if (messageReceived) {
                messageReceived = false;
              
                ret = loramesh.receivePacket();
                if (ret) {
                    if (loramesh.mydd.devtype == DEV_TYPE_ROUTER){
                        //o router nao faz nada com o dado
                        //somente calcula indices de desempenho
                        setindpolls();

                        //proximo estado eh enviar o beacon novamente
                        nextstate = ST_STANDBY;
                    }
                    else {
                        //o end device deve tratar o pacote e retransmitir se for beacon
                        //sincronizar o relogio atual
                        log_i("RX PCT RECEIVED. SeqNum = %d",loramesh.lastpkt.seqnum );
                        send_pct = 1;
                        nextstate = ST_RXWAIT; 
                    }
                }
            }
            break;
        case ST_TXDATA: 
            if ((loramesh.mydd.devtype == DEV_TYPE_ENDDEV) && (actualslot == DATA_SLOT)){
                //aqui eh minha janela de dados...devo enviar o pacote de dados...
                //log_i("Aqui eh hora de enviar os dados!!!!");
                nextstate = ST_RXWAIT; 
            }    
        default:
            break;
    } 

    //controle de slots
    if ((millis() - lastSendTime) > SLOT_INTERVAL){
        lastSendTime = millis();
        actualslot++;

        if (actualslot >= MAX_SLOTS){
            actualslot=0;
            if (loramesh.mydd.devtype == DEV_TYPE_ROUTER) 
                nextstate = ST_TXBEACON;
            else
                nextstate = ST_RXWAIT;
        }
        //log_i("ActualSlot=%d",actualslot);
    }

    if (send_pct){
        //Marcamos o tempo que ocorreu o último envio
        send_pct = 0;
        //Envia o pacote para informar ao Slave que queremos receber os dados
        if (loramesh.mydd.devtype == DEV_TYPE_ROUTER){
            framesize = loramesh.sendPacketReq(lastSendTime);
        }
        else{
            rtaddr = loramesh.getrouteaddr();
            framesize = loramesh.sendPacketRes(rtaddr,lastSendTime);
        }
    }

    delay(10);

}

