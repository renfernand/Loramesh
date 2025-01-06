#ifndef LORA_H
#define LORA_H

#include <Arduino.h>
#include <SPI.h>
#include "LinkedQueue.hpp"
#include "AppPacket.h"
#include "ControlPacket.h"
#include "DataPacket.h"
#include "QueuePacket.h"
#include "PacketQueueService.h"
#include "Packet.h"

#if defined( WIFI_LoRa_32_V3 )
#define LORA_DEFAULT_SS_PIN     8
#define LORA_DEFAULT_RESET_PIN  12
#define LORA_DEFAULT_DIO0_PIN   14
#endif

#if defined( WIFI_LoRa_32_V2)
#define LORA_DEFAULT_SS_PIN     18
#define LORA_DEFAULT_RESET_PIN  14
#define LORA_DEFAULT_DIO0_PIN   26
#endif

#define PA_OUTPUT_PA_BOOST_PIN  1
#define PA_OUTPUT_RFO_PIN       0

#define LOCAL_ADDRESS 1
/*!
 * RegPaConfig
 */
#define RF_PACONFIG_PASELECT_MASK                   0x7F
#define RF_PACONFIG_PASELECT_PABOOST                0x80
#define RF_PACONFIG_PASELECT_RFO                    0x00 // Default

#define RF_PACONFIG_MAX_POWER_MASK                  0x8F

#define RF_PACONFIG_OUTPUTPOWER_MASK                0xF0

/*!
 * RegPaDac
 */
#define RF_PADAC_20DBM_MASK                         0xF8
#define RF_PADAC_20DBM_ON                           0x07
#define RF_PADAC_20DBM_OFF                          0x04  // Default


void LoraSendFrame(String data,size_t len);
uint8_t LoraReceiveFrame(char *pframe);
uint32_t getRssi(void);

#if defined (__STM32F1__)
inline unsigned char  digitalPinToInterrupt(unsigned char Interrupt_pin) { return Interrupt_pin; } //This isn't included in the stm32duino libs (yet)
#define portOutputRegister(port) (volatile byte *)( &(port->regs->ODR) ) //These are defined in STM32F1/variants/generic_stm32f103c/variant.h but return a non byte* value
#define portInputRegister(port) (volatile byte *)( &(port->regs->IDR) ) //These are defined in STM32F1/variants/generic_stm32f103c/variant.h but return a non byte* value
#endif

class LoRaClass : public Stream {
public:
  LoRaClass();

    enum LoraModules {
        SX1276_MOD,
        SX1262_MOD,
        SX1278_MOD,
        SX1268_MOD,
        SX1280_MOD,
    };


  int16_t standby();

  int begin();
  void end();
  void initializeLoRa();

  bool sendPacket(uint8_t *data, uint8_t len);
  
  void VextON(void);
  void VextOFF(void);
    
  void restartRadio(void);
  int startReceiving(void);
  void setDioActionsForReceivePacket(void);
  void clearDioActions(void);
  void onReceive(void);

  uint8_t ReceiveFrame(char *pframe); 

  int beginPacket(int implicitHeader = false);
  int endPacket(bool async = false);

  int parsePacket(int size = 0);
  int packetRssi();
  float packetSnr();

  // from Print
  virtual size_t write(uint8_t byte);
  virtual size_t write(const uint8_t *buffer, size_t size);

  // from Stream
  virtual int available();
  virtual int read();
  virtual int peek();
  virtual void flush();

  void onReceive(void(*callback)(int));

  void receive(int size = 0);
  void idle();
  void sleep();

  void setTxPower(int8_t power, int8_t outputPin);
  void setTxPowerMax(int level);
  void setFrequency(long frequency);
  void setSpreadingFactor(int sf);
  void setSignalBandwidth(long sbw);
  void setCodingRate4(int denominator);
  void setPreambleLength(long length);
  void setSyncWord(int sw);
  void enableCrc();
  void disableCrc();
  void enableTxInvertIQ();
  void enableRxInvertIQ();
  void enableInvertIQ();
  void disableInvertIQ();

  // deprecated
  void crc() { enableCrc(); }
  void noCrc() { disableCrc(); }

  byte random();

  void setPins(int ss = LORA_DEFAULT_SS_PIN, int reset = LORA_DEFAULT_RESET_PIN, int dio0 = LORA_DEFAULT_DIO0_PIN);
  void setSPIFrequency(uint32_t frequency);
  void sendPackets();
  
  void dumpRegisters(Stream& out);
  void createPacketAndSend(uint16_t dst, uint8_t* payload, uint8_t payloadSize);
  DataPacket* createDataPacket(uint16_t dst, uint16_t src, uint8_t type, uint8_t* payload, uint8_t payloadSize);
  //void setPackedForSend(Packet<uint8_t>* p, uint8_t priority); 

    /**
     * @brief Get the Instance of the LoRaMesher
     *
     * @return LoraMesher&
     */
    static LoRaClass& getInstance() {
        static LoRaClass instance;
        return instance;
    };  


   /**
     * @brief Create a Packet And Send it
     *
     * @tparam T
     * @param dst Destination
     * @param payload Payload of type T
     * @param payloadSize Length of the payload in T
     */
    template <typename T>
    void createPacketAndSend(uint16_t dst, T* payload, uint8_t payloadSize) {
        //Cannot send an empty packet
        log_i("createPacketAndSend");  
        if (payloadSize == 0)
            return;

        //Get the size of the payload in bytes
        size_t payloadSizeInBytes = payloadSize * sizeof(T);
        //Create a data packet with the payload
        DataPacket* dPacket = PacketService::createDataPacket(dst, LOCAL_ADDRESS, DATA_P, reinterpret_cast<uint8_t*>(payload), payloadSizeInBytes);

        //Create the packet and set it to the send queue
        setPackedForSend(reinterpret_cast<Packet<uint8_t>*>(dPacket), DEFAULT_PRIORITY);
    }



private:
  void explicitHeaderMode();
  void implicitHeaderMode();

  void handleDio0Rise();

  uint8_t readRegister(uint8_t address);
  void writeRegister(uint8_t address, uint8_t value);
  uint8_t singleTransfer(uint8_t address, uint8_t value);
  static void onDio0Rise();

  LM_LinkedList<AppPacket<uint8_t>>* ReceivedAppPackets = new LM_LinkedList<AppPacket<uint8_t>>();

  LM_LinkedList<QueuePacket<Packet<uint8_t>>>* ReceivedPackets = new LM_LinkedList<QueuePacket<Packet<uint8_t>>>();

  LM_LinkedList<QueuePacket<Packet<uint8_t>>>* ToSendPackets = new LM_LinkedList<QueuePacket<Packet<uint8_t>>>();

  void addToSendOrderedAndNotify(QueuePacket<Packet<uint8_t>>* qp);
    /**
     * @brief Sets the packet in a Fifo with priority and will send the packet when needed.
     *
     * @param p packet<uint8_t>*
     * @param priority Priority set DEFAULT_PRIORITY by default. 0 most priority
     */
    void setPackedForSend(Packet<uint8_t>* p, uint8_t priority) {
        //ESP_LOGI(LM_TAG, "Adding packet to Q_SP");
        QueuePacket<Packet<uint8_t>>* send = PacketQueueService::createQueuePacket(p, priority);
        //ESP_LOGI(LM_TAG, "Created packet to Q_SP");
        addToSendOrderedAndNotify(send);
        log_i ("setPackedForSend");
        //TODO: Using vTaskDelay to kill the packet inside LoraMesher
    }


  SPISettings _spiSettings;
  int _ss;
  int _reset;
  int _dio0;
  int _frequency;
  int _packetIndex;
  int _implicitHeaderMode;
  void (*_onReceive)(int);
};

//extern LoRaClass loramesh;

#endif
