#include "devconfig.h"
#include "loramesh.h"
#include <RadioLib.h>
#include "radio.h"

#if defined ( WIFI_LoRa_32_V3 )
#include <modules/sx126x/sx1262.h>

SX1262 radio = new Module(SS,DIO0,RST_LoRa,BUSY_LoRa);

#endif

#if defined ( WIFI_LoRa_32_V2 )
#include <modules/SX127x/SX1276.h>

SX1276 radio = new Module(SS, DIO0, RST_LoRa, DIO1);
#endif

LoRaClass loramesh;

// flag to indicate that a preamble was detected
volatile bool detectedFlag = false;
// flag to indicate that a preamble was not detected
volatile bool timeoutFlag = false;

volatile bool rxFlag = false;
char buf [10];
char Readback[50];
bool newvalue=0;
int packetSize = 0;
char frame[50];

// save transmission states between loops
int transmissionState = RADIOLIB_ERR_NONE;

// flag to indicate transmission or reception state
bool transmitFlag = false;

// flag to indicate that a packet was sent or received
volatile bool operationDone = false;

LoRaClass::LoRaClass() :
  _spiSettings(8E6, MSBFIRST, SPI_MODE0),
  _ss(LORA_DEFAULT_SS_PIN), _reset(LORA_DEFAULT_RESET_PIN), _dio0(LORA_DEFAULT_DIO0_PIN),
  _frequency(0),
  _packetIndex(0),
  _implicitHeaderMode(0),
  _onReceive(NULL)
{
  // overide Stream timeout value
  setTimeout(0);
}

void setFlag(void) {
  // we sent or received  packet, set the flag
  operationDone = true;
}

#if ENABLE_RX_INTERRUPT
// Can't do Serial or display things here, takes too much time for the interrupt
void rx() {
  rxFlag = true;
}
#endif
void LoRaClass::VextON(void)
{
	pinMode(Vext,OUTPUT);
	digitalWrite(Vext, LOW);
}

void LoRaClass::VextOFF(void) //Vext default OFF
{
	pinMode(Vext,OUTPUT);
	digitalWrite(Vext, HIGH);
}

int16_t LoRaClass::standby(){
  return (radio.standby());
}

void setFlagTimeout(void) {
  // we timed out, set the flag
  timeoutFlag = true;
}

void setFlagDetected(void) {
  // we got a preamble, set the flag
  detectedFlag = true;
}

int LoRaClass::begin()
{
  float freq = LORA_FREQUENCY; 
  float bw = LORA_BW; 
  uint8_t sf = LORA_SF; 
  uint8_t cr = LORA_CR; 
  uint8_t syncWord = RADIOLIB_SX127X_SYNC_WORD; 
  int8_t power = LORA_POWER; 
  uint16_t preambleLength = 8; 
  uint8_t gain = 0;

  #if defined( WIFI_LoRa_32_V3 ) 
      SPI.begin(SCK,MISO,MOSI,SS);
      /*int16_t begin(float freq = 434.0, float bw = 125.0, uint8_t sf = 9, uint8_t cr = 7, 
                      uint8_t syncWord = RADIOLIB_SX126X_SYNC_WORD_PRIVATE, int8_t power = 10, 
                      uint16_t preambleLength = 8, float tcxoVoltage = 1.6, bool useRegulatorLDO = false);
        */ 
      int state = radio.begin(LORA_FREQUENCY_V3,LORA_BW_V3,LORA_SF,LORA_CR,RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 
                                LORA_POWER, 8, 1.6, false);

      if (state == RADIOLIB_ERR_NONE)
        Serial.println("Success!!!!");
      else {
        Serial.print("failed, code =");
        Serial.println(state);
        while(true);
      } 

      radio.setFrequency(frequency);
      //radio.setDataRate(LORA_DATARATE);
      radio.setBandwidth(LORA_BW_V3);
      radio.setSpreadingFactor(LORA_SF);
      radio.setOutputPower(TRANSMIT_POWER);

  #if ENABLE_RX_INTERRUPT
      // Set the callback function for received packets
      radio.setDio1Action(rx);
      radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);      
  #endif

#else //( WIFI_LoRa_32_V2 ) 

  VextON();

  // Initialize the radio
  setPins(SS,RST_LoRa,DIO0);

  int state = radio.begin(freq,bw,sf,cr,syncWord, power, preambleLength, gain);
  if (state == RADIOLIB_ERR_NONE) {
    log_i("Radio begin success!");
    
  } else {
    log_v("failed, code =%d",state);
    while (true) { delay(10); }
  }
  // set the function that will be called
  // when LoRa preamble is not detected within CAD timeout period
  radio.setDio0Action(setFlagTimeout, RISING);

  // set the function that will be called
  // when LoRa preamble is detected
  //radio.setDio1Action(setFlagDetected, RISING);

  #if ROUTER
      // send the first packet on this node
      //sprintf(frame,"%s%d",frame1,count1);
      //count1++;
      //transmissionState = radio.startTransmit(frame);
      //transmitFlag = true;
  #else
      // start listening for LoRa packets on this node
      state = radio.startReceive();
      if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success!"));
      } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true) { delay(10); }
      }
  #endif

#endif
  
  return 1;
}

void LoRaClass::end()
{
  // put in sleep mode
  sleep();
  // stop SPI
  SPI.end();
}

void LoRaClass::onReceive(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

#if 0
    xHigherPriorityTaskWoken = xTaskNotifyFromISR(
        LoraMesher::getInstance().ReceivePacket_TaskHandle,
        0,
        eSetValueWithoutOverwrite,
        &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken == pdTRUE)
        portYIELD_FROM_ISR();
#endif

}

//TODO: Retry start receiving if it fails
void LoRaClass::clearDioActions () {
    radio.clearDio0Action();
    radio.clearDio1Action();
}

void LoRaClass::setDioActionsForReceivePacket() {
    clearDioActions();

  #if defined( WIFI_LoRa_32_V2 ) 
    //radio.setDioActionForReceiving(onReceive);
    //radio.setDio0Action(onReceive, RISING);
  #endif

}

void LoRaClass::restartRadio() {
    radio.reset();
    //initializeLoRa();
    log_e("Restarting radio DONE");
}

int LoRaClass::startReceiving() {
    //setDioActionsForReceivePacket();

    int res = radio.startReceive();
    if (res != 0) {
        log_e("Starting receiving gave error: %d", res);
        restartRadio();
        return startReceiving();
    }
    return res;
}

int LoRaClass::beginPacket(int implicitHeader)
{
  // put in standby mode
  idle();
  if (implicitHeader) {
    implicitHeaderMode();
  } else {
    explicitHeaderMode();
  }
  // reset FIFO address and paload length
  writeRegister(REG_FIFO_ADDR_PTR, 0);
  writeRegister(REG_PAYLOAD_LENGTH, 0);
  
  return 1;
}

int LoRaClass::endPacket(bool async)
{
  // put in TX mode
  writeRegister(REG_OPMODE, MODE_LONG_RANGE_MODE | MODE_TX);
  if (async) {
    // grace time is required for the radio
    delayMicroseconds(150);
  } else {
    // wait for TX done
    //V3 -  
#if defined( WIFI_LoRa_32_V3 )    
    while ((readRegister(REG_IRQ_FLAGS) & RADIOLIB_SX126X_IRQ_TX_DONE) == 0) {
      log_i("write3");
      yield();
    }
    // clear IRQ's
    log_i("write4");
    writeRegister(REG_IRQ_FLAGS, RADIOLIB_SX126X_IRQ_TX_DONE);
#else
    while ((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
      yield();
    }
    // clear IRQ's
    writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
#endif

  }

  return 1;
}

int LoRaClass::parsePacket(int size)
{
  int packetLength = 0;
  int irqFlags = readRegister(REG_IRQ_FLAGS);

  if (size > 0) {
    implicitHeaderMode();
    writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    explicitHeaderMode();
  }

  // clear IRQ's
  writeRegister(REG_IRQ_FLAGS, irqFlags);
//RADIOLIB_SX126X_IRQ_RX_DONE
#if defined(WIFI_LoRa_32_V3)
  if ((irqFlags & RADIOLIB_SX126X_IRQ_RX_DONE) && (irqFlags & RADIOLIB_SX126X_IRQ_CRC_ERR) == 0) {

    // received a packet
    _packetIndex = 0;
    // read packet length
    if (_implicitHeaderMode) {
      packetLength = readRegister(REG_PAYLOAD_LENGTH);
    } else {4
      packetLength = readRegister(REG_RX_NB_BYTES);
    }
    // set FIFO address to current RX address
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));
    // put in standby mode
    idle();
  }
  else if (readRegister(REG_OPMODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
    // not currently in RX mode
    // reset FIFO address
    writeRegister(REG_FIFO_ADDR_PTR, 0);
    // put in single RX mode
    writeRegister(REG_OPMODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
  }
#else

  if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
    // received a packet
    _packetIndex = 0;
    // read packet length
    if (_implicitHeaderMode) {
      packetLength = readRegister(REG_PAYLOAD_LENGTH);
    } else {
      packetLength = readRegister(REG_RX_NB_BYTES);
    }
    //log_i("irqflags1 =%2x packetsize=%d",irqFlags,packetLength);
    // set FIFO address to current RX address
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));
    // put in standby mode
    idle();
  }
  else if (readRegister(REG_OPMODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
    // not currently in RX mode
    // reset FIFO address
    writeRegister(REG_FIFO_ADDR_PTR, 0);
    // put in single RX mode
    writeRegister(REG_OPMODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
  }

#endif

  return packetLength;
}

int LoRaClass::packetRssi()
{
	int8_t snr=0;
    int8_t SnrValue = readRegister( 0x19 );
    int16_t rssi = readRegister(REG_PKT_RSSI_VALUE);

	if( SnrValue & 0x80 ) // The SNR sign bit is 1
	{
		// Invert and divide by 4
		snr = ( ( ~SnrValue + 1 ) & 0xFF ) >> 2;
		snr = -snr;
	}
	else
	{
		// Divide by 4
		snr = ( SnrValue & 0xFF ) >> 2;
	}
    if(snr<0)
    {
    	rssi = rssi - (_frequency < 525E6 ? 164 : 157) + ( rssi >> 4 ) + snr;
    }
    else
    {
    	rssi = rssi - (_frequency < 525E6 ? 164 : 157) + ( rssi >> 4 );
    }

  return ( rssi );
}

float LoRaClass::packetSnr()
{
  return (((int8_t)readRegister(REG_PKT_SNR_VALUE) +2) >> 2);
}

size_t LoRaClass::write(uint8_t byte)
{
  return write(&byte, sizeof(byte));
}

size_t LoRaClass::write(const uint8_t *buffer, size_t size)
{
  int currentLength = readRegister(REG_PAYLOAD_LENGTH);
  // check size
  if ((currentLength + size) > MAX_PKT_LENGTH) {
    size = MAX_PKT_LENGTH - currentLength;
  }
  // write data
  for (size_t i = 0; i < size; i++) {
    writeRegister(REG_FIFO, buffer[i]);
  }
  // update length
  writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);
  return size;
}

int LoRaClass::available()
{
  return (readRegister(REG_RX_NB_BYTES) - _packetIndex);
}

int LoRaClass::read()
{
  if (!available()) {
  	return -1; 
	}
  _packetIndex++;
  return readRegister(REG_FIFO);
}

int LoRaClass::peek()
{
  if (!available()) {
  	return -1; 
	}
  // store current FIFO address
  int currentAddress = readRegister(REG_FIFO_ADDR_PTR);
  // read
  uint8_t b = readRegister(REG_FIFO);
  // restore FIFO address
  writeRegister(REG_FIFO_ADDR_PTR, currentAddress);
  return b;
}

void LoRaClass::flush()
{
}

void LoRaClass::onReceive(void(*callback)(int))
{
  _onReceive = callback;

  if (callback) {
    writeRegister(REG_DIOMAPPING1, 0x00);
    attachInterrupt(digitalPinToInterrupt(_dio0), LoRaClass::onDio0Rise, RISING);
//    attachInterrupt(digitalPinToInterrupt(_dio0), LoRaClass::onDio0Rise, RISING);
  } else {
    detachInterrupt(digitalPinToInterrupt(_dio0));
  }
}

void LoRaClass::receive(int size)
{
  if (size > 0) {
    implicitHeaderMode();
    writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    explicitHeaderMode();
  }

  writeRegister(REG_OPMODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void LoRaClass::idle()
{
  writeRegister(REG_OPMODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void LoRaClass::sleep()
{
  writeRegister(REG_OPMODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void LoRaClass::setTxPower(int8_t power, int8_t outputPin)
{
	  uint8_t paConfig = 0;
	  uint8_t paDac = 0;

	  paConfig = readRegister( REG_PACONFIG );
	  paDac = readRegister( REG_PADAC );

	  paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | outputPin;
	  paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;

	  if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
	  {
	    if( power > 17 )
	    {
	      paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
	    }
	    else
	    {
	      paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
	    }
	    if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
	    {
	      if( power < 5 )
	      {
	        power = 5;
	      }
	      if( power > 20 )
	      {
	        power = 20;
	      }
	      paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
	    }
	    else
	    {
	      if( power < 2 )
	      {
	        power = 2;
	      }
	      if( power > 17 )
	      {
	        power = 17;
	      }
	      paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
	    }
	  }
	  else
	  {
	    if( power < -1 )
	    {
	      power = -1;
	    }
	    if( power > 14 )
	    {
	      power = 14;
	    }

	    paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
	  }
	  writeRegister( REG_PACONFIG, paConfig );
	  writeRegister( REG_PADAC, paDac );
}

void LoRaClass::setTxPowerMax(int level)
{
	if (level < 5)		{
		level = 5;
	}
	else if(level > 20)	{
		level = 20;
	}
	writeRegister(REG_OCP,0x3f);
	writeRegister(REG_PADAC,0x87);//Open PA_BOOST
	writeRegister(REG_PACONFIG, RF_PACONFIG_PASELECT_PABOOST | (level - 5));
}

void LoRaClass::setFrequency(long frequency)
{
  _frequency = frequency;

  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
  writeRegister(REG_FRFMSB, (uint8_t)(frf >> 16));
  writeRegister(REG_FRFMID, (uint8_t)(frf >> 8));
  writeRegister(REG_FRFLSB, (uint8_t)(frf >> 0));
}

void LoRaClass::setSpreadingFactor(int sf)
{
  if (sf < 6) {
  	sf = 6; 
	}
  else if (sf > 12) {
  	sf = 12; 
  	}
  if (sf == 6) {
    writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
    writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
  } else {
    writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
    writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
  }

  writeRegister(REG_MODEM_CONFIG_2, (readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));

}

void LoRaClass::setSignalBandwidth(long sbw)
{
  int bw;

  if (sbw <= 7.8E3) { bw = 0; }
  else if (sbw <= 10.4E3) { bw = 1; }
  else if (sbw <= 15.6E3) { bw = 2; }
  else if (sbw <= 20.8E3) { bw = 3; }
  else if (sbw <= 31.25E3) { bw = 4; }
  else if (sbw <= 41.7E3) { bw = 5; }
  else if (sbw <= 62.5E3) { bw = 6; }
  else if (sbw <= 125E3) { bw = 7; }
  else if (sbw <= 250E3) { bw = 8; }
  else /*if (sbw <= 250E3)*/ { bw = 9; }

  writeRegister(REG_MODEM_CONFIG_1,(readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}

void LoRaClass::setCodingRate4(int denominator)
{
  if (denominator < 5) {
    denominator = 5;
  } else if (denominator > 8) {
    denominator = 8;
  }
  int cr = denominator - 4;

  writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void LoRaClass::setPreambleLength(long length)
{
  writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void LoRaClass::setSyncWord(int sw)
{
  writeRegister(REG_SYNC_WORD, sw);
}

void LoRaClass::enableCrc()
{
  writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) | 0x04);
}

void LoRaClass::disableCrc()
{
  writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

void LoRaClass::enableTxInvertIQ()
{
  writeRegister( REG_LR_INVERTIQ, ( ( readRegister( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON ) );
  writeRegister( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
}

void LoRaClass::enableRxInvertIQ()
{
  writeRegister( REG_LR_INVERTIQ, ( ( readRegister( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF ) );
  writeRegister( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
}

void LoRaClass::disableInvertIQ()
{
  writeRegister( REG_LR_INVERTIQ, ( ( readRegister( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
  writeRegister( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
}

void LoRaClass::enableInvertIQ()
{
  writeRegister( REG_LR_INVERTIQ, ( ( readRegister( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_ON ) );
  writeRegister( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
}

byte LoRaClass::random()
{
  return readRegister(REG_RSSI_WIDEBAND);
}

void LoRaClass::setPins(int ss, int reset, int dio0)
{
  _ss = ss;
  _reset = reset;
  _dio0 = dio0;
}

void LoRaClass::setSPIFrequency(uint32_t frequency)
{
  _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
}

void LoRaClass::dumpRegisters(Stream& out)
{
  for (int i = 0; i < 128; i++) {
    out.print("0x");
    out.print(i, HEX);
    out.print(": 0x");
    out.println(readRegister(i), HEX);
  }
}

void LoRaClass::explicitHeaderMode()
{
  _implicitHeaderMode = 0;
  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

void LoRaClass::implicitHeaderMode()
{
  _implicitHeaderMode = 1;
  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

void LoRaClass::handleDio0Rise()
{
  int irqFlags = readRegister(REG_IRQ_FLAGS);
  // clear IRQ's
  writeRegister(REG_IRQ_FLAGS, irqFlags);
#if defined(WIFI_LoRa_32_V3) 
  if ((irqFlags & RADIOLIB_SX126X_IRQ_CRC_ERR) == 0) {
#else
  if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
#endif    
    // received a packet
    _packetIndex = 0;
    // read packet length
    int packetLength = _implicitHeaderMode ? readRegister(REG_PAYLOAD_LENGTH) : readRegister(REG_RX_NB_BYTES);
    // set FIFO address to current RX address
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));
    if (_onReceive) { _onReceive(packetLength); }
    // reset FIFO address
    writeRegister(REG_FIFO_ADDR_PTR, 0);
  }
}

uint8_t LoRaClass::readRegister(uint8_t address)
{
  return singleTransfer(address & 0x7f, 0x00);
}

void LoRaClass::writeRegister(uint8_t address, uint8_t value)
{
  singleTransfer(address | 0x80, value);
}

uint8_t LoRaClass::singleTransfer(uint8_t address, uint8_t value)
{
  uint8_t response;
  digitalWrite(_ss, LOW);
  SPI.beginTransaction(_spiSettings);
  SPI.transfer(address);
  response = SPI.transfer(value);
  SPI.endTransaction();
  digitalWrite(_ss, HIGH);
  return response;
}

void LoRaClass::onDio0Rise()
{
  loramesh.handleDio0Rise();
}


bool LoRaClass::sendPacket(uint8_t* p,uint8_t len) {
    
    //waitBeforeSend(1);

#if  defined ( WIFI_LoRa_32_V3 )
  //int16_t SX126x::transmit(uint8_t* data, size_t len, uint8_t addr) {
  int16_t ret = radio.transmit(data);
  Serial.print("transmited ");
  Serial.print(var1);
  Serial.print(" ret=");
  Serial.println(ret);

#else  //WIFI_LoRa_32_V2

    //clearDioActions();

    //Blocking transmit, it is necessary due to deleting the packet after sending it. 
    int transmissionState = radio.transmit(p, len,1);

    //Start receiving again after sending a packet
    startReceiving();

   if (transmissionState == RADIOLIB_ERR_NONE) {
    return true;
  } else {
    log_e("transmission failed, code=%d ",transmissionState);
    return false;
  }   

#endif

    return true;
}

#if 0
void LoRaClass::sendPackets() {

  int sendCounter = 0;
  uint8_t sendId = 0;
  uint8_t resendMessage = 0;
  uint16_t nextHop=0;

  while (ToSendPackets->getLength() > 0) {

    log_e("sendPackets");
    ToSendPackets->setInUse();

    QueuePacket<Packet<uint8_t>>* tx = ToSendPackets->Pop();

    ToSendPackets->releaseInUse();

    if (tx) {
        if (tx->packet->src == SOURCE_ADDRESS)
            tx->packet->id = sendId++;

        //If the packet has a data packet and its destination is not broadcast add the via to the packet and forward the packet
        if (PacketService::isDataPacket(tx->packet->type) && tx->packet->dst != BROADCAST_ADDR) {
            //uint16_t nextHop = RoutingTableService::getNextHop(tx->packet->dst);
            nextHop = 1;
            //Next hop not found
            if (nextHop == 0) {
                PacketQueueService::deleteQueuePacketAndPacket(tx);
                //incDestinyUnreachable();
                continue;
            }

            //(reinterpret_cast<DataPacket*>(tx->packet))->via = nextHop;
        }

        //recordState(LM_StateType::STATE_TYPE_SENT, tx->packet);

        //Send packet
        bool hasSend = sendPacket(tx->packet);

        sendCounter++;
    } 
  }
}
#endif

uint32_t getRssi(void) {
  uint32_t retRssi=0;

 #if  defined ( WIFI_LoRa_32_V3 ) 
     retRssi = (uint32_t) radio.getRSSI(1);
 #else
     retRssi = (uint32_t) loramesh.packetRssi();
 #endif
 
 return retRssi;
}

uint8_t LoRaClass::ReceiveFrame(char *pframe) {

 #if  defined ( WIFI_LoRa_32_V3 ) 
  String str;

#if ENABLE_RX_INTERRUPT
  //radio.clearDio1Action();
  packetSize = 0;

  if (rxFlag) {
    rxFlag = false;
  int state = radio.receive(str);

  if (state == RADIOLIB_ERR_NONE) {
    // Packet received successfully
    packetSize = str.length();
    strcpy(pframe,str.c_str());

    log_i("Received packet [%d] rssi=%d",packetSize,getRssi());
  }
  //else {
      // Some other error occurred (excluding timeout and CRC mismatch)
  //  log_e("Failed to receive packet!!!! code=%d",state);
  //}

    //radio.setDio1Action(rx);
    radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);    
  }
#else
  int state = radio.receive(str);

  if (state == RADIOLIB_ERR_NONE) {
    // Packet received successfully
    packetSize = str.length();
    strcpy(pframe,str.c_str());

    log_i("Received packet [%d] rssi=%d",packetSize,getRssi());
  }
#endif


#else // WIFI_LoRa_V2

#if 0
  packetSize = loramesh.parsePacket();
   Serial.print("Rx Packetsize=");
    Serial.println(packetSize);
  if (packetSize) {
    // read packet
    while (loramesh.available()) {
	   sprintf(Readback+strlen(Readback),"%c",(char)loramesh.read());
    }
    memcpy(pframe,Readback,sizeof(packetSize));
   
#if DISPLAY_ENABLE 
    //Heltec.DisplayShow(packetSize,Readback);
    //DisplayShow1(packetSize);
 
#endif	
    memset(Readback,0,50);
    
    // print RSSI of packet
    // Serial.print(" with RSSI ");
    // Serial.println(loramesh.packetRssi());
  }
#else
  String str;

  //disableCrc();
  enableCrc();
  int state = radio.readData(str);
  if (state == RADIOLIB_ERR_NONE) {
    // packet was successfully received
    log_i("Received packet len=%d",str.length());
    Serial.println(str);
  }
#endif
#endif

  return packetSize;
}

